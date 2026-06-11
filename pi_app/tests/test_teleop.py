"""Fail-safe phone-teleop tests (TRANCHE 1).

All tests use a mock motor sink and an injected clock — no hardware, no network,
no WebSocket. They exercise the server-side enforcement directly via
``TeleopSession.tick`` (the watchdog body) exactly as the production watchdog
thread calls it.
"""

import pytest

from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.io.bt_proto import floats_to_bytes
from pi_app.web.teleop import TeleopSession

NEUTRAL = (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)


class MockMotor:
    """Records every (left, right) byte pair the session emits."""

    def __init__(self):
        self.calls = []

    def __call__(self, left, right):
        self.calls.append((left, right))

    @property
    def last(self):
        return self.calls[-1] if self.calls else None


def make_session(*, require_rc_arm=False, rc_state=None, **kw):
    motor = MockMotor()
    provider = (lambda: rc_state["v"]) if rc_state is not None else None
    s = TeleopSession(
        command_sink=motor,
        rc_state_provider=provider,
        require_rc_arm=require_rc_arm,
        **kw,
    )
    return s, motor


def arm(s, now=0.0):
    ok, reason = s.arm(hold_ms=600, now=now)
    assert ok, reason


# --------------------------------------------------------------------------
# A. Deadman
# --------------------------------------------------------------------------

def test_deadman_trips_to_neutral_and_disarm_past_250ms():
    s, motor = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    assert s.tick(now=0.0) == "driving"
    assert motor.last != NEUTRAL  # actually driving
    # Heartbeats stop. Next tick once we're past the 250 ms window.
    assert s.tick(now=0.251) == "deadman_trip"
    assert s.armed is False
    assert s.tripped_reason == "deadman"
    assert motor.last == NEUTRAL


def test_deadman_no_trip_with_continuous_heartbeats():
    s, motor = make_session()
    arm(s, now=0.0)
    t = 0.0
    for _ in range(20):
        s.drive(left_f=0.5, right_f=0.5, seq=int(t * 1000) + 1, now=t)
        assert s.tick(now=t) == "driving"
        t += 0.1  # 10 Hz heartbeats — always inside the 250 ms window
    assert s.armed is True
    assert motor.last != NEUTRAL


def test_deadman_boundary_249_no_trip_251_trip():
    s, _ = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    assert s.tick(now=0.249) == "driving"   # 249 ms gap: no trip
    assert s.armed is True
    assert s.tick(now=0.251) == "deadman_trip"  # 251 ms gap: trip
    assert s.armed is False


def test_pure_heartbeat_keeps_session_alive():
    """A bare ``hb`` (no drive) must refresh the deadman too."""
    s, _ = make_session()
    arm(s, now=0.0)
    s.drive(left_f=0.4, right_f=0.4, seq=1, now=0.0)
    s.tick(now=0.0)
    s.heartbeat(now=0.2)
    assert s.tick(now=0.3) == "driving"   # 0.3-0.2 = 100 ms since last hb
    assert s.armed is True


# --------------------------------------------------------------------------
# B. Reconnect after a trip does not re-arm
# --------------------------------------------------------------------------

def test_reconnect_heartbeats_do_not_rearm_after_trip():
    s, motor = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    s.tick(now=0.3)  # deadman trip
    assert s.armed is False
    # "Reconnect": heartbeats and drives resume, but no explicit arm.
    s.heartbeat(now=0.4)
    assert s.drive(left_f=1.0, right_f=1.0, seq=2, now=0.4) == "ignored_disarmed"
    assert s.tick(now=0.4) == "idle"
    assert s.armed is False
    # Explicit re-arm is what brings it back.
    arm(s, now=0.5)
    s.drive(left_f=1.0, right_f=1.0, seq=3, now=0.5)
    assert s.tick(now=0.5) == "driving"


def test_arm_requires_press_and_hold():
    s, _ = make_session()
    ok, reason = s.arm(hold_ms=100, now=0.0)  # below 500 ms floor
    assert not ok and reason == "hold_too_short"


# --------------------------------------------------------------------------
# C. E-stop latches
# --------------------------------------------------------------------------

def test_estop_latches_and_ignores_fresh_drive():
    s, motor = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    s.estop()
    assert s.estop_latched is True
    assert s.armed is False
    assert motor.last == NEUTRAL
    # Fresh heartbeats + drives must be ignored while latched.
    s.heartbeat(now=0.05)
    assert s.drive(left_f=1.0, right_f=1.0, seq=2, now=0.05) == "ignored_estop"
    assert s.tick(now=0.05) == "estop"
    # Re-arm refused while latched.
    ok, reason = s.arm(hold_ms=600, now=0.05)
    assert not ok and reason == "estop_latched"


def test_estop_dismiss_then_rearm_restores():
    s, _ = make_session()
    s.estop()
    s.clear_estop()
    assert s.estop_latched is False
    arm(s, now=1.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=1.0)
    assert s.tick(now=1.0) == "driving"


def test_clear_estop_does_not_rearm_on_its_own():
    s, _ = make_session()
    arm(s, now=0.0)
    s.estop()
    s.clear_estop()
    assert s.armed is False  # must press-and-hold ARM again


# --------------------------------------------------------------------------
# D. Speed cap
# --------------------------------------------------------------------------

def test_speed_cap_scales_output_and_default_is_slow():
    s, motor = make_session()
    assert s.speed_level == "slow"
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    assert motor.last == floats_to_bytes(0.3, 0.3)  # slow = 0.3


def test_speed_cap_change_applies_immediately():
    s, motor = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    assert motor.last == floats_to_bytes(0.3, 0.3)
    s.set_speed("fast")
    assert s.tick(now=0.01) == "driving"
    assert motor.last == floats_to_bytes(1.0, 1.0)  # fast = 1.0, next tick
    s.set_speed("normal")
    s.tick(now=0.02)
    assert motor.last == floats_to_bytes(0.6, 0.6)


def test_unknown_speed_level_rejected():
    s, _ = make_session()
    assert s.set_speed("ludicrous") is False
    assert s.speed_level == "slow"


# --------------------------------------------------------------------------
# E. Stale / out-of-order command guard
# --------------------------------------------------------------------------

def test_out_of_order_sequence_dropped():
    s, _ = make_session()
    arm(s, now=0.0)
    assert s.drive(left_f=1.0, right_f=0.0, seq=5, now=0.0) == "accepted"
    assert s.drive(left_f=-1.0, right_f=0.0, seq=3, now=0.0) == "dropped_out_of_order"
    assert s.drive(left_f=-1.0, right_f=0.0, seq=5, now=0.0) == "dropped_out_of_order"
    assert s.drive(left_f=0.2, right_f=0.0, seq=6, now=0.0) == "accepted"


def test_stale_timestamp_dropped():
    s, _ = make_session(stale_command_s=0.30)
    arm(s, now=0.0)
    # client timestamps in ms.
    assert s.drive(left_f=1.0, right_f=1.0, seq=1, client_ts=1000.0, now=0.0) == "accepted"
    # 400 ms older than the newest seen -> stale (window is 300 ms).
    assert s.drive(left_f=1.0, right_f=1.0, seq=2, client_ts=600.0, now=0.0) == "dropped_stale"
    # within window -> accepted.
    assert s.drive(left_f=1.0, right_f=1.0, seq=3, client_ts=900.0, now=0.0) == "accepted"


# --------------------------------------------------------------------------
# F. RC authority overrides phone state at all times
# --------------------------------------------------------------------------

def test_rc_disarm_forces_session_disarm_despite_fresh_heartbeat():
    rc = {"v": (True, False)}  # rc_armed, rc_estop
    s, motor = make_session(rc_state=rc)
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    assert s.tick(now=0.0) == "driving"
    # RC operator disarms (ch3). Heartbeat is perfectly fresh.
    rc["v"] = (False, False)
    s.heartbeat(now=0.01)
    assert s.tick(now=0.01) == "rc_override"
    assert s.armed is False
    assert motor.last == NEUTRAL


def test_rc_estop_forces_session_disarm_despite_fresh_heartbeat():
    rc = {"v": (True, False)}
    s, motor = make_session(rc_state=rc)
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    rc["v"] = (True, True)  # ch5 e-stop latched on the RC side
    s.heartbeat(now=0.01)
    assert s.tick(now=0.01) == "rc_override"
    assert s.armed is False
    assert motor.last == NEUTRAL


def test_rc_not_armed_blocks_arming_when_required():
    rc = {"v": (False, False)}
    s, _ = make_session(require_rc_arm=True, rc_state=rc)
    ok, reason = s.arm(hold_ms=600, now=0.0)
    assert not ok and reason == "rc_not_armed"
    # Explicit "RC in my hand" confirmation lets the phone arm the session
    # (the downstream controller RC gate still applies on the robot).
    ok, reason = s.arm(hold_ms=600, rc_in_hand=True, now=0.0)
    assert ok, reason


def test_notify_rc_state_forces_disarm():
    s, motor = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)
    s.notify_rc_state(rc_armed=False, rc_estop=False)
    assert s.armed is False
    assert motor.last == NEUTRAL


# --------------------------------------------------------------------------
# Idle teleop must not suppress RC by stamping the override file every tick
# --------------------------------------------------------------------------

def test_idle_session_does_not_write_after_final_neutral():
    s, motor = make_session()
    # Never armed: ticks must not write anything (would clobber RC sticks).
    for t in (0.0, 0.02, 0.04):
        assert s.tick(now=t) == "idle"
    assert motor.calls == []


def test_driving_then_stop_emits_single_neutral_then_quiet():
    s, motor = make_session()
    arm(s, now=0.0)
    s.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    s.tick(now=0.0)             # driving -> writes
    n_before = len(motor.calls)
    s.tick(now=0.3)             # deadman trip -> one neutral
    assert motor.last == NEUTRAL
    n_after_trip = len(motor.calls)
    assert n_after_trip == n_before + 1
    # Subsequent idle ticks stay quiet (file allowed to go stale -> RC regains).
    s.tick(now=0.32)
    s.tick(now=0.34)
    assert len(motor.calls) == n_after_trip
