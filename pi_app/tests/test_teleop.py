"""Fail-safe phone-teleop tests (TRANCHE 1).

All tests use a mock motor sink and an injected clock — no hardware, no network,
no WebSocket. They exercise the server-side enforcement directly via
``TeleopSession.tick`` (the watchdog body) exactly as the production watchdog
thread calls it.
"""

import time

import pytest

from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.io.bt_proto import floats_to_bytes
from pi_app.web.teleop import (
    TeleopSession,
    make_recorder_rc_state_provider,
    resolve_teleop_token,
)

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


# ==========================================================================
# DEFECT-5 — new coverage for the hardening defects.
# ==========================================================================

class FakeRecorder:
    """Duck-typed stand-in for OakRecorder.get_latest_telemetry()."""

    def __init__(self, telem=None):
        self._t = telem

    def get_latest_telemetry(self):
        return self._t


class FakeTelem:
    """Minimal RecordingTelemetry duck — only the fields the provider reads."""

    def __init__(self, *, is_armed=True, emergency_active=False, ts_mono=0.0):
        self.is_armed = is_armed
        self.emergency_active = emergency_active
        self.ts_mono = ts_mono


# -- (1) ch5 e-stop path end-to-end via the REAL provider -------------------

def test_ch5_estop_via_real_provider_forces_disarm_on_next_tick():
    """emergency_active=True from telemetry must flow through the real
    make_recorder_rc_state_provider and force a session disarm/estop handling on
    the next watchdog tick (the bug: provider read a non-existent field)."""
    clock = {"t": 100.0}
    rec = FakeRecorder(FakeTelem(is_armed=True, emergency_active=False, ts_mono=100.0))
    provider = make_recorder_rc_state_provider(rec, now_fn=lambda: clock["t"])

    motor = MockMotor()
    s = TeleopSession(command_sink=motor, rc_state_provider=provider,
                      require_rc_arm=False, now_fn=lambda: clock["t"])
    ok, reason = s.arm(hold_ms=600)
    assert ok, reason
    s.drive(left_f=1.0, right_f=1.0, seq=1)
    assert s.tick() == "driving"

    # RC ch5 fires: emergency_active becomes True (fresh telemetry).
    rec._t = FakeTelem(is_armed=True, emergency_active=True, ts_mono=100.0)
    assert s.tick() == "rc_override"
    assert s.armed is False
    assert motor.last == NEUTRAL


# -- (2) provider staleness bound -------------------------------------------

def test_provider_staleness_forces_disarm():
    """A frozen telemetry object older than 1 s makes the provider report
    rc_armed=False, which force-disarms the session on the next tick."""
    clock = {"t": 100.0}
    # ts_mono matches the clock initially -> age 0 -> fresh.
    rec = FakeRecorder(FakeTelem(is_armed=True, emergency_active=False, ts_mono=100.0))
    provider = make_recorder_rc_state_provider(rec, now_fn=lambda: clock["t"])

    motor = MockMotor()
    s = TeleopSession(command_sink=motor, rc_state_provider=provider,
                      require_rc_arm=False, now_fn=lambda: clock["t"])
    ok, _ = s.arm(hold_ms=600)
    assert ok
    s.drive(left_f=1.0, right_f=1.0, seq=1)
    assert s.tick() == "driving"          # telemetry is fresh -> RC armed

    # Telemetry stops updating (ts_mono frozen at 100.0); the clock advances
    # past the 1 s staleness bound.
    clock["t"] = 101.5
    assert provider() == (False, False)   # stale -> fail-safe
    assert s.tick() == "rc_override"
    assert s.armed is False
    assert motor.last == NEUTRAL


def test_provider_none_when_no_telemetry_source():
    """No recorder / never-produced data -> None (bench mode unchanged)."""
    assert make_recorder_rc_state_provider(None)() is None
    rec = FakeRecorder(None)            # recorder present but no snapshot yet
    assert make_recorder_rc_state_provider(rec)() is None


def test_provider_fresh_telemetry_reports_state():
    clock = {"t": 50.0}
    rec = FakeRecorder(FakeTelem(is_armed=True, emergency_active=False, ts_mono=50.0))
    provider = make_recorder_rc_state_provider(rec, now_fn=lambda: clock["t"])
    assert provider() == (True, False)
    rec._t = FakeTelem(is_armed=False, emergency_active=False, ts_mono=50.0)
    assert provider() == (False, False)


# -- (3) reverse-direction speed cap symmetry -------------------------------

def test_reverse_direction_speed_cap_matches_forward_magnitude():
    """A full-reverse stick at a given cap must clamp to the same deflection
    magnitude (distance from neutral) as full-forward at the same cap."""
    # Forward
    sf, mf = make_session()
    arm(sf, now=0.0)
    sf.set_speed("slow")
    sf.drive(left_f=1.0, right_f=1.0, seq=1, now=0.0)
    sf.tick(now=0.0)
    fwd_left, fwd_right = mf.last
    # Reverse
    sr, mr = make_session()
    arm(sr, now=0.0)
    sr.set_speed("slow")
    sr.drive(left_f=-1.0, right_f=-1.0, seq=1, now=0.0)
    sr.tick(now=0.0)
    rev_left, rev_right = mr.last

    # Both directions hit the cap (0.3), so the byte deflection from neutral
    # must be equal in magnitude (forward above neutral, reverse below).
    assert fwd_left > CENTER_OUTPUT_VALUE and rev_left < CENTER_OUTPUT_VALUE
    assert (fwd_left - CENTER_OUTPUT_VALUE) == (CENTER_OUTPUT_VALUE - rev_left)
    assert (fwd_right - CENTER_OUTPUT_VALUE) == (CENTER_OUTPUT_VALUE - rev_right)
    # And it equals the direct mapping of the capped value.
    assert (rev_left, rev_right) == floats_to_bytes(-0.3, -0.3)


# -- (4) single-driver lock -------------------------------------------------

def test_driver_lock_second_client_rejected_estop_accepted_release_on_trip():
    s, motor = make_session()
    # Client A completes the arm ceremony -> becomes the driver.
    ok, reason = s.arm(hold_ms=600, client_id="A", now=0.0)
    assert ok, reason
    assert s.drive(left_f=1.0, right_f=1.0, seq=1, client_id="A", now=0.0) == "accepted"
    assert s.tick(now=0.0) == "driving"

    # Client B cannot drive or arm the session.
    assert s.drive(left_f=-1.0, right_f=-1.0, seq=2, client_id="B", now=0.0) == "not_driver"
    okB, reasonB = s.arm(hold_ms=600, client_id="B", now=0.0)
    assert not okB and reasonB == "not_driver"

    # B's e-stop IS accepted — never lock out a stop.
    s.estop(client_id="B")
    assert s.estop_latched is True
    assert s.armed is False
    assert motor.last == NEUTRAL

    # The lock was released by the e-stop: after clearing, B can now arm.
    s.clear_estop()
    okB2, reasonB2 = s.arm(hold_ms=600, client_id="B", now=0.1)
    assert okB2, reasonB2


def test_driver_lock_released_after_deadman_trip():
    s, _ = make_session()
    ok, _ = s.arm(hold_ms=600, client_id="A", now=0.0)
    assert ok
    s.drive(left_f=1.0, right_f=1.0, seq=1, client_id="A", now=0.0)
    s.tick(now=0.0)
    # Heartbeats stop -> deadman trip releases the lock.
    assert s.tick(now=0.3) == "deadman_trip"
    # A different client can now take over.
    okB, reasonB = s.arm(hold_ms=600, client_id="B", now=0.4)
    assert okB, reasonB
    assert s.drive(left_f=0.5, right_f=0.5, seq=2, client_id="B", now=0.4) == "accepted"


def test_driver_lock_released_on_disconnect():
    s, _ = make_session()
    ok, _ = s.arm(hold_ms=600, client_id="A", now=0.0)
    assert ok
    s.release_driver("A")           # simulate WS close for the driver
    assert s.armed is False         # ownerless session disarmed
    okB, reasonB = s.arm(hold_ms=600, client_id="B", now=0.1)
    assert okB, reasonB


# -- (5) e-stop physical-clear gate -----------------------------------------

def test_estop_clear_gate_with_rc_present():
    """With RC available, clear is refused until an RC ch3 cycle (False->True)."""
    rc = {"v": (True, False)}   # rc_armed, rc_estop
    s, _ = make_session(rc_state=rc)
    arm(s, now=0.0)
    s.tick(now=0.0)             # provider returns non-None -> rc_ever_seen=True
    s.estop()
    assert s.estop_latched is True

    # Tick while still RC-armed: no ch3 cycle yet -> clear refused.
    s.tick(now=0.1)
    ok, reason = s.clear_estop()
    assert not ok and reason == "cycle_rc_ch3"
    assert s.estop_latched is True

    # RC ch3 goes False (disarm) then True (re-arm) -> physical re-arm observed.
    rc["v"] = (False, False)
    s.tick(now=0.2)
    rc["v"] = (True, False)
    s.tick(now=0.3)
    ok2, reason2 = s.clear_estop()
    assert ok2 and reason2 == "cleared"
    assert s.estop_latched is False


def test_estop_clear_bench_mode_two_tap_clear_still_works():
    """No RC ever observed -> the existing browser clear works with no gate."""
    s, _ = make_session()          # no rc_state provider
    arm(s, now=0.0)
    s.estop()
    ok, reason = s.clear_estop()
    assert ok and reason == "cleared"
    assert s.estop_latched is False


# -- (7) one real-thread watchdog integration test --------------------------

def test_real_watchdog_thread_trips_deadman_and_disarms():
    """Start the actual watchdog thread, arm + drive, stop sending, then assert
    it writes neutral and disarms on its own. Uses real wall-clock time."""
    motor = MockMotor()
    s = TeleopSession(command_sink=motor, require_rc_arm=False)
    s.start_watchdog()
    try:
        ok, reason = s.arm(hold_ms=600)
        assert ok, reason
        # Send a few fresh drives so the thread actually writes drive bytes.
        for _ in range(5):
            s.drive(left_f=1.0, right_f=1.0, seq=int(time.monotonic() * 1e6))
            time.sleep(0.02)
        assert s.armed is True
        # Stop sending heartbeats; the deadman (250 ms) must trip within ~0.4 s.
        time.sleep(0.4)
        assert s.armed is False
        assert s.tripped_reason == "deadman"
        assert motor.last == NEUTRAL
    finally:
        s.stop_watchdog()


# ==========================================================================
# Auth (Item F) — token resolution + Flask route gating.
# ==========================================================================

def test_resolve_token_explicit_wins():
    assert resolve_teleop_token("explicit-tok", env={}) == "explicit-tok"


def test_resolve_token_env_set_empty_disables_auth(tmp_path):
    # Env present-but-empty -> auth OFF (bench).
    assert resolve_teleop_token(env={"WALL_E_TELEOP_TOKEN": ""}) == ""


def test_resolve_token_env_nonempty(tmp_path):
    assert resolve_teleop_token(env={"WALL_E_TELEOP_TOKEN": "secret"}) == "secret"


def test_resolve_token_file_generated_when_env_unset(tmp_path):
    tok_file = tmp_path / "teleop_token"
    tok = resolve_teleop_token(env={}, token_path=str(tok_file))
    assert tok and len(tok) == 32          # secrets.token_hex(16) -> 32 hex chars
    assert tok_file.exists()
    # chmod 600
    import stat
    assert (tok_file.stat().st_mode & 0o777) == 0o600
    # Stable on second resolution.
    assert resolve_teleop_token(env={}, token_path=str(tok_file)) == tok


def _make_flask_app(token, tmp_path):
    from flask import Flask
    from pi_app.web.teleop import register_teleop
    app = Flask(__name__)
    motor = MockMotor()
    s = TeleopSession(command_sink=motor, require_rc_arm=False)
    # Force a fixed token via the env override path.
    register_teleop(app, s, token=token,
                    token_path=str(tmp_path / "tok"))
    return app, s


def test_auth_routes_gated_by_token(tmp_path):
    app, _ = _make_flask_app("T0K", tmp_path)
    c = app.test_client()
    # No token -> 401 on the page, REST mirror, and status.
    assert c.get("/drive").status_code == 401
    assert c.get("/api/teleop/session/status").status_code == 401
    assert c.post("/api/teleop/session/arm", json={"hold_ms": 600}).status_code == 401
    # With ?token= -> ok.
    assert c.get("/drive?token=T0K").status_code == 200
    assert c.get("/api/teleop/session/status?token=T0K").status_code == 200
    # With X-Teleop-Token header -> ok (fixes WS/REST inconsistency).
    assert c.get("/api/teleop/session/status",
                 headers={"X-Teleop-Token": "T0K"}).status_code == 200
    # Wrong token -> 401.
    assert c.get("/api/teleop/session/status?token=nope").status_code == 401


def test_auth_pwa_routes_stay_open(tmp_path):
    app, _ = _make_flask_app("T0K", tmp_path)
    c = app.test_client()
    assert c.get("/drive/manifest.json").status_code == 200
    assert c.get("/drive/icon.svg").status_code == 200
    assert c.get("/drive/icon-192.png").status_code == 200


def test_auth_disabled_when_env_empty(tmp_path):
    import os
    prev = os.environ.get("WALL_E_TELEOP_TOKEN")
    os.environ["WALL_E_TELEOP_TOKEN"] = ""   # set-but-empty -> auth off
    try:
        from flask import Flask
        from pi_app.web.teleop import register_teleop
        app = Flask(__name__)
        s = TeleopSession(command_sink=MockMotor(), require_rc_arm=False)
        # Pass falsy token (like oak_viewer's os.environ.get(..,"")) -> resolves
        # to env-empty -> auth disabled.
        register_teleop(app, s, token="", token_path=str(tmp_path / "tok"))
        c = app.test_client()
        assert c.get("/drive").status_code == 200          # open
        assert c.get("/api/teleop/session/status").status_code == 200
    finally:
        if prev is None:
            os.environ.pop("WALL_E_TELEOP_TOKEN", None)
        else:
            os.environ["WALL_E_TELEOP_TOKEN"] = prev


# ==========================================================================
# Camera feed (backlog item F — optional camera on /drive page).
# ==========================================================================

def _make_flask_app_with_camera(token, tmp_path, frame_source=None):
    """Like _make_flask_app but optionally wires in a frame_source."""
    from flask import Flask
    from pi_app.web.teleop import register_teleop
    app = Flask(__name__)
    motor = MockMotor()
    s = TeleopSession(command_sink=motor, require_rc_arm=False)
    register_teleop(app, s, token=token,
                    token_path=str(tmp_path / "tok"),
                    frame_source=frame_source)
    return app, s


_FAKE_JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 16 + b"\xff\xd9"  # minimal JPEG sentinel


def test_camera_frame_returns_jpeg_with_valid_token(tmp_path):
    """GET /api/teleop/camera/frame returns the bytes from frame_source."""
    def _src():
        return _FAKE_JPEG

    app, _ = _make_flask_app_with_camera("CAM_TOK", tmp_path, frame_source=_src)
    c = app.test_client()
    r = c.get("/api/teleop/camera/frame?token=CAM_TOK")
    assert r.status_code == 200
    assert r.content_type == "image/jpeg"
    assert r.data == _FAKE_JPEG


def test_camera_frame_401_without_token(tmp_path):
    """Omitting the token on a token-gated app returns 401."""
    app, _ = _make_flask_app_with_camera("CAM_TOK", tmp_path, frame_source=lambda: _FAKE_JPEG)
    c = app.test_client()
    assert c.get("/api/teleop/camera/frame").status_code == 401
    assert c.get("/api/teleop/camera/frame?token=wrong").status_code == 401


def test_camera_frame_503_when_source_returns_none(tmp_path):
    """frame_source returning None (no frame yet) gives 503."""
    app, _ = _make_flask_app_with_camera("CAM_TOK", tmp_path, frame_source=lambda: None)
    c = app.test_client()
    r = c.get("/api/teleop/camera/frame?token=CAM_TOK")
    assert r.status_code == 503


def test_camera_frame_503_when_no_source_registered(tmp_path):
    """No frame_source registered at all gives 503."""
    app, _ = _make_flask_app_with_camera("CAM_TOK", tmp_path, frame_source=None)
    c = app.test_client()
    r = c.get("/api/teleop/camera/frame?token=CAM_TOK")
    assert r.status_code == 503


def test_camera_available_in_status_with_source(tmp_path):
    """/api/teleop/session/status reports camera_available=True when source given."""
    app, _ = _make_flask_app_with_camera("CAM_TOK", tmp_path, frame_source=lambda: _FAKE_JPEG)
    c = app.test_client()
    import json as _json
    r = c.get("/api/teleop/session/status?token=CAM_TOK")
    assert r.status_code == 200
    data = _json.loads(r.data)
    assert data["camera_available"] is True


def test_camera_available_false_when_no_source(tmp_path):
    """/api/teleop/session/status reports camera_available=False when no source."""
    app, _ = _make_flask_app_with_camera("CAM_TOK", tmp_path, frame_source=None)
    c = app.test_client()
    import json as _json
    r = c.get("/api/teleop/session/status?token=CAM_TOK")
    assert r.status_code == 200
    data = _json.loads(r.data)
    assert data["camera_available"] is False


def test_register_teleop_backward_compat_no_frame_source(tmp_path):
    """register_teleop without frame_source works; existing routes unaffected."""
    app, _ = _make_flask_app("T0K", tmp_path)   # uses old helper (no frame_source)
    c = app.test_client()
    # Core drive routes still work.
    assert c.get("/drive?token=T0K").status_code == 200
    assert c.get("/api/teleop/session/status?token=T0K").status_code == 200
    # Camera frame returns 503 (no source), not 404 or 500.
    assert c.get("/api/teleop/camera/frame?token=T0K").status_code == 503


def test_camera_ws_route_registered_only_when_frame_source_present(tmp_path):
    """The /ws/camera URL pattern is only in the app's URL map when
    frame_source is provided.  (No live socket needed — we inspect url_map.)"""
    # With source: /ws/camera must appear in the URL map.
    app_with, _ = _make_flask_app_with_camera(
        "T0K", tmp_path, frame_source=lambda: _FAKE_JPEG
    )
    rules_with = {r.rule for r in app_with.url_map.iter_rules()}
    assert "/ws/camera" in rules_with

    # Without source: /ws/camera must NOT appear.
    app_without, _ = _make_flask_app_with_camera("T0K", tmp_path, frame_source=None)
    rules_without = {r.rule for r in app_without.url_map.iter_rules()}
    assert "/ws/camera" not in rules_without


# ==========================================================================
# frame_client_hook — REST camera frame path
# ==========================================================================

def _make_flask_app_with_hook(token, tmp_path, frame_source, frame_client_hook):
    """Build a Flask test app with both frame_source and frame_client_hook wired in."""
    from flask import Flask
    from pi_app.web.teleop import register_teleop
    app = Flask(__name__)
    motor = MockMotor()
    s = TeleopSession(command_sink=motor, require_rc_arm=False)
    register_teleop(app, s, token=token,
                    token_path=str(tmp_path / "tok"),
                    frame_source=frame_source,
                    frame_client_hook=frame_client_hook)
    return app, s


def test_rest_camera_hook_called_and_frame_returned_after_poll(tmp_path):
    """REST /api/teleop/camera/frame: hook(True) called, frame_source returns
    None twice then bytes, endpoint returns 200 with the bytes, hook(False) called."""
    hook_calls = []
    call_count = [0]

    def _frame_source():
        call_count[0] += 1
        # Return None for first two calls, then the JPEG.
        if call_count[0] < 3:
            return None
        return _FAKE_JPEG

    def _hook(connected: bool):
        hook_calls.append(connected)

    app, _ = _make_flask_app_with_hook("TOK", tmp_path, _frame_source, _hook)
    c = app.test_client()
    r = c.get("/api/teleop/camera/frame?token=TOK")
    assert r.status_code == 200
    assert r.data == _FAKE_JPEG
    # hook(True) before poll, hook(False) in finally — exactly once each.
    assert hook_calls.count(True) == 1
    assert hook_calls.count(False) == 1
    assert hook_calls[0] is True    # True was first
    assert hook_calls[-1] is False  # False was last


def test_rest_camera_no_hook_503_when_source_returns_none(tmp_path):
    """No hook provided: immediate 503 when frame_source returns None, unchanged."""
    app, _ = _make_flask_app_with_camera("TOK", tmp_path, frame_source=lambda: None)
    c = app.test_client()
    r = c.get("/api/teleop/camera/frame?token=TOK")
    assert r.status_code == 503


def test_rest_camera_hook_raises_does_not_500_endpoint(tmp_path):
    """A hook that raises must not propagate; the frame is still served if available."""
    call_count = [0]

    def _frame_source():
        call_count[0] += 1
        if call_count[0] < 2:
            return None
        return _FAKE_JPEG

    def _bad_hook(connected: bool):
        raise RuntimeError("hook exploded")

    app, _ = _make_flask_app_with_hook("TOK", tmp_path, _frame_source, _bad_hook)
    c = app.test_client()
    r = c.get("/api/teleop/camera/frame?token=TOK")
    # Hook errors must not kill the endpoint — frame was found during poll.
    assert r.status_code == 200
    assert r.data == _FAKE_JPEG


def test_register_teleop_backward_compat_without_frame_client_hook(tmp_path):
    """register_teleop without frame_client_hook kwarg is fully backward-compatible.
    Existing routes work and camera/frame returns 503 (no hook, source returns None)."""
    from flask import Flask
    from pi_app.web.teleop import register_teleop
    app = Flask(__name__)
    motor = MockMotor()
    s = TeleopSession(command_sink=motor, require_rc_arm=False)
    # Deliberately omit frame_client_hook — must not raise.
    register_teleop(app, s, token="T0K",
                    token_path=str(tmp_path / "tok"),
                    frame_source=lambda: None)
    c = app.test_client()
    assert c.get("/drive?token=T0K").status_code == 200
    assert c.get("/api/teleop/session/status?token=T0K").status_code == 200
    # Without hook, None frame -> immediate 503.
    assert c.get("/api/teleop/camera/frame?token=T0K").status_code == 503


# ==========================================================================
# Wave 1 web-surface hardening:
#   - legacy /api/teleop + /api/teleop/stop are retired (410 Gone)
#   - /api/follow_me and /api/nav/start /api/nav/go refuse (409) while the
#     /drive TeleopSession e-stop is latched
#   - the /drive session itself is unaffected (still arms/drives normally)
# ==========================================================================

def test_legacy_api_teleop_retired_410(tmp_path):
    """POST /api/teleop and /api/teleop/stop are gone — 410, not silently 200."""
    from config import OakWebViewerConfig
    from pi_app.web.oak_viewer import create_app

    app = create_app(None, OakWebViewerConfig(), controller=None)
    c = app.test_client()
    r1 = c.post("/api/teleop", json={"left_f": 1.0, "right_f": 1.0})
    assert r1.status_code == 410
    assert "error" in r1.get_json()
    # No trace of the old 200 payload shape (which echoed left_byte/right_byte
    # after writing the shared override file) — this is an error, not a motor ack.
    assert b"left_byte" not in r1.data

    r2 = c.post("/api/teleop/stop")
    assert r2.status_code == 410
    assert "error" in r2.get_json()


class _SpyController:
    """Minimal controller stand-in that records whether autonomy was armed.

    ``activate_follow_me`` would only be reached if the e-stop gate in
    ``api_follow_me_toggle`` let the request through — so a call count of 0
    after a 409 proves the gate short-circuited before touching the
    controller at all.
    """

    def __init__(self):
        self.activate_calls = 0
        self._mode = "MANUAL"

    def activate_follow_me(self):
        self.activate_calls += 1
        self._mode = "FOLLOW_ME"
        return True

    def deactivate_follow_me(self):
        self._mode = "MANUAL"


def test_follow_me_refuses_409_while_session_estop_latched():
    from config import OakWebViewerConfig
    from pi_app.web.oak_viewer import create_app

    ctrl = _SpyController()
    app = create_app(None, OakWebViewerConfig(), controller=ctrl,
                     estop_check=lambda: True)
    c = app.test_client()
    r = c.post("/api/follow_me")
    assert r.status_code == 409
    assert "e-stop" in r.get_json()["error"].lower()
    assert ctrl.activate_calls == 0


def test_follow_me_allowed_when_session_not_estopped():
    from config import OakWebViewerConfig
    from pi_app.web.oak_viewer import create_app

    ctrl = _SpyController()
    app = create_app(None, OakWebViewerConfig(), controller=ctrl,
                     estop_check=lambda: False)
    c = app.test_client()
    r = c.post("/api/follow_me")
    assert r.status_code == 200
    assert ctrl.activate_calls == 1


def test_follow_me_unaffected_when_estop_check_omitted():
    """estop_check defaults to None — behavior identical to before the gate existed."""
    from config import OakWebViewerConfig
    from pi_app.web.oak_viewer import create_app

    ctrl = _SpyController()
    app = create_app(None, OakWebViewerConfig(), controller=ctrl)
    c = app.test_client()
    r = c.post("/api/follow_me")
    assert r.status_code == 200
    assert ctrl.activate_calls == 1


def test_nav_start_refuses_409_while_session_estop_latched():
    from pi_app.web.waypoint_nav_ui import create_nav_blueprint
    from flask import Flask

    app = Flask(__name__)
    app.register_blueprint(
        create_nav_blueprint(controller=object(), estop_check=lambda: True)
    )
    c = app.test_client()
    r = c.post("/api/nav/start", json={"waypoints": [{"lat": 1.0, "lon": 2.0}]})
    assert r.status_code == 409
    assert "e-stop" in r.get_json()["error"].lower()


def test_nav_go_refuses_409_while_session_estop_latched():
    from pi_app.web.waypoint_nav_ui import create_nav_blueprint
    from flask import Flask

    app = Flask(__name__)
    app.register_blueprint(
        create_nav_blueprint(controller=object(), estop_check=lambda: True)
    )
    c = app.test_client()
    r = c.post("/api/nav/go", json={"lat": 1.0, "lon": 2.0})
    assert r.status_code == 409
    assert "e-stop" in r.get_json()["error"].lower()


def test_nav_start_and_go_unaffected_when_estop_check_omitted():
    """No estop_check passed (None default) — the 503 'no controller' path
    still fires for a bare object() controller with no _waypoint_nav, proving
    the gate itself did not swallow the request (no 409 short-circuit)."""
    from pi_app.web.waypoint_nav_ui import create_nav_blueprint
    from flask import Flask

    app = Flask(__name__)
    app.register_blueprint(create_nav_blueprint(controller=object()))
    c = app.test_client()
    r = c.post("/api/nav/start", json={"waypoints": [{"lat": 1.0, "lon": 2.0}]})
    assert r.status_code != 409
    r2 = c.post("/api/nav/go", json={"lat": 1.0, "lon": 2.0})
    assert r2.status_code != 409


def test_nav_start_not_gated_when_estop_check_returns_false():
    from pi_app.web.waypoint_nav_ui import create_nav_blueprint
    from flask import Flask

    app = Flask(__name__)
    app.register_blueprint(
        create_nav_blueprint(controller=object(), estop_check=lambda: False)
    )
    c = app.test_client()
    r = c.post("/api/nav/start", json={"waypoints": [{"lat": 1.0, "lon": 2.0}]})
    # Not gated by e-stop; falls through to whatever object() lacking
    # _waypoint_nav produces (a 500 from the AttributeError, not a 409).
    assert r.status_code != 409


def test_drive_session_still_arms_and_drives_with_estop_gate_wired(tmp_path):
    """The hardening pass must not touch /drive's own semantics: arming and
    driving through the REST mirror still works exactly as before, and the
    TeleopSession's own estop_latched flag is what the OakWebViewer-level
    gate reads (verified directly here without needing the full OakWebViewer
    thread)."""
    app, s = _make_flask_app("T0K", tmp_path)
    c = app.test_client()

    # Arm + drive still works normally.
    r = c.post("/api/teleop/session/arm?token=T0K", json={"hold_ms": 600})
    assert r.status_code == 200
    assert r.get_json()["ok"] is True
    assert s.estop_latched is False

    r = c.post("/api/teleop/session/drive?token=T0K",
              json={"left": 0.5, "right": 0.5, "seq": 1, "t": time.time() * 1000})
    assert r.status_code == 200

    # e-stop latches on the session object exactly as the gate expects to read it.
    c.post("/api/teleop/session/estop?token=T0K")
    assert s.estop_latched is True

    # And clearing it un-latches, restoring normal semantics.
    ok, _ = s.clear_estop()
    assert ok is True
    assert s.estop_latched is False
