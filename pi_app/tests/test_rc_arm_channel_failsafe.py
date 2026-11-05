"""RC arm-channel fail-safe (2026-11-05): arm persistence and mid-band disarm.

The HotRC "CH3/CH4 hybrid (jog)" mix rests ch3 at ~1500 us and sends ~2004
or ~1000 only in one short burst per button press. update_safety now arms
only on a held high (SafetyParams.arm_high_persist_s) and, while armed,
disarms and latches when ch3 rests mid-band for longer than
SafetyParams.arm_midband_disarm_s. A ch3-low disarm and the ch5 e-stop keep
their old timing. Real-log replays are in test_rc_arm_channel_replay.py.

Times are multiples of 1/16 s so the threshold comparisons are exact.
"""
import logging
from unittest.mock import patch

import pytest

from pi_app.control.controller import Controller, RCInputs
from pi_app.control.safety import (
    SafetyEvent,
    SafetyParams,
    SafetyState,
    force_disarm,
    update_safety,
)

P = SafetyParams()
HIGH, LOW, MID = 2004, 1000, 1500
CH4_LOW, CH5_LOW, CH5_HIGH = 992, 996, 2000


def tick(state, t, ch3, ch4=CH4_LOW, ch5=CH5_LOW, params=P):
    return update_safety(state, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5, now_epoch_s=t, params=params)


def armed_state(t0=10.0, params=P):
    """Arm by holding ch3 high for 0.25 s (>= the 0.2 s persistence); return (state, t_armed)."""
    s, ev = tick(SafetyState(), t0, HIGH, params=params)
    assert ev == []
    t = t0 + 0.25
    s, ev = tick(s, t, HIGH, params=params)
    assert ev == [SafetyEvent.ARMED] and s.is_armed
    return s, t


def test_defaults():
    assert P.arm_high_persist_s == 0.2
    assert P.arm_midband_disarm_s == 0.5
    s = SafetyState()
    assert s.ch3_high_since_epoch_s is None
    assert s.ch3_midband_since_epoch_s is None


# --- Arm persistence ---------------------------------------------------------


def test_single_high_frame_does_not_arm():
    s, ev = tick(SafetyState(), 10.0, HIGH)
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.0625, MID)
    assert not s.is_armed and ev == []
    assert s.ch3_high_since_epoch_s is None


def test_hybrid_burst_of_140ms_does_not_arm():
    s = SafetyState()
    for t in (10.0, 10.0625, 10.125):  # 30 Hz-ish ticks inside one burst
        s, ev = tick(s, t, HIGH)
        assert ev == []
    s, ev = tick(s, 10.1875, MID)
    assert not s.is_armed and ev == []


def test_held_high_arms_exactly_at_the_persistence_time():
    params = SafetyParams(arm_high_persist_s=0.25)
    s, ev = tick(SafetyState(), 10.0, HIGH, params=params)
    s, ev = tick(s, 10.1875, HIGH, params=params)
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.25, HIGH, params=params)  # held exactly 0.25 s
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_held_high_arms_with_default_persistence():
    s, ev = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.125, HIGH)
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.25, HIGH)
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_high_threshold_value_counts_as_high():
    s, _ = tick(SafetyState(), 10.0, P.arm_high_threshold_us)
    s, ev = tick(s, 10.25, P.arm_high_threshold_us)
    assert s.is_armed and ev == [SafetyEvent.ARMED]


@pytest.mark.parametrize("gap_ch3", [MID, P.arm_high_threshold_us - 1, LOW])
def test_a_non_high_frame_restarts_the_hold(gap_ch3):
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.125, gap_ch3)
    assert not s.is_armed and s.ch3_high_since_epoch_s is None
    s, ev = tick(s, 10.1875, HIGH)
    s, ev = tick(s, 10.3125, HIGH)  # 0.3125 since the first high, 0.125 since the restart
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.4375, HIGH)  # 0.25 since the restart
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_arm_still_waits_for_the_debounce_after_a_disarm():
    s, t = armed_state()
    s, ev = tick(s, t + 1.0, LOW)
    assert ev == [SafetyEvent.DISARMED]
    t_dis = t + 1.0
    s, ev = tick(s, t_dis + 0.0625, HIGH)
    s, ev = tick(s, t_dis + 0.25, HIGH)  # held 0.1875 s
    assert not s.is_armed
    s, ev = tick(s, t_dis + 0.28125, HIGH)  # held 0.21875 s, debounce 0.28125 < 0.3
    assert not s.is_armed and ev == []
    s, ev = tick(s, t_dis + 0.3125, HIGH)
    assert s.is_armed and ev == [SafetyEvent.ARMED]


# --- ch3-low disarm keeps its timing -----------------------------------------


def test_one_low_frame_disarms_immediately():
    s, t = armed_state()
    s, ev = tick(s, t + 2.0, LOW)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]
    assert not s.rearm_requires_switch_cycle


def test_low_threshold_value_counts_as_low():
    s, t = armed_state()
    s, ev = tick(s, t + 2.0, P.arm_low_threshold_us)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]


def test_post_arm_debounce_runs_from_when_ch3_went_high():
    """The old rule armed on the first high frame; its 0.3 s debounce ran
    from there. The persistence must not hold a ch3-low disarm off longer
    (2026-11-04 19:42:18: switch up 0.45 s, then down)."""
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.25, HIGH)
    assert ev == [SafetyEvent.ARMED]
    assert s.last_transition_epoch_s == 10.0
    # 0.3125 s after the switch went up, 0.0625 s after the arm grant.
    s, ev = tick(s, 10.3125, LOW)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]


def test_low_inside_the_old_debounce_window_is_still_held_off():
    """Unchanged from the old rule: debounce_seconds after the switch went up."""
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.25, HIGH)
    s, ev = tick(s, 10.28125, LOW)
    assert s.is_armed and ev == []


# --- Mid-band disarm ---------------------------------------------------------


def test_midband_dip_shorter_than_the_limit_does_not_disarm():
    s, t = armed_state()
    s, ev = tick(s, t + 1.0, MID)
    s, ev = tick(s, t + 1.25, MID)
    s, ev = tick(s, t + 1.5, MID)  # exactly 0.5 s: not longer than the limit
    assert s.is_armed and ev == []
    s, ev = tick(s, t + 1.5625, HIGH)
    assert s.is_armed and ev == [] and s.ch3_midband_since_epoch_s is None
    # A new dip starts its own clock.
    s, ev = tick(s, t + 2.0, MID)
    s, ev = tick(s, t + 2.4375, MID)
    assert s.is_armed and ev == []


def test_midband_longer_than_the_limit_disarms_and_latches():
    s, t = armed_state()
    s, ev = tick(s, t + 1.0, MID)
    s, ev = tick(s, t + 1.5, MID)
    assert s.is_armed
    s, ev = tick(s, t + 1.5625, MID)
    assert not s.is_armed
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]
    assert s.rearm_requires_switch_cycle
    assert s.last_transition_epoch_s == t + 1.5625


@pytest.mark.parametrize("ch3", [P.arm_low_threshold_us + 1, P.arm_high_threshold_us - 1])
def test_band_edges_count_as_midband(ch3):
    s, t = armed_state()
    s, ev = tick(s, t + 1.0, ch3)
    s, ev = tick(s, t + 1.5625, ch3)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


def test_midband_disarm_exits_follow_me():
    s, t = armed_state()
    s, ev = tick(s, t + 0.5, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED] and s.follow_me_active
    # Hybrid mix: ch4 rests mid-band too, so follow-me would never exit on ch4.
    s, ev = tick(s, t + 1.0, MID, ch4=MID)
    assert s.follow_me_active and ev == []
    s, ev = tick(s, t + 1.5625, MID, ch4=MID)
    assert ev == [
        SafetyEvent.FOLLOW_ME_EXITED,
        SafetyEvent.DISARMED,
        SafetyEvent.ARM_MIDBAND_DISARM,
    ]
    assert not s.follow_me_active and not s.is_armed


def test_midband_disarm_does_not_wait_for_the_debounce():
    params = SafetyParams(debounce_seconds=5.0)
    s, t = armed_state(params=params)
    s, ev = tick(s, t + 0.0625, MID, params=params)
    s, ev = tick(s, t + 0.625, MID, params=params)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


def test_latch_after_midband_disarm_clears_only_on_low():
    s, t = armed_state()
    s, _ = tick(s, t + 1.0, MID)
    s, ev = tick(s, t + 1.5625, MID)
    assert SafetyEvent.ARM_MIDBAND_DISARM in ev
    t = t + 1.5625
    # Held high (mix switched off with the switch up) and mid-band: no arm.
    for dt, ch3 in ((1.0, HIGH), (1.5, HIGH), (3.0, HIGH), (3.5, MID), (5.0, MID), (5.5, HIGH), (7.0, HIGH)):
        s, ev = tick(s, t + dt, ch3)
        assert not s.is_armed and ev == [] and s.rearm_requires_switch_cycle, (dt, ch3)
    # One low frame clears the latch; a held high then arms.
    s, ev = tick(s, t + 8.0, LOW)
    assert not s.rearm_requires_switch_cycle and ev == []
    s, ev = tick(s, t + 8.0625, HIGH)
    s, ev = tick(s, t + 8.3125, HIGH)
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_disarmed_midband_does_nothing():
    s = SafetyState()
    for i in range(40):
        s, ev = tick(s, 10.0 + i * 0.25, MID)
        assert ev == [] and not s.is_armed and not s.rearm_requires_switch_cycle


def test_backward_clock_step_restarts_the_midband_run():
    s, t = armed_state(t0=1000.0)
    s, ev = tick(s, t + 1.0, MID)
    s, ev = tick(s, t - 60.0, MID)  # NTP steps the wall clock back 61 s
    assert s.is_armed and s.ch3_midband_since_epoch_s == t - 60.0
    s, ev = tick(s, t - 59.4375, MID)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


# --- Untouched paths ---------------------------------------------------------


def test_estop_is_immediate_while_midband():
    s, t = armed_state()
    s, ev = tick(s, t + 1.0, MID)
    s, ev = tick(s, t + 1.0625, MID, ch5=CH5_HIGH)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.EMERGENCY_TRIGGERED]
    assert s.emergency_active and not s.is_armed
    # The e-stop latch owns the state from here: no mid-band event, no arm.
    for dt in (1.0, 2.0, 3.0):
        s, ev = tick(s, t + 1.0625 + dt, MID)
        assert ev == []
    s, _ = tick(s, t + 5.0, HIGH)
    s, ev = tick(s, t + 6.0, HIGH)
    assert ev == [] and not s.is_armed


def test_estop_on_a_single_frame_before_the_arm_hold_ends():
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.0625, HIGH, ch5=CH5_HIGH)
    assert ev == [SafetyEvent.EMERGENCY_TRIGGERED]
    s, ev = tick(s, 11.0, HIGH)
    assert not s.is_armed and ev == []


def test_follow_me_rising_edge_after_the_arm_still_enters():
    s, t = armed_state()
    s, ev = tick(s, t + 1.0, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED]
    s, ev = tick(s, t + 2.0, HIGH, ch4=CH4_LOW)
    assert ev == [SafetyEvent.FOLLOW_ME_EXITED]


def test_ch4_edge_before_the_arm_grant_does_not_enter_follow_me():
    """The hybrid mix moves ch3 and ch4 together; one press must not arm
    and start FOLLOW_ME. A ch4 rising edge after the arm is still needed."""
    s, ev = tick(SafetyState(), 10.0, HIGH, ch4=HIGH)
    assert ev == []
    s, ev = tick(s, 10.25, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.ARMED] and not s.follow_me_active
    s, ev = tick(s, 10.5, HIGH, ch4=CH4_LOW)
    s, ev = tick(s, 10.75, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED]


def test_force_disarm_resets_the_ch3_runs():
    s, t = armed_state()
    s, _ = tick(s, t + 0.5, MID)
    s, ev = force_disarm(s, t + 0.75)
    assert s.ch3_high_since_epoch_s is None
    assert s.ch3_midband_since_epoch_s is None
    assert s.rearm_requires_switch_cycle


# --- Controller: journal warning, telemetry, JSON-log hook -------------------


class _Motor:
    def __init__(self):
        self.stops = 0

    def set_tracks(self, left_byte, right_byte):
        pass

    def stop(self):
        self.stops += 1


class _Relay:
    def __init__(self):
        self.states = []

    def set_armed(self, armed):
        self.states.append(armed)


class _Shutdown:
    def schedule_shutdown(self, delay_seconds):
        raise AssertionError("a mid-band disarm must not schedule a shutdown")


@pytest.fixture
def ctrl():
    clock = {"t": 0.0}
    with patch("pi_app.control.controller.time.monotonic", side_effect=lambda: clock["t"]):
        motor, relay = _Motor(), _Relay()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=_Shutdown())

        def step(t, ch3, ch4=CH4_LOW):
            clock["t"] = t
            rc = RCInputs(
                ch1_us=1500, ch2_us=1500, ch3_us=ch3, ch4_us=ch4, ch5_us=CH5_LOW,
                last_update_epoch_s=t,
            )
            return c.process(rc, now_epoch_s=t)

        yield c, step, motor, relay


def test_controller_logs_and_reports_the_midband_disarm(ctrl, caplog):
    c, step, motor, relay = ctrl
    cmd, ev, _ = step(100.0, HIGH)
    assert not cmd.is_armed
    cmd, ev, _ = step(100.25, HIGH)
    assert cmd.is_armed and SafetyEvent.ARMED in ev
    cmd, ev, telem = step(101.0, MID, ch4=MID)
    assert cmd.is_armed and "arm_midband_disarm" not in telem
    stops = motor.stops
    with caplog.at_level(logging.WARNING, logger="pi_app.control.controller"):
        cmd, ev, telem = step(101.5625, 1504, ch4=MID)
    assert not cmd.is_armed
    assert (cmd.left_byte, cmd.right_byte) == (c.NEUTRAL, c.NEUTRAL)
    assert SafetyEvent.ARM_MIDBAND_DISARM in ev and SafetyEvent.DISARMED in ev
    assert telem["arm_midband_disarm"] == {
        "ch3_us": 1504,
        "ch4_us": MID,
        "midband_s": 0.562,
        "mode": "MANUAL",
    }
    assert telem["rearm_requires_switch_cycle"] is True
    assert relay.states[-1] is False
    assert motor.stops > stops
    lines = [r.getMessage() for r in caplog.records if "MID-BAND" in r.getMessage()]
    assert len(lines) == 1
    assert "ch3 read 1504 us (between 1200 and 1800) for 0.56 s" in lines[0]
    assert "CH3/CH4 mix" in lines[0]
    # Next tick: still disarmed, no second warning or telemetry record.
    caplog.clear()
    cmd, ev, telem = step(102.0, MID, ch4=MID)
    assert not cmd.is_armed and ev == [] and "arm_midband_disarm" not in telem
    assert not [r for r in caplog.records if "MID-BAND" in r.getMessage()]
