"""RC arm-channel fail-safe (2026-11-05): arm hold and mid-band disarm.

The HotRC "CH3/CH4 hybrid (jog)" mix rests ch3 at ~1500 us and sends ~2004
or ~1000 only in one short burst per button press. update_safety now arms
only on a held high (ch3 >= 1800 us on every tick for
SafetyParams.arm_high_persist_s) and, while armed and ch3 is not low,
disarms and latches when ch3 has not been held high for longer than
SafetyParams.arm_midband_disarm_s. A ch3-low disarm and the ch5 e-stop keep
their old timing. All safety timing runs on the monotonic clock
(now_mono_s), never on the wall clock. Real-log replays are in
test_rc_arm_channel_replay.py.

Times are multiples of 1/32 s so the threshold comparisons are exact.
"""
import logging
import math
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
DT = 1.0 / 32  # ~30 Hz control tick, exact in binary


def tick(state, t, ch3, ch4=CH4_LOW, ch5=CH5_LOW, params=P, epoch=None):
    """One tick; t is the safety (monotonic) clock, epoch the wall clock."""
    return update_safety(
        state, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5,
        now_epoch_s=t if epoch is None else epoch, params=params, now_mono_s=t,
    )


def run(state, t0, t1, ch3, **kw):
    """Ticks every DT from t0 to t1 inclusive; returns (state, t_last, all events)."""
    events = []
    n = int(round((t1 - t0) / DT))
    t = t0
    for i in range(n + 1):
        t = t0 + i * DT
        state, ev = tick(state, t, ch3, **kw)
        events.extend(ev)
    return state, t, events


def armed_state(t0=10.0, params=P):
    """Arm by holding ch3 high for 0.25 s (>= the 0.2 s hold); return (state, t_armed)."""
    s, ev = tick(SafetyState(), t0, HIGH, params=params)
    assert ev == []
    t = t0 + 0.25
    s, ev = tick(s, t, HIGH, params=params)
    assert ev == [SafetyEvent.ARMED] and s.is_armed
    assert s.ch3_held_high_mono_s == t
    return s, t


def test_defaults():
    assert P.arm_high_persist_s == 0.2
    assert P.arm_midband_disarm_s == 0.5
    s = SafetyState()
    assert s.last_transition_mono_s == -math.inf
    assert s.ch3_high_since_mono_s is None
    assert s.ch3_held_high_mono_s is None


# --- Arm hold ----------------------------------------------------------------


def test_single_high_frame_does_not_arm():
    s, ev = tick(SafetyState(), 10.0, HIGH)
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.0625, MID)
    assert not s.is_armed and ev == []
    assert s.ch3_high_since_mono_s is None and s.ch3_held_high_mono_s is None


def test_hybrid_burst_of_140ms_does_not_arm():
    s, _t, ev = run(SafetyState(), 10.0, 10.125, HIGH)  # 5 ticks, 125 ms
    assert ev == []
    s, ev = tick(s, 10.15625, MID)
    assert not s.is_armed and ev == []


def test_held_high_arms_exactly_at_the_hold_time():
    params = SafetyParams(arm_high_persist_s=0.25)
    s, ev = tick(SafetyState(), 10.0, HIGH, params=params)
    s, ev = tick(s, 10.1875, HIGH, params=params)
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.25, HIGH, params=params)  # held exactly 0.25 s
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_held_high_arms_with_default_hold():
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
    assert not s.is_armed and s.ch3_high_since_mono_s is None
    s, ev = tick(s, 10.1875, HIGH)
    s, ev = tick(s, 10.3125, HIGH)  # 0.3125 since the first high, 0.125 since the restart
    assert not s.is_armed and ev == []
    s, ev = tick(s, 10.4375, HIGH)  # 0.25 since the restart
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_arm_still_waits_for_the_debounce_after_a_disarm():
    s, t = armed_state()
    s, ev = tick(s, t + 0.25, LOW)
    assert ev == [SafetyEvent.DISARMED]
    t_dis = t + 0.25
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
    s, t, ev = run(s, t + DT, t + 2.0, HIGH)
    assert ev == [] and s.is_armed
    s, ev = tick(s, t + DT, LOW)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]
    assert not s.rearm_requires_switch_cycle


def test_low_threshold_value_counts_as_low():
    s, t = armed_state()
    s, ev = tick(s, t + 0.25, P.arm_low_threshold_us)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]


def test_post_arm_debounce_runs_from_when_ch3_went_high():
    """The old rule armed on the first high frame; its 0.3 s debounce ran
    from there. The hold must not hold a ch3-low disarm off longer
    (2026-11-04 19:42:18: switch up 0.45 s, then down)."""
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.25, HIGH)
    assert ev == [SafetyEvent.ARMED]
    assert s.last_transition_mono_s == 10.0
    # 0.3125 s after the switch went up, 0.0625 s after the arm grant.
    s, ev = tick(s, 10.3125, LOW)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]


def test_low_inside_the_old_debounce_window_is_still_held_off():
    """Unchanged from the old rule: debounce_seconds after the switch went up."""
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.25, HIGH)
    s, ev = tick(s, 10.28125, LOW)
    assert s.is_armed and ev == []


def test_a_low_tick_after_a_dip_takes_the_normal_disarm():
    s, t = armed_state()
    s, t, ev = run(s, t + DT, t + 0.375, MID)
    assert ev == [] and s.is_armed
    s, ev = tick(s, t + DT, LOW)
    assert ev == [SafetyEvent.DISARMED]
    assert not s.rearm_requires_switch_cycle


# --- Mid-band disarm ---------------------------------------------------------


def test_one_33ms_midband_glitch_while_held_high_does_not_disarm():
    s, t = armed_state()
    s, t, ev = run(s, t + DT, t + 2.0, HIGH)
    s, ev2 = tick(s, t + DT, MID)  # one glitch frame
    s, t, ev3 = run(s, t + 2 * DT, t + 4.0, HIGH)
    assert ev + ev2 + ev3 == [] and s.is_armed
    assert s.ch3_held_high_mono_s == t


def test_midband_up_to_the_limit_does_not_disarm():
    s, t_held = armed_state()
    s, t, ev = run(s, t_held + DT, t_held + 0.5, MID)  # last tick exactly 0.5 s after
    assert ev == [] and s.is_armed


def test_midband_past_the_limit_disarms_and_latches():
    s, t_held = armed_state()
    s, t, ev = run(s, t_held + DT, t_held + 0.5, MID)
    s, ev = tick(s, t + DT, MID)
    assert not s.is_armed
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]
    assert s.rearm_requires_switch_cycle
    assert s.last_transition_mono_s == t + DT


def test_a_high_return_that_is_not_yet_held_does_not_save_it():
    """A 0.4375 s dip leaves 62.5 ms of budget, less than the 0.2 s hold."""
    s, t_held = armed_state()
    s, t, ev = run(s, t_held + DT, t_held + 0.4375, MID)
    s, t, ev2 = run(s, t + DT, t + 3 * DT, HIGH)  # back high at +0.46875 .. +0.53125
    assert ev + ev2 == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


def test_a_short_dip_then_held_high_recovers():
    """0.25 s mid, then high: held again 0.2 s later, inside the 0.5 s budget."""
    s, t_held = armed_state()
    s, t, ev = run(s, t_held + DT, t_held + 0.25, MID)
    s, t, ev2 = run(s, t + DT, t + 3.0, HIGH)
    assert ev + ev2 == [] and s.is_armed


def test_button_blips_every_0_4_s_do_not_keep_it_armed():
    """Hybrid mix at rest 1500, a 100 ms high blip every 0.4 s: the blips
    are never held high, so it disarms about 0.5 s after the last held tick."""
    s, t_held = armed_state()
    t = t_held
    trip = None
    for i in range(1, 200):
        t = t_held + i * DT
        phase = (i * DT) % 0.4
        ch3 = HIGH if phase >= 0.3 else MID
        s, ev = tick(s, t, ch3)
        if ev:
            trip = (t, ch3, ev)
            break
    assert trip is not None
    t_trip, _ch3, ev = trip
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]
    assert P.arm_midband_disarm_s < t_trip - t_held <= P.arm_midband_disarm_s + DT
    assert s.rearm_requires_switch_cycle


def test_noise_above_1800_does_not_refresh_the_hold():
    s, t_held = armed_state()
    events = []
    for i in range(1, 40):
        ch3 = HIGH if i % 4 == 0 else MID  # one high sample every 4th tick
        s, ev = tick(s, t_held + i * DT, ch3)
        events.extend(ev)
        if ev:
            break
    assert events == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


@pytest.mark.parametrize("ch3", [P.arm_low_threshold_us + 1, P.arm_high_threshold_us - 1])
def test_band_edges_count_as_midband(ch3):
    s, t = armed_state()
    s, t, ev = run(s, t + DT, t + 0.5, ch3)
    assert ev == []
    s, ev = tick(s, t + DT, ch3)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


def test_midband_disarm_exits_follow_me():
    s, t = armed_state()
    s, ev = tick(s, t + DT, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED] and s.follow_me_active
    # Hybrid mix: ch4 rests mid-band too, so follow-me would never exit on ch4.
    s, t2, ev = run(s, t + 2 * DT, t + DT + 0.5, MID, ch4=MID)
    assert s.follow_me_active and ev == []
    s, ev = tick(s, t2 + DT, MID, ch4=MID)
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
    s, ev = tick(s, t + 0.5625, MID, params=params)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


def test_latch_after_midband_disarm_clears_only_on_low():
    s, t = armed_state()
    s, ev = tick(s, t + 0.5625, MID)
    assert SafetyEvent.ARM_MIDBAND_DISARM in ev
    t = t + 0.5625
    # Held high (mix switched off with the switch up) and mid-band: no arm.
    for t0, t1, ch3 in ((t + 0.5, t + 3.0, HIGH), (t + 3.5, t + 5.0, MID), (t + 5.5, t + 7.0, HIGH)):
        s, _t, ev = run(s, t0, t1, ch3)
        assert not s.is_armed and ev == [] and s.rearm_requires_switch_cycle, (t0, ch3)
    # One low frame clears the latch; a held high then arms.
    s, ev = tick(s, t + 8.0, LOW)
    assert not s.rearm_requires_switch_cycle and ev == []
    s, ev = tick(s, t + 8.0625, HIGH)
    s, ev = tick(s, t + 8.3125, HIGH)
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_disarmed_midband_does_nothing():
    s, _t, ev = run(SafetyState(), 10.0, 20.0, MID)
    assert ev == [] and not s.is_armed and not s.rearm_requires_switch_cycle


def test_hand_built_armed_state_starts_the_clock_on_its_first_tick():
    s = SafetyState(is_armed=True, last_transition_mono_s=10.0)
    s, ev = tick(s, 20.0, MID)
    assert s.is_armed and ev == [] and s.ch3_held_high_mono_s == 20.0
    s, ev = tick(s, 20.53125, MID)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


# --- Clock: the wall clock never moves safety timing --------------------------


def _script():
    """(mono, ch3) ticks: a two-frame burst, a held arm, a dip, a trip."""
    out = [(10.0, HIGH), (10.0 + DT, HIGH), (10.0 + 2 * DT, MID)]
    out += [(11.0 + i * DT, HIGH) for i in range(40)]
    out += [(12.25 + i * DT, MID) for i in range(20)]
    return out


@pytest.mark.parametrize("step_s", [3600.0, -3600.0, 0.05, -0.05])
def test_wall_clock_steps_do_not_change_safety_timing(step_s):
    script = _script()
    ref, ref_trace = SafetyState(), []
    for t, ch3 in script:
        ref, ev = tick(ref, t, ch3, epoch=1.79e9 + t)
        ref_trace.append((ref, ev))
    for k in range(1, len(script)):  # step the wall clock before every tick in turn
        s = SafetyState()
        for i, (t, ch3) in enumerate(script):
            epoch = 1.79e9 + t + (step_s if i >= k else 0.0)
            s, ev = tick(s, t, ch3, epoch=epoch)
            assert (s, ev) == ref_trace[i], (k, i)
    # The reference itself: the burst never armed, the hold did, the dip tripped.
    events = [e for _s, ev in ref_trace for e in ev]
    assert events == [
        SafetyEvent.ARMED,
        SafetyEvent.DISARMED,
        SafetyEvent.ARM_MIDBAND_DISARM,
    ]


def test_forward_wall_clock_step_inside_a_burst_does_not_arm():
    s, ev = tick(SafetyState(), 10.0, HIGH, epoch=1000.0)
    s, ev = tick(s, 10.0 + DT, HIGH, epoch=1000.0 + 3600.0)
    s, ev = tick(s, 10.0 + 2 * DT, MID, epoch=1000.0 + 3600.0 + DT)
    assert not s.is_armed and ev == []


def test_backward_wall_clock_step_does_not_postpone_the_midband_disarm():
    s, t_held = armed_state()
    s, ev = tick(s, t_held + 0.25, MID, epoch=1000.0)
    s, ev = tick(s, t_held + 0.53125, MID, epoch=1000.0 - 600.0)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]


# --- Untouched paths ---------------------------------------------------------


def test_estop_is_immediate_while_midband():
    s, t = armed_state()
    s, ev = tick(s, t + 0.25, MID)
    s, ev = tick(s, t + 0.28125, MID, ch5=CH5_HIGH)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.EMERGENCY_TRIGGERED]
    assert s.emergency_active and not s.is_armed
    # The e-stop latch owns the state from here: no mid-band event, no arm.
    s, _t, ev = run(s, t + 1.0, t + 4.0, MID)
    assert ev == []
    s, _t, ev = run(s, t + 5.0, t + 7.0, HIGH)
    assert ev == [] and not s.is_armed


def test_estop_clears_the_ch3_runs():
    s, t = armed_state()
    s, _t, _ev = run(s, t + DT, t + 0.5, HIGH)
    assert s.ch3_high_since_mono_s is not None and s.ch3_held_high_mono_s is not None
    s, ev = tick(s, t + 0.53125, HIGH, ch5=CH5_HIGH)
    assert SafetyEvent.EMERGENCY_TRIGGERED in ev
    assert s.ch3_high_since_mono_s is None and s.ch3_held_high_mono_s is None
    s, _t, ev = run(s, t + 1.0, t + 2.0, HIGH)
    assert ev == [] and s.ch3_high_since_mono_s is None and s.ch3_held_high_mono_s is None


def test_estop_on_a_single_frame_before_the_arm_hold_ends():
    s, _ = tick(SafetyState(), 10.0, HIGH)
    s, ev = tick(s, 10.0625, HIGH, ch5=CH5_HIGH)
    assert ev == [SafetyEvent.EMERGENCY_TRIGGERED]
    s, ev = tick(s, 11.0, HIGH)
    assert not s.is_armed and ev == []


def test_follow_me_rising_edge_after_the_arm_still_enters():
    s, t = armed_state()
    s, ev = tick(s, t + DT, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED]
    s, ev = tick(s, t + 2 * DT, HIGH, ch4=CH4_LOW)
    assert ev == [SafetyEvent.FOLLOW_ME_EXITED]


def test_ch4_edge_before_the_arm_grant_does_not_enter_follow_me():
    """The hybrid mix moves ch3 and ch4 together; one press must not arm
    and start FOLLOW_ME. A ch4 rising edge after the arm is still needed."""
    s, ev = tick(SafetyState(), 10.0, HIGH, ch4=HIGH)
    assert ev == []
    s, ev = tick(s, 10.25, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.ARMED] and not s.follow_me_active
    s, ev = tick(s, 10.375, HIGH, ch4=CH4_LOW)
    s, ev = tick(s, 10.5, HIGH, ch4=HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED]


def test_force_disarm_resets_the_ch3_runs():
    s, t = armed_state()
    s, _ = tick(s, t + 0.25, MID)
    s, ev = force_disarm(s, t + 0.375)
    assert s.ch3_high_since_mono_s is None
    assert s.ch3_held_high_mono_s is None
    assert s.rearm_requires_switch_cycle
    assert s.last_transition_mono_s == t + 0.375


# --- Controller: monotonic clock, journal warning, telemetry ------------------


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

        def step(t, ch3, ch4=CH4_LOW, epoch=None):
            """t: monotonic (also the RC frame stamp); epoch: wall clock (now_epoch_s).

            Since 2026-11-10 the RC reader stamps frames with time.monotonic(),
            so a wall-clock step moves only now_epoch_s, never the RC age.
            """
            clock["t"] = t
            e = t if epoch is None else epoch
            rc = RCInputs(
                ch1_us=1500, ch2_us=1500, ch3_us=ch3, ch4_us=ch4, ch5_us=CH5_LOW,
                last_update_mono_s=t,
            )
            return c.process(rc, now_epoch_s=e)

        yield c, step, motor, relay


def test_controller_logs_and_reports_the_midband_disarm(ctrl, caplog):
    c, step, motor, relay = ctrl
    cmd, ev, _ = step(100.0, HIGH)
    assert not cmd.is_armed
    cmd, ev, _ = step(100.25, HIGH)
    assert cmd.is_armed and SafetyEvent.ARMED in ev
    cmd, ev, telem = step(100.5, MID, ch4=MID)
    cmd, ev, telem = step(100.75, MID, ch4=MID)
    assert cmd.is_armed and "arm_midband_disarm" not in telem
    stops = motor.stops
    with caplog.at_level(logging.WARNING, logger="pi_app.control.controller"):
        cmd, ev, telem = step(100.8125, 1504, ch4=MID)
    assert not cmd.is_armed
    assert (cmd.left_byte, cmd.right_byte) == (c.NEUTRAL, c.NEUTRAL)
    assert SafetyEvent.ARM_MIDBAND_DISARM in ev and SafetyEvent.DISARMED in ev
    assert telem["arm_midband_disarm"] == {
        "ch3_us": 1504,
        "ch4_us": MID,
        "since_held_high_s": 0.562,
        "mode": "MANUAL",
    }
    assert telem["rearm_requires_switch_cycle"] is True
    assert relay.states[-1] is False
    assert motor.stops > stops
    lines = [r.getMessage() for r in caplog.records if "MID-BAND" in r.getMessage()]
    assert len(lines) == 1
    assert "ch3 has not held >= 1800 us for 0.56 s (now 1504 us)" in lines[0]
    assert "CH3/CH4 mix" in lines[0]
    # Next tick: still disarmed, no second warning or telemetry record.
    caplog.clear()
    cmd, ev, telem = step(101.0, MID, ch4=MID)
    assert not cmd.is_armed and ev == [] and "arm_midband_disarm" not in telem
    assert not [r for r in caplog.records if "MID-BAND" in r.getMessage()]


def test_controller_safety_timing_ignores_wall_clock_steps(ctrl):
    c, step, _motor, _relay = ctrl
    # One high frame, then the wall clock jumps an hour forward: no arm.
    cmd, ev, _ = step(50.0, HIGH, epoch=1000.0)
    cmd, ev, _ = step(50.0 + DT, HIGH, epoch=4600.0)
    cmd, ev, _ = step(50.0 + 2 * DT, MID, epoch=4600.0 + DT)
    assert not cmd.is_armed and SafetyEvent.ARMED not in ev
    # A held high arms on the monotonic hold, whatever the wall clock says.
    cmd, ev, _ = step(51.0, HIGH, epoch=4601.0)
    cmd, ev, _ = step(51.125, HIGH, epoch=9000.0)
    assert not cmd.is_armed
    cmd, ev, _ = step(51.25, HIGH, epoch=9000.125)
    assert cmd.is_armed and SafetyEvent.ARMED in ev
    # The wall clock steps back 10 min during a dip: the disarm is not postponed.
    cmd, ev, _ = step(51.5, MID, epoch=9000.375)
    cmd, ev, _ = step(51.78125, MID, epoch=8400.0)
    assert not cmd.is_armed and SafetyEvent.ARM_MIDBAND_DISARM in ev
