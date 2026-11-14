"""RC no signal: safety reaction (findings 1 and 5, docs/rc_arduino_gps_wiring.md).

Required behaviour (the Pi changes first, S12):
- no signal on CH3 or CH5: disarm at once, set the re-arm latch, never the
  e-stop (no shutdown -h);
- no signal on CH1 / CH2: that track neutral;
- no signal on CH4: FOLLOW_ME off;
- a no-signal value is never high, never mid-band, never low ("switch OFF");
- a journal line and a JSON event on entry and exit per channel,
  rate-limited.

Times are multiples of 1/32 s so the threshold comparisons are exact.
"""
import logging
from unittest.mock import patch

import pytest

from pi_app.app.vision_hang import restart_permitted
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.rc_signal import (
    RC_NO_SIGNAL_US,
    RcNoSignalMonitor,
    rc_signal_ok,
    rc_switch_high,
    rc_switch_low,
    rc_track_us,
)
from pi_app.control.safety import SafetyEvent, SafetyParams, SafetyState, update_safety

P = SafetyParams()
HIGH, LOW, MID = 2004, 1000, 1500
CH4_LOW, CH4_HIGH, CH5_LOW, CH5_HIGH = 992, 2004, 996, 2000
NS = RC_NO_SIGNAL_US
DT = 1.0 / 32
# The sentinel and every value the Pi must read as "no signal".
NO_SIGNAL_VALUES = [NS, 1, 499, 2501, 3000, 65535, -1]


def tick(state, t, ch3, ch4=CH4_LOW, ch5=CH5_LOW, params=P):
    return update_safety(
        state, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5, now_epoch_s=t, params=params, now_mono_s=t,
    )


def armed_state(t0=10.0):
    s, _ = tick(SafetyState(), t0, HIGH)
    s, ev = tick(s, t0 + 0.25, HIGH)
    assert ev == [SafetyEvent.ARMED] and s.is_armed
    return s, t0 + 0.25


def follow_me_state():
    s, t = armed_state()
    s, _ = tick(s, t + DT, HIGH, ch4=CH4_LOW)
    s, ev = tick(s, t + 2 * DT, HIGH, ch4=CH4_HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED] and s.follow_me_active
    return s, t + 2 * DT


# --- rc_signal helpers -----------------------------------------------------------


@pytest.mark.parametrize("v", NO_SIGNAL_VALUES + [None, True, "x"])
def test_no_signal_values_are_never_high_low_or_a_track_command(v):
    assert not rc_signal_ok(v)
    assert not rc_switch_low(v, 1200)
    assert not rc_switch_high(v, 1800)
    assert rc_track_us(v) == 1500


@pytest.mark.parametrize("v", [500, 992, 1200, 1500, 1800, 2004, 2500])
def test_widths_are_signal(v):
    assert rc_signal_ok(v)
    assert rc_track_us(v) == v
    assert rc_switch_low(v, 1200) == (v <= 1200)
    assert rc_switch_high(v, 1800) == (v >= 1800)


# --- CH3 no signal -----------------------------------------------------------------


@pytest.mark.parametrize("ns", NO_SIGNAL_VALUES)
def test_ch3_no_signal_disarms_at_once_and_latches(ns):
    s, t = armed_state()
    # Inside the 0.3 s debounce of the arm: a ch3-low disarm would wait.
    s, ev = tick(s, t + DT, ns)
    assert not s.is_armed
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.RC_NO_SIGNAL_DISARM]
    assert SafetyEvent.ARM_MIDBAND_DISARM not in ev  # not read as mid-band
    assert SafetyEvent.EMERGENCY_TRIGGERED not in ev
    assert s.rearm_requires_switch_cycle
    assert not s.emergency_active
    assert s.ch3_high_since_mono_s is None and s.ch3_held_high_mono_s is None


def test_ch3_no_signal_then_back_high_does_not_arm_until_cycled():
    s, t = armed_state()
    s, _ = tick(s, t + DT, NS)
    # The lead comes back with the switch still up: held high for 2 s.
    t2 = t + 1.0
    for i in range(64):
        s, ev = tick(s, t2 + i * DT, HIGH)
        assert not s.is_armed and SafetyEvent.ARMED not in ev
    assert s.rearm_requires_switch_cycle
    # Switch off (a measured low) clears the latch; switch up re-arms.
    t3 = t2 + 3.0
    s, _ = tick(s, t3, LOW)
    assert not s.rearm_requires_switch_cycle
    s, _ = tick(s, t3 + 0.5, HIGH)
    s, ev = tick(s, t3 + 0.75, HIGH)
    assert s.is_armed and ev == [SafetyEvent.ARMED]


def test_ch3_no_signal_while_disarmed_sets_the_latch():
    s, _ = tick(SafetyState(), 10.0, LOW)
    assert not s.rearm_requires_switch_cycle
    s, ev = tick(s, 10.0 + DT, NS)
    assert ev == [] and s.rearm_requires_switch_cycle
    s, _ = tick(s, 11.0, HIGH)
    s, ev = tick(s, 11.5, HIGH)
    assert not s.is_armed and ev == []


def test_no_signal_never_clears_the_latch():
    s = SafetyState(rearm_requires_switch_cycle=True)
    for i in range(32):
        s, ev = tick(s, 10.0 + i * DT, NS)
        assert s.rearm_requires_switch_cycle and not s.is_armed and ev == []


# --- CH5 no signal -----------------------------------------------------------------


@pytest.mark.parametrize("ns", NO_SIGNAL_VALUES)
def test_ch5_no_signal_disarms_and_latches_without_the_estop(ns):
    s, t = armed_state()
    for i in range(1, 33):
        s, ev = tick(s, t + i * DT, HIGH, ch5=ns)
        assert SafetyEvent.EMERGENCY_TRIGGERED not in ev
        if i == 1:
            assert ev == [SafetyEvent.DISARMED, SafetyEvent.RC_NO_SIGNAL_DISARM]
        else:
            assert ev == []
        assert not s.is_armed and not s.emergency_active
        assert s.rearm_requires_switch_cycle
        assert s.last_ch5_high is False


def test_ch5_back_low_after_no_signal_needs_the_arm_switch_cycled():
    s, t = armed_state()
    s, _ = tick(s, t + DT, HIGH, ch5=NS)
    s, _ = tick(s, t + 1.0, HIGH, ch5=CH5_LOW)
    s, ev = tick(s, t + 1.5, HIGH, ch5=CH5_LOW)
    assert not s.is_armed and not s.emergency_active and ev == []


def test_ch5_back_high_after_no_signal_is_the_estop():
    """A measured CH5 high after no signal is a rising edge: the operator's
    e-stop. Only a measured width can trigger it, never the sentinel."""
    s, t = armed_state()
    s, _ = tick(s, t + DT, HIGH, ch5=NS)
    s, ev = tick(s, t + 2 * DT, HIGH, ch5=CH5_HIGH)
    assert ev == [SafetyEvent.EMERGENCY_TRIGGERED] and s.emergency_active


# --- CH4 no signal -----------------------------------------------------------------


@pytest.mark.parametrize("ns", NO_SIGNAL_VALUES)
def test_ch4_no_signal_ends_follow_me_and_stays_armed(ns):
    s, t = follow_me_state()
    s, ev = tick(s, t + DT, HIGH, ch4=ns)
    assert ev == [SafetyEvent.FOLLOW_ME_EXITED]
    assert s.is_armed and not s.follow_me_active


def test_ch4_back_high_after_no_signal_does_not_restart_follow_me():
    s, t = follow_me_state()
    s, _ = tick(s, t + DT, HIGH, ch4=NS)
    for i in range(2, 34):
        s, ev = tick(s, t + i * DT, HIGH, ch4=CH4_HIGH)
        assert ev == [] and not s.follow_me_active
    # The operator puts CH4 low, then high again: Follow Me starts.
    s, _ = tick(s, t + 2.0, HIGH, ch4=CH4_LOW)
    s, ev = tick(s, t + 2.0 + DT, HIGH, ch4=CH4_HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED] and s.follow_me_active


def test_ch4_no_signal_never_starts_follow_me():
    s, t = armed_state()
    s, _ = tick(s, t + DT, HIGH, ch4=CH4_LOW)
    for i in range(2, 34):
        s, ev = tick(s, t + i * DT, HIGH, ch4=NS)
        assert ev == [] and not s.follow_me_active and s.is_armed


# --- Today's values: unchanged ----------------------------------------------------


def test_todays_values_still_follow_the_old_rules():
    s, t = armed_state()
    # A ch3-low disarm still waits for the debounce and does not latch.
    s, ev = tick(s, t + DT, LOW)
    assert s.is_armed and ev == []
    s, ev = tick(s, t + 0.375, LOW)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]
    assert not s.rearm_requires_switch_cycle
    # The e-stop still triggers on a measured high.
    s, t = armed_state(20.0)
    s, ev = tick(s, t + DT, HIGH, ch5=CH5_HIGH)
    assert ev == [SafetyEvent.DISARMED, SafetyEvent.EMERGENCY_TRIGGERED]


# --- vision-hang restart gate ---------------------------------------------------


def test_vision_hang_gate_does_not_read_no_signal_as_switch_off():
    assert restart_permitted(False, 0.1, NS, 1200) == (False, "ch3 no signal")
    assert restart_permitted(False, 0.1, 3000, 1200) == (False, "ch3 no signal")
    assert restart_permitted(False, 0.1, 1000, 1200) == (True, "disarmed, switch low")
    assert restart_permitted(False, 0.1, 2000, 1200) == (False, "switch up")
    assert restart_permitted(False, 0.1, None, 1200) == (False, "switch up")


# --- RcNoSignalMonitor ------------------------------------------------------------


def test_monitor_logs_the_first_loss_at_once_and_the_exit():
    m = RcNoSignalMonitor(min_interval_s=5.0)
    assert m.update(0.0, (1500, 1500, 2000, 1000, 1000, None)) == []
    ev = m.update(1.0, (1500, 1500, NS, 1000, 1000, None))
    assert [(e.channel, e.no_signal, e.transitions) for e in ev] == [(3, True, 1)]
    assert ev[0].as_log_dict()["event"] == "rc_no_signal"
    assert "RC CH3 NO SIGNAL" in ev[0].journal_line()
    assert "disarmed and latched" in ev[0].journal_line()
    assert m.no_signal_channels() == [3]
    # Back after 7 s: the interval has passed, so the exit is logged now.
    ev = m.update(8.0, (1500, 1500, 2000, 1000, 1000, None))
    assert [(e.channel, e.no_signal) for e in ev] == [(3, False)]
    assert ev[0].previous_state_s == pytest.approx(7.0)
    assert ev[0].as_log_dict()["event"] == "rc_signal_restored"
    assert "RC CH3 signal back after 7.0 s" in ev[0].journal_line()
    assert m.no_signal_channels() == []


def test_monitor_rate_limits_a_flapping_lead_and_ends_on_the_true_state():
    m = RcNoSignalMonitor(min_interval_s=5.0)
    m.update(0.0, (1500,) * 5 + (None,))
    logged = []
    t = 0.0
    for i in range(200):  # 25 Hz flapping for 8 s
        t = 0.02 + i * 0.04
        ch5 = NS if i % 2 == 0 else 1000
        logged += m.update(t, (1500, 1500, 2000, 1000, ch5, None))
    assert len(logged) == 2  # the first loss, then one merged line at 5 s
    assert all(e.channel == 5 for e in logged)
    assert logged[0].no_signal and logged[0].transitions == 1
    assert logged[1].transitions > 100
    assert "the lead may be loose" in logged[1].journal_line()
    # The flapping stops with the signal back: the log catches up, and its
    # last line for CH5 says the signal is back.
    logged += m.update(t + 5.0, (1500, 1500, 2000, 1000, 1000, None))
    logged += m.update(t + 10.0, (1500, 1500, 2000, 1000, 1000, None))
    assert logged[-1].no_signal is False
    assert m.no_signal_channels() == []


def test_monitor_skips_ch6_when_the_firmware_does_not_send_it():
    m = RcNoSignalMonitor()
    assert m.update(0.0, (1500,) * 5 + (None,)) == []
    ev = m.update(0.1, (1500,) * 5 + (NS,))
    assert [(e.channel, e.no_signal) for e in ev] == [(6, True)]


def test_monitor_logs_a_receiver_power_loss_once_per_channel():
    m = RcNoSignalMonitor()
    m.update(0.0, (1500, 1500, 2000, 1000, 1000, 1500))
    ev = m.update(0.02, (NS,) * 6)
    assert [e.channel for e in ev] == [1, 2, 3, 4, 5, 6]
    assert m.update(0.04, (NS,) * 6) == []


# --- Controller ---------------------------------------------------------------------


class _Motor:
    def __init__(self):
        self.stops = 0
        self.tracks = []
        self.contexts = []

    def set_tracks(self, left_byte, right_byte):
        self.tracks.append((left_byte, right_byte))

    def stop(self):
        self.stops += 1

    def set_drive_context(self, armed, manual, request_neutral=None):
        self.contexts.append(armed)


class _Relay:
    def __init__(self):
        self.states = []

    def set_armed(self, armed):
        self.states.append(armed)


class _Shutdown:
    def schedule_shutdown(self, delay_seconds):
        raise AssertionError("no signal must never schedule the e-stop shutdown")


@pytest.fixture
def ctrl():
    clock = {"t": 0.0}
    with patch("pi_app.control.controller.time.monotonic", side_effect=lambda: clock["t"]):
        motor, relay = _Motor(), _Relay()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=_Shutdown())

        def step(t, ch1=1500, ch2=1500, ch3=HIGH, ch4=CH4_LOW, ch5=CH5_LOW, ch6=None):
            clock["t"] = t
            rc = RCInputs(
                ch1_us=ch1, ch2_us=ch2, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5,
                last_update_mono_s=t, ch6_us=ch6,
            )
            return c.process(rc, now_epoch_s=t)

        yield c, step, motor, relay


def _arm(step, t0=100.0):
    step(t0)
    cmd, ev, _ = step(t0 + 0.25)
    assert cmd.is_armed and SafetyEvent.ARMED in ev
    return t0 + 0.25


@pytest.mark.parametrize("which", ["ch3", "ch5"])
def test_controller_no_signal_disarms_without_shutdown(ctrl, caplog, which):
    c, step, motor, relay = ctrl
    t = _arm(step)
    stops = motor.stops
    with caplog.at_level(logging.WARNING, logger="pi_app.control.controller"):
        cmd, ev, telem = step(t + DT, **{which: NS})
    assert not cmd.is_armed and not cmd.emergency_active
    assert (cmd.left_byte, cmd.right_byte) == (c.NEUTRAL, c.NEUTRAL)
    assert SafetyEvent.RC_NO_SIGNAL_DISARM in ev and SafetyEvent.DISARMED in ev
    assert SafetyEvent.EMERGENCY_TRIGGERED not in ev
    assert telem["rearm_requires_switch_cycle"] is True
    assert relay.states[-1] is False
    assert motor.stops > stops
    ch = int(which[2])
    assert telem["rc_no_signal_events"] == [{
        "event": "rc_no_signal",
        "channel": ch,
        "since_s": 0.0,
        "previous_state_s": pytest.approx(0.28125, abs=1e-3),
        "transitions": 1,
        "effect": telem["rc_no_signal_events"][0]["effect"],
    }]
    lines = [r.getMessage() for r in caplog.records if "NO SIGNAL" in r.getMessage()]
    assert len(lines) == 1 and f"RC CH{ch} NO SIGNAL" in lines[0]
    # No signal is not a deliberate disarm: the driver keeps "armed", so a
    # VESC pack-low latch is not cleared into limp home by a dead lead.
    assert motor.contexts[-1] is True
    # Many more ticks: still no shutdown, no second journal line.
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="pi_app.control.controller"):
        for i in range(2, 40):
            cmd, ev, telem = step(t + i * DT, **{which: NS})
            assert not cmd.is_armed and "rc_no_signal_events" not in telem
    assert not [r for r in caplog.records if "NO SIGNAL" in r.getMessage()]


def test_controller_ch3_low_still_clears_the_driver_arm_state(ctrl):
    c, step, motor, _ = ctrl
    t = _arm(step)
    step(t + 0.5, ch3=LOW)
    assert motor.contexts[-1] is False


def _drive(step, t0, ticks, **kw):
    out = []
    for i in range(ticks):
        cmd, _ev, _ = step(t0 + i * DT, **kw)
        assert cmd.is_armed
        out.append((cmd.left_byte, cmd.right_byte))
    return out


@pytest.mark.parametrize("dead, live", [("ch1", "ch2"), ("ch2", "ch1")])
def test_controller_no_signal_track_is_neutral(dead, live):
    """A dead track lead gives exactly what a centred stick gives."""
    results = []
    for dead_value in (NS, 1500):
        clock = {"t": 0.0}
        with patch("pi_app.control.controller.time.monotonic", side_effect=lambda: clock["t"]):
            c = Controller(motor_driver=_Motor(), arm_relay=_Relay(), shutdown_scheduler=_Shutdown())

            def step(t, **kw):
                clock["t"] = t
                vals = dict(ch1_us=1500, ch2_us=1500, ch3_us=HIGH, ch4_us=CH4_LOW, ch5_us=CH5_LOW)
                vals.update({f"{k}_us": v for k, v in kw.items()})
                return c.process(RCInputs(last_update_mono_s=t, **vals), now_epoch_s=t)

            t = _arm(step)
            out = _drive(step, t + DT, 40, **{dead: dead_value, live: 1800})
        results.append(out)
    assert results[0] == results[1]
    # And the live track did drive while the dead one stayed at neutral.
    idx_dead = 0 if dead == "ch1" else 1
    assert all(pair[idx_dead] == Controller.NEUTRAL for pair in results[0])
    assert any(pair[1 - idx_dead] != Controller.NEUTRAL for pair in results[0])


def test_controller_both_tracks_dead_is_full_neutral(ctrl):
    c, step, _, _ = ctrl
    t = _arm(step)
    out = _drive(step, t + DT, 20, ch1=NS, ch2=NS)
    assert all(pair == (c.NEUTRAL, c.NEUTRAL) for pair in out)


def test_controller_ch4_no_signal_leaves_follow_me_armed(ctrl):
    c, step, _, _ = ctrl
    t = _arm(step)
    # Put the controller in FOLLOW_ME through the safety state (no camera
    # in this test, so the real start would be refused for "no target").
    c._safety_state.set_follow_me_active(True)
    c._mode = "FOLLOW_ME"
    cmd, ev, telem = step(t + DT, ch4=NS)
    assert cmd.is_armed
    assert SafetyEvent.FOLLOW_ME_EXITED in ev
    assert telem["mode"] == "MANUAL"
    assert [e["channel"] for e in telem["rc_no_signal_events"]] == [4]


def test_controller_todays_values_have_no_no_signal_events(ctrl):
    c, step, _, _ = ctrl
    t = _arm(step)
    for i in range(40):
        _cmd, _ev, telem = step(t + i * DT, ch1=1600, ch2=1400)
        assert "rc_no_signal_events" not in telem
