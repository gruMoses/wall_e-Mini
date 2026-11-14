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

from pi_app.app.vision_hang import NoSignalRun, restart_permitted
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
    # Switch off (a measured low held 0.2 s) clears the latch; switch up
    # re-arms.
    t3 = t2 + 3.0
    s, _ = tick(s, t3, LOW)
    assert s.rearm_requires_switch_cycle  # one sample is not a switch-off
    s, _ = tick(s, t3 + 0.25, LOW)
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
    # The operator puts CH4 low (held 0.25 s), then high again: Follow Me
    # starts.
    s, _ = tick(s, t + 2.0, HIGH, ch4=CH4_LOW)
    s, _ = tick(s, t + 2.25, HIGH, ch4=CH4_LOW)
    s, ev = tick(s, t + 2.25 + DT, HIGH, ch4=CH4_HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED] and s.follow_me_active


@pytest.mark.parametrize("between", [MID, 1300, 1799])
def test_ch4_no_signal_then_mid_then_high_does_not_restart_follow_me(between):
    """Grok review 2026-11-13: high -> no signal -> a measured value that is
    not high (failsafe neutral, a corrupt field) -> high again was a rising
    edge, so Follow Me restarted with the switch up all the time. Only a
    measured ch4 low clears the latch."""
    s, t = follow_me_state()
    s, ev = tick(s, t + DT, HIGH, ch4=NS)
    assert ev == [SafetyEvent.FOLLOW_ME_EXITED] and s.ch4_requires_low
    s, ev = tick(s, t + 2 * DT, HIGH, ch4=between)
    assert ev == [] and s.ch4_requires_low
    s, ev = tick(s, t + 3 * DT, HIGH, ch4=CH4_HIGH)
    assert ev == [] and not s.follow_me_active and s.is_armed
    s, ev = tick(s, t + 4 * DT, HIGH, ch4=CH4_LOW)
    assert ev == [] and s.ch4_requires_low  # one low sample: not yet
    s, ev = tick(s, t + 4 * DT + 0.25, HIGH, ch4=CH4_LOW)
    assert ev == [] and not s.ch4_requires_low
    s, ev = tick(s, t + 5 * DT + 0.25, HIGH, ch4=CH4_HIGH)
    assert ev == [SafetyEvent.FOLLOW_ME_ENTERED]


def test_ch4_latch_survives_a_disarm_and_rearm():
    s, t = follow_me_state()
    s, _ = tick(s, t + DT, HIGH, ch4=NS)
    s, _ = tick(s, t + 0.5, LOW, ch4=MID)  # disarm
    assert not s.is_armed and s.ch4_requires_low
    s, _ = tick(s, t + 1.0, HIGH, ch4=MID)
    s, ev = tick(s, t + 1.25, HIGH, ch4=MID)
    assert s.is_armed and ev == [SafetyEvent.ARMED]
    s, ev = tick(s, t + 1.25 + DT, HIGH, ch4=CH4_HIGH)
    assert ev == [] and not s.follow_me_active


def test_ch4_no_signal_never_starts_follow_me():
    s, t = armed_state()
    s, _ = tick(s, t + DT, HIGH, ch4=CH4_LOW)
    for i in range(2, 34):
        s, ev = tick(s, t + i * DT, HIGH, ch4=NS)
        assert ev == [] and not s.follow_me_active and s.is_armed


# --- Held low clears a latch (Kevin, 2026-11-13) ----------------------------------
# The two cases of the review's sim_bounce.py, at its 30 Hz tick: a loose lead
# that comes back with one 500-1200 us bounce must not re-arm the robot or
# restart FOLLOW_ME while the switch stays up.

TICK30 = 1.0 / 30


def _sim(seq, state=None):
    st = SafetyState(rearm_requires_switch_cycle=True) if state is None else state
    t = 0.0
    log = []
    for ch3, ch4, ch5, n in seq:
        for _ in range(n):
            st, ev = update_safety(st, ch3, ch4, ch5, t, P, now_mono_s=t)
            log.extend(ev)
            t += TICK30
    return st, log


@pytest.mark.parametrize("bounce", [500, 900, 1200])
def test_ch3_bounce_after_no_signal_does_not_rearm(bounce):
    st, log = _sim([
        (1000, 1000, 1000, 15), (2000, 1000, 1000, 30), (2000, 2000, 1000, 10),
        (NS, 2000, 1000, 10), (bounce, 2000, 1000, 1), (2000, 2000, 1000, 60),
    ])
    assert log.count(SafetyEvent.ARMED) == 1  # the operator's first arm only
    assert log.count(SafetyEvent.FOLLOW_ME_ENTERED) == 1
    assert not st.is_armed and not st.follow_me_active
    assert st.rearm_requires_switch_cycle


@pytest.mark.parametrize("bounce", [500, 900, 1200])
def test_ch4_bounce_after_no_signal_does_not_restart_follow_me(bounce):
    st, log = _sim([
        (1000, 1000, 1000, 15), (2000, 1000, 1000, 30), (2000, 2000, 1000, 10),
        (2000, NS, 1000, 5), (2000, bounce, 1000, 1), (2000, 2000, 1000, 60),
    ])
    assert log.count(SafetyEvent.FOLLOW_ME_ENTERED) == 1
    assert st.is_armed and not st.follow_me_active and st.ch4_requires_low


def test_a_bouncing_low_run_never_clears_the_latch():
    """Low samples broken by no signal or mid-band never add up to a held
    low, however long the bouncing goes on."""
    seq = []
    for _ in range(40):
        seq += [(900, CH4_LOW, CH5_LOW, 4), (NS, CH4_LOW, CH5_LOW, 1),
                (900, CH4_LOW, CH5_LOW, 4), (MID, CH4_LOW, CH5_LOW, 1)]
    st, log = _sim(seq)
    assert st.rearm_requires_switch_cycle and log == []


def test_a_held_low_clears_the_latch_and_a_held_high_arms():
    # 7 ticks at 30 Hz is 0.2 s: the latch clears on the 7th low tick.
    st, _ = _sim([(LOW, CH4_LOW, CH5_LOW, 7)])
    assert st.rearm_requires_switch_cycle
    st, _ = _sim([(LOW, CH4_LOW, CH5_LOW, 8)])
    assert not st.rearm_requires_switch_cycle
    st, log = _sim([(LOW, CH4_LOW, CH5_LOW, 8), (HIGH, CH4_LOW, CH5_LOW, 20)])
    assert st.is_armed and log == [SafetyEvent.ARMED]


def test_the_ch3_low_disarm_is_still_one_sample():
    """Only clearing a latch needs the held low; disarming does not."""
    s, t = armed_state()
    s, ev = tick(s, t + 0.375, LOW)
    assert not s.is_armed and ev == [SafetyEvent.DISARMED]


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


def test_vision_hang_gate_permits_after_30_s_of_ch3_no_signal():
    """Review S2: a parked robot, transmitter off, a receiver that stops its
    outputs: ch3 reads no signal on every fresh line. Blocking forever would
    bring back the 2026-10-04 four-day hang."""
    run = NoSignalRun()
    gate = lambda ch3, ns_s: restart_permitted(False, 0.02, ch3, 1200, ch3_no_signal_s=ns_s)
    t = 100.0
    assert run.update(t, NS) == 0.0
    assert gate(NS, run.update(t + 29.9, NS)) == (False, "ch3 no signal")
    assert gate(NS, run.update(t + 30.0, NS)) == (True, "disarmed, ch3 no signal for 30 s")
    assert gate(NS, run.update(t + 400.0, NS)) == (True, "disarmed, ch3 no signal for 400 s")
    # Any measured width ends the run; the next no signal starts again.
    assert run.update(t + 401.0, 2000) is None
    assert gate(2000, None) == (False, "switch up")  # measured high still blocks
    assert run.update(t + 401.02, NS) == 0.0
    assert gate(NS, run.update(t + 420.0, NS)) == (False, "ch3 no signal")
    assert run.update(t + 421.0, 1600) is None  # mid-band also ends it
    assert run.update(t + 421.02, NS) == 0.0
    # Armed always blocks, whatever the run.
    assert restart_permitted(True, 0.02, NS, 1200, ch3_no_signal_s=999.0) == (False, "armed")
    # rc_off_s <= 0 never permits on no signal.
    assert restart_permitted(False, 0.02, NS, 1200, rc_off_s=0.0, ch3_no_signal_s=999.0) == (
        False, "ch3 no signal",
    )


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


def test_controller_ch3_held_low_clears_the_driver_arm_state(ctrl):
    """The VESC drive context (pack-low latch into limp home) and the
    person-stop hold release need the arm switch HELD off for 0.2 s."""
    c, step, motor, _ = ctrl
    t = _arm(step)
    step(t + 0.5, ch3=LOW)  # disarms (one sample) ...
    assert motor.contexts[-1] is True and not c.arm_switch_held_off  # ... not yet "off"
    step(t + 0.75, ch3=LOW)  # held 0.25 s
    assert motor.contexts[-1] is False and c.arm_switch_held_off


def test_controller_one_bounced_low_is_not_a_switch_off(ctrl):
    c, step, motor, _ = ctrl
    t = _arm(step)
    step(t + DT, ch3=NS)  # lead lost: disarm, latch
    step(t + 2 * DT, ch3=900)  # one bounce
    for i in range(3, 40):
        step(t + i * DT, ch3=HIGH)  # switch up the whole time
        assert not c.arm_switch_held_off
    assert motor.contexts[-1] is True


def _drive(step, t0, ticks, **kw):
    out = []
    for i in range(ticks):
        cmd, _ev, _ = step(t0 + i * DT, **kw)
        assert cmd.is_armed
        out.append((cmd.left_byte, cmd.right_byte))
    return out


@pytest.mark.parametrize("dead, live", [("ch1", "ch2"), ("ch2", "ch1")])
def test_controller_one_dead_track_makes_both_tracks_neutral(ctrl, dead, live):
    """Contract changed on review (2026-11-13): one dead track lead makes
    BOTH tracks neutral. With only the dead one neutral, a held stick on the
    other turned the robot in place."""
    c, step, _, _ = ctrl
    t = _arm(step)
    out = _drive(step, t + DT, 40, **{dead: NS, live: 1800})
    assert all(pair == (c.NEUTRAL, c.NEUTRAL) for pair in out)


def _ramp_limit_bytes():
    from config import config
    return float(config.slew_limiter.manual_accel_bps) * DT + 1.0


def test_track_signal_back_under_a_held_stick_ramps_not_snaps(ctrl):
    """Note 3 of the review: armed, sticks centred since the arm (the slew
    has seen no motion), a track lead dies, then comes back with both sticks
    held forward. The first command must be one ramp step from neutral, not
    the full stick (snap_first_command)."""
    c, step, _, _ = ctrl
    t = _arm(step)
    _drive(step, t + DT, 5)  # centred sticks
    _drive(step, t + 6 * DT, 5, ch1=NS, ch2=1500)  # lead dead
    out = _drive(step, t + 11 * DT, 30, ch1=1900, ch2=1900)  # back, held
    # The return tick pins the slew at neutral (dt 0, so it stays at 126);
    # from the next tick it ramps. Without the fix it snapped to the stick.
    first_left, first_right = out[0]
    assert 0 <= first_left - c.NEUTRAL <= _ramp_limit_bytes()
    assert 0 <= first_right - c.NEUTRAL <= _ramp_limit_bytes()
    # It keeps ramping up, one step per tick.
    lefts = [p[0] for p in out]
    assert all(0 <= b - a <= _ramp_limit_bytes() for a, b in zip(lefts, lefts[1:]))
    assert lefts[-1] > lefts[0]


def test_first_command_after_a_rearm_still_snaps(ctrl):
    """Kevin's ruling 2026-11-10: the first command after a re-arm stays as
    it is (snap_first_command). The track-return ramp does not change it."""
    c, step, _, _ = ctrl
    t = _arm(step)
    out = _drive(step, t + DT, 1, ch1=1900, ch2=1900)
    assert out[0][0] - c.NEUTRAL > _ramp_limit_bytes()


def test_track_signal_back_while_ramping_down_does_not_step_to_neutral(ctrl):
    c, step, _, _ = ctrl
    t = _arm(step)
    out = _drive(step, t + DT, 20, ch1=1900, ch2=1900)  # driving
    high = out[-1][0]
    out = _drive(step, t + 21 * DT, 2, ch1=NS, ch2=1900)  # 2 dead ticks: ramping down
    mid = out[-1][0]
    assert c.NEUTRAL < mid < high
    out = _drive(step, t + 23 * DT, 1, ch1=1900, ch2=1900)  # back
    assert out[0][0] >= mid  # ramps on from where it was, no step to 126


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


def test_controller_ch6_no_signal_has_a_json_event_but_no_journal_line(ctrl, caplog):
    """Review note 6: CH6 is not used, so no journal WARNING (it would fire
    on every boot without a CH6 lead). The JSON event stays."""
    c, step, _, _ = ctrl
    with caplog.at_level(logging.WARNING, logger="pi_app.control.controller"):
        _cmd, _ev, telem = step(100.0, ch6=NS)
    assert [e["channel"] for e in telem["rc_no_signal_events"]] == [6]
    assert not [r for r in caplog.records if "CH6" in r.getMessage()]


def test_controller_todays_values_have_no_no_signal_events(ctrl):
    c, step, _, _ = ctrl
    t = _arm(step)
    for i in range(40):
        _cmd, _ev, telem = step(t + i * DT, ch1=1600, ch2=1400)
        assert "rc_no_signal_events" not in telem
