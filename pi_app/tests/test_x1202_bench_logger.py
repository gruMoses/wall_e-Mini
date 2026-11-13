"""Tests for tools/x1202_bench_logger.py (no hardware: fake inputs, fake clock)."""
import os
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "tools"))

import x1202_bench_logger as xl  # noqa: E402


# ---------------------------------------------------------------- decode
def test_vcell_decode():
    assert xl.decode_vcell(0xD200) == pytest.approx(4.2000, abs=1e-9)


def test_soc_decode():
    assert round(xl.decode_soc(0x17F5), 2) == 23.96


def test_swap_word_matches_smbus_byte_order():
    # read_word_data returns the low byte first: register 0xD2 0x00 -> 0x00D2
    assert xl.swap_word(0x00D2) == 0xD200
    assert xl.decode_vcell(xl.swap_word(0x00D2)) == pytest.approx(4.2)


def test_parse_pinctrl():
    p = xl.parse_pinctrl_line("16: op dh pd | hi // GPIO16 = output")
    assert p == {"func": "op", "drive": "dh", "pull": "pd", "level": 1}
    p = xl.parse_pinctrl_line(" 6: ip    pn | lo // GPIO6 = input")
    assert p == {"func": "ip", "drive": None, "pull": "pn", "level": 0}
    assert xl.parse_pinctrl_line("garbage")["level"] is None


# ---------------------------------------------------------------- hold
def make_hold(now=0.0):
    return xl.HoldController(hold_v=3.80, hyst=0.05, rest_s=120, burst_s=120, now=now)


def test_hold_waits_for_rest_before_burst():
    h = make_hold()
    assert h.decide(119.9, True, 3.70) is True       # not rested yet
    assert h.decide(120.0, True, 3.70) is False      # burst starts


def test_hold_no_burst_inside_hysteresis():
    h = make_hold()
    assert h.decide(500, True, 3.76) is True         # 3.76 >= 3.75
    assert h.decide(501, True, 3.749) is False       # just below 3.75


def test_hold_burst_lasts_burst_s_then_rests_again():
    h = make_hold()
    assert h.decide(200, True, 3.70) is False
    assert h.decide(319.9, True, 4.10) is False      # still bursting even at high V
    assert h.decide(320, True, 4.10) is True         # burst over
    # must rest rest_s again before the next burst
    assert h.decide(439, True, 3.70) is True
    assert h.decide(440, True, 3.70) is False


def test_hold_input_loss_ends_burst_and_resets_rest():
    h = make_hold()
    assert h.decide(200, True, 3.70) is False
    assert h.decide(210, False, 3.60) is True        # input gone: disabled
    assert not h.bursting
    assert h.decide(300, True, 3.70) is True         # only 90 s rest since loss
    assert h.decide(330, True, 3.70) is False


def test_hold_unreadable_vcell_never_starts_burst():
    h = make_hold()
    assert h.decide(1000, True, None) is True


# ---------------------------------------------------------------- fail-to-disabled
def test_failsafe_after_ten_bad_reads_in_hold():
    h = make_hold()
    cm = xl.ChargeManager("hold", h)
    assert cm.desired(200, True, 3.70) is False      # burst running
    for i in range(9):
        assert cm.desired(201 + i, True, None) is True    # burst ended on the 1st bad read
    assert not h.bursting
    assert not cm.failsafe
    assert cm.desired(210, True, None) is True       # 10th: latched fail-safe
    assert cm.failsafe
    assert cm.desired(211, True, None) is True       # stays
    # gauge reads again: failsafe clears, but the burst does not resume
    assert cm.desired(212, True, 3.70) is True
    assert not cm.failsafe
    assert cm.desired(212 + 120, True, 3.70) is False   # new burst only after a fresh rest


def test_failsafe_counter_needs_consecutive_failures():
    cm = xl.ChargeManager("hold", make_hold())
    for i in range(9):
        cm.desired(i, True, None)
    cm.desired(9, True, 3.85)
    for i in range(9):
        cm.desired(10 + i, True, None)
    assert not cm.failsafe


def test_disable_mode_always_disabled_and_leave_never_touches():
    d = xl.ChargeManager("disable")
    assert d.desired(0, True, 3.5) is True
    lv = xl.ChargeManager("leave")
    for i in range(20):
        assert lv.desired(i, True, None) is None
    assert lv.failsafe                               # reported, not acted on


# ---------------------------------------------------------------- guard
class Flag:
    def __init__(self, v=True):
        self.v = v

    def __call__(self):
        return self.v


def run_low(g, t0, t1, vcell=3.6, dt=0.25):
    t = t0
    out = None
    while t <= t1 + 1e-9:
        r = g.update(t, False, vcell)
        if r:
            out = (t, r)
            break
        t += dt
    return out


def test_guard_fires_at_180s_of_low():
    g = xl.Guard(Flag(True), halt_s=180)
    hit = run_low(g, 0.0, 200.0)
    assert hit is not None
    assert hit[0] == pytest.approx(180.0)
    assert hit[1].startswith("input_low")


def test_guard_resets_after_three_high_samples():
    g = xl.Guard(Flag(True), halt_s=180)
    assert run_low(g, 0.0, 179.0) is None
    for t in (179.25, 179.5, 179.75):                # three high samples in a row
        assert g.update(t, True, 3.8) is None
    assert run_low(g, 180.0, 359.75) is None         # 179.75 s of low only
    assert g.update(360.0, False, 3.6) is not None   # 180 s after the restart


@pytest.mark.parametrize("highs", [1, 2])
def test_guard_one_or_two_high_samples_do_not_clear(highs):
    g = xl.Guard(Flag(True), halt_s=180)
    assert run_low(g, 0.0, 100.0) is None
    t = 100.25
    for _ in range(highs):
        assert g.update(t, True, 3.8) is None
        t += 0.25
    assert run_low(g, t, 179.75) is None
    assert g.update(180.0, False, 3.6) is not None   # timer kept running from 0.0


def test_guard_high_count_must_be_consecutive():
    g = xl.Guard(Flag(True), halt_s=180)
    g.update(0.0, False, 3.6)
    for t in (10.0, 10.25, 10.5, 10.75, 11.0, 11.25):
        g.update(t, (t * 4) % 3 != 0, 3.8)           # never 3 highs in a row
    assert g.update(180.0, False, 3.6) is not None


def test_guard_clear_after_is_configurable():
    g = xl.Guard(Flag(True), halt_s=180, clear_after=1)
    g.update(0.0, False, 3.6)
    g.update(100.0, True, 3.8)
    assert g.update(180.0, False, 3.6) is None       # cleared by one high sample


def test_guard_none_input_neither_clears_nor_fires():
    g = xl.Guard(Flag(True), halt_s=180)
    assert run_low(g, 0.0, 100.0) is None
    t = 100.25
    while t < 200.0:                                 # pin reads fail past halt_s
        assert g.update(t, None, 3.6) is None
        t += 0.25
    assert g.low_since == 0.0
    assert g.update(200.0, False, 3.6) is not None   # timer ran from 0.0


def test_guard_none_input_does_not_reset_high_count():
    g = xl.Guard(Flag(True), halt_s=180)
    g.update(0.0, False, 3.6)
    g.update(10.0, True, 3.8)
    g.update(10.25, None, 3.8)
    g.update(10.5, True, 3.8)
    g.update(10.75, True, 3.8)                       # third high: cleared
    assert g.low_since is None


def test_guard_none_input_keeps_fired_at():
    g = xl.Guard(Flag(True), halt_s=180, retry_s=30)
    hit = run_low(g, 0.0, 200.0)
    assert g.update(hit[0] + 1.0, None, 3.6) is None
    assert g.fired_at == hit[0]


def test_guard_never_fires_without_gauge():
    g = xl.Guard(Flag(True), halt_s=180)
    assert run_low(g, 0.0, 1000.0, vcell=None) is None


def test_guard_never_fires_without_flag_file():
    g = xl.Guard(Flag(False), halt_s=180)
    assert run_low(g, 0.0, 1000.0, vcell=3.0) is None


def test_guard_respects_inhibit():
    g = xl.Guard(Flag(True), halt_s=180, inhibited=lambda: True)
    assert run_low(g, 0.0, 1000.0, vcell=3.0) is None


def test_guard_low_vcell_rule_5s():
    g = xl.Guard(Flag(True), halt_s=180, vcell_v=3.30, vcell_s=5)
    assert g.update(0.0, False, 3.40) is None
    assert g.update(10.0, False, 3.30) is None       # at the limit: timer starts
    assert g.update(14.9, False, 3.29) is None
    r = g.update(15.0, False, 3.28)
    assert r and r.startswith("vcell")


def test_guard_low_vcell_timer_resets_on_recovery():
    g = xl.Guard(Flag(True), halt_s=180, vcell_v=3.30, vcell_s=5)
    g.update(0.0, False, 3.25)
    g.update(4.0, False, 3.35)                       # back above
    assert g.update(8.0, False, 3.25) is None
    assert g.update(12.9, False, 3.25) is None
    assert g.update(13.0, False, 3.25) is not None


def test_guard_low_vcell_does_not_fire_with_input_good():
    g = xl.Guard(Flag(True), halt_s=180, vcell_v=3.30, vcell_s=5)
    for i in range(100):
        assert g.update(i * 0.25, True, 3.0) is None


def test_guard_retries_only_after_retry_s():
    g = xl.Guard(Flag(True), halt_s=180, retry_s=30)
    hit = run_low(g, 0.0, 200.0)
    assert hit is not None
    assert run_low(g, hit[0] + 0.25, hit[0] + 29.9) is None
    assert g.update(hit[0] + 30.0, False, 3.6) is not None


# ---------------------------------------------------------------- inhibit file
def test_read_inhibit(tmp_path):
    p = tmp_path / "inh"
    assert xl.read_inhibit(str(p), "abc", 10.0) is False
    p.write_text("abc 100\n")
    assert xl.read_inhibit(str(p), "abc", 99.0) is True
    assert xl.read_inhibit(str(p), "abc", 100.0) is False
    assert xl.read_inhibit(str(p), "other-boot", 50.0) is False
    p.write_text("junk")
    assert xl.read_inhibit(str(p), "abc", 1.0) is False


# ---------------------------------------------------------------- gauge plausibility
class FakeBus:
    def __init__(self, vcell_word, soc_word=0x0050):
        self.words = {xl.REG_VCELL: vcell_word, xl.REG_SOC: soc_word}

    def read_word_data(self, addr, reg):
        return self.words[reg]

    def close(self):
        pass


def gauge_with(raw_be):
    hw = xl.Hardware()
    hw.bus = FakeBus(xl.swap_word(raw_be))
    return hw.read_gauge()


def test_gauge_good_reading_passes():
    v, s, err = gauge_with(0xD200)
    assert v == pytest.approx(4.2) and err is None


@pytest.mark.parametrize("raw", [0x0000, 0xFFFF, 0x7000, 0xE800])
def test_gauge_implausible_vcell_is_unreadable(raw):
    v, s, err = gauge_with(raw)
    assert v is None and s is None
    assert err.startswith("implausible_vcell") and ("0x%04X" % raw) in err


@pytest.mark.parametrize("raw", [0x0000, 0xFFFF])
def test_garbage_vcell_does_not_arm_guard_fast_path(raw):
    v, _, _ = gauge_with(raw)
    g = xl.Guard(Flag(True), halt_s=180, vcell_v=3.30, vcell_s=5)
    assert run_low(g, 0.0, 100.0, vcell=v) is None   # 0.0 V never counts as low cell
    assert g.vlow_since is None


def test_garbage_vcell_is_not_gauge_present_for_charge_manager():
    cm = xl.ChargeManager("hold", make_hold())
    v, _, _ = gauge_with(0xFFFF)
    for i in range(10):
        cm.desired(i, True, v)
    assert cm.failsafe


def test_garbage_vcell_never_starts_burst():
    h = make_hold()
    v, _, _ = gauge_with(0x0000)                     # 0.0 V would look far below hold_v
    assert h.decide(1000, True, v) is True
    assert not h.bursting


# ---------------------------------------------------------------- burst ends on bad read
def test_hold_burst_ends_on_first_unreadable_sample():
    h = make_hold()
    assert h.decide(200, True, 3.70) is False
    assert h.decide(201, True, None) is True
    assert not h.bursting
    assert h.decide(202, True, 3.70) is True          # must rest again
    assert h.decide(321, True, 3.70) is False


def test_charge_manager_burst_ends_on_first_unreadable_sample():
    h = make_hold()
    cm = xl.ChargeManager("hold", h)
    assert cm.desired(200, True, 3.70) is False
    assert cm.desired(201, True, None) is True
    assert not h.bursting


# ---------------------------------------------------------------- drive decision
def test_disabled_is_driven_when_pin_state_unknown():
    assert xl.should_drive_charge(True, None) is True


def test_enabled_is_not_driven_blind():
    assert xl.should_drive_charge(False, None) is False


def test_drive_decision_with_known_pin():
    dh = {"func": "op", "drive": "dh", "level": 1}
    dl = {"func": "op", "drive": "dl", "level": 0}
    ip = {"func": "ip", "drive": None, "level": 0}
    assert xl.should_drive_charge(True, dh) is False
    assert xl.should_drive_charge(True, dl) is True
    assert xl.should_drive_charge(True, ip) is True
    assert xl.should_drive_charge(False, dl) is False
    assert xl.should_drive_charge(False, dh) is True
    assert xl.should_drive_charge(None, None) is False
    assert xl.should_drive_charge(None, dl) is False


# ---------------------------------------------------------------- exit and errors
class FakeHw:
    def __init__(self, ok=True):
        self.calls = []
        self.ok = ok
        self.last_err = None

    def set_charge_disabled(self, disabled):
        self.calls.append(disabled)
        return self.ok


@pytest.mark.parametrize("mode", ["disable", "hold"])
def test_exit_disables_charging(mode):
    hw = FakeHw()
    ev = xl.exit_charge_safe(hw, mode)
    assert hw.calls == [True]
    assert ev["event"] == "charge_set_exit" and ev["ok"] is True


def test_exit_leave_never_touches_pin():
    hw = FakeHw()
    assert xl.exit_charge_safe(hw, "leave") is None
    assert hw.calls == []


@pytest.mark.parametrize("exc", [xl.subprocess.TimeoutExpired("pinctrl", 2), OSError("nope")])
def test_set_charge_disabled_never_raises(monkeypatch, exc):
    def boom(*a, **k):
        raise exc
    monkeypatch.setattr(xl.subprocess, "run", boom)
    hw = xl.Hardware()
    assert hw.set_charge_disabled(True) is False
    assert hw.last_err.startswith(type(exc).__name__)
    ev = xl.exit_charge_safe(hw, "hold")
    assert ev["ok"] is False and ev["err"]
