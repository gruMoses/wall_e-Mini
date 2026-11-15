"""Tests for bin/walle_power.py, the X1202 power watcher.

No hardware, no real subprocess, no files outside tmp_path: a fake OS layer
(FakeSys) and an explicit clock. The specification is
docs/x1202_power_watcher_design.md (sections 2, 3, 5, 6 and 7).
"""
import json
import os
import subprocess
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "bin"))

import walle_power as wp  # noqa: E402


# ---------------------------------------------------------------- fakes
class FakeSys:
    """Records every side effect, in order, in self.calls."""

    def __init__(self, tmp_path, marker=None):
        self.g6 = 1                       # level 1/0, or None = failed read
        self.g6_cfg = ("ip", "pd")
        self.g16 = ("op", "dh")           # None = failed read
        self.g16_stuck = False            # pin ignores writes
        self.vcell = 3.80                 # None = unreadable / implausible
        self.robot = ["inactive"]         # is-active answers; the last one repeats
        self.marker = marker
        self.eeprom = True
        self.calls, self.logs, self.notes, self.statuses = [], [], [], []
        self.events, self.event_err = [], None
        self.nohalt_path = str(tmp_path / "nohalt")

    def boot_id(self):
        return "boot-1"

    def wall(self):
        return 1.7e9

    def log(self, msg, level=6):
        self.logs.append((level, msg))

    def read_pins(self):
        self.calls.append(("read_pins",))
        g6 = None if self.g6 is None else {"func": self.g6_cfg[0], "drive": None,
                                            "pull": self.g6_cfg[1], "level": self.g6}
        g16 = None if self.g16 is None else {"func": self.g16[0], "drive": self.g16[1],
                                              "pull": "pd", "level": int(self.g16[1] == "dh")}
        return g6, g16

    def set_pin(self, pin, func, mode):
        self.calls.append(("set_pin", pin, func, mode))
        if pin == wp.GPIO_CHG and not self.g16_stuck and self.g16 is not None:
            self.g16 = (func, mode)
        return not (pin == wp.GPIO_CHG and self.g16_stuck)

    def read_vcell(self):
        self.calls.append(("read_vcell",))
        return (self.vcell, None) if self.vcell is not None else (None, "i2c OSError: x")

    def ext5v(self):
        self.calls.append(("ext5v",))
        return 5.1

    def eeprom_ok(self):
        self.calls.append(("eeprom",))
        return self.eeprom

    def kill_robot(self):
        self.calls.append(("kill",))
        self.calls.append(("stop_no_block",))

    def robot_state(self):
        self.calls.append(("is_active",))
        return self.robot.pop(0) if len(self.robot) > 1 else self.robot[0]

    def start_robot(self):
        self.calls.append(("start_robot",))

    def poweroff(self, args):
        self.calls.append(("poweroff",) + tuple(args))

    def read_marker(self):
        return self.marker

    def write_marker(self, d):
        self.calls.append(("marker", d["kind"], d["poweroff_at"]))
        self.marker = dict(d)

    def delete_marker(self):
        self.calls.append(("delete_marker",))
        self.marker = None

    def write_status(self, d):
        self.statuses.append(d)

    def notify(self, msg):
        self.notes.append(msg)
        self.calls.append(("notify", msg))

    def event(self, rec):
        self.events.append(rec)
        self.calls.append(("event", rec["event"]))
        return self.event_err

    def trim_events(self):
        return None

    # helpers
    def actions(self):
        keep = ("marker", "kill", "stop_no_block", "is_active", "delete_marker",
                "start_robot", "poweroff")
        return [c for c in self.calls if c[0] in keep]

    def poweroffs(self):
        return [c for c in self.calls if c[0] == "poweroff"]


def make(tmp_path, hold=False, t0=0.0, marker=None):
    fs = FakeSys(tmp_path, marker)
    return fs, wp.Watcher(fs, t0, hold=hold)


def run_ticks(w, t_from, t_to, step=wp.PERIOD_S, before=None):
    """Tick at t_from, t_from+step, ... <= t_to. Returns the next time."""
    i = 0
    while True:
        t = round(t_from + i * step, 6)
        if t > t_to + 1e-9:
            return t
        if before:
            before(t)
        w.tick(t)
        i += 1


def assert_ping_order(fs):
    """Per pass: a WATCHDOG=1 after the GPIO 6 path and before any I2C read,
    and the pass ends with a WATCHDOG=1 (W5, W6)."""
    passes, cur = [], None
    for c in fs.calls:
        if c[0] == "read_pins":
            cur = []
            passes.append(cur)
        elif cur is not None:
            cur.append(c)
    assert passes
    for p in passes:
        assert p[-1] == ("notify", "WATCHDOG=1")
        first_ping = p.index(("notify", "WATCHDOG=1"))
        assert all(c[0] not in ("read_vcell", "ext5v") for c in p[:first_ping])
        if ("kill",) in p:
            # only the (fsynced, local) event line sits between the kill and the ping
            between = p[p.index(("stop_no_block",)) + 1:first_ping]
            assert all(c[0] == "event" for c in between)


# ---------------------------------------------------------------- decode and parse
def test_vcell_decode_swaps_bytes():
    class Bus:
        def __init__(self, n):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read_word_data(self, addr, reg):
            assert (addr, reg) == (0x36, 0x02)
            return 0x00D2                  # register bytes D2 00 -> 0xD200 -> 4.200 V

    v, err = wp.read_vcell(Bus)
    assert err is None and v == pytest.approx(4.2)


@pytest.mark.parametrize("word", [0x0000, 0xFFFF])
def test_vcell_implausible_counts_as_unreadable(word):
    class Bus:
        def __init__(self, n):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read_word_data(self, addr, reg):
            return word

    v, err = wp.read_vcell(Bus)
    assert v is None and "implausible" in err


@pytest.mark.parametrize("exc", [ImportError("no smbus2"), FileNotFoundError("/dev/i2c-1")])
def test_vcell_without_smbus2_or_bus_is_unreadable(exc):
    def factory(n):
        raise exc

    v, err = wp.read_vcell(factory)
    assert v is None and type(exc).__name__ in err


def test_parse_pinctrl():
    assert wp.parse_pinctrl_line("16: op dh pd | hi // GPIO16 = output") == \
        {"func": "op", "drive": "dh", "pull": "pd", "level": 1}
    assert wp.parse_pinctrl_line(" 6: ip    pd | lo // GPIO6 = input") == \
        {"func": "ip", "drive": None, "pull": "pd", "level": 0}
    assert wp.parse_pinctrl_line("garbage")["level"] is None


# ---------------------------------------------------------------- subprocess bounds
def fake_popen(behaviour="ok", outs=None, rc=0):
    """A Popen stand-in. behaviour: ok | timeout (dies on kill) | stuck
    (survives the kill) | raise (cannot start)."""
    made = []

    class Pipe:
        closed = False

        def close(self):
            self.closed = True

    class P:
        def __init__(self, args, **kw):
            if behaviour == "raise":
                raise OSError("no such tool")
            self.args, self.kw, self.returncode, self.killed, self.waits = args, kw, None, False, []
            self.stdin, self.stdout, self.stderr = None, Pipe(), None
            self.polls = 0
            made.append(self)

        def poll(self):
            self.polls += 1
            return self.returncode

        def communicate(self, timeout=None):
            self.waits.append(timeout)
            if behaviour == "ok":
                self.returncode = rc
                return (outs or {}).get(self.args[0], ""), None
            if behaviour == "timeout" and self.killed:
                self.returncode = -9
                return "", None
            raise subprocess.TimeoutExpired(self.args, timeout)

        def kill(self):
            self.killed = True
    return P, made


def test_run_returns_output(monkeypatch):
    P, made = fake_popen(outs={"pinctrl": "6: ip pd | hi\n"})
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    assert wp.run(["pinctrl", "get", "6"]) == (0, "6: ip pd | hi\n")
    assert made[0].waits == [2.0]


def test_run_kills_and_reaps_on_timeout(monkeypatch):
    # W7 (replaces test_run_has_timeout_and_never_raises, which faked subprocess.run)
    P, made = fake_popen("timeout")
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    rc, err = wp.run(["vcgencmd", "x"])
    assert rc is None and "TimeoutExpired" in err and "abandoned" not in err
    assert made[0].killed and made[0].waits == [2.0, wp.KILL_WAIT_S]


def test_run_abandons_a_child_stuck_after_kill(monkeypatch):
    P, made = fake_popen("stuck")
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    rc, err = wp.run(["pinctrl", "get", "6"])
    assert rc is None and "abandoned" in err
    assert made[0].waits == [2.0, wp.KILL_WAIT_S]        # bounded twice, then given up


def test_run_cannot_start(monkeypatch):
    P, _ = fake_popen("raise")
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    rc, err = wp.run(["nope"])
    assert rc is None and "OSError" in err


@pytest.mark.parametrize("behaviour", ["stuck", "raise"])
def test_system_calls_survive_timeouts(monkeypatch, tmp_path, behaviour):
    # W7: changed to fake Popen (run() no longer uses subprocess.run)
    P, _ = fake_popen(behaviour)
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    s = wp.System(str(tmp_path / "run"), str(tmp_path / "state"))
    s.log = lambda *a, **k: None
    assert s.read_pins() == (None, None)
    assert s.set_pin(16, "op", "dh") is False
    assert s.ext5v() is None
    assert s.eeprom_ok() is None
    assert s.robot_state() is None         # "cannot tell": the halt keeps waiting
    s.kill_robot()
    s.start_robot()
    s.poweroff(["--force"])                # never raises


def test_system_parses_pins_and_eeprom(monkeypatch, tmp_path):
    outs = {"pinctrl": " 6: ip    pd | hi // GPIO6 = input\n16: op dh pd | hi // GPIO16 = output\n",
            "rpi-eeprom-config": "[all]\nBOOT_UART=1\nPOWER_OFF_ON_HALT=1\n",
            "vcgencmd": "EXT5V_V volt(24)=5.12345000V\n"}
    P, _ = fake_popen(outs=outs)
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    s = wp.System(str(tmp_path / "run"), str(tmp_path / "state"))
    g6, g16 = s.read_pins()
    assert g6["level"] == 1 and g6["pull"] == "pd" and g16["drive"] == "dh"
    assert s.eeprom_ok() is True
    assert s.ext5v() == pytest.approx(5.12345)
    outs["rpi-eeprom-config"] = "[all]\nPOWER_OFF_ON_HALT=0\n"
    assert s.eeprom_ok() is False


def test_log_never_raises(monkeypatch):
    # W10
    class Broken:
        def write(self, *a):
            raise BrokenPipeError("journal gone")

        def flush(self):
            raise ValueError("closed")
    monkeypatch.setattr(sys, "stdout", Broken())
    wp.System.log("x", 3)


def test_notify_abstract_socket(monkeypatch):
    sent = []

    class Sock:
        def __init__(self, *a):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def settimeout(self, t):
            pass

        def sendto(self, data, addr):
            sent.append((data, addr))

    monkeypatch.setattr(wp.socket, "socket", Sock)
    monkeypatch.setenv("NOTIFY_SOCKET", "@/org/freedesktop/systemd1/notify")
    wp.System.notify("WATCHDOG=1")
    assert sent == [(b"WATCHDOG=1", "\0/org/freedesktop/systemd1/notify")]
    monkeypatch.delenv("NOTIFY_SOCKET")
    wp.System.notify("READY=1")            # no socket: no error
    assert len(sent) == 1


# ---------------------------------------------------------------- debounce
def feed(d, t_from, t_to, good, step=0.1):
    n = int(round((t_to - t_from) / step))
    for i in range(n + 1):
        d.update(round(t_from + i * step, 6), good)


def test_debounce_4_9_s_does_not_trip_5_0_s_does():
    d = wp.Debounce()
    d.update(-0.1, True)
    feed(d, 0.0, 4.9, False)           # low for 4.9 s since the first low sample
    assert d.low_s == pytest.approx(4.9)
    assert not d.tripped
    d.update(5.0, False)
    assert d.tripped


def test_debounce_low_time_older_than_window_ages_out():
    d = wp.Debounce()
    d.update(0.0, True)
    feed(d, 0.1, 4.5, False)           # 4.5 s low
    feed(d, 4.6, 12.0, True)           # high long enough for the lows to leave 7.5 s
    feed(d, 12.1, 13.0, False)         # 0.9 s low
    assert d.low_s == pytest.approx(0.9)
    assert not d.tripped


def test_debounce_chattering_input_accumulates():
    d = wp.Debounce()
    d.update(0.0, True)
    tripped_at = None
    for i in range(1, 60):             # 3 low, 1 high, repeated (75 % low)
        t = i * 0.25
        d.update(t, (i % 4) == 0)
        if d.tripped and tripped_at is None:
            tripped_at = t
    assert tripped_at is not None and tripped_at <= 7.0
    # a single high sample does not reset the low time
    d3 = wp.Debounce()
    d3.update(0.0, True)
    feed(d3, 0.25, 3.0, False, step=0.25)
    before = d3.low_s
    d3.update(3.25, True)
    d3.update(3.5, False)
    assert d3.low_s >= before


def test_debounce_failed_read_neither_clears_nor_advances():
    # Changed for F2: the gap after a failed read is credited (capped) on the
    # next readable low sample, from the last READABLE sample.
    d = wp.Debounce()
    d.update(0.0, True)
    feed(d, 0.25, 4.0, False, step=0.25)
    assert d.low_s == pytest.approx(3.75)
    feed(d, 4.25, 5.0, None, step=0.25)       # failed reads change nothing
    assert d.low_s == pytest.approx(3.75)
    d.update(5.25, False)                     # 1.25 s since the last readable low
    assert d.low_s == pytest.approx(5.0)
    assert d.tripped


def test_debounce_caps_a_long_gap():
    d = wp.Debounce()
    d.update(0.0, False)
    d.update(4.0, False)                      # loop stall: at most 1 s credited
    assert d.low_s == pytest.approx(wp.DEBOUNCE_MAX_DT_S)


# ---------------------------------------------------------------- tiers (pure)
@pytest.mark.parametrize("good", [True, False, None])
def test_tier_b_fires_with_any_gpio6_state(good):
    lv = wp.LowVoltage()
    assert lv.update(0, good, 3.10) is None
    assert lv.update(1, good, 3.05) is None
    assert lv.update(2, good, 3.00).startswith("low_voltage_b")


def test_tier_b_needs_three_in_a_row():
    lv = wp.LowVoltage()
    lv.update(0, True, 3.0)
    lv.update(1, True, 3.0)
    assert lv.update(2, True, 3.2) is None
    assert lv.update(3, True, 3.0) is None
    assert lv.update(4, True, 3.0) is None


def test_tier_a_needs_gpio6_low():
    lv = wp.LowVoltage()
    for t in range(10):
        assert lv.update(t, True, 3.25) is None
    lv = wp.LowVoltage()
    for t in range(10):
        assert lv.update(t, None, 3.25) is None
    lv = wp.LowVoltage()
    for t in range(5):
        assert lv.update(t, False, 3.25) is None
    assert lv.update(5, False, 3.25).startswith("low_voltage_a")


def test_tier_a_resets_when_vcell_recovers_or_input_returns():
    lv = wp.LowVoltage()
    for t in range(4):
        lv.update(t, False, 3.30)
    lv.update(4, False, 3.31)
    for t in range(5, 10):
        assert lv.update(t, False, 3.30) is None
    lv.update(10, True, 3.30)
    assert lv.update(11, False, 3.30) is None


def test_implausible_vcell_arms_no_tier():
    lv = wp.LowVoltage()
    for t in range(20):
        assert lv.update(t, False, None) is None
    assert lv.b_count == 0 and lv.a_since is None


# ---------------------------------------------------------------- input-loss halt
def low_from(fs, t_low):
    return lambda t: setattr(fs, "g6", 0 if t >= t_low else 1)


def test_input_loss_halt_order_and_watchdog_in_every_wait(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active", "deactivating", "inactive"]
    run_ticks(w, 0.0, 30.0, before=low_from(fs, 1.0))
    acts = fs.actions()
    names = [a[0] for a in acts]
    assert names[:3] == ["marker", "kill", "stop_no_block"]
    assert acts[0] == ("marker", "input_loss", None)
    # is-active polled until inactive, then the re-check, then poweroff
    i_po = names.index("poweroff")
    assert names[3:i_po] == ["is_active"] * 3 + ["marker"]
    assert acts[i_po] == ("poweroff", "--no-wall")
    assert acts[i_po - 1][0] == "marker" and acts[i_po - 1][2] is not None
    assert "start_robot" not in names and "delete_marker" not in names
    # the halt decision at 6 s of real time: 5 s of low after the first low sample
    halt_t = next(s for s in fs.statuses if s["state"] == "halting")["hb_mono"]
    assert 5.9 <= halt_t <= 6.3
    # W5/W6: READY at the end of the constructor; pings in every pass (each wait
    # is a pass), one right after the kill step and before any I2C read.
    assert fs.notes[0] == "READY=1\nWATCHDOG=1" and sum("READY" in n for n in fs.notes) == 1
    assert_ping_order(fs)
    assert wp.PERIOD_S <= 0.5


def test_is_active_poll_every_half_second_and_15_s_bound(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active"]              # never stops
    times = []
    orig = fs.robot_state

    def rs():
        times.append(cur[0])
        return orig()
    fs.robot_state = rs
    cur = [0.0]

    def before(t):
        cur[0] = t
        fs.g6 = 0 if t >= 1.0 else 1
    run_ticks(w, 0.0, 30.0, before=before)
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert all(g >= 0.5 - 1e-9 and g <= 0.75 + 1e-9 for g in gaps)
    kill_t = times[0] - 0.25
    po = fs.poweroffs()
    assert po[0] == ("poweroff", "--no-wall")
    # the poweroff comes after the 15 s bound plus the two re-check samples
    last_poll = times[-1]
    assert last_poll - kill_t <= wp.STOP_WAIT_S


def test_cancel_restores_everything(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active", "active", "active", "inactive"]

    def before(t):
        fs.g6 = 0 if 1.0 <= t < 7.0 else 1   # input back during the stop wait
    run_ticks(w, 0.0, 20.0, before=before)
    names = [a[0] for a in fs.actions()]
    assert names[-2:] == ["delete_marker", "start_robot"]
    assert "poweroff" not in names
    assert fs.marker is None
    assert not w.halter.active and w.debounce.low_s == 0
    last = fs.statuses[-1]
    assert last["state"] == "run" and last["halt_reason"] is None
    # no second halt from the same low time
    assert names.count("kill") == 1


def test_recheck_needs_two_high_samples(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["inactive"]
    seq = {}

    def before(t):
        if t < 1.0:
            fs.g6 = 1
        elif not w.halter.active or w.halter.phase in ("kill", "wait_stop"):
            fs.g6 = 0
        else:                               # recheck: one high, then low
            seq[t] = True
            fs.g6 = 1 if len(seq) == 1 else 0
    run_ticks(w, 0.0, 12.0, before=before)
    assert ("poweroff", "--no-wall") in fs.poweroffs()
    assert ("start_robot",) not in fs.calls


def test_unreadable_gpio6_in_recheck_does_not_cancel(tmp_path):
    fs, w = make(tmp_path)

    def before(t):
        if t < 1.0:
            fs.g6 = 1
        elif w.halter.phase == "recheck":
            fs.g6 = None
        else:
            fs.g6 = 0
    run_ticks(w, 0.0, 12.0, before=before)
    assert ("poweroff", "--no-wall") in fs.poweroffs()


def test_nohalt_turns_off_input_loss_halt_only(tmp_path):
    (tmp_path / "nohalt").write_text("3\n")
    fs, w = make(tmp_path)
    assert w.nohalt
    run_ticks(w, 0.0, 30.0, before=low_from(fs, 0.0))
    assert fs.actions() == []
    assert "nohalt" in fs.statuses[-1]["faults"]
    # tier (a) stays armed (decision D3)
    fs.vcell = 3.25
    run_ticks(w, 30.25, 40.0, before=low_from(fs, 0.0))
    assert fs.poweroffs()[0] == ("poweroff", "--force")


# ---------------------------------------------------------------- low-voltage halt
@pytest.mark.parametrize("g6", [1, 0, None])
def test_tier_b_halt_with_any_gpio6_state_forced_no_cancel(tmp_path, g6):
    fs, w = make(tmp_path)
    fs.g6, fs.vcell = g6, 3.05
    run_ticks(w, 0.0, 2.0)
    acts = fs.actions()
    assert acts[0] == ("marker", "low_voltage", 2.0)
    assert acts[1] == ("poweroff", "--force")
    fs.g6, fs.vcell = 1, 3.9                     # input good again: no cancel
    run_ticks(w, 2.25, 100.0)
    names = [a[0] for a in fs.actions()]
    assert "start_robot" not in names and "delete_marker" not in names and "kill" not in names
    assert w.halter.active and w.halter.kind == "low_voltage"


def test_tier_a_halt_needs_gpio6_low(tmp_path):
    fs, w = make(tmp_path)
    w.nohalt = True                              # keep the input-loss halt out of the way
    fs.vcell, fs.g6 = 3.25, 1
    run_ticks(w, 0.0, 20.0)
    assert fs.poweroffs() == []
    fs.g6 = 0
    run_ticks(w, 20.25, 26.0)
    assert fs.poweroffs()[0] == ("poweroff", "--force")


def test_tier_during_input_loss_halt_jumps_to_force(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active"]                        # stop wait runs its 15 s

    def before(t):
        fs.g6 = 0 if t >= 1.0 else 1
        fs.vcell = 3.0 if t >= 8.0 else 3.8
    run_ticks(w, 0.0, 12.0, before=before)
    assert fs.poweroffs() == [("poweroff", "--force")]
    assert w.halter.kind == "low_voltage" and w.halter.phase == "ladder"
    fs.g6 = 1                                    # input back: still no cancel
    run_ticks(w, 12.25, 40.0)
    assert ("start_robot",) not in fs.calls


def test_tier_during_input_loss_ladder_jumps_to_force(tmp_path):
    fs, w = make(tmp_path)

    def before(t):
        fs.g6 = 0 if t >= 1.0 else 1
        fs.vcell = 3.0 if t >= 15.0 else 3.8
    run_ticks(w, 0.0, 20.0, before=before)
    po = fs.poweroffs()
    assert po[0] == ("poweroff", "--no-wall") and po[1] == ("poweroff", "--force")
    assert len(po) == 2


def test_charge_burst_does_not_mask_tier_b(tmp_path):
    fs, w = make(tmp_path, hold=True)
    fs.vcell = 3.70
    run_ticks(w, 0.0, 125.0)
    assert w.charge.state == "burst"
    fs.vcell = 3.08                              # falls during the burst
    run_ticks(w, 125.25, 130.0)
    assert ("poweroff", "--force") in fs.poweroffs()
    assert w.charge.state == "disabled"
    assert fs.g16 == ("op", "dh")


# ---------------------------------------------------------------- ladder
def test_ladder_timings_with_failing_systemctl(tmp_path):
    # Every systemctl call "fails": is-active cannot answer, and no poweroff
    # takes effect (the process keeps running). The ladder runs on time alone.
    fs, w = make(tmp_path)
    fs.robot = [None]                            # is-active fails: wait the full 15 s
    t_seen = []
    fs.poweroff = lambda args: (fs.calls.append(("poweroff",) + tuple(args)),
                                t_seen.append(cur[0]))
    cur = [0.0]

    def before(t):
        cur[0] = t
        fs.g6 = 0 if t >= 1.0 else 1
    run_ticks(w, 0.0, 200.0, before=before)
    t_po = w.halter.poweroff_at
    # trip at 6.0, the 15 s stop bound, then one low re-check sample
    assert t_po == pytest.approx(6.0 + wp.STOP_WAIT_S + wp.PERIOD_S)
    po = fs.poweroffs()
    assert po[:3] == [("poweroff", "--no-wall"), ("poweroff", "--force"),
                      ("poweroff", "--force", "--force")]
    assert all(p == ("poweroff", "--force", "--force") for p in po[3:])
    assert [t - t_po for t in t_seen[:5]] == pytest.approx([0, 30, 60, 90, 120])
    assert_ping_order(fs)


def test_ladder_rung_times(tmp_path):
    fs = FakeSys(tmp_path)
    h = wp.Halter(fs)
    h.start(100.0, "low_voltage", "x")
    assert fs.poweroffs() == [("poweroff", "--force")]
    assert h.step(129.9, True) is None
    assert h.step(130.0, True) == "force"
    assert h.step(159.9, True) is None
    assert h.step(160.0, True) == "force_force"
    assert h.step(190.0, True) == "force_force"      # retried each 30 s
    assert fs.poweroffs()[1:] == [("poweroff", "--force"), ("poweroff", "--force", "--force"),
                                  ("poweroff", "--force", "--force")]


def test_tiers_stay_armed_during_ladder(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 20.0, before=low_from(fs, 1.0))
    assert w.halter.phase == "ladder" and w.halter.kind == "input_loss"
    fs.vcell = 3.0
    run_ticks(w, 20.25, 24.0, before=low_from(fs, 1.0))
    assert w.halter.kind == "low_voltage"
    assert fs.poweroffs()[-1] == ("poweroff", "--force")


# ---------------------------------------------------------------- restart with marker
def test_restart_before_poweroff_continues_at_step_2(tmp_path):
    fs, w = make(tmp_path, t0=50.0,
                 marker={"kind": "input_loss", "reason": "r", "start": 40.0, "poweroff_at": None})
    assert w.halter.phase == "kill"
    fs.g6 = 0
    run_ticks(w, 50.0, 70.0)
    names = [a[0] for a in fs.actions()]
    assert names[:2] == ["kill", "stop_no_block"]
    assert ("poweroff", "--no-wall") in fs.poweroffs()


def test_restart_before_poweroff_can_still_cancel(tmp_path):
    fs, w = make(tmp_path, t0=50.0,
                 marker={"kind": "input_loss", "reason": "r", "start": 40.0, "poweroff_at": None})
    fs.g6 = 1
    run_ticks(w, 50.0, 55.0)
    assert fs.actions()[-2:] == [("delete_marker",), ("start_robot",)]
    assert fs.poweroffs() == []


@pytest.mark.parametrize("kind", ["input_loss", "low_voltage"])
def test_restart_in_ladder_continues_from_poweroff_at(tmp_path, kind):
    fs, w = make(tmp_path, t0=145.0,
                 marker={"kind": kind, "reason": "r", "start": 99.0, "poweroff_at": 100.0})
    assert w.halter.phase == "ladder"
    fs.g6 = 1
    run_ticks(w, 145.0, 145.0)
    assert fs.poweroffs() == [("poweroff", "--force")]      # the 30 s rung, at once
    run_ticks(w, 145.25, 160.0)
    assert fs.poweroffs()[-1] == ("poweroff", "--force", "--force")
    assert ("start_robot",) not in fs.calls and ("kill",) not in fs.calls


def test_restart_in_ladder_early_waits(tmp_path):
    fs, w = make(tmp_path, t0=110.0,
                 marker={"kind": "input_loss", "reason": "r", "start": 99.0, "poweroff_at": 100.0})
    run_ticks(w, 110.0, 129.75)
    assert fs.poweroffs() == []
    run_ticks(w, 130.0, 130.0)
    assert fs.poweroffs() == [("poweroff", "--force")]


def test_restart_low_voltage_marker_without_poweroff_at_forces(tmp_path):
    fs, w = make(tmp_path, t0=5.0,
                 marker={"kind": "low_voltage", "reason": "r", "start": 4.0, "poweroff_at": None})
    assert fs.poweroffs() == [("poweroff", "--force")]


def test_restart_with_unparsable_marker_continues_input_loss(tmp_path):
    fs, w = make(tmp_path, t0=5.0, marker={})
    assert w.halter.phase == "kill" and w.halter.kind == "input_loss"


def test_system_marker_round_trip(tmp_path):
    s = wp.System(str(tmp_path / "run"), str(tmp_path / "state"))
    assert s.read_marker() is None
    s.write_marker({"kind": "input_loss", "poweroff_at": None})
    assert s.read_marker()["kind"] == "input_loss"
    (tmp_path / "run" / "halting").write_text("{bad")
    assert s.read_marker() == {}
    s.delete_marker()
    s.delete_marker()
    assert s.read_marker() is None


# ---------------------------------------------------------------- nohalt
@pytest.mark.parametrize("content,active,left", [
    ("3\n", True, "2"), ("2", True, "1"), ("1", True, None), ("", True, "2"),
    ("garbage", True, None), ("0", False, None), ("-4", False, None)])
def test_nohalt_count(tmp_path, content, active, left):
    p = tmp_path / "nohalt"
    p.write_text(content)
    on, _ = wp.consume_nohalt(str(p))
    assert on is active
    if left is None:
        assert not p.exists()
    else:
        assert p.read_text().strip() == left


def test_nohalt_absent_and_expiry_over_starts(tmp_path):
    p = tmp_path / "nohalt"
    assert wp.consume_nohalt(str(p)) == (False, None)
    p.write_text("3")
    assert [wp.consume_nohalt(str(p))[0] for _ in range(4)] == [True, True, True, False]


# ---------------------------------------------------------------- charge
def test_hold_off_always_disabled(tmp_path):
    fs, w = make(tmp_path, hold=False)
    fs.vcell = 3.50
    run_ticks(w, 0.0, 400.0)
    assert ("set_pin", 16, "op", "dl") not in fs.calls
    assert all(s["charge"] == "disabled" and s["hold"] == "off" for s in fs.statuses)


def test_charge_failsafe_after_three_bad_reads():
    c = wp.Charge(hold=False, now=0.0)
    c.update(0.0, True, True, 3.8)
    for t in (1.0, 2.0):
        assert c.update(t, True, True, None) == "disabled"
    assert c.update(3.0, True, True, None) == "failsafe"
    assert c.update(4.0, True, True, 3.8) == "disabled"


def test_gauge_fault_after_60_s():
    c = wp.Charge(hold=False, now=0.0)
    c.update(0.0, True, True, 3.8)
    for t in range(1, 61):
        c.update(float(t), True, True, None)
    assert not c.gauge_fault(60.9)
    assert c.gauge_fault(61.0)
    c.update(62.0, True, True, 3.8)
    assert not c.gauge_fault(62.0)


def test_hold_burst_start_rules():
    c = wp.Charge(hold=True, now=0.0)
    assert c.update(119.0, True, True, 3.70) == "disabled"     # not rested
    assert c.update(120.0, True, True, 3.75) == "disabled"     # not below 3.75
    assert c.update(121.0, False, True, 3.70) == "disabled"    # input lost: rest restarts
    assert c.update(240.0, True, True, 3.70) == "disabled"
    assert c.update(241.0, True, False) == "disabled"          # needs a gauge sample
    assert c.update(241.0, True, True, 3.70) == "burst"


def test_hold_burst_lasts_120_s_then_rests():
    c = wp.Charge(hold=True, now=0.0)
    assert c.update(120.0, True, True, 3.70) == "burst"
    assert c.update(239.0, True, True, 3.90) == "burst"
    assert c.update(240.0, True, True, 3.90) == "disabled"
    assert c.update(359.0, True, True, 3.70) == "disabled"
    assert c.update(360.0, True, True, 3.70) == "burst"


@pytest.mark.parametrize("end", ["input", "input_unknown", "gauge", "vcell_fall", "halting"])
def test_hold_burst_ends_at_once(end):
    c = wp.Charge(hold=True, now=0.0)
    assert c.update(120.0, True, True, 3.700) == "burst"
    args = {"input": (130.0, False, False, None), "input_unknown": (130.0, None, False, None),
            "gauge": (130.0, True, True, None), "vcell_fall": (130.0, True, True, 3.699),
            "halting": (130.0, True, False, None)}[end]
    assert c.update(*args, halting=(end == "halting")) == "disabled"
    # a burst does not resume when the cause goes away
    assert c.update(131.0, True, True, 3.70) == "disabled"


def test_hold_burst_drives_pin_low_then_high(tmp_path):
    fs, w = make(tmp_path, hold=True)
    fs.vcell = 3.70
    run_ticks(w, 0.0, 121.0)
    assert fs.g16 == ("op", "dl") and fs.statuses[-1]["charge"] == "burst"
    fs.g6 = 0                                    # input loss ends it at once (4 Hz)
    run_ticks(w, 121.25, 121.25)
    assert fs.g16 == ("op", "dh")


def test_gauge_failsafe_in_watcher_and_fault(tmp_path):
    fs, w = make(tmp_path, hold=True)
    fs.vcell = None
    run_ticks(w, 0.0, 3.0)
    assert fs.statuses[-1]["charge"] == "failsafe"
    run_ticks(w, 3.25, 61.0)
    assert "gauge" in fs.statuses[-1]["faults"]
    assert fs.g16 == ("op", "dh")


def test_no_gauge_input_loss_halt_still_works(tmp_path):
    fs, w = make(tmp_path)
    fs.vcell = None                              # no smbus2 / no /dev/i2c-1
    run_ticks(w, 0.0, 30.0, before=low_from(fs, 1.0))
    assert ("poweroff", "--no-wall") in fs.poweroffs()


# ---------------------------------------------------------------- pins and faults
def test_start_sets_both_pins(tmp_path):
    fs, w = make(tmp_path)
    assert fs.calls[:2] == [("set_pin", 16, "op", "dh"), ("set_pin", 6, "ip", "pd")]


def test_gpio6_cfg_fault_on_wrong_pull_or_failed_read(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 1.0)
    assert "gpio6_cfg" not in fs.statuses[-1]["faults"]
    fs.g6_cfg = ("ip", "pn")
    run_ticks(w, 1.25, 2.0)
    assert "gpio6_cfg" in fs.statuses[-1]["faults"]
    fs.g6_cfg, fs.g6 = ("ip", "pd"), None
    run_ticks(w, 2.25, 3.0)
    assert "gpio6_cfg" in fs.statuses[-1]["faults"]
    assert fs.statuses[-1]["input_good"] is None


def test_charge_pin_fault_and_retry(tmp_path):
    fs, w = make(tmp_path)
    fs.g16, fs.g16_stuck = ("ip", None), True    # the pin stays an input
    run_ticks(w, 0.0, 2.0)
    sets = [c for c in fs.calls if c[:2] == ("set_pin", 16)]
    assert len(sets) >= 8                        # retried on every pass
    assert "charge_pin" in fs.statuses[-1]["faults"]
    fs.g16_stuck = False
    run_ticks(w, 2.25, 3.0)
    assert "charge_pin" not in fs.statuses[-1]["faults"]


def test_unknown_gpio16_driven_high_but_never_low_blind(tmp_path):
    fs, w = make(tmp_path, hold=True)
    fs.vcell = 3.70
    run_ticks(w, 0.0, 121.0)
    assert w.charge.state == "burst"
    fs.calls.clear()
    fs.g16 = None
    run_ticks(w, 121.25, 122.0)
    assert ("set_pin", 16, "op", "dl") not in fs.calls


def test_eeprom_fault_and_retry(tmp_path):
    # Changed (W6 + Opus LOW 15): READY is sent by the constructor, so the
    # check runs on the first pass; a non-yes answer is retried once a minute.
    fs, w = make(tmp_path)
    assert fs.notes == ["READY=1\nWATCHDOG=1"] and ("eeprom",) not in fs.calls
    fs.eeprom = None                             # timeout or no tool
    run_ticks(w, 0.0, 59.75)
    assert fs.calls.count(("eeprom",)) == 1
    assert "eeprom" in fs.statuses[-1]["faults"]
    fs.eeprom = True
    run_ticks(w, 60.0, 200.0)
    assert fs.calls.count(("eeprom",)) == 2      # yes at 60 s, then no more checks
    assert "eeprom" not in fs.statuses[-1]["faults"]


def test_eeprom_not_checked_while_halting(tmp_path):
    fs, w = make(tmp_path)
    fs.vcell, fs.eeprom = 3.0, None
    w.nohalt = True
    run_ticks(w, 0.0, 2.0)
    assert w.halter.active and ("eeprom",) in fs.calls   # checked on the first pass
    n = fs.calls.count(("eeprom",))
    run_ticks(w, 2.25, 200.0)
    assert fs.calls.count(("eeprom",)) == n


def test_cells_high_not_from_the_first_samples(tmp_path):
    # Replaces test_cells_high_on_first_reading (B2): the board charges until
    # GPIO 16 is driven, so the first samples after a start read high.
    fs, w = make(tmp_path)
    fs.vcell = 4.2
    run_ticks(w, 0.0, 119.75)
    assert all("cells_high" not in s["faults"] for s in fs.statuses)


def test_cells_high_follows_rested_samples(tmp_path):
    fs, w = make(tmp_path)
    fs.vcell = 4.2
    run_ticks(w, 0.0, 121.0)
    assert "cells_high" in fs.statuses[-1]["faults"]
    assert sum("notice cells_high on" in m for _, m in fs.logs) == 1
    fs.vcell = 3.95
    run_ticks(w, 121.25, 123.0)
    assert "cells_high" not in fs.statuses[-1]["faults"]
    assert sum("notice cells_high off" in m for _, m in fs.logs) == 1


def test_cells_high_ignores_input_loss_and_halts_and_bursts(tmp_path):
    fs, w = make(tmp_path)
    w.nohalt = True
    fs.vcell, fs.g6 = 4.2, 0
    run_ticks(w, 0.0, 200.0)
    assert "cells_high" not in fs.statuses[-1]["faults"]
    fs2, w2 = make(tmp_path, hold=True)
    fs2.vcell = 3.70
    run_ticks(w2, 0.0, 121.0)                    # a burst runs from 120 s
    assert w2.charge.state == "burst"
    fs2.vcell = 4.0
    run_ticks(w2, 121.25, 300.0)                 # ends at 240 s; rested from 360 s
    assert "cells_high" not in fs2.statuses[-1]["faults"]
    run_ticks(w2, 300.25, 361.0)
    assert "cells_high" in fs2.statuses[-1]["faults"]


def test_cells_high_alone_gives_no_periodic_warning(tmp_path):
    fs, w = make(tmp_path)
    fs.vcell = 4.2
    run_ticks(w, 0.0, 400.0)
    assert "cells_high" in fs.statuses[-1]["faults"]
    assert not any(m.startswith("FAULT") for _, m in fs.logs)
    fs.g6_cfg = ("ip", "pn")                     # a real fault: named, cells_high is not
    run_ticks(w, 400.25, 401.0)
    warns = [m for _, m in fs.logs if m.startswith("FAULT")]
    assert warns and "gpio6_cfg" in warns[0] and "cells_high" not in warns[0]


def test_fault_warning_once_a_minute(tmp_path):
    fs, w = make(tmp_path)
    fs.g6_cfg = ("ip", "pn")
    run_ticks(w, 0.0, 130.0)
    warns = [m for lvl, m in fs.logs if m.startswith("FAULT")]
    assert len(warns) == 3                       # t = 0, 60, 120


def test_state_changes_are_logged(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 2.0, before=low_from(fs, 1.0))
    msgs = [m for _, m in fs.logs]
    assert any(m.startswith("good: True -> False") for m in msgs)
    assert any(m.startswith("state: run -> debounce") for m in msgs)


# ---------------------------------------------------------------- status file
STATUS_KEYS = {"schema", "boot_id", "hb_mono", "bus_ok_mono", "bus_ok_ts", "input_good",
               "low_s", "vcell_v", "ext5v_v", "charge", "hold", "state", "halt_reason",
               "faults", "last_error"}


def test_status_schema_and_rate(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 9.75)
    assert len(fs.statuses) == 10                # 1 Hz with no halt
    s = fs.statuses[-1]
    assert set(s) == STATUS_KEYS
    assert s["schema"] == "walle-power/1" and s["boot_id"] == "boot-1"
    assert s["state"] == "run" and s["charge"] == "disabled" and s["hold"] == "off"
    assert s["faults"] == [] and s["halt_reason"] is None and s["input_good"] is True


def test_status_written_at_each_halt_step(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active", "inactive"]
    run_ticks(w, 0.0, 12.0, before=low_from(fs, 1.0))
    halting = [s for s in fs.statuses if s["state"] == "halting"]
    assert halting[0]["halt_reason"].startswith("input_loss")
    # kill, stopped and poweroff each write a status, also off the 1 Hz grid
    steps = [m for _, m in fs.logs if m.startswith("halt step")]
    assert [m.split(":")[0] for m in steps] == ["halt step kill", "halt step stopped",
                                                "halt step poweroff"]
    # kill at 6.0, stopped at 6.75 (off the 1 Hz grid), poweroff at 7.0
    assert {6.0, 6.75, 7.0} <= {s["hb_mono"] for s in halting}


def test_bus_ok_mono_frozen_on_failed_read(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 2.0)
    ok = fs.statuses[-1]["bus_ok_mono"]
    assert ok == 2.0
    fs.vcell = None
    run_ticks(w, 2.25, 5.0)
    s = fs.statuses[-1]
    assert s["bus_ok_mono"] == ok and s["hb_mono"] == 5.0 and s["vcell_v"] is None
    assert s["last_error"].startswith("i2c")


def test_real_status_file_is_atomic(tmp_path):
    s = wp.System(str(tmp_path / "run"), str(tmp_path / "state"))
    assert s.write_status({"a": 1}) is None
    assert json.loads((tmp_path / "run" / "status.json").read_text()) == {"a": 1}
    assert not (tmp_path / "run" / "status.json.tmp").exists()


# ---------------------------------------------------------------- packaging
def _read(name):
    with open(os.path.join(_REPO_ROOT, "bin", name)) as f:
        return f.read()


def test_unit_file_matches_design():
    u = _read("walle-power.service")
    for line in ("Type=notify", "WatchdogSec=15", "TimeoutStartSec=10", "Restart=always",
                 "RestartSec=15", "StartLimitIntervalSec=0", "RuntimeDirectory=walle-power",
                 "RuntimeDirectoryPreserve=yes", "StateDirectory=walle-power",
                 "ExecStartPre=-/usr/bin/pinctrl set 16 op dh",
                 "ExecStartPre=-/usr/bin/pinctrl set 6 ip pd",
                 # --hold: Kevin turned the charge hold on (2026-11-15, decision 6).
                 "ExecStart=/usr/bin/python3 /usr/local/lib/walle-power/walle_power.py --hold",
                 "ExecStopPost=-/usr/bin/pinctrl set 16 op dh"):
        assert line in u.splitlines()


def test_poweroff_dropin():
    d = _read("walle-poweroff-timeout.conf").splitlines()
    assert "[Unit]" in d and "JobTimeoutSec=60s" in d and "JobTimeoutAction=poweroff-force" in d


def test_hold_default_off():
    # The code default stays off: a start without --hold never charges.
    assert wp.HOLD_ENABLED is False


def test_main_passes_hold_option(monkeypatch):
    seen = []
    monkeypatch.setattr(wp, "System", lambda: object())
    monkeypatch.setattr(wp, "Watcher", lambda ops, now, hold: seen.append(hold))
    monkeypatch.setattr(wp, "loop", lambda *a, **k: None)
    monkeypatch.setattr(wp, "boottime", lambda: 0.0)
    monkeypatch.setattr(wp.signal, "signal", lambda *a: None)
    wp.main([])
    wp.main(["--hold"])
    assert seen == [False, True]


# ---------------------------------------------------------------- review round 1
def test_w1_recheck_ignores_unreadable_samples_between_highs(tmp_path):
    fs, w = make(tmp_path)
    seq = iter([1, None, None, 1])

    def before(t):
        if t < 1.0:
            fs.g6 = 1
        elif w.halter.phase == "recheck":
            fs.g6 = next(seq, 1)
        else:
            fs.g6 = 0
    run_ticks(w, 0.0, 12.0, before=before)
    assert fs.actions()[-2:] == [("delete_marker",), ("start_robot",)]
    assert fs.poweroffs() == []


def test_w1_recheck_first_readable_low_powers_off(tmp_path):
    fs, w = make(tmp_path)
    seq = iter([None, 1, None, 0])
    seen = []

    def before(t):
        if t < 1.0:
            fs.g6 = 1
        elif w.halter.phase == "recheck":
            fs.g6 = next(seq, 0)
            seen.append(t)
        else:
            fs.g6 = 0
    run_ticks(w, 0.0, 12.0, before=before)
    assert fs.poweroffs()[0] == ("poweroff", "--no-wall")
    assert w.halter.poweroff_at == seen[3]             # the low sample, not a None


def test_w1_recheck_unreadable_for_2_s_powers_off(tmp_path):
    fs, w = make(tmp_path)
    entered = []

    def before(t):
        if t < 1.0:
            fs.g6 = 1
        elif w.halter.phase == "recheck":
            entered.append(t)
            fs.g6 = None
        else:
            fs.g6 = 0
    run_ticks(w, 0.0, 20.0, before=before)
    stopped_t = entered[0] - wp.PERIOD_S                # the "stopped" tick
    assert w.halter.poweroff_at == pytest.approx(stopped_t + wp.RECHECK_UNREADABLE_S)
    assert fs.poweroffs()[0] == ("poweroff", "--no-wall")


def test_w2_no_halt_start_on_a_high_sample(tmp_path):
    fs, w = make(tmp_path)

    def before(t):
        fs.g6 = 0 if 1.0 <= t <= 5.75 else 1    # the 5.0 s is reached on the 6.0 high sample
    run_ticks(w, 0.0, 20.0, before=before)
    assert fs.actions() == []
    # proof that the debounce did reach 5 s on that high sample
    d = wp.Debounce()
    d.update(0.75, True)
    for i in range(0, 21):
        t = 1.0 + i * 0.25
        d.update(t, t > 5.75)
    assert t == 6.0 and d.tripped


def test_w2_halt_starts_when_low_returns_inside_the_window(tmp_path):
    fs, w = make(tmp_path)

    def before(t):
        fs.g6 = 0 if (1.0 <= t <= 5.75 or t >= 6.25) else 1
    run_ticks(w, 0.0, 7.0, before=before)
    assert fs.actions()[0] == ("marker", "input_loss", None)


@pytest.mark.parametrize("bad", [
    {"kind": "input_loss", "start": float("inf"), "poweroff_at": None},
    {"kind": "low_voltage", "start": 1.0, "poweroff_at": float("nan")},
    {"kind": "low_voltage", "start": 1.0, "poweroff_at": True},
    {"kind": "low_voltage", "start": 1.0, "poweroff_at": -5.0},
    {"kind": "low_voltage", "start": 1.0, "poweroff_at": 1e9},        # in the future
    {"kind": "low_voltage", "start": 10 ** 400, "poweroff_at": 2.0},
    {"kind": "low_voltage", "start": "1.0", "poweroff_at": 2.0},
    {"kind": "input_loss"}, [1, 2], "x"])
def test_w3_invalid_marker_is_unparsable(tmp_path, bad):
    fs, w = make(tmp_path, t0=50.0, marker=bad)
    assert w.halter.phase == "kill" and w.halter.kind == "input_loss"
    assert w.halter.t0 == 50.0 and fs.poweroffs() == []
    assert fs.notes[0].startswith("READY=1")


def test_w3_infinity_in_the_marker_file_does_not_crash(tmp_path):
    s = wp.System(str(tmp_path / "run"), str(tmp_path / "state"))
    os.makedirs(str(tmp_path / "run"))
    (tmp_path / "run" / "halting").write_text(
        '{"kind": "low_voltage", "start": Infinity, "poweroff_at": -Infinity}')
    s.log = s.notify = lambda *a, **k: None
    s.set_pin = lambda *a: True
    w = wp.Watcher(s, 100.0)
    assert w.halter.phase == "kill"


def test_w3_resume_exception_falls_back(tmp_path):
    class Odd(dict):
        def get(self, *a):
            raise RuntimeError("boom")
    fs, w = make(tmp_path, t0=5.0, marker=Odd())
    assert w.halter.phase == "kill" and fs.notes[0].startswith("READY=1")


def test_w4_debounce_trips_with_a_2_s_tick_period(tmp_path):
    fs, w = make(tmp_path)
    fs.g6 = 0
    run_ticks(w, 0.0, 6.0, step=2.0)
    assert fs.actions()[0] == ("marker", "input_loss", None)
    assert next(s for s in fs.statuses if s["state"] == "halting")["hb_mono"] == 6.0


def test_w4_cap_still_limits_a_long_gap():
    d = wp.Debounce()
    d.update(0.0, False)
    d.update(10.0, False)
    assert d.low_s == pytest.approx(wp.DEBOUNCE_MAX_DT_S) and not d.tripped


def test_w5_gpio6_path_before_the_gauge(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 10.0, before=low_from(fs, 1.0))
    assert_ping_order(fs)
    # a tier firing on the gauge read still starts its halt in the same pass
    fs2, w2 = make(tmp_path)
    fs2.vcell = 3.0
    run_ticks(w2, 0.0, 2.0)
    before = fs2.calls[:fs2.calls.index(("poweroff", "--force"))]
    last_pass = before[max(i for i, c in enumerate(before) if c == ("read_pins",)):]
    assert ("read_vcell",) in last_pass          # same pass as the gauge read


def test_w6_ready_in_constructor_and_no_ext5v_while_halting(tmp_path):
    fs, w = make(tmp_path)
    assert fs.calls[-1] == ("notify", "READY=1\nWATCHDOG=1")
    assert [c for c in fs.calls if c[0] == "notify"] == [("notify", "READY=1\nWATCHDOG=1")]
    run_ticks(w, 0.0, 30.0, before=low_from(fs, 1.0))
    first_halt = fs.calls.index(("marker", "input_loss", None))
    assert ("ext5v",) not in fs.calls[first_halt:]
    assert fs.statuses[-1]["ext5v_v"] is None


def test_w8_bus_ok_mono_starts_at_watcher_start(tmp_path):
    fs = FakeSys(tmp_path)
    fs.vcell = None                                  # bus stuck from boot
    w = wp.Watcher(fs, 42.0)
    run_ticks(w, 42.0, 50.0)
    s = fs.statuses[-1]
    assert s["bus_ok_mono"] == 42.0 and s["bus_ok_ts"] is None
    fs.vcell = 3.8
    run_ticks(w, 50.25, 51.0)
    s = fs.statuses[-1]
    assert s["bus_ok_mono"] == 51.0 and s["bus_ok_ts"] == 1.7e9


@pytest.mark.parametrize("volts,ok", [(2.4, True), (2.0, True), (1.99, False), (4.5, True), (4.51, False)])
def test_w9_plausibility_window(volts, ok):
    raw = int(round(volts / 1.25e-3)) << 4

    class Bus:
        def __init__(self, n):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read_word_data(self, addr, reg):
            return ((raw & 0xFF) << 8) | (raw >> 8)
    v, _ = wp.read_vcell(Bus)
    assert (v is not None) is ok


def test_w9_tier_b_sees_a_deep_discharge():
    lv = wp.LowVoltage()
    lv.update(0, True, 3.2)
    lv.update(1, True, 2.4)
    lv.update(2, True, 2.2)
    assert lv.update(3, True, 2.1).startswith("low_voltage_b")


def test_w10_loop_survives_tick_errors(tmp_path):
    fs = FakeSys(tmp_path)
    clock = [0.0]

    class W:
        n = 0

        def tick(self, now):
            self.n += 1
            if self.n % 19 != 0:                 # 18 failures, then one good pass
                raise KeyError("bad")

    w = W()

    def sleep(d):
        clock[0] += 0.25
    # Changed for F3: 20 failures in a row now exit, so a good pass every 19th
    wp.loop(w, fs, lambda: clock[0], sleep, ticks=60)
    assert w.n == 60
    errs = [m for lvl, m in fs.logs if lvl == 3]
    assert errs[0].startswith("tick error KeyError") and len(errs) == 2   # t=0 and t=10


@pytest.mark.parametrize("exc", [SystemExit(0), KeyboardInterrupt()])
def test_w10_loop_lets_exit_pass(tmp_path, exc):
    class W:
        def tick(self, now):
            raise exc
    with pytest.raises(type(exc)):
        wp.loop(W(), FakeSys(tmp_path), lambda: 0.0, lambda d: None, ticks=5)


def test_low13_gpio6_pull_set_again_once_a_minute(tmp_path):
    fs, w = make(tmp_path)
    fs.g6_cfg = ("ip", "pn")
    run_ticks(w, 0.0, 130.0)
    sets = [c for c in fs.calls if c[:2] == ("set_pin", 6)]
    assert len(sets) == 1 + 2                            # start, then at 60 s and 120 s


def test_low14_charge_pin_no_flicker_after_a_good_set(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 1.0)
    fs.g16 = ("ip", None)                                # something else changed the pin
    run_ticks(w, 1.25, 3.0)
    assert all("charge_pin" not in s["faults"] for s in fs.statuses)
    assert not any("charge_pin" in m for _, m in fs.logs)


def test_low16_nohalt_directory_is_ignored(tmp_path):
    (tmp_path / "nohalt").mkdir()
    active, note = wp.consume_nohalt(str(tmp_path / "nohalt"))
    assert active is False and "directory" in note


# ---------------------------------------------------------------- review round 2
def test_f1_nohalt_non_utf8_keeps_halt_on(tmp_path):
    p = tmp_path / "nohalt"
    p.write_bytes(b"\xff\xfe3\x80")
    active, note = wp.consume_nohalt(str(p))
    assert active is False and "UnicodeDecodeError" in note and "stays ON" in note
    fs, w = make(tmp_path)                       # the constructor starts, READY is sent
    assert w.nohalt is False and fs.notes[0].startswith("READY=1")
    assert any("stays ON" in m for _, m in fs.logs)


def test_f1_nohalt_unreadable_keeps_halt_on(tmp_path, monkeypatch):
    p = tmp_path / "nohalt"
    p.write_text("3")
    real_open = open

    def fake_open(path, *a, **kw):
        if str(path) == str(p) and (not a or a[0] == "r"):
            raise PermissionError(13, "Permission denied")
        return real_open(path, *a, **kw)
    monkeypatch.setattr("builtins.open", fake_open)
    active, note = wp.consume_nohalt(str(p))
    assert active is False and "PermissionError" in note
    assert p.exists()


def test_f1_nohalt_rewrite_fails_is_logged(tmp_path, monkeypatch):
    p = tmp_path / "nohalt"
    p.write_text("3")
    real_open = open

    def fake_open(path, mode="r", *a, **kw):
        if str(path) == str(p) and "w" in mode:
            raise OSError(30, "Read-only file system")
        return real_open(path, mode, *a, **kw)
    monkeypatch.setattr("builtins.open", fake_open)
    active, note = wp.consume_nohalt(str(p))
    assert active is True and "not updated" in note


def test_f1_constructor_survives_a_raising_nohalt(tmp_path, monkeypatch):
    def boom(path):
        raise RuntimeError("x")
    monkeypatch.setattr(wp, "consume_nohalt", boom)
    fs, w = make(tmp_path)
    assert w.nohalt is False and fs.notes[0].startswith("READY=1")


def test_f2_low_fail_alternating_reaches_the_halt(tmp_path):
    fs, w = make(tmp_path)
    run_ticks(w, 0.0, 10.0, before=lambda t: setattr(
        fs, "g6", 1 if t < 1.0 else (0 if int(round(t * 4)) % 2 == 0 else None)))
    t_halt = next(s for s in fs.statuses if s["state"] == "halting")["hb_mono"]
    assert 5.75 <= t_halt <= 6.5


def test_f2_fail_only_never_trips():
    d = wp.Debounce()
    for i in range(200):
        d.update(i * 0.25, None)
    assert d.low_s == 0 and not d.tripped


def test_f2_low_fail_high_credits_one_capped_step():
    d = wp.Debounce()
    d.update(0.0, False)
    for i in range(1, 40):
        d.update(i * 0.25, None)                 # 9.75 s of failed reads
    d.update(10.0, True)
    assert d.low_s == pytest.approx(wp.DEBOUNCE_MAX_DT_S)


def test_f3_start_of_tick_ping(tmp_path):
    fs, w = make(tmp_path)
    fs.calls.clear()
    w.tick(0.0)
    assert fs.calls[0] == ("notify", "WATCHDOG=1") and fs.calls[1] == ("read_pins",)


def test_f3_no_ping_from_the_error_path_and_exit_after_20(tmp_path):
    fs = FakeSys(tmp_path)

    class W:
        n = 0

        def tick(self, now):
            self.n += 1
            raise ValueError("dead tiers")
    w = W()
    with pytest.raises(SystemExit) as e:
        wp.loop(w, fs, lambda: 0.0, lambda d: None)
    assert e.value.code == 1 and w.n == wp.MAX_FAILED_TICKS == 20
    assert fs.notes == []
    assert any("20 failed passes" in m for _, m in fs.logs)


def test_f3_clock_and_sleep_errors_are_caught(tmp_path):
    fs = FakeSys(tmp_path)
    calls = {"clock": 0}

    def clock():
        calls["clock"] += 1
        if calls["clock"] in (5, 9, 30):
            raise OSError("clock")
        return 0.0

    def sleep(d):
        raise OSError("sleep")

    class W:
        n = 0

        def tick(self, now):
            self.n += 1
    w = W()
    try:
        wp.loop(w, fs, clock, sleep, ticks=30)
    except SystemExit:
        pytest.fail("a good tick must reset the failed count")
    assert w.n >= 25                             # the loop went on after each error
    assert any("OSError" in m for _, m in fs.logs)


def test_f4_abandoned_children_hold_no_pipe_and_are_polled(monkeypatch):
    wp._ABANDONED.clear()
    P, made = fake_popen("stuck")
    monkeypatch.setattr(wp.subprocess, "Popen", P)
    for _ in range(50):
        wp.run(["pinctrl", "get", "6"])
    assert len(made) == 50 and all(c.stdout.closed for c in made)
    assert len(wp._ABANDONED) == 50
    for c in made:
        c.returncode = -9                        # they finally die
    P2, _ = fake_popen("ok")
    monkeypatch.setattr(wp.subprocess, "Popen", P2)
    wp.run(["pinctrl", "get", "6"])
    assert all(c.polls >= 1 for c in made) and wp._ABANDONED == []


def test_f5_ext5v_not_finite(monkeypatch, tmp_path):
    for text in ("EXT5V_V volt(24)=nanV\n", "EXT5V_V volt(24)=infV\n"):
        P, _ = fake_popen(outs={"vcgencmd": text})
        monkeypatch.setattr(wp.subprocess, "Popen", P)
        assert wp.System(str(tmp_path), str(tmp_path)).ext5v() is None


# ---------------------------------------------------------------- bench round 1: events
def ev_names(fs):
    return [e["event"] for e in fs.events]


def test_b1_event_lines_input_loss_halt(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active", "inactive"]
    run_ticks(w, 0.0, 200.0, before=low_from(fs, 1.0))
    assert ev_names(fs) == ["start", "halt", "kill", "stopped", "poweroff", "force",
                            "force_force"] + ["force_force"] * (ev_names(fs).count("force_force") - 1)
    st = fs.events[0]
    assert st["version"] == wp.VERSION and st["nohalt"] is False and st["resumed_marker"] is False
    h = fs.events[1]
    assert set(h) == {"boot_id", "bt", "wall", "event", "kind", "reason", "vcell_v", "low_s",
                      "input_good"}
    assert h["kind"] == "input_loss" and h["low_s"] == 5.0 and h["input_good"] is False
    assert h["bt"] == 6.0 and h["boot_id"] == "boot-1" and h["wall"] == 1.7e9
    # the marker first, then the event line, then the poweroff call
    c = fs.calls
    assert c.index(("marker", "input_loss", None)) < c.index(("event", "halt"))
    i_ev = c.index(("event", "poweroff"))
    assert c[i_ev - 1][0] == "marker" and c[i_ev + 1] == ("poweroff", "--no-wall")
    i_ff = c.index(("event", "force_force"))
    assert c[i_ff + 1] == ("poweroff", "--force", "--force")


def test_b1_event_lines_cancelled_halt(tmp_path):
    fs, w = make(tmp_path)
    fs.robot = ["active", "active", "inactive"]
    run_ticks(w, 0.0, 20.0, before=lambda t: setattr(fs, "g6", 0 if 1.0 <= t < 7.0 else 1))
    assert ev_names(fs) == ["start", "halt", "kill", "stopped", "cancel"]
    assert fs.events[-1]["kind"] == "input_loss"


def test_b1_event_lines_low_voltage_halt(tmp_path):
    fs, w = make(tmp_path)
    fs.vcell = 3.05
    run_ticks(w, 0.0, 70.0)
    assert ev_names(fs) == ["start", "tier", "halt", "force", "force_force"]
    assert fs.events[1]["reason"].startswith("low_voltage_b") and fs.events[1]["vcell_v"] == 3.05
    c = fs.calls
    i = c.index(("event", "halt"))
    assert c[i - 2] == ("marker", "low_voltage", 2.0) and c[i + 1] == ("poweroff", "--force")


def test_b1_event_lines_resume(tmp_path):
    fs, w = make(tmp_path, t0=145.0,
                 marker={"kind": "input_loss", "reason": "r", "start": 99.0, "poweroff_at": 100.0})
    assert ev_names(fs) == ["start", "resume"]
    assert fs.events[0]["resumed_marker"] is True
    r = fs.events[1]
    assert (r["phase"], r["kind"], r["reason"]) == ("ladder", "input_loss", "r")
    run_ticks(w, 145.0, 145.0)
    assert ev_names(fs)[-1] == "force"


def test_b1_write_error_does_not_stop_the_halt(tmp_path):
    fs, w = make(tmp_path)
    fs.event_err = "events: OSError: disk full"
    run_ticks(w, 0.0, 30.0, before=low_from(fs, 1.0))
    assert ("poweroff", "--no-wall") in fs.poweroffs()
    assert any(s["last_error"] == "events: OSError: disk full" for s in fs.statuses)

    class Raising(FakeSys):
        def event(self, rec):
            raise OSError("boom")
    fs2 = Raising(tmp_path)
    w2 = wp.Watcher(fs2, 0.0)
    run_ticks(w2, 0.0, 30.0, before=low_from(fs2, 1.0))
    assert ("poweroff", "--no-wall") in fs2.poweroffs()


def test_b1_exit_failed_ticks_event(tmp_path):
    fs, w = make(tmp_path)

    def bad(now):
        raise ValueError("x")
    w.tick = bad
    with pytest.raises(SystemExit):
        wp.loop(w, fs, lambda: 5.0, lambda d: None)
    e = fs.events[-1]
    assert e["event"] == "exit_failed_ticks" and e["failed"] == 20 and e["error"] == "ValueError"


def test_b1_system_event_appends_and_fsyncs(tmp_path, monkeypatch):
    s = wp.System(str(tmp_path / "run"), str(tmp_path))
    synced = []
    real = wp.os.fsync
    monkeypatch.setattr(wp.os, "fsync", lambda fd: (synced.append(fd), real(fd)))
    for i in range(3):
        assert s.event({"event": "kill", "i": i}) is None
    lines = (tmp_path / "events.jsonl").read_text().splitlines()
    assert [json.loads(l)["i"] for l in lines] == [0, 1, 2]
    assert len(synced) == 3


def test_b1_system_event_error_never_raises(tmp_path):
    s = wp.System(str(tmp_path / "run"), str(tmp_path / "no" / "such" / "dir"))
    err = s.event({"event": "halt"})
    assert err.startswith("events: FileNotFoundError")
    assert s.event({"bad": object()}).startswith("events: TypeError")


def test_b1_size_cap_keeps_the_newest_half(tmp_path):
    s = wp.System(str(tmp_path / "run"), str(tmp_path))
    path = tmp_path / "events.jsonl"
    line = json.dumps({"event": "kill", "pad": "x" * 100})
    n = (300 * 1024) // (len(line) + 1)
    path.write_text("".join(json.dumps({"event": "kill", "i": i, "pad": "x" * 100}) + "\n"
                            for i in range(n)))
    assert path.stat().st_size > wp.EVENTS_MAX_BYTES
    assert s.trim_events() is None
    data = path.read_text()
    assert len(data.encode()) <= wp.EVENTS_MAX_BYTES // 2
    recs = [json.loads(l) for l in data.splitlines()]   # whole lines only
    assert recs[-1]["i"] == n - 1 and recs[0]["i"] > 0
    assert not (tmp_path / "events.jsonl.tmp").exists()
    assert s.trim_events() is None                       # small now: untouched
    assert path.read_text() == data


def test_b1_size_cap_problems_never_fail_the_start(tmp_path):
    s = wp.System(str(tmp_path / "run"), str(tmp_path))
    assert s.trim_events() is None                       # no file yet
    fs = FakeSys(tmp_path)

    def boom():
        raise RuntimeError("trim")
    fs.trim_events = boom
    w = wp.Watcher(fs, 0.0)
    assert fs.notes[0].startswith("READY=1") and "trim" in w.last_error
