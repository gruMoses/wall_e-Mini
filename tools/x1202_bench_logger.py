#!/usr/bin/env python3
"""Bench logger for the Geekworm X1202 UPS board (Pi 5).

Runs on the Pi. Samples the power-good pin (GPIO 6), the charge-disable pin
(GPIO 16), the MAX17040 fuel gauge (VCELL, SOC) and, at 1 Hz, EXT5V,
get_throttled and the SoC temperature. Writes one JSON object per line.

Every line carries seq, boot_id, boottime (CLOCK_BOOTTIME seconds) and wall
time. The wall clock is wrong after a power cut until NTP syncs (no RTC
coin cell), so analyse with boot_id + boottime.

Optional parts:
  --charge leave|disable|hold   GPIO 16 policy (see HoldController)
  --guard                       bench halt on a lasting input loss (see Guard)

Bench tool only; it is not part of the robot app. Hardware imports are lazy
so this file imports on a Mac for the unit tests.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Callable, Optional

GAUGE_ADDR = 0x36
REG_VCELL = 0x02
REG_SOC = 0x04
GPIO_PG = 6        # high = input power good
GPIO_CHG = 16      # high = charging disabled
FLAG_FILE = "/home/pi/x1202_bench/ENABLED"
GUARD_INHIBIT_FILE = "/home/pi/x1202_bench/GUARD_INHIBIT"
# While this file is active (same format as the guard inhibit) the logger does
# not drive GPIO 16, so the bench harness can enable charging by hand. It
# expires by itself, after which the --charge policy applies again.
CHARGE_LEAVE_FILE = "/home/pi/x1202_bench/CHARGE_LEAVE"
VCELL_STEP_V = 0.030
GAUGE_FAIL_LIMIT = 10
# A decoded VCELL outside this band is not a real 1S Li-ion reading (0.0 V and
# ~5.12 V = raw 0x0000 / 0xFFFF come from a dead or floating bus): the gauge
# counts as unreadable.
VCELL_MIN_V = 2.5
VCELL_MAX_V = 4.5
GUARD_CLEAR_HIGH_SAMPLES = 3


# --------------------------------------------------------------------------
# Pure decode helpers
# --------------------------------------------------------------------------
def swap_word(w: int) -> int:
    """SMBus read_word_data is little-endian; the MAX17040 is big-endian."""
    return ((w & 0xFF) << 8) | ((w >> 8) & 0xFF)


def decode_vcell(raw: int) -> float:
    """raw = big-endian register value. 12-bit, 1.25 mV per LSB."""
    return (raw >> 4) * 1.25 / 1000.0


def vcell_plausible(v: Optional[float]) -> bool:
    return v is not None and VCELL_MIN_V <= v <= VCELL_MAX_V


def decode_soc(raw: int) -> float:
    """raw = big-endian register value. High byte = percent, low byte = 1/256."""
    return raw / 256.0


def parse_pinctrl_line(line: str) -> dict:
    """Parse one `pinctrl get` line, e.g. '16: op dh pd | hi // GPIO16 = output'.

    Returns {"func": "op", "drive": "dh"|None, "pull": "pd", "level": 1|0|None}.
    """
    out = {"func": None, "drive": None, "pull": None, "level": None}
    try:
        left, rest = line.split(":", 1)
        cfg, after = rest.split("|", 1)
        toks = cfg.split()
        if toks:
            out["func"] = toks[0]
        for t in toks[1:]:
            if t in ("dh", "dl"):
                out["drive"] = t
            elif t in ("pu", "pd", "pn"):
                out["pull"] = t
        lv = after.split("//")[0].strip()
        out["level"] = 1 if lv == "hi" else 0 if lv == "lo" else None
    except ValueError:
        pass
    return out


# --------------------------------------------------------------------------
# Charge policy
# --------------------------------------------------------------------------
class HoldController:
    """Keeps the cells near hold_v with charging disabled most of the time.

    Decision per sample. Returns True when charging must be DISABLED
    (GPIO 16 high), False when a charge burst is running (GPIO 16 low).

    - Rest timer starts at construction, at the end of each burst, and on
      each sample with the input absent (the cells carry the Pi: that is load,
      not rest).
    - A burst starts when the input is good, the cells have rested for
      rest_s, and VCELL < hold_v - hyst.
    - A burst ends after burst_s, or at once if the input fails.
    - An unreadable VCELL never starts a burst, and ends a running burst at
      once.
    """

    def __init__(self, hold_v=3.80, hyst=0.05, rest_s=120.0, burst_s=120.0, now=0.0):
        self.hold_v = hold_v
        self.hyst = hyst
        self.rest_s = rest_s
        self.burst_s = burst_s
        self.rest_since = now
        self.burst_until: Optional[float] = None

    @property
    def bursting(self) -> bool:
        return self.burst_until is not None

    def decide(self, now: float, input_good: bool, vcell: Optional[float]) -> bool:
        if not input_good:
            self.burst_until = None
            self.rest_since = now
            return True
        if self.burst_until is not None:
            if vcell is None or now >= self.burst_until:
                self.burst_until = None
                self.rest_since = now
                return True
            return False
        if (vcell is not None and now - self.rest_since >= self.rest_s
                and vcell < self.hold_v - self.hyst):
            self.burst_until = now + self.burst_s
            return False
        return True


class ChargeManager:
    """Wraps the --charge mode and the fail-to-disabled rule.

    desired() returns True (disable), False (enable) or None (do not touch).
    Fail rule (modes disable and hold): GAUGE_FAIL_LIMIT unreadable samples in
    a row force "disabled" until the gauge reads again. In mode leave the
    logger never drives the pin; it only reports the failure.
    """

    def __init__(self, mode: str, hold: Optional[HoldController] = None,
                 fail_limit: int = GAUGE_FAIL_LIMIT):
        if mode not in ("leave", "disable", "hold"):
            raise ValueError(mode)
        self.mode = mode
        self.hold = hold
        self.fail_limit = fail_limit
        self.fail_count = 0
        self.failsafe = False

    def desired(self, now: float, input_good: bool, vcell: Optional[float]) -> Optional[bool]:
        if vcell is None:
            self.fail_count += 1
        else:
            self.fail_count = 0
            self.failsafe = False
        if self.fail_count >= self.fail_limit:
            self.failsafe = True
        if self.mode == "leave":
            return None
        if self.failsafe:
            if self.hold is not None:
                # A burst must not resume after the gauge comes back.
                self.hold.burst_until = None
                self.hold.rest_since = now
            return True
        if self.mode == "disable":
            return True
        return self.hold.decide(now, input_good, vcell)


# --------------------------------------------------------------------------
# Bench guard
# --------------------------------------------------------------------------
class Guard:
    """Bench-only halt on a lasting input loss.

    Fires (returns a reason string) when ALL of these hold on a sample:
      - the flag file exists (flag_exists()),
      - no inhibit is active (inhibited()),
      - the gauge answered on this sample,
    and EITHER GPIO 6 has been low for halt_s, OR VCELL has been at or below
    vcell_v for vcell_s while GPIO 6 is low.
    The timers clear only after GPIO 6 reads high on clear_after samples in a
    row; a single high glitch does not restart them. A failed pin read
    (input_good None) or a failed gauge read neither clears nor fires.
    Without the gauge it does nothing (a Pi without an X1202).
    After firing it re-arms after retry_s (if the poweroff did not happen).
    """

    def __init__(self, flag_exists: Callable[[], bool], halt_s=180.0, vcell_v=3.30,
                 vcell_s=5.0, retry_s=30.0, inhibited: Callable[[], bool] = lambda: False,
                 clear_after: int = GUARD_CLEAR_HIGH_SAMPLES):
        self.flag_exists = flag_exists
        self.clear_after = max(1, int(clear_after))
        self.high_count = 0
        self.inhibited = inhibited
        self.halt_s = halt_s
        self.vcell_v = vcell_v
        self.vcell_s = vcell_s
        self.retry_s = retry_s
        self.low_since: Optional[float] = None
        self.vlow_since: Optional[float] = None
        self.fired_at: Optional[float] = None

    def update(self, now: float, input_good: Optional[bool], vcell: Optional[float]) -> Optional[str]:
        # Track the input-low duration whatever the gauge does.
        if input_good is None:
            return None                      # pin unreadable: keep timers, never fire
        if input_good:
            self.high_count += 1
            if self.high_count >= self.clear_after:
                self.low_since = None
                self.vlow_since = None
                self.fired_at = None
            return None
        self.high_count = 0
        if self.low_since is None:
            self.low_since = now
        if vcell is not None:
            if vcell <= self.vcell_v:
                if self.vlow_since is None:
                    self.vlow_since = now
            else:
                self.vlow_since = None
        if vcell is None:
            return None                      # gauge absent: do nothing at all
        if not self.flag_exists() or self.inhibited():
            return None
        if self.fired_at is not None and now - self.fired_at < self.retry_s:
            return None
        reason = None
        if now - self.low_since >= self.halt_s:
            reason = "input_low_%.0fs" % (now - self.low_since)
        elif self.vlow_since is not None and now - self.vlow_since >= self.vcell_s:
            reason = "vcell_%.3f_le_%.2f_for_%.0fs" % (vcell, self.vcell_v, now - self.vlow_since)
        if reason:
            self.fired_at = now
        return reason


def read_inhibit(path: str, boot_id: str, boottime: float) -> bool:
    """Inhibit file content: '<boot_id> <boottime deadline>'. Active only in
    the same boot and before the deadline. Any doubt = not inhibited."""
    try:
        with open(path) as f:
            parts = f.read().split()
        return len(parts) == 2 and parts[0] == boot_id and boottime < float(parts[1])
    except (OSError, ValueError):
        return False


# --------------------------------------------------------------------------
# Hardware access (Pi only)
# --------------------------------------------------------------------------
def boottime() -> float:
    return time.clock_gettime(time.CLOCK_BOOTTIME)


def read_boot_id() -> str:
    try:
        with open("/proc/sys/kernel/random/boot_id") as f:
            return f.read().strip()
    except OSError:
        return "unknown"


def run_cmd(args, timeout=2.0) -> str:
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


class Hardware:
    def __init__(self):
        self.bus = None
        self.last_err: Optional[str] = None

    def _bus(self):
        if self.bus is None:
            import smbus2  # lazy
            self.bus = smbus2.SMBus(1)
        return self.bus

    def read_gauge(self):
        """Returns (vcell, soc, error_str)."""
        try:
            b = self._bus()
            raw_v = swap_word(b.read_word_data(GAUGE_ADDR, REG_VCELL))
            v = decode_vcell(raw_v)
            s = decode_soc(swap_word(b.read_word_data(GAUGE_ADDR, REG_SOC)))
            if not vcell_plausible(v):
                return None, None, "implausible_vcell:raw=0x%04X:v=%.4f" % (raw_v, v)
            return v, s, None
        except Exception as e:  # noqa: BLE001
            try:
                if self.bus is not None:
                    self.bus.close()
            except Exception:
                pass
            self.bus = None
            return None, None, type(e).__name__ + ":" + str(e)[:60]

    def read_pins(self):
        out = run_cmd(["pinctrl", "get", "%d,%d" % (GPIO_PG, GPIO_CHG)])
        pins = {}
        for line in out.splitlines():
            if ":" in line:
                try:
                    n = int(line.split(":", 1)[0])
                except ValueError:
                    continue
                pins[n] = parse_pinctrl_line(line)
        return pins.get(GPIO_PG), pins.get(GPIO_CHG)

    def set_charge_disabled(self, disabled: bool) -> bool:
        """Never raises; on failure returns False and sets self.last_err."""
        self.last_err = None
        try:
            r = subprocess.run(["pinctrl", "set", str(GPIO_CHG), "op", "dh" if disabled else "dl"],
                               capture_output=True, timeout=2)
        except (subprocess.TimeoutExpired, OSError) as e:
            self.last_err = type(e).__name__ + ":" + str(e)[:60]
            return False
        if r.returncode != 0:
            self.last_err = "rc=%d" % r.returncode
        return r.returncode == 0

    def ext5v(self) -> Optional[float]:
        out = run_cmd(["vcgencmd", "pmic_read_adc", "EXT5V_V"])
        try:
            return float(out.split("=")[-1].strip().rstrip("V"))
        except ValueError:
            return None

    def throttled(self) -> Optional[str]:
        out = run_cmd(["vcgencmd", "get_throttled"]).strip()
        return out.split("=")[-1] if "=" in out else None

    def soc_temp(self) -> Optional[float]:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                return int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            return None


class EdgeMonitor(threading.Thread):
    """Best effort: kernel edge events on GPIO 6 through gpiod v1, so a
    glitch shorter than the sample period is still seen. Bias AS_IS."""

    def __init__(self, emit: Callable[[dict], None]):
        super().__init__(daemon=True)
        self.emit = emit
        self.ok = False
        self.err = None

    def run(self):
        try:
            import gpiod  # lazy
            chip = gpiod.Chip("gpiochip0")
            if chip.label() != "pinctrl-rp1":
                raise RuntimeError("gpiochip0 is " + chip.label())
            line = chip.get_line(GPIO_PG)
            line.request(consumer="x1202_bench", type=gpiod.LINE_REQ_EV_BOTH_EDGES)
            self.ok = True
            while True:
                if line.event_wait(sec=1):
                    ev = line.event_read()
                    rising = ev.type == gpiod.LineEvent.RISING_EDGE
                    self.emit({"event": "gpio6_kernel_edge", "edge": "rise" if rising else "fall",
                               "ev_ts": ev.sec + ev.nsec / 1e9})
        except Exception as e:  # noqa: BLE001
            self.err = type(e).__name__ + ":" + str(e)[:80]
            self.emit({"event": "edge_monitor_unavailable", "err": self.err})


def should_drive_charge(want: Optional[bool], chg: Optional[dict]) -> bool:
    """True when GPIO 16 must be driven to `want` this sample.

    An unknown pin state (failed read) is driven when the wanted state is
    "disabled" (the safe state); "enabled" is never driven blind.
    """
    if want is None:
        return False
    if chg is None:
        return bool(want)
    have_disabled = chg.get("func") == "op" and chg.get("drive") == "dh"
    have_enabled = chg.get("func") == "op" and chg.get("drive") == "dl"
    return (want and not have_disabled) or (not want and not have_enabled)


def exit_charge_safe(hw, mode: str) -> Optional[dict]:
    """On exit, modes disable and hold drive GPIO 16 high (charging disabled),
    best effort, one command timeout at most. Mode leave never touches the pin.
    Returns the event to log, or None."""
    if mode not in ("disable", "hold"):
        return None
    ok = hw.set_charge_disabled(True)
    return {"event": "charge_set_exit", "disabled": True, "ok": ok,
            "err": getattr(hw, "last_err", None)}


# --------------------------------------------------------------------------
# Writer
# --------------------------------------------------------------------------
class JsonlWriter:
    def __init__(self, path: str, boot_id: str):
        self.path = path
        self.boot_id = boot_id
        self.seq = 0
        self.lock = threading.RLock()  # the SIGTERM handler writes from the main thread
        self.last_fsync = 0.0
        self.f = None
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.f = open(path, "a", buffering=1)
        except OSError as e:
            sys.stderr.write("x1202 logger: cannot open %s: %s\n" % (path, e))

    def write(self, obj: dict, sync: bool = False):
        with self.lock:
            rec = {"seq": self.seq, "boot_id": self.boot_id,
                   "bt": round(boottime(), 3), "wall": round(time.time(), 3)}
            rec.update(obj)
            self.seq += 1
            line = json.dumps(rec, separators=(",", ":"))
            if self.f is None:
                return
            try:
                self.f.write(line + "\n")
                self.f.flush()
                now = time.monotonic()
                if sync or now - self.last_fsync >= 1.0:
                    os.fsync(self.f.fileno())
                    self.last_fsync = now
            except (OSError, ValueError):
                pass  # read-only root late in shutdown, full disk: keep going

    def close(self):
        with self.lock:
            if self.f is None:
                return
            try:
                self.f.flush()
                os.fsync(self.f.fileno())
                self.f.close()
            except (OSError, ValueError):
                pass
            self.f = None


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--out-dir", default="/home/pi/x1202_bench/logs/")
    ap.add_argument("--charge", choices=("leave", "disable", "hold"), default="leave")
    ap.add_argument("--hold-v", type=float, default=3.80)
    ap.add_argument("--hold-hyst", type=float, default=0.05)
    ap.add_argument("--hold-rest-s", type=float, default=120.0)
    ap.add_argument("--hold-burst-s", type=float, default=120.0)
    ap.add_argument("--guard", action="store_true")
    ap.add_argument("--guard-halt-s", type=float, default=180.0)
    ap.add_argument("--guard-vcell", type=float, default=3.30)
    ap.add_argument("--no-edge-monitor", action="store_true")
    a = ap.parse_args(argv)

    hw = Hardware()
    boot_id = read_boot_id()
    bt0 = boottime()
    path = os.path.join(a.out_dir, "x1202_%s_up%07.1f.jsonl" % (boot_id[:8], bt0))
    w = JsonlWriter(path, boot_id)

    # Header: GPIO 16 AS FOUND, before anything touches it.
    pg, chg = hw.read_pins()
    v, s, err = hw.read_gauge()
    w.write({"type": "header", "uptime_s": round(bt0, 3), "gpio16_found": chg,
             "gpio6_found": pg, "vcell": v, "soc": s, "gauge_err": err,
             "args": vars(a), "pid": os.getpid()}, sync=True)

    stop = threading.Event()

    def on_term(signum, frame):
        w.write({"type": "event", "event": "sigterm", "signal": signum}, sync=True)
        stop.set()

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    if not a.no_edge_monitor:
        EdgeMonitor(lambda d: w.write(dict(type="event", **d), sync=True)).start()

    now = boottime()
    hold = HoldController(a.hold_v, a.hold_hyst, a.hold_rest_s, a.hold_burst_s, now=now)
    cm = ChargeManager(a.charge, hold if a.charge == "hold" else None)
    guard = Guard(lambda: os.path.exists(FLAG_FILE), halt_s=a.guard_halt_s, vcell_v=a.guard_vcell,
                  inhibited=lambda: read_inhibit(GUARD_INHIBIT_FILE, boot_id, boottime()))

    try:
        _loop(a, hw, w, stop, cm, guard, pg, chg, v, err, boot_id)
    finally:
        ev = exit_charge_safe(hw, a.charge)
        if ev is not None:
            w.write(dict(type="event", **ev), sync=True)
        w.write({"type": "event", "event": "exit"}, sync=True)
        w.close()
    return 0


def _loop(a, hw, w, stop, cm, guard, pg, chg, v, err, boot_id):
    period = 1.0 / max(a.rate, 0.5)
    last_pg = pg.get("level") if pg else None
    last_chg = (chg.get("func"), chg.get("drive"), chg.get("level")) if chg else None
    last_v = v
    i2c_bad = err is not None
    last_slow = -9.0
    last_failsafe = False
    last_leave = False
    next_t = boottime()

    while not stop.is_set():
        now = boottime()
        pg, chg = hw.read_pins()
        if stop.is_set():
            break
        v, s, err = hw.read_gauge()
        lvl = pg.get("level") if pg else None
        rec = {"type": "s", "g6": lvl,
               "g16": chg.get("level") if chg else None,
               "g16f": (chg.get("func") or "") + "/" + (chg.get("drive") or "") if chg else None,
               "vcell": round(v, 4) if v is not None else None,
               "soc": round(s, 2) if s is not None else None}
        if err is not None:
            rec["gauge_err"] = err
        if now - last_slow >= 1.0:
            last_slow = now
            # Skip the remaining slow reads once a stop is requested.
            for key, fn in (("ext5v", hw.ext5v), ("thr", hw.throttled), ("temp", hw.soc_temp)):
                if stop.is_set():
                    break
                rec[key] = fn()
        if stop.is_set():
            break                            # no new subprocess after stop
        events = []
        if lvl != last_pg:
            events.append({"event": "gpio6_edge", "from": last_pg, "to": lvl})
            last_pg = lvl
        cur_chg = (chg.get("func"), chg.get("drive"), chg.get("level")) if chg else None
        if cur_chg != last_chg:
            events.append({"event": "gpio16_change", "from": last_chg, "to": cur_chg})
            last_chg = cur_chg
        if v is not None and last_v is not None and abs(v - last_v) >= VCELL_STEP_V:
            events.append({"event": "vcell_step", "from": round(last_v, 4), "to": round(v, 4)})
        if v is not None:
            last_v = v
        if (err is not None) != i2c_bad:
            i2c_bad = err is not None
            events.append({"event": "i2c_error_start" if i2c_bad else "i2c_error_end", "err": err})

        input_good = None if lvl is None else bool(lvl)
        want = cm.desired(now, bool(input_good), v)
        leave = read_inhibit(CHARGE_LEAVE_FILE, boot_id, now)
        if leave != last_leave:
            events.append({"event": "charge_leave_on" if leave else "charge_leave_off"})
            last_leave = leave
        if leave and not cm.failsafe:
            want = None
        if cm.failsafe != last_failsafe:
            events.append({"event": "charge_failsafe_on" if cm.failsafe else "charge_failsafe_off"})
            last_failsafe = cm.failsafe
        if should_drive_charge(want, chg):
            ok = hw.set_charge_disabled(want)
            events.append({"event": "charge_set", "disabled": want, "ok": ok,
                           "mode": a.charge, "vcell": v, "pin_known": chg is not None,
                           "err": hw.last_err})

        if a.guard:
            reason = guard.update(now, input_good, v)
            if reason:
                w.write(dict(rec, type="event", event="guard_poweroff", reason=reason), sync=True)
                try:
                    subprocess.Popen(["sudo", "-n", "poweroff"])
                except OSError as e:
                    events.append({"event": "guard_poweroff_error",
                                   "err": type(e).__name__ + ":" + str(e)[:60]})
                rec["guard_fired"] = reason

        if events:
            for e in events:
                w.write(dict(rec, type="event", **e), sync=True)
        else:
            w.write(rec)

        next_t += period
        delay = next_t - boottime()
        if delay < -period:
            next_t = boottime()
            delay = 0
        stop.wait(max(0.0, delay))


if __name__ == "__main__":
    sys.exit(main())
