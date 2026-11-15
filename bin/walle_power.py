#!/usr/bin/env python3
"""WALL-E power watcher for the Geekworm X1202 UPS board (Raspberry Pi 5).

Runs as walle-power.service (Type=notify, WatchdogSec). It halts the Pi when
the board input is lost (GPIO 6 low for 5 s within 7.5 s; a re-check can
cancel), forces a power-off when the cells are low (tiers a and b; never
cancelled), keeps charging disabled (GPIO 16 driven high) except for the
optional hold bursts (off by default), and writes /run/walle-power/status.json.

The specification is docs/x1202_power_watcher_design.md. Change it first.
All decisions are in the small classes below; they take the clock and the
readings as arguments. Only System touches the OS, and none of its calls
raises or blocks without a bound.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
from collections import deque

GPIO_IN, GPIO_CHG = 6, 16          # 6 high = input good; 16 high = charging disabled
GPIO_IN_PULL = "pd"                # design section 3: "pn" if the bench rejects the pull-down
GAUGE_ADDR, REG_VCELL = 0x36, 0x02
VCELL_MIN_V, VCELL_MAX_V = 2.0, 4.5   # low floor: tier (b) must still see a deep discharge
PERIOD_S, GAUGE_PERIOD_S, CMD_TIMEOUT_S, KILL_WAIT_S = 0.25, 1.0, 2.0, 1.0
# The cap stops a long gap from counting in full; at 2.5 s a 2 s loop period
# (pinctrl at its full timeout) still reaches 5 s in 7.5 s.
DEBOUNCE_NEED_S, DEBOUNCE_WINDOW_S, DEBOUNCE_MAX_DT_S = 5.0, 7.5, 2.5
RECHECK_UNREADABLE_S = 2.0         # re-check: pin unreadable this long -> power off
RETRY_S = 60.0                     # EEPROM check and GPIO 6 re-configuration retry
ERROR_LOG_S, MAX_FAILED_TICKS = 10.0, 20
EVENTS_MAX_BYTES = 256 * 1024
VERSION = "walle-power 2026-11-15"
CELLS_REST_S = 120.0               # cells_high is judged only on a rested sample
NOTICES = {"cells_high"}           # in "faults" for /debug, but no periodic FAULT warning
TIER_A_V, TIER_A_S = 3.30, 5.0
TIER_B_V, TIER_B_SAMPLES = 3.10, 3
STOP_WAIT_S, POLL_S, RECHECK_SAMPLES = 15.0, 0.5, 2
LADDER_STEP_S = 30.0               # --force at 30 s, --force --force at 60 s, again each 30 s
GAUGE_FAILSAFE_READS, GAUGE_FAULT_S, FAULT_WARN_S = 3, 60.0, 60.0
HOLD_ENABLED = False               # charge hold, branch A (design section 5); ships OFF
HOLD_V, HOLD_REST_S, HOLD_BURST_S = 3.75, 120.0, 120.0
CELLS_HIGH_V = 3.95
NOHALT_DEFAULT = 3
ROBOT = "wall-e.service"
BUSY = ("active", "activating", "deactivating", "reloading", "refreshing")
EPS = 1e-6


def boottime() -> float:
    return time.clock_gettime(time.CLOCK_BOOTTIME)


_ABANDONED = []                    # children abandoned by run(); reaped by later calls
ABANDONED_MAX = 64


def run(args, timeout=CMD_TIMEOUT_S):
    """(returncode, stdout), or (None, error text). Never raises, and never
    waits more than timeout + KILL_WAIT_S: subprocess.run() waits without a
    limit after its kill, so a child stuck in the kernel is abandoned here
    (pipes closed, kept in _ABANDONED until a later call reaps it)."""
    for c in list(_ABANDONED):
        try:
            if c.poll() is not None:
                _ABANDONED.remove(c)
        except Exception:  # noqa: BLE001
            _ABANDONED.remove(c)
    try:
        p = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, text=True)
    except Exception as e:  # noqa: BLE001 - a missing tool, anything
        return None, ("%s: %s" % (type(e).__name__, e))[:120]
    try:
        out, _ = p.communicate(timeout=timeout)
        return p.returncode, out or ""
    except Exception as e:  # noqa: BLE001
        err = ("%s: %s" % (type(e).__name__, e))[:120]
    try:
        p.kill()
        p.communicate(timeout=KILL_WAIT_S)
    except Exception:  # noqa: BLE001 - still stuck: abandon the child
        err += " (child abandoned)"
        for pipe in (getattr(p, n, None) for n in ("stdin", "stdout", "stderr")):
            try:
                if pipe is not None:
                    pipe.close()
            except Exception:  # noqa: BLE001
                pass
        if len(_ABANDONED) < ABANDONED_MAX:
            _ABANDONED.append(p)
    return None, err


def parse_pinctrl_line(line: str) -> dict:
    """'16: op dh pd | hi // GPIO16 = output' -> func, drive, pull, level."""
    out = {"func": None, "drive": None, "pull": None, "level": None}
    try:
        cfg, after = line.split(":", 1)[1].split("|", 1)
    except (IndexError, ValueError):
        return out
    toks = cfg.split()
    out["func"] = toks[0] if toks else None
    for t in toks[1:]:
        if t in ("dh", "dl"):
            out["drive"] = t
        elif t in ("pu", "pd", "pn"):
            out["pull"] = t
    lv = after.split("//")[0].strip()
    out["level"] = 1 if lv == "hi" else 0 if lv == "lo" else None
    return out


def read_vcell(smbus_factory=None):
    """(volts, None) or (None, error). The bus is opened for each read, so a
    bus-guard unbind and rebind needs no restart. Without smbus2 or
    /dev/i2c-1 the gauge only counts as unreadable."""
    try:
        if smbus_factory is None:
            from smbus2 import SMBus as smbus_factory  # lazy: optional on this host
        with smbus_factory(1) as bus:
            w = bus.read_word_data(GAUGE_ADDR, REG_VCELL)
    except Exception as e:  # noqa: BLE001
        return None, ("i2c %s: %s" % (type(e).__name__, e))[:120]
    raw = ((w & 0xFF) << 8) | ((w >> 8) & 0xFF)   # SMBus word is little-endian
    v = (raw >> 4) * 1.25e-3
    if not VCELL_MIN_V <= v <= VCELL_MAX_V:
        return None, "implausible vcell raw=0x%04X" % raw
    return v, None


class Debounce:
    """Cumulative GPIO 6 low time in the last window. On each readable
    sample, the time since the last READABLE sample counts as low if that
    sample was low (at most DEBOUNCE_MAX_DT_S per step), so a steady low
    trips exactly 5 s after its first low sample. A failed read (None)
    changes nothing; a read that always fails never trips. Old low time
    ages out."""

    def __init__(self, need_s=DEBOUNCE_NEED_S, window_s=DEBOUNCE_WINDOW_S):
        self.need_s, self.window_s = need_s, window_s
        self.lows = deque()
        self.last_t = self.last_good = None

    def update(self, now, good):
        if good is not None:
            if self.last_good is False and now > self.last_t:
                self.lows.append((now, min(now - self.last_t, DEBOUNCE_MAX_DT_S)))
            self.last_t, self.last_good = now, good
        while self.lows and self.lows[0][0] <= now - self.window_s:
            self.lows.popleft()

    @property
    def low_s(self):
        return sum(dt for _, dt in self.lows)

    @property
    def tripped(self):
        return self.low_s >= self.need_s - EPS

    def clear(self):
        self.lows.clear()


class LowVoltage:
    """Tier (a): GPIO 6 low and VCELL <= 3.30 V for 5 s. Tier (b): VCELL
    <= 3.10 V on 3 samples in a row, any GPIO 6 state. An unreadable or
    implausible VCELL (None) neither arms nor clears a tier."""

    def __init__(self):
        self.a_since = None
        self.b_count = 0

    def update(self, now, good, vcell):
        if vcell is None:
            return None
        self.b_count = self.b_count + 1 if vcell <= TIER_B_V else 0
        if vcell > TIER_A_V or good is True:
            self.a_since = None
        elif good is False and self.a_since is None:
            self.a_since = now
        if self.b_count >= TIER_B_SAMPLES:
            return "low_voltage_b vcell=%.3f" % vcell
        if good is False and self.a_since is not None and now - self.a_since >= TIER_A_S - EPS:
            return "low_voltage_a vcell=%.3f" % vcell
        return None


class Charge:
    """GPIO 16 policy. Hold off: always disabled. Hold on (branch A): a burst
    starts when the input is good, the cells rested HOLD_REST_S and VCELL <
    HOLD_V; it lasts HOLD_BURST_S and ends at once on an input loss, a gauge
    error or a VCELL below its start value. 3 bad reads in a row: failsafe."""

    def __init__(self, hold, now):
        self.hold = hold
        self.fails = 0
        self.bad_since = now            # unknown until the first good read
        self.rest_since = now
        self.burst = None               # (start vcell, end time)
        self.burst_end = None           # end time of the last burst

    def _end(self, now):
        if self.burst:
            self.burst_end = now
        self.burst = None
        self.rest_since = now

    def update(self, now, good, sampled=False, vcell=None, halting=False):
        if sampled:
            if vcell is None:
                self.fails += 1
                self.bad_since = now if self.bad_since is None else self.bad_since
                self._end(now)
            else:
                self.fails, self.bad_since = 0, None
                if self.burst and vcell < self.burst[0]:
                    self._end(now)
        if good is not True or halting:
            self._end(now)
        elif self.burst and now >= self.burst[1] - EPS:
            self._end(now)
        elif (self.hold and self.burst is None and sampled and vcell is not None
              and now - self.rest_since >= HOLD_REST_S - EPS and vcell < HOLD_V):
            self.burst = (vcell, now + HOLD_BURST_S)
        return self.state

    @property
    def state(self):
        return "failsafe" if self.fails >= GAUGE_FAILSAFE_READS else "burst" if self.burst else "disabled"

    def gauge_fault(self, now):
        return self.bad_since is not None and now - self.bad_since >= GAUGE_FAULT_S - EPS


def _time_ok(v, now):
    """A marker time: a real number (not bool), finite, within [0, now]."""
    try:
        return (isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) and 0.0 <= v <= now + EPS)
    except (OverflowError, ValueError):   # an int too large for a float
        return False


class Halter:
    """Halt state machine: idle -> kill -> wait_stop -> recheck -> ladder
    (input loss), or idle -> ladder at once with --force (low voltage).
    step() performs at most one action per call and returns its name.
    note(now, event) is called for each halt event, after the marker and
    before a poweroff call, so the record reaches the disk first."""

    def __init__(self, ops, note=None):
        self.ops, self.note = ops, note or (lambda now, event: None)
        self.phase, self.kind, self.reason = "idle", None, None
        self.t0 = self.poweroff_at = None
        self.rung, self.deadline, self.next_poll, self.highs = 0, 0.0, 0.0, 0
        self.recheck_t = 0.0

    @property
    def active(self):
        return self.phase != "idle"

    def _marker(self):
        self.ops.write_marker({"kind": self.kind, "reason": self.reason,
                               "start": self.t0, "poweroff_at": self.poweroff_at})

    def start(self, now, kind, reason):
        """True when a halt starts, or an input-loss halt becomes a low-voltage one."""
        if kind == "input_loss":
            if self.active:
                return False
            self.kind, self.reason, self.t0, self.poweroff_at = kind, reason, now, None
            self.phase = "kill"
            self._marker()
            self.note(now, "halt")
            return True
        if self.active and self.kind == "low_voltage":
            return False
        if not self.active:
            self.t0 = now
        if self.phase != "ladder":
            self.poweroff_at, self.rung, self.phase = now, 0, "ladder"
        self.kind, self.reason = kind, reason
        self._marker()
        self.note(now, "halt")
        self.ops.poweroff(["--force"])
        return True

    def resume(self, now, marker):
        """Continue a halt that a previous instance started. A marker whose
        start or poweroff_at is not a finite number in [0, now] (or null for
        poweroff_at) counts as unparsable: an input-loss halt at step 2."""
        if marker is None:
            return
        if not isinstance(marker, dict):
            marker = {}
        pa, t0 = marker.get("poweroff_at"), marker.get("start")
        if not (_time_ok(t0, now) and (pa is None or _time_ok(pa, now))):
            marker, pa, t0 = {}, None, now
        reason = str(marker.get("reason") or "resumed")[:200]
        if pa is not None:
            self.kind = "low_voltage" if marker.get("kind") == "low_voltage" else "input_loss"
            self.reason, self.phase, self.poweroff_at, self.t0 = reason, "ladder", float(pa), float(t0)
            self.rung = max(0, int((now - self.poweroff_at) // LADDER_STEP_S) - 1)
        elif marker.get("kind") == "low_voltage":
            self.start(now, "low_voltage", reason)
        else:
            self.kind, self.reason, self.phase, self.t0 = "input_loss", reason, "kill", float(t0)

    def step(self, now, good):
        if self.phase == "kill":
            self.ops.kill_robot()
            self.phase, self.deadline, self.next_poll = "wait_stop", now + STOP_WAIT_S, now
            self.note(now, "kill")
            return "kill"
        if self.phase == "wait_stop":
            done = now >= self.deadline - EPS
            if not done and now >= self.next_poll - EPS:
                self.next_poll = now + POLL_S
                st = self.ops.robot_state()
                done = st is not None and st not in BUSY
            if done:
                self.phase, self.highs, self.recheck_t = "recheck", 0, now
                self.note(now, "stopped")
                return "stopped"
            return None
        if self.phase == "recheck":
            # Only readable samples decide: 2 highs in a row cancel, the first
            # low powers off. Unreadable for RECHECK_UNREADABLE_S: power off.
            if good is True:
                self.highs += 1
                self.recheck_t = now
                if self.highs >= RECHECK_SAMPLES:
                    self.ops.delete_marker()
                    self.ops.start_robot()
                    self.note(now, "cancel")
                    self.__init__(self.ops, self.note)
                    return "cancel"
                return None
            if good is None and now - self.recheck_t < RECHECK_UNREADABLE_S - EPS:
                return None
            self.poweroff_at, self.rung, self.phase = now, 0, "ladder"
            self._marker()
            self.note(now, "poweroff")
            self.ops.poweroff(["--no-wall"])
            return "poweroff"
        if self.phase == "ladder" and now - self.poweroff_at >= LADDER_STEP_S * (self.rung + 1) - EPS:
            self.rung += 1
            ev = "force" if self.rung == 1 else "force_force"
            self.note(now, ev)
            self.ops.poweroff(["--force"] if ev == "force" else ["--force", "--force"])
            return ev
        return None


def consume_nohalt(path):
    """Count one start against the nohalt file. Returns (active, note).
    Empty file = NOHALT_DEFAULT starts; unparsable = 1; deleted at 0."""
    if os.path.isdir(path):                     # could never expire: ignore it
        return False, "nohalt is a directory: ignored"
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read().strip()
    except FileNotFoundError:
        return False, None
    except Exception as e:  # noqa: BLE001 - decode, permission, I/O: halt stays ON
        return False, "nohalt file unreadable (%s: %s): input-loss halt stays ON" % (
            type(e).__name__, str(e)[:80])
    try:
        n = int(text) if text else NOHALT_DEFAULT
    except ValueError:
        n = 1
    try:
        if n - 1 <= 0:
            os.remove(path)
        else:
            with open(path, "w") as f:
                f.write("%d\n" % (n - 1))
    except Exception as e:  # noqa: BLE001
        return n >= 1, "nohalt file not updated: %s" % e
    return n >= 1, "nohalt: %d start(s) left after this one" % max(n - 1, 0)


def write_atomic(path, obj):
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path + ".tmp", "w") as f:
            json.dump(obj, f, sort_keys=True)
        os.replace(path + ".tmp", path)
        return None
    except (OSError, TypeError, ValueError) as e:
        return str(e)[:120]


class System:
    """The only OS layer. Tools are found on PATH (bench test 7 puts a fake
    systemctl first)."""

    def __init__(self, run_dir="/run/walle-power", state_dir="/var/lib/walle-power"):
        self.status_path = os.path.join(run_dir, "status.json")
        self.marker_path = os.path.join(run_dir, "halting")
        self.nohalt_path = os.path.join(state_dir, "nohalt")
        self.events_path = os.path.join(state_dir, "events.jsonl")

    def event(self, rec):
        """Append one JSON line to the event file, then fsync: it must be on
        the disk before a power-off (the journal of a halted boot is lost on
        this board). Returns None, or the error text. Never raises."""
        try:
            data = (json.dumps(rec, sort_keys=True) + "\n").encode()
            fd = os.open(self.events_path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
            try:
                os.write(fd, data)
                os.fsync(fd)
            finally:
                os.close(fd)
            return None
        except Exception as e:  # noqa: BLE001
            return ("events: %s: %s" % (type(e).__name__, e))[:120]

    def trim_events(self, max_bytes=EVENTS_MAX_BYTES):
        """At a start: a file above max_bytes keeps its newest half (whole
        lines), rewritten atomically. Returns None or the error text."""
        try:
            if os.path.getsize(self.events_path) <= max_bytes:
                return None
            with open(self.events_path, "rb") as f:
                f.seek(-(max_bytes // 2), os.SEEK_END)
                tail = f.read()
            tail = tail[tail.find(b"\n") + 1:]
            tmp = self.events_path + ".tmp"
            with open(tmp, "wb") as f:
                f.write(tail)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.events_path)
            return None
        except FileNotFoundError:
            return None
        except Exception as e:  # noqa: BLE001
            return ("events trim: %s: %s" % (type(e).__name__, e))[:120]

    @staticmethod
    def log(msg, level=6):
        try:
            print("<%d>%s" % (level, msg), flush=True)   # journald reads the <n> priority
        except (OSError, ValueError):                    # a broken or closed stdout
            pass

    @staticmethod
    def boot_id():
        try:
            with open("/proc/sys/kernel/random/boot_id") as f:
                return f.read().strip()
        except OSError:
            return "unknown"

    wall = staticmethod(time.time)
    read_vcell = staticmethod(read_vcell)

    def read_pins(self):
        rc, out = run(["pinctrl", "get", "%d,%d" % (GPIO_IN, GPIO_CHG)])
        pins = {}
        for line in out.splitlines() if rc == 0 else ():
            head = line.split(":", 1)[0].strip()
            if head.isdigit():
                pins[int(head)] = parse_pinctrl_line(line)
        return pins.get(GPIO_IN), pins.get(GPIO_CHG)

    def set_pin(self, pin, func, mode):
        return run(["pinctrl", "set", str(pin), func, mode])[0] == 0

    def ext5v(self):
        rc, out = run(["vcgencmd", "pmic_read_adc", "EXT5V_V"])
        try:
            v = float(out.split("=")[-1].strip().rstrip("V")) if rc == 0 else None
        except ValueError:
            return None
        return v if v is not None and math.isfinite(v) else None

    def eeprom_ok(self):
        rc, out = run(["rpi-eeprom-config"])
        return None if rc != 0 else any(l.replace(" ", "") == "POWER_OFF_ON_HALT=1"
                                        for l in out.splitlines())

    def kill_robot(self):
        run(["systemctl", "kill", "--signal=SIGKILL", ROBOT])
        run(["systemctl", "stop", "--no-block", ROBOT])   # cancels Restart=on-failure

    def robot_state(self):
        rc, out = run(["systemctl", "is-active", ROBOT])
        return None if rc is None else out.strip()

    def start_robot(self):
        run(["systemctl", "start", "--no-block", ROBOT])

    def poweroff(self, args):
        """A child process: the loop keeps running (ladder, tiers, watchdog)."""
        try:
            subprocess.Popen(["systemctl", "poweroff"] + list(args), stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:  # noqa: BLE001
            self.log("systemctl poweroff %s failed: %s" % (" ".join(args), e), 3)

    def read_marker(self):
        try:
            with open(self.marker_path) as f:
                d = json.load(f)
            return d if isinstance(d, dict) else {}
        except FileNotFoundError:
            return None
        except (OSError, ValueError):
            return {}

    def write_marker(self, d):
        err = write_atomic(self.marker_path, d)
        if err:
            self.log("halting marker not written: %s" % err, 3)

    def delete_marker(self):
        try:
            os.remove(self.marker_path)
        except OSError:
            pass

    def write_status(self, d):
        return write_atomic(self.status_path, d)

    @staticmethod
    def notify(msg):
        addr = os.environ.get("NOTIFY_SOCKET")
        if not addr:
            return
        if addr[0] == "@":
            addr = "\0" + addr[1:]
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
                s.settimeout(1.0)
                s.sendto(msg.encode(), addr)
        except OSError:
            pass


class Watcher:
    """One loop pass per tick(). Wires the parts; holds no rule of its own."""

    def __init__(self, ops, now, hold=HOLD_ENABLED):
        self.ops, self.hold, self.boot_id = ops, hold, ops.boot_id()
        self.debounce, self.tiers, self.charge = Debounce(), LowVoltage(), Charge(hold, now)
        self.halter = Halter(ops, self._on_halt)
        self.start_t, self.good, self.pending_tier = now, None, None
        self.next_gauge, self.vcell, self.ext5v, self.last_error = now, None, None, None
        # bus_ok_mono: the last good bus-1 read, or the watcher's start if there
        # was none yet, so a bus stuck from boot still goes stale for the guard.
        self.bus_ok_mono, self.bus_ok_ts = now, None
        self.cells_high, self.eeprom_bad = False, None
        self.prev, self.faults, self.next_warn = {}, set(), now
        self.next_eeprom, self.next_g6_fix = now, now + RETRY_S
        self.want_prev, self.charge_pin_bad, self.pin_mismatch = "dh", False, False
        try:
            self.nohalt, note = consume_nohalt(ops.nohalt_path)
        except Exception as e:  # noqa: BLE001 - nothing here may stop the start
            self.nohalt, note = False, "nohalt check failed (%s): input-loss halt stays ON" % e
        if note:
            ops.log(note, 4)
        for pin, func, mode in ((GPIO_CHG, "op", "dh"), (GPIO_IN, "ip", GPIO_IN_PULL)):
            if not ops.set_pin(pin, func, mode):
                ops.log("pinctrl set %d %s %s failed" % (pin, func, mode), 3)
        try:
            err = ops.trim_events()
        except Exception as e:  # noqa: BLE001 - never fail the start for this file
            err = "events trim: %s" % e
        self.last_error = err or self.last_error
        try:
            marker = ops.read_marker()
        except Exception:  # noqa: BLE001
            marker = {}
        self.record(now, "start", version=VERSION, nohalt=self.nohalt,
                    resumed_marker=marker is not None)
        try:
            self.halter.resume(now, marker)
        except Exception as e:  # noqa: BLE001 - never crash-loop before READY
            ops.log("halting marker unusable (%s): continuing as input loss" % e, 3)
            self.halter = Halter(ops, self._on_halt)
            self.halter.resume(now, {})
        if self.halter.active:
            ops.log("continuing halt from marker: %s (%s)" % (self.halter.reason, self.halter.phase), 4)
            self.record(now, "resume", phase=self.halter.phase, kind=self.halter.kind,
                        reason=self.halter.reason)
        ops.notify("READY=1\nWATCHDOG=1")   # at most 2 bounded calls (6 s) since the start

    # Watchdog budget (WatchdogSec=15, kept below 10 s). Every run() is at most
    # 3 s (2 s timeout + 1 s kill wait), the I2C read about 1 s (kernel bound),
    # Popen and file writes are short. Ping to ping, worst case:
    #   start ping -> read_pins 3 + kill step 6              -> ping 1   = 9 s
    #   ping 1 -> I2C 1 + ext5v 3 (never while halting)     -> ping 2   = 4 s
    #   ping 2 -> set_pin 16: 3 + one retry (EEPROM or GPIO 6 set): 3 -> final = 6 s
    # Start: consume nohalt + 2 x set_pin = 6 s before READY=1 and the first ping.
    def tick(self, now):
        ops = self.ops
        ops.notify("WATCHDOG=1")            # an error late in a tick cannot stretch the gap
        g6, g16 = ops.read_pins()
        good = None if g6 is None or g6["level"] is None else g6["level"] == 1
        self.good = good
        self.debounce.update(now, good)
        # Never START an input-loss halt on a sample that shows the input back.
        if self.debounce.tripped and good is not True and not self.nohalt and not self.halter.active:
            self.halter.start(now, "input_loss", "input_loss low=%.2fs" % self.debounce.low_s)
        action = self.halter.step(now, good)
        if action == "cancel":
            self.debounce.clear()           # else the same low time trips again at once
        ops.notify("WATCHDOG=1")            # the GPIO 6 path never waits behind I2C
        sampled = now >= self.next_gauge - EPS
        if sampled:
            self.next_gauge = now + GAUGE_PERIOD_S
            self.vcell, err = ops.read_vcell()
            if self.vcell is None:
                self.last_error = err
            else:
                self.bus_ok_mono, self.bus_ok_ts = now, ops.wall()
                self._judge_cells(now, good)
            reason = self.tiers.update(now, good, self.vcell)
            if reason:
                self.pending_tier = reason
                if self.halter.start(now, "low_voltage", reason):
                    action = action or "low_voltage"
                self.pending_tier = None
            self.ext5v = None if self.halter.active else ops.ext5v()
            ops.notify("WATCHDOG=1")
        charge = self.charge.update(now, good, sampled, self.vcell, self.halter.active)
        self._drive_charge(charge, g16)
        g6_bad = g6 is None or g6["func"] != "ip" or g6["pull"] != GPIO_IN_PULL
        self._retry_one(now, g6_bad)
        faults = {name for name, on in (
            ("gauge", self.charge.gauge_fault(now)), ("nohalt", self.nohalt),
            ("eeprom", self.eeprom_bad), ("cells_high", self.cells_high),
            ("charge_pin", self.charge_pin_bad), ("gpio6_cfg", g6_bad)) if on}
        state = "halting" if self.halter.active else "debounce" if self.debounce.low_s > 0 else "run"
        self._log_changes(now, good=good, state=state, charge=charge, faults=faults)
        if sampled or action:
            self._status(now, good, charge, state, faults)
        ops.notify("WATCHDOG=1")

    def _retry_one(self, now, g6_bad):
        """At most one slow call per tick, none while halting: the EEPROM
        check until it answers yes, and the GPIO 6 pull when its read-back is
        wrong (each at most once a minute)."""
        if self.halter.active:
            return
        if self.eeprom_bad is not False and now >= self.next_eeprom - EPS:
            self.next_eeprom = now + RETRY_S
            self.eeprom_bad = self.ops.eeprom_ok() is not True   # FAULT until a yes
            return
        if g6_bad and now >= self.next_g6_fix - EPS:
            self.next_g6_fix = now + RETRY_S
            self.ops.set_pin(GPIO_IN, "ip", GPIO_IN_PULL)

    def _drive_charge(self, charge, g16):
        want = "dl" if charge == "burst" else "dh"
        have = g16 is not None and g16["func"] == "op" and g16["drive"] == want
        mismatch = not have and want == self.want_prev
        # FAULT only when the pin did not follow a set on the previous tick.
        self.charge_pin_bad = mismatch and self.pin_mismatch
        if not have and (g16 is not None or want == "dh"):   # never enable blind
            if not self.ops.set_pin(GPIO_CHG, "op", want):
                self.charge_pin_bad = True
        self.want_prev, self.pin_mismatch = want, mismatch

    def record(self, now, event, **fields):
        """One line in the persistent event file. Never raises."""
        rec = {"boot_id": self.boot_id, "bt": round(now, 3), "event": event}
        try:
            rec["wall"] = round(self.ops.wall(), 3)
        except Exception:  # noqa: BLE001
            rec["wall"] = None
        rec.update(fields)
        try:
            err = self.ops.event(rec)
        except Exception as e:  # noqa: BLE001
            err = "events: %s" % e
        if err:
            self.last_error = err

    def _on_halt(self, now, event):
        """Halter hook: journal line and event line, after the marker and
        before any poweroff call."""
        h = self.halter
        v = None if self.vcell is None else round(self.vcell, 4)
        if event == "halt":
            if h.kind == "low_voltage" and self.pending_tier:
                self.record(now, "tier", reason=self.pending_tier, vcell_v=v, input_good=self.good)
            self.record(now, "halt", kind=h.kind, reason=h.reason, vcell_v=v,
                        low_s=round(self.debounce.low_s, 2), input_good=self.good)
            what = "HALT (forced, no cancel)" if h.kind == "low_voltage" else "HALT"
        else:
            self.record(now, event, kind=h.kind, reason=h.reason)
            what = "halt step " + event
        self.ops.log("%s: %s vcell=%s uptime=%.1f" % (
            what, h.reason, "unreadable" if v is None else "%.3f" % v, now), 4)

    def _judge_cells(self, now, good):
        """cells_high follows RESTED samples only: 120 s after the start and
        after a burst (the board charges until GPIO 16 is driven), input good,
        no halt. A notice, logged when it appears and when it clears."""
        be = self.charge.burst_end
        if (good is not True or self.halter.active or self.charge.burst
                or now - self.start_t < CELLS_REST_S - EPS
                or (be is not None and now - be < CELLS_REST_S - EPS)):
            return
        high = self.vcell > CELLS_HIGH_V
        if high != self.cells_high:
            self.ops.log("notice cells_high %s: rested vcell=%.3f" % ("on" if high else "off", self.vcell), 5)
        self.cells_high = high

    def _log_changes(self, now, **cur):
        for key, val in cur.items():
            if key in self.prev and self.prev[key] != val:
                self.ops.log("%s: %s -> %s" % (key, _fmt(self.prev[key]), _fmt(val)), 5)
        self.prev = cur
        real = cur["faults"] - NOTICES
        if real and now >= self.next_warn - EPS:
            self.ops.log("FAULT %s last_error=%s" % (",".join(sorted(real)), self.last_error), 4)
            self.next_warn = now + FAULT_WARN_S

    def _status(self, now, good, charge, state, faults):
        err = self.ops.write_status({
            "schema": "walle-power/1", "boot_id": self.boot_id, "hb_mono": round(now, 3),
            "bus_ok_mono": self.bus_ok_mono, "bus_ok_ts": self.bus_ok_ts, "input_good": good,
            "low_s": round(self.debounce.low_s, 2),
            "vcell_v": None if self.vcell is None else round(self.vcell, 4),
            "ext5v_v": self.ext5v, "charge": charge, "hold": "on" if self.hold else "off",
            "state": state, "halt_reason": self.halter.reason, "faults": sorted(faults),
            "last_error": self.last_error})
        if err:
            self.last_error = "status: " + err


def _fmt(v):
    return (",".join(sorted(v)) or "none") if isinstance(v, set) else v


def main(argv=None):
    ap = argparse.ArgumentParser(description="WALL-E power watcher (Geekworm X1202)")
    ap.add_argument("--hold", action="store_true", default=HOLD_ENABLED,
                    help="enable the branch A charge-hold bursts (default: charging always disabled)")
    a = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    ops = System()
    loop(Watcher(ops, boottime(), hold=a.hold), ops, boottime, time.sleep)


def loop(w, ops, clock, sleep, ticks=None):
    """Run ticks at PERIOD_S. An unexpected error in a pass (tick, clock or
    sleep) is logged (at most once each ERROR_LOG_S) and the next pass runs,
    with no watchdog ping from the error path. MAX_FAILED_TICKS failed passes
    in a row exit with status 1, so systemd starts a fresh process (the
    marker continues a halt). SystemExit and KeyboardInterrupt pass. ticks
    limits the passes (tests)."""
    next_t, next_err, n, failed = None, None, 0, 0
    while ticks is None or n < ticks:
        n += 1
        try:
            if next_t is None:
                next_t = clock()
            w.tick(clock())
            failed = 0
            next_t += PERIOD_S
            delay = next_t - clock()
            if delay < -PERIOD_S:
                next_t, delay = clock(), 0.0
            sleep(max(0.0, delay))
        except Exception as e:  # noqa: BLE001
            failed += 1
            try:
                now = clock()
            except Exception:  # noqa: BLE001
                now = None
            if next_err is None or now is None or now >= next_err:
                next_err = None if now is None else now + ERROR_LOG_S
                ops.log("tick error %s: %s" % (type(e).__name__, str(e)[:200]), 3)
            if failed >= MAX_FAILED_TICKS:
                ops.log("%d failed passes in a row: exit for a fresh start" % failed, 3)
                try:
                    w.record(clock(), "exit_failed_ticks", failed=failed, error=type(e).__name__)
                except Exception:  # noqa: BLE001
                    pass
                sys.exit(1)
            next_t = None
            try:
                sleep(PERIOD_S)
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
