#!/usr/bin/env python3
"""WALL-E UPS watcher for the Waveshare UPS HAT (B).

Replaces the UPSPlus daemon (scripts/upsPlus_power_daemon.py, removed
2026-11-03 when the UPSPlus HAT was retired).

Hardware facts (schematic UPS_HAT_B_Schematic.pdf, read 2026-11-03):
  * The HAT has NO microcontroller and NO charger IC. Its 8.4 V jack is
    wired straight to the 2S 18650 pack node through the HY2120-driven
    protection MOSFETs. Whatever voltage the jack supplies is the voltage
    the pack floats at, so the robot's buck setpoint IS the charge limit
    (7.6 V = 3.80 V/cell, about 50 %, Kevin's choice).
  * The only sensor is an INA219 at I2C 0x42 across a shunt in the BATTERY
    leg. The schematic labels it 0.1R; the readings say 10 mOhm: on the
    bench (2026-11-03, brick unplugged) the shunt showed 5-9 mV while the
    Pi's PMIC rails alone summed to ~2.2 W plus NVMe and USB, which is
    0.5-0.9 A at 7.5 V, not 50-90 mA. Positive current = into the pack.
    With mains on the jack the supply carries the Pi and the pack sees a
    small positive trickle (+40..+150 mA floating at 8.35 V on the bench
    brick, noise +/-40 mA). With mains gone the pack carries the Pi: the
    shunt reads -500 mA or worse (-520..-910 mA measured).
  * The SY8286 buck keeps the 5 V output on until the HY2120 low-voltage
    cutoff. Nothing on the board can switch the output off or signal
    "mains is back". The Pi 5 wakes only from its power button, the RTC
    alarm, or 5 V being removed and re-applied.

Why the rules look the way they do (2026-11-04)
  The HAT does not sense its input jack. Mains is INFERRED from the battery
  current I (+ = into the pack) and the pack voltage V against the supply
  setpoint S (--supply-v). With a CC/CV supply present the pack floats at S
  and I is near zero. The hard case is a pack that sits ABOVE the supply
  (after an 8.4 V top-up, or when a lower supply is plugged in): the supply
  is present but idle, the cells carry the Pi, and (I, V) look exactly like
  a lost supply. Field, 2026-11-04: cells of about 1.1 ohm resting at
  8.27 V on a 7.5 V supply read 7.59-7.70 V at -0.46..-0.56 A, and the
  2026-11-03 rules powered the Pi off four times with the supply connected.
  Design reviewed by Grok twice that day (sessions 01a1a78e and the diff
  review recorded in docs/ups_hat_b.md).

Classification of one sample, the same in both modes:
  present       I >= PRESENT_MA (-80 mA)                      resets the timer
  unknown       -250 < I < -80 mA, an I2C error, or an implausible reading
                (V outside 4-9.5 V, |I| over 5 A, overflow)  holds
  discharging, I <= ABSENT_MA (-250 mA), judged by V against S:
    above_supply  V > S + GATE_MARGIN_V (0.15)   the pack is fuller than the
                                                 float; holds
    absent        V < S - HARD_MARGIN_V (0.30)   a present CC/CV supply holds
                                                 the node at S, so it is gone;
                                                 counts
    discharge     the band between               ambiguous; holds

  The edge rule (--edge-s, OFF by default): with EDGE_S > 0, run mode counts
  a band sample as absent when it comes within EDGE_S of the last present
  sample, because a real pull flips floating to discharging at once. It is
  off because healthy, low-resistance cells make a small supply droop under
  a load step look the same: 40 mV of droop through a 0.07 ohm pack is
  0.57 A out of the cells, right after a floating sample (Grok review
  01a1a7a9). With the rule off, a pull from the float is caught when the
  pack sags below S - 0.30 V (at once on high-resistance cells, after part
  of the buffer on healthy ones) or by the floor. Turn it on only after
  measuring that the supply's droop under load stays well under the
  hard margin.

run mode  (walle-ups.service, root, after boot)
  Polls at 2 Hz. 5 s of counted absent samples halts. The count resets on
  any present sample and after GAP_S (10 s) without a counted sample, so
  scattered blips cannot add up. The hard line has hysteresis: a count
  starts below S - 0.30 V, and while it runs a discharging sample below
  S - 0.30 + HARD_HYST_V (0.10) also counts, so a pack that hovers at the
  line (7.14-7.27 V for 6 min on 2026-11-04) cannot keep restarting it.
  Low battery halts whatever the class, except present:
    V <= FLOOR_V (7.0 V, 3.5 V/cell under load) for FLOOR_S (10 s)
    V <= CRIT_V  (6.4 V) on a single sample
  While the pack carries the Pi above the supply, the floor is the only
  detector, so a held discharge still ends in a clean halt and not in the
  HY2120 cutoff. The halt, when halting is enabled:
    1. persist loss.json, an event line, and chain.json (the loss chain,
       with the halt kind: edge, hard, floor, critical or simulated). An
       open chain keeps its wake count, so power-offs from both modes
       count toward the cap, and a low-battery kind stays until it closes;
    2. SIGKILL + stop wall-e.service. Its camera teardown can take the
       full 90 s TimeoutStopSec, and the RTC alarm must not fire while the
       shutdown is still running or it is consumed and lost;
    3. arm the RTC alarm, clear-write-read-back: +FIRST_ALARM_S (90 s),
       +SLOW_WAKE_S (600 s) after a low-battery halt, and at least +300 s
       when a killed unit is still stopping (stuck in the kernel), so the
       alarm cannot fire during the shutdown and be lost;
    4. sync, `systemctl poweroff`. Still running 30 s later: re-arm and
       retry; after 60 s, `systemctl poweroff --force`.
  It warns when the pack floats more than 0.2 V away from S for a minute:
  --supply-v is wrong.

boot-check mode  (walle-ups-wakecheck.service, early boot, before basic.target)
  About 2.5 s of samples and no history. Errors stay out of the
  denominator; fewer than 4 valid samples, or more than 25 % bad ones,
  fails open. Any present sample boots (and closes the chain). Otherwise,
  on 75 % or more of the valid samples:
    V <= FLOOR_V, whatever the     power off, +SLOW_WAKE_S (600 s); never
      current                      capped. No supply is holding the pack at
                                   S, and a boot's load peak is where the
                                   HY2120 cutoff lands. A present supply
                                   holds the pack at S, so this cannot lock
                                   a live supply out.
    absent                         power off, +WAKE_S (60 s); opens a chain
    discharge, inside any chain    power off, +WAKE_S: under the light
      except a simulated one       early-boot load the pack rebounds into
                                   the band while the outage goes on
    above_supply                   boot, close the chain
  Every power-off in a chain, from either mode, counts. After
  CHAIN_MAX_WAKES (24, about 30 min) the 60 s power-offs stop and the boot
  continues. Run mode then grants GRACE_S (600 s) once, with mains-loss
  halts off and the low-battery halts on, saves it in /run on the boot
  clock (so a restarted watcher keeps it), and closes the chain. A wrong
  --supply-v, or a supply below the pack plugged in mid-outage, cannot
  keep the Pi off for good. Anything else FAILS OPEN: a Pi that powers
  itself off because a sensor is unreadable cannot be debugged. The chain
  counts power-offs, not time: with no coin cell the RTC resets to 0
  whenever 5 V drops (seen 2026-11-04).

  POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0 in the bootloader EEPROM (set
  2026-11-03) make the halt a PMIC standby (~3 mA) that the RTC alarm can
  wake. The HAT keeps 5 V on the Pi, so the RTC keeps running.

Operator rule: when you LOWER the supply (an 8.4 V top-up back to 7.5 V),
change --supply-v in both unit files and reinstall. The pack then rides
down to the new setpoint as above_supply and discharge, which hold.

Status file (/tmp/ups_status.json, schema walle-ups/1): `ts` is the time
of the last SUCCESSFUL INA219 read. bin/i2c_bus_guard.py uses its age as
the "reader can see the bus" witness for stuck-bus detection, so a blind
daemon must look stale. `ts_loop` says the daemon process itself is alive.

Halt override: the file /var/lib/walle-ups/nohalt (persistent and loud:
both modes log it and the dashboard shows HALT DISABLED), or --detect-only
on the command line for a bench run. `walle_ups.py once` prints a reading.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import subprocess
import sys
import time

LOG = logging.getLogger("walle-ups")

SCHEMA = "walle-ups/1"

# --- INA219 ---------------------------------------------------------------
REG_CONFIG = 0x00
REG_SHUNT = 0x01
REG_BUS = 0x02
# Power-on default: 32 V range, PGA /8 (+/-320 mV), 12-bit bus + shunt,
# shunt-and-bus continuous. The daemon reads the raw shunt and bus
# registers, which never need the calibration register (the current
# register does, and reads 0 until it is written).
INA219_CONFIG_DEFAULT = 0x399F
SHUNT_LSB_V = 10e-6
BUS_LSB_V = 4e-3

# --- classification -------------------------------------------------------
PRESENT = "present"
ABSENT = "absent"
DISCHARGE = "discharge"
ABOVE_SUPPLY = "above_supply"
UNKNOWN = "unknown"

# Why a halt happened. The chain kind decides what the boot check may count.
KIND_EDGE = "edge"            # band samples right after a present sample
KIND_HARD = "hard"            # pack well below the supply setpoint
KIND_FLOOR = "floor"          # low battery
KIND_CRIT = "critical"        # very low battery
KIND_SIM = "simulated"        # simulate-loss on the bench
MAINS_KINDS = (KIND_EDGE, KIND_HARD, KIND_SIM)
LOW_KINDS = (KIND_FLOOR, KIND_CRIT)
_SEVERITY = {KIND_CRIT: 2, KIND_FLOOR: 1}  # a chain keeps its most severe kind

# --- defaults (every one is a CLI knob) -----------------------------------
DEFAULT_BUS = 1
DEFAULT_ADDR = 0x42
DEFAULT_SHUNT_OHM = 0.01        # 10 mOhm by measurement; the schematic's "0.1R" label is wrong
DEFAULT_SUPPLY_V = 7.5          # robot supply (Kevin, 2026-11-04); the bench brick is 8.4
DEFAULT_GATE_MARGIN_V = 0.15    # above S + this: the pack is fuller than the float
DEFAULT_HARD_MARGIN_V = 0.30    # below S - this: no CC/CV supply is holding the node
DEFAULT_HARD_HYST_V = 0.10      # while a loss count runs, the hard line moves up by this
DEFAULT_PRESENT_MA = -80.0      # floating noise is +/-40 mA at this scale
DEFAULT_ABSENT_MA = -250.0
DEFAULT_LOSS_S = 5.0
DEFAULT_EDGE_S = 0.0            # OFF: see the module doc. 10 counts band samples for 10 s after present
DEFAULT_GAP_S = 10.0            # the loss count resets after this long without a counted sample
DEFAULT_FLOOR_V = 7.0           # 3.5 V/cell under load
DEFAULT_FLOOR_S = 10.0
DEFAULT_CRIT_V = 6.4            # 3.2 V/cell under load: halt now
DEFAULT_CRIT_S = 0.5            # one sample
DEFAULT_MIN_V = 4.0             # outside 4.0-9.5 V the reading is not a 2S pack: hold
DEFAULT_MAX_V = 9.5
DEFAULT_MAX_ABS_MA = 5000.0
DEFAULT_POLL_S = 0.5
DEFAULT_FIRST_ALARM_S = 90      # outlasts the measured halt (3-8 s) and the wall-e kill
DEFAULT_WAKE_S = 60             # Kevin: "let's do a minute"
DEFAULT_SLOW_WAKE_S = 600       # nearly empty pack: poll less, so the polls do not drain it
DEFAULT_CHAIN_MAX_WAKES = 24    # about 30 min of power-offs, then boot with a grace period
DEFAULT_GRACE_S = 600
DEFAULT_GRACE_FILE = "/run/walle-ups/grace"
DEFAULT_MISMATCH_V = 0.20       # floating this far from S for a minute: --supply-v is wrong
DEFAULT_MISMATCH_S = 60.0
DEFAULT_STATUS_FILE = os.environ.get("UPS_STATUS_FILE", "/tmp/ups_status.json")
DEFAULT_STATE_DIR = "/var/lib/walle-ups"
DEFAULT_RTC = "/sys/class/rtc/rtc0"
DEFAULT_KILL_SERVICES = ("wall-e.service",)
DEFAULT_BOOT_CHECK_S = 2.5
DEFAULT_BOOT_POLL_S = 0.25
BOOT_MIN_VALID = 4
BOOT_MAX_BAD_FRACTION = 0.25
BOOT_MAJORITY = 0.75

# Open-circuit voltage per cell -> approximate state of charge, generic NMC
# 18650 at rest. Indicative only: while the pack floats on the supply it
# reads the supply voltage, and under load it reads low.
_OCV_TABLE = (
    (3.20, 0), (3.40, 3), (3.50, 8), (3.60, 17), (3.65, 25), (3.70, 34),
    (3.75, 42), (3.80, 50), (3.85, 57), (3.92, 68), (4.00, 78), (4.10, 90),
    (4.20, 100),
)


def soc_estimate_pct(batt_v: float, cells: int = 2) -> int:
    """Rough resting state of charge for a `cells`S pack, 0..100."""
    v = batt_v / cells
    if v <= _OCV_TABLE[0][0]:
        return 0
    if v >= _OCV_TABLE[-1][0]:
        return 100
    for (v0, p0), (v1, p1) in zip(_OCV_TABLE, _OCV_TABLE[1:]):
        if v0 <= v <= v1:
            return int(round(p0 + (p1 - p0) * (v - v0) / (v1 - v0)))
    return 0


@dataclasses.dataclass(frozen=True)
class Thresholds:
    supply_v: float = DEFAULT_SUPPLY_V
    gate_margin_v: float = DEFAULT_GATE_MARGIN_V
    hard_margin_v: float = DEFAULT_HARD_MARGIN_V
    hard_hyst_v: float = DEFAULT_HARD_HYST_V
    present_ma: float = DEFAULT_PRESENT_MA
    absent_ma: float = DEFAULT_ABSENT_MA
    loss_s: float = DEFAULT_LOSS_S
    edge_s: float = DEFAULT_EDGE_S
    gap_s: float = DEFAULT_GAP_S
    floor_v: float = DEFAULT_FLOOR_V
    floor_s: float = DEFAULT_FLOOR_S
    crit_v: float = DEFAULT_CRIT_V
    crit_s: float = DEFAULT_CRIT_S
    min_v: float = DEFAULT_MIN_V
    max_v: float = DEFAULT_MAX_V
    max_abs_ma: float = DEFAULT_MAX_ABS_MA


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    batt_v: float
    batt_ma: float
    overflow: bool = False


def plausible(sample: Sample, th: Thresholds) -> bool:
    """A reading a 2S pack can produce. Anything else holds; it never counts."""
    return (not sample.overflow and th.min_v <= sample.batt_v <= th.max_v
            and abs(sample.batt_ma) <= th.max_abs_ma)


def classify(batt_ma: float, batt_v: float, th: Thresholds) -> str:
    """One sample, no history (see the module doc). Pure. The edge rule that
    can turn DISCHARGE into ABSENT lives in Watcher, which has the history."""
    if batt_ma >= th.present_ma:
        return PRESENT
    if batt_ma > th.absent_ma:
        return UNKNOWN
    if batt_v > th.supply_v + th.gate_margin_v:
        return ABOVE_SUPPLY
    if batt_v < th.supply_v - th.hard_margin_v:
        return ABSENT
    return DISCHARGE


class LossTimer:
    """Accumulates counted absent samples. Present resets; everything else
    holds, but a gap longer than `gap_s` since the last counted sample resets."""

    def __init__(self, loss_s: float, poll_s: float, gap_s: float = DEFAULT_GAP_S) -> None:
        self.loss_s = float(loss_s)
        self.poll_s = float(poll_s)
        self.gap_s = float(gap_s)
        self.absent_s = 0.0
        self._last_counted: float | None = None

    def update(self, state: str, now: float | None = None) -> bool:
        if state == PRESENT:
            self.reset()
            return False
        if (self.absent_s > 0 and now is not None and self._last_counted is not None
                and now - self._last_counted > self.gap_s):
            self.reset()  # also before an absent sample: a stale count never completes
        if state == ABSENT:
            self.absent_s += self.poll_s
            self._last_counted = now
        return self.absent_s >= self.loss_s - 1e-9

    def reset(self) -> None:
        self.absent_s = 0.0
        self._last_counted = None

    def running(self, now: float) -> bool:
        """A count is in progress and its last counted sample is within the gap."""
        return (self.absent_s > 0 and self._last_counted is not None
                and now - self._last_counted <= self.gap_s)


class LevelTimer:
    """Time spent at or below a voltage. Present resets, and so does a valid
    reading more than `hysteresis_v` above the level; errors hold."""

    def __init__(self, level_v: float, hold_s: float, poll_s: float,
                 hysteresis_v: float = 0.10) -> None:
        self.level_v = float(level_v)
        self.hold_s = float(hold_s)
        self.poll_s = float(poll_s)
        self.hysteresis_v = float(hysteresis_v)
        self.below_s = 0.0

    def update(self, state: str, batt_v: float | None) -> bool:
        if state == PRESENT:
            self.below_s = 0.0
        elif batt_v is None:
            pass
        elif batt_v <= self.level_v:
            self.below_s += self.poll_s
        elif batt_v > self.level_v + self.hysteresis_v:
            self.below_s = 0.0
        return self.below_s >= self.hold_s - 1e-9

    def reset(self) -> None:
        self.below_s = 0.0


# --- hardware access (all replaceable in tests) ---------------------------
class Ina219:
    """Raw shunt/bus reader. Opens lazily; `reopen()` after a bus recovery."""

    def __init__(self, bus_num: int = DEFAULT_BUS, addr: int = DEFAULT_ADDR,
                 shunt_ohm: float = DEFAULT_SHUNT_OHM) -> None:
        self.bus_num = bus_num
        self.addr = addr
        self.shunt_ohm = float(shunt_ohm)
        self._bus = None

    @property
    def dev_path(self) -> str:
        return f"/dev/i2c-{self.bus_num}"

    def open(self) -> None:
        if self._bus is None:
            import smbus2  # imported here so the pure logic is testable without it
            self._bus = smbus2.SMBus(self.bus_num)

    def close(self) -> None:
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.close()
            except Exception:  # noqa: BLE001 - closing a dead handle must not raise
                pass

    def reopen(self) -> None:
        self.close()
        self.open()

    def _read_u16(self, reg: int) -> int:
        self.open()
        v = self._bus.read_word_data(self.addr, reg)
        return ((v & 0xFF) << 8) | (v >> 8)

    def _write_u16(self, reg: int, value: int) -> None:
        self.open()
        self._bus.write_word_data(self.addr, reg, ((value & 0xFF) << 8) | (value >> 8))

    def configure(self) -> None:
        """Put the chip in its continuous power-on configuration."""
        self._write_u16(REG_CONFIG, INA219_CONFIG_DEFAULT)

    def read(self, now: float | None = None) -> Sample:
        raw_sh = self._read_u16(REG_SHUNT)
        if raw_sh & 0x8000:
            raw_sh -= 0x10000
        raw_bus = self._read_u16(REG_BUS)
        batt_v = (raw_bus >> 3) * BUS_LSB_V
        batt_ma = raw_sh * SHUNT_LSB_V / self.shunt_ohm * 1000.0
        return Sample(ts=time.time() if now is None else now, batt_v=batt_v,
                      batt_ma=batt_ma, overflow=bool(raw_bus & 0x1))


class Rtc:
    """The Pi 5 RTC alarm through sysfs. `+N` is relative to the RTC's own
    counter, so a wrong wall clock or timezone cannot move it."""

    def __init__(self, path: str = DEFAULT_RTC) -> None:
        self.path = path

    def _file(self, name: str) -> str:
        return os.path.join(self.path, name)

    def name(self) -> str:
        with open(self._file("name")) as f:
            return f.read().strip()

    def now(self) -> int:
        with open(self._file("since_epoch")) as f:
            return int(f.read().strip())

    def pending(self) -> int | None:
        with open(self._file("wakealarm")) as f:
            raw = f.read().strip()
        return int(raw) if raw else None

    def clear(self) -> None:
        with open(self._file("wakealarm"), "w") as f:
            f.write("0")

    def arm(self, seconds: int) -> int:
        """Clear, write +seconds, read back; raise unless the alarm is in the
        future. Returns the alarm time in the RTC's epoch."""
        seconds = int(seconds)
        if seconds <= 0:
            raise ValueError("alarm seconds must be positive")
        self.clear()
        with open(self._file("wakealarm"), "w") as f:
            f.write(f"+{seconds}")
        alarm = self.pending()
        now = self.now()
        if alarm is None or alarm <= now:
            raise RuntimeError(
                f"wakealarm read-back {alarm!r} is not in the future (rtc now {now})"
            )
        return alarm


class SystemActions:
    """The three side effects of a loss. `dry_run` logs instead."""

    def __init__(self, dry_run: bool = False) -> None:
        self.dry_run = dry_run

    def _run(self, argv: list[str], timeout: float) -> int:
        if self.dry_run:
            LOG.warning("DRY-RUN: %s", " ".join(argv))
            return 0
        try:
            proc = subprocess.run(argv, timeout=timeout, capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            LOG.error("timed out after %.0fs: %s", timeout, " ".join(argv))
            return -1
        except OSError as error:
            LOG.error("could not run %s: %s", " ".join(argv), error)
            return -1
        if proc.returncode != 0:
            LOG.warning("%s -> rc=%s %s", " ".join(argv), proc.returncode,
                        (proc.stderr or "").strip())
        return proc.returncode

    def kill_service(self, name: str) -> None:
        """SIGKILL the unit's cgroup, then `stop` so systemd records it
        inactive (Restart=on-failure would otherwise relaunch it in 5 s)."""
        self._run(["systemctl", "kill", "--signal=SIGKILL", name], timeout=5)
        self._run(["systemctl", "stop", name], timeout=15)

    def sync(self) -> None:
        if self.dry_run:
            LOG.warning("DRY-RUN: sync")
            return
        os.sync()

    def poweroff(self, force: bool = False) -> int:
        """`force` skips stopping the units (still syncs and unmounts)."""
        argv = ["systemctl", "poweroff", "--no-wall"] + (["--force"] if force else [])
        return self._run(argv, timeout=10)

    def is_active(self, name: str) -> bool:
        """True while the unit is active or still stopping."""
        if self.dry_run:
            return False
        try:
            proc = subprocess.run(["systemctl", "is-active", name], timeout=5,
                                  capture_output=True, text=True)
        except (subprocess.TimeoutExpired, OSError):
            return True  # cannot tell: assume the slow case
        return proc.stdout.strip() in ("active", "activating", "deactivating", "reloading")


class StateDir:
    """Persistent state on the root filesystem (survives the halt)."""

    LOSS = "loss.json"
    CHAIN = "chain.json"
    NOHALT = "nohalt"
    EVENTS = "events.jsonl"

    def __init__(self, path: str = DEFAULT_STATE_DIR) -> None:
        self.path = path

    def _file(self, name: str) -> str:
        return os.path.join(self.path, name)

    def ensure(self) -> None:
        os.makedirs(self.path, mode=0o755, exist_ok=True)

    def nohalt(self) -> bool:
        return os.path.exists(self._file(self.NOHALT))

    def _read_json(self, name: str) -> dict | None:
        try:
            with open(self._file(name)) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return None
        return data if isinstance(data, dict) else None

    def _write_json(self, name: str, record: dict) -> None:
        self.ensure()
        tmp = self._file(name + ".tmp")
        with open(tmp, "w") as f:
            json.dump(record, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self._file(name))

    def read_loss(self) -> dict | None:
        return self._read_json(self.LOSS)

    def write_loss(self, record: dict) -> None:
        self._write_json(self.LOSS, record)

    def read_chain(self) -> dict | None:
        """The open loss chain: {"kind": ..., "wakes": n}. None when closed."""
        chain = self._read_json(self.CHAIN)
        if chain is None:
            return None
        try:
            chain["wakes"] = int(chain.get("wakes", 0))
        except (TypeError, ValueError):
            chain["wakes"] = 0
        chain.setdefault("kind", KIND_HARD)
        return chain

    def write_chain(self, record: dict) -> None:
        self._write_json(self.CHAIN, record)

    def clear_chain(self) -> bool:
        try:
            os.unlink(self._file(self.CHAIN))
            return True
        except FileNotFoundError:
            return False
        except OSError as error:
            LOG.warning("could not clear %s: %s", self.CHAIN, error)
            return False

    def append_event(self, event: str, /, **fields) -> None:
        """One JSON line per event; never raises. `event` is positional-only
        so a field may be called "kind"."""
        try:
            self.ensure()
            record = {"ts": time.time(), "event": event}
            record.update(fields)
            with open(self._file(self.EVENTS), "a") as f:
                f.write(json.dumps(record) + "\n")
        except OSError as error:
            LOG.warning("could not append event %s: %s", event, error)


def boot_clock() -> float:
    """Seconds since this boot. Survives a service restart and ignores the
    wall clock, which jumps when the RTC has reset and NTP corrects it."""
    try:
        return time.clock_gettime(time.CLOCK_BOOTTIME)
    except (AttributeError, OSError):  # not Linux (tests on a Mac)
        return time.monotonic()


def write_grace(path: str, seconds: float, boot_now: float | None = None) -> bool:
    """Boot check -> run mode: mains-loss halts off until boot-clock time
    now + `seconds`. Lives in /run (tmpfs), so it never outlasts this boot,
    and it is not removed when read, so a restarted watcher keeps it."""
    boot_now = boot_clock() if boot_now is None else boot_now
    tmp = f"{path}.tmp.{os.getpid()}"
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(tmp, "w") as f:
            f.write(f"{boot_now + float(seconds):.1f}\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        return True
    except OSError as error:
        LOG.error("could not write the grace file %s: %s", path, error)
        try:
            os.unlink(tmp)
        except OSError:
            pass
        return False


def read_grace(path: str, boot_now: float | None = None) -> float:
    """Seconds of grace left, 0.0 when there is none."""
    boot_now = boot_clock() if boot_now is None else boot_now
    try:
        with open(path) as f:
            expiry = float(f.read().strip())
    except (OSError, ValueError):
        return 0.0
    return max(0.0, expiry - boot_now)


def write_status_file(path: str, payload: dict) -> bool:
    """Atomic write (tmp + rename). Returns False instead of raising."""
    tmp = f"{path}.tmp.{os.getpid()}"
    try:
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
        return True
    except OSError:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        return False


# --- run mode --------------------------------------------------------------
class Watcher:
    """One `step()` per poll. Everything with a side effect is injected."""

    REOPEN_AFTER_ERRORS = 10
    BLIND_WARN_PERIOD_S = 60.0
    HEARTBEAT_S = 600.0
    NOHALT_WARN_PERIOD_S = 300.0
    GRACE_WARN_PERIOD_S = 60.0
    HALT_RETRY_S = 30.0
    STUCK_ALARM_S = 300   # outlasts wall-e's 90 s TimeoutStopSec and a slow shutdown
    MISMATCH_WARN_PERIOD_S = 3600.0

    def __init__(self, sensor, th: Thresholds, actions: SystemActions, rtc: Rtc,
                 state: StateDir, *, status_file: str = DEFAULT_STATUS_FILE,
                 poll_s: float = DEFAULT_POLL_S,
                 first_alarm_s: int = DEFAULT_FIRST_ALARM_S,
                 kill_services=DEFAULT_KILL_SERVICES,
                 detect_only: bool = False,
                 grace_s: float = 0.0, now: float | None = None,
                 clock=None, chain_max_wakes: int = DEFAULT_CHAIN_MAX_WAKES,
                 cap_grace_s: float = DEFAULT_GRACE_S,
                 grace_file: str | None = None,
                 slow_wake_s: int = DEFAULT_SLOW_WAKE_S) -> None:
        """`grace_s`: seconds of grace left from the grace file (read_grace).
        `clock`: the timing clock (boot_clock); every timer, window and the
        grace deadline use it, never the wall clock, which jumps when NTP
        corrects an RTC that reset to 0. `now`, when a test passes it, is
        used for both."""
        self.sensor = sensor
        self.th = th
        self.actions = actions
        self.rtc = rtc
        self.state = state
        self.status_file = status_file
        self.poll_s = float(poll_s)
        self.first_alarm_s = int(first_alarm_s)
        self.slow_wake_s = int(slow_wake_s)
        self.kill_services = tuple(kill_services)
        self.detect_only = detect_only
        self.timer = LossTimer(th.loss_s, poll_s, th.gap_s)
        self.floor = LevelTimer(th.floor_v, th.floor_s, poll_s)
        self.crit = LevelTimer(th.crit_v, th.crit_s, poll_s)
        self.last_sample: Sample | None = None
        self.last_state: str | None = None
        self.last_present_at: float | None = None
        self._run_edge = False
        self.errors_total = 0
        self.errors_run = 0
        self.implausible_total = 0
        self.last_error: str | None = None
        self.halting = False
        self._halt_at = 0.0
        self._halt_retries = 0
        self._halt_alarm_s = int(first_alarm_s)
        self.clock = clock or boot_clock
        self.grace_file = grace_file
        self._mono = self.clock() if now is None else now
        start = self._mono
        self.grace_until = start + grace_s if grace_s and grace_s > 0 else None
        chain = state.read_chain()
        self.chain_open = chain is not None
        if chain is not None and chain["wakes"] >= chain_max_wakes:
            # The boot check let a capped chain through (or failed open on
            # one that ran past the cap): start the grace here, once, and
            # keep it across restarts through the grace file.
            LOG.warning("loss chain at %d power-offs (cap %d): mains-loss halts off for %.0f s; "
                        "the low-battery halts stay on. Check --supply-v and the supply.",
                        chain["wakes"], chain_max_wakes, cap_grace_s)
            self.grace_until = max(self.grace_until or 0.0, start + cap_grace_s)
            saved = bool(grace_file) and write_grace(grace_file, cap_grace_s, boot_now=start)
            state.append_event("chain_cap", wakes=chain["wakes"], kind=chain["kind"],
                               grace_s=cap_grace_s, saved=saved)
            if saved:
                # Otherwise keep the chain: a restarted watcher grants the
                # grace again from it instead of losing it.
                state.clear_chain()
                self.chain_open = False
        self._float_since: float | None = None
        self.supply_mismatch = False
        # The boot clock starts near 0, so "never" is far in the past.
        self._blind_warned_at = -1e18
        self._heartbeat_at = -1e18
        self._nohalt_warned_at = -1e18
        self._grace_warned_at = -1e18
        self._mismatch_warned_at = -1e18
        self._ignored_until = -1e18
        self._status_write_failed = False

    # -- helpers ----------------------------------------------------------
    def halt_enabled(self) -> bool:
        return not (self.detect_only or self.state.nohalt())

    def mode(self) -> str:
        if self.detect_only:
            return "detect-only"
        if self.state.nohalt():
            return "nohalt-flag"
        return "armed"

    def in_grace(self, now: float) -> bool:
        return self.grace_until is not None and now < self.grace_until

    def status(self, now: float, state_name: str) -> dict:
        """`now` is wall time (the bus guard compares `ts` with its own clock)."""
        s = self.last_sample
        return {
            "schema": SCHEMA,
            "ts": s.ts if s else None,
            "ts_loop": now,
            "input": self.last_state,
            "batt_v": round(s.batt_v, 3) if s else None,
            "batt_ma": round(s.batt_ma, 1) if s else None,
            "soc_pct": soc_estimate_pct(s.batt_v) if s else None,
            "overflow": s.overflow if s else None,
            "supply_v": self.th.supply_v,
            "supply_mismatch": self.supply_mismatch,
            "absent_s": round(self.timer.absent_s, 2),
            "loss_s": self.th.loss_s,
            "floor_s": round(self.floor.below_s, 2),
            "floor_v": self.th.floor_v,
            "state": state_name,
            "mode": self.mode(),
            "halt_enabled": self.halt_enabled(),
            "grace_s_left": (round(self.grace_until - self._mono, 1)
                             if self.in_grace(self._mono) else 0.0),
            "chain_open": self.chain_open,
            "errors": self.errors_total,
            "errors_run": self.errors_run,
            "implausible": self.implausible_total,
            "last_error": self.last_error,
            "last_loss": self.state.read_loss(),
            "first_alarm_s": self.first_alarm_s,
        }

    def publish(self, now: float, state_name: str) -> None:
        ok = write_status_file(self.status_file, self.status(now, state_name))
        if not ok and not self._status_write_failed:
            LOG.warning("cannot write %s (will keep trying quietly)", self.status_file)
        self._status_write_failed = not ok

    def _classify_live(self, sample: Sample, now: float) -> str:
        c = classify(sample.batt_ma, sample.batt_v, self.th)
        if c == PRESENT:
            self.last_present_at = now
        elif (c == DISCHARGE and self.timer.running(now)
              and sample.batt_v < self.th.supply_v - self.th.hard_margin_v + self.th.hard_hyst_v):
            # Hysteresis on the hard line: a count only starts below S - 0.30 V,
            # but once it runs, a discharging pack below S - 0.20 V keeps it
            # going. Field 2026-11-04: the pack hovered at 7.14-7.27 V for 6 min
            # and the count kept resetting. A reset (present, or the 10 s gap)
            # ends the hysteresis.
            return ABSENT
        elif (c == DISCHARGE and self.th.edge_s > 0 and self.last_present_at is not None
              and now - self.last_present_at <= self.th.edge_s):
            self._run_edge = True
            return ABSENT
        return c

    def _check_setpoint(self, sample: Sample, state: str, now: float) -> None:
        """Floating on the supply, the pack reads the supply's voltage."""
        floating = state == PRESENT and abs(sample.batt_ma) < abs(self.th.present_ma)
        if not floating:
            self._float_since = None
            return
        if self._float_since is None:
            self._float_since = now
            return
        if now - self._float_since < DEFAULT_MISMATCH_S:
            return
        off = abs(sample.batt_v - self.th.supply_v) > DEFAULT_MISMATCH_V
        if off and now - self._mismatch_warned_at >= self.MISMATCH_WARN_PERIOD_S:
            self._mismatch_warned_at = now
            LOG.warning("SUPPLY SETPOINT MISMATCH: --supply-v is %.2f V but the pack has "
                        "rested at %.3f V with no current for %.0f s. With the HAT on and the "
                        "supply connected this means --supply-v is wrong: fix it in both unit "
                        "files and reinstall; the loss rules depend on it. (With the HAT "
                        "switched off the pack just rests; ignore this.)",
                        self.th.supply_v, sample.batt_v, now - self._float_since)
        self.supply_mismatch = off

    # -- the poll -----------------------------------------------------------
    def step(self, now: float | None = None) -> str:
        """One poll. `now` (tests only) stands in for both clocks; otherwise
        the wall clock stamps the status and the boot clock, read after the
        sensor, drives every timer."""
        wall = time.time() if now is None else now
        if self.halting:
            mono = self.clock() if now is None else now
            self._mono = mono
            if mono - self._halt_at >= self.HALT_RETRY_S * (self._halt_retries + 1):
                # A working poweroff stops this process within seconds. Still
                # here: the request failed or hung. Re-arm, then force it.
                self._halt_retries += 1
                force = self._halt_retries >= 2
                LOG.error("still running %.0f s after the poweroff request; retry %d%s",
                          mono - self._halt_at, self._halt_retries, " (forced)" if force else "")
                self.state.append_event("poweroff_retry", retry=self._halt_retries, force=force)
                self._arm_alarm(self._halt_alarm_s)
                self.actions.sync()
                self.actions.poweroff(force=force)
            self.publish(wall, "shutdown")
            return "shutdown"
        batt_v: float | None = None
        read_error: Exception | None = None
        sample: Sample | None = None
        try:
            sample = self.sensor.read(wall)
        except Exception as error:  # noqa: BLE001 - any bus failure is "unknown"
            read_error = error
        mono = self.clock() if now is None else now
        self._mono = mono

        if read_error is not None:
            self.errors_total += 1
            self.errors_run += 1
            self.last_error = f"{type(read_error).__name__}: {read_error}"
            state = UNKNOWN
            if self.errors_run == self.REOPEN_AFTER_ERRORS:
                LOG.warning("%d consecutive INA219 read errors; reopening the bus (%s)",
                            self.errors_run, self.last_error)
                try:
                    self.sensor.reopen()
                    self.sensor.configure()
                except Exception as reopen_error:  # noqa: BLE001
                    LOG.warning("reopen failed: %s", reopen_error)
            elif mono - self._blind_warned_at >= self.BLIND_WARN_PERIOD_S:
                self._blind_warned_at = mono
                LOG.warning("INA219 unreadable (%d in a row): %s", self.errors_run,
                            self.last_error)
        else:
            if self.errors_run:
                LOG.info("INA219 readable again after %d errors", self.errors_run)
            self.errors_run = 0
            self.last_sample = sample
            if not plausible(sample, self.th):
                self.implausible_total += 1
                self.last_error = (f"implausible reading {sample.batt_v:.3f} V "
                                   f"{sample.batt_ma:+.0f} mA (overflow={sample.overflow})")
                state = UNKNOWN
                if mono - self._blind_warned_at >= self.BLIND_WARN_PERIOD_S:
                    self._blind_warned_at = mono
                    LOG.warning("INA219 %s; holding", self.last_error)
            else:
                batt_v = sample.batt_v
                state = self._classify_live(sample, mono)
                self._check_setpoint(sample, state, mono)
            if state != self.last_state:
                LOG.info("input %s -> %s (batt %.3f V %+.0f mA, soc~%d%%)",
                         self.last_state, state, sample.batt_v, sample.batt_ma,
                         soc_estimate_pct(sample.batt_v))
            self.last_state = state

        triggered = self.timer.update(state, mono)
        if self.timer.absent_s == 0.0:
            self._run_edge = False
        low = self.floor.update(state, batt_v)
        critical = self.crit.update(state, batt_v)
        if state in (PRESENT, ABOVE_SUPPLY) and self.chain_open:
            if self.state.clear_chain():
                LOG.info("loss chain closed (%s)", state)
            self.chain_open = False

        if not self.halt_enabled() and mono - self._nohalt_warned_at >= self.NOHALT_WARN_PERIOD_S:
            self._nohalt_warned_at = mono
            LOG.warning("HALT DISABLED (%s): a real power loss will NOT shut the Pi down",
                        self.mode())
        if mono - self._heartbeat_at >= self.HEARTBEAT_S and self.last_sample:
            self._heartbeat_at = mono
            LOG.info("heartbeat: input=%s batt %.3f V %+.0f mA absent_s=%.1f floor_s=%.1f "
                     "errors=%d", state, self.last_sample.batt_v, self.last_sample.batt_ma,
                     self.timer.absent_s, self.floor.below_s, self.errors_total)

        if (critical or low or triggered) and not self.halt_enabled() \
                and mono < self._ignored_until:
            # Halting is off and this was already logged: keep counting quietly.
            self.publish(wall, "loss-ignored")
            return "loss-ignored"
        if critical:
            return self._on_loss(wall, kind=KIND_CRIT)
        if low:
            return self._on_loss(wall, kind=KIND_FLOOR)
        if triggered:
            if self.in_grace(mono):
                if mono - self._grace_warned_at >= self.GRACE_WARN_PERIOD_S:
                    self._grace_warned_at = mono
                    LOG.warning("mains loss ignored: grace period after a capped loss chain "
                                "(%.0f s left); the low-battery halts stay on",
                                self.grace_until - mono)
                self.timer.reset()
                self._run_edge = False
                self.publish(wall, "grace")
                return "grace"
            return self._on_loss(wall, kind=KIND_EDGE if self._run_edge else KIND_HARD)
        state_name = "countdown" if (self.timer.absent_s > 0 or self.floor.below_s > 0) \
            else "monitoring"
        self.publish(wall, state_name)
        return state_name

    # -- the loss -----------------------------------------------------------
    def _on_loss(self, now: float, *, kind: str = KIND_HARD, simulated: bool = False) -> str:
        s = self.last_sample
        reason = "mains" if kind in MAINS_KINDS else "low_battery"
        record = {
            "ts": now,
            "batt_v": round(s.batt_v, 3) if s else None,
            "batt_ma": round(s.batt_ma, 1) if s else None,
            "supply_v": self.th.supply_v,
            "kind": kind,
            "reason": reason,
            "halt_enabled": self.halt_enabled(),
            "mode": self.mode(),
            "simulated": simulated,
        }
        if reason == "mains":
            what = (f"MAINS LOST ({kind}): {self.timer.absent_s:.1f} s of absent samples "
                    f"(batt {record['batt_v']} V {record['batt_ma']} mA).")
        else:
            what = (f"LOW BATTERY ({kind}): pack {record['batt_v']} V at "
                    f"{record['batt_ma']} mA.")
        LOG.warning("%s%s %s", "SIMULATED " if simulated else "", what,
                    "Shutting down." if self.halt_enabled() else
                    f"HALT DISABLED ({self.mode()}), staying up.")
        self.state.append_event("loss", **record)
        try:
            self.state.write_loss(record)
        except OSError as error:
            LOG.error("could not persist loss.json: %s", error)

        if not self.halt_enabled():
            self.timer.reset()
            self.floor.reset()
            self.crit.reset()
            self._run_edge = False
            self._ignored_until = self._mono + 60.0
            self.publish(now, "loss-ignored")
            return "loss-ignored"

        self.halting = True
        try:
            # An outage can alternate boot-check power-offs, fail-open boots
            # and run-mode halts; all of them count toward the cap. The most
            # severe kind (critical, then floor) stays until the chain closes.
            prev = self.state.read_chain()
            chain_kind = kind
            if prev and _SEVERITY.get(prev.get("kind"), 0) > _SEVERITY.get(kind, 0):
                chain_kind = prev["kind"]
            self.state.write_chain({            # "wakes" counts power-offs, both modes
                "kind": chain_kind,
                "wakes": prev["wakes"] + 1 if prev else 1,
                "opened": prev.get("opened") if prev else now})
            self.chain_open = True
        except OSError as error:
            LOG.error("could not persist chain.json: %s", error)
        self.publish(now, "shutdown")
        for name in self.kill_services:
            LOG.warning("killing %s before the halt", name)
            self.actions.kill_service(name)
        # A low pack polls slowly so the polls do not finish it. A unit that
        # survived the kill (stuck in the kernel) can stretch the shutdown past
        # the first alarm, which would then fire early and be lost.
        alarm_s = self.slow_wake_s if kind in LOW_KINDS else self.first_alarm_s
        stuck = [n for n in self.kill_services if self.actions.is_active(n)]
        if stuck:
            alarm_s = max(alarm_s, self.STUCK_ALARM_S)
            LOG.warning("%s still active after the kill; alarm +%d s", ", ".join(stuck), alarm_s)
        self._halt_alarm_s = alarm_s
        self._halt_at = self._mono
        self._arm_alarm(alarm_s)
        self.actions.sync()
        LOG.warning("systemctl poweroff")
        self.actions.poweroff()
        return "shutdown"

    def _arm_alarm(self, seconds: int) -> None:
        try:
            alarm = self.rtc.arm(seconds)
            LOG.warning("RTC alarm armed: +%d s (rtc epoch %d)", seconds, alarm)
            self.state.append_event("alarm", seconds=seconds, alarm=alarm)
        except Exception as error:  # noqa: BLE001
            LOG.error("could not arm the RTC alarm (%s); powering off anyway. "
                      "Recovery: the power button, or the pack cutoff followed by mains.",
                      error)
            self.state.append_event("alarm_failed", error=str(error))


def run_loop(w: Watcher) -> None:
    try:
        w.sensor.open()
        w.sensor.configure()
    except Exception as error:  # noqa: BLE001 - keep polling; errors become "unknown"
        LOG.warning("INA219 not ready at start (%s); polling anyway", error)
    try:
        w.rtc.clear()
    except OSError as error:
        LOG.warning("could not clear a stale RTC alarm: %s", error)
    th = w.th
    edge = (f"edge rule {th.edge_s:.0f} s" if th.edge_s > 0 else "edge rule off")
    LOG.warning("walle-ups watching INA219 0x%02x on bus %d: supply %.2f V, present >= %.0f mA, "
                "discharging <= %.0f mA, absent below %.2f V (%s), above_supply over %.2f V, "
                "loss after %.1f s, floor %.2f V for %.1f s, critical %.2f V for %.1f s, "
                "first alarm +%d s (low battery +%d s), mode %s",
                w.sensor.addr, w.sensor.bus_num, th.supply_v, th.present_ma, th.absent_ma,
                th.supply_v - th.hard_margin_v, edge, th.supply_v + th.gate_margin_v,
                th.loss_s, th.floor_v, th.floor_s, th.crit_v, th.crit_s, w.first_alarm_s,
                w.slow_wake_s, w.mode())
    if w.in_grace(w._mono):
        LOG.warning("GRACE: mains-loss halts are off for %.0f s more (a capped loss chain let "
                    "this boot through); the low-battery halts stay on", w.grace_until - w._mono)
    while True:
        started = time.monotonic()
        result = w.step()
        if result == "shutdown":
            time.sleep(600)
            continue
        elapsed = time.monotonic() - started
        time.sleep(max(0.05, w.poll_s - elapsed))


# --- boot-check mode -------------------------------------------------------
def wait_for_path(path: str, timeout_s: float, sleep=time.sleep, exists=os.path.exists) -> bool:
    deadline = time.monotonic() + timeout_s
    while not exists(path):
        if time.monotonic() >= deadline:
            return False
        sleep(0.1)
    return True


def boot_check(sensor, th: Thresholds, actions: SystemActions, rtc: Rtc, state: StateDir,
               *, duration_s: float = DEFAULT_BOOT_CHECK_S, poll_s: float = DEFAULT_BOOT_POLL_S,
               wake_s: int = DEFAULT_WAKE_S, slow_wake_s: int = DEFAULT_SLOW_WAKE_S,
               chain_max_wakes: int = DEFAULT_CHAIN_MAX_WAKES,
               now_fn=time.time, sleep=time.sleep, dev_wait_s: float = 5.0) -> str:
    """Returns the decision: "present", "nohalt", "inconclusive", "cap" or
    "poweroff". Only "poweroff" has a side effect on the power state."""
    if state.nohalt():
        LOG.warning("boot-check: nohalt flag present (%s); continuing the boot",
                    os.path.join(state.path, StateDir.NOHALT))
        state.append_event("boot_check", decision="nohalt")
        return "nohalt"

    dev = getattr(sensor, "dev_path", None)
    if dev and not wait_for_path(dev, dev_wait_s, sleep=sleep):
        LOG.warning("boot-check: %s never appeared; continuing the boot (fail open)", dev)
        state.append_event("boot_check", decision="inconclusive", reason="no-device")
        return "inconclusive"

    valid: list[Sample] = []
    classes: list[str] = []
    errors = 0
    implausible = 0
    deadline = now_fn() + duration_s
    try:
        sensor.open()
        sensor.configure()
    except Exception as error:  # noqa: BLE001
        LOG.warning("boot-check: INA219 not ready (%s)", error)
    while True:
        now = now_fn()
        try:
            s = sensor.read(now)
        except Exception as error:  # noqa: BLE001
            errors += 1
            LOG.info("boot-check: read error %s", error)
        else:
            if plausible(s, th):
                valid.append(s)
                classes.append(classify(s.batt_ma, s.batt_v, th))
            else:
                implausible += 1
        if now_fn() >= deadline:
            break
        sleep(poll_s)

    chain = state.read_chain()
    n = len(valid)
    low_n = sum(1 for s, c in zip(valid, classes) if c != PRESENT and s.batt_v <= th.floor_v)
    summary = {
        "n": n, "errors": errors, "implausible": implausible,
        "present": classes.count(PRESENT), "absent": classes.count(ABSENT),
        "discharge": classes.count(DISCHARGE), "above": classes.count(ABOVE_SUPPLY),
        "unknown": classes.count(UNKNOWN), "low": low_n,
        "chain": chain["kind"] if chain else None,
        "chain_wakes": chain["wakes"] if chain else None,
        "batt_v": round(valid[-1].batt_v, 3) if valid else None,
        "batt_ma": round(valid[-1].batt_ma, 1) if valid else None,
    }

    def fail_open(reason: str) -> str:
        LOG.warning("boot-check: inconclusive (%s, %s); continuing the boot (fail open)",
                    reason, summary)
        state.append_event("boot_check", decision="inconclusive", reason=reason, **summary)
        return "inconclusive"

    if summary["present"]:
        if chain and state.clear_chain():
            LOG.info("boot-check: loss chain closed")
        LOG.warning("boot-check: mains present (%s); continuing the boot", summary)
        state.append_event("boot_check", decision="present", **summary)
        return "present"
    bad = errors + implausible
    if n < BOOT_MIN_VALID or bad > BOOT_MAX_BAD_FRACTION * (n + bad):
        return fail_open("too few valid samples")
    need = BOOT_MAJORITY * n
    if summary["above"] >= need:
        if chain:
            state.clear_chain()
        return fail_open("pack above the supply")

    chain_kind = chain["kind"] if chain else None
    if low_n >= need:
        # At or below the floor at boot, whatever the current: no supply is
        # holding the pack at S. Never boot it on a cap (a boot's load peak
        # is where the HY2120 cutoff lands) and poll slowly. A present supply
        # holds the pack at S, so this cannot lock a live supply out.
        kind, seconds, capped = (chain_kind or KIND_FLOOR), slow_wake_s, False
    elif summary["absent"] >= need:
        kind, seconds, capped = (chain_kind or KIND_HARD), wake_s, True
    elif (chain_kind and chain_kind != KIND_SIM
          and summary["absent"] + summary["discharge"] >= need):
        # A real halt opened this chain. Under the light early-boot load the
        # pack rebounds into the band, so the band is the outage still going.
        # The cap bounds the case where a supply below the pack came back.
        kind, seconds, capped = chain_kind, wake_s, True
    else:
        return fail_open("no clear loss")

    wakes = chain["wakes"] if chain else 0
    if capped and wakes >= chain_max_wakes:
        # Run mode sees the capped chain at start, grants the grace, writes
        # the grace file and closes the chain (Watcher.__init__).
        LOG.warning("boot-check: loss chain capped after %d power-offs (%s); continuing the boot. "
                    "Run mode starts with a grace for mains-loss halts. Check --supply-v and "
                    "the supply.", wakes, summary)
        state.append_event("boot_check", decision="cap", **summary)
        return "cap"

    LOG.warning("boot-check: mains still absent (%s); re-arming +%d s and powering off",
                summary, seconds)
    try:
        state.write_chain({"kind": kind, "wakes": wakes + 1,
                           "opened": chain.get("opened") if chain else now_fn()})
    except OSError as error:
        LOG.error("boot-check: could not persist chain.json: %s", error)
    state.append_event("boot_check", decision="poweroff", wake_s=seconds, **summary)
    try:
        alarm = rtc.arm(seconds)
        state.append_event("alarm", seconds=seconds, alarm=alarm, source="boot-check")
    except Exception as error:  # noqa: BLE001
        LOG.error("boot-check: could not arm the RTC alarm (%s); powering off anyway", error)
        state.append_event("alarm_failed", error=str(error), source="boot-check")
    actions.sync()
    actions.poweroff()
    return "poweroff"


# --- CLI -------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="WALL-E UPS watcher for the Waveshare UPS HAT (B)")
    p.add_argument("mode", choices=("run", "boot-check", "once", "simulate-loss"),
                   help="run = the daemon; boot-check = the early-boot re-check; "
                        "once = print one reading and exit; simulate-loss = run the "
                        "real halt sequence now (kill, alarm, poweroff) as a bench test "
                        "of the wake chain with mains still connected; needs --yes")
    p.add_argument("--yes", action="store_true",
                   help="confirm simulate-loss (it powers the Pi off)")
    p.add_argument("--bus", type=int, default=DEFAULT_BUS)
    p.add_argument("--addr", type=lambda s: int(s, 0), default=DEFAULT_ADDR)
    p.add_argument("--shunt-ohm", type=float, default=DEFAULT_SHUNT_OHM, dest="shunt_ohm")
    p.add_argument("--supply-v", type=float, default=DEFAULT_SUPPLY_V, dest="supply_v",
                   help="the supply setpoint the pack floats at (robot 7.5, bench brick 8.4)")
    p.add_argument("--gate-margin-v", type=float, default=DEFAULT_GATE_MARGIN_V, dest="gate_margin_v")
    p.add_argument("--hard-margin-v", type=float, default=DEFAULT_HARD_MARGIN_V, dest="hard_margin_v")
    p.add_argument("--hard-hyst-v", type=float, default=DEFAULT_HARD_HYST_V, dest="hard_hyst_v")
    p.add_argument("--present-ma", type=float, default=DEFAULT_PRESENT_MA, dest="present_ma")
    p.add_argument("--absent-ma", type=float, default=DEFAULT_ABSENT_MA, dest="absent_ma")
    p.add_argument("--loss-s", type=float, default=DEFAULT_LOSS_S, dest="loss_s")
    p.add_argument("--edge-s", type=float, default=DEFAULT_EDGE_S, dest="edge_s")
    p.add_argument("--gap-s", type=float, default=DEFAULT_GAP_S, dest="gap_s")
    p.add_argument("--floor-v", type=float, default=DEFAULT_FLOOR_V, dest="floor_v")
    p.add_argument("--floor-s", type=float, default=DEFAULT_FLOOR_S, dest="floor_s")
    p.add_argument("--crit-v", type=float, default=DEFAULT_CRIT_V, dest="crit_v")
    p.add_argument("--crit-s", type=float, default=DEFAULT_CRIT_S, dest="crit_s")
    p.add_argument("--poll-s", type=float, default=DEFAULT_POLL_S, dest="poll_s")
    p.add_argument("--first-alarm-s", type=int, default=DEFAULT_FIRST_ALARM_S, dest="first_alarm_s")
    p.add_argument("--wake-s", type=int, default=DEFAULT_WAKE_S, dest="wake_s")
    p.add_argument("--slow-wake-s", type=int, default=DEFAULT_SLOW_WAKE_S, dest="slow_wake_s")
    p.add_argument("--chain-max-wakes", type=int, default=DEFAULT_CHAIN_MAX_WAKES,
                   dest="chain_max_wakes")
    p.add_argument("--grace-s", type=float, default=DEFAULT_GRACE_S, dest="grace_s")
    p.add_argument("--grace-file", default=DEFAULT_GRACE_FILE, dest="grace_file")
    p.add_argument("--status-file", default=DEFAULT_STATUS_FILE, dest="status_file")
    p.add_argument("--state-dir", default=DEFAULT_STATE_DIR, dest="state_dir")
    p.add_argument("--rtc", default=DEFAULT_RTC)
    p.add_argument("--kill-service", action="append", dest="kill_services",
                   help="unit to SIGKILL before the halt (repeatable; default wall-e.service)")
    p.add_argument("--detect-only", action="store_true", dest="detect_only",
                   help="never halt or arm anything; log what would happen")
    p.add_argument("--verbose", action="store_true")
    return p


def thresholds_from_args(args) -> Thresholds:
    return Thresholds(
        supply_v=args.supply_v, gate_margin_v=args.gate_margin_v,
        hard_margin_v=args.hard_margin_v, hard_hyst_v=args.hard_hyst_v, present_ma=args.present_ma,
        absent_ma=args.absent_ma, loss_s=args.loss_s, edge_s=args.edge_s, gap_s=args.gap_s,
        floor_v=args.floor_v, floor_s=args.floor_s, crit_v=args.crit_v, crit_s=args.crit_s)


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s: %(message)s", stream=sys.stderr)
    th = thresholds_from_args(args)
    sensor = Ina219(args.bus, args.addr, args.shunt_ohm)
    actions = SystemActions(dry_run=args.detect_only)
    rtc = Rtc(args.rtc)
    state = StateDir(args.state_dir)
    kill = tuple(args.kill_services) if args.kill_services else DEFAULT_KILL_SERVICES

    if args.mode == "once":
        sensor.open()
        s = sensor.read()
        print(json.dumps({
            "batt_v": round(s.batt_v, 3), "batt_ma": round(s.batt_ma, 1),
            "soc_pct": soc_estimate_pct(s.batt_v), "overflow": s.overflow,
            "plausible": plausible(s, th),
            "input": classify(s.batt_ma, s.batt_v, th) if plausible(s, th) else UNKNOWN,
            "supply_v": th.supply_v, "nohalt_flag": state.nohalt(),
            "chain": state.read_chain(),
        }, indent=2))
        return 0

    if args.mode == "boot-check":
        boot_check(sensor, th, actions, rtc, state, wake_s=args.wake_s,
                   slow_wake_s=args.slow_wake_s, chain_max_wakes=args.chain_max_wakes)
        return 0

    grace = read_grace(args.grace_file) if args.mode == "run" else 0.0
    w = Watcher(sensor, th, actions, rtc, state, status_file=args.status_file,
                poll_s=args.poll_s, first_alarm_s=args.first_alarm_s,
                kill_services=kill, detect_only=args.detect_only,
                grace_s=grace or 0.0, chain_max_wakes=args.chain_max_wakes,
                cap_grace_s=args.grace_s, slow_wake_s=args.slow_wake_s,
                grace_file=args.grace_file if args.mode == "run" else None)

    if args.mode == "simulate-loss":
        if not args.yes:
            print("simulate-loss powers the Pi off; add --yes to confirm", file=sys.stderr)
            return 2
        # Same code path as a real loss, with a real reading in the record.
        # The running walle-ups.service keeps publishing; this process just
        # fires the sequence once. With mains connected the wake check will
        # see "present" and the Pi boots back fully after the first alarm.
        try:
            w.last_sample = sensor.read()
            w.last_state = classify(w.last_sample.batt_ma, w.last_sample.batt_v, th)
        except Exception as error:  # noqa: BLE001
            LOG.warning("simulate-loss: sensor read failed (%s); continuing", error)
        w.timer.absent_s = th.loss_s
        w._on_loss(time.time(), kind=KIND_SIM, simulated=True)
        time.sleep(30)
        return 0

    run_loop(w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
