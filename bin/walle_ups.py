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

Design (Grok-reviewed 2026-11-03, Grok session 01a1a410-deab-79f3-a336-30eea7088807):

  run mode  (walle-ups.service, root, after boot)
    Poll the INA219 at 2 Hz and classify every sample:
      present       batt_ma >= PRESENT_MA (-80)                  -> loss timer := 0
      absent        batt_ma <= ABSENT_MA (-250) and
                    batt_v  <= supply_v + GATE_MARGIN_V (0.15)   -> loss timer += poll
      above_supply  batt_ma <= ABSENT_MA but the pack is above the supply
                    setpoint. The pack is simply higher than the charger
                    (the day the buck is turned down from 8.4 V). Not a
                    loss; the timer holds. The Pi rides the pack down.
      unknown       the middle band, or an I2C error             -> timer holds
    The timer counts absent SAMPLES (5 s worth), not wall-clock time, so a
    stalled bus cannot jump it. When it expires and halting is enabled:
      1. persist loss.json and an event line (NVMe, survives the halt);
      2. SIGKILL + stop wall-e.service. Its camera teardown can take the
         full 90 s TimeoutStopSec, and the RTC alarm must not fire while
         the shutdown is still running or it is consumed and lost;
      3. arm the RTC alarm (+FIRST_ALARM_S), clear-write-read-back;
      4. sync, `systemctl poweroff`.

  boot-check mode  (walle-ups-wakecheck.service, early boot, before basic.target)
    Read the sensor for ~2.5 s. Any present sample -> continue the boot.
    A clear majority of absent samples and no present ones -> re-arm the
    alarm (+WAKE_S) and power off again (about 8 s of boot per check).
    Anything else (sensor missing, I2C errors, middle band, pack above the
    supply) FAILS OPEN: the boot continues and run mode takes over. A Pi
    that powers itself off because a sensor is unreadable cannot be
    debugged; Grok argued for fail-closed here and that trade was rejected.

  POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0 in the bootloader EEPROM (set
  2026-11-03) make the halt a PMIC standby (~3 mA) that the RTC alarm can
  wake. The HAT keeps 5 V on the Pi, so the RTC keeps running without a
  coin cell.

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
ABOVE_SUPPLY = "above_supply"
UNKNOWN = "unknown"

# --- defaults (every one is a CLI knob) -----------------------------------
DEFAULT_BUS = 1
DEFAULT_ADDR = 0x42
DEFAULT_SHUNT_OHM = 0.01        # 10 mOhm by measurement; the schematic's "0.1R" label is wrong
DEFAULT_SUPPLY_V = 8.4          # bench brick today; 7.6 once the robot buck is in
DEFAULT_GATE_MARGIN_V = 0.15
DEFAULT_PRESENT_MA = -80.0      # floating noise is +/-40 mA at this scale
DEFAULT_ABSENT_MA = -250.0
DEFAULT_LOSS_S = 5.0
DEFAULT_POLL_S = 0.5
DEFAULT_FIRST_ALARM_S = 90      # must outlast the measured shutdown; bench 2026-11-03 pending
DEFAULT_WAKE_S = 60             # Kevin: "let's do a minute"
DEFAULT_WAKE_FAST_S = 60
DEFAULT_WAKE_FAST_WINDOW_S = 600
DEFAULT_STATUS_FILE = os.environ.get("UPS_STATUS_FILE", "/tmp/ups_status.json")
DEFAULT_STATE_DIR = "/var/lib/walle-ups"
DEFAULT_RTC = "/sys/class/rtc/rtc0"
DEFAULT_KILL_SERVICES = ("wall-e.service",)
DEFAULT_BOOT_CHECK_S = 2.5
DEFAULT_BOOT_POLL_S = 0.25

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
    present_ma: float = DEFAULT_PRESENT_MA
    absent_ma: float = DEFAULT_ABSENT_MA
    loss_s: float = DEFAULT_LOSS_S


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    batt_v: float
    batt_ma: float
    overflow: bool = False


def classify(batt_ma: float, batt_v: float, th: Thresholds) -> str:
    """Three bands plus the above-supply gate. Pure; see the module doc."""
    if batt_ma >= th.present_ma:
        return PRESENT
    if batt_ma <= th.absent_ma:
        if batt_v > th.supply_v + th.gate_margin_v:
            return ABOVE_SUPPLY
        return ABSENT
    return UNKNOWN


class LossTimer:
    """Accumulates absent samples. Present resets; unknown/above hold."""

    def __init__(self, loss_s: float, poll_s: float) -> None:
        self.loss_s = float(loss_s)
        self.poll_s = float(poll_s)
        self.absent_s = 0.0

    def update(self, state: str) -> bool:
        if state == PRESENT:
            self.absent_s = 0.0
        elif state == ABSENT:
            self.absent_s += self.poll_s
        return self.absent_s >= self.loss_s

    def reset(self) -> None:
        self.absent_s = 0.0


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

    def poweroff(self) -> None:
        self._run(["systemctl", "poweroff", "--no-wall"], timeout=10)


class StateDir:
    """Persistent state on the root filesystem (survives the halt)."""

    LOSS = "loss.json"
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

    def read_loss(self) -> dict | None:
        try:
            with open(self._file(self.LOSS)) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return None
        return data if isinstance(data, dict) else None

    def write_loss(self, record: dict) -> None:
        self.ensure()
        tmp = self._file(self.LOSS + ".tmp")
        with open(tmp, "w") as f:
            json.dump(record, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self._file(self.LOSS))

    def append_event(self, kind: str, **fields) -> None:
        """One JSON line per event; never raises."""
        try:
            self.ensure()
            record = {"ts": time.time(), "event": kind}
            record.update(fields)
            with open(self._file(self.EVENTS), "a") as f:
                f.write(json.dumps(record) + "\n")
        except OSError as error:
            LOG.warning("could not append event %s: %s", kind, error)


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

    def __init__(self, sensor, th: Thresholds, actions: SystemActions, rtc: Rtc,
                 state: StateDir, *, status_file: str = DEFAULT_STATUS_FILE,
                 poll_s: float = DEFAULT_POLL_S,
                 first_alarm_s: int = DEFAULT_FIRST_ALARM_S,
                 kill_services=DEFAULT_KILL_SERVICES,
                 detect_only: bool = False) -> None:
        self.sensor = sensor
        self.th = th
        self.actions = actions
        self.rtc = rtc
        self.state = state
        self.status_file = status_file
        self.poll_s = float(poll_s)
        self.first_alarm_s = int(first_alarm_s)
        self.kill_services = tuple(kill_services)
        self.detect_only = detect_only
        self.timer = LossTimer(th.loss_s, poll_s)
        self.last_sample: Sample | None = None
        self.last_state: str | None = None
        self.errors_total = 0
        self.errors_run = 0
        self.last_error: str | None = None
        self.halting = False
        self._blind_warned_at = 0.0
        self._heartbeat_at = 0.0
        self._nohalt_warned_at = 0.0
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

    def status(self, now: float, state_name: str) -> dict:
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
            "absent_s": round(self.timer.absent_s, 2),
            "loss_s": self.th.loss_s,
            "state": state_name,
            "mode": self.mode(),
            "halt_enabled": self.halt_enabled(),
            "errors": self.errors_total,
            "errors_run": self.errors_run,
            "last_error": self.last_error,
            "last_loss": self.state.read_loss(),
            "first_alarm_s": self.first_alarm_s,
        }

    def publish(self, now: float, state_name: str) -> None:
        ok = write_status_file(self.status_file, self.status(now, state_name))
        if not ok and not self._status_write_failed:
            LOG.warning("cannot write %s (will keep trying quietly)", self.status_file)
        self._status_write_failed = not ok

    # -- the poll -----------------------------------------------------------
    def step(self, now: float | None = None) -> str:
        now = time.time() if now is None else now
        if self.halting:
            self.publish(now, "shutdown")
            return "shutdown"
        try:
            sample = self.sensor.read(now)
        except Exception as error:  # noqa: BLE001 - any bus failure is "unknown"
            self.errors_total += 1
            self.errors_run += 1
            self.last_error = f"{type(error).__name__}: {error}"
            state = UNKNOWN
            if self.errors_run == self.REOPEN_AFTER_ERRORS:
                LOG.warning("%d consecutive INA219 read errors; reopening the bus (%s)",
                            self.errors_run, self.last_error)
                try:
                    self.sensor.reopen()
                    self.sensor.configure()
                except Exception as reopen_error:  # noqa: BLE001
                    LOG.warning("reopen failed: %s", reopen_error)
            elif now - self._blind_warned_at >= self.BLIND_WARN_PERIOD_S:
                self._blind_warned_at = now
                LOG.warning("INA219 unreadable (%d in a row): %s", self.errors_run,
                            self.last_error)
        else:
            if self.errors_run:
                LOG.info("INA219 readable again after %d errors", self.errors_run)
            self.errors_run = 0
            self.last_sample = sample
            state = classify(sample.batt_ma, sample.batt_v, self.th)
            if state != self.last_state:
                LOG.info("input %s -> %s (batt %.3f V %+.0f mA, soc~%d%%)",
                         self.last_state, state, sample.batt_v, sample.batt_ma,
                         soc_estimate_pct(sample.batt_v))
            self.last_state = state

        triggered = self.timer.update(state)
        if not self.halt_enabled() and now - self._nohalt_warned_at >= self.NOHALT_WARN_PERIOD_S:
            self._nohalt_warned_at = now
            LOG.warning("HALT DISABLED (%s): a real power loss will NOT shut the Pi down",
                        self.mode())
        if now - self._heartbeat_at >= self.HEARTBEAT_S and self.last_sample:
            self._heartbeat_at = now
            LOG.info("heartbeat: input=%s batt %.3f V %+.0f mA absent_s=%.1f errors=%d",
                     state, self.last_sample.batt_v, self.last_sample.batt_ma,
                     self.timer.absent_s, self.errors_total)

        if triggered:
            return self._on_loss(now)
        state_name = "countdown" if self.timer.absent_s > 0 else "monitoring"
        self.publish(now, state_name)
        return state_name

    # -- the loss -----------------------------------------------------------
    def _on_loss(self, now: float, *, simulated: bool = False) -> str:
        s = self.last_sample
        record = {
            "ts": now,
            "batt_v": round(s.batt_v, 3) if s else None,
            "batt_ma": round(s.batt_ma, 1) if s else None,
            "supply_v": self.th.supply_v,
            "halt_enabled": self.halt_enabled(),
            "mode": self.mode(),
            "simulated": simulated,
        }
        LOG.warning("%sMAINS LOST: %.1f s of absent samples (batt %s V %s mA). %s",
                    "SIMULATED " if simulated else "",
                    self.timer.absent_s, record["batt_v"], record["batt_ma"],
                    "Shutting down." if self.halt_enabled() else
                    f"HALT DISABLED ({self.mode()}), staying up.")
        self.state.append_event("loss", **record)
        try:
            self.state.write_loss(record)
        except OSError as error:
            LOG.error("could not persist loss.json: %s", error)

        if not self.halt_enabled():
            self.timer.reset()
            self.publish(now, "loss-ignored")
            return "loss-ignored"

        self.halting = True
        self.publish(now, "shutdown")
        for name in self.kill_services:
            LOG.warning("killing %s before the halt", name)
            self.actions.kill_service(name)
        try:
            alarm = self.rtc.arm(self.first_alarm_s)
            LOG.warning("RTC alarm armed: +%d s (rtc epoch %d)", self.first_alarm_s, alarm)
            self.state.append_event("alarm", seconds=self.first_alarm_s, alarm=alarm)
        except Exception as error:  # noqa: BLE001
            LOG.error("could not arm the RTC alarm (%s); powering off anyway. "
                      "Recovery: the power button, or the pack cutoff followed by mains.",
                      error)
            self.state.append_event("alarm_failed", error=str(error))
        self.actions.sync()
        LOG.warning("systemctl poweroff")
        self.actions.poweroff()
        return "shutdown"


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
    LOG.warning("walle-ups watching INA219 0x%02x on bus %d: supply %.2f V, present >= %.0f mA, "
                "absent <= %.0f mA, loss after %.1f s, first alarm +%d s, mode %s",
                w.sensor.addr, w.sensor.bus_num, w.th.supply_v, w.th.present_ma,
                w.th.absent_ma, w.th.loss_s, w.first_alarm_s, w.mode())
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
               wake_s: int = DEFAULT_WAKE_S, wake_fast_s: int = DEFAULT_WAKE_FAST_S,
               fast_window_s: float = DEFAULT_WAKE_FAST_WINDOW_S,
               now_fn=time.time, sleep=time.sleep, dev_wait_s: float = 5.0) -> str:
    """Returns the decision: "present", "nohalt", "poweroff" or "inconclusive".
    Only "poweroff" has a side effect."""
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

    states: list[str] = []
    samples: list[Sample] = []
    errors = 0
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
            samples.append(s)
            states.append(classify(s.batt_ma, s.batt_v, th))
        if now_fn() >= deadline:
            break
        sleep(poll_s)

    summary = {
        "n": len(samples), "errors": errors,
        "present": states.count(PRESENT), "absent": states.count(ABSENT),
        "above": states.count(ABOVE_SUPPLY), "unknown": states.count(UNKNOWN),
        "batt_v": round(samples[-1].batt_v, 3) if samples else None,
        "batt_ma": round(samples[-1].batt_ma, 1) if samples else None,
    }
    if summary["present"]:
        LOG.warning("boot-check: mains present (%s); continuing the boot", summary)
        state.append_event("boot_check", decision="present", **summary)
        return "present"
    clear_majority = (summary["n"] >= 4 and summary["absent"] >= 0.75 * summary["n"]
                      and summary["above"] == 0)
    if not clear_majority:
        LOG.warning("boot-check: inconclusive (%s); continuing the boot (fail open)", summary)
        state.append_event("boot_check", decision="inconclusive", **summary)
        return "inconclusive"

    loss = state.read_loss()
    age = None
    if loss and isinstance(loss.get("ts"), (int, float)):
        age = max(0.0, now_fn() - float(loss["ts"]))
    seconds = wake_fast_s if (age is not None and age <= fast_window_s) else wake_s
    LOG.warning("boot-check: mains still absent (%s, loss %s s ago); re-arming +%d s and "
                "powering off", summary, None if age is None else int(age), seconds)
    state.append_event("boot_check", decision="poweroff", wake_s=seconds, loss_age_s=age,
                       **summary)
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
                   help="charger/buck setpoint the pack floats at (bench brick 8.4, robot buck 7.6)")
    p.add_argument("--gate-margin-v", type=float, default=DEFAULT_GATE_MARGIN_V, dest="gate_margin_v")
    p.add_argument("--present-ma", type=float, default=DEFAULT_PRESENT_MA, dest="present_ma")
    p.add_argument("--absent-ma", type=float, default=DEFAULT_ABSENT_MA, dest="absent_ma")
    p.add_argument("--loss-s", type=float, default=DEFAULT_LOSS_S, dest="loss_s")
    p.add_argument("--poll-s", type=float, default=DEFAULT_POLL_S, dest="poll_s")
    p.add_argument("--first-alarm-s", type=int, default=DEFAULT_FIRST_ALARM_S, dest="first_alarm_s")
    p.add_argument("--wake-s", type=int, default=DEFAULT_WAKE_S, dest="wake_s")
    p.add_argument("--wake-fast-s", type=int, default=DEFAULT_WAKE_FAST_S, dest="wake_fast_s")
    p.add_argument("--wake-fast-window-s", type=float, default=DEFAULT_WAKE_FAST_WINDOW_S,
                   dest="wake_fast_window_s")
    p.add_argument("--status-file", default=DEFAULT_STATUS_FILE, dest="status_file")
    p.add_argument("--state-dir", default=DEFAULT_STATE_DIR, dest="state_dir")
    p.add_argument("--rtc", default=DEFAULT_RTC)
    p.add_argument("--kill-service", action="append", dest="kill_services",
                   help="unit to SIGKILL before the halt (repeatable; default wall-e.service)")
    p.add_argument("--detect-only", action="store_true", dest="detect_only",
                   help="never halt or arm anything; log what would happen")
    p.add_argument("--verbose", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s: %(message)s", stream=sys.stderr)
    th = Thresholds(supply_v=args.supply_v, gate_margin_v=args.gate_margin_v,
                    present_ma=args.present_ma, absent_ma=args.absent_ma, loss_s=args.loss_s)
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
            "input": classify(s.batt_ma, s.batt_v, th), "supply_v": th.supply_v,
            "nohalt_flag": state.nohalt(),
        }, indent=2))
        return 0

    if args.mode == "boot-check":
        decision = boot_check(sensor, th, actions, rtc, state, wake_s=args.wake_s,
                              wake_fast_s=args.wake_fast_s, fast_window_s=args.wake_fast_window_s)
        return 0 if decision else 1

    w = Watcher(sensor, th, actions, rtc, state, status_file=args.status_file,
                poll_s=args.poll_s, first_alarm_s=args.first_alarm_s,
                kill_services=kill, detect_only=args.detect_only)

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
        w._on_loss(time.time(), simulated=True)
        time.sleep(30)
        return 0

    run_loop(w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
