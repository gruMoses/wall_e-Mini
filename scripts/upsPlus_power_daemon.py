#!/usr/bin/env python3
"""UPSPlus (Geekworm EP-0136 family) power watcher.

Watches the UPS MCU + INA219 fuel gauge over I2C and triggers a graceful
Pi shutdown if the charger is genuinely removed for a sustained period.
Restores AC power automatically re-boots the Pi via the UPS "Back-to-AC"
auto power-on feature.

DEPLOYMENT NOTE: this file is the source of truth in version control. The
live copy runs from /home/pi/bin/upsPlus_power_daemon.py under the
upsplus-power.service systemd unit (ExecStart uses
/home/pi/.venv/bin/python3). Deploying a new version means copying this
file over the live one and restarting the service — see
docs/ups_shutdown_bench_test.md for the full validated procedure. This
repo copy alone does NOT change anything on the robot.

────────────────────────────────────────────────────────────────────────
BUG THIS FILE FIXES (diagnosed live 2026-07-05, see docs/ups_shutdown_bench_test.md):

The previous version of this daemon inferred "AC present/absent" primarily
from the INA219 BATTERY CURRENT SIGN: negative current (battery discharging)
was treated as "no charger," positive current (battery charging) as
"charger present." On a topped-off/near-full pack, the charge controller
naturally taper-cycles — it stops pushing current into a full battery for
a bit, the pack drifts to a slight discharge, current sign flips negative,
then charging resumes. This charge/taper cycling happens on a ~1-2 minute
period even though the charger has never been unplugged.

The live journal showed this exact pattern, over and over, for 2+ hours:
    UPS AC state changed -> missing (typec=5048mV microusb=0mV batt=3.732V -3372.0mA)
    UPS AC state changed -> present (typec=4833mV microusb=... +2370.0mA)      (~8-9s later)

Meanwhile typec_mv (USB-C charger input voltage) read ~4.7-5.1V THE ENTIRE
TIME — the charger was never actually disconnected. Only the current sign
flipped. Each "missing" transition armed a 30s shutdown-countdown grace
window (NO_CHARGE_GRACE_SECONDS); it always got cancelled ~8-9s later when
current flipped back positive. But a sustained load or a longer charge-taper
window (>30s) would have let the countdown complete and FALSELY SHUT DOWN
THE PI while genuinely on AC power. This is the same anti-pattern as a BMS
bug fixed previously elsewhere in this codebase: instantaneous current sign
used as a boolean power-source state without debounce.

THE FIX:
  1. AC-present detection is now PRIMARY on charger INPUT VOLTAGE
     (typec_mv or microusb_mv from the UPS MCU registers), not battery
     current sign. Charger voltage is a direct, stable measurement of
     "is a charger physically connected and delivering power" — it does
     not flap on charge-taper cycles the way current sign does.
  2. Battery current sign is now only logged as corroborating context,
     never used to flip the authoritative state.
  3. A debounce window (AC_LOSS_DEBOUNCE_S) requires charger voltage to
     stay below threshold continuously before "AC lost" is declared. A
     momentary I2C glitch or a single low reading no longer arms anything.
  4. Genuine, sustained power loss (charger voltage actually gone, for
     the full debounce window) still starts the existing
     NO_CHARGE_GRACE_SECONDS grace timer and, if it persists further,
     still shuts the Pi down cleanly. This safety behavior is preserved
     unchanged — only the trigger condition is fixed.
────────────────────────────────────────────────────────────────────────
"""

import json
import logging
import os
import secrets
import shutil
import subprocess
import sys
import time

import smbus2
from ina219 import INA219

# I2C configuration
DEVICE_BUS = 1

# Register map used by this UPS firmware family
REG_BAT_PROTECT_LOW = 17
REG_BAT_PROTECT_HIGH = 18
REG_SAMPLE_PERIOD_LOW = 21
REG_SAMPLE_PERIOD_HIGH = 22
REG_SHUTDOWN_COUNTDOWN = 24
REG_AUTO_POWER_ON = 25

# Retry / hardening behavior
I2C_RETRY_COUNT = 5
I2C_RETRY_DELAY_SECONDS = 0.15

# Behavior configuration
# NO_CHARGE_GRACE_SECONDS: once AC is *debounced* as lost (see
# AC_LOSS_DEBOUNCE_S below), how long we wait before starting the
# shutdown sequence. This absorbs any residual short blips on top of
# the debounce and gives a human a window to notice/replug.
NO_CHARGE_GRACE_SECONDS = 30
# UPS_SHUTDOWN_COUNTDOWN_SECONDS: after the OS halt command, how long the UPS
# waits before cutting its own output. The Pi halts in ~10-20s, so 60s is a
# ~3x margin over worst-case halt while cutting battery drain to a halted Pi
# much sooner than the old 120s. Do NOT reduce below 10s: the EP-0136 (FW v14)
# shutdown-countdown register 0x18 floor is 10s (valid range 0 or 10-255s).
UPS_SHUTDOWN_COUNTDOWN_SECONDS = 60
BOOT_GRACE_SECONDS = 10

# --- AC-present detection (charger voltage primary) ------------------------
#
# Charger input voltage threshold (mV). Both typec_mv and microusb_mv read
# ~4.7-5.1V whenever a USB charger is actually connected and delivering
# power on this UPS MCU family; they read ~0V when nothing is plugged in.
# 4000mV leaves comfortable margin below a sagging/marginal 5V rail while
# staying well above "definitely nothing connected."
AC_PRESENT_VOLTAGE_THRESHOLD_MV = 4000

# How long charger voltage must stay continuously BELOW threshold before we
# declare AC lost. This is what stops instantaneous glitches/noise from
# arming the shutdown path. Chosen well above the ~1-2 minute charge-taper
# current-sign flapping period's *voltage* noise (which in practice is near
# zero — voltage does not flap the way current sign does) and well above a
# couple of consecutive I2C hiccups (each retried internally already), but
# still short enough that a real unplug is caught quickly. 10s means at our
# ~1s poll cadence we need roughly 10 consecutive low readings.
AC_LOSS_DEBOUNCE_S = 10.0

# INA219 current is NO LONGER authoritative for AC presence (see bug note
# above). It is retained only as corroborating context in log lines, using
# this threshold purely for human-readable "charging/discharging" labels.
BATTERY_DISCHARGE_CURRENT_MA = -50.0

# --- Status-file bridge for the web debug board ----------------------------
#
# Each poll loop this daemon atomically publishes a tiny JSON snapshot here so
# WALL-E's web dashboard (pi_app/web/oak_viewer.py, /api/ups) can show live UPS
# state WITHOUT touching the I2C bus itself. This is best-effort telemetry only:
# the write is fully exception-isolated (see write_status_file) so a failure can
# NEVER perturb the daemon's AC-detection / shutdown decision logic. Path is
# overridable via env so tests and the reader can point at the same location.
UPS_STATUS_FILE = os.environ.get("UPS_STATUS_FILE", "/tmp/ups_status.json")

# 18650 Li-ion battery protection threshold (mV)
# Owner prioritizes cell longevity over backup runtime: this UPS exists to give
# a ~60s graceful-shutdown window, not to ride through an outage, so we keep the
# cells off the deep-discharge floor. 3400mV lands at a ~15-20% SoC floor vs the
# ~5-10% a 3200mV cutoff allowed. Pack is 1S, so per-cell == pack; valid range
# for protection regs 0x11/0x12 is 0-4500mV.
BATTERY_PROTECTION_MV = 3400


def detect_addr(bus: smbus2.SMBus) -> int:
    for addr in (0x17, 0x18):
        try:
            bus.read_byte_data(addr, 1)
            return addr
        except Exception:
            pass
    raise RuntimeError("UPS MCU not found at 0x17 or 0x18")


def read_reg_with_retry(bus: smbus2.SMBus, device_addr: int, register: int) -> int:
    for attempt in range(1, I2C_RETRY_COUNT + 1):
        try:
            return bus.read_byte_data(device_addr, register)
        except Exception as error:
            if attempt == I2C_RETRY_COUNT:
                raise RuntimeError(
                    f"read reg {register} failed after {I2C_RETRY_COUNT} tries: {error}"
                ) from error
            time.sleep(I2C_RETRY_DELAY_SECONDS)
    raise RuntimeError(f"unreachable read failure for register {register}")


def write_reg_verified(
    bus: smbus2.SMBus, device_addr: int, register: int, value: int
) -> bool:
    value &= 0xFF
    for attempt in range(1, I2C_RETRY_COUNT + 1):
        try:
            bus.write_byte_data(device_addr, register, value)
            readback = read_reg_with_retry(bus, device_addr, register)
            if readback == value:
                return True
            logging.warning(
                "I2C verify mismatch reg=%d wrote=%d read=%d attempt=%d/%d",
                register,
                value,
                readback,
                attempt,
                I2C_RETRY_COUNT,
            )
        except Exception as error:
            logging.warning(
                "I2C write/verify error reg=%d value=%d attempt=%d/%d err=%s",
                register,
                value,
                attempt,
                I2C_RETRY_COUNT,
                error,
            )
        time.sleep(I2C_RETRY_DELAY_SECONDS)
    return False


def read_charger_voltages(bus: smbus2.SMBus, device_addr: int) -> tuple[int, int]:
    """Read typec_mv / microusb_mv charger-input voltage registers.

    Register layout matches the original daemon's read_ups_snapshot: regs[]
    is 0-indexed over register addresses 1..10, with typec low/high byte at
    regs[6]/regs[7] and microusb low/high byte at regs[8]/regs[9].
    """
    regs = [read_reg_with_retry(bus, device_addr, idx) for idx in range(1, 11)]
    typec_mv = (regs[7] << 8) | regs[6]
    microusb_mv = (regs[9] << 8) | regs[8]
    return typec_mv, microusb_mv


def is_ac_present_instant(typec_mv: int, microusb_mv: int) -> bool:
    """Single-sample AC-present decision from charger voltage alone.

    This is intentionally the ONLY signal used for the authoritative
    decision (battery current sign is not consulted here at all) — see the
    module docstring for why current sign was unreliable. Debouncing this
    instantaneous read into a stable state is handled by AcPresenceTracker
    below; this function has no memory of its own.
    """
    return (typec_mv > AC_PRESENT_VOLTAGE_THRESHOLD_MV) or (
        microusb_mv > AC_PRESENT_VOLTAGE_THRESHOLD_MV
    )


class AcPresenceTracker:
    """Debounces instantaneous AC-present voltage readings into a stable state.

    PRESENT is reported immediately on any single reading above threshold
    (a charger coming back is good news; no need to delay recognizing it).
    MISSING is only reported after the voltage has been continuously below
    threshold for AC_LOSS_DEBOUNCE_S seconds — a single low reading (I2C
    glitch, transient sag) does not flip the state.

    This is pure logic with no I2C/hardware access, so it is unit-testable
    with synthetic (typec_mv, microusb_mv, now) sequences. See
    pi_app/tests/test_ups_ac_debounce.py.
    """

    def __init__(self, debounce_s: float = AC_LOSS_DEBOUNCE_S):
        self._debounce_s = debounce_s
        self._state: bool | None = None  # None until first sample
        self._below_threshold_since: float | None = None

    @property
    def state(self) -> bool | None:
        return self._state

    def update(self, typec_mv: int, microusb_mv: int, now: float) -> bool:
        """Feed one instantaneous reading; returns the current debounced state."""
        instant_present = is_ac_present_instant(typec_mv, microusb_mv)

        if instant_present:
            # Voltage seen above threshold: AC is present, no debounce needed.
            self._below_threshold_since = None
            self._state = True
            return self._state

        # Voltage below threshold this sample.
        if self._below_threshold_since is None:
            self._below_threshold_since = now

        if self._state is None:
            # First-ever sample was already low; require the debounce window
            # to elapse before committing to MISSING rather than assuming it
            # instantly (avoids a bad very-first read declaring missing).
            if now - self._below_threshold_since >= self._debounce_s:
                self._state = False
            else:
                self._state = True
            return self._state

        if self._state is True:
            # Was present; only flip to missing once low continuously for
            # the full debounce window.
            if now - self._below_threshold_since >= self._debounce_s:
                self._state = False
            # else: stay PRESENT, still inside debounce window.
        # else: already False and still below threshold — stays False.

        return self._state


def read_ups_snapshot(
    bus: smbus2.SMBus, device_addr: int, ina_batt: INA219 | None = None
) -> dict[str, float | int | None]:
    regs = [read_reg_with_retry(bus, device_addr, idx) for idx in range(1, 11)]
    typec_mv = (regs[7] << 8) | regs[6]
    microusb_mv = (regs[9] << 8) | regs[8]
    protect_low = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_LOW)
    protect_high = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_HIGH)
    protect_mv = (protect_high << 8) | protect_low
    countdown = read_reg_with_retry(bus, device_addr, REG_SHUTDOWN_COUNTDOWN)
    auto_on = read_reg_with_retry(bus, device_addr, REG_AUTO_POWER_ON)
    sample_low = read_reg_with_retry(bus, device_addr, REG_SAMPLE_PERIOD_LOW)
    sample_high = read_reg_with_retry(bus, device_addr, REG_SAMPLE_PERIOD_HIGH)
    sample_minutes = (sample_high << 8) | sample_low

    batt_v = None
    batt_i = None
    if ina_batt is not None:
        try:
            batt_v = round(float(ina_batt.voltage()), 3)
            batt_i = round(float(ina_batt.current()), 1)
        except Exception:
            pass

    return {
        "typec_mv": typec_mv,
        "microusb_mv": microusb_mv,
        "protect_mv": protect_mv,
        "shutdown_countdown_s": countdown,
        "auto_power_on": auto_on,
        "sample_period_min": sample_minutes,
        "battery_v": batt_v,
        "battery_i_ma": batt_i,
    }


def write_status_file(
    *,
    ac_present: bool,
    typec_mv: int,
    microusb_mv: int,
    seconds_without_charge: int,
    detect_only: bool,
    batt_v: float | None = None,
    batt_ma: float | None = None,
    protect_mv: int | None = None,
    path: str = UPS_STATUS_FILE,
) -> bool:
    """Atomically publish a small UPS status snapshot for the web dashboard.

    Best-effort TELEMETRY ONLY. This is deliberately wrapped so that ANY
    failure — bad/unwritable path, disk full, permission error, serialization
    error — is swallowed and can NEVER propagate into the caller's poll loop
    or perturb the AC-detection / shutdown decision logic. This mirrors the
    flight-recorder try/except discipline used elsewhere in this codebase.

    The file is written to a temp path in the same directory and then
    ``os.replace``d into place, so a concurrent reader never observes a
    half-written file (os.replace is atomic within a filesystem).

    SYMLINK HARDENING: /tmp is world-writable and this daemon typically runs
    as root, so a predictable temp name would let a local attacker pre-plant
    a symlink and turn the open() into an arbitrary-file-write-as-root. The
    temp file is therefore created with O_CREAT|O_EXCL|O_NOFOLLOW (refuses to
    open ANY pre-existing path — O_EXCL never follows a symlink, dangling or
    not; O_NOFOLLOW is belt-and-suspenders) under an unpredictable
    secrets.token_hex suffix so the name can't be guessed and squatted ahead
    of time. Any failure — including EEXIST from a squatted name — is
    swallowed like every other error here: a skipped publish cycle is fine,
    the next poll retries. os.replace itself is symlink-safe (it renames over
    the destination rather than writing through it).

    Returns True on a successful write, False if the write was suppressed by an
    error (callers ignore the return; it exists purely for tests).
    """
    tmp = None
    created = False
    try:
        payload = {
            "ts": time.time(),
            "ac_present": bool(ac_present),
            "typec_mv": typec_mv,
            "microusb_mv": microusb_mv,
            "batt_v": batt_v,
            "batt_ma": batt_ma,
            "protect_mv": protect_mv,
            "detect_only": bool(detect_only),
            "seconds_without_charge": seconds_without_charge,
        }
        data = json.dumps(payload).encode("utf-8")
        tmp = f"{path}.tmp.{os.getpid()}.{secrets.token_hex(6)}"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
        created = True
        try:
            os.write(fd, data)
        finally:
            os.close(fd)
        os.replace(tmp, path)
        return True
    except Exception as error:  # noqa: BLE001 — intentional catch-all (see docstring)
        logging.debug("UPS status file write failed (non-fatal): %s", error)
        # Only clean up a temp file WE created — never unlink a pre-existing
        # path (e.g. an attacker-planted symlink that made os.open EEXIST).
        if created:
            try:
                os.unlink(tmp)
            except Exception:
                pass
        return False


def _publish_status(
    bus: smbus2.SMBus,
    device_addr: int,
    ina_batt: INA219 | None,
    *,
    ac_present: bool,
    typec_mv: int,
    microusb_mv: int,
    seconds_without_charge: int,
    detect_only: bool,
) -> None:
    """Read battery/protect context and publish the status file — fully isolated.

    Wraps BOTH the supplemental I2C/INA read and the file write in one broad
    try/except so neither can ever raise into the poll loop. The authoritative
    charger-voltage readings (typec_mv/microusb_mv) are passed in already-read;
    only the extra "nice to have" fields (batt V/mA, protect threshold) are
    fetched here, and if that read fails they degrade to None rather than
    aborting the publish.
    """
    try:
        batt_v = batt_ma = protect_mv = None
        try:
            snap = read_ups_snapshot(bus, device_addr, ina_batt)
            batt_v = snap.get("battery_v")
            batt_ma = snap.get("battery_i_ma")
            protect_mv = snap.get("protect_mv")
        except Exception:
            pass  # supplemental context only; publish what we have
        write_status_file(
            ac_present=ac_present,
            typec_mv=typec_mv,
            microusb_mv=microusb_mv,
            seconds_without_charge=seconds_without_charge,
            detect_only=detect_only,
            batt_v=batt_v,
            batt_ma=batt_ma,
            protect_mv=protect_mv,
            path=UPS_STATUS_FILE,
        )
    except Exception as error:  # noqa: BLE001 — must never reach the poll loop
        logging.debug("UPS status publish failed (non-fatal): %s", error)


def stop_rover_service() -> None:
    """Stop the wall-e service so OAK-D pipeline shuts down cleanly."""
    logging.info("Stopping wall-e.service for clean OAK-D teardown...")
    try:
        result = subprocess.run(
            ["sudo", "systemctl", "stop", "wall-e.service"],
            timeout=15,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            logging.info("wall-e.service stopped.")
        else:
            logging.warning(
                "wall-e.service stop returned %d: %s",
                result.returncode,
                result.stderr.strip(),
            )
    except subprocess.TimeoutExpired:
        logging.warning("wall-e.service stop timed out after 15s.")
    except Exception as error:
        logging.warning("Failed to stop wall-e.service: %s", error)


def shed_usb_load() -> None:
    """Cut USB power to the OAK-D hub to reduce battery drain.

    Pi 5 root hubs support per-port power switching.  The OAK-D sits
    behind a Genesys hub on Bus 3 Port 1 (USB 2.0) / Bus 4 Port 1
    (USB 3.0 companion).  The VESC CH340 is on Bus 3 Port 2 and is
    left untouched.
    """
    uhubctl = shutil.which("uhubctl")
    if uhubctl:
        for loc, port, label in [
            ("3", "1", "USB2.0 hub (OAK-D)"),
            ("4", "1", "USB3.0 companion hub"),
        ]:
            try:
                result = subprocess.run(
                    ["sudo", uhubctl, "-l", loc, "-p", port, "-a", "off"],
                    timeout=5,
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    logging.info("USB power off: %s (location %s port %s).", label, loc, port)
                else:
                    logging.warning(
                        "uhubctl power off failed for %s: %s", label, result.stderr.strip()
                    )
            except Exception as error:
                logging.warning("uhubctl error for %s: %s", label, error)
    else:
        logging.info("uhubctl not found; falling back to sysfs deauthorize.")
        for devpath, label in [
            ("/sys/bus/usb/devices/3-1.3/authorized", "OAK-D (3-1.3)"),
            ("/sys/bus/usb/devices/3-1/authorized", "USB2.0 hub (3-1)"),
            ("/sys/bus/usb/devices/4-1/authorized", "USB3.0 hub (4-1)"),
        ]:
            try:
                with open(devpath, "w") as f:
                    f.write("0")
                logging.info("Deauthorized %s via sysfs.", label)
            except Exception as error:
                logging.warning("sysfs deauthorize failed for %s: %s", label, error)


def _resolve_detect_only(argv: list[str] | None = None) -> bool:
    """Decide whether DETECT-ONLY (bench-test) mode is active.

    Activated by EITHER the ``--detect-only`` CLI flag OR the environment
    variable ``UPS_DETECT_ONLY`` set to a truthy value (1/true/yes/on). The
    env-var path exists so a systemd drop-in can flip the mode with
    ``Environment=UPS_DETECT_ONLY=1`` without editing the unit's ExecStart.

    Pure/argument-injectable so it is unit-testable without touching argv or
    the real environment.
    """
    if argv is None:
        argv = sys.argv[1:]
    env_val = os.environ.get("UPS_DETECT_ONLY", "").strip().lower()
    env_on = env_val in ("1", "true", "yes", "on")
    flag_on = "--detect-only" in argv
    return env_on or flag_on


def run_shutdown_sequence(
    bus: smbus2.SMBus,
    device_addr: int,
    ina_batt: INA219 | None = None,
    *,
    detect_only: bool = False,
) -> None:
    """Grace window expired with the charger still absent — react.

    In NORMAL (armed) mode this performs the full validated shutdown
    sequence: stop the rover service, shed USB load, write the UPS hardware
    shutdown-countdown register, then sync and halt the Pi. Behavior here is
    unchanged from the pre-detect-only daemon.

    In DETECT-ONLY (bench-test) mode EVERY hardware/side-effecting action is
    suppressed at its OWN call site — deliberately NOT behind a single early
    return — so a future refactor cannot accidentally re-arm one of them in
    test mode. Nothing is stopped, no USB is shed, NO UPS register is
    written, and no sync/shutdown is issued; the function simply returns so
    the caller keeps monitoring. Only reads happen (the grace-expiry snapshot
    below), which are safe.

    The grace-expiry report itself is detection logging and is emitted
    identically in both modes.
    """
    # --- Grace-expiry report: detection logging, identical in both modes. ---
    try:
        snap = read_ups_snapshot(bus, device_addr, ina_batt)
        logging.warning(
            "No charger for %ss (debounced); safe shutdown sequence begins. "
            "typec=%smV microusb=%smV protect=%smV batt=%sV %smA",
            NO_CHARGE_GRACE_SECONDS,
            snap["typec_mv"],
            snap["microusb_mv"],
            snap["protect_mv"],
            snap["battery_v"],
            snap["battery_i_ma"],
        )
    except Exception as error:
        logging.warning(
            "No charger for %ss; safe shutdown sequence begins (snapshot failed: %s).",
            NO_CHARGE_GRACE_SECONDS,
            error,
        )

    if detect_only:
        logging.warning(
            "DETECT-ONLY: AC loss confirmed (grace expired) — would begin "
            "shutdown sequence NOW (suppressed). Continuing to monitor; "
            "replug to reset and repeat."
        )

    # --- ACTION SITE 1: stop the wall-e rover service. ----------------------
    if detect_only:
        logging.info("DETECT-ONLY: suppressed stop_rover_service() — wall-e left running.")
    else:
        stop_rover_service()

    # --- ACTION SITE 2: shed USB load (OAK-D hub power). --------------------
    if detect_only:
        logging.info("DETECT-ONLY: suppressed shed_usb_load() — USB power left on.")
    else:
        shed_usb_load()

    # --- ACTION SITE 3: write the UPS hardware shutdown-countdown register. --
    if detect_only:
        logging.info(
            "DETECT-ONLY: suppressed UPS shutdown-countdown register write "
            "(reg %d would be set to %ss).",
            REG_SHUTDOWN_COUNTDOWN,
            UPS_SHUTDOWN_COUNTDOWN_SECONDS,
        )
    else:
        logging.info(
            "Setting UPS shutdown countdown to %ss.",
            UPS_SHUTDOWN_COUNTDOWN_SECONDS,
        )
        # Request UPS to cut power after countdown; with Back-to-AC enabled it
        # powers back on only when AC returns.
        if write_reg_verified(
            bus,
            device_addr,
            REG_SHUTDOWN_COUNTDOWN,
            UPS_SHUTDOWN_COUNTDOWN_SECONDS,
        ):
            logging.info(
                "UPS shutdown countdown verified at %ss.",
                UPS_SHUTDOWN_COUNTDOWN_SECONDS,
            )
        else:
            logging.warning(
                "Failed to set UPS shutdown countdown after retries; proceeding with OS shutdown."
            )

    # --- ACTION SITE 4: sync + OS halt. -------------------------------------
    if detect_only:
        logging.info(
            "DETECT-ONLY: suppressed sync + '/sbin/shutdown -h now' — Pi stays up."
        )
        return

    logging.info("Running sync and OS halt now.")
    os.system("sudo sync")
    os.system("sudo /sbin/shutdown -h now")
    # Give systemd time; if still running, wait to avoid repeated triggers.
    time.sleep(600)
    # After halt, we should not reach here under normal conditions.


def main(detect_only: bool | None = None) -> None:
    if detect_only is None:
        detect_only = _resolve_detect_only()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

    if detect_only:
        logging.warning("=" * 64)
        logging.warning("MODE: DETECT-ONLY — shutdown actions suppressed.")
        logging.warning(
            "Bench-test mode: the Pi will NOT shut down on charger loss and NO "
            "UPS register WRITES are performed (strictly read-only against the "
            "hardware). Detection, debounce, and grace logging run normally. "
            "Do NOT leave this enabled in service — the robot's power protection "
            "is DISARMED until the mode banner is gone."
        )
        logging.warning("=" * 64)

    # Guard the startup hardware probe: if the UPS MCU or INA219 isn't
    # present (e.g. running on hardware without the HAT, or a bench test
    # without it wired up), log clearly and exit 0 rather than crash-loop.
    # systemd Restart=always + RestartSec=2 would otherwise spin forever.
    try:
        bus = smbus2.SMBus(DEVICE_BUS)
        device_addr = detect_addr(bus)
    except Exception as error:
        logging.warning(
            "UPS MCU not detected on I2C bus %d (%s). Nothing to watch; exiting cleanly.",
            DEVICE_BUS,
            error,
        )
        return

    ina_batt = None
    try:
        ina_batt = INA219(0.005, busnum=DEVICE_BUS, address=0x45)
        ina_batt.configure()
    except Exception as error:
        logging.warning(
            "INA219 fuel gauge not available (%s). Continuing with charger-voltage "
            "detection only; battery current will not be logged.",
            error,
        )
        ina_batt = None

    # Log initial observed config for traceability.
    try:
        snap = read_ups_snapshot(bus, device_addr, ina_batt)
        logging.info(
            "UPS initial snapshot: typec=%smV microusb=%smV protect=%smV auto_on=%s "
            "countdown=%ss sample=%smin batt=%sV %smA",
            snap["typec_mv"],
            snap["microusb_mv"],
            snap["protect_mv"],
            snap["auto_power_on"],
            snap["shutdown_countdown_s"],
            snap["sample_period_min"],
            snap["battery_v"],
            snap["battery_i_ma"],
        )
    except Exception as error:
        logging.warning("Unable to read UPS initial snapshot: %s", error)

    # Ensure auto power-on when AC returns.
    # ACTION SITE (startup): UPS register write, suppressed in detect-only so
    # the mode is read-only against the UPS MCU (no UPS register writes; INA219 sensor configure still runs).
    if detect_only:
        logging.info(
            "DETECT-ONLY: skipped Back-to-AC auto power-on register write "
            "(reg %d) — read-only mode.",
            REG_AUTO_POWER_ON,
        )
    elif write_reg_verified(bus, device_addr, REG_AUTO_POWER_ON, 1):
        logging.info("Back-to-AC auto power-on enabled.")
    else:
        logging.warning("Failed to enable Back-to-AC after retries.")

    # Ensure battery protection threshold is set and verified.
    # ACTION SITE (startup): UPS register writes, suppressed in detect-only.
    if detect_only:
        logging.info(
            "DETECT-ONLY: skipped battery-protection threshold register writes "
            "(regs %d/%d, would be %smV) — read-only mode.",
            REG_BAT_PROTECT_LOW,
            REG_BAT_PROTECT_HIGH,
            BATTERY_PROTECTION_MV,
        )
    else:
        protect_low = BATTERY_PROTECTION_MV & 0xFF
        protect_high = (BATTERY_PROTECTION_MV >> 8) & 0xFF
        low_ok = write_reg_verified(bus, device_addr, REG_BAT_PROTECT_LOW, protect_low)
        high_ok = write_reg_verified(bus, device_addr, REG_BAT_PROTECT_HIGH, protect_high)
        if low_ok and high_ok:
            try:
                low_read = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_LOW)
                high_read = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_HIGH)
                protect_readback = (high_read << 8) | low_read
                logging.info("Battery protection threshold set/readback: %smV", protect_readback)
            except Exception as error:
                logging.warning("Battery protection threshold set but readback failed: %s", error)
        else:
            logging.warning(
                "Failed to set battery protection threshold to %smV after retries.",
                BATTERY_PROTECTION_MV,
            )

    seconds_without_charge = 0
    boot_elapsed = 0
    last_ac_present = None
    tracker = AcPresenceTracker(AC_LOSS_DEBOUNCE_S)
    consecutive_i2c_errors = 0

    while True:
        try:
            typec_mv, microusb_mv = read_charger_voltages(bus, device_addr)
            consecutive_i2c_errors = 0
        except Exception as error:
            consecutive_i2c_errors += 1
            logging.warning(
                "I2C read error (%d consecutive): %s", consecutive_i2c_errors, error
            )
            # Back off proportionally so a persistent bus fault doesn't
            # busy-loop, but don't let a couple of transient errors change
            # any state — we simply skip this sample and try again.
            time.sleep(min(2.0 * consecutive_i2c_errors, 10.0))
            continue

        now = time.monotonic()
        ac_present = tracker.update(typec_mv, microusb_mv, now)

        if boot_elapsed < BOOT_GRACE_SECONDS:
            boot_elapsed += 1
            time.sleep(1)
            continue

        if last_ac_present is None:
            last_ac_present = ac_present
            logging.info("UPS AC state initialized: %s", "present" if ac_present else "missing")
        elif ac_present != last_ac_present:
            last_ac_present = ac_present
            try:
                snap = read_ups_snapshot(bus, device_addr, ina_batt)
                logging.info(
                    "UPS AC state changed -> %s (typec=%smV microusb=%smV batt=%sV %smA)",
                    "present" if ac_present else "missing",
                    snap["typec_mv"],
                    snap["microusb_mv"],
                    snap["battery_v"],
                    snap["battery_i_ma"],
                )
            except Exception as error:
                logging.info("UPS AC state changed -> %s (snapshot failed: %s)", ac_present, error)

        if ac_present:
            if seconds_without_charge != 0:
                logging.info("Charger present again; resetting grace timer.")
            seconds_without_charge = 0
        else:
            seconds_without_charge += 1
            if seconds_without_charge == 1:
                logging.info(
                    "Charger absent (debounced over %ss); starting %ss glitch grace window.",
                    AC_LOSS_DEBOUNCE_S,
                    NO_CHARGE_GRACE_SECONDS,
                )
            if seconds_without_charge == NO_CHARGE_GRACE_SECONDS:
                # Grace expired with the charger still gone. In normal mode
                # this halts the Pi and never returns; in detect-only mode
                # every action is suppressed at its own call site and this
                # returns so we keep monitoring for a replug (repeatable).
                run_shutdown_sequence(
                    bus, device_addr, ina_batt, detect_only=detect_only
                )

        # Best-effort status publish for the web debug board. Fully isolated —
        # cannot raise into this loop and does not consult or change any state
        # used by the AC-detection / shutdown decisions above.
        _publish_status(
            bus, device_addr, ina_batt,
            ac_present=ac_present,
            typec_mv=typec_mv,
            microusb_mv=microusb_mv,
            seconds_without_charge=seconds_without_charge,
            detect_only=detect_only,
        )

        time.sleep(1)


if __name__ == "__main__":
    main()
