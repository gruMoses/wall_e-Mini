#!/usr/bin/env python3
"""I2C bus 1 auto-unlock guard for the Raspberry Pi 5.

WHY THIS EXISTS (incident 2026-09-25 21:36:16): the kernel logged
    i2c_designware 1f00074000.i2c: i2c_dw_handle_tx_abort: SDA stuck at low
A device on I2C bus 1 froze mid-transfer and held SDA low. Both devices on
that bus went dark for 17 hours until a human power-cycled the robot: the
RTK GPS co-processor (address 0x20, read by the wall-e service) and the
UPSPlus HAT (read by upsPlus_power_daemon.py / upsplus-power.service, which
triggers a graceful shutdown on real power loss). The UPS daemon logged
"WARNING: I2C read error (N consecutive)" every ~10 s the whole time and its
/tmp/ups_status.json status file went stale, but nothing on the Pi 5 clears
a stuck bus on its own -- there is no GPIO bus-recovery configured for this
controller.

WHAT THIS SCRIPT DOES

Every --interval seconds (default 5):
  1. Reads the age of the UPS daemon's status file (its `ts` field). A
     missing or unreadable file counts as age UNKNOWN, not stale -- the UPS
     daemon might simply be stopped, and that alone must never trigger a
     bus recovery.
  2. If (and only if) that age is stale, samples SDA (GPIO2) --low-samples
     times, about 1 s apart, parsed from `pinctrl get 2`.
  3. Calls the bus "stuck" only when BOTH the UPS status is stale AND every
     sample in that burst reads low. A single non-low sample breaks the
     streak -- ordinary I2C traffic never looks like this for 3+ seconds.
  4. On stuck, runs the standard I2C bus-clear recovery (see recover_bus):
     unbind the i2c-designware driver, manually clock SCL to walk a wedged
     slave off SDA, emit a STOP, restore both pins to their I2C alt
     function, rebind. Pin restore and rebind always run, even if a step
     in between raises. Every pin drive is open-drain emulation -- a line
     is either driven LOW or released to input-with-pull-up, never driven
     HIGH push-pull, so this can never fight a slave that is still holding
     a line low.
  5. Recoveries are capped at --max-per-hour (default 6) in a rolling
     window, so a bus that will not stay fixed cannot be pulsed forever.
  6. A successful recovery restarts upsplus-power.service (unless
     --no-restart-ups): unbinding/rebinding the adapter invalidates the
     smbus2.SMBus handle that daemon opened at startup, so without a
     restart it stays blind even after the bus is clear. wall-e.service is
     deliberately never restarted here -- see docs/i2c_bus_guard.md for why,
     and for the one case that still needs a person.
  7. Independently of whether the bus is stuck, a UPS status that is stale
     for any reason (its daemon crashed, was stopped, anything) gets its
     own throttled warning -- a stuck bus is only one way the UPS monitor
     can go blind.

All hardware access -- pinctrl, the sysfs unbind/bind files, and the UPS
service restart -- goes through the Hardware class below. Detection
(detect_stuck) and recovery (recover_bus) are plain functions over that
interface, so tests exercise them against a fake with no real GPIO/
subprocess/sysfs involved. See pi_app/tests/test_i2c_bus_guard.py and
docs/i2c_bus_guard.md.
"""

from __future__ import annotations

import argparse
import collections
import json
import logging
import os
import re
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# --- Hardware facts (Raspberry Pi 5, this robot) ----------------------------
SDA_PIN = 2   # GPIO2 = SDA1
SCL_PIN = 3   # GPIO3 = SCL1
DEVICE_NAME = "1f00074000.i2c"
_DRIVER_DIR = "/sys/bus/platform/drivers/i2c_designware"
UNBIND_PATH = f"{_DRIVER_DIR}/unbind"
BIND_PATH = f"{_DRIVER_DIR}/bind"
UPS_SERVICE_NAME = "upsplus-power.service"

# --- Recovery shape ----------------------------------------------------------
MAX_PULSES = 16
PULSE_HALF_PERIOD_S = 0.01
POST_REBIND_WAIT_S = 2.0

# --- Detection shape ---------------------------------------------------------
SAMPLE_INTERVAL_S = 1.0

# --- Rate limiting / log hygiene ---------------------------------------------
RATE_LIMIT_WINDOW_S = 3600.0
WARNING_REPEAT_S = 300.0

# --- CLI defaults -------------------------------------------------------------
DEFAULT_INTERVAL_S = 5.0
DEFAULT_STALE_S = 30.0
DEFAULT_LOW_SAMPLES = 3
DEFAULT_MAX_PER_HOUR = 6
DEFAULT_STATUS_FILE = "/tmp/i2c_guard_status.json"
DEFAULT_UPS_STATUS_FILE = os.environ.get("UPS_STATUS_FILE", "/tmp/ups_status.json")

# `pinctrl get 2` prints a line shaped like:
#      2: a3    pu | hi // GPIO2 = SDA1
_PINCTRL_LEVEL_RE = re.compile(r"\|\s*(hi|lo)\b", re.IGNORECASE)


def parse_pinctrl_level(output: str) -> str:
    """Parse one `pinctrl get <N>` line into "hi", "lo", or "unknown".

    Anything that is not a clean hi/lo match -- empty output, an error
    message, an unexpected format from a future pinctrl version -- comes
    back as "unknown" so a parsing surprise is never mistaken for a
    stuck-low reading.
    """
    match = _PINCTRL_LEVEL_RE.search(output)
    if not match:
        return "unknown"
    return match.group(1).lower()


class Hardware:
    """The only place this script touches real hardware.

    Five methods: read_sda() (pinctrl get, read-only), set_pin() (pinctrl
    set), unbind()/bind() (sysfs driver bind control), and
    restart_ups_service() (systemctl, so the UPS daemon reopens its I2C
    handle after a recovery). Tests replace this whole class with a fake;
    detect_stuck() and recover_bus() never touch subprocess or sysfs
    directly.

    dry_run logs every mutating action instead of performing it. Reads
    (read_sda) are always real -- a `pinctrl get` is non-destructive, and
    seeing the true pin state is what makes a dry run useful for manual
    testing on the robot.
    """

    def __init__(self, dry_run: bool = False) -> None:
        self.dry_run = dry_run

    def read_sda(self) -> str:
        try:
            result = subprocess.run(
                ["pinctrl", "get", str(SDA_PIN)],
                capture_output=True,
                text=True,
                timeout=5,
                check=True,
            )
        except Exception as error:
            logger.debug("pinctrl get %d failed: %s", SDA_PIN, error)
            return "unknown"
        return parse_pinctrl_level(result.stdout)

    def set_pin(self, pin: int, *mode: str) -> None:
        if self.dry_run:
            logger.info("DRY-RUN: pinctrl set %d %s", pin, " ".join(mode))
            return
        subprocess.run(
            ["pinctrl", "set", str(pin), *mode], timeout=5, check=True
        )

    def unbind(self) -> None:
        if self.dry_run:
            logger.info("DRY-RUN: unbind %s", DEVICE_NAME)
            return
        self._write_sysfs(UNBIND_PATH, DEVICE_NAME)

    def bind(self) -> None:
        if self.dry_run:
            logger.info("DRY-RUN: bind %s", DEVICE_NAME)
            return
        self._write_sysfs(BIND_PATH, DEVICE_NAME)

    @staticmethod
    def _write_sysfs(path: str, value: str) -> None:
        with open(path, "w") as f:
            f.write(value)

    def restart_ups_service(self) -> None:
        """Restart upsplus-power.service so it reopens its I2C handle.

        Unbinding/rebinding the i2c_designware driver invalidates any
        smbus2.SMBus handle opened before the unbind. The UPS daemon opens
        its handle once at startup and only ever backs off and retries on
        read errors -- it never reopens the handle itself -- so without
        this restart it stays blind to real power loss even after the bus
        is clear again.

        Best-effort: unlike unbind()/bind()/set_pin(), a failure here is
        logged and swallowed, never raised. A failed UPS restart must
        never be mistaken for a failed bus recovery -- the bus is already
        clear by the time this runs.
        """
        if self.dry_run:
            logger.info("DRY-RUN: systemctl restart %s", UPS_SERVICE_NAME)
            return
        try:
            subprocess.run(
                ["systemctl", "restart", UPS_SERVICE_NAME], timeout=15, check=True
            )
        except Exception as error:
            logger.error("Failed to restart %s: %s", UPS_SERVICE_NAME, error)


def read_ups_age_s(path: str, now: float) -> float | None:
    """Age, in seconds, of the UPS daemon's status file `ts` field.

    Returns None when the file is missing, unreadable, or malformed --
    treated by the caller as NOT stale, because the UPS daemon might
    simply be stopped rather than blind on a stuck bus.
    """
    try:
        with open(path, "r") as f:
            payload = json.load(f)
        ts = float(payload["ts"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return now - ts


def is_ups_stale(ups_age_s: float | None, stale_s: float) -> bool:
    """True only when the UPS status age is known and exceeds stale_s.

    An unknown age (missing/unreadable status file) is NOT stale -- the
    UPS daemon might simply be stopped, not blind on a stuck bus.
    """
    return ups_age_s is not None and ups_age_s > stale_s


def sample_sda_levels(
    hw: Hardware,
    count: int,
    interval_s: float = SAMPLE_INTERVAL_S,
    sleep_fn=time.sleep,
) -> list[str]:
    """Read SDA up to `count` times, ~interval_s apart.

    Stops early -- returning fewer than `count` samples -- the moment a
    non-"lo" reading appears, since that already breaks the low streak the
    caller is checking for. There is no point burning the rest of the
    burst once one high sample has reset it.
    """
    samples: list[str] = []
    for i in range(count):
        if i > 0:
            sleep_fn(interval_s)
        level = hw.read_sda()
        samples.append(level)
        if level != "lo":
            break
    return samples


def is_low_streak(samples: list[str], count: int) -> bool:
    """True only if `samples` is exactly `count` long and every one is "lo"."""
    return len(samples) == count and all(s == "lo" for s in samples)


def detect_stuck(
    hw: Hardware,
    *,
    ups_age_s: float | None,
    stale_s: float,
    low_samples: int,
    sample_interval_s: float = SAMPLE_INTERVAL_S,
    sleep_fn=time.sleep,
) -> tuple[bool, list[str]]:
    """Evaluate this cycle's stuck-bus condition. Returns (stuck, sda_samples).

    Both conditions are required: the UPS status must be stale (age >
    stale_s) AND SDA must read low for `low_samples` consecutive ~1 s
    samples. Either alone -- normal bus traffic holding SDA low briefly, or
    a UPS daemon that is simply not running -- must never trigger a
    recovery. SDA sampling is skipped once the UPS is not stale, since the
    result cannot change the outcome; this keeps a healthy cycle fast.
    """
    if not is_ups_stale(ups_age_s, stale_s):
        return False, [hw.read_sda()]
    samples = sample_sda_levels(hw, low_samples, sample_interval_s, sleep_fn=sleep_fn)
    return is_low_streak(samples, low_samples), samples


def recover_bus(
    hw: Hardware,
    *,
    max_pulses: int = MAX_PULSES,
    pulse_half_period_s: float = PULSE_HALF_PERIOD_S,
    post_rebind_wait_s: float = POST_REBIND_WAIT_S,
    sleep_fn=time.sleep,
) -> tuple[bool, str | None]:
    """Standard I2C bus-clear recovery. Returns (ok, error).

    Open-drain emulation throughout: a line is only ever driven LOW
    ("op", "dl") or released to input-with-pull-up ("ip", "pu"), which the
    pull-up reads back as high. A line is NEVER driven high push-pull
    ("op", "dh") -- if a wedged slave is still holding that line low, a
    push-pull high output would fight it on the same wire. This matches
    how every real I2C driver behaves and is why a stuck bus is normally
    recoverable at all.

    Sequence: unbind the driver; release SCL and SDA (the normal idle-bus
    wiring, so a wedged slave can still pull SDA low); pulse SCL low then
    released up to `max_pulses` times, checking SDA after each release
    and stopping as soon as it reads high; emit a textbook STOP (drive SCL
    low, drive SDA low, release SCL, release SDA -- SDA then transitions
    low-to-high while SCL is high, the STOP condition); restore both pins
    to the I2C alt function; rebind the driver.

    Pin restore and rebind run in a `finally` block, so they happen even
    if a step in between raises -- a half-finished recovery must never
    strand the pins in a non-I2C GPIO mode or leave the driver unbound.
    Any exception anywhere is reported as a failure (ok=False); only when
    nothing raised do we wait `post_rebind_wait_s` and check SDA one more
    time to decide the final result.
    """
    error: str | None = None
    try:
        hw.unbind()
        hw.set_pin(SCL_PIN, "ip", "pu")
        hw.set_pin(SDA_PIN, "ip", "pu")
        for _ in range(max_pulses):
            hw.set_pin(SCL_PIN, "op", "dl")
            sleep_fn(pulse_half_period_s)
            hw.set_pin(SCL_PIN, "ip", "pu")
            sleep_fn(pulse_half_period_s)
            if hw.read_sda() == "hi":
                break
        # Textbook STOP. SCL is released (high, via its pull-up) at the
        # end of the loop above, so SCL is driven low FIRST here -- driving
        # SDA low while SCL is still high would look like a START, not a
        # STOP. Then SDA low, then release SCL (goes high), then release
        # SDA: SDA transitions low-to-high while SCL is high == STOP.
        hw.set_pin(SCL_PIN, "op", "dl")
        sleep_fn(pulse_half_period_s)
        hw.set_pin(SDA_PIN, "op", "dl")
        sleep_fn(pulse_half_period_s)
        hw.set_pin(SCL_PIN, "ip", "pu")
        sleep_fn(pulse_half_period_s)
        hw.set_pin(SDA_PIN, "ip", "pu")
        sleep_fn(pulse_half_period_s)
    except Exception as exc:
        error = str(exc)
    finally:
        for pin in (SCL_PIN, SDA_PIN):
            try:
                hw.set_pin(pin, "a3", "pu")
            except Exception as exc:
                error = error or str(exc)
        try:
            hw.bind()
        except Exception as exc:
            error = error or str(exc)

    if error is not None:
        return False, error

    sleep_fn(post_rebind_wait_s)
    level = hw.read_sda()
    if level == "hi":
        return True, None
    return False, f"SDA still {level} after recovery"


class RecoveryRateLimiter:
    """At most `max_per_hour` recoveries in any trailing window.

    Pure logic over a caller-supplied clock (pass time.monotonic() values)
    so it is unit-testable with no real waiting. Mirrors the hardware-free
    design of AcPresenceTracker in scripts/upsPlus_power_daemon.py.
    """

    def __init__(self, max_per_hour: int, window_s: float = RATE_LIMIT_WINDOW_S) -> None:
        self._max_per_hour = max_per_hour
        self._window_s = window_s
        self._events: collections.deque[float] = collections.deque()

    def _prune(self, now: float) -> None:
        while self._events and now - self._events[0] >= self._window_s:
            self._events.popleft()

    def count(self, now: float) -> int:
        self._prune(now)
        return len(self._events)

    def allow(self, now: float) -> bool:
        """True if a recovery may run now without exceeding the cap."""
        return self.count(now) < self._max_per_hour

    def record(self, now: float) -> None:
        self._prune(now)
        self._events.append(now)


class _Throttle:
    """Suppresses a repeated log key to at most once per `period_s`."""

    def __init__(self, period_s: float) -> None:
        self._period_s = period_s
        self._last_at: dict[str, float] = {}

    def ready(self, key: str, now: float) -> bool:
        last = self._last_at.get(key)
        if last is not None and (now - last) < self._period_s:
            return False
        self._last_at[key] = now
        return True


def write_status_file(path: str, status: dict) -> None:
    """Best-effort atomic publish of the guard's status snapshot.

    Write-temp-then-os.replace means a concurrent reader (`cat`, a future
    dashboard card) never observes a half-written file. tempfile.mkstemp
    opens with O_EXCL under an unpredictable name in the same directory,
    so a pre-planted symlink or file at a guessed temp path is refused
    rather than followed -- the same concern write_status_file in
    scripts/upsPlus_power_daemon.py documents in more detail, since this
    guard is also a root daemon writing into a world-writable /tmp.

    Any failure is logged and swallowed: a skipped status publish must
    never stop the guard from doing its actual job.
    """
    tmp = None
    try:
        data = json.dumps(status, indent=2).encode("utf-8")
        directory = os.path.dirname(path) or "."
        fd, tmp = tempfile.mkstemp(
            prefix=os.path.basename(path) + ".tmp.", dir=directory
        )
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
            tmp = None
        finally:
            if tmp is not None:
                os.unlink(tmp)
    except Exception:
        logger.debug("i2c guard status write failed (non-fatal)", exc_info=True)


@dataclass
class GuardConfig:
    interval_s: float = DEFAULT_INTERVAL_S
    stale_s: float = DEFAULT_STALE_S
    low_samples: int = DEFAULT_LOW_SAMPLES
    max_per_hour: int = DEFAULT_MAX_PER_HOUR
    status_file: str = DEFAULT_STATUS_FILE
    ups_status_file: str = DEFAULT_UPS_STATUS_FILE
    dry_run: bool = False
    restart_ups: bool = True


class I2cBusGuard:
    """Ties detection, recovery, rate limiting, and status publishing
    together for one cycle. `now_fn`/`monotonic_fn`/`sleep_fn` are
    injectable so tests run every cycle instantly against a fake clock."""

    def __init__(
        self,
        hw: Hardware,
        cfg: GuardConfig,
        *,
        now_fn=time.time,
        monotonic_fn=time.monotonic,
        sleep_fn=time.sleep,
        warning_repeat_s: float = WARNING_REPEAT_S,
    ) -> None:
        self._hw = hw
        self._cfg = cfg
        self._now = now_fn
        self._monotonic = monotonic_fn
        self._sleep = sleep_fn
        self._limiter = RecoveryRateLimiter(cfg.max_per_hour)
        self._throttle = _Throttle(warning_repeat_s)
        self._rate_limit_warned = False
        self._ups_unknown_logged = False

        self.recoveries_total = 0
        self.last_recovery_ts: float | None = None
        self.last_result: str | None = None
        self.last_error: str | None = None

    def run_cycle(self) -> dict:
        now_wall = self._now()
        now_mono = self._monotonic()

        ups_age = read_ups_age_s(self._cfg.ups_status_file, now_wall)
        ups_stale = is_ups_stale(ups_age, self._cfg.stale_s)
        self._warn_if_ups_blind(now_mono, ups_age, ups_stale)

        stuck, samples = detect_stuck(
            self._hw,
            ups_age_s=ups_age,
            stale_s=self._cfg.stale_s,
            low_samples=self._cfg.low_samples,
            sleep_fn=self._sleep,
        )
        sda_level = samples[-1] if samples else "unknown"

        if stuck:
            self._handle_stuck(now_wall, now_mono, ups_age, samples)
        else:
            self._rate_limit_warned = False

        status = {
            "ts": now_wall,
            "sda_level": sda_level,
            "ups_age_s": round(ups_age, 2) if ups_age is not None else None,
            "ups_stale": ups_stale,
            "stuck": stuck,
            "recoveries_total": self.recoveries_total,
            "last_recovery_ts": self.last_recovery_ts,
            "last_result": self.last_result,
            "last_error": self.last_error,
        }
        write_status_file(self._cfg.status_file, status)
        return status

    def _warn_if_ups_blind(
        self, now_mono: float, ups_age: float | None, ups_stale: bool
    ) -> None:
        """UPS-blind alert for ANY cause -- independent of whether the bus
        is stuck. A stuck bus is only one way the UPS daemon can go blind
        (it might also just be stopped or crashed); either way, a stale
        status means safe shutdown on real power loss is not active, and
        that is worth a warning even when SDA looks fine.

        Missing/unreadable status (age unknown) is logged once, at INFO,
        for the life of this guard -- it may simply mean no UPS hardware
        is fitted, so it is not worth repeating. A genuinely stale age is
        logged at WARNING, throttled to once per five minutes like the
        other repeating warnings.
        """
        if ups_age is None:
            if not self._ups_unknown_logged:
                logger.info(
                    "UPS status file unreadable or missing (%s); age unknown, "
                    "not treated as stale.",
                    self._cfg.ups_status_file,
                )
                self._ups_unknown_logged = True
            return
        if ups_stale and self._throttle.ready("ups_stale", now_mono):
            logger.warning(
                "UPS status stale for %.1f s -- safe shutdown on power loss "
                "is NOT active",
                ups_age,
            )

    def _handle_stuck(
        self, now_wall: float, now_mono: float, ups_age: float | None, samples: list[str]
    ) -> None:
        if self._throttle.ready("stuck", now_mono):
            logger.warning(
                "I2C bus 1 stuck: SDA low x%d (%s), UPS status age %.1fs (> %.1fs)",
                len(samples), ", ".join(samples), ups_age, self._cfg.stale_s,
            )
            logger.warning(
                "UPS safe-shutdown is blind while the bus is stuck -- the UPS "
                "daemon cannot read the MCU until this clears."
            )

        if not self._limiter.allow(now_mono):
            if not self._rate_limit_warned:
                logger.error(
                    "I2C bus stuck but %d recoveries already ran in the last "
                    "hour (cap %d); holding off until the window frees.",
                    self._limiter.count(now_mono), self._cfg.max_per_hour,
                )
                self._rate_limit_warned = True
            return

        self._rate_limit_warned = False
        self._run_recovery(now_wall, now_mono)

    def _run_recovery(self, now_wall: float, now_mono: float) -> None:
        logger.warning("Attempting I2C bus recovery.")
        if self._cfg.dry_run:
            logger.info(
                "DRY-RUN: recovery actions logged only; bus and driver untouched."
            )
        ok, error = recover_bus(self._hw, sleep_fn=self._sleep)

        self._limiter.record(now_mono)
        self.recoveries_total += 1
        self.last_recovery_ts = now_wall
        self.last_result = "ok" if ok else "failed"
        self.last_error = error

        if ok:
            logger.info("I2C bus recovery succeeded; SDA is high.")
            if self._cfg.restart_ups:
                logger.warning(
                    "Restarting %s so it reopens its I2C handle after the "
                    "bus recovery.",
                    UPS_SERVICE_NAME,
                )
                self._hw.restart_ups_service()
        else:
            logger.error("I2C bus recovery failed: %s", error)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="I2C bus 1 auto-unlock guard (see docs/i2c_bus_guard.md)."
    )
    parser.add_argument(
        "--interval", type=float, default=DEFAULT_INTERVAL_S,
        help=f"seconds between cycles (default: {DEFAULT_INTERVAL_S})",
    )
    parser.add_argument(
        "--stale-s", type=float, default=DEFAULT_STALE_S, dest="stale_s",
        help=f"UPS status age, in seconds, that counts as stale (default: {DEFAULT_STALE_S})",
    )
    parser.add_argument(
        "--low-samples", type=int, default=DEFAULT_LOW_SAMPLES, dest="low_samples",
        help=(
            "consecutive low SDA samples, ~1s apart, required to call the "
            f"bus stuck (default: {DEFAULT_LOW_SAMPLES})"
        ),
    )
    parser.add_argument(
        "--max-per-hour", type=int, default=DEFAULT_MAX_PER_HOUR, dest="max_per_hour",
        help=f"recovery attempts allowed per rolling hour (default: {DEFAULT_MAX_PER_HOUR})",
    )
    parser.add_argument(
        "--status-file", default=DEFAULT_STATUS_FILE, dest="status_file",
        help=f"path for this guard's own status JSON (default: {DEFAULT_STATUS_FILE})",
    )
    parser.add_argument(
        "--ups-status-file", default=DEFAULT_UPS_STATUS_FILE, dest="ups_status_file",
        help=(
            "path to the UPS daemon's status JSON "
            "(default: $UPS_STATUS_FILE or /tmp/ups_status.json)"
        ),
    )
    parser.add_argument(
        "--no-restart-ups", dest="restart_ups", action="store_false", default=True,
        help=(
            "do not restart upsplus-power.service after a successful "
            "recovery (default: restart it, since unbind/rebind invalidates "
            "its I2C handle)"
        ),
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="log every hardware action instead of performing it",
    )
    parser.add_argument(
        "--once", action="store_true",
        help="run a single cycle then exit, instead of looping forever",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
        stream=sys.stdout,
    )

    cfg = GuardConfig(
        interval_s=args.interval,
        stale_s=args.stale_s,
        low_samples=args.low_samples,
        max_per_hour=args.max_per_hour,
        status_file=args.status_file,
        ups_status_file=args.ups_status_file,
        dry_run=args.dry_run,
        restart_ups=args.restart_ups,
    )
    if cfg.dry_run:
        logger.warning("DRY-RUN: no hardware will be changed.")

    hw = Hardware(dry_run=cfg.dry_run)
    guard = I2cBusGuard(hw, cfg)

    while True:
        try:
            guard.run_cycle()
        except Exception:
            logger.exception("i2c bus guard cycle failed unexpectedly")
        if args.once:
            break
        time.sleep(cfg.interval_s)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
