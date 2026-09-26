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
  1. Self-heals first (cheap): if the i2c_designware driver is unbound, or
     either pin has drifted off the I2C (a3) alt function, that is the
     signature of a kill (SIGKILL, or a SIGTERM before this guard installed
     its handler) that hit recover_bus mid-sequence and skipped its
     `finally` block. Restore both pins to a3 pu, bind if needed, log
     WARNING, and apply the UPS-restart rule below if it rebound the
     driver. This runs at the start of every cycle, so it doubles as the
     startup check -- the first cycle IS startup.
  2. Reads the age of the UPS daemon's status file (its `ts` field). A
     missing or unreadable file counts as age UNKNOWN, not stale -- the UPS
     daemon might simply be stopped, and that alone must never trigger a
     bus recovery.
  3. If (and only if) that age is stale, samples SDA and SCL TOGETHER --
     one `pinctrl get 2,3` call per sample -- --sample-count times (default
     10), about --sample-interval-s apart (default 0.1s).
  4. Calls the bus "stuck" only when BOTH the UPS status is stale AND every
     joint sample has SDA low AND SCL high. A wedged slave holds SDA low
     with the clock idle high; a live transfer toggles SCL, and a released
     bus reads SDA high -- either one breaks the streak immediately.
     Ordinary I2C traffic never looks like this for a full second.
  5. On stuck, runs the standard I2C bus-clear recovery (see recover_bus):
     unbind the i2c-designware driver, manually clock SCL to walk a wedged
     slave off SDA, emit a STOP, restore both pins to their I2C alt
     function, rebind. Pin restore and rebind always run, even if a step
     in between raises. Every pin drive is open-drain emulation -- a line
     is either driven LOW or released to input-with-pull-up, never driven
     HIGH push-pull, so this can never fight a slave that is still holding
     a line low. Success is 3 SDA samples 200ms apart with at least 2 high
     -- one sample can land inside a live transfer the instant a client
     reopens its handle and starts talking again.
  6. Recoveries are capped at --max-per-hour (default 6) in a rolling
     window, so a bus that will not stay fixed cannot be pulsed forever.
  7. A SIGTERM or SIGINT only sets a flag; the loop notices it after the
     current cycle finishes and exits cleanly, rather than the process
     dying mid-recovery and skipping recover_bus's `finally` (that is what
     step 1's self-heal repairs on a later cycle if it ever happens anyway
     -- SIGKILL cannot be caught at all).
  8. UPS daemon restart: unbinding the adapter invalidates the UPS
     daemon's only I2C file handle, and it never reopens that handle on
     its own. After ANY bus recovery attempt (success or failure), or any
     self-heal that rebound the driver, once the driver is bound again,
     this guard restarts upsplus-power.service (unless --no-restart-ups).
     wall-e.service is deliberately never restarted here -- see
     docs/i2c_bus_guard.md for why, and for the one case that still needs
     a person.
  9. UPS safety net: if the UPS status has been stale for more than 120s
     while the bus itself looks healthy (SDA high, SCL high -- not
     stuck), a wedged bus cannot be the cause, so this guard restarts the
     UPS daemon anyway, on the chance that fixes it. Rate-limited to once
     per 15 minutes. A missing status file never triggers this.
  10. Independently of whether the bus is stuck, a UPS status that is
      stale for any reason (its daemon crashed, was stopped, anything)
      gets its own throttled warning -- a stuck bus is only one way the
      UPS monitor can go blind.

All hardware access -- pinctrl, the sysfs unbind/bind files, and the UPS
service restart -- goes through the Hardware class below. Detection
(detect_stuck), recovery (recover_bus), and self-heal (self_heal) are plain
functions over that interface, so tests exercise them against a fake with
no real GPIO/subprocess/sysfs involved. See pi_app/tests/test_i2c_bus_guard.py
and docs/i2c_bus_guard.md.
"""

from __future__ import annotations

import argparse
import collections
import json
import logging
import os
import re
import signal
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
DEVICE_LINK_PATH = f"{_DRIVER_DIR}/{DEVICE_NAME}"  # symlink present only while bound
UPS_SERVICE_NAME = "upsplus-power.service"

# --- Recovery shape ----------------------------------------------------------
MAX_PULSES = 16
PULSE_HALF_PERIOD_S = 0.01
POST_REBIND_WAIT_S = 2.0
SUCCESS_CHECK_SAMPLES = 3
SUCCESS_CHECK_INTERVAL_S = 0.2
SUCCESS_CHECK_MIN_HI = 2

# --- Detection shape ---------------------------------------------------------
DEFAULT_SAMPLE_COUNT = 10
DEFAULT_SAMPLE_INTERVAL_S = 0.1

# --- UPS safety net (rule B, stale-with-a-healthy-bus) -----------------------
UPS_SAFETY_NET_STALE_S = 120.0
UPS_SAFETY_NET_WINDOW_S = 900.0  # 15 minutes

# --- Rate limiting / log hygiene ---------------------------------------------
RATE_LIMIT_WINDOW_S = 3600.0
WARNING_REPEAT_S = 300.0

# --- CLI defaults -------------------------------------------------------------
DEFAULT_INTERVAL_S = 5.0
DEFAULT_STALE_S = 30.0
DEFAULT_MAX_PER_HOUR = 6
DEFAULT_STATUS_FILE = "/tmp/i2c_guard_status.json"
DEFAULT_UPS_STATUS_FILE = os.environ.get("UPS_STATUS_FILE", "/tmp/ups_status.json")

# `pinctrl get 2` prints a line shaped like:
#      2: a3    pu | hi // GPIO2 = SDA1
_PINCTRL_LEVEL_RE = re.compile(r"\|\s*(hi|lo)\b", re.IGNORECASE)

# `pinctrl get 2,3` prints one such line per requested pin. This regex
# additionally captures the pin number and its function (e.g. "a3"), for
# the joint SDA+SCL reads self-heal and detection both need.
# Real Pi 5 formats (2026-09-26): " 2: a3    pu | hi // GPIO2 = SDA1",
# " 0: ip    pu | hi // ...", " 7: op dh pu | hi // GPIO7 = output". The
# function is the first token after "N:"; anything else before "|" (drive,
# pull) is skipped.
_PINCTRL_LINE_RE = re.compile(
    r"^\s*(?P<pin>\d+):\s*(?P<function>\S+)[^|\n]*\|\s*(?P<level>hi|lo)\b",
    re.IGNORECASE | re.MULTILINE,
)


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


@dataclass(frozen=True)
class PinReading:
    function: str  # e.g. "a3" (I2C alt function), "ip", "op", or "unknown"
    level: str      # "hi", "lo", or "unknown"


def parse_pinctrl_pins(output: str, pins: tuple[int, ...]) -> dict[int, PinReading]:
    """Parse `pinctrl get <pins...>` output (one line per pin) into
    {pin_number: PinReading}.

    A requested pin whose line is missing or does not parse cleanly still
    gets an entry -- PinReading("unknown", "unknown") -- so callers never
    need a second "was this pin even in the output" check.
    """
    found: dict[int, PinReading] = {}
    for match in _PINCTRL_LINE_RE.finditer(output):
        pin = int(match.group("pin"))
        found[pin] = PinReading(
            match.group("function").lower(), match.group("level").lower()
        )
    return {pin: found.get(pin, PinReading("unknown", "unknown")) for pin in pins}


class Hardware:
    """The only place this script touches real hardware.

    read_sda() and read_pins() are read-only (`pinctrl get`) and always
    real, even in dry-run -- a read is non-destructive, and seeing the
    true pin state is what makes a dry run useful for manual testing on
    the robot. is_bound() is a plain, always-real filesystem check for
    the same reason. set_pin() / unbind() / bind() / restart_ups_service()
    are the mutating actions, each dry-run aware (log the action, change
    nothing). Tests replace this whole class with a fake; detect_stuck(),
    recover_bus() and self_heal() never touch subprocess or sysfs
    directly.
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

    def read_pins(self) -> dict[int, PinReading]:
        """One `pinctrl get 2,3` call: function + level for SDA and SCL
        together. Used by self-heal's pin-function check and by the joint
        SDA+SCL stuck-detection sampling."""
        pins = (SDA_PIN, SCL_PIN)
        try:
            result = subprocess.run(
                ["pinctrl", "get", ",".join(str(p) for p in pins)],
                capture_output=True,
                text=True,
                timeout=5,
                check=True,
            )
        except Exception as error:
            logger.debug("pinctrl get %s failed: %s", pins, error)
            return {pin: PinReading("unknown", "unknown") for pin in pins}
        return parse_pinctrl_pins(result.stdout, pins)

    def is_bound(self) -> bool:
        """True if the i2c_designware driver is currently bound to this
        device. A plain filesystem check -- the driver directory holds a
        symlink named after the device only while bound."""
        return os.path.exists(DEVICE_LINK_PATH)

    def set_pin(self, pin: int, *mode: str) -> None:
        if self.dry_run:
            logger.info("DRY-RUN: pinctrl set %d %s", pin, " ".join(mode))
            return
        subprocess.run(
            ["pinctrl", "set", str(pin), *mode], timeout=5, check=True
        )

    def unbind(self) -> None:
        """Unbind the driver. A no-op when the device is already unbound
        -- e.g. self-heal running right after a kill left it that way --
        so this is always safe to call unconditionally."""
        if not self.is_bound():
            logger.debug("unbind: %s already unbound; skipping.", DEVICE_NAME)
            return
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


def sample_joint_levels(
    hw: Hardware,
    count: int,
    interval_s: float,
    sleep_fn=time.sleep,
) -> list[tuple[str, str]]:
    """Read (SDA, SCL) together, up to `count` times, ~interval_s apart.

    Stops early -- returning fewer than `count` samples -- the moment a
    sample does NOT have SDA low AND SCL high, since that already breaks
    the stuck-bus streak the caller is checking for. A live transfer
    toggles SCL; a released bus reads SDA high; either one ends the burst
    immediately, so a healthy or busy bus is recognized fast.
    """
    samples: list[tuple[str, str]] = []
    for i in range(count):
        if i > 0:
            sleep_fn(interval_s)
        pins = hw.read_pins()
        sample = (pins[SDA_PIN].level, pins[SCL_PIN].level)
        samples.append(sample)
        if not (sample[0] == "lo" and sample[1] == "hi"):
            break
    return samples


def is_stuck_streak(samples: list[tuple[str, str]], count: int) -> bool:
    """True only if `samples` is exactly `count` long and every sample has
    SDA low AND SCL high -- a wedged slave holding SDA low with the clock
    idle high. A single sample with SCL low (a live transfer) or SDA high
    (a released bus) already broke the streak in sample_joint_levels,
    which is why a short list here is never mistaken for "stuck".
    """
    return (
        len(samples) == count
        and all(sda == "lo" and scl == "hi" for sda, scl in samples)
    )


def detect_stuck(
    hw: Hardware,
    *,
    ups_age_s: float | None,
    stale_s: float,
    sample_count: int = DEFAULT_SAMPLE_COUNT,
    sample_interval_s: float = DEFAULT_SAMPLE_INTERVAL_S,
    sleep_fn=time.sleep,
) -> tuple[bool, list[tuple[str, str]]]:
    """Evaluate this cycle's stuck-bus condition.

    Returns (stuck, joint_samples), where each sample is (sda_level,
    scl_level). Both conditions are required: the UPS status must be
    stale (age > stale_s) AND every one of `sample_count` joint SDA/SCL
    reads, ~sample_interval_s apart, must show SDA low AND SCL high.
    Either alone -- normal bus traffic, or a UPS daemon that is simply
    not running -- must never trigger a recovery. Sampling is skipped
    once the UPS is not stale, since the result cannot change the
    outcome; this keeps a healthy cycle fast, at the cost of one single
    joint read so the caller still has a current (sda, scl) reading for
    the status file and the UPS safety net.
    """
    if not is_ups_stale(ups_age_s, stale_s):
        pins = hw.read_pins()
        return False, [(pins[SDA_PIN].level, pins[SCL_PIN].level)]
    samples = sample_joint_levels(hw, sample_count, sample_interval_s, sleep_fn=sleep_fn)
    return is_stuck_streak(samples, sample_count), samples


def check_recovery_success(
    hw: Hardware,
    *,
    samples: int = SUCCESS_CHECK_SAMPLES,
    interval_s: float = SUCCESS_CHECK_INTERVAL_S,
    min_hi: int = SUCCESS_CHECK_MIN_HI,
    sleep_fn=time.sleep,
) -> tuple[bool, list[str]]:
    """SDA samples after rebind. Returns (ok, levels).

    A single low reading right after rebind is not necessarily a failed
    recovery -- it can land squarely inside a live transfer the instant a
    client (the RTK GPS reader, say) reopens its handle and starts
    talking again. A majority vote over a few samples absorbs that
    without weakening the check into "any high reading passes".
    """
    levels: list[str] = []
    for i in range(samples):
        if i > 0:
            sleep_fn(interval_s)
        levels.append(hw.read_sda())
    ok = sum(1 for level in levels if level == "hi") >= min_hi
    return ok, levels


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
    (A kill signal that skips this `finally` entirely -- SIGKILL, or a
    SIGTERM before this guard's handler is installed -- is repaired by
    self_heal() on a later cycle instead; there is nothing recover_bus
    itself can do about a signal Python never lets it react to.) Any
    exception anywhere is reported as a failure (ok=False); only when
    nothing raised do we wait `post_rebind_wait_s` and run
    check_recovery_success() to decide the final result.
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
    ok, levels = check_recovery_success(hw, sleep_fn=sleep_fn)
    if ok:
        return True, None
    return False, f"SDA samples after recovery: {levels}"


def is_driver_healthy(hw: Hardware) -> bool:
    """True if the driver is bound AND both pins are on the I2C (a3) alt
    function -- the state recover_bus's own finally block leaves things
    in when it runs to completion. False is the signature of a kill that
    hit recover_bus mid-sequence and skipped that finally block.
    """
    if not hw.is_bound():
        return False
    pins = hw.read_pins()
    # Only a pin POSITIVELY read in another function counts as drift. An
    # unreadable pinctrl line ("unknown") must not trigger a repair every
    # cycle on a healthy, busy bus.
    for pin in (SDA_PIN, SCL_PIN):
        function = pins[pin].function
        if function not in ("a3", "unknown"):
            return False
    return True


def self_heal(hw: Hardware) -> bool:
    """Idempotent repair for a half-finished recovery left by a kill.

    Restores both pins to a3 pu, then binds only if the driver was not
    already bound -- exactly steps 5 and 6 of recover_bus, the ones a
    kill can skip. Never unbinds: if the driver is already bound and only
    a pin drifted off a3 for some unrelated reason, unbinding first would
    needlessly interrupt live traffic to repair something that does not
    need it.

    Returns True only if this call actually transitioned the driver from
    unbound to bound -- the signal the caller uses for the UPS-restart
    rule (a pin drifting off a3 while the driver stayed bound never
    touched the UPS daemon's I2C handle, so that alone does not warrant a
    UPS restart).

    Swallows its own exceptions and logs them: a failed self-heal attempt
    must not crash the cycle, since the check that triggered this call
    reruns -- and retries -- at the start of every future cycle too.
    """
    was_bound = hw.is_bound()
    try:
        for pin in (SCL_PIN, SDA_PIN):
            hw.set_pin(pin, "a3", "pu")
        if not was_bound:
            hw.bind()
    except Exception:
        logger.exception("Self-heal step failed; will retry next cycle.")
    return (not was_bound) and hw.is_bound()


class RateLimiter:
    """At most `max_count` events in any trailing window.

    Pure logic over a caller-supplied clock (pass time.monotonic() values)
    so it is unit-testable with no real waiting. Mirrors the hardware-free
    design of AcPresenceTracker in scripts/upsPlus_power_daemon.py. Used
    for two independent limits: bus recoveries (--max-per-hour, an hour
    window) and the UPS safety-net restart (once per 15 minutes).
    """

    def __init__(self, max_count: int, window_s: float = RATE_LIMIT_WINDOW_S) -> None:
        self._max_count = max_count
        self._window_s = window_s
        self._events: collections.deque[float] = collections.deque()

    def _prune(self, now: float) -> None:
        while self._events and now - self._events[0] >= self._window_s:
            self._events.popleft()

    def count(self, now: float) -> int:
        self._prune(now)
        return len(self._events)

    def allow(self, now: float) -> bool:
        """True if an event may run now without exceeding the cap."""
        return self.count(now) < self._max_count

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
    sample_count: int = DEFAULT_SAMPLE_COUNT
    sample_interval_s: float = DEFAULT_SAMPLE_INTERVAL_S
    max_per_hour: int = DEFAULT_MAX_PER_HOUR
    status_file: str = DEFAULT_STATUS_FILE
    ups_status_file: str = DEFAULT_UPS_STATUS_FILE
    dry_run: bool = False
    restart_ups: bool = True


class I2cBusGuard:
    """Ties self-heal, detection, recovery, rate limiting, the UPS safety
    net, and status publishing together for one cycle. `now_fn`/
    `monotonic_fn`/`sleep_fn` are injectable so tests run every cycle
    instantly against a fake clock."""

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
        self._limiter = RateLimiter(cfg.max_per_hour, RATE_LIMIT_WINDOW_S)
        self._ups_safety_net_limiter = RateLimiter(1, UPS_SAFETY_NET_WINDOW_S)
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

        self._maybe_self_heal(now_mono)

        ups_age = read_ups_age_s(self._cfg.ups_status_file, now_wall)
        ups_stale = is_ups_stale(ups_age, self._cfg.stale_s)
        self._warn_if_ups_blind(now_mono, ups_age, ups_stale)

        stuck, samples = detect_stuck(
            self._hw,
            ups_age_s=ups_age,
            stale_s=self._cfg.stale_s,
            sample_count=self._cfg.sample_count,
            sample_interval_s=self._cfg.sample_interval_s,
            sleep_fn=self._sleep,
        )
        sda_level, scl_level = samples[-1] if samples else ("unknown", "unknown")

        if stuck:
            self._handle_stuck(now_wall, now_mono, ups_age, samples)
        else:
            self._rate_limit_warned = False
            self._maybe_ups_safety_net(now_mono, ups_age, ups_stale, sda_level, scl_level)

        status = {
            "ts": now_wall,
            "sda_level": sda_level,
            "scl_level": scl_level,
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

    def _maybe_self_heal(self, now_mono: float) -> bool:
        """Runs first in every cycle -- this doubles as the startup check,
        since the first cycle IS startup. Cheap when healthy: one sysfs
        stat (is_bound), and only a `pinctrl get 2,3` on top of that if
        the driver is in fact bound. Returns True if it repaired anything.
        """
        if is_driver_healthy(self._hw):
            return False
        rebound = self_heal(self._hw)
        logger.warning(
            "Self-heal: repaired pins/driver left by an interrupted recovery."
        )
        if rebound:
            self._restart_ups("self-heal rebound the driver")
        return True

    def _restart_ups(self, reason: str) -> None:
        """Shared by every UPS-restart trigger (a bus recovery, a
        self-heal that rebound the driver, and the stale-with-healthy-bus
        safety net). Respects --no-restart-ups for all three."""
        if not self._cfg.restart_ups:
            return
        logger.warning(
            "Restarting %s (%s) so it reopens its I2C handle.",
            UPS_SERVICE_NAME, reason,
        )
        self._hw.restart_ups_service()

    def _maybe_ups_safety_net(
        self,
        now_mono: float,
        ups_age: float | None,
        ups_stale: bool,
        sda_level: str,
        scl_level: str,
    ) -> None:
        """The bus itself looks fine (SDA high, SCL high -- not stuck) but
        the UPS status has been stale for a long time anyway, so a wedged
        bus cannot be the cause. Restarting the UPS daemon is the one
        action that can plausibly fix whatever else is wrong with it.
        Rate-limited separately from bus recoveries, at once per 15
        minutes, since this is a cheaper and more speculative action. A
        missing status file (ups_age is None) never triggers this.
        """
        if not ups_stale or ups_age is None:
            return
        if ups_age <= UPS_SAFETY_NET_STALE_S:
            return
        if not (sda_level == "hi" and scl_level == "hi"):
            return
        if not self._ups_safety_net_limiter.allow(now_mono):
            logger.debug("UPS safety-net restart rate-limited; will retry later.")
            return
        self._ups_safety_net_limiter.record(now_mono)
        self._restart_ups(f"UPS status stale {ups_age:.0f}s with a healthy bus")

    def _warn_if_ups_blind(
        self, now_mono: float, ups_age: float | None, ups_stale: bool
    ) -> None:
        """UPS-blind alert for ANY cause -- independent of whether the bus
        is stuck. A stuck bus is only one way the UPS daemon can go blind
        (it might also just be stopped or crashed); either way, a stale
        status means safe shutdown on real power loss is not active, and
        that is worth a warning even when SDA/SCL look fine.

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
        self,
        now_wall: float,
        now_mono: float,
        ups_age: float | None,
        samples: list[tuple[str, str]],
    ) -> None:
        if self._throttle.ready("stuck", now_mono):
            logger.warning(
                "I2C bus 1 stuck: %d consecutive joint samples with SDA low "
                "and SCL high; UPS status age %.1fs (> %.1fs)",
                len(samples), ups_age, self._cfg.stale_s,
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
        else:
            logger.error("I2C bus recovery failed: %s", error)

        # Rule B: the unbind inside recover_bus killed the UPS daemon's
        # only I2C fd regardless of outcome -- restart it once the driver
        # is bound again. Checked fresh (not trusted from `ok`, which is
        # about SDA, not about whether the finally block's bind() itself
        # actually succeeded).
        if self._hw.is_bound():
            self._restart_ups("bus recovery")


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
        "--sample-count", type=int, default=DEFAULT_SAMPLE_COUNT, dest="sample_count",
        help=(
            "consecutive joint SDA+SCL samples required to call the bus "
            f"stuck (default: {DEFAULT_SAMPLE_COUNT})"
        ),
    )
    parser.add_argument(
        "--sample-interval-s", type=float, default=DEFAULT_SAMPLE_INTERVAL_S,
        dest="sample_interval_s",
        help=f"seconds between joint samples (default: {DEFAULT_SAMPLE_INTERVAL_S})",
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
            "do not restart upsplus-power.service after a bus recovery, a "
            "self-heal that rebinds the driver, or the stale-UPS safety net "
            "(default: restart it)"
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


class ShutdownFlag:
    """A flag a signal handler sets; the main loop polls it once per
    cycle so an in-progress recovery's `finally` block always finishes.

    Python's default SIGTERM action terminates the process immediately,
    which would skip recover_bus's `finally` and strand the pins/driver
    -- exactly the failure self_heal() exists to repair on a later cycle,
    but it is much better to never need that repair. Installing a handler
    that only sets this flag means a signal interrupts nothing: Python
    runs the handler between bytecode steps, the handler returns normally
    (it does not raise), and execution resumes exactly where it was -- so
    a `finally` block already in progress always completes. (SIGKILL
    cannot be caught by any process; only self_heal() can recover from
    that one.)
    """

    def __init__(self) -> None:
        self.requested = False

    def handle(self, signum, frame) -> None:  # matches signal.signal's handler shape
        self.requested = True


def _run_loop(
    guard: I2cBusGuard,
    *,
    once: bool,
    shutdown: ShutdownFlag,
    interval_s: float,
    sleep_fn=time.sleep,
) -> None:
    """The guard's main loop, factored out of main() so tests can drive it
    against a fake clock/guard without installing real signal handlers.

    Exits as soon as `once` is set (after exactly one cycle) or `shutdown`
    is set -- checked both right after a cycle and right after the
    inter-cycle sleep, so a signal arriving during either is noticed
    promptly rather than waiting for a cycle that will never come.
    """
    while True:
        try:
            guard.run_cycle()
        except Exception:
            logger.exception("i2c bus guard cycle failed unexpectedly")
        if once or shutdown.requested:
            break
        sleep_fn(interval_s)
        if shutdown.requested:
            break


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
        sample_count=args.sample_count,
        sample_interval_s=args.sample_interval_s,
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
    shutdown = ShutdownFlag()

    # Only set a flag on SIGTERM/SIGINT -- see ShutdownFlag's docstring
    # for why this is the whole point. Restored before returning so
    # calling main() more than once in the same process (tests do) never
    # leaks a changed signal disposition into the rest of that process.
    old_sigterm = signal.signal(signal.SIGTERM, shutdown.handle)
    old_sigint = signal.signal(signal.SIGINT, shutdown.handle)
    try:
        _run_loop(
            guard, once=args.once, shutdown=shutdown,
            interval_s=cfg.interval_s, sleep_fn=time.sleep,
        )
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        signal.signal(signal.SIGINT, old_sigint)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
