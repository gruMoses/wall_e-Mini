"""Wheel-speed kinematics and the VESC RPM plausibility gate.

Two small, dependency-free pieces that the velocity PID re-enable needs:

1. Kinematics — the ONE place that turns a drive byte or an eRPM reading into a
   wheel speed in m/s. ``controller.py`` (telemetry -> actual_speed_mps) and
   ``FollowMeConfig.speed_loop_mps_per_byte`` (commanded byte -> target speed)
   must agree on this scale or the velocity loop carries a permanent bias.
   Derived from ``VescConfig`` constants; see docs/gearing_memo.md §a.

2. ``RpmPlausibilityGate`` — treats RPM telemetry as INVALID when the motors are
   being commanded non-trivially but report ~0 eRPM for a sustained window.
   That was the 2026-06-11 lunge/stall cycle: dead RPM readback looked like
   "robot not moving", the velocity PID kept adding throttle, the robot lunged.
   While the gate is tripped the controller nulls the RPM / speed fields so
   follow_me and slip detection fall back to open-loop, exactly as they do for
   stale telemetry. The gate is intentionally independent of the PID gains: it
   protects any consumer of actual_speed_mps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT

# Byte span from neutral to full scale (126 -> 254 = 128). Mirrors
# VescCanDriver._byte_to_rpm, which maps this span linearly onto max_erpm.
_BYTE_SPAN_UP = MAX_OUTPUT - CENTER_OUTPUT_VALUE


def erpm_to_wheel_mps(
    erpm: float,
    *,
    motor_poles: int,
    drive_gear_ratio: float,
    wheel_radius_m: float,
) -> float:
    """Electrical RPM -> wheel surface speed (m/s). Sign is preserved."""
    pole_pairs = max(int(motor_poles) // 2, 1)
    mech_rpm = float(erpm) / max(pole_pairs * max(float(drive_gear_ratio), 1e-6), 1e-6)
    return (mech_rpm / 60.0) * 2.0 * math.pi * float(wheel_radius_m)


def byte_offset_to_erpm(offset_bytes: float, *, max_erpm: int) -> float:
    """Drive-byte offset from neutral -> commanded eRPM (same linear map as the driver)."""
    return float(offset_bytes) / float(_BYTE_SPAN_UP) * float(max_erpm)


def wheel_mps_per_byte(
    *,
    max_erpm: int,
    motor_poles: int,
    drive_gear_ratio: float,
    wheel_radius_m: float,
) -> float:
    """Kinematic wheel speed produced by ONE drive byte above neutral (m/s).

    With the as-built drivetrain (15000 eRPM full scale, 14 poles, 34.2857:1,
    r = 0.18415 m) this is ~0.00942 m/s per byte, i.e. 1.205 m/s at byte 254.
    This is WHEEL speed (what eRPM telemetry measures), not ground speed: the
    GPS-calibrated ``trail_speed_scale_mps_per_byte`` (0.0075) is ~20 % lower
    because of track scrub on gravel. The velocity loop must use THIS value so
    that the commanded target and the measured feedback share one scale.
    """
    return erpm_to_wheel_mps(
        byte_offset_to_erpm(1.0, max_erpm=max_erpm),
        motor_poles=motor_poles,
        drive_gear_ratio=drive_gear_ratio,
        wheel_radius_m=wheel_radius_m,
    )


@dataclass
class RpmPlausibilityConfig:
    """Knobs for :class:`RpmPlausibilityGate` (mirrored in ``VescConfig``)."""
    enabled: bool = True
    # A commanded offset of at least this many bytes from neutral counts as
    # "non-trivial" (12 bytes ~= 1400 eRPM ~= 0.11 m/s wheel speed).
    min_cmd_bytes: int = 12
    # |eRPM| below this while commanded non-trivially is "~0" (a spinning
    # wheel at the minimum non-trivial command reports ~1400 eRPM).
    min_erpm: int = 150
    # The implausible condition must persist this long before we trip. Covers
    # normal spin-up latency after a command step (slew ramps from neutral).
    window_s: float = 0.5
    # Once tripped, stay tripped for at least this long even if RPM reappears,
    # so a flickering readback cannot chatter the loop open/closed.
    hold_s: float = 2.0


class RpmPlausibilityGate:
    """Detects "commanded to move, but RPM says stopped" and declares telemetry invalid.

    Call :meth:`update` once per telemetry poll with the bytes that were most
    recently EMITTED to the motors and the RPM values just read back. Returns
    ``True`` while telemetry is plausible, ``False`` while tripped.

    Per motor: a streak starts when |cmd - neutral| >= min_cmd_bytes AND the
    motor reports |rpm| < min_erpm. The streak is broken by a trivial command
    (nothing to check), a plausible RPM, or a missing RPM (``None`` — the
    staleness logic upstream already owns that case). A streak that lasts
    ``window_s`` trips the gate. Recovery needs both: ``hold_s`` elapsed since
    the trip AND a plausible RPM observed on the motor that tripped.
    """

    def __init__(self, cfg: RpmPlausibilityConfig | None = None, *, neutral: int = CENTER_OUTPUT_VALUE) -> None:
        self._cfg = cfg or RpmPlausibilityConfig()
        self._neutral = int(neutral)
        self._streak_start: dict[str, Optional[float]] = {"left": None, "right": None}
        self._tripped_at: dict[str, Optional[float]] = {"left": None, "right": None}
        self._trip_count: int = 0

    # ── Introspection (telemetry / debug board) ──────────────────────────────
    @property
    def tripped(self) -> bool:
        return any(t is not None for t in self._tripped_at.values())

    @property
    def tripped_motors(self) -> tuple[str, ...]:
        return tuple(m for m, t in self._tripped_at.items() if t is not None)

    @property
    def trip_count(self) -> int:
        return self._trip_count

    @property
    def window_s(self) -> float:
        return float(self._cfg.window_s)

    def reset(self) -> None:
        for m in self._streak_start:
            self._streak_start[m] = None
            self._tripped_at[m] = None

    # ── Core ─────────────────────────────────────────────────────────────────
    def update(
        self,
        now: float,
        *,
        left_cmd_byte: int,
        right_cmd_byte: int,
        left_rpm: Optional[float],
        right_rpm: Optional[float],
    ) -> bool:
        """Advance the gate; return True if RPM telemetry is plausible."""
        if not self._cfg.enabled:
            return True
        was_tripped = self.tripped
        self._update_motor("left", now, left_cmd_byte, left_rpm)
        self._update_motor("right", now, right_cmd_byte, right_rpm)
        if self.tripped and not was_tripped:
            self._trip_count += 1  # one event per plausible→tripped transition
        return not self.tripped

    def _update_motor(self, motor: str, now: float, cmd_byte: int, rpm: Optional[float]) -> None:
        cfg = self._cfg
        cmd_offset = abs(int(cmd_byte) - self._neutral)
        commanded = cmd_offset >= int(cfg.min_cmd_bytes)
        rpm_known = rpm is not None
        rpm_plausible = rpm_known and abs(float(rpm)) >= float(cfg.min_erpm)

        if self._tripped_at[motor] is not None:
            # Recovery: hold time elapsed AND we have seen a real RPM again.
            if rpm_plausible and (now - self._tripped_at[motor]) >= float(cfg.hold_s):
                self._tripped_at[motor] = None
                self._streak_start[motor] = None
            return

        if commanded and rpm_known and not rpm_plausible:
            if self._streak_start[motor] is None:
                self._streak_start[motor] = now
            elif (now - self._streak_start[motor]) >= float(cfg.window_s):
                self._tripped_at[motor] = now
        else:
            # Trivial command, plausible RPM, or no RPM at all: nothing to hold against.
            self._streak_start[motor] = None
