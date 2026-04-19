"""Shared drive math helpers for skid-steer motor outputs."""

from __future__ import annotations

from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT


def skid_steer_mix(
    speed_offset: float,
    yaw_offset: float,
    *,
    neutral: int = CENTER_OUTPUT_VALUE,
    min_output: int = MIN_OUTPUT,
    max_output: int = MAX_OUTPUT,
    deadband_byte: int = 0,
) -> tuple[int, int]:
    """Convert speed/yaw offsets into left/right motor bytes.

    ``speed_offset`` and ``yaw_offset`` are expressed in motor-byte units
    relative to ``neutral``. Positive ``yaw_offset`` turns right by increasing
    the left side and decreasing the right side.
    """
    left = int(round(neutral + speed_offset + yaw_offset))
    right = int(round(neutral + speed_offset - yaw_offset))

    if deadband_byte > 0:
        if speed_offset + yaw_offset != 0.0 and abs(left - neutral) < deadband_byte:
            left = neutral + deadband_byte * (1 if (speed_offset + yaw_offset) > 0 else -1)
        if speed_offset - yaw_offset != 0.0 and abs(right - neutral) < deadband_byte:
            right = neutral + deadband_byte * (1 if (speed_offset - yaw_offset) > 0 else -1)

    return (
        max(min_output, min(max_output, left)),
        max(min_output, min(max_output, right)),
    )
