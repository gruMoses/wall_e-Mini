"""
Pure mapping functions for converting RC pulse widths to normalized outputs.

- ch1/ch2: 850–2150 µs -> 0–255 (inclusive), with CENTER_OUTPUT_VALUE=128 at 1500 µs
- Deadband: ±25 µs around 1500 µs maps to center output (128)
- Inputs are clamped to [MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US]

No I/O, deterministic, testable.
"""

from typing import Final


MIN_PULSE_WIDTH_US: Final[int] = 850
MAX_PULSE_WIDTH_US: Final[int] = 2150
CENTER_PULSE_WIDTH_US: Final[int] = 1500
DEADBAND_US: Final[int] = 25

MIN_OUTPUT: Final[int] = 0
MAX_OUTPUT: Final[int] = 255
CENTER_OUTPUT_VALUE: Final[int] = 128

_SPAN_US: Final[int] = MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US  # 1300


def clamp(value: int, lo: int, hi: int) -> int:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def map_pulse_to_byte(pulse_us: int) -> int:
    """
    Map a pulse width in microseconds to a byte in [0..255] with deadband around center.

    Rules:
    - If |pulse - 1500| <= 25: return 128
    - Else clamp pulse to [850, 2150] and map linearly to [0, 255]
    - Endpoints map exactly: 850 -> 0, 2150 -> 255
    """
    if abs(pulse_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_OUTPUT_VALUE

    clamped = clamp(pulse_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
    # Normalize to [0.0, 1.0]
    normalized = (clamped - MIN_PULSE_WIDTH_US) / _SPAN_US
    # Map to [0, 255] and round to nearest integer for stability at edges
    mapped = int(round(normalized * MAX_OUTPUT))
    # Safety clamp against float rounding quirks
    return clamp(mapped, MIN_OUTPUT, MAX_OUTPUT)


# CH1/CH2 saturated mapping: early full-scale beyond configurable thresholds
def map_pulse_to_byte_saturated(
    pulse_us: int,
    forward_full_us: int,
    reverse_full_us: int,
) -> int:
    """
    Piecewise-linear mapping with early saturation for throttle channels.

    Rules:
    - Deadband: |pulse-1500| <= 25 -> 128
    - If pulse >= forward_full_us -> 255
    - If pulse <= reverse_full_us -> 0
    - Otherwise, map linearly:
      * [1500..forward_full_us] -> [128..255]
      * [reverse_full_us..1500] -> [0..128]
    - Inputs are clamped to [850, 2150] for safety.
    """
    # Deadband around center
    if abs(pulse_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_OUTPUT_VALUE

    # Safety clamp raw pulse to plausible range
    clamped_us = clamp(pulse_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

    # Early saturation
    if clamped_us >= forward_full_us:
        return MAX_OUTPUT
    if clamped_us <= reverse_full_us:
        return MIN_OUTPUT

    # Linear within segments toward center
    if clamped_us > CENTER_PULSE_WIDTH_US:
        # Upper segment: center -> forward_full_us maps 128 -> 255
        span_up = max(1, forward_full_us - CENTER_PULSE_WIDTH_US)
        frac = (clamped_us - CENTER_PULSE_WIDTH_US) / span_up
        val = CENTER_OUTPUT_VALUE + int(round(frac * (MAX_OUTPUT - CENTER_OUTPUT_VALUE)))
        return clamp(val, MIN_OUTPUT, MAX_OUTPUT)
    else:
        # Lower segment: reverse_full_us -> center maps 0 -> 128
        span_dn = max(1, CENTER_PULSE_WIDTH_US - reverse_full_us)
        frac = (clamped_us - reverse_full_us) / span_dn
        val = int(round(frac * CENTER_OUTPUT_VALUE))
        return clamp(val, MIN_OUTPUT, MAX_OUTPUT)

