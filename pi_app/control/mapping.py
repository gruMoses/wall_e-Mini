"""
Pure mapping functions for converting RC pulse widths to normalized outputs.

- ch1/ch2: 990–2010 µs -> 0–254 (inclusive), with CENTER_OUTPUT_VALUE=126 at 1500 µs
- Deadband: ±25 µs around 1500 µs maps to center output (126)
- Inputs are clamped to [MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US]

No I/O, deterministic, testable.
"""

from typing import Final


MIN_PULSE_WIDTH_US: Final[int] = 850
MAX_PULSE_WIDTH_US: Final[int] = 2150
CENTER_PULSE_WIDTH_US: Final[int] = 1500
DEADBAND_US: Final[int] = 25

MIN_OUTPUT: Final[int] = 0
MAX_OUTPUT: Final[int] = 254
CENTER_OUTPUT_VALUE: Final[int] = 126

_SPAN_US: Final[int] = MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US  # 1020


def clamp(value: int, lo: int, hi: int) -> int:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def map_pulse_to_byte(pulse_us: int) -> int:
    """
    Map a pulse width in microseconds to a byte in [0..254] with deadband around center.

    Rules:
    - If |pulse - 1500| <= 25: return 126
    - Else clamp pulse to [990, 2010] and map linearly to [0, 254]
    - Endpoints map exactly: 990 -> 0, 2010 -> 254
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


