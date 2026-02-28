"""
Pure mapping functions for converting RC pulse widths to normalized outputs.

- ch1/ch2: 850–2150 µs -> 0–254 motor byte, with CENTER_OUTPUT_VALUE=126 at 1500 µs
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


def clamp(value: int, lo: int, hi: int) -> int:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def map_pulse_to_byte(pulse_us: int) -> int:
    """
    Map a pulse width in microseconds to a motor byte in [0..254].

    Piecewise linear around center:
    - 850 µs  -> 0   (full reverse)
    - 1500 µs -> 126 (neutral)
    - 2150 µs -> 254 (full forward)
    - Deadband: |pulse - 1500| <= 25 -> 126
    """
    if abs(pulse_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_OUTPUT_VALUE

    clamped = clamp(pulse_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

    if clamped > CENTER_PULSE_WIDTH_US:
        span = MAX_PULSE_WIDTH_US - CENTER_PULSE_WIDTH_US
        frac = (clamped - CENTER_PULSE_WIDTH_US) / span
        val = CENTER_OUTPUT_VALUE + int(round(frac * (MAX_OUTPUT - CENTER_OUTPUT_VALUE)))
    else:
        span = CENTER_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US
        frac = (clamped - MIN_PULSE_WIDTH_US) / span
        val = int(round(frac * CENTER_OUTPUT_VALUE))

    return clamp(val, MIN_OUTPUT, MAX_OUTPUT)


def map_pulse_to_byte_saturated(
    pulse_us: int,
    forward_full_us: int,
    reverse_full_us: int,
) -> int:
    """
    Piecewise-linear mapping with early saturation for throttle channels.

    Rules:
    - Deadband: |pulse-1500| <= 25 -> 126
    - If pulse >= forward_full_us -> 254
    - If pulse <= reverse_full_us -> 0
    - Otherwise, map linearly:
      * [1500..forward_full_us] -> [126..254]
      * [reverse_full_us..1500] -> [0..126]
    - Inputs are clamped to [850, 2150] for safety.
    """
    if abs(pulse_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_OUTPUT_VALUE

    clamped_us = clamp(pulse_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

    if clamped_us >= forward_full_us:
        return MAX_OUTPUT
    if clamped_us <= reverse_full_us:
        return MIN_OUTPUT

    if clamped_us > CENTER_PULSE_WIDTH_US:
        span_up = max(1, forward_full_us - CENTER_PULSE_WIDTH_US)
        frac = (clamped_us - CENTER_PULSE_WIDTH_US) / span_up
        val = CENTER_OUTPUT_VALUE + int(round(frac * (MAX_OUTPUT - CENTER_OUTPUT_VALUE)))
        return clamp(val, MIN_OUTPUT, MAX_OUTPUT)
    else:
        span_dn = max(1, CENTER_PULSE_WIDTH_US - reverse_full_us)
        frac = (clamped_us - reverse_full_us) / span_dn
        val = int(round(frac * CENTER_OUTPUT_VALUE))
        return clamp(val, MIN_OUTPUT, MAX_OUTPUT)
