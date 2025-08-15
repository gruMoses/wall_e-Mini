import math

import unittest

from pi_app.control.mapping import (
    map_pulse_to_byte,
    MIN_PULSE_WIDTH_US,
    MAX_PULSE_WIDTH_US,
    CENTER_PULSE_WIDTH_US,
    DEADBAND_US,
    CENTER_OUTPUT_VALUE,
)


class TestMapping(unittest.TestCase):
    def test_endpoints(self):
        self.assertEqual(map_pulse_to_byte(MIN_PULSE_WIDTH_US), 0)
        self.assertEqual(map_pulse_to_byte(MAX_PULSE_WIDTH_US), 255)

    def test_center_deadband(self):
        for delta in range(-DEADBAND_US, DEADBAND_US + 1):
            self.assertEqual(map_pulse_to_byte(CENTER_PULSE_WIDTH_US + delta), CENTER_OUTPUT_VALUE)

    def test_linear_samples_outside_deadband(self):
        # Select sample pulses outside the deadband to validate linear mapping formula
        samples = [
            MIN_PULSE_WIDTH_US,
            MIN_PULSE_WIDTH_US + 102,  # ~10% of span
            CENTER_PULSE_WIDTH_US - (DEADBAND_US + 1),
            CENTER_PULSE_WIDTH_US + (DEADBAND_US + 1),
            MAX_PULSE_WIDTH_US - 102,
            MAX_PULSE_WIDTH_US,
        ]
        for pulse in samples:
            if abs(pulse - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
                expected = CENTER_OUTPUT_VALUE
            else:
                # Expected linear mapping to [0..255] with rounding
                expected = int(round((max(MIN_PULSE_WIDTH_US, min(MAX_PULSE_WIDTH_US, pulse)) - MIN_PULSE_WIDTH_US) / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) * 255))
            self.assertEqual(map_pulse_to_byte(pulse), expected)

    def test_clamping_out_of_range(self):
        self.assertEqual(map_pulse_to_byte(MIN_PULSE_WIDTH_US - 500), 0)
        self.assertEqual(map_pulse_to_byte(MAX_PULSE_WIDTH_US + 500), 255)


if __name__ == "__main__":
    unittest.main()


