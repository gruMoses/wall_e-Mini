import unittest

from pi_app.control.mapping import (
    map_pulse_to_byte,
    MIN_PULSE_WIDTH_US,
    MAX_PULSE_WIDTH_US,
    CENTER_PULSE_WIDTH_US,
    DEADBAND_US,
    CENTER_OUTPUT_VALUE,
    MIN_OUTPUT,
    MAX_OUTPUT,
)


class TestMapping(unittest.TestCase):
    def test_endpoints(self):
        self.assertEqual(map_pulse_to_byte(MIN_PULSE_WIDTH_US), MIN_OUTPUT)
        self.assertEqual(map_pulse_to_byte(MAX_PULSE_WIDTH_US), MAX_OUTPUT)

    def test_center_deadband(self):
        for delta in range(-DEADBAND_US, DEADBAND_US + 1):
            self.assertEqual(map_pulse_to_byte(CENTER_PULSE_WIDTH_US + delta), CENTER_OUTPUT_VALUE)

    def test_known_values(self):
        self.assertEqual(map_pulse_to_byte(MIN_PULSE_WIDTH_US), 0)
        self.assertEqual(map_pulse_to_byte(CENTER_PULSE_WIDTH_US), 126)
        self.assertEqual(map_pulse_to_byte(MAX_PULSE_WIDTH_US), 254)

    def test_piecewise_symmetry(self):
        """Verify the mapping is piecewise linear around 126."""
        midpoint_fwd = (CENTER_PULSE_WIDTH_US + MAX_PULSE_WIDTH_US) // 2
        midpoint_rev = (MIN_PULSE_WIDTH_US + CENTER_PULSE_WIDTH_US) // 2

        fwd_val = map_pulse_to_byte(midpoint_fwd)
        self.assertTrue(CENTER_OUTPUT_VALUE < fwd_val < MAX_OUTPUT,
                        f"midpoint fwd should be between {CENTER_OUTPUT_VALUE} and {MAX_OUTPUT}, got {fwd_val}")

        rev_val = map_pulse_to_byte(midpoint_rev)
        self.assertTrue(MIN_OUTPUT < rev_val < CENTER_OUTPUT_VALUE,
                        f"midpoint rev should be between {MIN_OUTPUT} and {CENTER_OUTPUT_VALUE}, got {rev_val}")

    def test_clamping_out_of_range(self):
        self.assertEqual(map_pulse_to_byte(MIN_PULSE_WIDTH_US - 500), MIN_OUTPUT)
        self.assertEqual(map_pulse_to_byte(MAX_PULSE_WIDTH_US + 500), MAX_OUTPUT)


if __name__ == "__main__":
    unittest.main()
