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


class TestStickExpo(unittest.TestCase):
    """Per-track stick expo (2026-09-27): gentle near the centre, full at the edge."""

    def setUp(self):
        from pi_app.control.mapping import apply_stick_expo
        self.expo = apply_stick_expo

    def test_zero_expo_is_identity(self):
        for b in range(MIN_OUTPUT, MAX_OUTPUT + 1):
            self.assertEqual(self.expo(b, 0.0), b)

    def test_neutral_and_full_scale_do_not_move(self):
        for e in (0.3, 0.6, 1.0):
            self.assertEqual(self.expo(CENTER_OUTPUT_VALUE, e), CENTER_OUTPUT_VALUE)
            self.assertEqual(self.expo(MAX_OUTPUT, e), MAX_OUTPUT)
            self.assertEqual(self.expo(MIN_OUTPUT, e), MIN_OUTPUT)

    def test_half_stick_at_the_default(self):
        # x = 0.5: y = 0.4 * 0.5 + 0.6 * 0.125 = 0.275
        self.assertEqual(self.expo(126 + 64, 0.6), 126 + round(0.275 * 128))
        self.assertEqual(self.expo(126 - 63, 0.6), 126 - round(0.275 * 126))

    def test_monotonic_keeps_sign_and_never_exceeds_linear(self):
        prev = -1
        for b in range(MIN_OUTPUT, MAX_OUTPUT + 1):
            y = self.expo(b, 0.6)
            self.assertGreaterEqual(y, prev)
            prev = y
            self.assertLessEqual(abs(y - CENTER_OUTPUT_VALUE), abs(b - CENTER_OUTPUT_VALUE))
            if b > CENTER_OUTPUT_VALUE:
                self.assertGreaterEqual(y, CENTER_OUTPUT_VALUE)
            elif b < CENTER_OUTPUT_VALUE:
                self.assertLessEqual(y, CENTER_OUTPUT_VALUE)

    def test_expo_is_clamped(self):
        self.assertEqual(self.expo(190, -1.0), 190)
        self.assertEqual(self.expo(190, 5.0), self.expo(190, 1.0))
