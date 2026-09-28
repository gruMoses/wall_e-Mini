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




class TestStickExpoPair(unittest.TestCase):
    """Ratio-preserving stick expo (2026-09-27): gentle near the centre,
    full at the edge, turn radius unchanged, never more difference than linear."""

    def setUp(self):
        from pi_app.control.mapping import apply_stick_expo_pair
        self.expo = apply_stick_expo_pair

    @staticmethod
    def _dev(b):
        return (b - 126) / 128.0 if b >= 126 else (b - 126) / 126.0

    def test_zero_expo_is_identity(self):
        for l in range(MIN_OUTPUT, MAX_OUTPUT + 1, 7):
            for r in range(MIN_OUTPUT, MAX_OUTPUT + 1, 11):
                self.assertEqual(self.expo(l, r, 0.0), (l, r))

    def test_neutral_and_full_scale_do_not_move(self):
        for e in (0.3, 0.6, 1.0):
            self.assertEqual(self.expo(126, 126, e), (126, 126))
            self.assertEqual(self.expo(MAX_OUTPUT, MAX_OUTPUT, e), (MAX_OUTPUT, MAX_OUTPUT))
            self.assertEqual(self.expo(MIN_OUTPUT, MIN_OUTPUT, e), (MIN_OUTPUT, MIN_OUTPUT))
            # One stick full, the other centred: a full-speed arc, unchanged.
            self.assertEqual(self.expo(MAX_OUTPUT, 126, e), (MAX_OUTPUT, 126))

    def test_equal_sticks_follow_the_expo_curve(self):
        # x = 0.5: f = 0.4 * 0.5 + 0.6 * 0.125 = 0.275
        self.assertEqual(self.expo(126 + 64, 126 + 64, 0.6), (126 + 35, 126 + 35))
        self.assertEqual(self.expo(126 - 63, 126 - 63, 0.6), (126 - 35, 126 - 35))

    def test_turn_ratio_is_preserved(self):
        l, r = self.expo(126 + 100, 126 + 50, 0.6)
        self.assertAlmostEqual(self._dev(l) / self._dev(r), 2.0, delta=0.1)
        l, r = self.expo(126 + 40, 126 - 40, 0.6)  # pivot
        self.assertEqual(l - 126, 126 - r)

    def test_never_more_difference_than_linear_and_keeps_signs(self):
        for l in range(MIN_OUTPUT, MAX_OUTPUT + 1, 5):
            for r in range(MIN_OUTPUT, MAX_OUTPUT + 1, 5):
                el, er = self.expo(l, r, 0.6)
                self.assertLessEqual(abs(el - er), abs(l - r) + 1)
                self.assertLessEqual(abs(el - 126), abs(l - 126))
                self.assertLessEqual(abs(er - 126), abs(r - 126))
                for raw, out in ((l, el), (r, er)):
                    if raw > 126:
                        self.assertGreaterEqual(out, 126)
                    elif raw < 126:
                        self.assertLessEqual(out, 126)

    def test_expo_is_clamped(self):
        self.assertEqual(self.expo(190, 160, -1.0), (190, 160))
        self.assertEqual(self.expo(190, 160, 5.0), self.expo(190, 160, 1.0))


if __name__ == "__main__":
    unittest.main()
