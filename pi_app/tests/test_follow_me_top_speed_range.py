"""Follow-me top speed and range after the 2026-09-26 outrun.

arm_20260926_125554.log: the operator walked away at ~1.55 m/s while
follow-me sat at its 1.38 m/s cap (max_follow_speed_byte 110). At 6.1-6.6 m
DetectionFilter dropped him on range alone (YOLO 0.83-0.89, stereo and
bbox-height ranges agreeing) and the robot stopped. These tests pin the fix:
the cap is the full max_erpm byte, the approach inside a 3.0 m gap keeps the
old speed law, and a cleanly detected operator at 6-8 m is still followed.
"""

import unittest

from config import FollowMeConfig, VescConfig
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT
from pi_app.hardware.vesc import VescCanDriver


def _old_open_loop(distance_m: float) -> float:
    """Speed law before 2026-09-26: min(110, error * 110 / 1.5)."""
    return min(110.0, (distance_m - 1.5) * 110.0 / 1.5)


class TestTopSpeed(unittest.TestCase):
    def test_cap_is_the_full_max_erpm_byte(self):
        cfg = FollowMeConfig()
        top_byte = CENTER_OUTPUT_VALUE + cfg.max_follow_speed_byte
        self.assertEqual(top_byte, MAX_OUTPUT)
        max_erpm = VescConfig().max_erpm
        self.assertEqual(VescCanDriver._byte_to_rpm(top_byte, max_erpm), max_erpm)

    def test_top_speed_outpaces_the_logged_walk(self):
        cfg = FollowMeConfig()
        top_mps = cfg.max_follow_speed_byte * cfg.speed_loop_mps_per_byte
        self.assertAlmostEqual(top_mps, 1.607, delta=0.01)
        self.assertGreater(top_mps, 1.55)  # operator pace in the 12:55 run

    def test_large_gap_reaches_the_new_cap(self):
        fm = FollowMeController(FollowMeConfig())
        self.assertEqual(fm._speed.compute(3.3), 128.0)
        self.assertEqual(fm._speed.compute(6.0), 128.0)


class TestApproachUnchanged(unittest.TestCase):
    def test_open_loop_inside_3m_matches_old_law(self):
        # speed_dead_zone_m is 0.2, so the law starts above 1.7 m.
        for d in (1.75, 1.9, 2.0, 2.3, 2.6, 2.9, 3.0):
            fm = FollowMeController(FollowMeConfig())
            new = fm._speed.compute(d)
            old = _old_open_loop(d)
            self.assertAlmostEqual(new, old, delta=max(0.003 * old, 1e-6),
                                   msg=f"distance {d} m")
            self.assertLessEqual(new, old + 1e-9, msg=f"distance {d} m")

    def test_old_cap_is_reached_at_the_same_gap(self):
        fm = FollowMeController(FollowMeConfig())
        self.assertLess(fm._speed.compute(3.0), 110.0)
        self.assertGreater(fm._speed.compute(3.0), 109.0)


class TestFollowRange(unittest.TestCase):
    def _det(self, z_m, bbox, conf):
        return PersonDetection(
            x_m=0.4, z_m=z_m, confidence=conf, bbox=bbox, track_id=155,
            depth_status="ok", z_stereo_m=z_m,
        )

    def test_logged_operator_at_6_2_and_6_6_m_passes(self):
        fm = FollowMeController(FollowMeConfig())
        for z, bbox, conf in (
            (6.21, (0.533, 0.274, 0.604, 0.644), 0.87),
            (6.62, (0.539, 0.292, 0.601, 0.634), 0.83),
        ):
            out = fm._filter.process([self._det(z, bbox, conf)])
            self.assertEqual(len(out), 1, msg=f"z {z} m")
            self.assertAlmostEqual(out[0].depth_m, z)

    def test_range_limit_is_8_m(self):
        fm = FollowMeController(FollowMeConfig())
        bbox = (0.45, 0.30, 0.52, 0.65)  # width 0.07 clears the 0.05 floor
        self.assertEqual(len(fm._filter.process([self._det(7.9, bbox, 0.86)])), 1)
        self.assertEqual(len(fm._filter.process([self._det(8.2, bbox, 0.86)])), 0)


if __name__ == "__main__":
    unittest.main()
