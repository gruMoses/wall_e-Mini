"""Tests for the DepthFilter rejection-latch fix and target-switch reseed.

Bug: rejections advance ``_last_time`` so dt stays tiny; after a sustained
depth step every subsequent frame is rejected forever and speed is regulated
against the old depth until the full loss timeout. Fix: accept the raw value
as a fresh seed after a run of consecutive rejections, and reseed on a
track_id change from the target-selection path.
"""

import unittest

from config import FollowMeConfig
from pi_app.control.follow_me import DepthFilter, FollowMeController, PersonDetection


class TestDepthFilterLatch(unittest.TestCase):
    def test_old_behavior_rejects_step_then_latch_breaks(self):
        """A sustained 1.8m -> 4.0m step (same target) is rejected for the
        first frames, then the latch-breaker accepts it within 5 frames."""
        df = DepthFilter(alpha=0.35, max_velocity_mps=5.0)
        # Seed at 1.8m.
        self.assertAlmostEqual(df.update(1.8, 0.0), 1.8)

        t = 0.0
        # Frames 1..4: implied velocity (~2.2m / 33ms ≈ 66 m/s) far exceeds the
        # 5 m/s gate, so each is rejected and the filtered value is held.
        for _ in range(4):
            t += 0.033
            self.assertAlmostEqual(df.update(4.0, t), 1.8)

        # Frame 5: fifth consecutive rejection -> accept raw as new seed.
        t += 0.033
        self.assertAlmostEqual(df.update(4.0, t), 4.0)
        # And it stays locked (no implausible jump now).
        t += 0.033
        self.assertAlmostEqual(df.update(4.0, t), 4.0)

    def test_accepted_sample_resets_reject_streak(self):
        """An accepted (plausible) sample zeroes the counter so the next step
        again gets the full rejection budget rather than latching early."""
        df = DepthFilter(alpha=0.35, max_velocity_mps=5.0)
        df.update(2.0, 0.0)
        t = 0.0
        # Two rejections.
        for _ in range(2):
            t += 0.033
            df.update(9.0, t)
        # A plausible reading is accepted and resets the streak.
        t += 0.033
        df.update(2.05, t)
        self.assertEqual(df._reject_count, 0)

    def test_reset_reseeds_on_next_update(self):
        df = DepthFilter()
        df.update(1.8, 0.0)
        df.reset()
        self.assertIsNone(df.value)
        # Next update seeds directly to the new value (no rejection).
        self.assertAlmostEqual(df.update(4.0, 0.1), 4.0)


class TestTargetSwitchReseed(unittest.TestCase):
    def _make(self, **overrides) -> FollowMeController:
        return FollowMeController(FollowMeConfig(**overrides))

    def _person(self, x_m=0.0, z_m=2.0, confidence=0.9,
                bbox=(0.4, 0.0, 0.6, 0.8), track_id=None) -> PersonDetection:
        return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                               bbox=bbox, track_id=track_id)

    def test_track_id_change_reseeds_depth_filter_immediately(self):
        """Switching to a candidate with a new track_id reseeds the depth
        filter to that target's depth on the same frame, instead of riding the
        rejection latch against the previous target's depth."""
        fm = self._make()
        # Lock onto track 1 at ~2.0m.
        fm.compute([self._person(z_m=2.0, track_id=1)])
        self.assertAlmostEqual(fm._depth_filter.value, 2.0, places=3)

        # Now only track 2 is visible, ~5.0m away. Without the reseed the 3m
        # jump would be rejected and the filtered depth would stay near 2.0m.
        fm.compute([self._person(z_m=5.0, track_id=2)])
        self.assertAlmostEqual(fm._depth_filter.value, 5.0, places=3)


if __name__ == "__main__":
    unittest.main()
