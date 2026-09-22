"""Hardware-free tests for the both-wrists-up pose detector."""

import unittest

from config import ArmsUpConfig
from pi_app.control.arms_up import PoseJoint, PoseSample, ArmsUpDetector, arms_up


def _j(x, y, vis=0.9) -> PoseJoint:
    return PoseJoint(x=x, y=y, visibility=vis)


def _pose(
    ts=1.0,
    l_sh_x=0.40,
    r_sh_x=0.60,
    l_sh_y=0.40,
    r_sh_y=0.40,
    l_el_y=0.35,
    r_el_y=0.35,
    l_wr_y=0.20,
    r_wr_y=0.20,
    l_wr_vis=0.9,
    r_wr_vis=0.9,
    l_sh_vis=0.9,
    r_sh_vis=0.9,
    l_el_vis=0.9,
    r_el_vis=0.9,
    nose_y=0.20,
    crop_y0=None,
    crop_y1=None,
) -> PoseSample:
    # Default: shoulders 0.2 apart → margin = 0.25 * 0.2 = 0.05.
    # Wrists at 0.20 are 0.20 above shoulders (0.40 - 0.05 = 0.35 threshold).
    return PoseSample(
        ts=ts,
        l_shoulder=_j(l_sh_x, l_sh_y, l_sh_vis),
        r_shoulder=_j(r_sh_x, r_sh_y, r_sh_vis),
        l_elbow=_j(0.40, l_el_y, l_el_vis),
        r_elbow=_j(0.60, r_el_y, r_el_vis),
        l_wrist=_j(0.38, l_wr_y, l_wr_vis),
        r_wrist=_j(0.62, r_wr_y, r_wr_vis),
        nose_y=nose_y,
        crop_y0=crop_y0,
        crop_y1=crop_y1,
    )


CFG = ArmsUpConfig()


class TestArmsUpPure(unittest.TestCase):
    def test_both_wrists_above_shoulders(self):
        self.assertTrue(arms_up(_pose(), CFG))

    def test_one_wrist_up_is_not_a_trigger(self):
        self.assertFalse(arms_up(_pose(r_wr_y=0.50), CFG))
        self.assertFalse(arms_up(_pose(l_wr_y=0.50), CFG))

    def test_low_wrist_visibility_rejected(self):
        self.assertFalse(arms_up(_pose(l_wr_vis=0.3), CFG))
        self.assertFalse(arms_up(_pose(r_wr_vis=0.3), CFG))

    def test_wrists_level_with_shoulders_rejected(self):
        self.assertFalse(arms_up(_pose(l_wr_y=0.40, r_wr_y=0.40), CFG))

    def test_degenerate_shoulder_width_rejected(self):
        self.assertFalse(arms_up(_pose(l_sh_x=0.50, r_sh_x=0.50), CFG))
        self.assertFalse(arms_up(_pose(l_sh_x=0.50, r_sh_x=0.50 + 1e-9), CFG))

    def test_elbows_far_below_shoulders_rejected(self):
        # Wrists still "up" — impossible pose / landmark glitch.
        self.assertFalse(arms_up(_pose(l_el_y=0.80, r_el_y=0.80), CFG))
        self.assertFalse(arms_up(_pose(l_el_y=0.80), CFG))

    def test_low_visibility_elbow_is_rejected(self):
        # Both elbows must be visible. A hidden elbow is not a measured arm.
        self.assertFalse(arms_up(_pose(l_el_y=0.80, l_el_vis=0.1), CFG))
        self.assertFalse(arms_up(_pose(l_el_vis=0.1), CFG))
        self.assertFalse(arms_up(_pose(r_el_vis=0.1), CFG))

    def test_wrist_at_crop_top_is_rejected(self):
        # Default wrists sit at y=0.20. A crop whose top is that row pins them.
        self.assertFalse(arms_up(_pose(crop_y0=0.20, crop_y1=0.90), CFG))
        # Inside the crop, clear of the top margin, the same pose is still true.
        self.assertTrue(arms_up(_pose(crop_y0=0.0, crop_y1=1.0), CFG))

    def test_none_sample_is_false(self):
        self.assertFalse(arms_up(None, CFG))


class TestArmsUpDetector(unittest.TestCase):
    def _det(self, **overrides) -> ArmsUpDetector:
        return ArmsUpDetector(ArmsUpConfig(**overrides) if overrides else CFG)

    def test_hold_requires_new_sample_timestamps(self):
        det = self._det()
        up = _pose(ts=1.0)
        st = det.update(up, now=1.0)
        self.assertFalse(st["active"])
        self.assertEqual(st["streak_s"], 0.0)

        # Same ts, later wall clock, inside max_sample_gap_s: must not
        # advance the streak. 0.3 s would itself restart the streak.
        st = det.update(up, now=1.2)
        self.assertFalse(st["active"])
        self.assertEqual(st["streak_s"], 0.0)
        self.assertAlmostEqual(st["last_sample_age_s"], 0.2)

        st = det.update(_pose(ts=1.2), now=1.2)
        self.assertFalse(st["active"])
        self.assertAlmostEqual(st["streak_s"], 0.2)

        st = det.update(_pose(ts=1.4), now=1.4)
        self.assertTrue(st["active"])
        self.assertTrue(st["rising_edge"])
        self.assertAlmostEqual(st["streak_s"], 0.4)

    def test_rising_edge_fires_exactly_once(self):
        det = self._det()
        edges = []
        for i, ts in enumerate((1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6)):
            st = det.update(_pose(ts=ts), now=ts)
            edges.append(st["rising_edge"])
        self.assertEqual(sum(1 for e in edges if e), 1)
        self.assertTrue(edges[4])  # ts=1.4 is the hold crossing
        self.assertTrue(det.active)
        self.assertFalse(det.update(_pose(ts=1.7), now=1.7)["rising_edge"])

    def test_release_after_release_s_of_false_or_none(self):
        det = self._det()
        det.update(_pose(ts=1.0), now=1.0)
        det.update(_pose(ts=1.2), now=1.2)
        det.update(_pose(ts=1.4), now=1.4)
        self.assertTrue(det.active)

        st = det.update(None, now=1.5)
        self.assertTrue(st["active"], "still inside release_s")
        st = det.update(None, now=1.8)
        self.assertFalse(st["active"])
        self.assertFalse(st["rising_edge"])

        det = self._det()
        det.update(_pose(ts=1.0), now=1.0)
        det.update(_pose(ts=1.2), now=1.2)
        det.update(_pose(ts=1.4), now=1.4)
        down = _pose(ts=1.5, l_wr_y=0.55, r_wr_y=0.55)
        det.update(down, now=1.5)
        st = det.update(_pose(ts=1.8, l_wr_y=0.55, r_wr_y=0.55), now=1.8)
        self.assertFalse(st["active"])

    def test_stale_releases_even_if_last_sample_was_true(self):
        det = self._det()
        det.update(_pose(ts=1.0), now=1.0)
        det.update(_pose(ts=1.2), now=1.2)
        det.update(_pose(ts=1.4), now=1.4)
        self.assertTrue(det.active)
        # Same True sample, no new ts, past stale_s (0.5).
        st = det.update(_pose(ts=1.4), now=2.0)
        self.assertFalse(st["active"])
        self.assertAlmostEqual(st["last_sample_age_s"], 0.6)


def _down(ts: float) -> PoseSample:
    return _pose(ts=ts, l_wr_y=0.80, r_wr_y=0.80)


class TestArmsUpContinuity(unittest.TestCase):
    def test_two_samples_with_no_update_between_do_not_edge(self):
        # Without the gap rule these two samples would complete the hold
        # (0.5 s >= 0.4 s, two samples). The 0.5 s hole restarts the streak.
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.4, min_hold_samples=2, max_sample_gap_s=0.25,
        ))
        det.update(_pose(ts=0.0), now=0.0)
        st = det.update(_pose(ts=0.5), now=0.5)
        self.assertFalse(st["rising_edge"])
        self.assertFalse(st["active"])

    def test_inter_sample_gap_restarts_streak(self):
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.0, min_hold_samples=2, max_sample_gap_s=0.25,
        ))
        det.update(_pose(ts=1.0), now=1.0)
        # Update gap stays inside the limit; the sample timestamp jumps.
        det.update(_pose(ts=1.0), now=1.2)
        st = det.update(_pose(ts=1.5), now=1.4)
        self.assertFalse(st["rising_edge"])
        self.assertFalse(st["active"])
        self.assertEqual(st["streak_s"], 0.0)

    def test_fewer_than_min_hold_samples_gives_no_edge(self):
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.0, min_hold_samples=3, max_sample_gap_s=0.25,
        ))
        det.update(_pose(ts=1.0), now=1.0)
        st = det.update(_pose(ts=1.1), now=1.1)
        self.assertFalse(st["rising_edge"])
        self.assertFalse(st["active"])
        st = det.update(_pose(ts=1.2), now=1.2)
        self.assertTrue(st["rising_edge"])

    def test_none_dropout_does_not_rearm(self):
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.0, min_hold_samples=1, release_s=0.0,
            stale_s=10.0, max_sample_gap_s=1.0,
        ))
        self.assertTrue(det.update(_pose(ts=1.0), now=1.0)["rising_edge"])
        det.update(None, now=1.05)
        st = det.update(_pose(ts=1.1), now=1.1)
        self.assertFalse(st["rising_edge"])
        self.assertFalse(st["active"])

    def test_stale_dropout_does_not_rearm(self):
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.0, min_hold_samples=1, stale_s=0.5, max_sample_gap_s=1.0,
        ))
        self.assertTrue(det.update(_pose(ts=1.0), now=1.0)["rising_edge"])
        st = det.update(_pose(ts=1.0), now=1.6)
        self.assertFalse(st["active"])
        st = det.update(_pose(ts=1.7), now=1.7)
        self.assertFalse(st["rising_edge"])

    def test_arms_down_then_up_allows_a_second_edge(self):
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.0, min_hold_samples=1, release_s=0.0, max_sample_gap_s=1.0,
        ))
        self.assertTrue(det.update(_pose(ts=1.0), now=1.0)["rising_edge"])
        det.update(_down(1.1), now=1.1)
        st = det.update(_pose(ts=1.2), now=1.2)
        self.assertTrue(st["rising_edge"])

    def test_reset_does_not_rearm(self):
        det = ArmsUpDetector(ArmsUpConfig(
            hold_s=0.0, min_hold_samples=1, max_sample_gap_s=1.0,
        ))
        self.assertTrue(det.update(_pose(ts=1.0), now=1.0)["rising_edge"])
        det.reset()
        st = det.update(_pose(ts=2.0), now=2.0)
        self.assertFalse(st["rising_edge"])
        self.assertFalse(st["active"])
