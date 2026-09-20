"""Tests for support-backed corridor near-distance and persistence.

Root cause (2026-09-19 18:50 follow-me, last 30 s, low evening sun): the
corridor distance was the 5th percentile of the VALID corridor pixels with
no minimum absolute pixel support. When few pixels were valid, 5% of them
was a handful of noisy near pixels, so obstacle.distance_m flickered
2.3 → 1.4 → 2.3 → 1.6 → 0.4 (depth_p5_mm 383) → 0.8 while the followed
person was 2.8–3.8 m ahead and the corridor was empty.

These tests drive the pure helpers (_corridor_near_distance_mm,
_CorridorPersistence) plus a thin _poll_depth wiring check using the same
fake-queue harness as test_depth_corridor_stale.py.
"""

import unittest

import numpy as np

from config import ObstacleAvoidanceConfig, FollowMeConfig
from pi_app.hardware.oak_depth import (
    OakDepthReader,
    _corridor_near_distance_mm,
    _CorridorPersistence,
)


MIN_SUPPORT_PX = 400


class _FakeFrame:
    """Wraps a numpy uint16 mm depth array and mimics the depthai ImgFrame API."""

    def __init__(self, array: np.ndarray):
        self._arr = array

    def getFrame(self):
        return self._arr


class _FakeQueue:
    """Single-item fake queue: returns one frame on the first tryGet, then None."""

    def __init__(self, frame=None):
        self._frame = frame
        self._served = False

    def tryGet(self):
        if self._frame is not None and not self._served:
            self._served = True
            return self._frame
        return None


def _make_reader(**obs_overrides) -> OakDepthReader:
    kwargs = dict(
        slow_distance_m=1.5,
        stop_distance_m=0.4,
        roi_width_pct=0.8,
        roi_height_pct=0.5,
        robot_width_m=0.820,
        min_depth_mm=350,
        min_valid_pct=8.0,
        update_rate_hz=15.0,
        stale_timeout_s=0.5,
        stale_policy="stop",
        safety_stop_radius_m=0.8,
        corridor_min_support_px=MIN_SUPPORT_PX,
        corridor_persistence_polls=2,
    )
    kwargs.update(obs_overrides)
    return OakDepthReader(
        obstacle_config=ObstacleAvoidanceConfig(**kwargs),
        follow_me_config=FollowMeConfig(),
    )


def _obstacle_frame(distance_mm=1000, h=400, w=640) -> np.ndarray:
    return np.full((h, w), distance_mm, dtype=np.uint16)


def _poll(reader: OakDepthReader, frame: np.ndarray) -> None:
    reader._poll_depth(_FakeQueue(frame=_FakeFrame(frame)), _FakeQueue(frame=None), np)


class TestCorridorNearDistanceSupport(unittest.TestCase):
    """(a) (b) (d) — k-th smallest with a minimum pixel floor."""

    def test_eight_noisy_near_pixels_do_not_pull_p5_down(self):
        """(a) 30 valid near pixels at 380 mm + 500 at 2500 mm → ~2500, not 380.

        This is the sparse-depth case from the 2026-09-19 run: few valid
        pixels, so 5 percent of them (26 here) is inside the noisy near blob
        and the old np.percentile(..., 5) reports 380 (the phantom). The
        support floor requires 400 pixels at or nearer than the reported
        distance, so the 30-pixel blob is ignored.
        """
        near = np.full(30, 380, dtype=np.uint16)
        far = np.full(500, 2500, dtype=np.uint16)
        valid = np.concatenate([near, far])
        raw_p5 = float(np.percentile(valid, 5))
        self.assertLess(raw_p5, 500.0, "sanity: old 5th percentile is the phantom")
        got = _corridor_near_distance_mm(valid, MIN_SUPPORT_PX)
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, 2500.0, places=0)

    def test_genuine_near_blob_is_reported(self):
        """(b) 2000 near pixels at 800 mm → ~800."""
        valid = np.full(2000, 800, dtype=np.uint16)
        got = _corridor_near_distance_mm(valid, MIN_SUPPORT_PX)
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, 800.0, places=0)

    def test_fewer_than_min_support_is_rejected(self):
        """(d) fewer than 400 valid pixels → corridor rejected (None)."""
        self.assertIsNone(_corridor_near_distance_mm(
            np.full(399, 800, dtype=np.uint16), MIN_SUPPORT_PX))
        self.assertIsNone(_corridor_near_distance_mm(
            np.array([], dtype=np.uint16), MIN_SUPPORT_PX))
        self.assertIsNone(_corridor_near_distance_mm(
            np.full(0, 800, dtype=np.uint16), MIN_SUPPORT_PX))

    def test_exactly_min_support_is_accepted(self):
        valid = np.full(MIN_SUPPORT_PX, 1200, dtype=np.uint16)
        got = _corridor_near_distance_mm(valid, MIN_SUPPORT_PX)
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, 1200.0, places=0)


class TestCorridorPersistence(unittest.TestCase):
    """(c) max(current, previous) so a one-poll phantom cannot lower distance."""

    def test_single_poll_phantom_never_lowers_distance(self):
        """Poll 1: 2500, poll 2: 400, poll 3: 2500 — never reports below 2500."""
        pers = _CorridorPersistence(persistence_polls=2)
        self.assertEqual(pers.update(2500.0), 2500.0)
        self.assertEqual(pers.update(400.0), 2500.0)
        self.assertEqual(pers.update(2500.0), 2500.0)

    def test_two_consecutive_near_polls_do_report_near(self):
        """Two consecutive 400 polls after a 2500 do report 400."""
        pers = _CorridorPersistence(persistence_polls=2)
        self.assertEqual(pers.update(2500.0), 2500.0)
        self.assertEqual(pers.update(400.0), 2500.0)
        self.assertEqual(pers.update(400.0), 400.0)

    def test_reset_drops_history(self):
        pers = _CorridorPersistence(persistence_polls=2)
        pers.update(2500.0)
        pers.reset()
        self.assertEqual(pers.update(400.0), 400.0)


class TestPollDepthWiresSupportAndPersistence(unittest.TestCase):
    """_poll_depth must call the helpers, not the old np.percentile path."""

    def test_uniform_far_then_single_near_stays_far(self):
        """Integration of (c): a one-frame 400 mm corridor cannot slam throttle."""
        reader = _make_reader()
        _poll(reader, _obstacle_frame(2500))
        d1, _ = reader.get_min_distance()
        _poll(reader, _obstacle_frame(400))
        d2, _ = reader.get_min_distance()
        _poll(reader, _obstacle_frame(2500))
        d3, _ = reader.get_min_distance()
        self.assertAlmostEqual(d1, 2.5, places=1)
        self.assertAlmostEqual(d2, 2.5, places=1)
        self.assertAlmostEqual(d3, 2.5, places=1)
        self.assertTrue(min(d1, d2, d3) >= 2.5 - 0.05)

    def test_two_consecutive_near_frames_report_near(self):
        reader = _make_reader()
        _poll(reader, _obstacle_frame(2500))
        _poll(reader, _obstacle_frame(400))
        _poll(reader, _obstacle_frame(400))
        dist_m, _ = reader.get_min_distance()
        self.assertAlmostEqual(dist_m, 0.4, places=1)

    def test_rejected_corridor_resets_persistence(self):
        """Empty (rejected) poll must drop history so the next real reading is fresh."""
        reader = _make_reader()
        _poll(reader, _obstacle_frame(2500))
        _poll(reader, np.zeros((400, 640), dtype=np.uint16))
        _poll(reader, _obstacle_frame(400))
        dist_m, _ = reader.get_min_distance()
        self.assertAlmostEqual(dist_m, 0.4, places=1)

    def test_stats_expose_corridor_valid_pct_and_support_px(self):
        reader = _make_reader()
        _poll(reader, _obstacle_frame(1000))
        stats = reader.get_depth_stats()
        self.assertGreater(stats.corridor_support_px, MIN_SUPPORT_PX)
        self.assertGreaterEqual(stats.corridor_valid_pct, 8.0)
        self.assertAlmostEqual(stats.p5_mm, 1000.0, places=0)

    def test_genuine_blob_through_poll_reports_near(self):
        """Integration of (b): a full-frame 800 mm obstacle reports ~0.8 m."""
        reader = _make_reader()
        _poll(reader, _obstacle_frame(800))
        dist_m, _ = reader.get_min_distance()
        self.assertAlmostEqual(dist_m, 0.8, places=1)


if __name__ == "__main__":
    unittest.main()
