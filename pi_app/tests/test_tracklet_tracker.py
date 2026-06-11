"""
Tests for the host-side tracklet layer (IoU + constant-velocity Kalman).

Coverage:
  - Stable track_id maintained across jittered / moving frames
  - No track_id reported before min_hits confirmations
  - Track dropped and a NEW id issued after the target is absent > max_age
  - Two crossing people keep distinct ids (greedy-IoU sanity)
  - Part A: velocity PID gains are zeroed; tracker knobs present in config
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.hardware.oak_depth import Tracklet, TrackletTracker


def _box(cx, cy, w=0.1, h=0.3):
    """Centre/size → (xmin, ymin, xmax, ymax)."""
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


class TestTrackletTracker(unittest.TestCase):

    # ── Stable id across jittered / moving bboxes ─────────────────────────────
    def test_stable_id_across_moving_frames(self):
        tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=15)
        jitter = [0.0, 0.004, -0.003, 0.002, -0.004, 0.003, -0.002, 0.004]
        seen_ids = []
        for i in range(8):
            cx = 0.4 + i * 0.01          # drifts slowly right
            cy = 0.5 + jitter[i]         # small vertical jitter
            ids = tracker.update([_box(cx, cy)])
            seen_ids.append(ids[0])

        # Confirmed only after min_hits=3 → first two frames are None.
        self.assertIsNone(seen_ids[0])
        self.assertIsNone(seen_ids[1])
        # From frame 3 on, a stable, non-None id.
        confirmed = seen_ids[2:]
        self.assertTrue(all(i is not None for i in confirmed),
                        f"expected ids from frame 3 on, got {seen_ids}")
        self.assertEqual(len(set(confirmed)), 1,
                         f"id should not flap: {confirmed}")

    # ── No id before min_hits ─────────────────────────────────────────────────
    def test_no_id_before_min_hits(self):
        tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=15)
        first = tracker.update([_box(0.5, 0.5)])
        second = tracker.update([_box(0.505, 0.5)])
        third = tracker.update([_box(0.51, 0.5)])
        self.assertEqual(first, [None])
        self.assertEqual(second, [None])
        self.assertIsNotNone(third[0])  # 3rd hit confirms

    # ── Drop + new id after absence > max_age ────────────────────────────────
    def test_new_id_after_target_absent_beyond_max_age(self):
        tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=5)
        # Confirm the first target.
        for i in range(3):
            ids = tracker.update([_box(0.5 + i * 0.005, 0.5)])
        first_id = ids[0]
        self.assertIsNotNone(first_id)

        # Target vanishes for longer than max_age.
        for _ in range(7):  # > max_age (5)
            self.assertEqual(tracker.update([]), [])
        self.assertEqual(len(tracker.tracklets), 0, "stale tracklet should age out")

        # Target reappears → fresh, confirmed id, distinct from the old one.
        for i in range(3):
            ids = tracker.update([_box(0.5 + i * 0.005, 0.5)])
        second_id = ids[0]
        self.assertIsNotNone(second_id)
        self.assertNotEqual(second_id, first_id,
                            "re-acquired target must get a new monotonic id")

    # ── Two crossing people keep distinct ids ────────────────────────────────
    def test_two_crossing_people_distinct_ids(self):
        tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=15)
        # A: small box, left → right.   B: larger box, right → left.
        # Distinct sizes + constant-velocity prediction keep greedy IoU correct
        # even as their centres pass through each other.
        # Motion ~0.025/frame keeps frame-to-frame IoU above threshold (a real
        # target at 15 fps doesn't jump most of a box-width per frame). A and B
        # close from 0.30 / 0.70 and cross near frame 8.
        a_ids, b_ids = [], []
        for i in range(14):
            a_cx = 0.30 + i * 0.025
            b_cx = 0.70 - i * 0.025
            dets = [_box(a_cx, 0.45, w=0.12, h=0.25),
                    _box(b_cx, 0.55, w=0.18, h=0.40)]
            ids = tracker.update(dets)
            a_ids.append(ids[0])
            b_ids.append(ids[1])

        a_final = [i for i in a_ids if i is not None]
        b_final = [i for i in b_ids if i is not None]
        self.assertTrue(a_final and b_final, "both tracks should confirm")
        # Each track keeps a single stable id...
        self.assertEqual(len(set(a_final)), 1, f"A id flapped: {a_ids}")
        self.assertEqual(len(set(b_final)), 1, f"B id flapped: {b_ids}")
        # ...and the two people are never conflated.
        self.assertNotEqual(set(a_final), set(b_final),
                            f"crossing people share an id: A={a_ids} B={b_ids}")

    # ── Empty frame advances the tracker without error ───────────────────────
    def test_empty_frame_ages_and_returns_empty(self):
        tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=2)
        for i in range(3):
            tracker.update([_box(0.5, 0.5)])
        self.assertEqual(len(tracker.tracklets), 1)
        for _ in range(3):  # > max_age
            self.assertEqual(tracker.update([]), [])
        self.assertEqual(len(tracker.tracklets), 0)

    # ── Single-tracklet Kalman keeps the bbox near its measurements ──────────
    def test_kalman_tracks_measurement(self):
        t = Tracklet(track_id=1, bbox=_box(0.5, 0.5))
        t.predict()
        t.update(_box(0.55, 0.5))
        cx = (t.predicted_bbox()[0] + t.predicted_bbox()[2]) / 2.0
        self.assertAlmostEqual(cx, 0.55, delta=0.05)


class TestPartAVelocityPidDisabled(unittest.TestCase):

    def test_speed_pid_gains_zeroed(self):
        cfg = FollowMeConfig()
        self.assertEqual(cfg.speed_kp, 0.0)
        self.assertEqual(cfg.speed_ki, 0.0)
        self.assertEqual(cfg.speed_kd, 0.0)

    def test_tracklet_knobs_present(self):
        cfg = FollowMeConfig()
        self.assertEqual(cfg.tracklet_iou_threshold, 0.3)
        self.assertEqual(cfg.tracklet_min_hits, 3)
        self.assertEqual(cfg.tracklet_max_age, 15)


if __name__ == "__main__":
    unittest.main()
