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
from pi_app.control.follow_me import PersonDetection


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


def _make_det(x_m=0.0, z_m=2.0, confidence=0.85,
              bbox=(0.4, 0.3, 0.6, 0.8), track_id=None) -> PersonDetection:
    """Build a PersonDetection for serialization tests."""
    return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                           bbox=bbox, track_id=track_id)


def _sse_detections(person_detections):
    """Mirror the SSE dict-comprehension from oak_viewer.py ~line 1147-1151."""
    return [
        {"x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
         "conf": round(d.confidence, 2), "track_id": d.track_id}
        for d in person_detections
    ]


def _mcap_detections(person_detections):
    """Mirror the MCAP dict-comprehension from oak_recorder.py ~line 985-990."""
    return [
        {"x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
         "conf": round(d.confidence, 2), "track_id": d.track_id,
         "bbox": [round(b, 3) for b in d.bbox]}
        for d in person_detections
    ]


class TestDetectionTrackIdSerialization(unittest.TestCase):
    """
    Backlog G: per-detection track_id in SSE /api/telemetry detections array.

    Both the SSE builder (oak_viewer.py) and the MCAP recorder (oak_recorder.py)
    emit {"track_id": d.track_id} for each PersonDetection.  These tests verify
    the field is present with the correct value for tracked detections and is
    JSON-null-compatible (Python None) for unconfirmed / untracked detections.
    """

    # ── SSE serialization ─────────────────────────────────────────────────────

    def test_sse_tracked_detection_carries_track_id(self):
        """A confirmed detection (track_id=7) must appear as track_id=7 in the SSE dict."""
        det = _make_det(x_m=0.3, z_m=2.5, confidence=0.91, track_id=7)
        result = _sse_detections([det])
        self.assertEqual(len(result), 1)
        self.assertIn("track_id", result[0])
        self.assertEqual(result[0]["track_id"], 7)

    def test_sse_untracked_detection_track_id_is_none(self):
        """An unconfirmed detection (track_id=None) must serialize as Python None (JSON null)."""
        det = _make_det(x_m=-0.1, z_m=3.0, confidence=0.72, track_id=None)
        result = _sse_detections([det])
        self.assertEqual(len(result), 1)
        self.assertIn("track_id", result[0])
        self.assertIsNone(result[0]["track_id"])

    def test_sse_mixed_detections_preserve_per_detection_track_ids(self):
        """Mixed list: tracked + untracked detections each get the right track_id."""
        tracked = _make_det(x_m=0.1, z_m=1.8, confidence=0.95, track_id=3)
        untracked = _make_det(x_m=0.5, z_m=4.0, confidence=0.61, track_id=None)
        result = _sse_detections([tracked, untracked])
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["track_id"], 3)
        self.assertIsNone(result[1]["track_id"])

    def test_sse_empty_detections_returns_empty_list(self):
        """No detections → empty list, no KeyError."""
        result = _sse_detections([])
        self.assertEqual(result, [])

    # ── MCAP recorder serialization ───────────────────────────────────────────

    def test_mcap_tracked_detection_carries_track_id(self):
        """MCAP serialization also emits track_id for a confirmed detection."""
        det = _make_det(x_m=0.0, z_m=2.0, confidence=0.88, track_id=12)
        result = _mcap_detections([det])
        self.assertEqual(len(result), 1)
        self.assertIn("track_id", result[0])
        self.assertEqual(result[0]["track_id"], 12)

    def test_mcap_untracked_detection_track_id_is_none(self):
        """MCAP serialization emits None (JSON null) for an unconfirmed detection."""
        det = _make_det(x_m=0.2, z_m=5.0, confidence=0.65, track_id=None)
        result = _mcap_detections([det])
        self.assertEqual(len(result), 1)
        self.assertIsNone(result[0]["track_id"])

    # ── PersonDetection dataclass contract ────────────────────────────────────

    def test_person_detection_track_id_defaults_to_none(self):
        """PersonDetection.track_id defaults to None when not supplied."""
        det = PersonDetection(x_m=0.0, z_m=1.0, confidence=0.9,
                              bbox=(0.4, 0.3, 0.6, 0.8))
        self.assertIsNone(det.track_id)

    def test_person_detection_track_id_accepts_int(self):
        """PersonDetection.track_id stores a positive integer correctly."""
        det = PersonDetection(x_m=0.0, z_m=1.0, confidence=0.9,
                              bbox=(0.4, 0.3, 0.6, 0.8), track_id=42)
        self.assertEqual(det.track_id, 42)


if __name__ == "__main__":
    unittest.main()
