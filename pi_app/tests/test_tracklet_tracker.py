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
from pi_app.web.oak_viewer import sse_detection_dict
from pi_app.hardware.oak_recorder import mcap_detection_dict


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

    def test_speed_pid_gains_enabled_with_guards(self):
        """2026-09-19: the velocity PID is re-enabled. It was zeroed 2026-06-11
        because dead RPM readback drove a lunge/stall cycle; the re-enable ships
        with the RPM plausibility gate (VescConfig) and a bounded correction, so
        the gains must be non-zero AND both guards must be present in config."""
        cfg = FollowMeConfig()
        self.assertGreater(cfg.speed_kp, 0.0)
        self.assertGreater(cfg.speed_ki, 0.0)
        self.assertEqual(cfg.speed_kd, 0.0, "no derivative on 20 Hz eRPM — too noisy")
        self.assertGreater(cfg.speed_pid_max_correction_mps, 0.0)
        self.assertLessEqual(cfg.speed_pid_max_correction_mps, 0.3,
                             "closed-loop authority must stay a nudge, not a lunge")
        self.assertLessEqual(cfg.speed_ki * cfg.speed_integral_limit,
                             cfg.speed_pid_max_correction_mps,
                             "integral authority must fit inside the correction clamp")
        from config import VescConfig
        vcfg = VescConfig()
        self.assertTrue(vcfg.rpm_plausibility_enabled)
        self.assertGreater(vcfg.rpm_plausibility_window_s, 0.0)

    def test_tracklet_knobs_present(self):
        cfg = FollowMeConfig()
        self.assertEqual(cfg.tracklet_iou_threshold, 0.3)
        self.assertEqual(cfg.tracklet_min_hits, 3)
        self.assertEqual(cfg.tracklet_max_age, 15)


def _make_det(x_m=0.0, z_m=2.0, confidence=0.85,
              bbox=(0.4, 0.0, 0.6, 0.8), track_id=None) -> PersonDetection:
    """Build a PersonDetection for serialization tests."""
    return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                           bbox=bbox, track_id=track_id)


def _sse_detections(person_detections):
    """Apply the real SSE serializer (pi_app/web/oak_viewer.py) to a list."""
    return [sse_detection_dict(d) for d in person_detections]


def _mcap_detections(person_detections):
    """Apply the real MCAP serializer (pi_app/hardware/oak_recorder.py) to a list."""
    return [mcap_detection_dict(d) for d in person_detections]


class TestDetectionTrackIdSerialization(unittest.TestCase):
    """
    Backlog G: per-detection track_id in SSE /api/telemetry detections array.

    Both the SSE builder (oak_viewer.sse_detection_dict) and the MCAP recorder
    (oak_recorder.mcap_detection_dict) emit {"track_id": d.track_id} for each
    PersonDetection. These tests call the real production serializers
    (not mirrors) to verify the field is present with the correct value for
    tracked detections and is JSON-null-compatible (Python None) for
    unconfirmed / untracked detections.
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
                              bbox=(0.4, 0.0, 0.6, 0.8))
        self.assertIsNone(det.track_id)

    def test_person_detection_track_id_accepts_int(self):
        """PersonDetection.track_id stores a positive integer correctly."""
        det = PersonDetection(x_m=0.0, z_m=1.0, confidence=0.9,
                              bbox=(0.4, 0.0, 0.6, 0.8), track_id=42)
        self.assertEqual(det.track_id, 42)


# 2026-09-20 field evidence: consecutive detections of the same person whose
# IoU is 0 (or < 0.3) because the box is narrow at range and the robot yaws.
_EVIDENCE_SEQS = (
    (
        ((0.62, 0.15, 0.71, 0.65), 4.3),
        ((0.43, 0.15, 0.53, 0.64), 4.6),
        ((0.36, 0.16, 0.46, 0.66), 4.6),
    ),
    (
        ((0.41, 0.17, 0.52, 0.74), 3.9),
        ((0.26, 0.13, 0.37, 0.71), 4.0),
        ((0.21, 0.14, 0.34, 0.72), 4.0),
    ),
)


class TestCentreDistanceFallback(unittest.TestCase):
    """Host-side tracklet layer: centre-distance fallback after greedy IoU."""

    def _fallback(self, **kw) -> TrackletTracker:
        opts = dict(
            iou_threshold=0.3, min_hits=3, max_age=15,
            center_gate_min=0.20, center_gate_width_mult=1.5,
        )
        opts.update(kw)
        return TrackletTracker(**opts)

    def _confirm(self, tracker: TrackletTracker, box, depth=None) -> int:
        ids = None
        depths = None if depth is None else [depth]
        for _ in range(4):
            ids = tracker.update([box], depths=depths)
        self.assertIsNotNone(ids[0], "tracklet must be confirmed")
        return ids[0]

    def test_evidence_sequences_keep_id_with_fallback(self):
        for seq in _EVIDENCE_SEQS:
            tracker = self._fallback()
            op_id = self._confirm(tracker, seq[0][0], seq[0][1])
            for box, depth in seq[1:]:
                ids = tracker.update([box], depths=[depth])
                self.assertEqual(
                    ids[0], op_id,
                    f"fallback must keep id across {seq[0][0]} -> {box}",
                )

    def test_evidence_sequences_mint_new_id_without_fallback(self):
        """Documents the 2026-09-20 bug: defaults disable the fallback, so
        IoU-0 jumps mint a new tentative tracklet (assigned None)."""
        for seq in _EVIDENCE_SEQS:
            tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=15)
            op_id = self._confirm(tracker, seq[0][0], seq[0][1])
            ids = tracker.update([seq[1][0]], depths=[seq[1][1]])
            self.assertNotEqual(ids[0], op_id)
            self.assertIsNone(ids[0], "new tracklet is tentative until min_hits")

    def test_depth_gate_still_wins_inside_centre_gate(self):
        tracker = self._fallback(
            depth_gate_m=0.75, depth_gate_growth_m_per_frame=0.10,
        )
        box_a, depth_a = _EVIDENCE_SEQS[0][0]
        box_b, _ = _EVIDENCE_SEQS[0][1]
        op_id = self._confirm(tracker, box_a, depth_a)
        ids = tracker.update([box_b], depths=[depth_a - 1.5])
        self.assertNotEqual(ids[0], op_id)
        self.assertIsNone(ids[0], "1.5 m closer must not inherit the id")

    def test_ambiguity_guard_two_dets_in_one_gate(self):
        tracker = self._fallback()
        op_id = self._confirm(tracker, _box(0.50, 0.50, w=0.10, h=0.30))
        left = _box(0.35, 0.50, w=0.10, h=0.30)
        right = _box(0.65, 0.50, w=0.10, h=0.30)
        ids = tracker.update([left, right])
        self.assertIsNone(ids[0])
        self.assertIsNone(ids[1])
        self.assertNotIn(op_id, ids)

    def test_tentative_tracklet_does_not_use_fallback(self):
        tracker = self._fallback(min_hits=3)
        first = tracker.update([_EVIDENCE_SEQS[0][0][0]], depths=[4.3])
        self.assertIsNone(first[0], "unconfirmed after one hit")
        second = tracker.update([_EVIDENCE_SEQS[0][1][0]], depths=[4.6])
        self.assertIsNone(second[0], "tentative must not inherit via fallback")
        self.assertEqual(len(tracker.tracklets), 2,
                         "unmatched det must mint a new tracklet")


if __name__ == "__main__":
    unittest.main()
