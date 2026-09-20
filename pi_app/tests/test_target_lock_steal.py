"""
"Closer person steals the lock" — regression tests for the 2026-09-19 fix.

Before the fix, three independent paths let a second person who stepped
nearer take the Follow-Me lock on a SINGLE frame:

  1. None-id positional continuity was x-only: a closer person at the
     operator's x WAS "the operator" (TargetTracker._find_committed).
  2. An id'd committed target absent for one frame (occluded by the closer
     person) triggered an IMMEDIATE "trusted hand-off" to the closest
     floor-clearing candidate — the occluder.
  3. The host-side tracklet layer matched by IoU alone, so an occluder's
     bigger, overlapping box could inherit the operator's track_id and the
     id then lied to the tracker.

Each test below fails on the pre-fix code and passes with the fix:
  - depth continuity on both id and positional matches (1, 3),
  - sustained hand-off: a challenger must persist target_switch_min_s (2),
  - depth-gated association in TrackletTracker, with depths supplied by the
    live YOLOv8 host-NMS path via OakDepthReader._assign_track_ids (3).
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3]))

from config import FollowMeConfig
from pi_app.control.follow_me import PersonDetection, TargetTracker, _FilteredDetection
from pi_app.hardware.oak_depth import OakDepthReader, TrackletTracker


GRACE = 1.5
SWITCH_MIN_S = 1.0
MIN_FRAMES = 3


def _tracker(**overrides) -> TargetTracker:
    kw = dict(
        ema_alpha=1.0,               # no smoothing → raw selections are easy to assert
        persistence_s=2.0,
        switch_grace_s=GRACE,
        acquire_confidence=0.65,
        acquire_min_frames=MIN_FRAMES,
        switch_min_s=SWITCH_MIN_S,
        depth_continuity_m=0.6,
        depth_continuity_rate_mps=1.5,
    )
    kw.update(overrides)
    return TargetTracker(**kw)


def _det(normalized_x=0.0, depth_m=3.0, confidence=0.9, track_id=None) -> _FilteredDetection:
    return _FilteredDetection(
        normalized_x=normalized_x,
        x_m=normalized_x,
        depth_m=depth_m,
        confidence=confidence,
        bbox=(0.4, 0.0, 0.6, 0.8),
        track_id=track_id,
    )


class TestCloserPersonCannotStealLock(unittest.TestCase):
    """TargetTracker: the operator keeps the lock when someone steps nearer."""

    def test_config_wires_the_new_knobs(self):
        cfg = FollowMeConfig()
        self.assertGreater(cfg.target_switch_min_s, 0.0)
        self.assertLess(cfg.target_switch_min_s, cfg.target_switch_grace_s,
                        "switch window must fit inside grace or it never applies")
        self.assertGreater(cfg.target_depth_continuity_m, 0.0)
        self.assertGreater(cfg.tracklet_depth_gate_m, 0.0)

    # ── (1) None-id path: closer person at the operator's x ─────────────────
    def test_none_id_closer_person_at_same_x_does_not_steal(self):
        trk = _tracker()
        t = 100.0
        op = trk.update([_det(normalized_x=0.08, depth_m=3.0)], now=t)
        self.assertEqual(op.depth_m, 3.0)

        # Next frame: operator drifted to x=0.16; a closer person appears at
        # x=0.05 (nearer to the held x=0.08 than the operator now is) at 1.2 m.
        t += 0.05
        st = trk.update([
            _det(normalized_x=0.16, depth_m=3.0),
            _det(normalized_x=0.05, depth_m=1.2, confidence=0.95),
        ], now=t)
        self.assertEqual(st.depth_m, 3.0, "lock must stay on the 3 m operator")
        self.assertAlmostEqual(trk.fresh_raw_x_norm, 0.16)

    def test_depth_gate_disabled_restores_x_only_rule(self):
        """Documents the knob: depth_continuity_m=0 is the pre-fix x-only match
        (and therefore the pre-fix steal)."""
        trk = _tracker(depth_continuity_m=0.0)
        t = 100.0
        trk.update([_det(normalized_x=0.08, depth_m=3.0)], now=t)
        t += 0.05
        st = trk.update([
            _det(normalized_x=0.16, depth_m=3.0),
            _det(normalized_x=0.05, depth_m=1.2, confidence=0.95),
        ], now=t)
        self.assertEqual(st.depth_m, 1.2)

    def test_switch_window_zero_restores_immediate_handoff(self):
        """Documents the knob: switch_min_s=0 + acquire_min_frames=1 is the
        pre-fix immediate hand-off — the occluder takes the lock on frame one."""
        trk = _tracker(switch_min_s=0.0, acquire_min_frames=1)
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        t += 0.05
        st = trk.update([_det(normalized_x=0.0, depth_m=1.2, confidence=0.95, track_id=2)], now=t)
        self.assertEqual(st.track_id, 2)

    # ── (2) Id'd path: occluder present while the operator is hidden ────────
    def test_occluder_does_not_take_lock_within_switch_window(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)

        # Closer person (id 2, high conf) walks in front; operator hidden for
        # 0.5 s (10 frames at 20 Hz) — well inside target_switch_min_s.
        for _ in range(10):
            t += 0.05
            st = trk.update([_det(normalized_x=0.0, depth_m=1.2, confidence=0.95, track_id=2)], now=t)
            self.assertIsNotNone(st, "grace-hold must keep the held state")
            self.assertEqual(st.track_id, 1, "occluder must not take the lock")
            self.assertIsNone(trk.fresh_raw_x_norm, "grace-hold frames are not fresh")

        # Operator reappears (same id, plausible depth) → fresh lock on them.
        t += 0.05
        st = trk.update([
            _det(normalized_x=0.05, depth_m=2.9, track_id=1),
            _det(normalized_x=0.0, depth_m=1.2, confidence=0.95, track_id=2),
        ], now=t)
        self.assertEqual(st.track_id, 1)
        self.assertEqual(st.depth_m, 2.9)
        self.assertAlmostEqual(trk.fresh_raw_x_norm, 0.05)

    def test_sustained_challenger_eventually_wins(self):
        """A genuinely new person (operator gone) still gets the lock — after
        target_switch_min_s, not on frame one, and before grace expires."""
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        appeared = t + 0.05
        switched_at = None
        for _ in range(28):  # 1.4 s at 20 Hz — inside the 1.5 s grace window
            t += 0.05
            st = trk.update([_det(normalized_x=0.0, depth_m=1.2, confidence=0.95, track_id=2)], now=t)
            if st is not None and st.track_id == 2:
                switched_at = t
                break
        self.assertIsNotNone(switched_at, "sustained challenger must be adopted")
        self.assertGreaterEqual(switched_at - appeared, SWITCH_MIN_S - 1e-9)
        self.assertLessEqual(switched_at - 100.0, GRACE)
        self.assertAlmostEqual(trk.fresh_raw_x_norm, 0.0)

    def test_challenger_streak_resets_when_operator_reappears(self):
        """Operator blinks twice with the occluder present between — the
        challenger's streak must restart each time, never accumulate."""
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        for _ in range(3):
            for _ in range(15):  # 0.75 s occluded
                t += 0.05
                trk.update([_det(normalized_x=0.0, depth_m=1.2, confidence=0.95, track_id=2)], now=t)
            t += 0.05
            st = trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
            self.assertEqual(st.track_id, 1)
            self.assertIsNotNone(trk.fresh_raw_x_norm)

    def test_different_confirmed_id_at_operator_position_is_not_the_operator(self):
        # Contract change 2026-09-20: a LONE candidate with a new confirmed id
        # rebinds (tracklet churn on a single operator). The original rule
        # still holds for multi-candidate frames — uniqueness is the steal
        # defence — so this test now includes a second body.
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        t += 0.05
        st = trk.update([
            _det(normalized_x=0.05, depth_m=3.0, confidence=0.95, track_id=2),
            _det(normalized_x=0.40, depth_m=2.0, confidence=0.90, track_id=3),
        ], now=t)
        self.assertEqual(st.track_id, 1, "a different confirmed id must earn the lock")
        self.assertIsNone(trk.fresh_raw_x_norm)

    # ── (3) Id transferred onto a closer occluder ────────────────────────────
    def test_id_match_with_depth_jump_is_rejected(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        # Tracklet layer handed id 1 to the occluder at 1.2 m; the operator is
        # still visible at 3.0 m as a new tentative (None-id) detection.
        t += 0.05
        st = trk.update([
            _det(normalized_x=0.0, depth_m=1.2, confidence=0.95, track_id=1),
            _det(normalized_x=0.05, depth_m=3.0, confidence=0.8, track_id=None),
        ], now=t)
        self.assertEqual(st.depth_m, 3.0, "depth-inconsistent id match must not win")
        self.assertIsNone(st.track_id, "tracker adopts the tentative detection at our position")
        self.assertAlmostEqual(trk.fresh_raw_x_norm, 0.05)

    def test_id_churn_after_occlusion_is_bridged(self):
        """Operator re-emerges with a new tracklet (None for min_hits frames,
        then a fresh id) — the lock follows without a grace-hold or hand-off."""
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        for _ in range(4):
            t += 0.05
            trk.update([], now=t)  # brief dropout
        t += 0.05
        st = trk.update([_det(normalized_x=0.05, depth_m=2.9, track_id=None)], now=t)
        self.assertIsNotNone(trk.fresh_raw_x_norm)
        self.assertIsNone(st.track_id)
        t += 0.05
        st = trk.update([_det(normalized_x=0.06, depth_m=2.9, track_id=7)], now=t)
        self.assertIsNotNone(trk.fresh_raw_x_norm)
        self.assertEqual(st.track_id, 7)

    # ── Regression guards ────────────────────────────────────────────────────
    def test_single_person_closing_fast_stays_fresh(self):
        trk = _tracker()
        t = 100.0
        for depth in (3.0, 2.6, 2.2, 1.9, 1.6):
            t += 0.1
            st = trk.update([_det(normalized_x=0.0, depth_m=depth)], now=t)
            self.assertIsNotNone(trk.fresh_raw_x_norm, f"depth {depth} must be fresh")
            self.assertEqual(st.depth_m, depth)

    def test_depth_tolerance_grows_with_absence(self):
        """After coasting 1.0 s the operator may have walked 1.5 m; a match at
        that distance is still them."""
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=1)], now=t)
        t += 1.0
        st = trk.update([_det(normalized_x=0.0, depth_m=1.6, track_id=1)], now=t)
        self.assertEqual(st.depth_m, 1.6)
        self.assertIsNotNone(trk.fresh_raw_x_norm)


class TestTrackletDepthGate(unittest.TestCase):
    """Host-side tracklet layer: association must respect depth, not just IoU."""

    OP_BOX = (0.45, 0.2, 0.55, 0.6)          # operator at 3 m, small box
    OCCLUDER_BOX = (0.42, 0.1, 0.58, 0.7)    # closer, bigger, overlapping (IoU ≈ 0.42)

    def _confirm_operator(self, tracker: TrackletTracker) -> int:
        ids = None
        for _ in range(4):
            ids = tracker.update([self.OP_BOX], depths=[3.0])
        self.assertIsNotNone(ids[0], "operator tracklet must be confirmed")
        return ids[0]

    def test_without_gate_occluder_inherits_operator_id(self):
        """Shows the pre-fix failure the gate exists for (gate disabled)."""
        tracker = TrackletTracker(iou_threshold=0.3, min_hits=3, max_age=15, depth_gate_m=0.0)
        op_id = self._confirm_operator(tracker)
        ids = tracker.update([self.OCCLUDER_BOX], depths=[1.2])
        self.assertEqual(ids[0], op_id)

    def test_gate_blocks_id_transfer_to_closer_occluder(self):
        tracker = TrackletTracker(
            iou_threshold=0.3, min_hits=3, max_age=15,
            depth_gate_m=0.75, depth_gate_growth_m_per_frame=0.1,
        )
        op_id = self._confirm_operator(tracker)
        ids = tracker.update([self.OCCLUDER_BOX], depths=[1.2])
        self.assertIsNone(ids[0], "occluder must start a NEW tentative tracklet")
        # Operator's tracklet survives (aged one frame) and re-attaches when
        # their box is visible again at a consistent depth.
        self.assertIn(op_id, [t.track_id for t in tracker.tracklets])
        ids = tracker.update([self.OP_BOX, self.OCCLUDER_BOX], depths=[3.0, 1.2])
        self.assertEqual(ids[0], op_id)
        self.assertNotEqual(ids[1], op_id)

    def test_gate_widens_with_missed_frames(self):
        tracker = TrackletTracker(
            iou_threshold=0.3, min_hits=3, max_age=15,
            depth_gate_m=0.75, depth_gate_growth_m_per_frame=0.1,
        )
        op_id = self._confirm_operator(tracker)
        for _ in range(5):
            tracker.update([], depths=[])
        # 5 missed frames → tolerance 0.75 + 0.1·5 = 1.25 m ≥ the 1.0 m change.
        ids = tracker.update([self.OP_BOX], depths=[2.0])
        self.assertEqual(ids[0], op_id)

    def test_no_depths_keeps_bbox_only_behaviour(self):
        tracker = TrackletTracker(
            iou_threshold=0.3, min_hits=3, max_age=15,
            depth_gate_m=0.75, depth_gate_growth_m_per_frame=0.1,
        )
        ids = None
        for _ in range(4):
            ids = tracker.update([self.OP_BOX])  # legacy call shape
        self.assertIsNotNone(ids[0])

    def test_live_yolo_path_supplies_depths_to_tracker(self):
        """OakDepthReader._assign_track_ids (the live YOLOv8 host-NMS path)
        must pass each person's z_m to the tracklet layer."""
        recorded: dict = {}

        class _RecordingTracker:
            def update(self, detections, depths=None):
                recorded["detections"] = list(detections)
                recorded["depths"] = list(depths) if depths is not None else None
                return [None] * len(detections)

        reader = OakDepthReader.__new__(OakDepthReader)
        reader._tracklet_tracker = _RecordingTracker()
        persons = [
            PersonDetection(x_m=0.0, z_m=2.5, confidence=0.9, bbox=(0.4, 0.0, 0.6, 0.8)),
            PersonDetection(x_m=0.5, z_m=1.1, confidence=0.8, bbox=(0.7, 0.0, 0.9, 0.9)),
        ]
        out = reader._assign_track_ids(persons)
        self.assertEqual(recorded["depths"], [2.5, 1.1])
        self.assertEqual(len(out), 2)

    def test_reader_constructs_tracker_with_config_gate(self):
        cfg = FollowMeConfig()
        tracker = TrackletTracker(
            iou_threshold=cfg.tracklet_iou_threshold,
            min_hits=cfg.tracklet_min_hits,
            max_age=cfg.tracklet_max_age,
            depth_gate_m=cfg.tracklet_depth_gate_m,
            depth_gate_growth_m_per_frame=cfg.tracklet_depth_gate_growth_m_per_frame,
        )
        self.assertGreater(tracker._depth_gate_m, 0.0)


class TestSingleCandidateRebind(unittest.TestCase):
    """2026-09-20: lone operator whose tracklet id churns stays the target."""

    def test_lone_operator_id_flip_stays_fresh(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=272)], now=t)
        t += 0.05
        st = trk.update(
            [_det(normalized_x=0.0, depth_m=3.0, confidence=0.9, track_id=271)],
            now=t,
        )
        self.assertIsNotNone(trk.fresh_raw_x_norm)
        self.assertEqual(st.track_id, 271)

    def test_id_flip_with_second_candidate_is_not_fresh(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=272)], now=t)
        t += 0.05
        st = trk.update([
            _det(normalized_x=0.0, depth_m=3.0, confidence=0.9, track_id=271),
            _det(normalized_x=0.4, depth_m=2.5, confidence=0.9, track_id=99),
        ], now=t)
        self.assertEqual(st.track_id, 272)
        self.assertIsNone(trk.fresh_raw_x_norm)

    def test_lone_new_id_1_5m_closer_is_not_fresh(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=272)], now=t)
        t += 0.05
        st = trk.update(
            [_det(normalized_x=0.0, depth_m=1.5, confidence=0.9, track_id=271)],
            now=t,
        )
        self.assertEqual(st.track_id, 272)
        self.assertIsNone(trk.fresh_raw_x_norm)

    def test_lone_new_id_below_acquire_floor_is_not_fresh(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=272)], now=t)
        t += 0.05
        st = trk.update(
            [_det(normalized_x=0.0, depth_m=3.0, confidence=0.5, track_id=271)],
            now=t,
        )
        self.assertEqual(st.track_id, 272)
        self.assertIsNone(trk.fresh_raw_x_norm)

    def test_lone_new_id_after_grace_must_earn_the_lock(self):
        # Both rebind gates widen with age; past switch_grace_s they would
        # admit anyone, so the exception is bounded by the grace window.
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=272)], now=t)
        t += 5.0
        trk.update(
            [_det(normalized_x=0.0, depth_m=3.0, confidence=0.9, track_id=271)],
            now=t,
        )
        self.assertIsNone(trk.fresh_raw_x_norm)

    def test_committed_id_depth_inconsistent_alone_is_not_fresh(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=3.0, track_id=272)], now=t)
        t += 0.05
        st = trk.update(
            [_det(normalized_x=0.0, depth_m=1.5, confidence=0.9, track_id=272)],
            now=t,
        )
        self.assertEqual(st.track_id, 272)
        self.assertEqual(st.depth_m, 3.0)
        self.assertIsNone(trk.fresh_raw_x_norm)


if __name__ == "__main__":
    unittest.main()
