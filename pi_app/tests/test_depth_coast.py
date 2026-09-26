"""Depth-unknown coast, and the 15:49 nearer-person speed cap.

Helpers mirror pi_app/tests/test_target_lock_steal.py (_det / _tracker).
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.append(str(Path(__file__).resolve().parents[3]))

from config import FollowMeConfig
from pi_app.control.follow_me import (
    DetectionFilter,
    FollowMeController,
    PersonDetection,
    TargetTracker,
    _FilteredDetection,
)


def _tracker(**overrides) -> TargetTracker:
    kw = dict(
        ema_alpha=1.0,
        persistence_s=2.0,
        switch_grace_s=1.5,
        acquire_confidence=0.65,
        acquire_min_frames=3,
        switch_min_s=1.0,
        depth_continuity_m=0.6,
        depth_continuity_rate_mps=1.5,
        depth_coast_max_s=1.0,
    )
    kw.update(overrides)
    return TargetTracker(**kw)


def _det(
    normalized_x=0.0,
    depth_m=3.0,
    confidence=0.9,
    track_id=None,
    depth_known=True,
) -> _FilteredDetection:
    return _FilteredDetection(
        normalized_x=normalized_x,
        x_m=normalized_x,
        depth_m=depth_m,
        confidence=confidence,
        bbox=(0.4, 0.0, 0.6, 0.8),
        track_id=track_id,
        depth_known=depth_known,
    )


class TestDepthUnknownCoast(unittest.TestCase):
    """TargetTracker: depth-unknown continues a lock; the gate stays for known range."""

    def test_same_id_unknown_depth_holds_range_and_is_fresh(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=5.8, track_id=10)], now=t)
        t += 0.1
        st = trk.update(
            [_det(normalized_x=0.05, depth_m=0.0, track_id=10, depth_known=False)],
            now=t,
        )
        self.assertIsNotNone(st)
        self.assertIsNotNone(trk.fresh_raw_x_norm)
        self.assertEqual(st.depth_m, 5.8)
        self.assertFalse(st.depth_known)
        self.assertEqual(st.track_id, 10)
        self.assertAlmostEqual(st.normalized_x, 0.05)
        self.assertIsNone(trk.last_reject_reason)

    def test_coast_expires_after_one_second(self):
        # H1c: timer starts on the FIRST unknown frame; expiry is >= max_s
        # from that instant (the first unknown used to be skipped because
        # _depth_coast_since was assigned in _apply_ema after the check).
        trk = _tracker()
        t = 100.0
        trk.update([_det(depth_m=5.8, track_id=10)], now=t)
        unk = _det(depth_m=0.0, track_id=10, depth_known=False)
        coast_start = None
        for i in range(1, 13):
            t = 100.0 + i * 0.1
            st = trk.update([unk], now=t)
            if coast_start is None:
                coast_start = t
            elapsed = t - coast_start
            if elapsed < 1.0 - 1e-9:
                self.assertIsNotNone(
                    trk.fresh_raw_x_norm, f"still fresh at coast {elapsed:.1f}s"
                )
                self.assertEqual(st.depth_m, 5.8)
                self.assertIsNone(trk.last_reject_reason)
            else:
                self.assertIsNone(
                    trk.fresh_raw_x_norm,
                    f"must expire at 1.0 s from first unknown (elapsed={elapsed})",
                )
                self.assertEqual(trk.last_reject_reason, "depth_coast_expired")
                self.assertEqual(st.depth_m, 5.8)

    def test_depth_returns_after_coast_uses_last_depth_time(self):
        """0.8 s of coast refreshes last_seen; continuity age must use last_depth_time.

        last_seen-based tol would be 0.6 + 1.5*0.1 = 0.75 m and would reject
        a 0.8 m close; last_depth_time gives 0.6 + 1.5*0.8 = 1.8 m.
        """
        trk = _tracker()
        t = 100.0
        trk.update([_det(depth_m=5.8, track_id=10)], now=t)
        unk = _det(depth_m=0.0, track_id=10, depth_known=False)
        for _ in range(7):
            t += 0.1
            st = trk.update([unk], now=t)
            self.assertIsNotNone(trk.fresh_raw_x_norm)
            self.assertEqual(st.depth_m, 5.8)
        t += 0.1  # 100.8 — 0.8 s since last known depth
        st = trk.update([_det(depth_m=5.0, track_id=10)], now=t)
        self.assertIsNotNone(trk.fresh_raw_x_norm)
        self.assertEqual(st.depth_m, 5.0)
        self.assertTrue(st.depth_known)
        self.assertIsNone(trk.last_reject_reason)

    def test_known_depth_jump_still_fails_the_gate(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(depth_m=5.8, track_id=10)], now=t)
        t += 0.05
        st = trk.update([_det(depth_m=4.0, track_id=10)], now=t)  # 1.8 m closer
        self.assertIsNotNone(st)
        self.assertEqual(st.track_id, 10)
        self.assertEqual(st.depth_m, 5.8)
        self.assertIsNone(trk.fresh_raw_x_norm)
        self.assertEqual(trk.last_reject_reason, "depth_gate")

    def test_cold_start_unknown_depth_does_not_lock(self):
        trk = _tracker()
        st = trk.update(
            [_det(depth_m=0.0, track_id=10, depth_known=False)],
            now=100.0,
        )
        self.assertIsNone(st)
        self.assertIsNone(trk.fresh_raw_x_norm)

    def test_unknown_different_id_with_second_body_does_not_rebind(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=5.8, track_id=10)], now=t)
        t += 0.05
        st = trk.update(
            [
                _det(normalized_x=0.0, depth_m=0.0, track_id=11, depth_known=False),
                _det(normalized_x=0.40, depth_m=2.0, confidence=0.90, track_id=12),
            ],
            now=t,
        )
        self.assertEqual(st.track_id, 10)
        self.assertEqual(st.depth_m, 5.8)
        self.assertIsNone(trk.fresh_raw_x_norm)

    def test_lone_unknown_new_id_rebinds_and_holds_depth(self):
        trk = _tracker()
        t = 100.0
        trk.update([_det(normalized_x=0.0, depth_m=5.8, track_id=10)], now=t)
        t += 0.05
        st = trk.update(
            [_det(normalized_x=0.0, depth_m=0.0, track_id=11, depth_known=False)],
            now=t,
        )
        self.assertIsNotNone(trk.fresh_raw_x_norm)
        self.assertEqual(st.track_id, 11)
        self.assertEqual(st.depth_m, 5.8)
        self.assertFalse(st.depth_known)


class TestDetectionFilterDepthUnknown(unittest.TestCase):
    """Layer 1: unknown-range detections skip depth-range and implied height."""

    _BBOX = (0.4, 0.1, 0.6, 0.9)  # not top-clipped; height would reject at z=0

    def _filter(self) -> DetectionFilter:
        cfg = FollowMeConfig()
        return DetectionFilter(
            conf_threshold=cfg.detection_confidence,
            min_depth_m=cfg.min_distance_m,
            max_depth_m=cfg.max_distance_m,
            min_bbox_area=cfg.min_bbox_area,
            edge_margin=cfg.detect_edge_margin,
            min_bbox_width=cfg.detect_min_bbox_width,
            min_person_height_m=cfg.detect_min_person_height_m,
            camera_vfov_deg=cfg.detect_camera_vfov_deg,
        )

    def _person(self, z_m, confidence=0.9, depth_status="ok") -> PersonDetection:
        return PersonDetection(
            x_m=0.0,
            z_m=z_m,
            confidence=confidence,
            bbox=self._BBOX,
            track_id=10,
            depth_status=depth_status,
        )

    def test_no_support_passes_with_depth_unknown(self):
        flt = self._filter()
        out = flt.process([self._person(z_m=0.0, depth_status="no_support")])
        self.assertEqual(len(out), 1)
        self.assertFalse(out[0].depth_known)
        self.assertEqual(out[0].depth_m, 0.0)

    def test_no_support_below_conf_rejected_as_conf(self):
        flt = self._filter()
        out = flt.process(
            [self._person(z_m=0.0, confidence=0.20, depth_status="no_support")]
        )
        self.assertEqual(len(out), 0)
        self.assertEqual(flt.last_reject_counts["conf"], 1)

    def test_known_range_beyond_max_rejected_as_depth_range(self):
        flt = self._filter()
        # One metre past the configured limit (7.0 was "beyond max" while
        # max_distance_m was 6.0; it is 8.0 since 2026-09-26).
        beyond = FollowMeConfig().max_distance_m + 1.0
        out = flt.process([self._person(z_m=beyond)])
        self.assertEqual(len(out), 0)
        self.assertEqual(flt.last_reject_counts["depth_range"], 1)


class TestControllerDepthCoastDoesNotGrowTrail(unittest.TestCase):
    """FollowMeController: no breadcrumb / world-position update while coasting."""

    def test_trail_length_does_not_grow_during_depth_coast(self):
        fm = FollowMeController(FollowMeConfig(
            trail_follow_enabled=True,
            trail_min_spacing_m=0.05,
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
        ))
        t = 100.0
        known = PersonDetection(
            x_m=0.0, z_m=3.0, confidence=0.9,
            bbox=(0.4, 0.0, 0.6, 0.8), track_id=10,
        )
        unknown = PersonDetection(
            x_m=0.0, z_m=0.0, confidence=0.9,
            bbox=(0.4, 0.0, 0.6, 0.8), track_id=10,
            depth_status="no_support",
        )
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(15):
                t += 0.1
                mt.monotonic.return_value = t
                fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)
                fm.compute([known])
            before = fm.get_status()
            length_before = before["trail_length"]
            total_before = before["trail_total_points"]
            self.assertGreater(length_before, 0)
            self.assertGreater(total_before, 0)

            for _ in range(5):
                t += 0.1
                mt.monotonic.return_value = t
                fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)
                fm.compute([unknown])
            after = fm.get_status()
            self.assertEqual(after["trail_length"], length_before)
            self.assertEqual(after["trail_total_points"], total_before)
            self.assertFalse(after["follow_me_depth_known"])
            self.assertGreater(after["follow_me_depth_coast_s"], 0.0)
            self.assertIsNotNone(fm._tracker.fresh_raw_x_norm)


class TestNearestPersonSpeedLimit(unittest.TestCase):
    """15:49 incident: identity gates must not hide a closer range from speed."""

    def _make(self, **overrides) -> FollowMeController:
        defaults = dict(
            trail_follow_enabled=False,
            follow_output_rate_hz=10000.0,
        )
        defaults.update(overrides)
        return FollowMeController(FollowMeConfig(**defaults))

    def _person(
        self,
        z_m: float,
        bbox=(0.4, 0.0, 0.6, 0.8),
        track_id=10,
        x_m=0.0,
        confidence=0.9,
        depth_status="ok",
    ) -> PersonDetection:
        return PersonDetection(
            x_m=x_m, z_m=z_m, confidence=confidence, bbox=bbox,
            track_id=track_id, depth_status=depth_status,
        )

    def test_incident_replay_true_1_2m_zeros_speed_in_one_tick(self):
        fm = self._make()
        op = self._person(z_m=3.0)
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(12):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            self.assertGreater(fm._last_speed_offset, 50.0)
            self.assertEqual(fm.get_status()["follow_me_target_track_id"], 10)

            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([self._person(
                z_m=1.2, bbox=(0.82, 0.0, 1.0, 1.0), track_id=10,
            )])
            self.assertEqual(fm._last_speed_offset, 0.0)
            status = fm.get_status()
            self.assertAlmostEqual(status["follow_me_speed_depth_m"], 1.2)
            self.assertAlmostEqual(status["follow_me_nearest_person_m"], 1.2)
            self.assertEqual(status["follow_me_target_track_id"], 10)

            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([self._person(
                z_m=1.2, bbox=(0.82, 0.0, 1.0, 1.0), track_id=10,
            )])
            self.assertEqual(fm._last_speed_offset, 0.0)

    def test_lost_target_pursuit_stops_for_a_visible_close_person(self):
        # Blind trail/search pursuit (no locked target) is forced to return a
        # forward command; a person seen inside follow_distance_m zeroes it.
        # conf 0.5 clears detection_confidence but not the 0.65 acquire floor,
        # so no lock forms and the lost-target branch runs.
        fm = self._make()
        close = self._person(z_m=1.0, confidence=0.5, track_id=None)
        with patch("pi_app.control.follow_me.time") as mt, \
                patch.object(fm, "_handle_lost_target", return_value=(60.0, 5.0)):
            mt.monotonic.return_value = 100.1
            fm.compute([close])
            self.assertEqual(fm._last_speed_offset, 0.0)
            self.assertEqual(fm._last_steer_offset, 5.0)
            mt.monotonic.return_value = 100.2
            fm.compute([self._person(z_m=4.0, confidence=0.5, track_id=None)])
            self.assertEqual(fm._last_speed_offset, 60.0)

    def test_other_person_closer_zeros_speed_lock_unchanged(self):
        fm = self._make()
        op = self._person(z_m=3.5, track_id=10)
        other = self._person(
            z_m=1.0, bbox=(0.10, 0.0, 0.30, 0.8), track_id=20, x_m=-0.8,
        )
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(8):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            self.assertGreater(fm._last_speed_offset, 0.0)
            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([op, other])
            self.assertEqual(fm._last_speed_offset, 0.0)
            self.assertEqual(fm.get_status()["follow_me_target_track_id"], 10)

    def test_not_fresh_hold_never_accelerates_past_last_fresh_speed(self):
        fm = self._make(detect_edge_margin=0.0)
        # cx=0.91 → normalized_x=0.82, above the 0.30 turn-speed knee.
        edge = self._person(z_m=4.0, bbox=(0.82, 0.0, 1.0, 0.8), track_id=1)
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(8):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([edge])
            s1 = fm._last_speed_offset
            self.assertGreater(s1, 0.0)
            self.assertLess(fm._turn_speed_scale, 1.0)
            for _ in range(2):
                t += 0.1  # 0.2 s, inside steer_hold_grace_s=0.30
                mt.monotonic.return_value = t
                fm.compute([])
                self.assertLessEqual(fm._last_speed_offset, s1 + 1e-6)

    def test_flag_off_restores_speed_from_locked_target_only(self):
        fm = self._make(nearest_person_speed_limit_enabled=False)
        op = self._person(z_m=3.5, track_id=10)
        other = self._person(
            z_m=1.0, bbox=(0.10, 0.0, 0.30, 0.8), track_id=20, x_m=-0.8,
        )
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(8):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([op, other])
            self.assertGreater(fm._last_speed_offset, 0.0)
            self.assertEqual(fm.get_status()["follow_me_target_track_id"], 10)

    def test_no_support_does_not_count_toward_nearest(self):
        # Contract change 2026-09-20 (H1b): a FULL-HEIGHT no_support box is
        # now a close-unknown stop (see test_close_unknown_full_height_zeros_speed).
        # This test only checks that z_m=0 / no_support is not a nearest_person
        # range, so the ghost is a SMALL box that the geometry rule ignores.
        fm = self._make()
        op = self._person(z_m=3.0, track_id=10)
        ghost = self._person(
            z_m=0.0, bbox=(0.40, 0.25, 0.60, 0.75), track_id=11,
            depth_status="no_support",
        )
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(8):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([op, ghost])
            status = fm.get_status()
            self.assertAlmostEqual(status["follow_me_nearest_person_m"], 3.0)
            self.assertGreater(fm._last_speed_offset, 0.0)
            self.assertFalse(status["follow_me_close_unknown"])

    def test_close_unknown_full_height_zeros_speed(self):
        fm = self._make()
        op = self._person(z_m=3.0, track_id=10)
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(12):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            self.assertGreater(fm._last_speed_offset, 50.0)
            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([self._person(
                z_m=0.0, bbox=(0.82, 0.0, 1.0, 1.0), track_id=10,
                depth_status="no_support",
            )])
            self.assertEqual(fm._last_speed_offset, 0.0)
            self.assertTrue(fm.get_status()["follow_me_close_unknown"])

    def test_h3_accepted_close_range_caps_while_depth_filter_lags(self):
        """Tracker accepts 1.2 m after the gate ages; DepthFilter still holds ~3.0.

        Speed must be 0 from the first 1.2 m reading (S1 vs smoothed depth),
        including the later FRESH ticks where raw_depth_m has already jumped.
        """
        fm = self._make()
        op = self._person(z_m=3.0, track_id=10)
        close = self._person(z_m=1.2, bbox=(0.82, 0.0, 1.0, 1.0), track_id=10)
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(8):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            self.assertGreater(fm._last_speed_offset, 0.0)
            for i in range(12):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([close])
                status = fm.get_status()
                self.assertAlmostEqual(
                    status["follow_me_speed_depth_m"], 1.2,
                    msg=f"tick {i}: speed_depth must stay on the 1.2 m person",
                )
                self.assertEqual(fm._last_speed_offset, 0.0, f"tick {i}")

    def test_h4_ramp_in_progress_not_fresh_never_rises(self):
        # Depth gate would reject 1.5 -> 4.0; this test is about SafetyLayer
        # vs last request, so disable continuity.
        fm = self._make(target_depth_continuity_m=0.0)
        hold = self._person(z_m=1.5, track_id=10)
        far = self._person(z_m=4.0, track_id=10)
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            t += 0.1
            mt.monotonic.return_value = t
            fm.compute([hold])  # seeds SafetyLayer at speed 0
            fm._depth_filter.reset()  # next 4.0 m must not bounce off the 5 m/s gate
            for _ in range(3):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([far])
            s_fresh = fm._last_speed_offset
            self.assertGreater(s_fresh, 0.0)
            self.assertLess(s_fresh, 80.0, "ramp must still be below the request")
            for _ in range(2):
                t += 0.1  # inside steer_hold_grace_s=0.30
                mt.monotonic.return_value = t
                fm.compute([])
                self.assertLessEqual(fm._last_speed_offset, s_fresh + 1e-6)

    def test_unknown_small_box_coast_does_not_rise_or_zero(self):
        fm = self._make()
        op = self._person(z_m=3.0, track_id=10)
        small = self._person(
            z_m=0.0, bbox=(0.40, 0.25, 0.60, 0.75), track_id=10,
            depth_status="no_support",
        )
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(12):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([op])
            s0 = fm._last_speed_offset
            self.assertGreater(s0, 50.0)
            for _ in range(4):
                t += 0.1
                mt.monotonic.return_value = t
                fm.compute([small])
                self.assertLessEqual(fm._last_speed_offset, s0 + 1e-6)
                self.assertGreater(fm._last_speed_offset, 0.0)
                self.assertFalse(fm.get_status()["follow_me_close_unknown"])
                self.assertFalse(fm.get_status()["follow_me_depth_known"])

    def test_lost_target_close_unknown_zeros_forward(self):
        fm = self._make()
        ghost = self._person(
            z_m=0.0, bbox=(0.82, 0.0, 1.0, 1.0), track_id=None,
            confidence=0.5, depth_status="no_support",
        )
        with patch("pi_app.control.follow_me.time") as mt, \
                patch.object(fm, "_handle_lost_target", return_value=(60.0, 5.0)):
            mt.monotonic.return_value = 100.1
            fm.compute([ghost])
            self.assertEqual(fm._last_speed_offset, 0.0)
            self.assertEqual(fm._last_steer_offset, 5.0)
            self.assertTrue(fm.get_status()["follow_me_close_unknown"])


if __name__ == "__main__":
    unittest.main()
