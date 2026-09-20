"""Hardware-free tests for the 2026-09-20 person-depth sampler and safety-stop size backstop."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from config import FollowMeConfig, ObstacleAvoidanceConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.hardware.oak_depth import (
    OakDepthReader,
    sample_person_depth,
)
from tools.analyze_follow_me_log import analyze, render_report


def _fx_cx(frame):
    h, w = frame.shape
    fx = (w / 2.0) / math.tan(math.radians(35.0))
    return fx, w / 2.0


def _sample(bbox, frame, **over):
    cfg = FollowMeConfig()
    kwargs = dict(
        person_depth_sample_min_m=cfg.person_depth_sample_min_m,
        person_depth_sample_max_m=cfg.person_depth_sample_max_m,
        person_depth_min_valid_px=cfg.person_depth_min_valid_px,
        person_depth_min_valid_frac=cfg.person_depth_min_valid_frac,
        person_assumed_height_m=cfg.person_assumed_height_m,
        person_depth_height_veto_ratio=cfg.person_depth_height_veto_ratio,
        person_depth_fullheight_max_m=cfg.person_depth_fullheight_max_m,
        person_depth_max_spread_m=cfg.person_depth_max_spread_m,
        detect_camera_vfov_deg=cfg.detect_camera_vfov_deg,
    )
    kwargs.update(over)
    fx, cx = _fx_cx(frame) if frame is not None else (1.0, 0.0)
    return sample_person_depth(bbox, frame, fx, cx, **kwargs)


def _fill_norm(frame, xmin, ymin, xmax, ymax, mm):
    h, w = frame.shape
    x0, x1 = int(xmin * w), int(xmax * w)
    y0, y1 = int(ymin * h), int(ymax * h)
    x1 = max(x1, x0 + 1)
    y1 = max(y1, y0 + 1)
    frame[y0:y1, x0:x1] = mm


def _old_inner50_median_m(bbox, frame, min_m=0.5, max_m=6.0):
    """The pre-fix inner-50% median with a 6.0 m clip and no support floor."""
    x1, y1, x2, y2 = bbox
    dh, dw = frame.shape
    cx_d = int(((x1 + x2) / 2.0) * dw)
    cy_d = int(((y1 + y2) / 2.0) * dh)
    bw_half = max(1, int((x2 - x1) * 0.5 * dw / 2))
    bh_half = max(1, int((y2 - y1) * 0.5 * dh / 2))
    x0d = max(0, cx_d - bw_half)
    x1d = min(dw, cx_d + bw_half)
    y0d = max(0, cy_d - bh_half)
    y1d = min(dh, cy_d + bh_half)
    roi = frame[y0d:y1d, x0d:x1d]
    min_mm = int(min_m * 1000)
    max_mm = int(max_m * 1000)
    valid = roi[(roi > min_mm) & (roi < max_mm)]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid)) / 1000.0


class _StopDet:
    def __init__(self, safety_tier, z_m, depth_status="ok", bbox=(0.3, 0.1, 0.7, 0.8)):
        self.safety_tier = safety_tier
        self.z_m = z_m
        self.depth_status = depth_status
        self.bbox = bbox
        self.label_name = "person"


class TestPersonDepthSampler(unittest.TestCase):

    def test_far_person_keeps_pixels_past_old_6m_clip(self):
        # Documents the 6.0 m clip bug: operator at 6.1-6.3 m plus a few
        # stray nearer pixels. The old max_distance_m clip discarded the
        # person and the median landed on the strays.
        bbox = (0.20, 0.10, 0.80, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, *bbox, 6200)
        frame[100, 100:105] = 3000  # five stray pixels in the torso ROI
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ok")
        self.assertGreaterEqual(result.z_m, 6.1)
        self.assertLessEqual(result.z_m, 6.3)
        old_z = _old_inner50_median_m(bbox, frame)
        self.assertLess(old_z, 4.5)

    def test_five_valid_pixels_is_no_support(self):
        bbox = (0.20, 0.10, 0.80, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        frame[100, 100:105] = 2500
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "no_support")
        self.assertEqual(result.z_m, 0.0)
        self.assertAlmostEqual(result.z_stereo_m, 2.5, delta=0.05)
        self.assertEqual(result.depth_valid_px, 5)

    def test_right_edge_erosion_ignores_border_artifact(self):
        bbox = (0.82, 0.10, 1.00, 0.72)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, 0.82, 0.10, 0.91, 0.72, 2500)  # person, left part of box
        _fill_norm(frame, 0.91, 0.10, 1.00, 0.72, 900)   # dense 900 mm right-half artifact
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ok")
        self.assertAlmostEqual(result.z_m, 2.5, delta=0.15)

    def test_right_edge_artifact_whole_box_height_veto(self):
        # Ratio 0.55 is passed explicitly: production default is 0.0 (off).
        bbox = (0.82, 0.10, 1.00, 0.72)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, *bbox, 900)
        result = _sample(bbox, frame, person_depth_height_veto_ratio=0.55)
        self.assertEqual(result.depth_status, "height_veto")
        self.assertEqual(result.z_m, 0.0)
        self.assertAlmostEqual(result.z_stereo_m, 0.9, delta=0.05)
        self.assertGreater(result.z_height_m, 2.0)
        self.assertLess(result.z_stereo_m, 0.55 * result.z_height_m)

    def test_crouching_far_disagreement_never_vetoes(self):
        # Stereo 3.0 m, bbox height implies ~5.4 m. Ratio 0.55 is explicit
        # (production default is 0.0). 3.0/5.5=0.545 sits 0.005 under 0.55,
        # so a literal 5.5 m implied height would accidentally veto the
        # crouch this test exists to keep.
        cfg = FollowMeConfig()
        k = 2.0 * math.tan(math.radians(cfg.detect_camera_vfov_deg) / 2.0)
        z_height_target = 5.4
        bbox_h = cfg.person_assumed_height_m / (z_height_target * k)
        bbox = (0.35, 0.20, 0.65, 0.20 + bbox_h)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, *bbox, 3000)
        result = _sample(bbox, frame, person_depth_height_veto_ratio=0.55)
        self.assertEqual(result.depth_status, "ok")
        self.assertAlmostEqual(result.z_m, 3.0, delta=0.05)
        self.assertGreater(result.z_height_m, 5.0)

    def test_top_clipped_box_skips_height_veto(self):
        bbox = (0.35, 0.0, 0.65, 0.80)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, *bbox, 1200)
        result = _sample(bbox, frame, person_depth_height_veto_ratio=0.55)
        self.assertEqual(result.depth_status, "ok")
        self.assertAlmostEqual(result.z_m, 1.2, delta=0.05)
        self.assertEqual(result.z_height_m, 0.0)

    def test_default_config_never_hides_close_range(self):
        # 2026-09-20 15:49: true ~0.9-1.2 m stereo must stay "ok" under the
        # production default (veto ratio 0.0), even when bbox height implies
        # ~2.5 m on an unclipped box.
        cfg = FollowMeConfig()
        self.assertEqual(cfg.person_depth_height_veto_ratio, 0.0)
        k = 2.0 * math.tan(math.radians(cfg.detect_camera_vfov_deg) / 2.0)
        bbox_h = cfg.person_assumed_height_m / (2.5 * k)
        ymin = 0.04
        bbox = (0.30, ymin, 0.70, ymin + bbox_h)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, *bbox, 900)
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ok")
        self.assertAlmostEqual(result.z_m, 0.9, delta=0.05)
        self.assertAlmostEqual(result.z_height_m, 2.5, delta=0.15)

    def test_close_person_pixels_below_follow_min_are_kept(self):
        # H2a: 400 mm is below min_distance_m 0.5 but above the sampler
        # floor 0.30. 30 stray far pixels must not win the median.
        bbox = (0.20, 0.10, 0.80, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, 0.35, 0.26, 0.65, 0.54, 400)
        frame[100, 100:130] = 4000
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ok")
        self.assertAlmostEqual(result.z_m, 0.4, delta=0.05)
        self.assertLess(result.z_m, 1.0)

    def test_fullheight_background_is_far_veto(self):
        # H2b: box clipped top AND bottom, only background pixels valid.
        bbox = (0.40, 0.0, 0.60, 1.0)
        frame = np.zeros((200, 200), dtype=np.uint16)
        frame[50:52, 90:110] = 5000  # 40 px at 5.0 m, rest invalid
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "far_veto")
        self.assertEqual(result.z_m, 0.0)
        self.assertAlmostEqual(result.z_stereo_m, 5.0, delta=0.05)

    def test_unclipped_far_person_in_front_of_wall_is_ok(self):
        # H2b: same 40 background-range pixels, box NOT clipped — a far
        # small person in front of a wall is a legitimate measurement.
        bbox = (0.40, 0.10, 0.60, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        frame[60:62, 90:110] = 5000
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ok")
        self.assertAlmostEqual(result.z_m, 5.0, delta=0.05)

    def test_bimodal_roi_is_ambiguous(self):
        # H2c: two surfaces; median is not a measurement.
        bbox = (0.40, 0.10, 0.60, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        frame[60, 90:110] = 2600
        frame[61, 90:110] = 5800
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ambiguous")
        self.assertEqual(result.z_m, 0.0)
        self.assertGreater(result.z_spread_m, 1.0)

    def test_tight_far_cluster_is_ok(self):
        bbox = (0.20, 0.10, 0.80, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, 0.35, 0.26, 0.65, 0.54, 5800)
        frame[52, 70:130] = 5700
        frame[107, 70:130] = 5900
        result = _sample(bbox, frame)
        self.assertEqual(result.depth_status, "ok")
        self.assertGreaterEqual(result.z_m, 5.7)
        self.assertLessEqual(result.z_m, 5.9)

    def test_nan_bbox_publishes_no_frame(self):
        # H5: int(nan) must not freeze the person list.
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, 0.20, 0.10, 0.80, 0.90, 2500)
        reader = OakDepthReader(ObstacleAvoidanceConfig(), FollowMeConfig())
        nan_bbox = (float("nan"), 0.10, 0.80, 0.90)
        sample = reader._sample_person_depth_from_bbox(nan_bbox, frame)
        self.assertEqual(sample.depth_status, "no_frame")
        self.assertEqual(sample.z_m, 0.0)
        persons = [PersonDetection(
            x_m=sample.x_m, z_m=sample.z_m, confidence=0.9, bbox=nan_bbox,
            depth_status=sample.depth_status,
        )]
        self.assertEqual(len(persons), 1)
        self.assertEqual(persons[0].depth_status, "no_frame")
        # A later good box on the same reader still publishes.
        good = reader._sample_person_depth_from_bbox((0.20, 0.10, 0.80, 0.90), frame)
        self.assertEqual(good.depth_status, "ok")

    def test_compute_spatial_from_depth_still_returns_pair(self):
        bbox = (0.20, 0.10, 0.80, 0.90)
        frame = np.zeros((200, 200), dtype=np.uint16)
        _fill_norm(frame, *bbox, 2500)
        reader = OakDepthReader(ObstacleAvoidanceConfig(), FollowMeConfig())
        pair = reader._compute_spatial_from_depth(*bbox, frame)
        self.assertEqual(len(pair), 2)
        x_m, z_m = pair
        sample = reader._sample_person_depth_from_bbox(bbox, frame)
        self.assertAlmostEqual(x_m, sample.x_m)
        self.assertAlmostEqual(z_m, sample.z_m)
        self.assertEqual(sample.depth_status, "ok")


class TestSafetyStopSizeBackstop(unittest.TestCase):
    RADIUS = 0.8

    def test_ok_close_z_still_stops(self):
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [_StopDet("stop", 0.7, "ok")],
            corridor_p5_mm=3000.0,
            safety_stop_radius_m=self.RADIUS,
            safety_stop_bbox_width=0.40,
        )
        self.assertEqual(eff_mm, 0.0)
        self.assertIsNotNone(stop_det)

    def test_no_support_wide_bbox_stops_via_size(self):
        det = _StopDet("stop", 0.0, "no_support", bbox=(0.20, 0.10, 0.65, 0.80))  # width 0.45
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [det],
            corridor_p5_mm=3000.0,
            safety_stop_radius_m=self.RADIUS,
            safety_stop_bbox_width=0.40,
        )
        self.assertEqual(eff_mm, 0.0)
        self.assertIs(stop_det, det)

    def test_unknown_range_narrow_bbox_does_not_estop(self):
        # Incident-width box (0.18) with no range must not e-stop MANUAL
        # driving: the obstacle stop tier is mode-independent. Follow-me
        # handles this case itself with its close-by-geometry rule
        # (full-height box + unknown range => zero forward speed).
        det = _StopDet("stop", 0.0, "no_frame", bbox=(0.82, 0.10, 1.00, 0.72))  # width 0.18
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [det],
            corridor_p5_mm=3000.0,
            safety_stop_radius_m=self.RADIUS,
            safety_stop_bbox_width=0.40,
        )
        self.assertEqual(eff_mm, 3000.0)
        self.assertIsNone(stop_det)

    def test_zero_threshold_disables_size_rule(self):
        det = _StopDet("stop", 0.0, "no_support", bbox=(0.20, 0.10, 0.65, 0.80))  # width 0.45
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [det],
            corridor_p5_mm=3000.0,
            safety_stop_radius_m=self.RADIUS,
            safety_stop_bbox_width=0.0,
        )
        self.assertEqual(eff_mm, 3000.0)
        self.assertIsNone(stop_det)


class TestAnalyzerPersonDepth(unittest.TestCase):
    T0 = 1789861810.0

    def _write(self, objs) -> Path:
        self._tmp = tempfile.TemporaryDirectory()
        path = Path(self._tmp.name) / "run.jsonl"
        path.write_text(
            "".join(json.dumps(o) + "\n" for o in objs), encoding="utf-8",
        )
        return path

    def tearDown(self) -> None:
        tmp = getattr(self, "_tmp", None)
        if tmp is not None:
            tmp.cleanup()

    def test_report_with_new_fields(self):
        ticks = []
        statuses = ["ok", "ok", "no_support"]
        z_stereos = [5.8, 3.7, 3.5]  # 5.8 -> 3.7 is the one >1 m jump, same track
        valids = [40, 20, 5]
        for i, (st, z, vp) in enumerate(zip(statuses, z_stereos, valids)):
            ticks.append({
                "ts": self.T0 + i * 0.1,
                "ts_iso": f"2026-09-20 15:49:00.{i:03d}",
                "mode": "FOLLOW_ME",
                "detections": [{
                    "x_m": 0.1, "z_m": 0.0 if st != "ok" else z, "conf": 0.9,
                    "bbox": [0.4, 0.1, 0.6, 0.8], "track_id": 7,
                    "depth_status": st, "depth_valid_px": vp, "depth_roi_px": 200,
                    "z_stereo_m": z, "z_height_m": 6.0, "z_spread_m": 0.2,
                }],
                "oak": {"det_fps": 8.0, "depth_fps": 9.0, "det_latency_s": 0.08},
            })
        path = self._write(ticks)
        result = analyze([path])
        pd = result["person_depth"]
        self.assertEqual(pd["status_counts"]["ok"], 2)
        self.assertEqual(pd["status_counts"]["no_support"], 1)
        self.assertEqual(pd["n_z_stereo_jumps_same_track"], 1)
        self.assertEqual(pd["median_depth_valid_px"], 20)
        self.assertEqual(pd["median_det_fps"], 8.0)
        self.assertEqual(pd["median_depth_fps"], 9.0)
        report = render_report(result)
        self.assertIn("=== PERSON DEPTH ===", report)
        self.assertIn("ok=2", report)
        self.assertIn("no_support=1", report)

    def test_report_without_new_fields_is_na(self):
        ticks = [{
            "ts": self.T0,
            "ts_iso": "2026-09-20 15:49:00.000",
            "mode": "FOLLOW_ME",
            "detections": [{
                "x_m": 0.1, "z_m": 2.4, "conf": 0.9,
                "bbox": [0.4, 0.1, 0.6, 0.8],
            }],
        }]
        path = self._write(ticks)
        result = analyze([path])
        report = render_report(result)
        self.assertIn("=== PERSON DEPTH ===", report)
        self.assertIn("depth_status: n/a", report)
        self.assertIn("z_stereo_m jumps >1.0 m (same track_id): n/a", report)
        self.assertIn("median det_fps: n/a", report)
        self.assertIn("median depth_fps: n/a", report)


if __name__ == "__main__":
    unittest.main()
