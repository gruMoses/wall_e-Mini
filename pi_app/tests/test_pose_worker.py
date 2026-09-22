"""Pure crop / landmark mapping tests for the arms-up pose worker.

No MediaPipe import: the mapping helpers are ordinary arithmetic.
"""

import unittest

from config import ArmsUpConfig
from pi_app.hardware.pose_worker import (
    PoseWorker,
    det_bbox_to_frame,
    expanded_person_crop,
    landmark_to_full_frame,
)


class TestExpandedPersonCrop(unittest.TestCase):
    def test_bbox_at_frame_edge_clamps(self):
        x0, y0, x1, y1, crop_y0, crop_y1 = expanded_person_crop(
            (0.0, 0.0, 0.2, 0.2), 100, 80,
        )
        self.assertEqual(x0, 0)
        self.assertEqual(y0, 0)
        self.assertLessEqual(x1, 100)
        self.assertLessEqual(y1, 80)
        self.assertGreaterEqual(crop_y0, 0.0)
        self.assertLessEqual(crop_y1, 1.0)
        self.assertAlmostEqual(crop_y0, y0 / 80.0)
        self.assertAlmostEqual(crop_y1, y1 / 80.0)

        x0, y0, x1, y1, crop_y0, crop_y1 = expanded_person_crop(
            (0.85, 0.80, 1.0, 1.0), 100, 80,
        )
        self.assertGreaterEqual(x0, 0)
        self.assertGreaterEqual(y0, 0)
        self.assertEqual(x1, 100)
        self.assertEqual(y1, 80)
        self.assertAlmostEqual(crop_y1, 1.0)
        self.assertAlmostEqual(crop_y0, y0 / 80.0)
        self.assertAlmostEqual(crop_y1, y1 / 80.0)

    def test_centre_preserved_horizontally(self):
        bbox = (0.25, 0.30, 0.55, 0.80)
        frame_w = 200
        x0, y0, x1, y1, crop_y0, crop_y1 = expanded_person_crop(
            bbox, frame_w, 100,
        )
        bbox_cx = 0.5 * (bbox[0] + bbox[2]) * frame_w
        self.assertAlmostEqual(0.5 * (x0 + x1), bbox_cx, delta=1.0)
        self.assertLess(crop_y0, bbox[1])
        self.assertGreater(crop_y1, bbox[3])
        self.assertAlmostEqual(crop_y0, y0 / 100.0)
        self.assertAlmostEqual(crop_y1, y1 / 100.0)


class TestLandmarkToFullFrame(unittest.TestCase):
    def test_crop_centre_maps_to_bbox_centre(self):
        frame_w, frame_h = 100, 100
        bbox = (0.25, 0.25, 0.75, 0.75)
        crop = (25, 25, 75, 75)
        nx, ny = landmark_to_full_frame(0.5, 0.5, crop, frame_w, frame_h)
        bbox_cx = 0.5 * (bbox[0] + bbox[2])
        bbox_cy = 0.5 * (bbox[1] + bbox[3])
        self.assertAlmostEqual(nx, bbox_cx, places=2)
        self.assertAlmostEqual(ny, bbox_cy, places=2)

    def test_crop_corners_map_to_crop_box_corners(self):
        crop = (10, 20, 50, 80)
        nw, nh = 100, 100
        x, y = landmark_to_full_frame(0.0, 0.0, crop, nw, nh)
        self.assertAlmostEqual(x, 0.10)
        self.assertAlmostEqual(y, 0.20)
        x, y = landmark_to_full_frame(1.0, 1.0, crop, nw, nh)
        self.assertAlmostEqual(x, 0.50)
        self.assertAlmostEqual(y, 0.80)


class TestDetBboxToFrame(unittest.TestCase):
    # NN 640x352, principal point on the vertical centre. Preview 640x480.
    DET_WH = (640, 352)
    DET_INTR = (456.89, 456.89, 320.0, 176.0)
    FRAME_WH = (640, 480)
    FRAME_INTR = (462.2, 462.2, 320.0, 240.0)

    def test_centred_box_stays_centred(self):
        mapped = det_bbox_to_frame(
            (0.40, 0.40, 0.60, 0.60),
            self.DET_WH, self.DET_INTR, self.FRAME_WH, self.FRAME_INTR,
        )
        self.assertAlmostEqual(0.5 * (mapped[0] + mapped[2]), 0.5, places=3)
        self.assertAlmostEqual(0.5 * (mapped[1] + mapped[3]), 0.5, places=3)

    def test_nn_top_maps_near_0_13(self):
        mapped = det_bbox_to_frame(
            (0.40, 0.0, 0.60, 0.20),
            self.DET_WH, self.DET_INTR, self.FRAME_WH, self.FRAME_INTR,
        )
        self.assertAlmostEqual(mapped[1], 0.13, delta=0.005)

    def test_fallback_ratio_maps_top_near_0_133(self):
        mapped = det_bbox_to_frame(
            (0.40, 0.0, 0.60, 0.20),
            self.DET_WH, None, self.FRAME_WH, None,
        )
        self.assertAlmostEqual(mapped[0], 0.40, places=5)
        self.assertAlmostEqual(mapped[1], 0.133, delta=0.002)


class TestDetectionsFresh(unittest.TestCase):
    def _worker(self, det_ts, max_age=0.3):
        class _Oak:
            def get_person_detections_ts(self):
                return det_ts

        return PoseWorker(_Oak(), ArmsUpConfig(max_det_age_s=max_age))

    def test_stamp_zero_is_false(self):
        worker = self._worker(0.0)
        self.assertFalse(worker._detections_fresh(0.0))
        self.assertFalse(worker._detections_fresh(0.1))

    def test_older_than_max_age_is_false(self):
        worker = self._worker(1.0, max_age=0.3)
        self.assertFalse(worker._detections_fresh(1.0 + 0.3 + 0.001))

    def test_within_max_age_is_true(self):
        worker = self._worker(1.0, max_age=0.3)
        self.assertTrue(worker._detections_fresh(1.0))
        self.assertTrue(worker._detections_fresh(1.3))


class _FakePose:
    def __init__(self):
        self.calls = []

    def process(self, image):
        self.calls.append(image)


class _FakeReader:
    def get_latest_rgb_frame(self):
        return ("frame", 1.0)


class TestProcessOnceDisabled(unittest.TestCase):
    def test_disabled_does_not_call_pose_process(self):
        pose = _FakePose()
        worker = PoseWorker(_FakeReader(), ArmsUpConfig())
        worker._pose = pose
        worker._enabled = False
        worker._process_once()
        self.assertEqual(pose.calls, [])
