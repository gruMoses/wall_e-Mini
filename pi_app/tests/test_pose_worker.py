"""Pure crop / landmark mapping tests for the arms-up pose worker.

No MediaPipe import: the mapping helpers are ordinary arithmetic.
"""

import unittest

from pi_app.hardware.pose_worker import (
    det_bbox_to_frame,
    landmark_to_full_frame,
    person_crop_box,
)


class TestPersonCropBox(unittest.TestCase):
    def test_bbox_at_frame_edge_clamps(self):
        # Expand would go negative / past the far edge.
        x0, y0, x1, y1 = person_crop_box((0.0, 0.0, 0.2, 0.2), 100, 80)
        self.assertGreaterEqual(x0, 0)
        self.assertGreaterEqual(y0, 0)
        self.assertLessEqual(x1, 100)
        self.assertLessEqual(y1, 80)
        self.assertEqual(x0, 0)
        self.assertEqual(y0, 0)

        x0, y0, x1, y1 = person_crop_box((0.85, 0.80, 1.0, 1.0), 100, 80)
        self.assertGreaterEqual(x0, 0)
        self.assertGreaterEqual(y0, 0)
        self.assertEqual(x1, 100)
        self.assertEqual(y1, 80)

    def test_expanded_box_keeps_centre(self):
        x0, y0, x1, y1 = person_crop_box((0.25, 0.25, 0.75, 0.75), 100, 100)
        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        self.assertAlmostEqual(cx, 50.0, delta=1.0)
        self.assertAlmostEqual(cy, 50.0, delta=1.0)
        # 15 % wider, 10 % taller than the 50 px original.
        self.assertGreater(x1 - x0, 50)
        self.assertGreater(y1 - y0, 50)


class TestLandmarkToFullFrame(unittest.TestCase):
    def test_crop_centre_maps_to_bbox_centre(self):
        frame_w, frame_h = 100, 100
        bbox = (0.25, 0.25, 0.75, 0.75)
        crop = person_crop_box(bbox, frame_w, frame_h)
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
