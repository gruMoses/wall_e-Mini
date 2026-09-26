"""Corridor speckle filter and depth-aware person mask (2026-09-26).

Field evidence: 45 raw frames (logs/depth_snapshots on the robot). False
stereo matches on an iron gate gave 1,000-2,000 near pixels as tiny blobs
(largest 95 px) and a 0.70-0.81 m phantom. Real obstacles (gate bars, a
trash can, an umbrella pole, siding, woven wire fence) were kept at 80 px.
The person mask used to blank the whole person box, which also hid an
obstacle between the robot and the person.
"""

import unittest

import numpy as np

try:
    import cv2  # noqa: F401
except ImportError:
    cv2 = None

from config import FollowMeConfig, ObstacleAvoidanceConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.hardware.oak_depth import (
    OakDepthReader,
    _drop_small_near_blobs,
    _mask_person_boxes,
)

H, W = 400, 640
SLOW_MM = 1500.0


class _FakeFrame:
    def __init__(self, array):
        self._arr = array

    def getFrame(self):
        return self._arr


class _FakeQueue:
    def __init__(self, frame=None):
        self._frame = frame
        self._served = False

    def tryGet(self):
        if self._frame is not None and not self._served:
            self._served = True
            return self._frame
        return None


def _reader(**obs_overrides) -> OakDepthReader:
    kwargs = dict(
        slow_distance_m=1.5,
        stop_distance_m=0.4,
        roi_width_pct=0.8,
        roi_height_pct=0.5,
        robot_width_m=0.820,
        min_depth_mm=350,
        min_valid_pct=8.0,
        safety_stop_radius_m=0.8,
        corridor_min_support_px=400,
        corridor_min_support_frac=0.02,
        corridor_persistence_polls=2,
    )
    kwargs.update(obs_overrides)
    return OakDepthReader(
        obstacle_config=ObstacleAvoidanceConfig(**kwargs),
        follow_me_config=FollowMeConfig(),
    )


def _poll_twice(reader, frame):
    # Two polls: the corridor needs two consecutive near readings.
    for _ in range(2):
        reader._poll_depth(_FakeQueue(_FakeFrame(frame)), _FakeQueue(None), np)
    dist_m, _ = reader.get_min_distance()
    return dist_m, reader.get_depth_stats()


def _speckle_frame(background_mm=3000, speckle_mm=700):
    """Isolated near pixels (every 4th row and column) over a far wall."""
    frame = np.full((H, W), background_mm, dtype=np.uint16)
    frame[104:296:4, 120:560:4] = speckle_mm   # 48 x 110 = 5,280 singletons
    return frame


@unittest.skipUnless(cv2 is not None, "OpenCV not installed")
class TestDropSmallNearBlobs(unittest.TestCase):
    def _band(self, fill=3000):
        return np.full((200, W), fill, dtype=np.uint16)

    def test_isolated_near_pixels_are_dropped(self):
        band = self._band()
        band[::4, ::4] = 700
        valid = band > 350
        kept, dropped = _drop_small_near_blobs(band, valid, SLOW_MM, 80)
        self.assertEqual(dropped, int((band == 700).sum()))
        self.assertFalse((kept & (band <= SLOW_MM)).any())
        self.assertTrue(kept[band == 3000].all())  # far pixels untouched

    def test_solid_block_is_kept_and_speckle_around_it_dropped(self):
        band = self._band()
        # Offset grid: no speckle pixel touches the block. A speckle pixel
        # 8-adjacent to a real blob joins that blob and is kept (by design).
        band[2::4, 2::4] = 600
        band[80:120, 300:340] = 900                # 1,600 px solid
        valid = band > 350
        kept, dropped = _drop_small_near_blobs(band, valid, SLOW_MM, 80)
        self.assertTrue(kept[80:120, 300:340].all())
        self.assertFalse(kept[band == 600].any())
        self.assertEqual(dropped, int((band == 600).sum()))

    def test_connected_thin_lattice_is_kept(self):
        # A wire-fence-like grid of 1 px lines is one 8-connected blob.
        band = self._band()
        band[20:180:10, 100:500] = 1000
        band[20:180, 100:500:10] = 1000
        valid = band > 350
        kept, dropped = _drop_small_near_blobs(band, valid, SLOW_MM, 80)
        self.assertEqual(dropped, 0)
        self.assertTrue(kept[band == 1000].all())

    def test_threshold_boundary(self):
        band = self._band()
        band[10, 10:89] = 800        # 79 px line: dropped
        band[50, 10:90] = 800        # 80 px line: kept
        valid = band > 350
        kept, dropped = _drop_small_near_blobs(band, valid, SLOW_MM, 80)
        self.assertEqual(dropped, 79)
        self.assertFalse(kept[10, 10:89].any())
        self.assertTrue(kept[50, 10:90].all())

    def test_zero_disables(self):
        band = self._band()
        band[::4, ::4] = 700
        valid = band > 350
        kept, dropped = _drop_small_near_blobs(band, valid, SLOW_MM, 0)
        self.assertEqual(dropped, 0)
        self.assertIs(kept, valid)

    def test_isolated_far_pixels_are_never_dropped(self):
        band = np.zeros((200, W), dtype=np.uint16)
        band[::4, ::4] = 2500
        valid = band > 350
        kept, dropped = _drop_small_near_blobs(band, valid, SLOW_MM, 80)
        self.assertEqual(dropped, 0)
        self.assertTrue((kept == valid).all())


class TestMaskPersonBoxes(unittest.TestCase):
    # Band = rows 100..300 of a 400-row frame (y0 = 100).
    BOX = (0.40, 0.00, 0.60, 1.00)   # columns 256..384, all band rows

    def _band(self):
        band = np.full((200, W), 4000, dtype=np.uint16)
        band[:, 256:384] = 2500          # the person
        band[120:180, 290:350] = 1000    # an obstacle in front of the person
        return band

    def test_obstacle_nearer_than_the_person_stays(self):
        band = self._band()
        out = _mask_person_boxes(band, [(self.BOX, 2.5)], 100, H, W, 0.5)
        self.assertTrue((out[120:180, 290:350] == 1000).all())
        person = np.zeros_like(band, dtype=bool)
        person[:, 256:384] = True
        person[120:180, 290:350] = False
        self.assertTrue((out[person] == 0).all())
        self.assertTrue((out[:, :256] == 4000).all())   # outside the box
        self.assertTrue((band[:, 256:384] > 0).all())   # input not written

    def test_unknown_range_person_is_not_masked(self):
        band = self._band()
        out = _mask_person_boxes(band, [(self.BOX, 0.0)], 100, H, W, 0.5)
        self.assertIs(out, band)

    def test_zero_margin_restores_the_whole_box_mask(self):
        band = self._band()
        out = _mask_person_boxes(band, [(self.BOX, 2.5)], 100, H, W, 0.0)
        self.assertTrue((out[:, 256:384] == 0).all())
        self.assertTrue((band[120:180, 290:350] == 1000).all())

    def test_box_rows_are_clipped_to_the_band(self):
        band = self._band()
        box = (0.40, 0.60, 0.60, 1.00)   # frame rows 240..400 -> band 140..200
        out = _mask_person_boxes(band, [(box, 2.5)], 100, H, W, 0.5)
        self.assertTrue((out[:140, 256:384][band[:140, 256:384] == 2500] == 2500).all())
        self.assertTrue((out[140:, 256:384][band[140:, 256:384] == 2500] == 0).all())


@unittest.skipUnless(cv2 is not None, "OpenCV not installed")
class TestPollDepthSpeckleAndMask(unittest.TestCase):
    def test_speckle_field_no_longer_reports_a_phantom(self):
        legacy, _ = _poll_twice(_reader(corridor_min_blob_px=0), _speckle_frame())
        self.assertAlmostEqual(legacy, 0.7, places=1)   # the old phantom
        dist, stats = _poll_twice(_reader(corridor_min_blob_px=80), _speckle_frame())
        self.assertGreater(dist, 2.9)
        self.assertEqual(stats.corridor_speckle_px, 48 * 110 - self._outside(_speckle_frame()))

    @staticmethod
    def _outside(frame):
        # Speckles outside the lateral corridor never reach the filter.
        cfg = ObstacleAvoidanceConfig()
        r = OakDepthReader(obstacle_config=cfg, follow_me_config=FollowMeConfig())
        fx, _fy, cx, _cy = r._intrinsics_for(W, H)
        xs = np.arange(120, 560, 4)
        inside = (700.0 * np.abs(xs - cx)) <= fx * cfg.robot_width_m * 500.0
        return 48 * int((~inside).sum())

    def test_real_obstacle_among_speckle_is_reported(self):
        frame = _speckle_frame(speckle_mm=500)
        frame[140:260, 290:350] = 900            # 7,200 px solid, in corridor
        dist, _ = _poll_twice(_reader(corridor_min_blob_px=80), frame)
        self.assertAlmostEqual(dist, 0.9, places=1)

    def _person_scene(self, person_z_m):
        frame = np.full((H, W), 4000, dtype=np.uint16)
        frame[:, 256:384] = 2500                  # the person
        frame[160:260, 290:350] = 1000            # obstacle in front of them
        reader_persons = [PersonDetection(
            x_m=0.0, z_m=person_z_m, confidence=0.9,
            bbox=(0.40, 0.00, 0.60, 1.00), track_id=1,
        )]
        return frame, reader_persons

    def test_obstacle_between_robot_and_person_is_seen(self):
        frame, persons = self._person_scene(2.5)
        reader = _reader(corridor_min_blob_px=80, person_mask_depth_margin_m=0.5)
        reader._det_state.persons = persons
        dist, _ = _poll_twice(reader, frame)
        self.assertAlmostEqual(dist, 1.0, places=1)

    def test_whole_box_mask_hid_that_obstacle(self):
        frame, persons = self._person_scene(2.5)
        reader = _reader(corridor_min_blob_px=80, person_mask_depth_margin_m=0.0)
        reader._det_state.persons = persons
        dist, _ = _poll_twice(reader, frame)
        self.assertGreater(dist, 2.0)

    def test_unknown_range_person_counts_as_depth(self):
        frame = np.full((H, W), 4000, dtype=np.uint16)
        frame[:, 256:384] = 1200                  # a close person, range unknown
        reader = _reader(corridor_min_blob_px=80, person_mask_depth_margin_m=0.5)
        reader._det_state.persons = [PersonDetection(
            x_m=0.0, z_m=0.0, confidence=0.9,
            bbox=(0.40, 0.00, 0.60, 1.00), track_id=1, depth_status="no_support",
        )]
        dist, _ = _poll_twice(reader, frame)
        self.assertAlmostEqual(dist, 1.2, places=1)


if __name__ == "__main__":
    unittest.main()
