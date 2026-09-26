"""Corridor speckle filter and depth-aware person mask (2026-09-26).

Field evidence: 45 raw frames (logs/depth_snapshots on the robot). False
stereo matches on an iron gate gave 1,000-2,000 sparse near pixels at
random depths and a 0.70-0.81 m phantom. A near pixel now needs 16 near
neighbours within one 100 mm depth bin in its 13 x 13 window; real
surfaces (gate bars, a trash can, siding, woven wire fence, also with 30
percent of pixels removed) keep their range. The person mask used to blank
the whole person box, which also hid an obstacle between the robot and the
person.
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
    _drop_unsupported_near_pixels,
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


WIN, NEED = 13, 16


@unittest.skipUnless(cv2 is not None, "OpenCV not installed")
class TestDropUnsupportedNearPixels(unittest.TestCase):
    def _band(self, fill=3000):
        return np.full((200, W), fill, dtype=np.uint16)

    def _run(self, band):
        valid = band > 350
        return valid, _drop_unsupported_near_pixels(band, valid, SLOW_MM, WIN, NEED)

    def test_sparse_random_depth_speckle_is_dropped(self):
        band = self._band()
        rng = np.random.default_rng(3)
        ys, xs = np.nonzero(rng.random(band.shape) < 0.03)
        band[ys, xs] = rng.integers(360, 1500, size=ys.size)
        valid, (kept, dropped) = self._run(band)
        near = valid & (band <= SLOW_MM)
        self.assertGreater(near.sum(), 3000)
        self.assertEqual(dropped, int(near.sum()))
        self.assertTrue(kept[band == 3000].all())   # far pixels untouched

    def test_solid_block_is_kept(self):
        band = self._band()
        band[80:120, 300:340] = 900
        valid, (kept, dropped) = self._run(band)
        self.assertEqual(dropped, 0)
        self.assertTrue(kept[80:120, 300:340].all())

    def test_fence_like_lattice_with_holes_is_kept(self):
        # 2 px wires every 12 px at ~1 m, then 30 percent of pixels removed.
        band = self._band()
        for y in range(20, 180, 12):
            band[y:y + 2, 100:500] = 1000
        for x in range(100, 500, 12):
            band[20:180, x:x + 2] = 1000
        rng = np.random.default_rng(5)
        band[(rng.random(band.shape) < 0.3) & (band == 1000)] = 0
        valid, (kept, dropped) = self._run(band)
        wire = band == 1000
        self.assertGreater(kept[wire].mean(), 0.95)

    def test_neighbour_count_boundary(self):
        band = self._band()
        band[50:54, 100:104] = 800       # 4 x 4 = 16 agreeing px: kept
        band[150:153, 100:105] = 800     # 3 x 5 = 15: dropped
        valid, (kept, dropped) = self._run(band)
        self.assertTrue(kept[50:54, 100:104].all())
        self.assertFalse(kept[150:153, 100:105].any())
        self.assertEqual(dropped, 15)

    def test_single_pixel_line_is_dropped_by_design(self):
        # A 13 px window holds at most 13 px of a 1 px line (< 16). Stereo
        # renders real wires 2-4 px wide; a lone strand is below the corridor
        # support floor with or without this filter.
        band = self._band()
        band[50, 100:300] = 800
        valid, (kept, dropped) = self._run(band)
        self.assertEqual(dropped, 200)

    def test_depths_outside_one_bin_do_not_count(self):
        band = self._band()
        band[50:54, 100:104] = 800
        band[50:52, 100:104] = 1200      # half the block is 400 mm away
        valid, (kept, dropped) = self._run(band)
        self.assertEqual(dropped, 16)

    def test_zero_disables(self):
        band = self._band()
        band[::4, ::4] = 700
        valid = band > 350
        kept, dropped = _drop_unsupported_near_pixels(band, valid, SLOW_MM, WIN, 0)
        self.assertEqual(dropped, 0)
        self.assertIs(kept, valid)


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
        legacy, _ = _poll_twice(_reader(corridor_speckle_min_neighbours=0), _speckle_frame())
        self.assertAlmostEqual(legacy, 0.7, places=1)   # the old phantom
        dist, stats = _poll_twice(_reader(), _speckle_frame())
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
        dist, _ = _poll_twice(_reader(), frame)
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
        reader = _reader(person_mask_depth_margin_m=0.5)
        reader._det_state.persons = persons
        dist, _ = _poll_twice(reader, frame)
        self.assertAlmostEqual(dist, 1.0, places=1)

    def test_whole_box_mask_hid_that_obstacle(self):
        frame, persons = self._person_scene(2.5)
        reader = _reader(person_mask_depth_margin_m=0.0)
        reader._det_state.persons = persons
        dist, _ = _poll_twice(reader, frame)
        self.assertGreater(dist, 2.0)

    def test_filter_never_turns_a_measurable_corridor_into_clear(self):
        # Sparse scene: valid density sits just above min_valid_pct and most
        # valid pixels are unsupported near specks. Dropping them must not
        # push the corridor under the density floor and publish "clear".
        frame = np.zeros((H, W), dtype=np.uint16)
        rng = np.random.default_rng(9)
        band = frame[100:300]
        ys, xs = np.nonzero(rng.random(band.shape) < 0.16)
        band[ys, xs] = rng.integers(360, 1500, size=ys.size)  # random-depth specks
        dist, stats = _poll_twice(_reader(), frame)
        legacy, _ = _poll_twice(_reader(corridor_speckle_min_neighbours=0), frame)
        self.assertLess(legacy, 1.5)
        self.assertLess(dist, 1.5)               # unfiltered reading stands
        self.assertTrue(stats.corridor_speckle_fallback)

    def test_filter_skipped_below_the_support_floor(self):
        # 300 isolated near specks: fewer than the support floor, so the
        # reading is beyond slow_distance_m either way; the filter does not
        # run (no cost on a clear path).
        frame = np.full((H, W), 3000, dtype=np.uint16)
        frame[120:280:8, 280:400:8] = 700
        dist, stats = _poll_twice(_reader(), frame)
        self.assertEqual(stats.corridor_speckle_px, 0)
        self.assertGreater(dist, 1.5)

    def test_filter_skipped_for_a_huge_near_region(self):
        # A wall at 1.0 m filling the band is a real surface: above
        # corridor_speckle_max_near_px the unfiltered reading stands.
        frame = np.full((H, W), 1000, dtype=np.uint16)
        frame[104:296:4, 120:560:4] = 400        # specks nearer than the wall
        dist, stats = _poll_twice(_reader(corridor_speckle_max_near_px=20000), frame)
        self.assertEqual(stats.corridor_speckle_px, 0)
        self.assertLess(dist, 1.0)                # cautious unfiltered reading
        dist2, stats2 = _poll_twice(_reader(corridor_speckle_max_near_px=10**9), frame)
        self.assertGreater(stats2.corridor_speckle_px, 0)
        self.assertAlmostEqual(dist2, 1.0, places=1)

    def test_unknown_range_person_counts_as_depth(self):
        frame = np.full((H, W), 4000, dtype=np.uint16)
        frame[:, 256:384] = 1200                  # a close person, range unknown
        reader = _reader(person_mask_depth_margin_m=0.5)
        reader._det_state.persons = [PersonDetection(
            x_m=0.0, z_m=0.0, confidence=0.9,
            bbox=(0.40, 0.00, 0.60, 1.00), track_id=1, depth_status="no_support",
        )]
        dist, _ = _poll_twice(reader, frame)
        self.assertAlmostEqual(dist, 1.2, places=1)


if __name__ == "__main__":
    unittest.main()
