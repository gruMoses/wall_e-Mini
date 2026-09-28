"""People and animals cap the relaxed MANUAL throttle curve (2026-09-27 review).

The relaxed MANUAL curve (0.5 at 0.50 m) is for things like netting over
the lens. It must never let the robot approach a person or an animal
faster than the old linear law did: without a cap, full stick reached the
0.8 m person stop at about 1.05 m/s instead of 0.60 m/s.

For each stop-tier detection nearer than slow_distance_m, the depth poll
reports a cap distance (OakDepthReader._manual_person_limit_mm):
  * the reading is more than person_mask_depth_margin_m in front of the
    detection -> something else is nearer; cap at the detection's range;
  * otherwise the reading may be the person or animal itself (animals and
    unranged persons are never masked out of the corridor) -> cap at
    min(reading, range);
  * unranged detection -> cap at the reading.
MANUAL throttle = min(relaxed curve(reading), old law(cap)).
"""

import time
import unittest

import numpy as np

from config import ObstacleAvoidanceConfig
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.hardware.oak_depth import OakDepthReader, _AllDetsState
from pi_app.tests.test_depth_corridor_stale import (
    _FakeFrame, _FakeQueue, _all_zeros_frame, _make_reader, _obstacle_frame,
)

INF = float("inf")


def old_law(d):
    return max(0.0, min(1.0, (d - 0.4) / (1.5 - 0.4)))


def new_curve(d):
    if d >= 1.5:
        return 1.0
    if d >= 0.5:
        return 0.5 + 0.5 * (d - 0.5) / 1.0
    return max(0.15, 0.15 + 0.35 * (d - 0.35) / 0.15)


class _Det:
    safety_tier = "stop"

    def __init__(self, label_name, z_m):
        self.label_name = label_name
        self.z_m = z_m


def _person(z):
    return _Det("person", z)


def _dog(z):
    return _Det("dog", z)


def _poll(reader, frame, dets=()):
    with reader._lock:
        reader._all_dets_state = _AllDetsState(detections=list(dets), timestamp=time.monotonic())
    reader._poll_depth(_FakeQueue(frame=_FakeFrame(frame)), _FakeQueue(frame=None), np)
    return reader.get_min_distance_detail()


def _manual_scale(dist, limit):
    c = ObstacleAvoidanceController(ObstacleAvoidanceConfig())
    return c.compute_throttle_scale(dist, 0.0, is_manual=True, manual_person_limit_m=limit)


class TestCapRule(unittest.TestCase):
    """The pure rule, in mm (slow 1.5 m, margin 0.5 m)."""

    def limit(self, dets, eff_mm):
        return OakDepthReader._manual_person_limit_mm(dets, eff_mm, 1.5, 0.5)

    def test_no_detection_no_cap(self):
        self.assertEqual(self.limit([], 500.0), INF)

    def test_far_detection_no_cap(self):
        self.assertEqual(self.limit([_person(2.5)], 500.0), INF)

    def test_something_well_in_front_of_the_person(self):
        self.assertEqual(self.limit([_person(1.2)], 500.0), 1200.0)

    def test_reading_may_be_the_animal_itself(self):
        self.assertEqual(self.limit([_dog(0.81)], 760.0), 760.0)

    def test_unranged_person_caps_at_the_reading(self):
        self.assertEqual(self.limit([_person(0.0)], 1000.0), 1000.0)

    def test_non_stop_tier_ignored(self):
        d = _Det("chair", 0.9)
        d.safety_tier = "slow"
        self.assertEqual(self.limit([d], 500.0), INF)

    def test_smallest_cap_wins(self):
        self.assertEqual(self.limit([_person(1.2), _dog(0.9)], 850.0), 850.0)


class TestReaderReportsTheCap(unittest.TestCase):

    def test_corridor_only(self):
        dist, _, limit = _poll(_make_reader(), _obstacle_frame(1000))
        self.assertAlmostEqual(dist, 1.0, delta=0.05)
        self.assertEqual(limit, INF)

    def test_netting_in_front_of_the_operator_keeps_the_relief(self):
        dist, _, limit = _poll(_make_reader(), _obstacle_frame(500), [_person(1.2)])
        self.assertAlmostEqual(dist, 0.5, delta=0.03)
        self.assertAlmostEqual(limit, 1.2, places=3)
        self.assertAlmostEqual(_manual_scale(dist, limit), min(new_curve(dist), old_law(1.2)), places=6)
        self.assertGreater(_manual_scale(dist, limit), 0.4)

    def test_dog_as_the_reading_gets_the_old_law(self):
        # Review repro: dog, corridor 0.76 m, z 0.81 m -> was 0.630 with the relief.
        dist, _, limit = _poll(_make_reader(), _obstacle_frame(760), [_dog(0.81)])
        self.assertAlmostEqual(dist, 0.76, delta=0.03)
        self.assertAlmostEqual(_manual_scale(dist, limit), old_law(dist), places=6)

    def test_unranged_person_gets_the_old_law(self):
        # Review repro: person with no range at 1.0 m -> was 0.750 with the relief.
        dist, _, limit = _poll(_make_reader(), _obstacle_frame(1000), [_person(0.0)])
        self.assertAlmostEqual(_manual_scale(dist, limit), old_law(dist), places=6)

    def test_person_as_the_nearest_thing_gets_the_old_law(self):
        dist, _, limit = _poll(_make_reader(), _all_zeros_frame(), [_person(0.9)])
        self.assertAlmostEqual(dist, 0.9, places=3)
        self.assertAlmostEqual(_manual_scale(dist, limit), old_law(0.9), places=6)

    def test_person_inside_the_stop_radius(self):
        dist, _, limit = _poll(_make_reader(), _obstacle_frame(1000), [_person(0.5)])
        self.assertEqual(dist, 0.0)
        self.assertEqual(_manual_scale(dist, limit), 0.0)

    def test_empty_corridor_clears_the_cap(self):
        reader = _make_reader()
        _poll(reader, _obstacle_frame(1000), [_person(1.2)])
        dist, _, limit = _poll(reader, _all_zeros_frame())
        self.assertEqual(dist, INF)
        self.assertEqual(limit, INF)


class TestThrottleLaw(unittest.TestCase):

    def test_cap_never_raises_the_scale(self):
        for d in (0.36, 0.5, 0.8, 1.2, 1.6, INF):
            for lim in (0.81, 1.0, 1.4, INF):
                self.assertLessEqual(_manual_scale(d, lim), _manual_scale(d, INF) + 1e-12)

    def test_person_at_081_is_the_old_law(self):
        self.assertAlmostEqual(_manual_scale(0.81, 0.81), old_law(0.81))

    def test_autonomous_modes_ignore_the_cap(self):
        c = ObstacleAvoidanceController(ObstacleAvoidanceConfig())
        self.assertAlmostEqual(c.compute_throttle_scale(0.9, 0.0, is_manual=False, manual_person_limit_m=0.9),
                               old_law(0.9))


class FakeMotor:
    def set_tracks(self, left_byte, right_byte):
        pass

    def stop(self):
        pass


class FakeRelay:
    def set_armed(self, armed):
        pass


class FakeShutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


ARMED = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0)
FULL_FWD = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0)


class TestControllerAppliesTheCap(unittest.TestCase):

    def _telem(self, dist, limit):
        ctrl = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
                          obstacle_avoidance=ObstacleAvoidanceController(ObstacleAvoidanceConfig()))
        ctrl.process(ARMED, now_epoch_s=0.5)
        ctrl.set_obstacle_data(dist, 0.0, person_limit_m=limit)
        _, _, telem = ctrl.process(FULL_FWD, now_epoch_s=1.0)
        return telem

    def test_person_at_081(self):
        t = self._telem(0.81, 0.81)
        self.assertAlmostEqual(t["obstacle_throttle_scale"], old_law(0.81))
        self.assertAlmostEqual(t["obstacle_person_limit_m"], 0.81)

    def test_netting_with_the_operator_behind_it(self):
        t = self._telem(0.5, 1.2)
        self.assertAlmostEqual(t["obstacle_throttle_scale"], 0.5)

    def test_no_person(self):
        t = self._telem(0.81, INF)
        self.assertAlmostEqual(t["obstacle_throttle_scale"], new_curve(0.81))
        self.assertIsNone(t["obstacle_person_limit_m"])


if __name__ == "__main__":
    unittest.main()
