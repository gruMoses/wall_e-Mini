"""A person/animal range keeps the old MANUAL throttle law (2026-09-27 review).

The relaxed MANUAL curve (0.5 at 0.50 m) is for things like netting over
the lens. A stop-tier detection nearer than the corridor reading pulls the
obstacle distance down to its range; with the relaxed curve, full stick
would reach the 0.8 m person stop at about 1.05 m/s instead of 0.60 m/s.
The depth poll now records that a detection set the distance, and the
controller keeps the old law for it.
"""

import time
import unittest

import numpy as np

from config import ObstacleAvoidanceConfig
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.hardware.oak_depth import _AllDetsState
from pi_app.tests.test_depth_corridor_stale import (
    _FakeFrame, _FakeQueue, _all_zeros_frame, _make_reader, _obstacle_frame,
)

OLD_LAW_081 = (0.81 - 0.4) / (1.5 - 0.4)          # 0.373
NEW_CURVE_081 = 0.5 + 0.5 * (0.81 - 0.50) / 1.0   # 0.655


class _Person:
    safety_tier = "stop"
    label_name = "person"

    def __init__(self, z_m):
        self.z_m = z_m


def _poll(reader, frame, persons=()):
    with reader._lock:
        reader._all_dets_state = _AllDetsState(detections=list(persons), timestamp=time.monotonic())
    reader._poll_depth(_FakeQueue(frame=_FakeFrame(frame)), _FakeQueue(frame=None), np)
    return reader.get_min_distance_detail()


class TestReaderRecordsTheSource(unittest.TestCase):

    def test_corridor_obstacle_is_not_from_detection(self):
        dist, age, from_det = _poll(_make_reader(), _obstacle_frame(1000))
        self.assertAlmostEqual(dist, 1.0, delta=0.05)
        self.assertFalse(from_det)

    def test_nearer_person_sets_the_distance_and_the_flag(self):
        dist, age, from_det = _poll(_make_reader(), _obstacle_frame(1400), persons=[_Person(1.2)])
        self.assertAlmostEqual(dist, 1.2, places=3)
        self.assertTrue(from_det)

    def test_farther_person_leaves_the_corridor_reading(self):
        dist, age, from_det = _poll(_make_reader(), _obstacle_frame(1000), persons=[_Person(2.5)])
        self.assertAlmostEqual(dist, 1.0, delta=0.05)
        self.assertFalse(from_det)

    def test_person_inside_the_stop_radius(self):
        dist, age, from_det = _poll(_make_reader(), _obstacle_frame(1000), persons=[_Person(0.5)])
        self.assertEqual(dist, 0.0)
        self.assertTrue(from_det)

    def test_empty_corridor_clears_the_flag(self):
        reader = _make_reader()
        _poll(reader, _obstacle_frame(1400), persons=[_Person(1.2)])
        dist, age, from_det = _poll(reader, _all_zeros_frame())
        self.assertEqual(dist, float("inf"))
        self.assertFalse(from_det)


class TestThrottleLawForAPerson(unittest.TestCase):

    def test_manual_curve_off_uses_the_old_law(self):
        c = ObstacleAvoidanceController(ObstacleAvoidanceConfig())
        self.assertAlmostEqual(c.compute_throttle_scale(0.81, 0.0, is_manual=True, use_manual_curve=False),
                               OLD_LAW_081)
        self.assertAlmostEqual(c.compute_throttle_scale(0.81, 0.0, is_manual=True), NEW_CURVE_081)


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


class TestControllerKeepsTheOldLawForAPerson(unittest.TestCase):

    def _scale(self, from_detection):
        ctrl = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
                          obstacle_avoidance=ObstacleAvoidanceController(ObstacleAvoidanceConfig()))
        ctrl.process(ARMED, now_epoch_s=0.5)
        ctrl.set_obstacle_data(0.81, 0.0, from_detection=from_detection)
        _, _, telem = ctrl.process(FULL_FWD, now_epoch_s=1.0)
        return telem

    def test_person_range_in_manual(self):
        telem = self._scale(True)
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], OLD_LAW_081)
        self.assertTrue(telem["obstacle_from_detection"])

    def test_corridor_range_in_manual(self):
        telem = self._scale(False)
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], NEW_CURVE_081)
        self.assertFalse(telem["obstacle_from_detection"])


if __name__ == "__main__":
    unittest.main()
