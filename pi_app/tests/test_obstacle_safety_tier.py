"""Tests for the YOLO person/animal safety-stop tier (Bug #3).

The stop tier is wired in ONE canonical place: the depth poll. A "stop"-tier
detection (person/animal) within safety_stop_radius_m forces the reported
min_distance to 0, which flows through get_min_distance() into
ObstacleAvoidanceController.compute_throttle_scale() and yields throttle
scale 0.0 (full stop).

These cover:
  - OakDepthReader._apply_safety_tier_override (the canonical override logic),
  - compute_throttle_scale turning a 0 m reading into a 0.0 scale,
  - the Controller actually zeroing forward motion end-to-end for a person
    within the radius.
"""

import unittest

from pi_app.control.controller import (
    Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler,
)
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.safety import SafetyParams
from pi_app.hardware.oak_depth import OakDepthReader
from config import config as default_config

INF = float("inf")


class FakeDet:
    """Mimics oak_depth.ObjectDetection for the fields the stop tier reads."""

    def __init__(self, safety_tier, z_m):
        self.safety_tier = safety_tier
        self.z_m = z_m
        self.label_name = "person"


class FakeMotor(MotorDriver):
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        self.commands.append((left_byte, right_byte))

    def stop(self) -> None:
        self.stops += 1


class FakeRelay(ArmRelay):
    def set_armed(self, armed: bool) -> None:
        pass


class FakeShutdown(ShutdownScheduler):
    def schedule_shutdown(self, delay_seconds: float) -> None:
        pass


class TestSafetyTierOverride(unittest.TestCase):
    """The canonical override that runs inside the depth poll."""

    RADIUS = 0.8

    def test_person_within_radius_forces_hard_stop(self):
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [FakeDet("stop", 0.5)], corridor_p5_mm=3000.0, safety_stop_radius_m=self.RADIUS)
        self.assertEqual(eff_mm, 0.0)                  # forces stop
        self.assertIsNotNone(stop_det)

    def test_person_within_radius_overrides_clear_corridor(self):
        # Even when the depth corridor reads far/clear, a close person stops.
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [FakeDet("stop", 0.79)], corridor_p5_mm=INF, safety_stop_radius_m=self.RADIUS)
        self.assertEqual(eff_mm, 0.0)
        self.assertIsNotNone(stop_det)

    def test_person_outside_radius_does_not_force_stop(self):
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [FakeDet("stop", 1.5)], corridor_p5_mm=3000.0, safety_stop_radius_m=self.RADIUS)
        self.assertEqual(eff_mm, 1500.0)               # pulled down to detection, not a hard stop
        self.assertIsNone(stop_det)

    def test_distant_person_does_not_raise_corridor_distance(self):
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [FakeDet("stop", 5.0)], corridor_p5_mm=2000.0, safety_stop_radius_m=self.RADIUS)
        self.assertEqual(eff_mm, 2000.0)               # corridor reading kept (detection farther)
        self.assertIsNone(stop_det)

    def test_non_stop_tier_ignored(self):
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [FakeDet("log", 0.3), FakeDet("slow", 0.3)], corridor_p5_mm=3000.0,
            safety_stop_radius_m=self.RADIUS)
        self.assertEqual(eff_mm, 3000.0)
        self.assertIsNone(stop_det)

    def test_no_detections_returns_corridor(self):
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [], corridor_p5_mm=INF, safety_stop_radius_m=self.RADIUS)
        self.assertEqual(eff_mm, INF)
        self.assertIsNone(stop_det)


class TestThrottleScaleFromDistance(unittest.TestCase):
    """A 0 m reading (what the override produces) must yield scale 0.0."""

    def _oc(self):
        return ObstacleAvoidanceController(default_config.obstacle_avoidance)

    def test_zero_distance_stops(self):
        self.assertEqual(self._oc().compute_throttle_scale(distance_m=0.0, age_s=0.0), 0.0)

    def test_clear_distance_full_throttle(self):
        self.assertEqual(self._oc().compute_throttle_scale(distance_m=5.0, age_s=0.0), 1.0)


class TestControllerStopsForPersonInRadius(unittest.TestCase):
    """End-to-end: override output (0 m) → controller zeroes forward motion."""

    def _armed_controller_with_obstacle(self):
        motor = FakeMotor()
        oc = ObstacleAvoidanceController(default_config.obstacle_avoidance)
        c = Controller(
            motor_driver=motor,
            arm_relay=FakeRelay(),
            shutdown_scheduler=FakeShutdown(),
            obstacle_avoidance=oc,
            safety_params=SafetyParams(debounce_seconds=0.0),
        )
        arm = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        c.process(arm, now_epoch_s=100.0)
        return c, motor

    def test_person_within_radius_zeroes_forward_motion(self):
        c, motor = self._armed_controller_with_obstacle()
        # The depth poll, on a person within radius, reports min_distance ~0.
        eff_mm, _ = OakDepthReader._apply_safety_tier_override(
            [FakeDet("stop", 0.5)], corridor_p5_mm=INF, safety_stop_radius_m=0.8)
        c.set_obstacle_data(eff_mm / 1000.0, 0.0)  # 0.0 m, fresh
        fwd = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        cmd, events, telem = c.process(fwd, now_epoch_s=100.1)
        self.assertEqual(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertEqual(cmd.right_byte, CENTER_OUTPUT_VALUE)
        self.assertEqual(telem.get("obstacle_throttle_scale"), 0.0)

    def test_clear_path_allows_forward_motion(self):
        c, motor = self._armed_controller_with_obstacle()
        c.set_obstacle_data(5.0, 0.0)  # clear, fresh
        fwd = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        cmd, events, telem = c.process(fwd, now_epoch_s=100.1)
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertGreater(cmd.right_byte, CENTER_OUTPUT_VALUE)


if __name__ == "__main__":
    unittest.main()
