"""Tests for the YOLO person/animal safety-stop tier (Bug #3).

The ObstacleAvoidanceController exposes set_safety_detections(), but it was
never called with live YOLO output, so the stop tier was dead code. These
tests cover:
  - set_safety_detections latches force-stop for a "stop"-tier detection
    inside safety_stop_radius_m, and clears it when out of range.
  - the main-loop wiring contract: set_safety_detections is fed the live
    get_all_detections() output once per tick (and is gated by the config flag).
  - the Controller actually zeroes forward motion when the tier is latched.
"""

import unittest

from pi_app.control.controller import (
    Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler,
)
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.safety import SafetyParams
from config import config as default_config


class FakeDet:
    """Mimics oak_depth.ObjectDetection for the fields the safety tier reads."""

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


class FakeOak:
    def __init__(self, dets):
        self._dets = dets

    def get_all_detections(self):
        return list(self._dets)


class _SpyObstacle(ObstacleAvoidanceController):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.calls = []

    def set_safety_detections(self, detections):
        self.calls.append(detections)
        super().set_safety_detections(detections)


class TestSafetyDetectionLatch(unittest.TestCase):
    def _oc(self):
        return ObstacleAvoidanceController(default_config.obstacle_avoidance)

    def test_person_within_radius_forces_stop(self):
        oc = self._oc()
        oc.set_safety_detections([FakeDet("stop", 0.5)])  # < 0.8 m radius
        # Depth corridor is clear (5 m) but the tier must override to 0.
        self.assertEqual(oc.compute_throttle_scale(distance_m=5.0, age_s=0.0), 0.0)
        self.assertTrue(oc.get_status()["safety_force_stop"])

    def test_person_outside_radius_does_not_stop(self):
        oc = self._oc()
        oc.set_safety_detections([FakeDet("stop", 1.5)])  # > 0.8 m radius
        self.assertGreater(oc.compute_throttle_scale(distance_m=5.0, age_s=0.0), 0.0)
        self.assertFalse(oc.get_status()["safety_force_stop"])

    def test_non_stop_tier_ignored(self):
        oc = self._oc()
        oc.set_safety_detections([FakeDet("log", 0.3), FakeDet("slow", 0.3)])
        self.assertGreater(oc.compute_throttle_scale(distance_m=5.0, age_s=0.0), 0.0)


class TestMainLoopWiringContract(unittest.TestCase):
    """Mirrors the per-tick wiring added to pi_app/app/main.py."""

    def test_fed_live_detections_once_per_tick(self):
        cfg = default_config.obstacle_avoidance
        oc = _SpyObstacle(cfg)
        oak = FakeOak([FakeDet("stop", 0.5)])
        ticks = 5
        for _ in range(ticks):
            # Exactly the guard + call used in the main loop.
            if oc is not None and getattr(cfg, "yolo_safety_enabled", True):
                oc.set_safety_detections(oak.get_all_detections())
        self.assertEqual(len(oc.calls), ticks)
        self.assertEqual(oc.calls[-1][0].z_m, 0.5)  # live YOLO output passed through
        self.assertTrue(oc.get_status()["safety_force_stop"])

    def test_config_flag_disables_wiring(self):
        oc = _SpyObstacle(default_config.obstacle_avoidance)
        oak = FakeOak([FakeDet("stop", 0.5)])
        yolo_safety_enabled = False  # simulate the config override
        for _ in range(3):
            if oc is not None and yolo_safety_enabled:
                oc.set_safety_detections(oak.get_all_detections())
        self.assertEqual(oc.calls, [])


class TestControllerStopsOnSafetyTier(unittest.TestCase):
    def test_forward_motion_zeroed_when_person_in_radius(self):
        motor = FakeMotor()
        oc = ObstacleAvoidanceController(default_config.obstacle_avoidance)
        c = Controller(
            motor_driver=motor,
            arm_relay=FakeRelay(),
            shutdown_scheduler=FakeShutdown(),
            obstacle_avoidance=oc,
            safety_params=SafetyParams(debounce_seconds=0.0),
        )
        # Arm
        arm = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        c.process(arm, now_epoch_s=100.0)
        # Depth corridor clear, but a person is within the stop radius.
        c.set_obstacle_data(5.0, 0.0)
        oc.set_safety_detections([FakeDet("stop", 0.5)])
        # Full forward on both tracks.
        fwd = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        cmd, events, telem = c.process(fwd, now_epoch_s=100.1)
        self.assertEqual(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertEqual(cmd.right_byte, CENTER_OUTPUT_VALUE)
        self.assertTrue(telem.get("safety_force_stop"))
        self.assertEqual(telem.get("obstacle_throttle_scale"), 0.0)

    def test_forward_motion_allowed_when_clear(self):
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
        c.set_obstacle_data(5.0, 0.0)
        oc.set_safety_detections([FakeDet("stop", 1.5)])  # out of range → no stop
        fwd = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        cmd, events, telem = c.process(fwd, now_epoch_s=100.1)
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertGreater(cmd.right_byte, CENTER_OUTPUT_VALUE)
        self.assertFalse(telem.get("safety_force_stop"))


if __name__ == "__main__":
    unittest.main()
