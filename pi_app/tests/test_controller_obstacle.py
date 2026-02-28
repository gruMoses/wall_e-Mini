import unittest
from unittest.mock import patch

from pi_app.control.controller import Controller, RCInputs
from pi_app.control.safety import SafetyEvent, SafetyParams
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.follow_me import FollowMeController, PersonDetection
from config import ObstacleAvoidanceConfig, FollowMeConfig


class FakeMotor:
    def __init__(self):
        self.commands = []
        self.stops = 0
    def set_tracks(self, left_byte, right_byte):
        self.commands.append((left_byte, right_byte))
    def stop(self):
        self.stops += 1


class FakeRelay:
    def __init__(self):
        self.states = []
    def set_armed(self, armed):
        self.states.append(armed)


class FakeShutdown:
    def __init__(self):
        self.scheduled = []
    def schedule_shutdown(self, delay_seconds):
        self.scheduled.append(delay_seconds)


NEUTRAL_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=0.0)
ARMED_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                     last_update_epoch_s=0.0)
FWD_RC = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                   last_update_epoch_s=0.0)


class TestScaleTowardNeutral(unittest.TestCase):

    def test_scale_1_unchanged(self):
        self.assertEqual(Controller._scale_toward_neutral(200, 1.0), 200)

    def test_scale_0_returns_neutral(self):
        self.assertEqual(Controller._scale_toward_neutral(200, 0.0), 126)

    def test_scale_half(self):
        result = Controller._scale_toward_neutral(200, 0.5)
        expected = 126 + (200 - 126) * 0.5
        self.assertEqual(result, round(expected))

    def test_below_neutral(self):
        result = Controller._scale_toward_neutral(50, 0.5)
        expected = 126 + (50 - 126) * 0.5
        self.assertEqual(result, round(expected))

    def test_clamped_to_0_255(self):
        self.assertGreaterEqual(Controller._scale_toward_neutral(0, 2.0), 0)
        self.assertLessEqual(Controller._scale_toward_neutral(255, 2.0), 255)


class TestControllerObstacleAvoidance(unittest.TestCase):

    def _make_controller(self, oa_config=None, fm_config=None):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        oa = None
        if oa_config is not None:
            oa = ObstacleAvoidanceController(oa_config)
        fm = None
        if fm_config is not None:
            fm = FollowMeController(fm_config)
        ctrl = Controller(
            motor_driver=motor,
            arm_relay=relay,
            shutdown_scheduler=shutdown,
            obstacle_avoidance=oa,
            follow_me=fm,
        )
        return ctrl, motor

    def test_obstacle_scaling_reduces_speed(self):
        cfg = ObstacleAvoidanceConfig(stop_distance_m=0.4, slow_distance_m=1.5)
        ctrl, motor = self._make_controller(oa_config=cfg)

        # Arm
        ctrl.process(ARMED_RC, now_epoch_s=0.5)

        # Feed close obstacle
        ctrl.set_obstacle_data(distance_m=0.4, age_s=0.0)

        # Send forward command while armed
        cmd, events, telem = ctrl.process(FWD_RC, now_epoch_s=1.0)
        # At stop distance, scale = 0 -> neutral output
        self.assertEqual(cmd.left_byte, 126)
        self.assertEqual(cmd.right_byte, 126)

    def test_obstacle_far_away_no_scaling(self):
        cfg = ObstacleAvoidanceConfig(stop_distance_m=0.4, slow_distance_m=1.5)
        ctrl, motor = self._make_controller(oa_config=cfg)

        # Arm
        ctrl.process(ARMED_RC, now_epoch_s=0.5)

        # Feed far obstacle
        ctrl.set_obstacle_data(distance_m=5.0, age_s=0.0)

        cmd, events, telem = ctrl.process(FWD_RC, now_epoch_s=1.0)
        # Full speed through
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], 1.0)

    def test_no_obstacle_avoidance_no_effect(self):
        ctrl, motor = self._make_controller()  # no OA
        ctrl.process(ARMED_RC, now_epoch_s=0.5)
        cmd, events, telem = ctrl.process(FWD_RC, now_epoch_s=1.0)
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], 1.0)


class TestControllerFollowMe(unittest.TestCase):

    def _make_controller(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        fm_cfg = FollowMeConfig()
        fm = FollowMeController(fm_cfg)
        params = SafetyParams(debounce_seconds=0.0)
        ctrl = Controller(
            motor_driver=motor,
            arm_relay=relay,
            shutdown_scheduler=shutdown,
            safety_params=params,
            follow_me=fm,
        )
        return ctrl, motor

    def _enter_follow_me(self, ctrl, t):
        """Arm and flip ch4 high. Returns time after."""
        arm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                          last_update_epoch_s=0.0)
        ctrl.process(arm_rc, now_epoch_s=t)
        fm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1900, ch5_us=1000,
                         last_update_epoch_s=0.0)
        ctrl.process(fm_rc, now_epoch_s=t + 0.1)
        return t + 0.2

    def test_follow_me_mode_activates_on_ch4_high(self):
        ctrl, motor = self._make_controller()
        self._enter_follow_me(ctrl, 10.0)
        self.assertEqual(ctrl._mode, "FOLLOW_ME")

    def test_follow_me_mode_deactivates_on_ch4_low(self):
        ctrl, motor = self._make_controller()
        t = self._enter_follow_me(ctrl, 10.0)
        self.assertEqual(ctrl._mode, "FOLLOW_ME")

        # Ch4 low -> exit Follow Me
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_epoch_s=0.0)
        cmd, events, _ = ctrl.process(rc, now_epoch_s=t)
        self.assertEqual(ctrl._mode, "MANUAL")
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)

    def test_follow_me_mode_deactivates_on_disarm(self):
        ctrl, motor = self._make_controller()
        t = self._enter_follow_me(ctrl, 10.0)
        self.assertEqual(ctrl._mode, "FOLLOW_ME")

        # Disarm (ch3 low) while ch4 high -> exit Follow Me
        disarm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1900, ch5_us=1000,
                             last_update_epoch_s=0.0)
        cmd, events, _ = ctrl.process(disarm_rc, now_epoch_s=t)
        self.assertEqual(ctrl._mode, "MANUAL")
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)

    def test_follow_me_uses_detections_not_rc(self):
        ctrl, motor = self._make_controller()
        t = self._enter_follow_me(ctrl, 10.0)

        ctrl.set_person_detections([
            PersonDetection(x_m=0.0, z_m=3.0, confidence=0.9,
                            bbox=(0.45, 0.3, 0.55, 0.8))
        ])

        armed_fm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900,
                                ch4_us=1900, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, _, telem = ctrl.process(armed_fm_rc, now_epoch_s=t)
        self.assertEqual(telem["mode"], "FOLLOW_ME")
        self.assertGreater(cmd.left_byte, 126)
        self.assertGreater(cmd.right_byte, 126)

    def test_follow_me_no_person_returns_neutral(self):
        ctrl, motor = self._make_controller()
        t = self._enter_follow_me(ctrl, 10.0)

        ctrl.set_person_detections([])
        armed_fm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900,
                                ch4_us=1900, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, _, _ = ctrl.process(armed_fm_rc, now_epoch_s=t)
        self.assertEqual(cmd.left_byte, 126)
        self.assertEqual(cmd.right_byte, 126)


if __name__ == "__main__":
    unittest.main()
