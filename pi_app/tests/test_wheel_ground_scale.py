"""eRPM -> ground speed after the 2026-11-10 scale correction.

docs/wheel_speed_scale_2026-11-10.md: the nominal kinematics (14 poles,
34.2857:1, r = 0.18415 m) read 1.23x the GPS ground speed on straight steady
drives at 2-4 A of motor current. VescConfig.wheel_ground_scale (0.815)
carries the correction; the geometry constants stay as labelled.
"""

import math
import unittest

from config import FollowMeConfig, VescConfig
from pi_app.control.follow_me import PIDController, SpeedLayer
from pi_app.control.rpm_plausibility import erpm_to_wheel_mps, wheel_mps_per_byte


def _kw(v: VescConfig) -> dict:
    return dict(motor_poles=v.motor_poles, drive_gear_ratio=v.drive_gear_ratio,
                wheel_radius_m=v.wheel_radius_m, ground_speed_scale=v.wheel_ground_scale)


class TestWorkedExample(unittest.TestCase):
    def test_config_constants(self):
        v = VescConfig()
        self.assertEqual(v.motor_poles, 14)
        self.assertAlmostEqual(v.drive_gear_ratio, 240.0 / 7.0, places=6)
        self.assertAlmostEqual(v.wheel_radius_m, 0.18415, places=6)
        self.assertAlmostEqual(v.wheel_ground_scale, 0.815, places=6)

    def test_10000_erpm_by_hand(self):
        # 10000 eRPM / 7 pole pairs = 1428.6 motor rpm
        # / 34.2857 = 41.667 wheel rpm = 0.69444 rev/s
        # * 2 pi * 0.18415 m = 0.80349 m/s nominal
        # * 0.815 = 0.65484 m/s on the ground
        v = VescConfig()
        by_hand = 10000 / 7 / (240 / 7) / 60 * 2 * math.pi * 0.18415 * 0.815
        self.assertAlmostEqual(by_hand, 0.65484, places=4)
        self.assertAlmostEqual(erpm_to_wheel_mps(10000, **_kw(v)), by_hand, places=6)

    def test_scale_one_is_nominal_and_sign_kept(self):
        v = VescConfig()
        nominal = erpm_to_wheel_mps(10000, motor_poles=14, drive_gear_ratio=v.drive_gear_ratio,
                                    wheel_radius_m=0.18415)
        self.assertAlmostEqual(nominal, 0.80349, places=4)
        self.assertAlmostEqual(erpm_to_wheel_mps(-10000, **_kw(v)), -0.65484, places=4)

    def test_byte_254_ground_speed(self):
        v = VescConfig()
        per_byte = wheel_mps_per_byte(max_erpm=v.max_erpm, **_kw(v))
        self.assertAlmostEqual(per_byte, 0.010232, places=6)
        self.assertAlmostEqual(128 * per_byte, 1.3097, places=3)

    def test_scale_is_inside_the_measured_band(self):
        # Wheel/GPS ratio 1.18-1.29 across drives -> scale 0.775-0.85.
        self.assertGreater(VescConfig().wheel_ground_scale, 0.775)
        self.assertLess(VescConfig().wheel_ground_scale, 0.85)


class TestLoopIsRatiometric(unittest.TestCase):
    """The scale change must not move follow-me bytes while the loop is
    inside its clamps: target and feedback both carry the same scale."""

    def _layer(self, scale: float) -> SpeedLayer:
        cfg = FollowMeConfig()
        pid = PIDController(kp=cfg.speed_kp, ki=cfg.speed_ki, kd=0.0,
                            integral_limit=100.0, output_limit=100.0)
        return SpeedLayer(target_dist_m=cfg.follow_distance_m, dead_zone_m=cfg.speed_dead_zone_m,
                          speed_gain=cfg.max_follow_speed_byte / cfg.max_speed_error_m,
                          min_dist_m=cfg.min_distance_m, max_speed_byte=cfg.max_follow_speed_byte,
                          velocity_pid=pid, speed_scale_mps_per_byte=scale)

    def test_same_bytes_for_old_and_new_scale(self):
        v = VescConfig()
        old_k = wheel_mps_per_byte(max_erpm=v.max_erpm, motor_poles=v.motor_poles,
                                   drive_gear_ratio=v.drive_gear_ratio, wheel_radius_m=v.wheel_radius_m)
        new_k = wheel_mps_per_byte(max_erpm=v.max_erpm, **_kw(v))
        old, new = self._layer(old_k), self._layer(new_k)
        # The wheels run 10 bytes short of the open-loop request (same eRPM
        # in both worlds); each world converts that eRPM with its own scale.
        for depth in (2.0, 2.4, 2.4, 2.8, 3.0):
            open_loop = min(128.0, (depth - 1.2) * 128.0 / 1.75)
            wheel_bytes = max(open_loop - 10.0, 0.0)
            b_old = old.compute(depth, actual_speed_mps=wheel_bytes * old_k, dt=0.05)
            b_new = new.compute(depth, actual_speed_mps=wheel_bytes * new_k, dt=0.05)
            self.assertAlmostEqual(b_old, b_new, places=6)


if __name__ == "__main__":
    unittest.main()
