"""eRPM -> ground speed after the 2026-11-10 scale correction.

docs/wheel_speed_scale_2026-11-10.md: the nominal kinematics (14 poles,
34.2857:1, r = 0.18415 m) read 1.23x the GPS ground speed on straight steady
drives at 2-4 A of motor current. VescConfig.wheel_ground_scale (0.815)
carries the correction; the geometry constants stay as labelled.
"""

import dataclasses
import math
import unittest

from config import FollowMeConfig, VescConfig
from pi_app.control.follow_me import FollowMeController, PIDController, SpeedLayer
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


OLD_LOOP_SCALE = 0.012555        # speed_loop_mps_per_byte before 2026-11-10
OLD_MAX_CORRECTION_MPS = 0.20
OLD_INTEGRAL_LIMIT = 1.0


class TestIdenticalMotorBytes(unittest.TestCase):
    """The whole 2026-11-10 correction (ground scale + loop scale + the two
    scaled clamps) gives the SAME follow-me forward bytes as before, also when
    the loop saturates at the correction ceiling and at the integral clamp.

    Both worlds see the same eRPM; each converts it with its own scale.
    """

    def _worlds(self):
        new_cfg = FollowMeConfig()
        old_cfg = dataclasses.replace(
            new_cfg, speed_loop_mps_per_byte=OLD_LOOP_SCALE,
            speed_pid_max_correction_mps=OLD_MAX_CORRECTION_MPS,
            speed_integral_limit=OLD_INTEGRAL_LIMIT)
        v = VescConfig()
        new_conv = dict(motor_poles=v.motor_poles, drive_gear_ratio=v.drive_gear_ratio,
                        wheel_radius_m=v.wheel_radius_m, ground_speed_scale=v.wheel_ground_scale)
        old_conv = dict(new_conv, ground_speed_scale=1.0)
        return ((FollowMeController(old_cfg), old_conv), (FollowMeController(new_cfg), new_conv))

    def _run(self, steps):
        """steps: list of (depth_m, wheel_erpm, dt). Returns per-tick bytes."""
        (fm_old, conv_old), (fm_new, conv_new) = self._worlds()
        out_old, out_new = [], []
        em_old = em_new = None
        for depth, erpm, dt in steps:
            b_old = fm_old._speed.compute(depth, actual_speed_mps=erpm_to_wheel_mps(erpm, **conv_old),
                                          dt=dt, emitted_forward_byte=em_old)
            b_new = fm_new._speed.compute(depth, actual_speed_mps=erpm_to_wheel_mps(erpm, **conv_new),
                                          dt=dt, emitted_forward_byte=em_new)
            em_old, em_new = b_old, b_new
            out_old.append(b_old)
            out_new.append(b_new)
        return fm_old, fm_new, out_old, out_new

    def _assert_same(self, out_old, out_new):
        for i, (a, b) in enumerate(zip(out_old, out_new)):
            self.assertAlmostEqual(a, b, delta=1e-9, msg=f"tick {i}")
            self.assertEqual(round(a), round(b), msg=f"tick {i}")

    @staticmethod
    def _erpm(bytes_):
        return bytes_ / 128.0 * VescConfig().max_erpm

    def test_clamp_byte_values_match_old(self):
        new = FollowMeConfig()
        self.assertAlmostEqual(new.speed_pid_max_correction_mps / new.speed_loop_mps_per_byte,
                               OLD_MAX_CORRECTION_MPS / OLD_LOOP_SCALE, delta=0.01)   # 15.9
        self.assertAlmostEqual(new.speed_ki * new.speed_integral_limit / new.speed_loop_mps_per_byte,
                               new.speed_ki * OLD_INTEGRAL_LIMIT / OLD_LOOP_SCALE, delta=0.01)  # 11.9

    def test_step_hits_correction_ceiling(self):
        # Open loop 60 bytes (depth 2.02 m); wheels stuck at 30 bytes' eRPM:
        # P alone asks 0.6 * 30 = 18 bytes > the 15.9-byte ceiling.
        depth = 1.2 + 60.0 / (128.0 / 1.75)
        steps = [(1.3, 0.0, 0.05)] * 3 + [(depth, self._erpm(30.0), 0.05)] * 40
        fm_old, fm_new, a, b = self._run(steps)
        self._assert_same(a, b)
        for fm in (fm_old, fm_new):
            self.assertAlmostEqual(abs(fm._speed.last_corr_byte), 15.93, delta=0.02)

    def test_sustained_error_hits_integral_clamp(self):
        # 5-byte shortfall for 20 s: P = 3 bytes, I saturates at 11.9 bytes,
        # total 14.9 < 15.9, so the integral clamp (not the ceiling) binds.
        depth = 1.2 + 60.0 / (128.0 / 1.75)
        steps = [(depth, self._erpm(55.0), 0.1)] * 200
        fm_old, fm_new, a, b = self._run(steps)
        self._assert_same(a, b)
        for fm, lim in ((fm_old, OLD_INTEGRAL_LIMIT), (fm_new, FollowMeConfig().speed_integral_limit)):
            pid = fm._speed._velocity_pid
            self.assertAlmostEqual(pid._integral, lim, places=9)
            self.assertLess(abs(fm._speed.last_corr_byte), 15.9)

    def test_overspeed_and_mixed_sequence(self):
        depth_a = 1.2 + 40.0 / (128.0 / 1.75)
        depth_b = 1.2 + 90.0 / (128.0 / 1.75)
        steps = ([(depth_a, self._erpm(70.0), 0.05)] * 60       # wheels too fast: negative ceiling
                 + [(depth_b, self._erpm(50.0), 0.1)] * 120     # big shortfall, both clamps
                 + [(depth_b, self._erpm(88.0), 0.05)] * 40
                 + [(1.3, 0.0, 0.05)] * 5)                      # dead zone: reset
        _, _, a, b = self._run(steps)
        self._assert_same(a, b)


if __name__ == "__main__":
    unittest.main()
