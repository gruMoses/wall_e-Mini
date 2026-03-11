import math
import unittest

from pi_app.hardware.oak_imu import OakImuReader


class TestOakImuYawRateMode(unittest.TestCase):
    def test_direct_gyro_mode_returns_gz(self):
        gx = math.radians(15.0)
        gy = math.radians(-8.0)
        gz = math.radians(42.0)
        # Deliberately skewed accel direction should not matter in direct mode.
        sx, sy, sz = 0.3, 0.4, 0.2
        got = OakImuReader._compute_yaw_rate_rads(
            gx, gy, gz, sx, sy, sz, use_gravity_projected=False
        )
        self.assertAlmostEqual(got, gz, places=8)

    def test_projected_mode_uses_projection_when_accel_valid(self):
        gx = math.radians(10.0)
        gy = math.radians(20.0)
        gz = math.radians(30.0)
        sx, sy, sz = 0.0, 0.0, 1.0
        got = OakImuReader._compute_yaw_rate_rads(
            gx, gy, gz, sx, sy, sz, use_gravity_projected=True
        )
        self.assertAlmostEqual(got, gz, places=8)

    def test_projected_mode_falls_back_to_gz_when_accel_too_small(self):
        gx = math.radians(10.0)
        gy = math.radians(20.0)
        gz = math.radians(30.0)
        sx, sy, sz = 0.01, 0.01, 0.01  # norm^2 < 0.25
        got = OakImuReader._compute_yaw_rate_rads(
            gx, gy, gz, sx, sy, sz, use_gravity_projected=True
        )
        self.assertAlmostEqual(got, gz, places=8)


if __name__ == "__main__":
    unittest.main()
