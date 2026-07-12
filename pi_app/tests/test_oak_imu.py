import math
import unittest
from dataclasses import dataclass
from typing import Optional

from pi_app.hardware.oak_imu import OakImuReader, G_MSS


@dataclass
class FakeImuState:
    ax_mss: float = 0.0
    ay_mss: float = -G_MSS  # gravity along -Y (OAK camera frame flat)
    az_mss: float = 0.0
    gx_rads: float = 0.0
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    timestamp: float = 0.0
    device_timestamp_s: float = 0.0


class FakeOak:
    """Minimal OakDepthReader stub for OakImuReader unit tests."""

    def __init__(self) -> None:
        self.state = FakeImuState()
        self.age_s = 0.01
        self.health = {
            "connected": True,
            "reconnect_count": 0,
            "last_disconnect_ts": 0.0,
            "pipeline_running": True,
        }

    def get_imu_data(self):
        return self.state, self.age_s

    def get_health(self):
        return dict(self.health)

    def push(
        self,
        *,
        device_ts: float,
        host_ts: Optional[float] = None,
        gy_dps: float = 0.0,
        gx_dps: float = 0.0,
        gz_dps: float = 0.0,
        age_s: float = 0.01,
    ) -> None:
        self.age_s = age_s
        self.state = FakeImuState(
            ay_mss=-G_MSS,
            gx_rads=math.radians(gx_dps),
            gy_rads=math.radians(gy_dps),
            gz_rads=math.radians(gz_dps),
            timestamp=float(host_ts if host_ts is not None else device_ts),
            device_timestamp_s=float(device_ts),
        )


class TestOakImuYawRateMode(unittest.TestCase):
    def test_direct_gyro_mode_returns_gz(self):
        gx = math.radians(15.0)
        gy = math.radians(-8.0)
        gz = math.radians(42.0)
        sx, sy, sz = 0.3, 0.4, 0.2
        got = OakImuReader._compute_yaw_rate_rads(
            gx, gy, gz, sx, sy, sz,
            source="gyro_z", auto_axis="gyro_z", use_gravity_projected=False,
        )
        self.assertAlmostEqual(got, gz, places=8)

    def test_projected_mode_uses_projection_when_accel_valid(self):
        gx = math.radians(10.0)
        gy = math.radians(20.0)
        gz = math.radians(30.0)
        sx, sy, sz = 0.0, 0.0, 1.0
        got = OakImuReader._compute_yaw_rate_rads(
            gx, gy, gz, sx, sy, sz,
            source="gravity_projected", auto_axis="gyro_y", use_gravity_projected=True,
        )
        self.assertAlmostEqual(got, gz, places=8)

    def test_projected_mode_falls_back_to_gy_when_accel_too_small(self):
        gx = math.radians(10.0)
        gy = math.radians(20.0)
        gz = math.radians(30.0)
        sx, sy, sz = 0.01, 0.01, 0.01
        got = OakImuReader._compute_yaw_rate_rads(
            gx, gy, gz, sx, sy, sz,
            source="gravity_projected", auto_axis="gyro_y", use_gravity_projected=True,
        )
        self.assertAlmostEqual(got, gy, places=8)


class TestOakImuIntegrationHardening(unittest.TestCase):
    def _reader(self, **kwargs) -> tuple[FakeOak, OakImuReader]:
        oak = FakeOak()
        defaults = dict(
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=False,
            bias_adapt_enabled=False,
        )
        defaults.update(kwargs)
        return oak, OakImuReader(oak, **defaults)

    def test_fresh_integration_advances_heading(self):
        oak, imu = self._reader()
        # Constant +90 dps about Y for 0.5 s total → ~45° free yaw before sign.
        # heading_deg = (-yaw_deg) mod 360, so +gy yields negative heading change.
        oak.push(device_ts=1.000, gy_dps=90.0)
        d0 = imu.read()
        self.assertEqual(d0["integrate_status"], "init")
        h0 = d0["heading_deg"]

        oak.push(device_ts=1.050, gy_dps=90.0)
        d1 = imu.read()
        self.assertEqual(d1["integrate_status"], "fresh")
        self.assertEqual(imu.get_health()["count_integrated"], 1)
        # 90 dps * 0.05 s = 4.5° free yaw; heading moves opposite sign.
        self.assertAlmostEqual(d1["heading_deg"], (h0 - 4.5) % 360.0, places=4)

        oak.push(device_ts=1.100, gy_dps=90.0)
        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "fresh")
        self.assertAlmostEqual(d2["heading_deg"], (h0 - 9.0) % 360.0, places=4)

    def test_duplicate_device_timestamp_does_not_double_integrate(self):
        oak, imu = self._reader()
        oak.push(device_ts=2.000, gy_dps=50.0)
        imu.read()  # init
        oak.push(device_ts=2.020, gy_dps=50.0)
        d1 = imu.read()
        h1 = d1["heading_deg"]
        self.assertEqual(d1["integrate_status"], "fresh")

        # Same device timestamp, even if host time advances / gyro non-zero.
        oak.push(device_ts=2.020, host_ts=99.0, gy_dps=50.0)
        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "duplicate")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)
        self.assertEqual(d2["gz_dps"], 0.0)  # frozen rate for D-term
        self.assertEqual(imu.get_health()["count_duplicate"], 1)

        # Controller tick double-read of same packet.
        d3 = imu.read()
        self.assertEqual(d3["integrate_status"], "duplicate")
        self.assertAlmostEqual(d3["heading_deg"], h1, places=8)
        self.assertEqual(imu.get_health()["count_duplicate"], 2)
        self.assertEqual(imu.get_health()["count_integrated"], 1)

    def test_stale_sample_freezes_heading(self):
        oak, imu = self._reader()
        oak.push(device_ts=3.000, gy_dps=40.0, age_s=0.01)
        imu.read()
        oak.push(device_ts=3.020, gy_dps=40.0, age_s=0.01)
        d1 = imu.read()
        h1 = d1["heading_deg"]

        oak.push(device_ts=3.040, gy_dps=40.0, age_s=1.5)  # stale
        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "stale")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)
        self.assertEqual(imu.get_health()["count_stale"], 1)
        self.assertEqual(imu.get_health()["count_integrated"], 1)

    def test_timestamp_regression_reconnect_freezes_no_jump(self):
        oak, imu = self._reader()
        oak.push(device_ts=10.000, gy_dps=30.0)
        imu.read()
        oak.push(device_ts=10.050, gy_dps=30.0)
        d1 = imu.read()
        h1 = d1["heading_deg"]
        self.assertGreater(imu.get_health()["count_integrated"], 0)

        # USB reconnect: device clock restarts near zero while gyro non-zero.
        oak.push(device_ts=0.010, gy_dps=200.0)
        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "regressed")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)
        self.assertEqual(imu.get_health()["count_regressed"], 1)
        self.assertGreaterEqual(imu.get_health()["count_restart"], 1)

        # Next sample after reseed integrates from the new clock, no wrap jump.
        oak.push(device_ts=0.030, gy_dps=0.0)
        d3 = imu.read()
        self.assertEqual(d3["integrate_status"], "fresh")
        self.assertAlmostEqual(d3["heading_deg"], h1, places=5)

    def test_gap_freeze_avoids_huge_dt_jump(self):
        oak, imu = self._reader(max_integrate_dt_s=0.15)
        oak.push(device_ts=5.000, gy_dps=10.0)
        imu.read()
        oak.push(device_ts=5.050, gy_dps=10.0)
        h1 = imu.read()["heading_deg"]

        # 1 second gap at 100 dps would be a 100° phantom jump if integrated.
        oak.push(device_ts=6.050, gy_dps=100.0)
        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "gap_freeze")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)
        self.assertEqual(imu.get_health()["count_gap_freeze"], 1)

    def test_invalid_device_ts_uses_host_identity_not_loop_dt(self):
        oak, imu = self._reader()
        # No device timestamps — only host sample identity.
        oak.push(device_ts=0.0, host_ts=100.0, gy_dps=90.0)
        d0 = imu.read()
        self.assertEqual(d0["integrate_status"], "init")
        h0 = d0["heading_deg"]

        # Duplicate host sample: many controller polls.
        for _ in range(5):
            oak.push(device_ts=0.0, host_ts=100.0, gy_dps=90.0)
            d = imu.read()
            self.assertIn(d["integrate_status"], ("restart", "duplicate"))
            self.assertAlmostEqual(d["heading_deg"], h0, places=8)

        # New host sample advances with 50 ms dt.
        oak.push(device_ts=0.0, host_ts=100.050, gy_dps=90.0)
        d1 = imu.read()
        # First new sample after init+restart path may be restart then fresh:
        # after init seeded host 100.0, first push at 100.0 may be duplicate of init seed.
        # At 100.050 we expect fresh.
        self.assertEqual(d1["integrate_status"], "fresh")
        self.assertAlmostEqual(d1["heading_deg"], (h0 - 4.5) % 360.0, places=4)

    def test_telemetry_health_fields_present(self):
        oak, imu = self._reader(yaw_rate_source="gyro_x", yaw_rate_scale=0.46)
        oak.health["reconnect_count"] = 2
        oak.health["connected"] = True
        oak.push(device_ts=1.0, gx_dps=5.0, gy_dps=6.0, gz_dps=7.0)
        data = imu.read()
        for key in (
            "gx_body_dps",
            "gy_body_dps",
            "gz_body_dps",
            "yaw_rate_source_cfg",
            "yaw_rate_source_selected",
            "yaw_rate_scale",
            "sample_age_s",
            "device_timestamp_s",
            "integrate_status",
            "count_duplicate",
            "count_stale",
            "count_integrated",
        ):
            self.assertIn(key, data)

        health = imu.get_health()
        for key in (
            "yaw_rate_source_cfg",
            "yaw_rate_source_selected",
            "yaw_rate_sign",
            "yaw_rate_scale",
            "auto_axis",
            "integrate_status",
            "sample_age_s",
            "device_timestamp_s",
            "gx_body_dps",
            "gy_body_dps",
            "gz_body_dps",
            "count_duplicate",
            "count_stale",
            "count_regressed",
            "count_restart",
            "count_integrated",
            "oak_connected",
            "oak_reconnect_count",
        ):
            self.assertIn(key, health)
        self.assertEqual(health["yaw_rate_source_cfg"], "gyro_x")
        self.assertEqual(health["yaw_rate_source_selected"], "gyro_x")
        self.assertAlmostEqual(health["yaw_rate_scale"], 0.46)
        self.assertEqual(health["oak_reconnect_count"], 2)
        self.assertAlmostEqual(health["gx_body_dps"], 5.0, places=4)
        self.assertAlmostEqual(health["gy_body_dps"], 6.0, places=4)
        self.assertAlmostEqual(health["gz_body_dps"], 7.0, places=4)

    def test_controller_status_embeds_oak_health(self):
        from config import ImuSteeringConfig
        from pi_app.control.imu_steering import ImuSteeringCompensator
        from pi_app.control.controller import Controller
        from unittest.mock import patch

        oak, imu = self._reader()
        oak.push(device_ts=1.0, gy_dps=0.0)
        imu.read()

        class MiniController:
            def __init__(self_inner):
                with patch.object(imu, "calibrate_gyro", return_value=(0.0, 0.0, 0.0)):
                    self_inner._imu_compensator = ImuSteeringCompensator(
                        ImuSteeringConfig(enabled=True, calibration_timeout_s=0.1), imu
                    )

            get_imu_status = Controller.get_imu_status

        c = MiniController()
        st = c.get_imu_status()
        self.assertIsNotNone(st)
        self.assertIn("oak_imu", st)
        self.assertIn("count_duplicate", st["oak_imu"])
        self.assertIn("yaw_rate_source_selected", st["oak_imu"])


if __name__ == "__main__":
    unittest.main()
