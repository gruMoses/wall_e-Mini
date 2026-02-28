"""
OAK-D IMU reader that provides the same interface as ImuReader.

Reads accelerometer and gyroscope data from OakDepthReader's pipeline
(BMI270 on OAK-D Lite) and applies a gyro-only complementary filter to
produce heading, roll, and pitch suitable for ImuSteeringCompensator.

No magnetometer is available, so heading is relative to startup orientation
(adequate for heading-hold steering over short/medium durations).
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from pi_app.hardware.oak_depth import OakDepthReader

G_MSS = 9.80665


class OakImuReader:
    """Drop-in replacement for ImuReader, backed by the OAK-D's onboard IMU."""

    def __init__(
        self,
        oak_reader: OakDepthReader,
        complementary_alpha_rp: float = 0.98,
        gyro_bias_samples: int = 200,
    ) -> None:
        self._oak = oak_reader
        self.alpha_rp = complementary_alpha_rp
        self.use_mag = False
        self.calibration_path = None

        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.gyro_bias_dps = (0.0, 0.0, 0.0)

        self._last_monotonic: Optional[float] = None
        self._initialized = False
        self._gyro_bias_samples = gyro_bias_samples

    def calibrate_gyro(self, duration_s: float = 3.0) -> tuple:
        """Collect gyro samples from OAK-D IMU to estimate bias."""
        xs, ys, zs = [], [], []
        end = time.monotonic() + float(duration_s)
        while time.monotonic() < end:
            imu_state, age = self._oak.get_imu_data()
            if age < 0.5:
                xs.append(math.degrees(imu_state.gx_rads))
                ys.append(math.degrees(imu_state.gy_rads))
                zs.append(math.degrees(imu_state.gz_rads))
            time.sleep(0.01)
        if xs:
            self.gyro_bias_dps = (
                sum(xs) / len(xs),
                sum(ys) / len(ys),
                sum(zs) / len(zs),
            )
        return self.gyro_bias_dps

    def calibrate_mag_hard_iron(self, duration_s: float = 5.0) -> tuple:
        """No-op: OAK-D Lite has no magnetometer."""
        return (0.0, 0.0, 0.0)

    def calibrate_mag(self, duration_s: float = 8.0) -> tuple:
        """No-op: OAK-D Lite has no magnetometer."""
        return (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)

    def read(self) -> Dict[str, float]:
        """Read latest IMU data from OAK-D and return in ImuReader format."""
        imu_state, age = self._oak.get_imu_data()

        now = time.monotonic()
        if self._last_monotonic is None:
            self._last_monotonic = now
        dt = max(1e-3, now - self._last_monotonic)
        self._last_monotonic = now

        ax_g = imu_state.ax_mss / G_MSS
        ay_g = imu_state.ay_mss / G_MSS
        az_g = imu_state.az_mss / G_MSS

        gx_dps = math.degrees(imu_state.gx_rads) - self.gyro_bias_dps[0]
        gy_dps = math.degrees(imu_state.gy_rads) - self.gyro_bias_dps[1]
        gz_dps = math.degrees(imu_state.gz_rads) - self.gyro_bias_dps[2]

        # Accel-based roll and pitch
        norm = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g) or 1.0
        ax_n, ay_n, az_n = ax_g / norm, ay_g / norm, az_g / norm
        roll_acc = math.atan2(ay_n, az_n)
        pitch_acc = math.atan2(-ax_n, math.sqrt(ay_n * ay_n + az_n * az_n))

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        if not self._initialized:
            self.roll_rad = roll_acc
            self.pitch_rad = pitch_acc
            self.yaw_rad = 0.0
            self._initialized = True
        else:
            self.roll_rad = self.alpha_rp * (self.roll_rad + gx * dt) + (1.0 - self.alpha_rp) * roll_acc
            self.pitch_rad = self.alpha_rp * (self.pitch_rad + gy * dt) + (1.0 - self.alpha_rp) * pitch_acc
            self.yaw_rad += gz * dt

        heading_deg = (-math.degrees(self.yaw_rad) + 360.0) % 360.0

        return {
            "roll_deg": math.degrees(self.roll_rad),
            "pitch_deg": math.degrees(self.pitch_rad),
            "yaw_deg": -math.degrees(self.yaw_rad),
            "heading_deg": heading_deg,
            "ax_g": ax_g,
            "ay_g": ay_g,
            "az_g": az_g,
            "gx_dps": gx_dps,
            "gy_dps": gy_dps,
            "gz_dps": gz_dps,
            "mx_g": 0.0,
            "my_g": 0.0,
            "mz_g": 0.0,
            "temp_c": 0.0,
        }
