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
        nmni_enabled: bool = False,
        nmni_threshold_dps: float = 0.3,
        bias_adapt_enabled: bool = False,
        bias_adapt_alpha: float = 0.001,
        use_gravity_projected_yaw_rate: bool = False,
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
        self._last_device_ts_s: Optional[float] = None
        self._initialized = False
        self._gyro_bias_samples = gyro_bias_samples
        self._nmni_enabled = bool(nmni_enabled)
        self._nmni_threshold_dps = float(nmni_threshold_dps)
        self._bias_adapt_enabled = bool(bias_adapt_enabled)
        self._bias_adapt_alpha = max(0.0, min(1.0, float(bias_adapt_alpha)))
        self._use_gravity_projected_yaw_rate = bool(use_gravity_projected_yaw_rate)

        # EMA-smoothed accelerometer for gravity projection (reduces
        # vibration noise while preserving the true gravity direction).
        self._accel_ema_alpha = 0.12
        self._ax_ema: Optional[float] = None
        self._ay_ema: Optional[float] = None
        self._az_ema: Optional[float] = None

    @staticmethod
    def _compute_yaw_rate_rads(
        gx: float,
        gy: float,
        gz: float,
        sx: float,
        sy: float,
        sz: float,
        use_gravity_projected: bool,
    ) -> float:
        """Compute world-yaw rate from gyro, optionally using gravity projection.

        Direct-gyro mode is the default to avoid lever-arm acceleration coupling
        when the IMU is offset from the robot pivot.
        """
        if not use_gravity_projected:
            return gz
        a_norm_sq = sx * sx + sy * sy + sz * sz
        if a_norm_sq > 0.25:
            return (gx * sx + gy * sy + gz * sz) / a_norm_sq
        return gz

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

        # Prefer propagated device timestamp for dt; fall back to host monotonic.
        dt = None
        dev_ts = float(getattr(imu_state, "device_timestamp_s", 0.0) or 0.0)
        if dev_ts > 0.0:
            if self._last_device_ts_s is None:
                self._last_device_ts_s = dev_ts
            dt_dev = dev_ts - self._last_device_ts_s
            if dt_dev > 0.0:
                dt = dt_dev
            self._last_device_ts_s = dev_ts
        if dt is None:
            now = time.monotonic()
            if self._last_monotonic is None:
                self._last_monotonic = now
            dt = now - self._last_monotonic
            self._last_monotonic = now
        dt = max(1e-3, float(dt))

        ax_g = imu_state.ax_mss / G_MSS
        ay_g = imu_state.ay_mss / G_MSS
        az_g = imu_state.az_mss / G_MSS

        gx_raw_dps = math.degrees(imu_state.gx_rads)
        gy_raw_dps = math.degrees(imu_state.gy_rads)
        gz_raw_dps = math.degrees(imu_state.gz_rads)
        gx_dps = gx_raw_dps - self.gyro_bias_dps[0]
        gy_dps = gy_raw_dps - self.gyro_bias_dps[1]
        gz_dps = gz_raw_dps - self.gyro_bias_dps[2]
        if self._bias_adapt_enabled:
            if (
                abs(gx_dps) < self._nmni_threshold_dps
                and abs(gy_dps) < self._nmni_threshold_dps
                and abs(gz_dps) < self._nmni_threshold_dps
            ):
                a = self._bias_adapt_alpha
                bx, by, bz = self.gyro_bias_dps
                self.gyro_bias_dps = (
                    (1.0 - a) * bx + a * gx_raw_dps,
                    (1.0 - a) * by + a * gy_raw_dps,
                    (1.0 - a) * bz + a * gz_raw_dps,
                )

        # EMA-smooth the accelerometer to filter out driving vibration
        # while preserving the true gravity direction for yaw projection.
        a = self._accel_ema_alpha
        if self._ax_ema is None:
            self._ax_ema, self._ay_ema, self._az_ema = ax_g, ay_g, az_g
        else:
            self._ax_ema = a * ax_g + (1.0 - a) * self._ax_ema
            self._ay_ema = a * ay_g + (1.0 - a) * self._ay_ema
            self._az_ema = a * az_g + (1.0 - a) * self._az_ema

        # Use RAW accel for roll/pitch telemetry (complementary filter
        # already handles smoothing there).
        a_norm_sq_raw = ax_g * ax_g + ay_g * ay_g + az_g * az_g
        a_norm_raw = math.sqrt(a_norm_sq_raw) or 1.0
        ax_n = ax_g / a_norm_raw
        ay_n = ay_g / a_norm_raw
        az_n = az_g / a_norm_raw
        # OAK-D Lite BMI270 uses camera image frame: X=right, Y=down, Z=forward.
        # When flat: ay ≈ -g (Y points down, gravity is -Y), ax ≈ 0, az ≈ 0.
        # Remap to standard body frame (Z=up): roll=atan2(ax,-ay), pitch=atan2(az,-ay).
        roll_acc = math.atan2(ax_n, -ay_n)
        pitch_acc = math.atan2(az_n, -ay_n)

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        # Smoothed accel for optional gravity projection.
        sx, sy, sz = self._ax_ema, self._ay_ema, self._az_ema

        if not self._initialized:
            self.roll_rad = roll_acc
            self.pitch_rad = pitch_acc
            self.yaw_rad = 0.0
            self._initialized = True
            yaw_rate_world_dps = 0.0
        else:
            # gz = rotation around Z (forward = roll axis); gx = rotation around X (right = pitch axis).
            self.roll_rad = self.alpha_rp * (self.roll_rad + gz * dt) + (1.0 - self.alpha_rp) * roll_acc
            self.pitch_rad = self.alpha_rp * (self.pitch_rad + gx * dt) + (1.0 - self.alpha_rp) * pitch_acc

            yaw_rate_rads = self._compute_yaw_rate_rads(
                gx, gy, gz, sx, sy, sz, self._use_gravity_projected_yaw_rate
            )

            yaw_rate_world_dps = math.degrees(yaw_rate_rads)
            if self._nmni_enabled and abs(yaw_rate_world_dps) < self._nmni_threshold_dps:
                yaw_rate_world_dps = 0.0
                yaw_rate_rads = 0.0

            self.yaw_rad += yaw_rate_rads * dt

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
            "gz_dps": yaw_rate_world_dps,
            "mx_g": 0.0,
            "my_g": 0.0,
            "mz_g": 0.0,
            "temp_c": 0.0,
        }
