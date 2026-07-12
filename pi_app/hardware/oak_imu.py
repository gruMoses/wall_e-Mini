"""
OAK-D IMU reader that provides the same interface as ImuReader.

Reads accelerometer and gyroscope data from OakDepthReader's pipeline
(BMI270 on OAK-D Lite) and applies a gyro-only complementary filter to
produce heading, roll, and pitch suitable for ImuSteeringCompensator.

No magnetometer is available, so heading is relative to startup orientation
(adequate for heading-hold steering over short/medium durations).

Integration safety:
  Each distinct fresh IMU sample is integrated at most once. Duplicate
  device timestamps, stale samples, timestamp regressions (reconnect),
  invalid timestamps, and oversized gaps freeze heading rather than
  integrating cached gyro with host controller dt (which created phantom
  yaw when the controller polled faster than new packets arrived).
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from pi_app.hardware.oak_depth import OakDepthReader

G_MSS = 9.80665

# Reject samples older than this (host receive age from OakDepthReader).
_STALE_AGE_S = 0.50
# Cap sample-to-sample dt used for integration; larger gaps reseed without jump.
_MAX_INTEGRATE_DT_S = 0.15
# Treat device timestamps within this epsilon as identical.
_TS_EPS_S = 1e-9


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
        yaw_rate_source: str = "auto",
        yaw_rate_scale: float = 1.0,
        use_gravity_projected_yaw_rate: bool = False,
        stale_age_s: float = _STALE_AGE_S,
        max_integrate_dt_s: float = _MAX_INTEGRATE_DT_S,
    ) -> None:
        self._oak = oak_reader
        self.alpha_rp = complementary_alpha_rp
        self.use_mag = False
        self.calibration_path = None

        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.gyro_bias_dps = (0.0, 0.0, 0.0)

        self._last_device_ts_s: Optional[float] = None
        self._last_host_sample_ts: Optional[float] = None
        self._initialized = False
        self._gyro_bias_samples = gyro_bias_samples
        self._nmni_enabled = bool(nmni_enabled)
        self._nmni_threshold_dps = float(nmni_threshold_dps)
        self._bias_adapt_enabled = bool(bias_adapt_enabled)
        self._bias_adapt_alpha = max(0.0, min(1.0, float(bias_adapt_alpha)))
        src = str(yaw_rate_source or "auto").strip().lower()
        self._yaw_rate_source = (
            src if src in ("auto", "gyro_x", "gyro_y", "gyro_z", "gravity_projected") else "auto"
        )
        self._yaw_rate_scale = max(0.05, min(5.0, float(yaw_rate_scale)))
        self._use_gravity_projected_yaw_rate = bool(use_gravity_projected_yaw_rate)
        self._auto_axis: str = "gyro_y"
        self._auto_axis_lock_until_s: float = 0.0
        self._stale_age_s = max(0.05, float(stale_age_s))
        self._max_integrate_dt_s = max(0.02, float(max_integrate_dt_s))

        # EMA-smoothed accelerometer for gravity projection (reduces
        # vibration noise while preserving the true gravity direction).
        self._accel_ema_alpha = 0.12
        self._ax_ema: Optional[float] = None
        self._ay_ema: Optional[float] = None
        self._az_ema: Optional[float] = None

        # Observability (field diagnostics / chalk tests).
        self._integrate_status: str = "init"
        self._last_sample_age_s: float = float("inf")
        self._last_dt_s: float = 0.0
        self._last_dev_ts_seen: float = 0.0
        self._last_yaw_rate_world_dps: float = 0.0
        self._last_gx_body_dps: float = 0.0
        self._last_gy_body_dps: float = 0.0
        self._last_gz_body_dps: float = 0.0
        self._count_integrated: int = 0
        self._count_duplicate: int = 0
        self._count_stale: int = 0
        self._count_regressed: int = 0
        self._count_restart: int = 0
        self._count_invalid_ts: int = 0
        self._count_gap_freeze: int = 0
        self._count_read: int = 0

    @staticmethod
    def _compute_yaw_rate_rads(
        gx: float,
        gy: float,
        gz: float,
        sx: float,
        sy: float,
        sz: float,
        source: str,
        auto_axis: str,
        use_gravity_projected: bool,
    ) -> float:
        """Compute world-yaw rate from gyro, optionally using gravity projection.

        For OAK-D Lite camera frame (X=right, Y=down, Z=forward), robot yaw
        (turning about vertical) maps best to the body-frame Y gyro component.
        We also expose explicit axis selection for diagnostics.
        """
        if source == "auto":
            source = auto_axis
        if source == "gravity_projected" or use_gravity_projected:
            a_norm_sq = sx * sx + sy * sy + sz * sz
            if a_norm_sq > 0.25:
                return (gx * sx + gy * sy + gz * sz) / a_norm_sq
            return gy
        if source == "gyro_x":
            return gx
        if source == "gyro_z":
            return gz
        # Default: gyro_y (OAK camera frame yaw axis)
        return gy

    @staticmethod
    def _dominant_axis(gx_dps: float, gy_dps: float, gz_dps: float) -> str:
        ax = abs(gx_dps)
        ay = abs(gy_dps)
        az = abs(gz_dps)
        if ax >= ay and ax >= az:
            return "gyro_x"
        if ay >= ax and ay >= az:
            return "gyro_y"
        return "gyro_z"

    def _selected_source(self) -> str:
        if self._yaw_rate_source == "auto":
            return self._auto_axis
        if self._use_gravity_projected_yaw_rate:
            return "gravity_projected"
        return self._yaw_rate_source

    def _seed_sample_clocks(self, dev_ts: float, host_ts: float) -> None:
        """Remember sample identity without integrating (no heading jump)."""
        if dev_ts > 0.0:
            self._last_device_ts_s = dev_ts
        if host_ts > 0.0:
            self._last_host_sample_ts = host_ts

    def _resolve_integration_dt(
        self, dev_ts: float, host_ts: float, age_s: float
    ) -> tuple[Optional[float], str]:
        """Decide whether this sample may be integrated and with what dt.

        Returns (dt_s or None, status).
        status in:
          fresh | duplicate | stale | regressed | restart | invalid_ts | gap_freeze | init
        """
        if age_s > self._stale_age_s or not math.isfinite(age_s):
            self._count_stale += 1
            return None, "stale"

        # Prefer device timestamps when present and valid.
        if dev_ts > 0.0:
            last = self._last_device_ts_s
            if last is None:
                # First valid device clock after start — seed, do not invent dt.
                self._seed_sample_clocks(dev_ts, host_ts)
                self._count_restart += 1
                return None, "restart"
            delta = dev_ts - last
            if delta < -_TS_EPS_S:
                # Device clock went backwards (USB reconnect / pipeline rebuild).
                self._count_regressed += 1
                self._count_restart += 1
                self._seed_sample_clocks(dev_ts, host_ts)
                return None, "regressed"
            if abs(delta) <= _TS_EPS_S:
                self._count_duplicate += 1
                return None, "duplicate"
            if delta > self._max_integrate_dt_s:
                # Oversized gap: reseed without integrating a huge step.
                self._count_gap_freeze += 1
                self._seed_sample_clocks(dev_ts, host_ts)
                return None, "gap_freeze"
            self._seed_sample_clocks(dev_ts, host_ts)
            return float(delta), "fresh"

        # Invalid / missing device timestamp: still require a *new* host sample
        # identity. Never fall back to controller-loop host monotonic dt.
        self._count_invalid_ts += 1
        if host_ts <= 0.0 or not math.isfinite(host_ts):
            return None, "invalid_ts"
        last_h = self._last_host_sample_ts
        if last_h is None:
            self._seed_sample_clocks(0.0, host_ts)
            self._count_restart += 1
            return None, "restart"
        delta_h = host_ts - last_h
        if delta_h < -_TS_EPS_S:
            self._count_regressed += 1
            self._count_restart += 1
            self._seed_sample_clocks(0.0, host_ts)
            return None, "regressed"
        if abs(delta_h) <= _TS_EPS_S:
            self._count_duplicate += 1
            return None, "duplicate"
        if delta_h > self._max_integrate_dt_s:
            self._count_gap_freeze += 1
            self._seed_sample_clocks(0.0, host_ts)
            return None, "gap_freeze"
        self._seed_sample_clocks(0.0, host_ts)
        return float(delta_h), "fresh"

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

    def get_health(self) -> Dict[str, Any]:
        """Return heading-integration health / field diagnostics."""
        oak_health: Dict[str, Any] = {}
        getter = getattr(self._oak, "get_health", None)
        if callable(getter):
            try:
                raw = getter()
                if isinstance(raw, dict):
                    oak_health = {
                        "oak_connected": raw.get("connected"),
                        "oak_reconnect_count": raw.get("reconnect_count"),
                        "oak_last_disconnect_ts": raw.get("last_disconnect_ts"),
                        "oak_pipeline_running": raw.get("pipeline_running"),
                    }
            except Exception:
                oak_health = {}

        selected = self._selected_source()
        return {
            "yaw_rate_source_cfg": self._yaw_rate_source,
            "yaw_rate_source_selected": selected,
            "yaw_rate_sign": -1.0,  # heading_deg = (-yaw_rad_deg) mod 360
            "yaw_rate_scale": self._yaw_rate_scale,
            "auto_axis": self._auto_axis,
            "use_gravity_projected": self._use_gravity_projected_yaw_rate,
            "nmni_enabled": self._nmni_enabled,
            "nmni_threshold_dps": self._nmni_threshold_dps,
            "integrate_status": self._integrate_status,
            "sample_age_s": self._last_sample_age_s,
            "device_timestamp_s": self._last_dev_ts_seen,
            "last_integrated_device_ts_s": self._last_device_ts_s,
            "last_host_sample_ts": self._last_host_sample_ts,
            "last_dt_s": self._last_dt_s,
            "gx_body_dps": self._last_gx_body_dps,
            "gy_body_dps": self._last_gy_body_dps,
            "gz_body_dps": self._last_gz_body_dps,
            "yaw_rate_world_dps": self._last_yaw_rate_world_dps,
            "heading_deg": (-math.degrees(self.yaw_rad) + 360.0) % 360.0,
            "count_read": self._count_read,
            "count_integrated": self._count_integrated,
            "count_duplicate": self._count_duplicate,
            "count_stale": self._count_stale,
            "count_regressed": self._count_regressed,
            "count_restart": self._count_restart,
            "count_invalid_ts": self._count_invalid_ts,
            "count_gap_freeze": self._count_gap_freeze,
            **oak_health,
        }

    def read(self) -> Dict[str, float]:
        """Read latest IMU data from OAK-D and return in ImuReader format.

        Heading integration consumes each distinct fresh sample at most once.
        Duplicate controller-loop polls of the same packet freeze orientation
        (no phantom yaw from host dt).
        """
        self._count_read += 1
        imu_state, age = self._oak.get_imu_data()
        age_s = float(age) if age is not None else float("inf")
        self._last_sample_age_s = age_s

        dev_ts = float(getattr(imu_state, "device_timestamp_s", 0.0) or 0.0)
        host_ts = float(getattr(imu_state, "timestamp", 0.0) or 0.0)
        self._last_dev_ts_seen = dev_ts

        ax_g = imu_state.ax_mss / G_MSS
        ay_g = imu_state.ay_mss / G_MSS
        az_g = imu_state.az_mss / G_MSS

        gx_raw_dps = math.degrees(imu_state.gx_rads)
        gy_raw_dps = math.degrees(imu_state.gy_rads)
        gz_raw_dps = math.degrees(imu_state.gz_rads)
        gx_dps = gx_raw_dps - self.gyro_bias_dps[0]
        gy_dps = gy_raw_dps - self.gyro_bias_dps[1]
        gz_dps = gz_raw_dps - self.gyro_bias_dps[2]
        self._last_gx_body_dps = gx_dps
        self._last_gy_body_dps = gy_dps
        self._last_gz_body_dps = gz_dps

        if self._yaw_rate_source == "auto":
            now = time.monotonic()
            turn_mag = max(abs(gx_dps), abs(gy_dps), abs(gz_dps))
            # Re-select dominant axis only when truly turning, then hold briefly.
            if turn_mag > 8.0 and now >= self._auto_axis_lock_until_s:
                self._auto_axis = self._dominant_axis(gx_dps, gy_dps, gz_dps)
                self._auto_axis_lock_until_s = now + 0.8

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

        # Yaw rate for telemetry even when not integrating this tick.
        yaw_rate_rads = self._compute_yaw_rate_rads(
            gx,
            gy,
            gz,
            sx if sx is not None else 0.0,
            sy if sy is not None else 0.0,
            sz if sz is not None else 0.0,
            self._yaw_rate_source,
            self._auto_axis,
            self._use_gravity_projected_yaw_rate,
        )
        yaw_rate_rads *= self._yaw_rate_scale
        yaw_rate_world_dps = math.degrees(yaw_rate_rads)
        if self._nmni_enabled and abs(yaw_rate_world_dps) < self._nmni_threshold_dps:
            yaw_rate_world_dps = 0.0
            yaw_rate_rads = 0.0

        if not self._initialized:
            self.roll_rad = roll_acc
            self.pitch_rad = pitch_acc
            self.yaw_rad = 0.0
            self._initialized = True
            self._seed_sample_clocks(dev_ts, host_ts)
            self._integrate_status = "init"
            self._last_dt_s = 0.0
            self._last_yaw_rate_world_dps = 0.0
            yaw_rate_world_dps = 0.0
        else:
            dt, status = self._resolve_integration_dt(dev_ts, host_ts, age_s)
            self._integrate_status = status
            if dt is not None and status == "fresh":
                self._last_dt_s = dt
                # gz = rotation around Z (forward = roll axis);
                # gx = rotation around X (right = pitch axis).
                self.roll_rad = self.alpha_rp * (self.roll_rad + gz * dt) + (1.0 - self.alpha_rp) * roll_acc
                self.pitch_rad = self.alpha_rp * (self.pitch_rad + gx * dt) + (1.0 - self.alpha_rp) * pitch_acc
                self.yaw_rad += yaw_rate_rads * dt
                self._count_integrated += 1
                self._last_yaw_rate_world_dps = yaw_rate_world_dps
            else:
                # Freeze orientation; zero reported rate so PID D-term does not
                # act on a held packet as if it were continuous motion.
                self._last_dt_s = 0.0
                self._last_yaw_rate_world_dps = 0.0
                yaw_rate_world_dps = 0.0
                # Still gently pull roll/pitch toward accel when frozen so
                # attitude telemetry recovers after reconnect without yaw jump.
                if status in ("stale", "regressed", "restart", "gap_freeze"):
                    blend = 1.0 - self.alpha_rp
                    self.roll_rad = (1.0 - blend) * self.roll_rad + blend * roll_acc
                    self.pitch_rad = (1.0 - blend) * self.pitch_rad + blend * pitch_acc

        heading_deg = (-math.degrees(self.yaw_rad) + 360.0) % 360.0
        selected = self._selected_source()

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
            # ImuReader compatibility: gz_dps is the *selected* world yaw rate
            # used by steering D-term (not necessarily body-frame Z).
            "gz_dps": yaw_rate_world_dps,
            "mx_g": 0.0,
            "my_g": 0.0,
            "mz_g": 0.0,
            "temp_c": 0.0,
            # Explicit diagnostics (extra keys; safe for existing consumers).
            "gx_body_dps": gx_dps,
            "gy_body_dps": gy_dps,
            "gz_body_dps": gz_dps,
            "yaw_rate_world_dps": yaw_rate_world_dps,
            "yaw_rate_source_cfg": self._yaw_rate_source,
            "yaw_rate_source_selected": selected,
            "yaw_rate_scale": self._yaw_rate_scale,
            "sample_age_s": age_s,
            "device_timestamp_s": dev_ts,
            "integrate_status": self._integrate_status,
            "integrate_dt_s": self._last_dt_s,
            "count_duplicate": float(self._count_duplicate),
            "count_stale": float(self._count_stale),
            "count_regressed": float(self._count_regressed),
            "count_restart": float(self._count_restart),
            "count_integrated": float(self._count_integrated),
        }
