"""
OAK-D IMU reader that provides the same interface as ImuReader.

Reads accelerometer and gyroscope data from OakDepthReader's pipeline
(BMI270 on OAK-D Lite) and applies a gyro-only complementary filter to
produce heading, roll, and pitch suitable for ImuSteeringCompensator.

No magnetometer is available, so heading is relative to startup orientation
(adequate for heading-hold steering over short/medium durations).

Yaw integration (lossless producer path)
----------------------------------------
OakDepthReader integrates **every** drained IMU packet on the producer side
into unscaled cumulative free-yaw channels (body X/Y/Z + gravity projection)
using device timestamps. This class:

  * publishes the latest body rates for diagnostics
  * selects the configured yaw source (gyro_x/y/z / auto / gravity_projected)
  * applies ``yaw_rate_scale`` **exactly once** to producer cum deltas
  * never re-integrates gyro×dt from sparse snapshots (no double-integrate)
  * preserves unread cum deltas across producer generation bumps (timestamp
    jitter / clock reseed) — including legitimate +turn then -turn that
    returns free-yaw channels to ~0
  * freezes heading on stale samples and true producer replacement only
    (integrated-packet counter rewind / session identity — never a near-zero
    cum magnitude heuristic)

See ``pi_app.hardware.oak_imu_yaw_producer`` and ``docs/heading_tuning.md``.
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
# Cap sample-to-sample dt used for roll/pitch and legacy fallback only.
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
        yaw_rate_source: str = "gyro_y",
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
        src = str(yaw_rate_source or "gyro_y").strip().lower()
        self._yaw_rate_source = (
            src if src in ("auto", "gyro_x", "gyro_y", "gyro_z", "gravity_projected") else "gyro_y"
        )
        self._yaw_rate_scale = max(0.05, min(5.0, float(yaw_rate_scale)))
        self._use_gravity_projected_yaw_rate = bool(use_gravity_projected_yaw_rate)
        self._auto_axis: str = "gyro_y"
        self._auto_axis_lock_until_s: float = 0.0
        self._stale_age_s = max(0.05, float(stale_age_s))
        self._max_integrate_dt_s = max(0.02, float(max_integrate_dt_s))

        # EMA-smoothed accelerometer for gravity projection telemetry.
        self._accel_ema_alpha = 0.12
        self._ax_ema: Optional[float] = None
        self._ay_ema: Optional[float] = None
        self._az_ema: Optional[float] = None

        # Producer-cum consumption state (prevents double-integrate).
        self._last_cum_x: Optional[float] = None
        self._last_cum_y: Optional[float] = None
        self._last_cum_z: Optional[float] = None
        self._last_cum_grav: Optional[float] = None
        self._last_yaw_generation: Optional[int] = None
        self._last_producer_integrated: int = 0
        self._has_producer_cum = False

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
        self._count_producer_packets: int = 0
        self._count_generation_change: int = 0
        self._count_cum_reset: int = 0

        # Push bias + NMNI into producer. NMNI may be on before the first
        # calibrate_gyro; calibrate temporarily clears it so bias estimation
        # cannot be circular if the host snapshot is integrate-path corrected.
        self._sync_producer_config()

    def _push_producer_nmni(self, enabled: bool, threshold_dps: float) -> None:
        setter = getattr(self._oak, "set_imu_nmni", None)
        if callable(setter):
            try:
                setter(bool(enabled), float(threshold_dps))
            except Exception:
                pass

    def _push_producer_bias_dps(self, gx_dps: float, gy_dps: float, gz_dps: float) -> None:
        bias_setter = getattr(self._oak, "set_imu_gyro_bias_dps", None)
        if callable(bias_setter):
            try:
                bias_setter(float(gx_dps), float(gy_dps), float(gz_dps))
            except Exception:
                pass

    def _sync_producer_config(self) -> None:
        """Push current local bias + NMNI settings into the producer integrator."""
        bx, by, bz = self.gyro_bias_dps
        self._push_producer_bias_dps(bx, by, bz)
        self._push_producer_nmni(self._nmni_enabled, self._nmni_threshold_dps)

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

    def _select_cum(
        self,
        cum_x: float,
        cum_y: float,
        cum_z: float,
        cum_grav: float,
    ) -> float:
        src = self._selected_source()
        if src == "gyro_x":
            return cum_x
        if src == "gyro_z":
            return cum_z
        if src == "gravity_projected":
            return cum_grav
        return cum_y

    def _seed_sample_clocks(self, dev_ts: float, host_ts: float) -> None:
        """Remember sample identity without integrating (no heading jump)."""
        if dev_ts > 0.0:
            self._last_device_ts_s = dev_ts
        if host_ts > 0.0:
            self._last_host_sample_ts = host_ts

    def _seed_producer_cums(
        self,
        cum_x: float,
        cum_y: float,
        cum_z: float,
        cum_grav: float,
        generation: int,
        producer_integrated: int,
    ) -> None:
        self._last_cum_x = cum_x
        self._last_cum_y = cum_y
        self._last_cum_z = cum_z
        self._last_cum_grav = cum_grav
        self._last_yaw_generation = generation
        self._last_producer_integrated = producer_integrated
        self._has_producer_cum = True

    def _resolve_integration_dt(
        self, dev_ts: float, host_ts: float, age_s: float
    ) -> tuple[Optional[float], str]:
        """Legacy sparse-snapshot dt resolver (fallback when no producer cum)."""
        if age_s > self._stale_age_s or not math.isfinite(age_s):
            self._count_stale += 1
            return None, "stale"

        if dev_ts > 0.0:
            last = self._last_device_ts_s
            if last is None:
                self._seed_sample_clocks(dev_ts, host_ts)
                self._count_restart += 1
                return None, "restart"
            delta = dev_ts - last
            if delta < -_TS_EPS_S:
                self._count_regressed += 1
                self._count_restart += 1
                self._seed_sample_clocks(dev_ts, host_ts)
                return None, "regressed"
            if abs(delta) <= _TS_EPS_S:
                self._count_duplicate += 1
                return None, "duplicate"
            if delta > self._max_integrate_dt_s:
                self._count_gap_freeze += 1
                self._seed_sample_clocks(dev_ts, host_ts)
                return None, "gap_freeze"
            self._seed_sample_clocks(dev_ts, host_ts)
            return float(delta), "fresh"

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

    def _sample_raw_gyro_dps(self) -> tuple[Optional[tuple[float, float, float]], float]:
        """Return ((gx, gy, gz) dps raw body rates, age_s) or (None, age).

        Prefers ``OakDepthReader.get_imu_raw_gyro_dps`` (explicit raw contract).
        Falls back to ``get_imu_data().gx/gy/gz_rads``, which the producer
        documents as the latest **raw** sample (not bias/NMNI corrected).
        """
        raw_fn = getattr(self._oak, "get_imu_raw_gyro_dps", None)
        if callable(raw_fn):
            try:
                raw, age = raw_fn()
                age_s = float(age) if age is not None else float("inf")
                if raw is None:
                    return None, age_s
                return (
                    (float(raw[0]), float(raw[1]), float(raw[2])),
                    age_s,
                )
            except Exception:
                pass
        imu_state, age = self._oak.get_imu_data()
        age_s = float(age) if age is not None else float("inf")
        return (
            (
                math.degrees(float(imu_state.gx_rads)),
                math.degrees(float(imu_state.gy_rads)),
                math.degrees(float(imu_state.gz_rads)),
            ),
            age_s,
        )

    def calibrate_gyro(self, duration_s: float = 3.0) -> tuple:
        """Collect stationary RAW gyro samples to estimate bias.

        Circular-calibration guard
        --------------------------
        ``__init__`` may enable producer NMNI (and zero bias) before the first
        calibrate call. If the host snapshot ever exposed integrate-path rates
        (bias-subtracted then NMNI-gated), sub-threshold residual bias would
        read as exact zero and calibration would be a no-op — leaving that
        residual in the lossless producer integral (CW/CCW magnitude skew).

        During collection we force producer bias=0 and NMNI=off so any
        corrected snapshot collapses to raw. Current ``get_imu_data`` already
        returns raw latest rates; this is then a no-op on sample values but
        keeps calibration correct if that contract ever changes. Prefer
        ``get_imu_raw_gyro_dps`` when present.
        """
        prior_bias = self.gyro_bias_dps
        measured: Optional[tuple] = None
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        try:
            # Pause integrate-path transforms for the collection window.
            self.gyro_bias_dps = (0.0, 0.0, 0.0)
            self._push_producer_bias_dps(0.0, 0.0, 0.0)
            self._push_producer_nmni(False, self._nmni_threshold_dps)

            end = time.monotonic() + float(duration_s)
            while time.monotonic() < end:
                sample, age_s = self._sample_raw_gyro_dps()
                if sample is not None and age_s < 0.5:
                    xs.append(sample[0])
                    ys.append(sample[1])
                    zs.append(sample[2])
                time.sleep(0.01)
            if xs:
                measured = (
                    sum(xs) / len(xs),
                    sum(ys) / len(ys),
                    sum(zs) / len(zs),
                )
        finally:
            # Always land on a real bias — the measured mean, or the prior
            # estimate on empty collection *or* a mid-window sampling throw.
            # Never leave the transient bias=0 that opened the window: the
            # caller (imu_steering) swallows exceptions and does not retry, so
            # a zeroed bias would silently reintroduce the residual-integrate
            # skew this whole path exists to remove.
            self.gyro_bias_dps = measured if measured is not None else prior_bias
            # Restore NMNI + measured/prior bias on the producer.
            self._sync_producer_config()
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

        imu_metrics: Dict[str, Any] = {}
        metrics_fn = getattr(self._oak, "get_imu_metrics", None)
        if callable(metrics_fn):
            try:
                raw_m = metrics_fn()
                if isinstance(raw_m, dict):
                    imu_metrics = {
                        "producer_packets_received": raw_m.get("packets_received"),
                        "producer_packets_drained": raw_m.get("packets_drained", raw_m.get("packets_received")),
                        "producer_packets_parsed": raw_m.get("packets_parsed"),
                        "producer_packets_integrated": raw_m.get("packets_integrated"),
                        "producer_packets_coalesced": raw_m.get("packets_coalesced"),
                        "producer_packets_duplicate": raw_m.get("packets_duplicate"),
                        "producer_packets_regressed": raw_m.get("packets_regressed"),
                        "producer_packets_restart": raw_m.get("packets_restart"),
                        "producer_packets_gap_freeze": raw_m.get("packets_gap_freeze"),
                        "producer_packets_backlog_dropped": raw_m.get("packets_backlog_dropped"),
                        "producer_cadence_avg_s": raw_m.get("cadence_avg_s"),
                        "producer_cadence_max_s": raw_m.get("cadence_max_s"),
                        "producer_cum_yaw_x_deg": raw_m.get("producer_cum_yaw_x_deg"),
                        "producer_cum_yaw_y_deg": raw_m.get("producer_cum_yaw_y_deg"),
                        "producer_cum_yaw_z_deg": raw_m.get("producer_cum_yaw_z_deg"),
                        "producer_cum_yaw_grav_deg": raw_m.get("producer_cum_yaw_grav_deg"),
                        "producer_generation": raw_m.get("producer_generation"),
                        "producer_last_status": raw_m.get("producer_last_status"),
                        "queue_msgs_received": raw_m.get("queue_msgs_received"),
                        "queue_msgs_consumed": raw_m.get("queue_msgs_consumed"),
                        "queue_msgs_dropped": raw_m.get("queue_msgs_dropped"),
                        "queue_msgs_overwrite_observable": raw_m.get(
                            "queue_msgs_overwrite_observable", False
                        ),
                        "queue_drain_count": raw_m.get("queue_drain_count"),
                        "last_batch_packets": raw_m.get("last_batch_packets"),
                        "host_queue_max_size": raw_m.get("host_queue_max_size"),
                        "host_queue_blocking": raw_m.get("host_queue_blocking"),
                        "max_packets_per_drain": raw_m.get("max_packets_per_drain"),
                        # Drain-batch size stats — NOT host-queue occupancy/overflow.
                        "drain_batch_high_water_msgs": raw_m.get("drain_batch_high_water_msgs"),
                        "drain_batch_large_events": raw_m.get("drain_batch_large_events"),
                        "drain_batch_full_size_events": raw_m.get("drain_batch_full_size_events"),
                    }
            except Exception:
                imu_metrics = {}

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
            "integration_path": "producer" if self._has_producer_cum else "legacy_snapshot",
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
            "count_producer_packets": self._count_producer_packets,
            "count_generation_change": self._count_generation_change,
            "count_cum_reset": self._count_cum_reset,
            "yaw_generation": self._last_yaw_generation,
            **oak_health,
            **imu_metrics,
        }

    def read(self) -> Dict[str, float]:
        """Read latest IMU data from OAK-D and return in ImuReader format.

        Heading advances from producer cumulative free-yaw deltas (scale applied
        once). Duplicate controller-loop polls of an unchanged cum freeze
        orientation (no phantom yaw).
        """
        self._count_read += 1
        imu_state, age = self._oak.get_imu_data()
        age_s = float(age) if age is not None else float("inf")
        self._last_sample_age_s = age_s

        dev_ts = float(getattr(imu_state, "device_timestamp_s", 0.0) or 0.0)
        host_ts = float(getattr(imu_state, "timestamp", 0.0) or 0.0)
        self._last_dev_ts_seen = dev_ts

        cum_x = float(getattr(imu_state, "cum_yaw_x_rad", 0.0) or 0.0)
        cum_y = float(getattr(imu_state, "cum_yaw_y_rad", 0.0) or 0.0)
        cum_z = float(getattr(imu_state, "cum_yaw_z_rad", 0.0) or 0.0)
        cum_grav = float(getattr(imu_state, "cum_yaw_grav_rad", 0.0) or 0.0)
        generation = int(getattr(imu_state, "yaw_generation", 0) or 0)
        producer_integrated = int(
            getattr(imu_state, "producer_packets_integrated", 0) or 0
        )
        # Producer path when oak state exposes cum channels (real OakDepthReader
        # and producer-backed fakes). Legacy stubs without these fields keep the
        # sparse-snapshot path for unit-test compatibility.
        use_producer = hasattr(imu_state, "cum_yaw_y_rad")

        ax_g = imu_state.ax_mss / G_MSS
        ay_g = imu_state.ay_mss / G_MSS
        az_g = imu_state.az_mss / G_MSS

        gx_raw_dps = math.degrees(imu_state.gx_rads)
        gy_raw_dps = math.degrees(imu_state.gy_rads)
        gz_raw_dps = math.degrees(imu_state.gz_rads)
        # Body rates for diagnostics: subtract local bias view. Producer already
        # integrates with its own bias copy (kept in sync via calibrate / adapt).
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
                self._sync_producer_config()

        # EMA-smooth the accelerometer for telemetry / gravity rate display.
        a = self._accel_ema_alpha
        if self._ax_ema is None:
            self._ax_ema, self._ay_ema, self._az_ema = ax_g, ay_g, az_g
        else:
            self._ax_ema = a * ax_g + (1.0 - a) * self._ax_ema
            self._ay_ema = a * ay_g + (1.0 - a) * self._ay_ema
            self._az_ema = a * az_g + (1.0 - a) * self._az_ema

        a_norm_sq_raw = ax_g * ax_g + ay_g * ay_g + az_g * az_g
        a_norm_raw = math.sqrt(a_norm_sq_raw) or 1.0
        ax_n = ax_g / a_norm_raw
        ay_n = ay_g / a_norm_raw
        az_n = az_g / a_norm_raw
        # OAK-D Lite BMI270 uses camera image frame: X=right, Y=down, Z=forward.
        roll_acc = math.atan2(ax_n, -ay_n)
        pitch_acc = math.atan2(az_n, -ay_n)

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        sx, sy, sz = self._ax_ema, self._ay_ema, self._az_ema

        # Instantaneous yaw rate for telemetry (scale once).
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
            if use_producer:
                self._seed_producer_cums(
                    cum_x, cum_y, cum_z, cum_grav, generation, producer_integrated
                )
            self._integrate_status = "init"
            self._last_dt_s = 0.0
            self._last_yaw_rate_world_dps = 0.0
            yaw_rate_world_dps = 0.0
        elif use_producer:
            yaw_rate_world_dps = self._consume_producer_yaw(
                cum_x=cum_x,
                cum_y=cum_y,
                cum_z=cum_z,
                cum_grav=cum_grav,
                generation=generation,
                producer_integrated=producer_integrated,
                age_s=age_s,
                dev_ts=dev_ts,
                host_ts=host_ts,
                roll_acc=roll_acc,
                pitch_acc=pitch_acc,
                gx=gx,
                gy=gy,
                gz=gz,
                yaw_rate_world_dps=yaw_rate_world_dps,
            )
        else:
            # Legacy sparse-snapshot path (tests / stubs without cum fields).
            dt, status = self._resolve_integration_dt(dev_ts, host_ts, age_s)
            self._integrate_status = status
            if dt is not None and status == "fresh":
                self._last_dt_s = dt
                self.roll_rad = self.alpha_rp * (self.roll_rad + gz * dt) + (1.0 - self.alpha_rp) * roll_acc
                self.pitch_rad = self.alpha_rp * (self.pitch_rad + gx * dt) + (1.0 - self.alpha_rp) * pitch_acc
                self.yaw_rad += yaw_rate_rads * dt
                self._count_integrated += 1
                self._last_yaw_rate_world_dps = yaw_rate_world_dps
            else:
                self._last_dt_s = 0.0
                self._last_yaw_rate_world_dps = 0.0
                yaw_rate_world_dps = 0.0
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
            "integration_path": "producer" if self._has_producer_cum else "legacy_snapshot",
            "count_duplicate": float(self._count_duplicate),
            "count_stale": float(self._count_stale),
            "count_regressed": float(self._count_regressed),
            "count_restart": float(self._count_restart),
            "count_integrated": float(self._count_integrated),
            "count_producer_packets": float(self._count_producer_packets),
            "producer_cum_yaw_x_deg": math.degrees(cum_x),
            "producer_cum_yaw_y_deg": math.degrees(cum_y),
            "producer_cum_yaw_z_deg": math.degrees(cum_z),
            "producer_cum_yaw_grav_deg": math.degrees(cum_grav),
            "yaw_generation": float(generation),
        }

    def _producer_session_rewound(self, producer_integrated: int) -> bool:
        """True only on reliable producer-replacement / counter rewind.

        A legitimate +turn then -turn that returns free-yaw channels to ~0 is
        **not** a reset and must not discard the reverse delta. Generation
        bumps alone (timestamp reseed / pipeline clock) are also not a reset.

        Reliable signals only:
          * monotonic ``producer_packets_integrated`` counter rewound
            (new producer session / integrator replacement)
        """
        return int(producer_integrated) < int(self._last_producer_integrated)

    def _blend_attitude(self, roll_acc: float, pitch_acc: float) -> None:
        blend = 1.0 - self.alpha_rp
        self.roll_rad = (1.0 - blend) * self.roll_rad + blend * roll_acc
        self.pitch_rad = (1.0 - blend) * self.pitch_rad + blend * pitch_acc

    def _apply_producer_delta(
        self,
        *,
        delta_unscaled: float,
        packets_delta: int,
        roll_acc: float,
        pitch_acc: float,
        gx: float,
        gz: float,
        yaw_rate_world_dps: float,
    ) -> float:
        """Apply scale-once cum delta and advance roll/pitch. Returns yaw rate dps."""
        delta_scaled = delta_unscaled * self._yaw_rate_scale
        self.yaw_rad += delta_scaled
        self._count_integrated += 1
        self._count_producer_packets += max(0, packets_delta)
        self._integrate_status = "fresh"

        dt_rp = 0.0
        if packets_delta > 0 and self._max_integrate_dt_s > 0:
            dt_rp = min(self._max_integrate_dt_s, 0.01 * packets_delta)
        if dt_rp <= 0.0:
            dt_rp = min(self._max_integrate_dt_s, 0.02)
        self._last_dt_s = dt_rp
        self.roll_rad = self.alpha_rp * (self.roll_rad + gz * dt_rp) + (1.0 - self.alpha_rp) * roll_acc
        self.pitch_rad = self.alpha_rp * (self.pitch_rad + gx * dt_rp) + (1.0 - self.alpha_rp) * pitch_acc

        if abs(yaw_rate_world_dps) < 1e-9 and packets_delta > 0 and abs(delta_scaled) > 0.0:
            approx_dt = max(1e-3, 0.01 * packets_delta)
            yaw_rate_world_dps = math.degrees(delta_scaled / approx_dt)
        self._last_yaw_rate_world_dps = yaw_rate_world_dps
        return yaw_rate_world_dps

    def _consume_producer_yaw(
        self,
        *,
        cum_x: float,
        cum_y: float,
        cum_z: float,
        cum_grav: float,
        generation: int,
        producer_integrated: int,
        age_s: float,
        dev_ts: float,
        host_ts: float,
        roll_acc: float,
        pitch_acc: float,
        gx: float,
        gy: float,
        gz: float,
        yaw_rate_world_dps: float,
    ) -> float:
        """Advance heading from producer cum deltas; scale applied once here.

        Generation bumps alone (producer timestamp jitter / pipeline clock
        reseed) must **not** discard unread cum — including a legitimate
        reverse turn that returns free-yaw channels to ~0. Only a true
        producer session replacement (integrated-packet counter rewind)
        reseeds without applying the pending delta.
        """
        self._has_producer_cum = True

        if age_s > self._stale_age_s or not math.isfinite(age_s):
            self._count_stale += 1
            self._integrate_status = "stale"
            self._last_dt_s = 0.0
            self._last_yaw_rate_world_dps = 0.0
            self._blend_attitude(roll_acc, pitch_acc)
            return 0.0

        if self._last_cum_y is None or self._last_yaw_generation is None:
            self._seed_producer_cums(
                cum_x, cum_y, cum_z, cum_grav, generation, producer_integrated
            )
            self._seed_sample_clocks(dev_ts, host_ts)
            self._count_restart += 1
            self._integrate_status = "restart"
            self._last_dt_s = 0.0
            self._last_yaw_rate_world_dps = 0.0
            return 0.0

        # True producer replacement: counter rewind only (never near-zero cum).
        if self._producer_session_rewound(producer_integrated):
            self._count_cum_reset += 1
            self._count_restart += 1
            if generation != self._last_yaw_generation:
                self._count_generation_change += 1
            self._seed_producer_cums(
                cum_x, cum_y, cum_z, cum_grav, generation, producer_integrated
            )
            self._seed_sample_clocks(dev_ts, host_ts)
            self._integrate_status = "cum_reset"
            self._last_dt_s = 0.0
            self._last_yaw_rate_world_dps = 0.0
            self._blend_attitude(roll_acc, pitch_acc)
            return 0.0

        prev_sel = self._select_cum(
            self._last_cum_x or 0.0,
            self._last_cum_y or 0.0,
            self._last_cum_z or 0.0,
            self._last_cum_grav or 0.0,
        )
        cur_sel = self._select_cum(cum_x, cum_y, cum_z, cum_grav)
        delta_unscaled = cur_sel - prev_sel
        packets_delta = max(0, producer_integrated - self._last_producer_integrated)
        generation_changed = generation != self._last_yaw_generation

        if generation_changed:
            self._count_generation_change += 1

        # Continuous cum (same or new generation): apply pending delta once.
        # Generation bumps with continuous free-yaw (timestamp reseed / reverse
        # turn back to ~0) preserve the delta — never treat near-zero as reset.
        self._last_cum_x = cum_x
        self._last_cum_y = cum_y
        self._last_cum_z = cum_z
        self._last_cum_grav = cum_grav
        self._last_yaw_generation = generation
        self._last_producer_integrated = producer_integrated
        self._seed_sample_clocks(dev_ts, host_ts)

        if packets_delta == 0 and abs(delta_unscaled) <= _TS_EPS_S:
            self._count_duplicate += 1
            self._integrate_status = (
                "generation_reseed" if generation_changed else "duplicate"
            )
            self._last_dt_s = 0.0
            self._last_yaw_rate_world_dps = 0.0
            return 0.0

        return self._apply_producer_delta(
            delta_unscaled=delta_unscaled,
            packets_delta=(
                packets_delta
                if packets_delta > 0
                else (1 if generation_changed else 0)
            ),
            roll_acc=roll_acc,
            pitch_acc=pitch_acc,
            gx=gx,
            gz=gz,
            yaw_rate_world_dps=yaw_rate_world_dps,
        )
