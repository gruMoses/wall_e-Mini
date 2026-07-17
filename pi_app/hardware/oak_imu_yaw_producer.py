"""
Lossless OAK BMI270 yaw integration (producer side).

Problem
-------
The shared OakDepthReader vision loop drains the IMU queue at up to the
configured poll rate, but historically kept only the newest packet
(``latest`` mode) or a small tail (``bounded``). OakImuReader then
reconstructed heading from sparse snapshots. When consecutive samples
were >~0.15 s apart, integration gap-froze and lost rotation — full-service
chalk under-report at scale=1 while a light dedicated pipeline was accurate.

Solution
--------
Integrate **every** drained packet in device-timestamp order into cumulative
free-yaw channels (body X/Y/Z + gravity projection). Consumers read the
cumulative integrals and apply scale **once**; they never re-integrate gyro×dt
from snapshots (no double-integrate).

Safety
------
- Duplicate / zero / regressed device timestamps reseed without jumps.
- Gaps larger than ``max_integrate_dt_s`` freeze (no huge phantom steps).
- Invalid timestamps fall back to host sample identity with the same rules.
- Reconnect bumps ``generation`` so consumers reseed without a heading step.
- Memory is O(cap) per drain for sort then O(1) fold; the packet cap aligns
  with the host message buffer (512 at ~1 pkt/msg) so a full host backlog is
  not silently halved. Soft backlog drop of oldest samples is last-resort only.
- Bias and NMNI are applied at integrate time when configured; scale is not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

G_MSS = 9.80665

# Default caps (match OakImuReader hardening).
_MAX_INTEGRATE_DT_S = 0.15
_TS_EPS_S = 1e-9
# Packet cap per drain (packets, not host-queue messages). Aligns with
# oak_depth.IMU_HOST_QUEUE_MAX_SIZE (512) at batch≈1 so a full host backlog
# integrates without silent half-drop. One queue message may hold multiple
# packets — this is a packet budget, not a 5.1 s wall-clock guarantee.
# Keeps sort+fold CPU/memory bounded. Raise only with measured need.
_DEFAULT_MAX_PACKETS_PER_DRAIN = 512


@dataclass(frozen=True)
class ImuPacket:
    """One BMI270 sample (rates in rad/s, accel in m/s²).

    ``device_ts_s`` / ``host_ts_s`` use NaN as the "missing" sentinel so a real
    device clock value of 0.0 (session start) remains integrable.
    """

    device_ts_s: float = float("nan")
    host_ts_s: float = float("nan")
    gx_rads: float = 0.0
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    ax_mss: float = 0.0
    ay_mss: float = 0.0
    az_mss: float = 0.0


@dataclass
class ImuYawProducerSnapshot:
    """Thread-safe-friendly snapshot of producer state for consumers."""

    # Latest **raw** sample (body frame). NOT bias-subtracted, NOT NMNI-gated.
    # Gyro calibration and OakImuReader body-rate diagnostics rely on this.
    # Integrate-path rates (bias then NMNI) exist only as locals inside ingest.
    ax_mss: float = 0.0
    ay_mss: float = 0.0
    az_mss: float = 0.0
    gx_rads: float = 0.0
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    timestamp: float = 0.0  # host receive time of latest sample
    device_timestamp_s: float = 0.0

    # Unscaled cumulative free yaw (rad) — integral of bias/NMNI-corrected rates.
    cum_yaw_x_rad: float = 0.0
    cum_yaw_y_rad: float = 0.0
    cum_yaw_z_rad: float = 0.0
    cum_yaw_grav_rad: float = 0.0

    # Device time of last successfully integrated sample (0 if none).
    last_integrated_device_ts_s: float = 0.0
    last_integrated_host_ts_s: float = 0.0
    # Monotonic total device-time integrated (for bias diagnostics).
    integrated_time_s: float = 0.0
    # Bumps on regressed timestamps / explicit reset so consumers reseed.
    generation: int = 0

    # Counters
    packets_received: int = 0
    packets_integrated: int = 0
    packets_duplicate: int = 0
    packets_regressed: int = 0
    packets_restart: int = 0
    packets_gap_freeze: int = 0
    packets_invalid_ts: int = 0
    packets_backlog_dropped: int = 0
    last_batch_packets: int = 0
    last_dt_s: float = 0.0
    last_status: str = "init"
    cadence_samples: int = 0
    cadence_last_s: float = 0.0
    cadence_min_s: float = 0.0
    cadence_max_s: float = 0.0
    cadence_avg_s: float = 0.0


@dataclass
class ImuYawProducer:
    """Fold every packet into cumulative free-yaw channels (unscaled)."""

    max_integrate_dt_s: float = _MAX_INTEGRATE_DT_S
    max_packets_per_drain: int = _DEFAULT_MAX_PACKETS_PER_DRAIN
    # Bias in rad/s (body axes), applied before integration.
    bias_gx_rads: float = 0.0
    bias_gy_rads: float = 0.0
    bias_gz_rads: float = 0.0
    nmni_enabled: bool = False
    nmni_threshold_dps: float = 0.3
    # Accel EMA for gravity projection (same spirit as OakImuReader).
    accel_ema_alpha: float = 0.12

    _ax_ema: Optional[float] = field(default=None, init=False, repr=False)
    _ay_ema: Optional[float] = field(default=None, init=False, repr=False)
    _az_ema: Optional[float] = field(default=None, init=False, repr=False)

    # Cumulative free yaw (rad), unscaled.
    cum_x_rad: float = field(default=0.0, init=False)
    cum_y_rad: float = field(default=0.0, init=False)
    cum_z_rad: float = field(default=0.0, init=False)
    cum_grav_rad: float = field(default=0.0, init=False)

    last_device_ts_s: Optional[float] = field(default=None, init=False)
    last_host_ts_s: Optional[float] = field(default=None, init=False)
    integrated_time_s: float = field(default=0.0, init=False)
    generation: int = field(default=0, init=False)

    # Latest sample
    ax_mss: float = field(default=0.0, init=False)
    ay_mss: float = field(default=0.0, init=False)
    az_mss: float = field(default=0.0, init=False)
    gx_rads: float = field(default=0.0, init=False)
    gy_rads: float = field(default=0.0, init=False)
    gz_rads: float = field(default=0.0, init=False)
    timestamp: float = field(default=0.0, init=False)
    device_timestamp_s: float = field(default=0.0, init=False)

    packets_received: int = field(default=0, init=False)
    packets_integrated: int = field(default=0, init=False)
    packets_duplicate: int = field(default=0, init=False)
    packets_regressed: int = field(default=0, init=False)
    packets_restart: int = field(default=0, init=False)
    packets_gap_freeze: int = field(default=0, init=False)
    packets_invalid_ts: int = field(default=0, init=False)
    packets_backlog_dropped: int = field(default=0, init=False)
    last_batch_packets: int = field(default=0, init=False)
    last_dt_s: float = field(default=0.0, init=False)
    last_status: str = field(default="init", init=False)

    cadence_samples: int = field(default=0, init=False)
    cadence_last_s: float = field(default=0.0, init=False)
    cadence_min_s: float = field(default=0.0, init=False)
    cadence_max_s: float = field(default=0.0, init=False)
    cadence_avg_s: float = field(default=0.0, init=False)

    def set_gyro_bias_rads(self, gx: float, gy: float, gz: float) -> None:
        self.bias_gx_rads = float(gx)
        self.bias_gy_rads = float(gy)
        self.bias_gz_rads = float(gz)

    def set_gyro_bias_dps(self, gx_dps: float, gy_dps: float, gz_dps: float) -> None:
        self.set_gyro_bias_rads(
            math.radians(float(gx_dps)),
            math.radians(float(gy_dps)),
            math.radians(float(gz_dps)),
        )

    def set_nmni(self, enabled: bool, threshold_dps: float = 0.3) -> None:
        self.nmni_enabled = bool(enabled)
        self.nmni_threshold_dps = float(threshold_dps)

    def note_pipeline_restart(self) -> None:
        """Call when the OAK session rebuilds so the next clock is a clean reseed."""
        self.generation += 1
        self.last_device_ts_s = None
        self.last_host_ts_s = None
        self.last_status = "pipeline_restart"
        self.packets_restart += 1

    def snapshot(self) -> ImuYawProducerSnapshot:
        return ImuYawProducerSnapshot(
            ax_mss=self.ax_mss,
            ay_mss=self.ay_mss,
            az_mss=self.az_mss,
            gx_rads=self.gx_rads,
            gy_rads=self.gy_rads,
            gz_rads=self.gz_rads,
            timestamp=self.timestamp,
            device_timestamp_s=self.device_timestamp_s,
            cum_yaw_x_rad=self.cum_x_rad,
            cum_yaw_y_rad=self.cum_y_rad,
            cum_yaw_z_rad=self.cum_z_rad,
            cum_yaw_grav_rad=self.cum_grav_rad,
            last_integrated_device_ts_s=float(self.last_device_ts_s or 0.0),
            last_integrated_host_ts_s=float(self.last_host_ts_s or 0.0),
            integrated_time_s=self.integrated_time_s,
            generation=self.generation,
            packets_received=self.packets_received,
            packets_integrated=self.packets_integrated,
            packets_duplicate=self.packets_duplicate,
            packets_regressed=self.packets_regressed,
            packets_restart=self.packets_restart,
            packets_gap_freeze=self.packets_gap_freeze,
            packets_invalid_ts=self.packets_invalid_ts,
            packets_backlog_dropped=self.packets_backlog_dropped,
            last_batch_packets=self.last_batch_packets,
            last_dt_s=self.last_dt_s,
            last_status=self.last_status,
            cadence_samples=self.cadence_samples,
            cadence_last_s=self.cadence_last_s,
            cadence_min_s=self.cadence_min_s,
            cadence_max_s=self.cadence_max_s,
            cadence_avg_s=self.cadence_avg_s,
        )

    @staticmethod
    def _ts_valid(ts: float) -> bool:
        return ts is not None and math.isfinite(float(ts))

    @staticmethod
    def sort_packets(packets: Sequence[ImuPacket]) -> List[ImuPacket]:
        """Order by device timestamp when valid; preserve relative order otherwise."""

        def key(item: Tuple[int, ImuPacket]) -> Tuple[int, float, int]:
            idx, pkt = item
            if ImuYawProducer._ts_valid(pkt.device_ts_s):
                return (0, float(pkt.device_ts_s), idx)
            host = float(pkt.host_ts_s) if ImuYawProducer._ts_valid(pkt.host_ts_s) else 0.0
            return (1, host, idx)

        indexed = list(enumerate(packets))
        indexed.sort(key=key)
        return [p for _, p in indexed]

    def ingest(self, packets: Iterable[ImuPacket], *, host_now_s: Optional[float] = None) -> ImuYawProducerSnapshot:
        """Process a drained batch: all packets, timestamp order, O(cap) sort memory.

        Default ``max_packets_per_drain`` matches the host message buffer so a
        full configured backlog is integrated without silent truncation. Soft
        drop of the oldest samples only fires above that cap (last-resort CPU
        bound). Dropped samples are counted in ``packets_backlog_dropped`` —
        they never counted as "received" here; the host-side metrics counter
        records the full drain size separately.
        """
        batch = list(packets)
        self.last_batch_packets = len(batch)
        if not batch:
            return self.snapshot()

        cap = max(1, int(self.max_packets_per_drain))
        if len(batch) > cap:
            dropped = len(batch) - cap
            self.packets_backlog_dropped += dropped
            # Keep the newest by sort order (last after sort).
            ordered = self.sort_packets(batch)
            batch = ordered[-cap:]
        else:
            batch = self.sort_packets(batch)

        for pkt in batch:
            self.packets_received += 1
            self._ingest_one(pkt, host_now_s=host_now_s)

        return self.snapshot()

    def _seed_clocks(self, dev_ts: float, host_ts: float) -> None:
        if self._ts_valid(dev_ts):
            self.last_device_ts_s = float(dev_ts)
        if self._ts_valid(host_ts):
            self.last_host_ts_s = float(host_ts)

    def _resolve_dt(self, dev_ts: float, host_ts: float) -> Tuple[Optional[float], str]:
        max_dt = max(0.02, float(self.max_integrate_dt_s))

        if self._ts_valid(dev_ts):
            last = self.last_device_ts_s
            if last is None:
                self._seed_clocks(dev_ts, host_ts)
                self.packets_restart += 1
                self.generation += 1 if self.packets_integrated > 0 else 0
                return None, "restart"
            delta = float(dev_ts) - float(last)
            if delta < -_TS_EPS_S:
                self.packets_regressed += 1
                self.packets_restart += 1
                self.generation += 1
                self._seed_clocks(dev_ts, host_ts)
                return None, "regressed"
            if abs(delta) <= _TS_EPS_S:
                self.packets_duplicate += 1
                return None, "duplicate"
            if delta > max_dt:
                self.packets_gap_freeze += 1
                self._seed_clocks(dev_ts, host_ts)
                return None, "gap_freeze"
            self._seed_clocks(dev_ts, host_ts)
            return float(delta), "fresh"

        # Invalid / missing device timestamp path.
        self.packets_invalid_ts += 1
        if not self._ts_valid(host_ts):
            return None, "invalid_ts"
        last_h = self.last_host_ts_s
        if last_h is None:
            self._seed_clocks(float("nan"), host_ts)
            self.packets_restart += 1
            return None, "restart"
        delta_h = float(host_ts) - float(last_h)
        if delta_h < -_TS_EPS_S:
            self.packets_regressed += 1
            self.packets_restart += 1
            self.generation += 1
            self._seed_clocks(float("nan"), host_ts)
            return None, "regressed"
        if abs(delta_h) <= _TS_EPS_S:
            self.packets_duplicate += 1
            return None, "duplicate"
        if delta_h > max_dt:
            self.packets_gap_freeze += 1
            self._seed_clocks(float("nan"), host_ts)
            return None, "gap_freeze"
        self._seed_clocks(float("nan"), host_ts)
        return float(delta_h), "fresh"

    def _apply_nmni(self, rate_rads: float) -> float:
        if not self.nmni_enabled:
            return rate_rads
        if abs(math.degrees(rate_rads)) < self.nmni_threshold_dps:
            return 0.0
        return rate_rads

    def _update_accel_ema(self, ax_mss: float, ay_mss: float, az_mss: float) -> Tuple[float, float, float]:
        ax_g = ax_mss / G_MSS
        ay_g = ay_mss / G_MSS
        az_g = az_mss / G_MSS
        a = self.accel_ema_alpha
        if self._ax_ema is None:
            self._ax_ema, self._ay_ema, self._az_ema = ax_g, ay_g, az_g
        else:
            self._ax_ema = a * ax_g + (1.0 - a) * self._ax_ema
            self._ay_ema = a * ay_g + (1.0 - a) * self._ay_ema
            self._az_ema = a * az_g + (1.0 - a) * self._az_ema
        return self._ax_ema, self._ay_ema, self._az_ema

    def _gravity_rate(self, gx: float, gy: float, gz: float, sx: float, sy: float, sz: float) -> float:
        a_norm_sq = sx * sx + sy * sy + sz * sz
        if a_norm_sq > 0.25:
            return (gx * sx + gy * sy + gz * sz) / a_norm_sq
        return gy

    def _note_cadence(self, dt: float) -> None:
        self.cadence_last_s = dt
        if self.cadence_samples == 0:
            self.cadence_min_s = dt
            self.cadence_max_s = dt
            self.cadence_avg_s = dt
        else:
            if dt < self.cadence_min_s:
                self.cadence_min_s = dt
            if dt > self.cadence_max_s:
                self.cadence_max_s = dt
            n = self.cadence_samples
            self.cadence_avg_s += (dt - self.cadence_avg_s) / (n + 1)
        self.cadence_samples += 1

    def _ingest_one(self, pkt: ImuPacket, *, host_now_s: Optional[float]) -> None:
        if self._ts_valid(pkt.host_ts_s):
            host_ts = float(pkt.host_ts_s)
        elif host_now_s is not None and math.isfinite(float(host_now_s)):
            host_ts = float(host_now_s)
        else:
            host_ts = float("nan")
        dev_ts = float(pkt.device_ts_s) if self._ts_valid(pkt.device_ts_s) else float("nan")

        # Always publish latest **raw** snapshot (calibration / roll-pitch /
        # body-rate diagnostics). Bias and NMNI must NOT be written back here —
        # otherwise OakImuReader.calibrate_gyro would average already-gated
        # rates and sub-threshold residual bias would collapse to exact zero.
        self.ax_mss = float(pkt.ax_mss)
        self.ay_mss = float(pkt.ay_mss)
        self.az_mss = float(pkt.az_mss)
        self.gx_rads = float(pkt.gx_rads)
        self.gy_rads = float(pkt.gy_rads)
        self.gz_rads = float(pkt.gz_rads)
        self.timestamp = host_ts if self._ts_valid(host_ts) else float(host_now_s or 0.0)
        self.device_timestamp_s = dev_ts if self._ts_valid(dev_ts) else 0.0

        dt, status = self._resolve_dt(dev_ts, host_ts)
        self.last_status = status
        if dt is None or status != "fresh":
            self.last_dt_s = 0.0
            # Still update accel EMA so gravity projection recovers after gaps.
            self._update_accel_ema(self.ax_mss, self.ay_mss, self.az_mss)
            return

        self.last_dt_s = dt
        self._note_cadence(dt)

        # Integrate-path only (locals): bias then NMNI. Never overwrite gx_rads.
        gx = float(pkt.gx_rads) - self.bias_gx_rads
        gy = float(pkt.gy_rads) - self.bias_gy_rads
        gz = float(pkt.gz_rads) - self.bias_gz_rads
        gx = self._apply_nmni(gx)
        gy = self._apply_nmni(gy)
        gz = self._apply_nmni(gz)

        sx, sy, sz = self._update_accel_ema(self.ax_mss, self.ay_mss, self.az_mss)
        g_rate = self._apply_nmni(self._gravity_rate(gx, gy, gz, sx, sy, sz))

        self.cum_x_rad += gx * dt
        self.cum_y_rad += gy * dt
        self.cum_z_rad += gz * dt
        self.cum_grav_rad += g_rate * dt
        self.integrated_time_s += dt
        self.packets_integrated += 1


def sparse_snapshot_integrate_yaw_deg(
    samples: Sequence[Tuple[float, float]],
    *,
    max_dt_s: float = _MAX_INTEGRATE_DT_S,
) -> float:
    """Reproduce the OLD undercount path: integrate only successive sparse snapshots.

    ``samples`` is a sequence of (device_ts_s, gy_dps) as a late consumer would
    observe if the producer kept only the newest packet of each drain batch.
    """
    yaw_deg = 0.0
    last_ts: Optional[float] = None
    for ts, gy_dps in samples:
        if last_ts is None:
            last_ts = ts
            continue
        dt = ts - last_ts
        last_ts = ts
        if dt <= 0.0 or dt > max_dt_s:
            continue
        yaw_deg += gy_dps * dt
    return yaw_deg
