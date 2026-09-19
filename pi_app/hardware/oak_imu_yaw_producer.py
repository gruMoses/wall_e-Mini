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

Sign convention
---------------
``cum_y_rad`` and ``cum_grav_rad`` are both "rotation about the body-DOWN axis,
clockwise-positive viewed from above" — the canonical statement lives on
``OakImuReader.read``. ``gyro_y`` is already CW-positive (BMI270 +Y points
down); the gravity projection is negated to match. ``cum_x_rad`` / ``cum_z_rad``
are diagnostics whose sign depends on mounting.

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

Motion witness (stationary bias tracking / ZUPT)
-------------------------------------------------
An IMU cannot vouch for its own stillness: a genuine slow steady rotation (a
cross-slope creep, a gentle arc) looks exactly like quiet noise to the raw
gyro/accel window, so the stationary detector alone would freeze real rotation
as "bias". ``set_motion_witness`` accepts an independent wheels-stopped signal
(pushed by ``pi_app.app.main`` from VESC RPM / commanded drive bytes via
``pi_app.control.rpm_plausibility.wheels_stopped``). The robot is declared
stationary only when the window is quiet **and** a fresh witness says the
wheels are stopped; no witness, or a stale one, means never stationary — no
bias tracking, no ZUPT.
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Iterable, List, Optional, Sequence, Tuple

# The app installs no logging handler, so INFO is dropped: anything that must be
# visible in journalctl is logged at WARNING.
logger = logging.getLogger(__name__)

G_MSS = 9.80665

# At most one WARNING per event type in this window (ZUPT can chatter at a
# stop line; the counters carry the full history).
_ZUPT_LOG_MIN_INTERVAL_S = 10.0

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

    # Stationary detection / gyro-bias tracking / ZUPT
    stationary: bool = False
    zupt_active: bool = False
    zupt_engage_count: int = 0
    bias_updates: int = 0
    bias_gx_dps: float = 0.0
    bias_gy_dps: float = 0.0
    bias_gz_dps: float = 0.0
    window_gyro_std_dps: float = 0.0   # worst of the three axes
    window_accel_std_g: float = 0.0
    last_bias_update_host_ts: float = 0.0

    # Motion witness (wheels-stopped signal; see module docstring).
    witness_still: Optional[bool] = None
    witness_age_s: float = float("inf")


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

    # ── Stationary bias tracking / ZUPT ──────────────────────────────────
    # Both DEFAULT OFF so existing callers and unit tests keep the pure
    # integrate-everything behaviour; production turns them on from config.
    stationary_tracking_enabled: bool = False
    zupt_enabled: bool = False
    stationary_window_s: float = 1.0
    stationary_gyro_std_dps: float = 0.3
    stationary_accel_std_g: float = 0.03
    stationary_max_rate_dps: float = 5.0
    stationary_bias_tau_s: float = 15.0
    # Motion witness freshness bound (see module docstring). A witness older
    # than this is treated as absent: no witness, no stationary declaration.
    witness_timeout_s: float = 1.0

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

    # Stationary state / counters (see _update_stationary_window).
    stationary: bool = field(default=False, init=False)
    zupt_active: bool = field(default=False, init=False)
    zupt_engage_count: int = field(default=0, init=False)
    bias_updates: int = field(default=0, init=False)
    window_gyro_std_dps: float = field(default=0.0, init=False)
    window_accel_std_g: float = field(default=0.0, init=False)
    last_bias_update_host_ts: float = field(default=0.0, init=False)

    # Motion witness (wheels-stopped signal; see module docstring). None means
    # "never received one". host_ts is whatever clock the caller passes to
    # set_motion_witness (production: time.monotonic(), matching packet host
    # timestamps in oak_depth._poll_imu).
    witness_still: Optional[bool] = field(default=None, init=False)
    witness_host_ts: float = field(default=0.0, init=False)
    _last_idle_log_mono: Optional[float] = field(default=None, init=False, repr=False)

    # Rolling window of RAW samples: (t_s, gx_dps, gy_dps, gz_dps, a_norm_g).
    # Running sums keep mean/std O(1) per packet (no numpy on the Pi).
    _win: Deque[Tuple[float, float, float, float, float]] = field(
        default_factory=deque, init=False, repr=False
    )
    _win_clock_s: float = field(default=0.0, init=False, repr=False)
    _win_sum: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0], init=False, repr=False
    )
    _win_sumsq: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0], init=False, repr=False
    )
    _bias_at_last_log_dps: Tuple[float, float, float] = field(
        default=(0.0, 0.0, 0.0), init=False, repr=False
    )
    _last_zupt_log_mono: dict = field(default_factory=dict, init=False, repr=False)

    # ── Stationary bias tracking / ZUPT ──────────────────────────────────

    def configure_stationary_tracking(
        self,
        *,
        enabled: Optional[bool] = None,
        zupt_enabled: Optional[bool] = None,
        window_s: Optional[float] = None,
        gyro_std_dps: Optional[float] = None,
        accel_std_g: Optional[float] = None,
        max_rate_dps: Optional[float] = None,
        bias_tau_s: Optional[float] = None,
        witness_timeout_s: Optional[float] = None,
    ) -> None:
        """Set the stationary detector knobs (None leaves a knob unchanged)."""
        if enabled is not None:
            self.stationary_tracking_enabled = bool(enabled)
        if zupt_enabled is not None:
            self.zupt_enabled = bool(zupt_enabled)
        if window_s is not None:
            self.stationary_window_s = max(0.05, float(window_s))
        if gyro_std_dps is not None:
            self.stationary_gyro_std_dps = max(0.0, float(gyro_std_dps))
        if accel_std_g is not None:
            self.stationary_accel_std_g = max(0.0, float(accel_std_g))
        if max_rate_dps is not None:
            self.stationary_max_rate_dps = max(0.0, float(max_rate_dps))
        if bias_tau_s is not None:
            self.stationary_bias_tau_s = max(0.1, float(bias_tau_s))
        if witness_timeout_s is not None:
            self.witness_timeout_s = max(0.05, float(witness_timeout_s))

    def set_motion_witness(self, still: bool, host_ts: float) -> None:
        """Record the latest wheels-stopped witness (see module docstring).

        Called from ``OakDepthReader.set_motion_witness``, itself called from
        the main control loop with ``pi_app.control.rpm_plausibility.
        wheels_stopped()``. ``host_ts`` must be on the same clock as the
        ``host_ts_s`` passed to :meth:`ingest` (production: ``time.monotonic()``)
        so freshness comparisons are meaningful.
        """
        self.witness_still = bool(still)
        if math.isfinite(float(host_ts)):
            self.witness_host_ts = float(host_ts)

    def _witness_age_s(self, host_now: float) -> float:
        if not math.isfinite(host_now) or not math.isfinite(self.witness_host_ts):
            return float("inf")
        return host_now - self.witness_host_ts

    def _witness_ok(self, host_now: float) -> bool:
        """True only for a FRESH witness that says the wheels are stopped.

        No witness ever received, a stale one, or one that says the wheels are
        turning are all treated the same: not stationary. An IMU cannot vouch
        for its own stillness — see the module docstring.
        """
        if self.witness_still is not True:
            return False
        return self._witness_age_s(host_now) <= float(self.witness_timeout_s)

    def _check_witness_idle(self, host_now: float) -> None:
        """WARNING once per 60 s while tracking is enabled but the witness is stale/absent.

        Uses the same age computation as ``_witness_ok`` (relative to
        ``host_now``, the sample's own clock) so the message reflects real
        elapsed time even if packets keep arriving. The rate limit itself uses
        ``time.monotonic()`` — a different, wall-clock question ("have I
        logged this recently") from "how old is the witness".
        """
        if not self.stationary_tracking_enabled or not math.isfinite(host_now):
            return
        if self._witness_age_s(host_now) <= 5.0:
            return
        now_mono = time.monotonic()
        last = self._last_idle_log_mono
        if last is not None and (now_mono - last) < 60.0:
            return
        self._last_idle_log_mono = now_mono
        logger.warning(
            "stationary bias tracking idle: no motion witness (wheels-stopped "
            "signal) received"
        )

    def _reset_stationary_window(self) -> None:
        """Drop the window on any non-fresh packet (gap / reseed / regression).

        The window clock is the integrated device time, so a gap would otherwise
        leave two samples straddling it and report a bogus "full" window.
        """
        self._win.clear()
        self._win_sum = [0.0, 0.0, 0.0, 0.0]
        self._win_sumsq = [0.0, 0.0, 0.0, 0.0]
        self.stationary = False
        self._set_zupt_active(False)

    @staticmethod
    def _std(sum_v: float, sumsq_v: float, n: int) -> float:
        if n <= 1:
            return 0.0
        mean = sum_v / n
        var = (sumsq_v / n) - (mean * mean)
        return math.sqrt(var) if var > 0.0 else 0.0

    def _update_stationary_window(self, pkt: ImuPacket, dt: float) -> bool:
        """Push one RAW sample and return whether the WINDOW looks quiet.

        This is necessary but not sufficient for ``stationary`` — see
        ``_ingest_one``, which also requires a fresh wheels-stopped witness.

        Window-quiet iff the window is FULL **and** every raw gyro axis has a
        standard deviation below ``stationary_gyro_std_dps``, **and** the accel
        norm has a standard deviation below ``stationary_accel_std_g``, **and**
        every axis RAW mean has absolute value below ``stationary_max_rate_dps``.
        The std gates do the real work; the max-rate bound is an ABSOLUTE bound
        on the raw mean (not a residual against the current bias estimate) —
        residual-to-bias is chicken-and-egg: it can never learn a bias larger
        than the bound itself. A raw mean above the bound while the window is
        otherwise quiet is not bias (e.g. a genuine slow steady turn); a hot
        bias (measured up to ~1.3 deg/s on this unit) must stay learnable.
        """
        self._win_clock_s += dt
        t = self._win_clock_s

        gx_dps = math.degrees(float(pkt.gx_rads))
        gy_dps = math.degrees(float(pkt.gy_rads))
        gz_dps = math.degrees(float(pkt.gz_rads))
        a_norm_g = (
            math.sqrt(
                float(pkt.ax_mss) ** 2
                + float(pkt.ay_mss) ** 2
                + float(pkt.az_mss) ** 2
            )
            / G_MSS
        )

        sample = (t, gx_dps, gy_dps, gz_dps, a_norm_g)
        self._win.append(sample)
        for i, v in enumerate((gx_dps, gy_dps, gz_dps, a_norm_g)):
            self._win_sum[i] += v
            self._win_sumsq[i] += v * v

        window_s = max(0.05, float(self.stationary_window_s))
        # Keep the oldest sample that still spans the full window, so the span
        # settles just above window_s instead of always just under it.
        while len(self._win) > 2 and (t - self._win[1][0]) >= window_s:
            old = self._win.popleft()
            for i, v in enumerate(old[1:]):
                self._win_sum[i] -= v
                self._win_sumsq[i] -= v * v

        n = len(self._win)
        gyro_std = max(self._std(self._win_sum[i], self._win_sumsq[i], n) for i in range(3))
        accel_std = self._std(self._win_sum[3], self._win_sumsq[3], n)
        self.window_gyro_std_dps = gyro_std
        self.window_accel_std_g = accel_std

        span = t - self._win[0][0]
        if n < 3 or span < window_s:
            return False

        if gyro_std >= self.stationary_gyro_std_dps:
            return False
        if accel_std >= self.stationary_accel_std_g:
            return False

        for i in range(3):
            mean_i = self._win_sum[i] / n
            if abs(mean_i) >= self.stationary_max_rate_dps:
                return False
        return True

    def _update_bias_from_window(self, dt: float, host_ts: float) -> None:
        """Relax the bias estimate toward the stationary window mean.

        ``bias += (dt / tau) * (window_mean - bias)`` per packet, so the bias
        follows thermal drift (measured on the robot: -0.03 dps right after a
        good 3 s boot calibration, then -0.46 / -0.83 / -1.07 / -1.25 dps over
        the next 37 minutes) without chasing noise.
        """
        n = len(self._win)
        if n <= 0:
            return
        alpha = dt / max(0.1, float(self.stationary_bias_tau_s))
        alpha = max(0.0, min(1.0, alpha))
        if alpha <= 0.0:
            return
        bx = self.bias_gx_dps + alpha * ((self._win_sum[0] / n) - self.bias_gx_dps)
        by = self.bias_gy_dps + alpha * ((self._win_sum[1] / n) - self.bias_gy_dps)
        bz = self.bias_gz_dps + alpha * ((self._win_sum[2] / n) - self.bias_gz_dps)
        self.set_gyro_bias_dps(bx, by, bz)
        self.bias_updates += 1
        if math.isfinite(host_ts):
            self.last_bias_update_host_ts = float(host_ts)

    def _set_zupt_active(self, active: bool) -> None:
        if bool(active) == bool(self.zupt_active):
            return
        self.zupt_active = bool(active)
        if self.zupt_active:
            self.zupt_engage_count += 1
        self._log_zupt_event("zupt_engage" if self.zupt_active else "zupt_disengage")

    def _log_zupt_event(self, event: str) -> None:
        now = time.monotonic()
        last = self._last_zupt_log_mono.get(event)
        if last is not None and (now - last) < _ZUPT_LOG_MIN_INTERVAL_S:
            return
        self._last_zupt_log_mono[event] = now
        bx, by, bz = self.bias_gx_dps, self.bias_gy_dps, self.bias_gz_dps
        pbx, pby, pbz = self._bias_at_last_log_dps
        self._bias_at_last_log_dps = (bx, by, bz)
        logger.warning(
            "OAK IMU %s: bias=(%+.4f, %+.4f, %+.4f) dps "
            "delta_since_last_log=(%+.4f, %+.4f, %+.4f) dps "
            "bias_updates=%d zupt_engages=%d gyro_std=%.4f dps accel_std=%.4f g",
            event,
            bx, by, bz,
            bx - pbx, by - pby, bz - pbz,
            self.bias_updates,
            self.zupt_engage_count,
            self.window_gyro_std_dps,
            self.window_accel_std_g,
        )

    @property
    def bias_gx_dps(self) -> float:
        return math.degrees(self.bias_gx_rads)

    @property
    def bias_gy_dps(self) -> float:
        return math.degrees(self.bias_gy_rads)

    @property
    def bias_gz_dps(self) -> float:
        return math.degrees(self.bias_gz_rads)

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
            stationary=self.stationary,
            zupt_active=self.zupt_active,
            zupt_engage_count=self.zupt_engage_count,
            bias_updates=self.bias_updates,
            bias_gx_dps=self.bias_gx_dps,
            bias_gy_dps=self.bias_gy_dps,
            bias_gz_dps=self.bias_gz_dps,
            window_gyro_std_dps=self.window_gyro_std_dps,
            window_accel_std_g=self.window_accel_std_g,
            last_bias_update_host_ts=self.last_bias_update_host_ts,
            witness_still=self.witness_still,
            # Relative to the latest processed sample's own host clock (the
            # same domain _witness_ok uses per packet), not wall time.monotonic
            # — deterministic for tests and identical in production, since
            # both host_ts values come from the same clock there.
            witness_age_s=(
                self._witness_age_s(self.timestamp)
                if self.witness_still is not None
                else float("inf")
            ),
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
        """Gravity-projected yaw rate in the shared CW-positive convention.

        ``cum_grav_rad`` must share the convention of ``cum_y_rad`` so a
        consumer can switch channels without a sign flip: "rotation about the
        body-DOWN axis, clockwise-positive viewed from above" (the canonical
        statement lives on ``OakImuReader.read``).

        The accelerometer vector is specific force, which at rest points **UP**
        (about -Y in the OAK camera frame, where +Y is down). Projecting onto it
        is therefore counter-clockwise-positive, so the projection is negated.
        The ``gy`` fallback is already CW-positive and is returned unchanged.
        """
        a_norm_sq = sx * sx + sy * sy + sz * sz
        if a_norm_sq > 0.25:
            return -(gx * sx + gy * sy + gz * sz) / a_norm_sq
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

        self._check_witness_idle(host_ts)

        dt, status = self._resolve_dt(dev_ts, host_ts)
        self.last_status = status
        if dt is None or status != "fresh":
            self.last_dt_s = 0.0
            # A gap / reseed / regression breaks window continuity.
            if status in ("restart", "regressed", "gap_freeze", "invalid_ts"):
                self._reset_stationary_window()
            # Still update accel EMA so gravity projection recovers after gaps.
            self._update_accel_ema(self.ax_mss, self.ay_mss, self.az_mss)
            return

        self.last_dt_s = dt
        self._note_cadence(dt)

        # Stationary requires BOTH a quiet window (raw gyro/accel alone) AND a
        # fresh wheels-stopped witness — the window alone cannot tell a quiet
        # slow turn from bias (see module docstring / _witness_ok).
        window_quiet = self._update_stationary_window(pkt, dt)
        self.stationary = bool(window_quiet and self._witness_ok(host_ts))
        tracking = self.stationary and self.stationary_tracking_enabled
        if tracking:
            self._update_bias_from_window(dt, host_ts)
        freeze = tracking and self.zupt_enabled
        self._set_zupt_active(freeze)

        # Integrate-path only (locals): bias then NMNI. Never overwrite gx_rads.
        gx = float(pkt.gx_rads) - self.bias_gx_rads
        gy = float(pkt.gy_rads) - self.bias_gy_rads
        gz = float(pkt.gz_rads) - self.bias_gz_rads
        gx = self._apply_nmni(gx)
        gy = self._apply_nmni(gy)
        gz = self._apply_nmni(gz)

        sx, sy, sz = self._update_accel_ema(self.ax_mss, self.ay_mss, self.az_mss)
        g_rate = self._apply_nmni(self._gravity_rate(gx, gy, gz, sx, sy, sz))

        # ZUPT: the robot is provably still, so nothing is added to cum. The
        # packet counters still advance — they mean "packets the integrator
        # processed", and the consumer's producer-replacement detector relies on
        # that counter being monotonic.
        if not freeze:
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
