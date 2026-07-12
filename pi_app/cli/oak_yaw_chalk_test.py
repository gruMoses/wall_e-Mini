#!/usr/bin/env python3
"""
Safe, disarmed OAK-D yaw chalk test harness.

Does NOT drive motors and does NOT write config. Integrates the body gyro triad
(and the configured production path) while you rotate the robot on chalk marks
so field data chooses axis/sign/scale — never guess production settings here.

Usage (on the Pi, with wall-e service stopped so the camera is free):

    sudo systemctl stop wall-e   # or whatever the unit is named
    python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90
    python3 -m pi_app.cli.oak_yaw_chalk_test --expected 180 --stream
    python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --no-nmni --stream

Procedure:
  1. Park robot on flat ground, mark chalk 0° reference on floor + chassis.
  2. Start this tool; wait for "READY" (gyro bias collected, samples flowing).
  3. Press Enter to MARK START, then rotate slowly/precisely to the chalk target
     (90° or 180° clockwise looking down — note your convention).
     A clean 90° turn should take roughly 5–10 seconds. The camera must rotate
     rigidly with the chassis (no flex mount / relative slip).
  4. Press Enter to MARK END.
  5. Read the report: pick the axis with |delta| closest to expected and stable
     sign; compute scale = expected / measured (scale=1 integration).

CLI extras (no config write):
  --nmni / --no-nmni   force NMNI on/off for this run; report source; keep defaults
  --expected 0         stationary check; recommended scale is nan (no div-by-zero)
  --bias-s 10          longer bias collection (safe with expected 0 / stream)

Pass criteria (document in docs/heading_tuning.md): see printed PASS/FAIL.

Baseline sync
-------------
Producer cumulative yaw advances continuously on the device thread. The harness
must poll while waiting for MARK START / MARK END (stream and non-stream) and
capture producer / triad / production baselines from the **same** poll so
start_prod cannot refresh ahead of a stale TriadIntegrator start.
"""

from __future__ import annotations

import argparse
import math
import select
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, TextIO, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config  # type: ignore
from pi_app.hardware.oak_depth import OakDepthReader  # type: ignore
from pi_app.hardware.oak_imu import OakImuReader  # type: ignore


# ---------------------------------------------------------------------------
# Pure helpers (unit-tested; no hardware)
# ---------------------------------------------------------------------------


def _unwrap_delta_deg(start: float, end: float) -> float:
    """Continuous free-angle delta (start/end are unwrapped accumulators)."""
    return float(end) - float(start)


def _mod360(x: float) -> float:
    return (x + 360.0) % 360.0


def _pass_band(measured: float, expected: float, abs_tol_deg: float, rel_tol: float) -> bool:
    """Pass when |measured| is within abs/rel of |expected|.

    expected==0: pass only if |measured| <= abs_tol_deg (no relative term).
    """
    if abs(expected) < 1e-12:
        return abs(measured) <= abs_tol_deg
    err = abs(abs(measured) - abs(expected))
    return err <= max(abs_tol_deg, abs(expected) * rel_tol)


def recommended_scale(
    measured: float,
    expected: float,
    current_scale: float = 1.0,
    *,
    production: bool = False,
) -> float:
    """Magnitude fit scale. Never divides by zero (returns nan)."""
    if abs(measured) < 1e-9:
        return float("nan")
    if abs(expected) < 1e-12:
        # Stationary / expected-zero: no meaningful scale fit.
        return float("nan")
    if production:
        return float(current_scale) * (abs(expected) / abs(measured))
    return float(expected) / float(measured)


def format_bias_dps(bias: Sequence[float]) -> str:
    bx, by, bz = (float(bias[0]), float(bias[1]), float(bias[2]))
    return f"x={bx:.6f} y={by:.6f} z={bz:.6f}"


def resolve_nmni_enabled(
    cli_nmni: Optional[bool],
    config_enabled: bool,
) -> Tuple[bool, str]:
    """Return (enabled, source_label). CLI overrides config without writing it."""
    if cli_nmni is None:
        return bool(config_enabled), "config"
    return bool(cli_nmni), "cli"


def add_nmni_cli_args(ap: argparse.ArgumentParser) -> None:
    g = ap.add_mutually_exclusive_group()
    g.add_argument(
        "--nmni",
        dest="nmni",
        action="store_true",
        default=None,
        help="Force NMNI on for this run (does not write config)",
    )
    g.add_argument(
        "--no-nmni",
        dest="nmni",
        action="store_false",
        help="Force NMNI off for this run (does not write config)",
    )


def parse_chalk_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="OAK-D yaw chalk test (disarmed, no config write)")
    ap.add_argument("--expected", type=float, default=90.0,
                    help="Chalk turn magnitude in degrees (90 or 180 typical; 0 = stationary)")
    ap.add_argument("--direction", choices=("cw", "ccw"), default="cw",
                    help="Physical turn direction looking down (for report only)")
    ap.add_argument("--bias-s", type=float, default=2.0, help="Stationary gyro bias seconds")
    ap.add_argument("--rate-hz", type=float, default=50.0, help="Poll rate during test")
    ap.add_argument("--stream", action="store_true", help="Print live heading/triad while waiting")
    ap.add_argument("--abs-tol-deg", type=float, default=8.0,
                    help="Absolute pass tolerance on |measured| vs |expected|")
    ap.add_argument("--rel-tol", type=float, default=0.10,
                    help="Relative pass tolerance (fraction of expected)")
    ap.add_argument(
        "--production-source",
        default=None,
        help="Override production OakImuReader source for comparison (default: config)",
    )
    ap.add_argument(
        "--production-scale",
        type=float,
        default=None,
        help="Override production scale for comparison (default: config; use 1.0 to compare raw)",
    )
    add_nmni_cli_args(ap)
    return ap.parse_args(list(argv) if argv is not None else None)


# ---------------------------------------------------------------------------
# Integrators / stats
# ---------------------------------------------------------------------------


class TriadIntegrator:
    """Body-axis free-yaw at scale=1 for chalk evidence.

    Prefer producer cumulative channels (lossless, every BMI270 packet). Fall
    back to host-side sample integration only if producer cum is unavailable.
    """

    def __init__(self) -> None:
        self.x_deg = 0.0
        self.y_deg = 0.0
        self.z_deg = 0.0
        self.last_dev_ts: Optional[float] = None
        self.last_host_ts: Optional[float] = None
        self.samples = 0
        self.skipped = 0
        self._baseline_set = False
        self._base_x = 0.0
        self._base_y = 0.0
        self._base_z = 0.0
        self.using_producer = False
        self.last_cum_x: Optional[float] = None
        self.last_cum_y: Optional[float] = None
        self.last_cum_z: Optional[float] = None
        self.last_packets_integrated: int = 0

    def reset_baseline_from_producer(
        self,
        cum_x_deg: float,
        cum_y_deg: float,
        cum_z_deg: float,
        packets_integrated: int,
    ) -> None:
        """Re-anchor triad at current producer cum (atomic mark baseline)."""
        self.using_producer = True
        self._base_x = float(cum_x_deg)
        self._base_y = float(cum_y_deg)
        self._base_z = float(cum_z_deg)
        self._baseline_set = True
        self.x_deg = 0.0
        self.y_deg = 0.0
        self.z_deg = 0.0
        self.samples = int(packets_integrated)
        self.last_cum_x = float(cum_x_deg)
        self.last_cum_y = float(cum_y_deg)
        self.last_cum_z = float(cum_z_deg)
        self.last_packets_integrated = int(packets_integrated)

    def update_from_producer(
        self,
        cum_x_deg: float,
        cum_y_deg: float,
        cum_z_deg: float,
        packets_integrated: int,
    ) -> None:
        """Mirror producer unscaled free-yaw (relative to first observation)."""
        self.using_producer = True
        self.last_cum_x = float(cum_x_deg)
        self.last_cum_y = float(cum_y_deg)
        self.last_cum_z = float(cum_z_deg)
        self.last_packets_integrated = int(packets_integrated)
        if not self._baseline_set:
            self._base_x = float(cum_x_deg)
            self._base_y = float(cum_y_deg)
            self._base_z = float(cum_z_deg)
            self._baseline_set = True
            self.samples = int(packets_integrated)
            self.x_deg = 0.0
            self.y_deg = 0.0
            self.z_deg = 0.0
            return
        self.x_deg = float(cum_x_deg) - self._base_x
        self.y_deg = float(cum_y_deg) - self._base_y
        self.z_deg = float(cum_z_deg) - self._base_z
        self.samples = int(packets_integrated)

    def update(self, gx_dps: float, gy_dps: float, gz_dps: float,
               dev_ts: float, host_ts: float, age_s: float,
               max_dt: float = 0.15, stale_s: float = 0.5) -> None:
        if self.using_producer:
            return
        if age_s > stale_s or not math.isfinite(age_s):
            self.skipped += 1
            return
        dt: Optional[float] = None
        if dev_ts > 0.0:
            if self.last_dev_ts is None:
                self.last_dev_ts = dev_ts
                self.last_host_ts = host_ts
                self.skipped += 1
                return
            d = dev_ts - self.last_dev_ts
            if d <= 0.0 or d > max_dt:
                self.last_dev_ts = dev_ts
                self.last_host_ts = host_ts
                self.skipped += 1
                return
            dt = d
            self.last_dev_ts = dev_ts
            self.last_host_ts = host_ts
        else:
            if host_ts <= 0.0:
                self.skipped += 1
                return
            if self.last_host_ts is None:
                self.last_host_ts = host_ts
                self.skipped += 1
                return
            d = host_ts - self.last_host_ts
            if d <= 0.0 or d > max_dt:
                self.last_host_ts = host_ts
                self.skipped += 1
                return
            dt = d
            self.last_host_ts = host_ts

        self.x_deg += gx_dps * dt
        self.y_deg += gy_dps * dt
        self.z_deg += gz_dps * dt
        self.samples += 1


@dataclass
class AxisRateStats:
    """Running stats for one body gyro axis (deg/s samples)."""

    values: List[float] = field(default_factory=list)

    def add(self, v: float) -> None:
        if math.isfinite(v):
            self.values.append(float(v))

    def summary(self) -> Dict[str, float]:
        vals = self.values
        n = len(vals)
        if n == 0:
            return {
                "count": 0.0,
                "mean": float("nan"),
                "std": float("nan"),
                "min": float("nan"),
                "max": float("nan"),
            }
        mean = statistics.fmean(vals)
        std = statistics.pstdev(vals) if n > 1 else 0.0
        return {
            "count": float(n),
            "mean": mean,
            "std": std,
            "min": min(vals),
            "max": max(vals),
        }


@dataclass
class RateWindow:
    """Per-axis raw rate stats + NMNI-below-threshold fraction for gyro_y."""

    x: AxisRateStats = field(default_factory=AxisRateStats)
    y: AxisRateStats = field(default_factory=AxisRateStats)
    z: AxisRateStats = field(default_factory=AxisRateStats)
    gy_below_nmni: int = 0
    gy_total: int = 0

    def add_sample(
        self,
        gx_dps: float,
        gy_dps: float,
        gz_dps: float,
        nmni_threshold_dps: float,
    ) -> None:
        self.x.add(gx_dps)
        self.y.add(gy_dps)
        self.z.add(gz_dps)
        if math.isfinite(gy_dps):
            self.gy_total += 1
            if abs(gy_dps) < float(nmni_threshold_dps):
                self.gy_below_nmni += 1

    def gy_below_nmni_fraction(self) -> float:
        if self.gy_total <= 0:
            return float("nan")
        return self.gy_below_nmni / float(self.gy_total)


@dataclass
class MarkSnapshot:
    """Atomic producer / triad / production baselines from one poll."""

    prod_heading_deg: float
    prod_free_yaw_deg: float
    triad_x_deg: float
    triad_y_deg: float
    triad_z_deg: float
    producer_cum_x_deg: Optional[float]
    producer_cum_y_deg: Optional[float]
    producer_cum_z_deg: Optional[float]
    producer_packets_integrated: Optional[int]
    yaw_generation: Optional[int]
    health: Dict


def poll_imu_once(
    imu: OakImuReader,
    triad: TriadIntegrator,
    reader: Optional[OakDepthReader] = None,
    rate_window: Optional[RateWindow] = None,
    nmni_threshold_dps: float = 0.3,
) -> Dict:
    """Single poll: advance production (imu.read) and triad together."""
    data = imu.read()
    health = imu.get_health()
    px = health.get("producer_cum_yaw_x_deg", data.get("producer_cum_yaw_x_deg"))
    py = health.get("producer_cum_yaw_y_deg", data.get("producer_cum_yaw_y_deg"))
    pz = health.get("producer_cum_yaw_z_deg", data.get("producer_cum_yaw_z_deg"))
    p_int = health.get("producer_packets_integrated", data.get("count_producer_packets"))
    if px is not None and py is not None and pz is not None:
        triad.update_from_producer(
            float(px), float(py), float(pz), int(p_int or 0)
        )
    else:
        gx = float(health.get("gx_body_dps", data.get("gx_body_dps", data.get("gx_dps", 0.0))))
        gy = float(health.get("gy_body_dps", data.get("gy_body_dps", data.get("gy_dps", 0.0))))
        gz = float(health.get("gz_body_dps", data.get("gz_body_dps", 0.0)))
        dev_ts = float(data.get("device_timestamp_s", 0.0) or 0.0)
        host_ts = float(health.get("last_host_sample_ts") or 0.0)
        age = float(health.get("sample_age_s", data.get("sample_age_s", 0.0)) or 0.0)
        if reader is not None:
            st, age2 = reader.get_imu_data()
            host_ts = float(getattr(st, "timestamp", 0.0) or host_ts)
            age = float(age2)
        triad.update(gx, gy, gz, dev_ts, host_ts, age)

    if rate_window is not None:
        gx_b = float(health.get("gx_body_dps", data.get("gx_body_dps", data.get("gx_dps", 0.0))) or 0.0)
        gy_b = float(health.get("gy_body_dps", data.get("gy_body_dps", data.get("gy_dps", 0.0))) or 0.0)
        gz_b = float(health.get("gz_body_dps", data.get("gz_body_dps", 0.0)) or 0.0)
        rate_window.add_sample(gx_b, gy_b, gz_b, nmni_threshold_dps)

    return data


def capture_mark_snapshot(
    imu: OakImuReader,
    triad: TriadIntegrator,
    poll_once: Callable[[], Dict],
    *,
    rebaseline_triad: bool = False,
) -> MarkSnapshot:
    """Atomically refresh production + triad, then snapshot all baselines.

    When ``rebaseline_triad`` is True (MARK START), re-anchor triad to the
    current producer cum so reported triad deltas start at 0 together with
    production free-yaw deltas from this same poll.
    """
    data = poll_once()
    health = imu.get_health()
    px = health.get("producer_cum_yaw_x_deg", data.get("producer_cum_yaw_x_deg"))
    py = health.get("producer_cum_yaw_y_deg", data.get("producer_cum_yaw_y_deg"))
    pz = health.get("producer_cum_yaw_z_deg", data.get("producer_cum_yaw_z_deg"))
    p_int = health.get("producer_packets_integrated", data.get("count_producer_packets"))
    gen = health.get("yaw_generation", data.get("yaw_generation"))

    if rebaseline_triad and px is not None and py is not None and pz is not None:
        triad.reset_baseline_from_producer(
            float(px), float(py), float(pz), int(p_int or 0)
        )

    return MarkSnapshot(
        prod_heading_deg=float(data["heading_deg"]),
        prod_free_yaw_deg=-math.degrees(imu.yaw_rad),
        triad_x_deg=float(triad.x_deg),
        triad_y_deg=float(triad.y_deg),
        triad_z_deg=float(triad.z_deg),
        producer_cum_x_deg=float(px) if px is not None else None,
        producer_cum_y_deg=float(py) if py is not None else None,
        producer_cum_z_deg=float(pz) if pz is not None else None,
        producer_packets_integrated=int(p_int) if p_int is not None else None,
        yaw_generation=int(gen) if gen is not None else None,
        health=dict(health),
    )


def wait_for_enter_with_poll(
    poll_once: Callable[[], Dict],
    period_s: float,
    *,
    stream: bool = False,
    stream_line: Optional[Callable[[Dict], str]] = None,
    stdin: Optional[TextIO] = None,
    select_fn: Optional[Callable] = None,
    sleep_fn: Callable[[float], None] = time.sleep,
    prompt: str = "Press Enter...",
    print_fn: Callable[..., None] = print,
) -> None:
    """Poll continuously until Enter (stream and non-stream).

    Non-stream previously blocked on ``input()`` without polling, so producer
    cum advanced while TriadIntegrator froze and a later start_prod refresh
    desynchronized baselines. Both modes now drain via poll_once.
    """
    in_stream = stdin if stdin is not None else sys.stdin
    do_select = select_fn if select_fn is not None else select.select
    period = max(1e-3, float(period_s))

    if stream:
        print_fn(prompt)
    else:
        print_fn(prompt, end="", flush=True)

    while True:
        data = poll_once()
        if stream and stream_line is not None:
            print_fn(f"\r{stream_line(data)}", end="", flush=True)
        ready = do_select([in_stream], [], [], 0 if stream else period)[0]
        if in_stream in ready or (hasattr(ready, "__iter__") and in_stream in list(ready)):
            try:
                in_stream.readline()
            except Exception:
                pass
            if stream:
                print_fn()
            else:
                print_fn()  # complete the prompt line
            return
        if stream:
            sleep_fn(period)


def production_matches_neg_producer_y(
    d_prod_free: float,
    d_producer_y: float,
    *,
    abs_tol_deg: float = 0.5,
    rel_tol: float = 0.02,
) -> bool:
    """Production free-yaw delta should be ≈ −producer gyro_y delta (scale=1)."""
    expected = -float(d_producer_y)
    err = abs(float(d_prod_free) - expected)
    band = max(abs_tol_deg, abs(expected) * rel_tol)
    return err <= band


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _fmt_axis_stats(name: str, summary: Dict[str, float]) -> str:
    if summary["count"] <= 0:
        return f"  {name}: count=0"
    return (
        f"  {name}: count={int(summary['count'])} "
        f"mean={summary['mean']:+.4f} std={summary['std']:.4f} "
        f"min={summary['min']:+.4f} max={summary['max']:+.4f}"
    )


def print_results(
    *,
    expected: float,
    direction: str,
    src: str,
    scale: float,
    triad: TriadIntegrator,
    start: MarkSnapshot,
    end: MarkSnapshot,
    d_prod_free: float,
    d_x: float,
    d_y: float,
    d_z: float,
    d_producer_y: Optional[float],
    rate_window: RateWindow,
    nmni_enabled: bool,
    nmni_threshold_dps: float,
    nmni_source: str,
    abs_tol_deg: float,
    rel_tol: float,
    print_fn: Callable[..., None] = print,
) -> None:
    end_health = end.health
    print_fn()
    print_fn("=== RESULTS (scale=1 triad; production uses configured scale) ===")
    print_fn(f"  direction note: physical {direction}, expected |Δ|={expected:.1f}°")
    print_fn(
        f"  NMNI: enabled={nmni_enabled} threshold={nmni_threshold_dps:.4f} dps "
        f"(source={nmni_source}; config defaults preserved)"
    )
    triad_src = "producer_cum" if triad.using_producer else "host_sparse_fallback"
    print_fn(
        f"  triad source: {triad_src}  samples/packets: {triad.samples}  "
        f"skipped: {triad.skipped}"
    )

    # Exact cumulative start/end
    print_fn("  exact cumulative (start → end):")
    print_fn(
        f"    production free-yaw: {start.prod_free_yaw_deg:+.6f}° → "
        f"{end.prod_free_yaw_deg:+.6f}°  Δ={d_prod_free:+.6f}°"
    )
    print_fn(
        f"    production heading:  {start.prod_heading_deg:7.2f}° → "
        f"{end.prod_heading_deg:7.2f}°"
    )
    print_fn(
        f"    triad x/y/z: ({start.triad_x_deg:+.6f},{start.triad_y_deg:+.6f},"
        f"{start.triad_z_deg:+.6f}) → "
        f"({end.triad_x_deg:+.6f},{end.triad_y_deg:+.6f},{end.triad_z_deg:+.6f})"
    )
    if start.producer_cum_y_deg is not None and end.producer_cum_y_deg is not None:
        print_fn(
            f"    producer cum x: {start.producer_cum_x_deg:+.6f}° → "
            f"{end.producer_cum_x_deg:+.6f}°"
        )
        print_fn(
            f"    producer cum y: {start.producer_cum_y_deg:+.6f}° → "
            f"{end.producer_cum_y_deg:+.6f}°  "
            f"Δ={_unwrap_delta_deg(start.producer_cum_y_deg, end.producer_cum_y_deg):+.6f}°"
        )
        print_fn(
            f"    producer cum z: {start.producer_cum_z_deg:+.6f}° → "
            f"{end.producer_cum_z_deg:+.6f}°"
        )
    if start.producer_packets_integrated is not None:
        print_fn(
            f"    producer packets integrated: {start.producer_packets_integrated} → "
            f"{end.producer_packets_integrated}"
        )

    print_fn(
        f"  oak_imu health: path={end_health.get('integration_path')} "
        f"status={end_health.get('integrate_status')} "
        f"dup={end_health.get('count_duplicate')} stale={end_health.get('count_stale')} "
        f"regress={end_health.get('count_regressed')} restart={end_health.get('count_restart')} "
        f"integrated={end_health.get('count_integrated')} "
        f"producer_pkts={end_health.get('count_producer_packets')}"
    )
    print_fn(
        "  producer metrics: "
        f"recv/drained={end_health.get('producer_packets_received')} "
        f"parsed={end_health.get('producer_packets_parsed')} "
        f"integrated={end_health.get('producer_packets_integrated')} "
        f"dup={end_health.get('producer_packets_duplicate')} "
        f"gap={end_health.get('producer_packets_gap_freeze')} "
        f"backlog_drop={end_health.get('producer_packets_backlog_dropped')} "
        f"cadence_avg={end_health.get('producer_cadence_avg_s')} "
        f"cadence_max={end_health.get('producer_cadence_max_s')} "
        f"batch={end_health.get('last_batch_packets')}"
    )
    print_fn(
        "  generation / restart / regression / gap / backlog: "
        f"yaw_gen={end_health.get('yaw_generation', end.yaw_generation)} "
        f"gen_change={end_health.get('count_generation_change')} "
        f"cum_reset={end_health.get('count_cum_reset')} "
        f"regress={end_health.get('count_regressed')} "
        f"restart={end_health.get('count_restart')} "
        f"gap_freeze={end_health.get('count_gap_freeze')} "
        f"producer_gap={end_health.get('producer_packets_gap_freeze')} "
        f"producer_backlog_drop={end_health.get('producer_packets_backlog_dropped')} "
        f"producer_restart={end_health.get('producer_packets_restart')} "
        f"producer_regress={end_health.get('producer_packets_regressed')}"
    )
    print_fn(
        "  drain-batch observability (NOT host-queue occupancy/overflow): "
        f"maxSize={end_health.get('host_queue_max_size')} "
        f"blocking={end_health.get('host_queue_blocking')} "
        f"pkt_cap={end_health.get('max_packets_per_drain')} "
        f"drain_hw={end_health.get('drain_batch_high_water_msgs')} "
        f"drain_large={end_health.get('drain_batch_large_events')} "
        f"drain_full={end_health.get('drain_batch_full_size_events')} "
        f"q_drop={end_health.get('queue_msgs_dropped')} "
        f"overwrite_obs={end_health.get('queue_msgs_overwrite_observable')} "
        f"selection_coalesced={end_health.get('producer_packets_coalesced')} "
        f"(structural only; not a loss proof)"
    )
    if end_health.get("oak_reconnect_count") is not None:
        print_fn(
            f"  oak reconnect_count={end_health.get('oak_reconnect_count')} "
            f"connected={end_health.get('oak_connected')}"
        )

    # Raw per-axis rate stats
    print_fn("  raw per-axis body rates (dps, bias-corrected path samples during turn):")
    print_fn(_fmt_axis_stats("gyro_x", rate_window.x.summary()))
    print_fn(_fmt_axis_stats("gyro_y", rate_window.y.summary()))
    print_fn(_fmt_axis_stats("gyro_z", rate_window.z.summary()))
    frac = rate_window.gy_below_nmni_fraction()
    frac_s = f"{frac:.4f}" if math.isfinite(frac) else "n/a"
    print_fn(
        f"  bias-corrected gyro_y fraction |rate| < NMNI threshold "
        f"({nmni_threshold_dps:.4f} dps): {frac_s} "
        f"({rate_window.gy_below_nmni}/{rate_window.gy_total})"
    )

    rows = [
        ("gyro_x (scale=1)", d_x, False),
        ("gyro_y (scale=1)", d_y, False),
        ("gyro_z (scale=1)", d_z, False),
        (f"production ({src}×{scale:.3f})", d_prod_free, True),
    ]
    best_name = None
    best_err = 1e9
    for name, meas, is_prod in rows:
        rec_scale = recommended_scale(
            meas, expected, scale, production=is_prod
        )
        ok = _pass_band(meas, expected, abs_tol_deg, rel_tol)
        flag = "PASS" if ok else "fail"
        rec_s = f"{rec_scale:6.3f}" if math.isfinite(rec_scale) else "   nan"
        print_fn(
            f"  {name:28s}  Δ={meas:+8.2f}°  |Δ|={abs(meas):7.2f}°  "
            f"rec_scale≈{rec_s}  [{flag}]"
        )
        err = abs(abs(meas) - abs(expected))
        if not is_prod and err < best_err:
            best_err = err
            best_name = name

    if d_producer_y is not None:
        match = production_matches_neg_producer_y(d_prod_free, d_producer_y)
        print_fn(
            f"  sync check: production Δ ({d_prod_free:+.4f}°) ≈ "
            f"−producer gyro_y Δ ({-d_producer_y:+.4f}°)  "
            f"[{'PASS' if match else 'fail'}]"
        )

    print_fn()
    print_fn("Interpretation:")
    print_fn(f"  - Best body axis for this mounting (by |Δ| vs {expected:.0f}°): {best_name}")
    print_fn("  - Production baseline (field-validated): gyro_y × 1.0 — never restore auto/0.46.")
    print_fn("  - Do NOT write config from this tool without review.")
    print_fn("  - A clean 90° chalk turn should take roughly 5–10 seconds;")
    print_fn("    the camera must rotate rigidly with the chassis (no mount flex).")
    print_fn("  - Packet-loss proof (truthful): integrated tracks received; backlog_drop=0;")
    print_fn("    cadence_avg≈0.01 s. Drain-batch high-water is messages drained per poll —")
    print_fn("    NOT host-queue occupancy. DepthAI nonblocking overwrite loss is not observable.")
    print_fn("    selection_coalesced=0 is structural only — not a manufactured loss proof.")
    print_fn("  - Sign: heading uses -yaw integration; if Δ sign is opposite your chalk CW/CCW")
    print_fn("    expectation, document it — do not flip sign in production without a full retest.")
    print_fn()
    print_fn("Pass criteria (single axis, scale=1):")
    if abs(expected) < 1e-12:
        print_fn(f"  expected 0°: |measured| <= {abs_tol_deg:.1f}°")
    else:
        print_fn(
            f"  | |measured| - {expected:.0f} | <= max({abs_tol_deg:.1f}°, "
            f"{rel_tol*100:.0f}% of expected)"
        )
    print_fn("  Loss proof = integrated/received + backlog/gap + drain-batch size, not coalesced.")
    print_fn("  Generation bumps preserve unread cum (incl. +turn/-turn back to ~0);")
    print_fn("  only integrated-counter rewind (true producer replacement) freezes heading.")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_chalk_args(argv)

    nmni_cfg = bool(getattr(config.imu_steering, "oak_nmni_enabled", True))
    nmni_threshold = float(getattr(config.imu_steering, "oak_nmni_threshold_dps", 0.3))
    nmni_enabled, nmni_source = resolve_nmni_enabled(args.nmni, nmni_cfg)

    print("=== OAK-D yaw chalk test (DISARMED / no motors / no config write) ===")
    print("Stop the wall-e service first if it owns the camera.")
    print(f"Expected turn: {args.expected:.1f}° {args.direction}")
    print(
        "Tip: a clean 90° turn should take roughly 5–10 seconds; "
        "camera must rotate rigidly with chassis."
    )
    print(
        f"NMNI: enabled={nmni_enabled} (source={nmni_source}; "
        f"config default={nmni_cfg}; not written)"
    )
    print()

    if not OakDepthReader.detect():
        print("ERROR: No OAK-D Lite found.")
        return 1

    reader = OakDepthReader(config.obstacle_avoidance, config.follow_me)
    reader.start()
    time.sleep(1.5)

    src = args.production_source or str(
        getattr(config.imu_steering, "oak_yaw_rate_source", "gyro_y")
    )
    scale = (
        float(args.production_scale)
        if args.production_scale is not None
        else float(getattr(config.imu_steering, "oak_yaw_rate_scale", 1.0))
    )
    imu = OakImuReader(
        reader,
        nmni_enabled=nmni_enabled,
        nmni_threshold_dps=nmni_threshold,
        bias_adapt_enabled=False,
        yaw_rate_source=src,
        yaw_rate_scale=scale,
        use_gravity_projected_yaw_rate=bool(
            getattr(config.imu_steering, "oak_use_gravity_projected_yaw_rate", False)
        ),
    )

    bias_s = max(0.0, float(args.bias_s))
    print(f"Collecting gyro bias for {bias_s:.1f}s — keep robot still...")
    bias = imu.calibrate_gyro(duration_s=bias_s if bias_s > 0.0 else 0.01)
    print(f"  bias dps: {format_bias_dps(bias)}")
    print(f"  production path under test: source={src!r} scale={scale:.4f}")
    print(f"  nmni_enabled={nmni_enabled} threshold={nmni_threshold:.4f} dps ({nmni_source})")
    print()

    triad = TriadIntegrator()
    period = 1.0 / max(1.0, float(args.rate_hz))
    rate_window = RateWindow()

    def poll_once() -> Dict:
        return poll_imu_once(
            imu,
            triad,
            reader=reader,
            rate_window=None,  # only accumulate during marked turn
            nmni_threshold_dps=nmni_threshold,
        )

    def poll_once_turn() -> Dict:
        return poll_imu_once(
            imu,
            triad,
            reader=reader,
            rate_window=rate_window,
            nmni_threshold_dps=nmni_threshold,
        )

    # Warm up integrators
    for _ in range(10):
        poll_once()
        time.sleep(period)

    print("READY. Align chalk marks (0°).")
    print("Press Enter to MARK START, then turn to chalk target.")
    print("Press Enter again to MARK END.")
    print(
        "(Polling continuously in both stream and non-stream modes so "
        "producer / triad / production stay synchronized.)"
    )
    if args.stream:
        print("(streaming live line until Enter)")

    try:
        def start_stream_line(d: Dict) -> str:
            h = imu.get_health()
            return (
                f" hdg={d['heading_deg']:7.2f}  "
                f"sel={h.get('yaw_rate_source_selected')}  "
                f"status={h.get('integrate_status')}  "
                f"age={float(h.get('sample_age_s') or 0):.3f}s  "
                f"triad=({triad.x_deg:+7.1f},{triad.y_deg:+7.1f},{triad.z_deg:+7.1f})  "
                f"dup={h.get('count_duplicate')} stale={h.get('count_stale')}   "
            )

        wait_for_enter_with_poll(
            poll_once,
            period,
            stream=bool(args.stream),
            stream_line=start_stream_line if args.stream else None,
            prompt="Press Enter for MARK START...",
        )
    except KeyboardInterrupt:
        print("\nAborted.")
        reader.stop()
        return 130

    # Atomic MARK START: one poll updates production + triad, then rebaseline triad
    # so start triad is (0,0,0) and production free-yaw is the mark reference.
    # Capture production free-yaw *before* rebaseline still from same poll —
    # capture_mark_snapshot does poll then optional rebaseline; production
    # already advanced in that poll and we record those values.
    start = capture_mark_snapshot(imu, triad, poll_once, rebaseline_triad=True)
    print(
        f"START  prod_hdg={start.prod_heading_deg:.2f}°  "
        f"prod_free={start.prod_free_yaw_deg:+.4f}°  "
        f"triad=({start.triad_x_deg:.4f},{start.triad_y_deg:.4f},{start.triad_z_deg:.4f})  "
        f"prod_cum_y="
        f"{start.producer_cum_y_deg if start.producer_cum_y_deg is not None else 'n/a'}  "
        f"sel={start.health.get('yaw_rate_source_selected')}"
    )
    print(
        "Turning... press Enter for MARK END when aligned on chalk "
        "(~5–10 s for a clean 90°; camera rigid with chassis)."
    )

    try:
        def end_stream_line(d: Dict) -> str:
            h = imu.get_health()
            return (
                f" hdg={d['heading_deg']:7.2f}  "
                f"Δprod={_mod360(d['heading_deg'] - start.prod_heading_deg):+7.2f}  "
                f"Δx={triad.x_deg - start.triad_x_deg:+7.1f} "
                f"Δy={triad.y_deg - start.triad_y_deg:+7.1f} "
                f"Δz={triad.z_deg - start.triad_z_deg:+7.1f}  "
                f"status={h.get('integrate_status')}   "
            )

        wait_for_enter_with_poll(
            poll_once_turn,
            period,
            stream=bool(args.stream),
            stream_line=end_stream_line if args.stream else None,
            prompt="Press Enter for MARK END...",
        )
    except KeyboardInterrupt:
        print("\nAborted.")
        reader.stop()
        return 130

    end = capture_mark_snapshot(imu, triad, poll_once_turn, rebaseline_triad=False)

    d_prod_free = _unwrap_delta_deg(start.prod_free_yaw_deg, end.prod_free_yaw_deg)
    d_x = _unwrap_delta_deg(start.triad_x_deg, end.triad_x_deg)
    d_y = _unwrap_delta_deg(start.triad_y_deg, end.triad_y_deg)
    d_z = _unwrap_delta_deg(start.triad_z_deg, end.triad_z_deg)
    d_producer_y: Optional[float] = None
    if start.producer_cum_y_deg is not None and end.producer_cum_y_deg is not None:
        d_producer_y = _unwrap_delta_deg(start.producer_cum_y_deg, end.producer_cum_y_deg)

    print_results(
        expected=float(args.expected),
        direction=str(args.direction),
        src=src,
        scale=scale,
        triad=triad,
        start=start,
        end=end,
        d_prod_free=d_prod_free,
        d_x=d_x,
        d_y=d_y,
        d_z=d_z,
        d_producer_y=d_producer_y,
        rate_window=rate_window,
        nmni_enabled=nmni_enabled,
        nmni_threshold_dps=nmni_threshold,
        nmni_source=nmni_source,
        abs_tol_deg=float(args.abs_tol_deg),
        rel_tol=float(args.rel_tol),
    )

    reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
