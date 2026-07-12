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

Procedure:
  1. Park robot on flat ground, mark chalk 0° reference on floor + chassis.
  2. Start this tool; wait for "READY" (gyro bias collected, samples flowing).
  3. Press Enter to MARK START, then rotate slowly/precisely to the chalk target
     (90° or 180° clockwise looking down — note your convention).
  4. Press Enter to MARK END.
  5. Read the report: pick the axis with |delta| closest to expected and stable
     sign; compute scale = expected / measured (scale=1 integration).

Pass criteria (document in docs/heading_tuning.md): see printed PASS/FAIL.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config  # type: ignore
from pi_app.hardware.oak_depth import OakDepthReader  # type: ignore
from pi_app.hardware.oak_imu import OakImuReader  # type: ignore


def _unwrap_delta_deg(start: float, end: float) -> float:
    """Signed shortest-path? No — for chalk we want continuous unwrapped delta
    from a single integration stream, so use raw difference of free angles.
    Here start/end are already free (not mod-360) accumulators.
    """
    return float(end) - float(start)


def _mod360(x: float) -> float:
    return (x + 360.0) % 360.0


def _pass_band(measured: float, expected: float, abs_tol_deg: float, rel_tol: float) -> bool:
    err = abs(abs(measured) - abs(expected))
    return err <= max(abs_tol_deg, abs(expected) * rel_tol)


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

    def update_from_producer(
        self,
        cum_x_deg: float,
        cum_y_deg: float,
        cum_z_deg: float,
        packets_integrated: int,
    ) -> None:
        """Mirror producer unscaled free-yaw (relative to first observation)."""
        self.using_producer = True
        if not self._baseline_set:
            self._base_x = cum_x_deg
            self._base_y = cum_y_deg
            self._base_z = cum_z_deg
            self._baseline_set = True
            self.samples = int(packets_integrated)
            return
        self.x_deg = cum_x_deg - self._base_x
        self.y_deg = cum_y_deg - self._base_y
        self.z_deg = cum_z_deg - self._base_z
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


def main() -> int:
    ap = argparse.ArgumentParser(description="OAK-D yaw chalk test (disarmed, no config write)")
    ap.add_argument("--expected", type=float, default=90.0,
                    help="Chalk turn magnitude in degrees (90 or 180 typical)")
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
    args = ap.parse_args()

    print("=== OAK-D yaw chalk test (DISARMED / no motors / no config write) ===")
    print("Stop the wall-e service first if it owns the camera.")
    print(f"Expected turn: {args.expected:.1f}° {args.direction}")
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
        nmni_enabled=bool(getattr(config.imu_steering, "oak_nmni_enabled", True)),
        nmni_threshold_dps=float(getattr(config.imu_steering, "oak_nmni_threshold_dps", 0.3)),
        bias_adapt_enabled=False,
        yaw_rate_source=src,
        yaw_rate_scale=scale,
        use_gravity_projected_yaw_rate=bool(
            getattr(config.imu_steering, "oak_use_gravity_projected_yaw_rate", False)
        ),
    )

    print(f"Collecting gyro bias for {args.bias_s:.1f}s — keep robot still...")
    bias = imu.calibrate_gyro(duration_s=args.bias_s)
    print(f"  bias dps: x={bias[0]:.3f} y={bias[1]:.3f} z={bias[2]:.3f}")
    print(f"  production path under test: source={src!r} scale={scale:.4f}")
    print()

    triad = TriadIntegrator()
    period = 1.0 / max(1.0, args.rate_hz)

    def poll_once() -> Dict:
        data = imu.read()
        health = imu.get_health()
        # Prefer lossless producer cumulative triad (proves no host-side loss).
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
            st, age2 = reader.get_imu_data()
            host_ts = float(getattr(st, "timestamp", 0.0) or host_ts)
            age = float(age2)
            triad.update(gx, gy, gz, dev_ts, host_ts, age)
        return data

    # Warm up integrators
    for _ in range(10):
        poll_once()
        time.sleep(period)

    print("READY. Align chalk marks (0°).")
    print("Press Enter to MARK START, then turn to chalk target.")
    print("Press Enter again to MARK END.")
    if args.stream:
        print("(streaming until you press Enter — type in another terminal is not required)")
    try:
        if args.stream:
            print("Streaming... press Enter for MARK START")
            # Non-blocking stream is awkward in raw CLI; poll with timeout via input thread.
            import select
            while True:
                d = poll_once()
                h = imu.get_health()
                print(
                    f"\r hdg={d['heading_deg']:7.2f}  "
                    f"sel={h.get('yaw_rate_source_selected')}  "
                    f"status={h.get('integrate_status')}  "
                    f"age={h.get('sample_age_s'):.3f}s  "
                    f"triad=({triad.x_deg:+7.1f},{triad.y_deg:+7.1f},{triad.z_deg:+7.1f})  "
                    f"dup={h.get('count_duplicate')} stale={h.get('count_stale')}   ",
                    end="",
                    flush=True,
                )
                # Enter available?
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    print()
                    break
                time.sleep(period)
        else:
            input("Press Enter for MARK START...")
    except KeyboardInterrupt:
        print("\nAborted.")
        reader.stop()
        return 130

    # Snapshot start
    start_prod = float(imu.read()["heading_deg"])
    # Free accumulators for production use signed unwrapped from continuous reads
    start_prod_yaw = -math.degrees(imu.yaw_rad)
    start_x, start_y, start_z = triad.x_deg, triad.y_deg, triad.z_deg
    start_health = imu.get_health()
    print(
        f"START  prod_hdg={start_prod:.2f}°  "
        f"triad=({start_x:.2f},{start_y:.2f},{start_z:.2f})  "
        f"sel={start_health.get('yaw_rate_source_selected')}"
    )
    print("Turning... press Enter for MARK END when aligned on chalk.")

    try:
        if args.stream:
            import select
            while True:
                d = poll_once()
                h = imu.get_health()
                print(
                    f"\r hdg={d['heading_deg']:7.2f}  "
                    f"Δprod={_mod360(d['heading_deg'] - start_prod):+7.2f}  "
                    f"Δx={triad.x_deg - start_x:+7.1f} "
                    f"Δy={triad.y_deg - start_y:+7.1f} "
                    f"Δz={triad.z_deg - start_z:+7.1f}  "
                    f"status={h.get('integrate_status')}   ",
                    end="",
                    flush=True,
                )
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    print()
                    break
                time.sleep(period)
        else:
            # Keep integrating while waiting for Enter in a simple loop
            # (user presses Enter when done; we still need samples during turn).
            import select
            while True:
                poll_once()
                if sys.stdin in select.select([sys.stdin], [], [], period)[0]:
                    sys.stdin.readline()
                    break
    except KeyboardInterrupt:
        print("\nAborted.")
        reader.stop()
        return 130

    end_data = poll_once()
    end_prod = float(end_data["heading_deg"])
    end_prod_yaw = -math.degrees(imu.yaw_rad)
    end_x, end_y, end_z = triad.x_deg, triad.y_deg, triad.z_deg
    end_health = imu.get_health()

    # Production path: prefer free yaw accumulator (no wrap), fall back to wrapped.
    d_prod_free = end_prod_yaw - start_prod_yaw
    d_x = _unwrap_delta_deg(start_x, end_x)
    d_y = _unwrap_delta_deg(start_y, end_y)
    d_z = _unwrap_delta_deg(start_z, end_z)

    expected = float(args.expected)
    # Looking down: CW physical often maps to negative heading_deg increase with
    # current OakImuReader sign (-yaw). Report both measured and recommended scale.

    print()
    print("=== RESULTS (scale=1 triad; production uses configured scale) ===")
    print(f"  direction note: physical {args.direction}, expected |Δ|={expected:.1f}°")
    triad_src = "producer_cum" if triad.using_producer else "host_sparse_fallback"
    print(f"  triad source: {triad_src}  samples/packets: {triad.samples}  skipped: {triad.skipped}")
    print(f"  oak_imu health: path={end_health.get('integration_path')} "
          f"status={end_health.get('integrate_status')} "
          f"dup={end_health.get('count_duplicate')} stale={end_health.get('count_stale')} "
          f"regress={end_health.get('count_regressed')} restart={end_health.get('count_restart')} "
          f"integrated={end_health.get('count_integrated')} "
          f"producer_pkts={end_health.get('count_producer_packets')}")
    print(
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
    print(
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
    print(
        "  consumer reseed: "
        f"gen_change={end_health.get('count_generation_change')} "
        f"cum_reset={end_health.get('count_cum_reset')} "
        f"regress={end_health.get('count_regressed')} "
        f"restart={end_health.get('count_restart')}"
    )
    if end_health.get("oak_reconnect_count") is not None:
        print(f"  oak reconnect_count={end_health.get('oak_reconnect_count')} "
              f"connected={end_health.get('oak_connected')}")

    rows = [
        ("gyro_x (scale=1)", d_x),
        ("gyro_y (scale=1)", d_y),
        ("gyro_z (scale=1)", d_z),
        (f"production ({src}×{scale:.3f})", d_prod_free),
    ]
    best_name = None
    best_err = 1e9
    for name, meas in rows:
        rec_scale = (expected / meas) if abs(meas) > 1e-3 else float("nan")
        # For production row, recommended *new* scale folds current scale.
        if name.startswith("production") and abs(meas) > 1e-3:
            rec_scale = scale * (expected / abs(meas)) * (1.0 if meas * expected > 0 else -1.0)
            # Prefer magnitude fit; sign handled separately.
            rec_scale = scale * (expected / abs(meas))
        ok = _pass_band(meas, expected, args.abs_tol_deg, args.rel_tol)
        flag = "PASS" if ok else "fail"
        print(f"  {name:28s}  Δ={meas:+8.2f}°  |Δ|={abs(meas):7.2f}°  "
              f"rec_scale≈{rec_scale:6.3f}  [{flag}]")
        err = abs(abs(meas) - expected)
        if not name.startswith("production") and err < best_err:
            best_err = err
            best_name = name

    print()
    print("Interpretation:")
    print(f"  - Best body axis for this mounting (by |Δ| vs {expected:.0f}°): {best_name}")
    print("  - Production baseline (field-validated): gyro_y × 1.0 — never restore auto/0.46.")
    print("  - Do NOT write config from this tool without review.")
    print("  - Packet-loss proof (truthful): integrated tracks received; backlog_drop=0;")
    print("    cadence_avg≈0.01 s. Drain-batch high-water is messages drained per poll —")
    print("    NOT host-queue occupancy. DepthAI nonblocking overwrite loss is not observable.")
    print("    selection_coalesced=0 is structural only — not a manufactured loss proof.")
    print("  - Sign: heading uses -yaw integration; if Δ sign is opposite your chalk CW/CCW")
    print("    expectation, document it — do not flip sign in production without a full retest.")
    print()
    print("Pass criteria (single axis, scale=1):")
    print(f"  | |measured| - {expected:.0f} | <= max({args.abs_tol_deg:.1f}°, "
          f"{args.rel_tol*100:.0f}% of expected)")
    print("  Loss proof = integrated/received + backlog/gap + drain-batch size, not coalesced.")
    print("  Generation bumps preserve unread cum (incl. +turn/-turn back to ~0);")
    print("  only integrated-counter rewind (true producer replacement) freezes heading.")

    reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
