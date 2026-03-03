#!/usr/bin/env python3
"""
Summarize IMU pipeline observability metrics from run_*.log JSON lines.

Outputs concise, docs-friendly text including:
  - loop timing guardrail stats (loop_dt_ms + >=30ms)
  - imu_pipeline availability and counter deltas
  - cadence/freshness/error snapshot
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def _pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    i = max(0, min(len(s) - 1, int(len(s) * p / 100.0) - 1))
    return float(s[i])


def _delta(rows: list[dict], key: str) -> float | None:
    vals: list[float] = []
    for r in rows:
        v = r.get(key)
        if isinstance(v, (int, float)):
            vals.append(float(v))
    if len(vals) < 2:
        return None
    return vals[-1] - vals[0]


def _latest_num(rows: list[dict], key: str) -> float | None:
    for r in reversed(rows):
        v = r.get(key)
        if isinstance(v, (int, float)):
            return float(v)
    return None


def _latest_text(rows: list[dict], key: str) -> str | None:
    for r in reversed(rows):
        v = r.get(key)
        if isinstance(v, str) and v.strip():
            return v.strip()
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze IMU pipeline metrics from structured run logs")
    ap.add_argument("--log", type=Path, required=True, help="Path to run_*.log")
    ap.add_argument(
        "--window-seconds",
        type=float,
        default=None,
        help="Only include final N seconds by log ts (epoch seconds)",
    )
    args = ap.parse_args()

    if not args.log.exists():
        print(f"error: log not found: {args.log}")
        return 2

    rows: list[dict] = []
    for ln in args.log.read_text(errors="ignore").splitlines():
        try:
            obj = json.loads(ln)
        except Exception:
            continue
        if isinstance(obj, dict):
            rows.append(obj)

    if not rows:
        print("error: no JSON rows found")
        return 2

    if args.window_seconds is not None:
        ts_vals = [float(r["ts"]) for r in rows if isinstance(r.get("ts"), (int, float))]
        if ts_vals:
            end_ts = max(ts_vals)
            start_ts = end_ts - float(args.window_seconds)
            rows = [
                r
                for r in rows
                if isinstance(r.get("ts"), (int, float)) and float(r["ts"]) >= start_ts
            ]
        if not rows:
            print("error: no rows in selected window")
            return 2

    loop_vals = [
        float(r["loop_dt_ms"])
        for r in rows
        if isinstance(r.get("loop_dt_ms"), (int, float))
    ]
    if not loop_vals:
        print("error: no loop_dt_ms samples found")
        return 2

    imu_rows = [r.get("imu_pipeline") for r in rows if isinstance(r.get("imu_pipeline"), dict)]
    imu_rows = [r for r in imu_rows if isinstance(r, dict)]
    imu_avail_rows = [r for r in imu_rows if bool(r.get("metrics_available"))]

    ge30_count = sum(v >= 30.0 for v in loop_vals)
    ge30_pct = (ge30_count / len(loop_vals) * 100.0) if loop_vals else 0.0
    mean_loop = statistics.mean(loop_vals)

    print(f"log={args.log}")
    if args.window_seconds is not None:
        print(f"window_seconds={args.window_seconds}")
    print(f"rows={len(rows)}")
    print(
        "loop_dt_ms="
        f"mean:{round(mean_loop, 2)} "
        f"p95:{round(_pct(loop_vals, 95), 2)} "
        f"p99:{round(_pct(loop_vals, 99), 2)} "
        f"max:{round(max(loop_vals), 2)} "
        f"ge30_pct:{round(ge30_pct, 2)} ({ge30_count}/{len(loop_vals)})"
    )

    if not imu_rows:
        print("imu_pipeline=none")
        return 0

    avail_pct = (len(imu_avail_rows) / len(imu_rows) * 100.0) if imu_rows else 0.0
    print(
        "imu_pipeline="
        f"rows:{len(imu_rows)} "
        f"metrics_available_pct:{round(avail_pct, 2)} "
        f"available_rows:{len(imu_avail_rows)}"
    )

    if not imu_avail_rows:
        return 0

    q_recv = _delta(imu_avail_rows, "queue_msgs_received")
    q_cons = _delta(imu_avail_rows, "queue_msgs_consumed")
    q_drop = _delta(imu_avail_rows, "queue_msgs_dropped")
    q_drain = _delta(imu_avail_rows, "queue_drain_count")
    q_drop_pct = (q_drop / q_recv * 100.0) if q_recv and q_recv > 0 and q_drop is not None else None
    print(
        "imu_queue="
        f"recv_delta:{round(q_recv, 1) if q_recv is not None else 'n/a'} "
        f"consumed_delta:{round(q_cons, 1) if q_cons is not None else 'n/a'} "
        f"dropped_delta:{round(q_drop, 1) if q_drop is not None else 'n/a'} "
        f"drop_pct:{round(q_drop_pct, 2) if q_drop_pct is not None else 'n/a'} "
        f"drain_delta:{round(q_drain, 1) if q_drain is not None else 'n/a'}"
    )

    p_recv = _delta(imu_avail_rows, "packets_received")
    p_cons = _delta(imu_avail_rows, "packets_consumed")
    p_coal = _delta(imu_avail_rows, "packets_coalesced")
    p_coal_pct = (p_coal / p_recv * 100.0) if p_recv and p_recv > 0 and p_coal is not None else None
    print(
        "imu_packets="
        f"recv_delta:{round(p_recv, 1) if p_recv is not None else 'n/a'} "
        f"consumed_delta:{round(p_cons, 1) if p_cons is not None else 'n/a'} "
        f"coalesced_delta:{round(p_coal, 1) if p_coal is not None else 'n/a'} "
        f"coalesced_pct:{round(p_coal_pct, 2) if p_coal_pct is not None else 'n/a'}"
    )

    c_avg = _latest_num(imu_avail_rows, "cadence_avg_s")
    c_min = _latest_num(imu_avail_rows, "cadence_min_s")
    c_max = _latest_num(imu_avail_rows, "cadence_max_s")
    c_last = _latest_num(imu_avail_rows, "cadence_last_s")
    c_samples = _latest_num(imu_avail_rows, "cadence_samples")
    sample_age = _latest_num(imu_avail_rows, "last_sample_age_s")
    print(
        "imu_cadence_s="
        f"avg:{round(c_avg, 4) if c_avg is not None else 'n/a'} "
        f"min:{round(c_min, 4) if c_min is not None else 'n/a'} "
        f"max:{round(c_max, 4) if c_max is not None else 'n/a'} "
        f"last:{round(c_last, 4) if c_last is not None else 'n/a'} "
        f"samples:{int(c_samples) if c_samples is not None else 'n/a'}"
    )
    print(f"imu_last_sample_age_s={round(sample_age, 4) if sample_age is not None else 'n/a'}")

    err_delta = _delta(imu_avail_rows, "error_count")
    warn_delta = _delta(imu_avail_rows, "warning_emits")
    err_msg = _latest_text(imu_avail_rows, "last_error_msg")
    print(
        "imu_errors="
        f"error_delta:{round(err_delta, 1) if err_delta is not None else 'n/a'} "
        f"warning_delta:{round(warn_delta, 1) if warn_delta is not None else 'n/a'} "
        f"last_error_msg:{err_msg if err_msg is not None else 'none'}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
