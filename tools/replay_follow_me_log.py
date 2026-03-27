#!/usr/bin/env python3
"""Offline replay of Follow Me control logic against a structured JSON log.

Feeds recorded IMU heading, motor outputs (for odometry), and person detections
through FollowMeController tick-by-tick, with time.monotonic() replaced by log
timestamps so dwell timers and odometry dt match the original run.

Usage (from repo root):
    python3 tools/replay_follow_me_log.py logs/run_YYYYMMDD_HHMMSS.log
    python3 tools/replay_follow_me_log.py logs/latest.log --run 0
    python3 tools/replay_follow_me_log.py logs/latest.log --csv replay.csv
    python3 tools/replay_follow_me_log.py logs/latest.log --no-gps

Requires: a log produced by app/main.py with mode, imu, motor, follow_me, gps.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

# Repo root on sys.path (tools/ is one level down)
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.follow_me_analyze import parse_follow_me_runs  # noqa: E402

from config import config  # noqa: E402
from pi_app.control.follow_me import FollowMeController, PersonDetection  # noqa: E402
from pi_app.control.mapping import CENTER_OUTPUT_VALUE  # noqa: E402


def _detections_from_frame(fr: dict) -> list[PersonDetection]:
    fm = fr.get("follow_me") or {}
    n = fm.get("num_detections")
    if n is None:
        n = fm.get("num_persons")
    try:
        n_int = int(n) if n is not None else 0
    except (TypeError, ValueError):
        n_int = 0
    if n_int < 1:
        return []
    tz = fm.get("target_z_m")
    tx = fm.get("target_x_m")
    if tz is None or tx is None:
        return []
    conf = fm.get("confidence")
    if conf is None:
        conf = 0.7
    tid = fm.get("target_track_id")
    try:
        tid_i = int(tid) if tid is not None else None
    except (TypeError, ValueError):
        tid_i = None
    return [
        PersonDetection(
            x_m=float(tx),
            z_m=float(tz),
            confidence=float(conf),
            bbox=(0.0, 0.0, 1.0, 1.0),
            track_id=tid_i,
        )
    ]


def _heading_deg(fr: dict) -> float:
    imu = fr.get("imu") or {}
    h = imu.get("heading_deg")
    if h is None:
        return 0.0
    try:
        return float(h)
    except (TypeError, ValueError):
        return 0.0


def _gps_feed(fr: dict) -> tuple[float, float, int] | None:
    g = fr.get("gps") or {}
    lat, lon = g.get("lat"), g.get("lon")
    fix = g.get("fix")
    if lat is None or lon is None or fix is None:
        return None
    try:
        fq = int(fix)
    except (TypeError, ValueError):
        return None
    if fq < 1:
        return None
    return (float(lat), float(lon), fq)


def replay_run(
    frames: list[dict],
    fm_cfg,
    *,
    no_gps: bool = False,
) -> list[dict]:
    """Return one dict per frame with replay outputs + optional error vs log."""
    ctrl = FollowMeController(fm_cfg)
    mono_holder = {"t": 0.0}

    import pi_app.control.follow_me as fm_mod

    real_mono = fm_mod.time.monotonic

    def fake_mono() -> float:
        return float(mono_holder["t"])

    fm_mod.time.monotonic = fake_mono  # type: ignore[assignment]

    rows_out: list[dict] = []
    prev_l = int(CENTER_OUTPUT_VALUE)
    prev_r = int(CENTER_OUTPUT_VALUE)
    t0 = float(frames[0]["ts"])

    try:
        for fr in frames:
            ts = float(fr["ts"])
            mono_holder["t"] = ts - t0

            hdg = _heading_deg(fr)
            ctrl.update_pose(hdg, prev_l, prev_r, mono_holder["t"])

            if not no_gps:
                gps = _gps_feed(fr)
                if gps is not None:
                    lat, lon, fq = gps
                    ctrl.update_gps(lat, lon, fq, mono_holder["t"])

            dets = _detections_from_frame(fr)
            left, right = ctrl.compute(dets)
            st = ctrl.get_status()

            motor = fr.get("motor") or {}
            log_l = motor.get("L")
            log_r = motor.get("R")
            fm = fr.get("follow_me") or {}
            log_spd = fm.get("speed_offset")
            log_str = fm.get("steer_offset")

            row = {
                "ts": ts,
                "ts_rel": mono_holder["t"],
                "replay_L": left,
                "replay_R": right,
                "log_L": log_l,
                "log_R": log_r,
                "replay_speed_offset": st.get("follow_me_speed_offset"),
                "replay_steer_offset": st.get("follow_me_steer_offset"),
                "log_speed_offset": log_spd,
                "log_steer_offset": log_str,
                "pursuit_mode": st.get("follow_me_pursuit_mode"),
                "trail_length": st.get("trail_length"),
                "trail_distance_m": st.get("trail_distance_m"),
                "trail_lookahead_x": st.get("trail_lookahead_x"),
                "trail_lookahead_y": st.get("trail_lookahead_y"),
                "odom_x": st.get("odom_x"),
                "odom_y": st.get("odom_y"),
                "odom_theta_deg": st.get("odom_theta_deg"),
                "odom_source": st.get("odom_source"),
            }
            if log_spd is not None and st.get("follow_me_speed_offset") is not None:
                row["err_speed"] = float(st["follow_me_speed_offset"]) - float(log_spd)
            if log_str is not None and st.get("follow_me_steer_offset") is not None:
                row["err_steer"] = float(st["follow_me_steer_offset"]) - float(log_str)

            rows_out.append(row)

            if log_l is not None:
                prev_l = int(log_l)
            else:
                prev_l = left
            if log_r is not None:
                prev_r = int(log_r)
            else:
                prev_r = right
    finally:
        fm_mod.time.monotonic = real_mono  # type: ignore[assignment]

    return rows_out


def _print_summary(rows: list[dict]) -> None:
    if not rows:
        print("No rows.")
        return
    err_s = [r["err_steer"] for r in rows if "err_steer" in r]
    err_sp = [r["err_speed"] for r in rows if "err_speed" in r]
    if err_s:
        mae_s = sum(abs(x) for x in err_s) / len(err_s)
        max_s = max(abs(x) for x in err_s)
        print(f"Steer offset: MAE={mae_s:.3f}  max|err|={max_s:.3f}")
    if err_sp:
        mae_sp = sum(abs(x) for x in err_sp) / len(err_sp)
        max_sp = max(abs(x) for x in err_sp)
        print(f"Speed offset: MAE={mae_sp:.3f}  max|err|={max_sp:.3f}")


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log_path", type=Path, help="Structured JSON log file")
    ap.add_argument("--run", type=int, default=0, help="FOLLOW_ME segment index (default 0)")
    ap.add_argument("--no-gps", action="store_true", help="Do not feed GPS; dead reckoning only")
    ap.add_argument("--csv", type=Path, default=None, help="Write per-frame replay CSV")
    ap.add_argument("--max-rows", type=int, default=0, help="Limit output rows (0 = all)")
    ap.add_argument("--list-runs", action="store_true", help="List FOLLOW_ME segment lengths and exit")
    args = ap.parse_args()

    if not args.log_path.is_file():
        print(f"File not found: {args.log_path}", file=sys.stderr)
        return 1

    runs = parse_follow_me_runs(str(args.log_path))
    if args.list_runs:
        print(f"Found {len(runs)} FOLLOW_ME segment(s):")
        for i, run in enumerate(runs):
            dur = run[-1]["ts"] - run[0]["ts"] if run else 0
            print(f"  [{i}] {len(run)} frames  duration={dur:.2f}s")
        return 0

    if args.run < 0 or args.run >= len(runs):
        print(f"No segment at index {args.run} (found {len(runs)}). Use --list-runs.", file=sys.stderr)
        return 1

    frames = runs[args.run]
    rows = replay_run(frames, config.follow_me, no_gps=args.no_gps)
    if args.max_rows > 0:
        rows = rows[: args.max_rows]

    _print_summary(rows)

    # Compact table: sample first/last rows on long runs
    print(f"\nReplay frames: {len(rows)} (log segment {args.run})")
    hdr = (
        f"{'t_rel':>6} {'mode':>6} {'r_spd':>6} {'r_str':>6} "
        f"{'log_spd':>7} {'log_str':>7} {'e_spd':>6} {'e_str':>6}"
    )
    print(hdr)
    print("-" * len(hdr))
    if len(rows) <= 12:
        for r in rows:
            _print_row(r)
    else:
        for r in rows[:5]:
            _print_row(r)
        print(" ...")
        for r in rows[-5:]:
            _print_row(r)

    if args.csv:
        _write_csv(args.csv, rows)
        print(f"\nWrote {args.csv}")

    return 0


def _print_row(r: dict) -> None:
    mode = (r.get("pursuit_mode") or "-")[:6]
    rs = r.get("replay_speed_offset")
    rst = r.get("replay_steer_offset")
    ls = r.get("log_speed_offset")
    lst = r.get("log_steer_offset")
    es = r.get("err_speed", "")
    est = r.get("err_steer", "")
    tr = r.get("ts_rel", 0)
    def fmt(v):
        if v == "" or v is None:
            return ""
        if isinstance(v, float):
            return f"{v:6.2f}"
        return f"{float(v):6.2f}"
    print(
        f"{tr:6.2f} {mode:>6} {fmt(rs)} {fmt(rst)} "
        f"{fmt(ls)} {fmt(lst)} {fmt(es)} {fmt(est)}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
