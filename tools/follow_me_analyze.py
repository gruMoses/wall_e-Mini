#!/usr/bin/env python3
"""Follow-Me run analyzer — structured diagnostics for test→tune iteration.

Usage:
    python3 tools/follow_me_analyze.py logs/run_*.log        # analyze latest
    python3 tools/follow_me_analyze.py logs/run_*.log --all   # all runs in file
    python3 tools/follow_me_analyze.py logs/run_*.log --raw   # frame-by-frame last 30s

Can also be run from the Pi:
    python3 ~/wall_e-Mini/tools/follow_me_analyze.py ~/wall_e-Mini/logs/run_*.log
"""

from __future__ import annotations
import json
import math
import sys
from pathlib import Path


def parse_follow_me_runs(path: str) -> list[list[dict]]:
    """Parse log file and return list of follow-me runs (each = list of frames)."""
    runs: list[list[dict]] = []
    current_run: list[dict] = []
    was_follow_me = False

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                continue

            is_follow_me = d.get("mode") == "FOLLOW_ME"
            if is_follow_me:
                current_run.append(d)
                was_follow_me = True
            elif was_follow_me and current_run:
                runs.append(current_run)
                current_run = []
                was_follow_me = False

    if current_run:
        runs.append(current_run)

    return runs


def analyze_run(frames: list[dict], run_idx: int = 0, show_raw: bool = False) -> None:
    """Print comprehensive analysis of a single follow-me run."""
    if not frames:
        print("  (empty run)")
        return

    t0 = frames[0]["ts"]
    duration = frames[-1]["ts"] - t0

    # Extract follow_me data
    fm_data = []
    for fr in frames:
        fm = fr.get("follow_me", {})
        t = fr["ts"] - t0
        fm_data.append({
            "t": t,
            "mode": fm.get("pursuit_mode", "?"),
            "spd": fm.get("speed_offset"),
            "str": fm.get("steer_offset"),
            "z": fm.get("target_z_m"),
            "x": fm.get("target_x_m"),
            "det": fm.get("num_detections") or 0,
            "trail": fm.get("trail_length"),
            "conf": fm.get("confidence"),
            "rej_j": fm.get("trail_rejected_jump_count") or 0,
            "rej_s": fm.get("trail_rejected_speed_count") or 0,
            "odom": fm.get("odom_source"),
            "tdist": fm.get("trail_distance_m"),
        })

    # Header
    ts_iso = frames[0].get("ts_iso", "?")
    print(f"\n{'='*72}")
    print(f"FOLLOW-ME RUN #{run_idx+1}  |  {ts_iso}  |  {duration:.1f}s  |  {len(frames)} frames")
    print(f"{'='*72}")

    # Overall stats
    dists = [d["z"] for d in fm_data if d["z"] is not None]
    speeds = [d["spd"] for d in fm_data if d["spd"] is not None]
    steers = [d["str"] for d in fm_data if d["str"] is not None]
    det_frames = sum(1 for d in fm_data if d["det"] > 0)
    det_rate = det_frames / len(fm_data) * 100 if fm_data else 0

    mode_counts = {}
    for d in fm_data:
        mode_counts[d["mode"]] = mode_counts.get(d["mode"], 0) + 1

    last_rej_s = fm_data[-1]["rej_s"] if fm_data else 0
    last_rej_j = fm_data[-1]["rej_j"] if fm_data else 0

    print(f"\n--- SUMMARY ---")
    if dists:
        print(f"  Distance:    mean={sum(dists)/len(dists):.2f}m  "
              f"min={min(dists):.2f}m  max={max(dists):.2f}m  "
              f"(target: 1.50m)")
    if speeds:
        print(f"  Speed:       mean={sum(speeds)/len(speeds):.0f}  "
              f"min={min(speeds):.0f}  max={max(speeds):.0f}")
    if steers:
        print(f"  |Steer|:     mean={sum(abs(s) for s in steers)/len(steers):.1f}  "
              f"max={max(abs(s) for s in steers):.1f}")
    print(f"  Detection:   {det_frames}/{len(fm_data)} ({det_rate:.0f}%)")
    mode_str = "  ".join(f"{m}={c}" for m, c in sorted(mode_counts.items()))
    print(f"  Modes:       {mode_str}")
    print(f"  Trail rej:   jump={last_rej_j}  speed={last_rej_s}")

    # Steer cap analysis
    cap = 25.0
    cap_frames = sum(1 for d in fm_data if d["str"] is not None and abs(d["str"]) >= cap - 0.5)
    if cap_frames > 0:
        print(f"  ⚠ STEER CAP:  {cap_frames} frames hit ±{cap}")

    # End state
    if fm_data:
        last = fm_data[-1]
        if last["z"] is not None and last["z"] <= 2.0 and last["det"] > 0:
            print(f"  ✓ END:       Caught up (z={last['z']:.1f}m, tracking)")
        elif last["det"] == 0:
            print(f"  ✗ END:       Lost person (z={last['z']}m, no detection)")
        else:
            print(f"  ~ END:       z={last['z']}m, mode={last['mode']}")

    # 5s window table
    print(f"\n--- 5-SECOND WINDOWS ---")
    print(f"  {'Window':>10s}  {'dist':>5s} {'dmax':>5s} {'spd':>4s} "
          f"{'|str|':>5s} {'det%':>4s} {'mode':>8s}  issues")

    window_s = 5.0
    w_start = 0.0
    w_data: list[dict] = []

    def print_window(ws, wd):
        if not wd:
            return
        wd_dists = [d["z"] for d in wd if d["z"] is not None]
        wd_spds = [d["spd"] for d in wd if d["spd"] is not None]
        wd_strs = [d["str"] for d in wd if d["str"] is not None]
        wd_dets = [d["det"] for d in wd]

        avg_d = sum(wd_dists) / len(wd_dists) if wd_dists else 0
        max_d = max(wd_dists) if wd_dists else 0
        avg_s = sum(wd_spds) / len(wd_spds) if wd_spds else 0
        max_str = max(abs(s) for s in wd_strs) if wd_strs else 0
        det_pct = sum(1 for d in wd_dets if d > 0) / len(wd_dets) * 100

        mc = {}
        for d in wd:
            mc[d["mode"]] = mc.get(d["mode"], 0) + 1
        dom = max(mc, key=mc.get)

        issues = []
        if det_pct < 30:
            issues.append(f"LOW_DET({det_pct:.0f}%)")
        if max_d > 4.0:
            issues.append(f"FAR({max_d:.1f}m)")
        if max_str >= cap - 0.5:
            issues.append("STEER_CAP")

        # Check for weak steer when person off-center
        weak = sum(1 for d in wd
                   if d["x"] is not None and abs(d["x"]) > 1.0
                   and d["str"] is not None and abs(d["str"]) < 5
                   and d["mode"] == "trail")
        if weak > 5:
            issues.append(f"WEAK_STEER({weak})")

        issue_str = "  ".join(issues) if issues else ""
        we = ws + window_s
        print(f"  {ws:4.0f}-{we:4.0f}s  {avg_d:5.2f} {max_d:5.2f} {avg_s:4.0f} "
              f"{max_str:5.1f} {det_pct:4.0f}% {dom:>8s}  {issue_str}")

    for d in fm_data:
        if d["t"] >= w_start + window_s and w_data:
            print_window(w_start, w_data)
            w_data = []
            w_start = d["t"] - (d["t"] % window_s)
        w_data.append(d)
    print_window(w_start, w_data)

    # Mode transitions
    print(f"\n--- MODE TRANSITIONS ---")
    last_mode = None
    transitions = []
    for d in fm_data:
        if d["mode"] != last_mode:
            transitions.append(d)
            last_mode = d["mode"]
    for d in transitions:
        z_s = f"z={d['z']:.1f}" if d["z"] is not None else "z=?"
        x_s = f"x={d['x']:+.1f}" if d["x"] is not None else "x=?"
        print(f"  t={d['t']:6.1f}s  -> {d['mode']:8s}  {z_s}  {x_s}  det={d['det']}")
    rapid = 0
    for i in range(1, len(transitions)):
        if transitions[i]["t"] - transitions[i - 1]["t"] < 1.0:
            rapid += 1
    if rapid > 0:
        print(f"  ⚠ {rapid} rapid switches (<1s)")

    # Detection gaps
    print(f"\n--- DETECTION GAPS > 1s ---")
    last_det_t = None
    gaps = []
    for d in fm_data:
        if d["det"] > 0:
            if last_det_t is not None:
                gap = d["t"] - last_det_t
                if gap > 1.0:
                    gaps.append((last_det_t, d["t"], gap))
            last_det_t = d["t"]
    if not gaps:
        print("  (none)")
    for start, end, gap in gaps:
        print(f"  t={start:5.1f}s → t={end:5.1f}s  ({gap:.1f}s)")

    # Problems detected
    print(f"\n--- DIAGNOSTIC FLAGS ---")
    flags = []
    if dists and sum(dists) / len(dists) > 3.0:
        flags.append("AVG_DISTANCE_HIGH: mean > 3.0m, robot can't close gap")
    if det_rate < 50:
        flags.append(f"LOW_DETECTION_RATE: {det_rate:.0f}%, person frequently lost")
    if cap_frames > 0:
        flags.append(f"STEER_CAP_HIT: {cap_frames} frames, may need higher max_steer_offset_byte")
    if last_rej_s > 50:
        flags.append(f"TRAIL_SPEED_REJECTIONS: {last_rej_s}, may need higher trail_max_speed_mps")
    if rapid > 5:
        flags.append(f"MODE_CHATTER: {rapid} rapid switches, increase mode_switch_dwell_s")
    if len(gaps) > 5:
        flags.append(f"FREQUENT_DET_GAPS: {len(gaps)} gaps > 1s")

    # Check for weak trail steering
    weak_trail = sum(1 for d in fm_data
                     if d["x"] is not None and abs(d["x"]) > 1.5
                     and d["str"] is not None and abs(d["str"]) < 5
                     and d["mode"] == "trail")
    if weak_trail > 20:
        flags.append(f"WEAK_TRAIL_STEER: {weak_trail} frames with |x|>1.5 but |steer|<5")

    if not flags:
        flags.append("✓ No major issues detected")
    for f in flags:
        print(f"  • {f}")

    # Raw frame dump (last 30s or on request)
    if show_raw:
        print(f"\n--- RAW FRAMES (last 30s) ---")
        cutoff = duration - 30.0
        print(f"  {'t':>6s} {'mode':>8s} {'spd':>6s} {'str':>6s} "
              f"{'z':>5s} {'x':>5s} {'det':>3s} {'trail':>5s}")
        for d in fm_data:
            if d["t"] < cutoff:
                continue
            spd_s = f"{d['spd']:6.1f}" if d["spd"] is not None else "     -"
            str_s = f"{d['str']:6.1f}" if d["str"] is not None else "     -"
            z_s = f"{d['z']:5.1f}" if d["z"] is not None else "    -"
            x_s = f"{d['x']:5.1f}" if d["x"] is not None else "    -"
            trail_s = f"{d['trail']:5.0f}" if d["trail"] is not None else "    -"
            print(f"  {d['t']:6.1f} {d['mode']:>8s} {spd_s} {str_s} "
                  f"{z_s} {x_s} {d['det']:3d} {trail_s}")


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)

    show_all = "--all" in args
    show_raw = "--raw" in args
    paths = [a for a in args if not a.startswith("--")]

    if not paths:
        print("No log file specified")
        sys.exit(1)

    # Use the latest file if glob gave multiple
    paths.sort()
    path = paths[-1]

    print(f"Analyzing: {path}")
    runs = parse_follow_me_runs(path)

    if not runs:
        print("No follow-me runs found in log")
        sys.exit(0)

    print(f"Found {len(runs)} follow-me run(s)")

    if show_all:
        for i, run in enumerate(runs):
            analyze_run(run, i, show_raw=show_raw)
    else:
        # Analyze only the last (most recent) run
        analyze_run(runs[-1], len(runs) - 1, show_raw=show_raw)


if __name__ == "__main__":
    main()
