#!/usr/bin/env python3
"""Offline analyzer for Follow Me structured JSON run logs.

Reads one-JSON-object-per-line logs (session_header / slow / event / per-tick)
and prints a diagnostic report for FOLLOW_ME ticks: header, binned timeline,
speed jumpiness, detection-filter rejects, steering, speed-cap, obstacles.

Usage (from repo root):
    python3 tools/analyze_follow_me_log.py logs/run_YYYYMMDD_HHMMSS.log
    python3 tools/analyze_follow_me_log.py logs/latest.log --bin 2 --json out.json
    python3 tools/analyze_follow_me_log.py logs/a.log logs/b.log --start 10 --end 40
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

GAP_S = 5.0
SURGE_BYTES = 30.0
JUMP_WINDOW_S = 1.0
DEFAULT_BIN_S = 2.0
DEFAULT_DIRECT_CAP = 32.0
DEFAULT_CLAMP_BYTE = 21.2
DEFAULT_MAX_SPEED_BYTE = 110.0
DEFAULT_DEAD_ZONE_M = 0.2
WINDOW_10S = 10.0


def _get(obj, *keys):
    cur = obj
    for k in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur


def _num(v):
    if v is None or isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        x = float(v)
    else:
        try:
            x = float(v)
        except (TypeError, ValueError):
            return None
    if math.isnan(x) or math.isinf(x):
        return None
    return x


def _mean(vals: list[float]) -> float | None:
    return sum(vals) / len(vals) if vals else None


def _std(vals: list[float]) -> float | None:
    return statistics.stdev(vals) if len(vals) >= 2 else None


def _median(vals: list[float]) -> float | None:
    return statistics.median(vals) if vals else None


def _pctile(vals: list[float], p: float) -> float | None:
    if not vals:
        return None
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (p / 100.0) * (len(s) - 1)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def _f(v, spec: str = ".2f") -> str:
    if v is None:
        return "n/a"
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return "n/a"


def _frac(n: int, d: int) -> float | None:
    return (n / d) if d else None


def _ts(tick) -> float | None:
    return _num(tick.get("ts")) if isinstance(tick, dict) else None


def _rel(tick, t0: float | None) -> float | None:
    ts = _ts(tick)
    if ts is None or t0 is None:
        return None
    return ts - t0


def _mean_key(ticks: list[dict], *keys) -> float | None:
    return _mean([v for v in (_num(_get(tk, *keys)) for tk in ticks) if v is not None])


def _detections(tick) -> list[dict]:
    d = tick.get("detections") if isinstance(tick, dict) else None
    if not isinstance(d, list):
        return []
    return [x for x in d if isinstance(x, dict)]


def _bbox_info(det: dict) -> dict | None:
    bb = det.get("bbox")
    if not (isinstance(bb, (list, tuple)) and len(bb) >= 4):
        return None
    xmin, ymin, xmax, ymax = (_num(bb[0]), _num(bb[1]), _num(bb[2]), _num(bb[3]))
    if None in (xmin, ymin, xmax, ymax):
        return None
    return {
        "xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax,
        "width": xmax - xmin, "conf": _num(det.get("conf")),
        "z_m": _num(det.get("z_m")), "x_m": _num(det.get("x_m")),
        "track_id": det.get("track_id"),
    }


def _hist5(vals: list[float], lo: float, hi: float) -> list[dict]:
    if hi <= lo:
        hi = lo + 1.0
    width = (hi - lo) / 5.0
    counts = [0] * 5
    for v in vals:
        if v >= hi:
            counts[4] += 1
        elif v < lo:
            counts[0] += 1
        else:
            counts[min(4, max(0, int((v - lo) / width)))] += 1
    return [{"lo": lo + i * width, "hi": lo + (i + 1) * width, "n": counts[i]} for i in range(5)]


def _linreg(xs: list[float], ys: list[float]) -> dict:
    n = len(xs)
    empty = {"slope": None, "intercept": None, "r": None, "n": n}
    if n < 2:
        return empty
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0.0:
        return empty
    slope = sxy / sxx
    r = (sxy / math.sqrt(sxx * syy)) if syy > 0.0 else None
    return {"slope": slope, "intercept": my - slope * mx, "r": r, "n": n}


def _mean_abs_delta(ticks: list[dict], *keys, max_dt: float = 1.0) -> float | None:
    prev_ts = prev_v = None
    diffs: list[float] = []
    for tk in ticks:
        ts, v = _ts(tk), _num(_get(tk, *keys))
        if ts is None or v is None:
            continue
        if prev_ts is not None and prev_v is not None and 0.0 < ts - prev_ts <= max_dt:
            diffs.append(abs(v - prev_v))
        prev_ts, prev_v = ts, v
    return _mean(diffs)


def _group_bins(ticks: list[dict], t0: float, width: float) -> dict[int, list[dict]]:
    groups: dict[int, list[dict]] = defaultdict(list)
    for tk in ticks:
        ts = _ts(tk)
        if ts is None:
            continue
        k = int(math.floor((ts - t0) / width)) if width > 0 else 0
        groups[max(k, 0)].append(tk)
    return groups


def _kv_line(name: str, block) -> str:
    if not isinstance(block, dict) or not block:
        return f"{name}: n/a"
    return f"{name}: " + " ".join(f"{k}={'n/a' if v is None else v}" for k, v in block.items())


def _load_records(paths: list) -> tuple[dict | None, list[dict]]:
    header = None
    records: list[dict] = []
    for p in paths:
        with open(p, encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except (json.JSONDecodeError, ValueError, TypeError):
                    continue
                if not isinstance(obj, dict):
                    continue
                if obj.get("type") == "session_header" and header is None:
                    header = obj
                records.append(obj)
    return header, [r for r in records if "type" not in r]


def _segments(ticks: list[dict]) -> list[list[dict]]:
    segs: list[list[dict]] = []
    cur: list[dict] = []
    last_ts: float | None = None
    for tk in ticks:
        ts = _ts(tk)
        if tk.get("mode") == "FOLLOW_ME":
            if cur and ts is not None and last_ts is not None and (ts - last_ts) > GAP_S:
                segs.append(cur)
                cur = []
            cur.append(tk)
            if ts is not None:
                last_ts = ts
        elif cur:
            segs.append(cur)
            cur = []
            last_ts = None
    if cur:
        segs.append(cur)
    return segs


def _mode_transitions(fm_ticks: list[dict], t0: float | None) -> list[dict]:
    out: list[dict] = []
    prev = None
    for tk in fm_ticks:
        mode = _get(tk, "follow_me", "pursuit_mode")
        if prev is not None and mode != prev:
            out.append({
                "t": _rel(tk, t0), "ts": _ts(tk),
                "ts_iso": tk.get("ts_iso") or "n/a",
                "from": prev, "to": mode if mode is not None else "n/a",
            })
        if mode is not None:
            prev = mode
    return out


def _window_ticks(ticks: list[dict], t_lo: float, t_hi: float) -> list[dict]:
    out = []
    for tk in ticks:
        ts = _ts(tk)
        if ts is not None and t_lo <= ts <= t_hi:
            out.append(tk)
    return out


def _classify_cut(window: list[dict], dead_zone: float) -> str:
    for tk in window:
        if _get(tk, "follow_me", "tracking") is False:
            return "lost"
    for tk in window:
        if _get(tk, "follow_me", "fresh_detection") is False:
            return "dropout"
    for tk in window:
        thr = _num(_get(tk, "obstacle", "throttle_scale"))
        if thr is not None and thr < 1.0:
            return "obstacle"
    for tk in window:
        err = _num(_get(tk, "follow_me", "distance_error_m"))
        if err is not None and abs(err) <= dead_zone:
            return "arrived"
    return "unknown"


def _first_bbox_in_window(window: list[dict]) -> dict | None:
    for tk in window:
        for det in _detections(tk):
            info = _bbox_info(det)
            if info is not None:
                return info
    return None


def _jump_events(fm_ticks: list[dict], t0: float | None, dead_zone: float, want_cut: bool) -> list[dict]:
    n = len(fm_ticks)
    flags = [False] * n
    for i, tk in enumerate(fm_ticks):
        so, ts = _num(_get(tk, "follow_me", "speed_offset")), _ts(tk)
        if so is None or ts is None:
            continue
        extreme = None
        for j in range(i - 1, -1, -1):
            tsj = _ts(fm_ticks[j])
            if tsj is None:
                continue
            if ts - tsj > JUMP_WINDOW_S:
                break
            soj = _num(_get(fm_ticks[j], "follow_me", "speed_offset"))
            if soj is None:
                continue
            extreme = soj if extreme is None else (max(extreme, soj) if want_cut else min(extreme, soj))
        if extreme is None:
            continue
        delta = so - extreme
        if want_cut and delta <= -SURGE_BYTES:
            flags[i] = True
        elif (not want_cut) and delta >= SURGE_BYTES:
            flags[i] = True
    events: list[dict] = []
    in_run = False
    for i, flag in enumerate(flags):
        if flag and not in_run:
            tk = fm_ticks[i]
            ts = _ts(tk)
            window = _window_ticks(fm_ticks, (ts or 0) - JUMP_WINDOW_S, ts or 0) if ts is not None else [tk]
            ev = {
                "t": _rel(tk, t0), "ts": ts, "ts_iso": tk.get("ts_iso") or "n/a",
                "z": _num(_get(tk, "follow_me", "target_z_m")),
                "x": _num(_get(tk, "follow_me", "target_x_m")),
                "kind": "cut" if want_cut else "surge",
            }
            if want_cut:
                ev["cause"] = _classify_cut(window, dead_zone)
                ev["bbox"] = _first_bbox_in_window(window)
            events.append(ev)
            in_run = True
        elif not flag:
            in_run = False
    return events


def _jumpiness_block(fm_ticks: list[dict], t0: float | None, dead_zone: float) -> dict:
    speeds = [v for v in (_num(_get(tk, "follow_me", "speed_offset")) for tk in fm_ticks) if v is not None]
    cuts = _jump_events(fm_ticks, t0, dead_zone, want_cut=True)
    surges = _jump_events(fm_ticks, t0, dead_zone, want_cut=False)
    return {
        "std_speed_offset": _std(speeds),
        "mean_abs_delta_speed_offset": _mean_abs_delta(fm_ticks, "follow_me", "speed_offset"),
        "n_surges": len(surges), "n_cuts": len(cuts),
        "surges": surges, "cuts": cuts,
    }


def _bearing(tick) -> float | None:
    x = _num(_get(tick, "follow_me", "target_x_m"))
    z = _num(_get(tick, "follow_me", "target_z_m"))
    if x is None or z is None:
        return None
    return math.degrees(math.atan2(x, z))


def _tick_dt(ticks: list[dict], i: int) -> float | None:
    ts = _ts(ticks[i])
    if i + 1 < len(ticks):
        ts2 = _ts(ticks[i + 1])
        if ts is not None and ts2 is not None and ts2 > ts:
            return ts2 - ts
    dt_ms = _num(ticks[i].get("loop_dt_ms"))
    return None if dt_ms is None else dt_ms / 1000.0


def _heading_deg(tick) -> float | None:
    h = _num(_get(tick, "imu", "heading_deg"))
    if h is not None:
        return h
    return _num(_get(tick, "imu", "oak_imu", "heading_deg"))


def _logged_yaw_rate_dps(tick) -> float | None:
    y = _num(_get(tick, "imu", "yaw_rate_dps"))
    if y is not None:
        return y
    return _num(_get(tick, "imu", "oak_imu", "yaw_rate_world_dps"))


def _unwrap_heading_deg(headings: list[float]) -> list[float]:
    if not headings:
        return []
    out = [float(headings[0])]
    prev = float(headings[0])
    acc = prev
    for raw in headings[1:]:
        v = float(raw)
        d = (v - prev + 180.0) % 360.0 - 180.0
        acc += d
        out.append(acc)
        prev = v
    return out


def _lr_yaw_bins(xs: list[float], ys: list[float]) -> list[dict]:
    bins: dict[float, list[float]] = defaultdict(list)
    for x, y in zip(xs, ys):
        bins[math.floor(x / 5.0) * 5.0].append(y)
    return [
        {"lr_lo": k, "lr_hi": k + 5.0, "mean_yaw_dps": _mean(bins[k]), "n": len(bins[k])}
        for k in sorted(bins)
    ]


def _yaw_fit(fm_ticks: list[dict], pred) -> dict:
    """Yaw vs L-R: heading derivative (primary) plus the logged-rate fit.

    Logged ``yaw_rate_dps`` is often zero-filled under follow-me load (the
    duplicate-branch freshness bound used to be tighter than the vision
    thread's IMU drain). Derive rate from unwrapped ``imu.heading_deg``
    with a +/-2-tick central difference, then fit against L-R delayed by
    a lag searched over 0..6 ticks.
    """
    rows: list[tuple[float, float, float]] = []
    logged_xs: list[float] = []
    logged_ys: list[float] = []
    n_zero = 0
    n_logged = 0
    for tk in fm_ticks:
        actual = _num(_get(tk, "follow_me", "speed_loop", "actual_mps"))
        if actual is None or not pred(actual):
            continue
        lft = _num(_get(tk, "motor", "L"))
        rgt = _num(_get(tk, "motor", "R"))
        ts = _ts(tk)
        heading = _heading_deg(tk)
        if None not in (lft, rgt, ts, heading):
            rows.append((float(ts), float(heading), float(lft) - float(rgt)))
        yaw = _logged_yaw_rate_dps(tk)
        if None in (lft, rgt, yaw):
            continue
        n_logged += 1
        if yaw == 0.0:
            n_zero += 1
        logged_xs.append(float(lft) - float(rgt))
        logged_ys.append(float(yaw))
    logged = _linreg(logged_xs, logged_ys)
    logged["bins"] = _lr_yaw_bins(logged_xs, logged_ys)
    logged["n_zero"] = n_zero
    logged["n_samples"] = n_logged

    empty = {
        "slope": None, "intercept": None, "r": None, "n": 0,
        "lag_ticks": None, "lag_s": None, "bins": [],
        "logged": logged,
    }
    n = len(rows)
    if n < 5:
        return empty

    ts_a = [r[0] for r in rows]
    unw = _unwrap_heading_deg([r[1] for r in rows])
    lr_a = [r[2] for r in rows]
    samples: list[tuple[int, float]] = []
    for i in range(2, n - 2):
        dt = ts_a[i + 2] - ts_a[i - 2]
        if dt <= 0.0 or dt > 1.0:
            continue
        samples.append((i, (unw[i + 2] - unw[i - 2]) / dt))
    if not samples:
        return empty

    best: dict | None = None
    best_abs_r = -1.0
    best_lag = 0
    best_xs: list[float] = []
    best_ys: list[float] = []
    for lag in range(0, 7):
        xs: list[float] = []
        ys: list[float] = []
        for i, rate in samples:
            j = i - lag
            if j < 0:
                continue
            xs.append(lr_a[j])
            ys.append(rate)
        fit = _linreg(xs, ys)
        r = fit.get("r")
        if r is None or fit.get("n", 0) < 2:
            continue
        if abs(r) > best_abs_r:
            best_abs_r = abs(r)
            best = fit
            best_lag = lag
            best_xs = xs
            best_ys = ys
    if best is None:
        return empty

    lag_dts: list[float] = []
    for i, _rate in samples:
        j = i - best_lag
        if j < 0:
            continue
        if best_lag == 0:
            lag_dts.append(0.0)
        else:
            dtl = ts_a[i] - ts_a[j]
            if dtl > 0.0:
                lag_dts.append(dtl)
    best["lag_ticks"] = best_lag
    best["lag_s"] = _median(lag_dts) if lag_dts else None
    best["bins"] = _lr_yaw_bins(best_xs, best_ys)
    best["logged"] = logged
    return best


def _section_timeline(fm_ticks: list[dict], t0: float, bin_s: float) -> list[dict]:
    rows = []
    for k, chunk in sorted(_group_bins(fm_ticks, t0, bin_s).items()):
        last_mode = None
        for tk in chunk:
            m = _get(tk, "follow_me", "pursuit_mode")
            if m is not None:
                last_mode = m
        n = len(chunk)
        thr_f = [v for v in (_num(_get(tk, "obstacle", "throttle_scale")) for tk in chunk) if v is not None]
        rows.append({
            "t": k * bin_s,
            "pursuit_mode": last_mode,
            "mean_target_x_m": _mean_key(chunk, "follow_me", "target_x_m"),
            "mean_target_z_m": _mean_key(chunk, "follow_me", "target_z_m"),
            "mean_distance_error_m": _mean_key(chunk, "follow_me", "distance_error_m"),
            "mean_speed_offset": _mean_key(chunk, "follow_me", "speed_offset"),
            "mean_steer_offset": _mean_key(chunk, "follow_me", "steer_offset"),
            "mean_motor_L": _mean_key(chunk, "motor", "L"),
            "mean_motor_R": _mean_key(chunk, "motor", "R"),
            "mean_left_rpm": _mean_key(chunk, "vesc", "left_rpm"),
            "mean_right_rpm": _mean_key(chunk, "vesc", "right_rpm"),
            "mean_actual_mps": _mean_key(chunk, "follow_me", "speed_loop", "actual_mps"),
            "mean_open_loop_byte": _mean_key(chunk, "follow_me", "speed_loop", "open_loop_byte"),
            "mean_corr_byte": _mean_key(chunk, "follow_me", "speed_loop", "corr_byte"),
            "frac_closed": _frac(sum(1 for tk in chunk if _get(tk, "follow_me", "speed_loop", "closed") is True), n),
            "mean_throttle_scale": _mean(thr_f),
            "min_throttle_scale": min(thr_f) if thr_f else None,
            "mean_depth_valid_pct": _mean_key(chunk, "obstacle", "depth_valid_pct"),
            "frac_fresh_detection": _frac(sum(1 for tk in chunk if _get(tk, "follow_me", "fresh_detection") is True), n),
            "frac_tracking": _frac(sum(1 for tk in chunk if _get(tk, "follow_me", "tracking") is True), n),
            "mean_n_detections": _mean([float(len(_detections(tk))) for tk in chunk]),
            "n_ticks": n,
        })
    return rows


def _count_target_track_id_changes(segs: list[list[dict]]) -> int:
    """Ticks where follow_me.target_track_id differs from the previous non-null
    value within a FOLLOW_ME segment. Logs that lack the field contribute 0.
    """
    n = 0
    for seg in segs:
        prev = None
        for tk in seg:
            tid = _num(_get(tk, "follow_me", "target_track_id"))
            if tid is None:
                continue
            if prev is not None and tid != prev:
                n += 1
            prev = tid
    return n


def _section_detection_filter(fm_ticks: list[dict], segs: list[list[dict]] | None = None) -> dict:
    infos: list[dict] = []
    n_ticks = 0
    for tk in fm_ticks:
        dets = _detections(tk)
        if not dets or _get(tk, "follow_me", "fresh_detection") is not False:
            continue
        n_ticks += 1
        for det in dets:
            info = _bbox_info(det)
            if info is not None:
                infos.append(info)
    widths = [i["width"] for i in infos if i.get("width") is not None]
    return {
        "n_ticks": n_ticks, "n_detections": len(infos),
        "hist_width": _hist5(widths, 0.0, 1.0),
        "hist_xmin": _hist5([i["xmin"] for i in infos if i.get("xmin") is not None], 0.0, 1.0),
        "hist_xmax": _hist5([i["xmax"] for i in infos if i.get("xmax") is not None], 0.0, 1.0),
        "hist_conf": _hist5([i["conf"] for i in infos if i.get("conf") is not None], 0.0, 1.0),
        "hist_z_m": _hist5([i["z_m"] for i in infos if i.get("z_m") is not None], 0.0, 5.0),
        "n_width_lt_0_09": sum(1 for w in widths if w < 0.09),
        "n_width_lt_0_07": sum(1 for w in widths if w < 0.07),
        "n_width_lt_0_05": sum(1 for w in widths if w < 0.05),
        "n_width_lt_0_04": sum(1 for w in widths if w < 0.04),
        "n_edge": sum(
            1 for i in infos
            if (i.get("xmin") is not None and i["xmin"] > 0.85)
            or (i.get("xmax") is not None and i["xmax"] < 0.15)
        ),
        "n_top_clipped": sum(1 for i in infos if i.get("ymin") is not None and i["ymin"] <= 0.01),
        "n_target_track_id_changes": _count_target_track_id_changes(
            segs if segs is not None else ([fm_ticks] if fm_ticks else [])
        ),
    }


def _section_steering(fm_ticks: list[dict], t0: float | None, direct_cap: float) -> dict:
    abs_steer = [abs(s) for s in (_num(_get(tk, "follow_me", "steer_offset")) for tk in fm_ticks) if s is not None]
    n_cap = sum(1 for v in abs_steer if v >= 0.95 * direct_cap)
    t15 = t25 = 0.0
    events: list[dict] = []
    active = None
    for i, tk in enumerate(fm_ticks):
        b = _bearing(tk)
        dt = _tick_dt(fm_ticks, i) or 0.0
        if b is not None:
            if abs(b) > 15.0:
                t15 += dt
            if abs(b) > 25.0:
                t25 += dt
        tracking = _get(tk, "follow_me", "tracking")
        if active is None:
            if b is not None and abs(b) > 20.0:
                active = {
                    "t_start": _rel(tk, t0), "ts_start": _ts(tk),
                    "ts_iso_start": tk.get("ts_iso") or "n/a", "bearing_start_deg": b,
                }
        elif tracking is False or (b is not None and abs(b) < 10.0):
            ts_end, ts_start = _ts(tk), active.get("ts_start")
            dur = (ts_end - ts_start) if (ts_end is not None and ts_start is not None) else None
            active.update({
                "t_end": _rel(tk, t0), "ts_end": ts_end, "duration_s": dur,
                "end_reason": "lost" if tracking is False else "returned",
            })
            events.append(active)
            active = None
    if active is not None:
        last = fm_ticks[-1]
        ts_end, ts_start = _ts(last), active.get("ts_start")
        active.update({
            "t_end": _rel(last, t0), "ts_end": ts_end,
            "duration_s": (ts_end - ts_start) if (ts_end is not None and ts_start is not None) else None,
            "end_reason": "open",
        })
        events.append(active)
    return {
        "abs_steer_p50": _pctile(abs_steer, 50), "abs_steer_p90": _pctile(abs_steer, 90),
        "abs_steer_max": max(abs_steer) if abs_steer else None,
        "frac_at_direct_cap": _frac(n_cap, len(fm_ticks)), "direct_cap": direct_cap,
        "time_bearing_gt_15_s": t15, "time_bearing_gt_25_s": t25,
        "bearing_events": events,
        "yaw_moving": _yaw_fit(fm_ticks, lambda a: a >= 0.3),
        "yaw_pivot": _yaw_fit(fm_ticks, lambda a: a < 0.15),
    }


def _section_speed_cap(fm_ticks: list[dict], max_speed_byte: float) -> dict:
    n_cap = 0
    max_l = max_r = duty_l = duty_r = None
    ratios: list[float] = []
    for tk in fm_ticks:
        so = _num(_get(tk, "follow_me", "speed_offset"))
        if so is not None and so >= max_speed_byte - 0.5:
            n_cap += 1
        lrpm, rrpm = _num(_get(tk, "vesc", "left_rpm")), _num(_get(tk, "vesc", "right_rpm"))
        ld, rd = _num(_get(tk, "vesc", "l_duty")), _num(_get(tk, "vesc", "r_duty"))
        if lrpm is not None and (max_l is None or lrpm > max_l):
            max_l, duty_l = lrpm, ld
        if rrpm is not None and (max_r is None or rrpm > max_r):
            max_r, duty_r = rrpm, rd
        for rpm, duty in ((lrpm, ld), (rrpm, rd)):
            if rpm is not None and duty not in (None, 0.0) and abs(rpm) > 8000:
                ratios.append(abs(rpm) / abs(duty))
    return {
        "max_follow_speed_byte": max_speed_byte,
        "frac_at_speed_cap": _frac(n_cap, len(fm_ticks)),
        "max_left_rpm": max_l, "l_duty_at_max_left_rpm": duty_l,
        "max_right_rpm": max_r, "r_duty_at_max_right_rpm": duty_r,
        "mean_erpm_per_duty": _mean(ratios), "n_erpm_gt_8000": len(ratios),
    }


def _section_obstacle(fm_ticks: list[dict], t0: float | None) -> dict:
    rows = []
    n_runs = longest = run = 0
    prev = False
    for tk in fm_ticks:
        thr = _num(_get(tk, "obstacle", "throttle_scale"))
        active = thr is not None and thr < 1.0
        if active:
            rows.append({
                "t": _rel(tk, t0), "ts": _ts(tk), "ts_iso": tk.get("ts_iso") or "n/a",
                "throttle_scale": thr,
                "distance_m": _num(_get(tk, "obstacle", "distance_m")),
                "depth_p5_mm": _num(_get(tk, "obstacle", "depth_p5_mm")),
                "depth_valid_pct": _num(_get(tk, "obstacle", "depth_valid_pct")),
                "target_z_m": _num(_get(tk, "follow_me", "target_z_m")),
            })
            run += 1
            if not prev:
                n_runs += 1
            longest = max(longest, run)
        else:
            run = 0
        prev = active
    n_low = sum(1 for r in rows if r["depth_valid_pct"] is not None and r["depth_valid_pct"] < 8.0)
    return {
        "ticks": rows, "n_ticks": len(rows),
        "n_depth_valid_pct_lt_8": n_low, "n_runs": n_runs, "longest_run": longest,
    }


def _clamp_stats(fm_ticks: list[dict], clamp_byte: float) -> dict:
    thr = 0.95 * clamp_byte
    n_clamp = n_wound = n_thr1 = n_thr05 = 0
    for tk in fm_ticks:
        corr = _num(_get(tk, "follow_me", "speed_loop", "corr_byte"))
        if corr is not None and abs(corr) >= thr:
            n_clamp += 1
            so = _num(_get(tk, "follow_me", "speed_offset"))
            olb = _num(_get(tk, "follow_me", "speed_loop", "open_loop_byte"))
            if so is not None and olb is not None and so < olb - 5.0:
                n_wound += 1
        scale = _num(_get(tk, "obstacle", "throttle_scale"))
        if scale is not None:
            if scale < 1.0:
                n_thr1 += 1
            if scale < 0.5:
                n_thr05 += 1
    n = len(fm_ticks)
    return {
        "clamp_byte": clamp_byte,
        "frac_corr_clamped": _frac(n_clamp, n), "n_corr_clamped": n_clamp,
        "frac_clamp_wound_up": _frac(n_wound, n_clamp),
        "frac_throttle_lt_1": _frac(n_thr1, n), "frac_throttle_lt_0_5": _frac(n_thr05, n),
    }


def _section_person_depth(fm_ticks: list[dict]) -> dict:
    """Person-stereo diagnostics over FOLLOW_ME ticks (first detection per tick)."""
    status_counts: dict[str, int] = {}
    saw_status = False
    n_z_jumps = 0
    valid_pxs: list[float] = []
    det_fps: list[float] = []
    depth_fps: list[float] = []
    saw_fps = False
    prev_z_stereo: float | None = None
    prev_tid = None
    for tk in fm_ticks:
        dets = _detections(tk)
        if dets:
            first = dets[0]
            if "depth_status" in first:
                saw_status = True
                st = first.get("depth_status")
                key = str(st) if st is not None else "n/a"
                status_counts[key] = status_counts.get(key, 0) + 1
            vp = _num(first.get("depth_valid_px"))
            if vp is not None and vp >= 0.0:
                valid_pxs.append(vp)
            z_st = _num(first.get("z_stereo_m"))
            tid = first.get("track_id")
            if (
                prev_z_stereo is not None
                and z_st is not None
                and tid is not None
                and prev_tid is not None
                and tid == prev_tid
                and abs(z_st - prev_z_stereo) > 1.0
            ):
                n_z_jumps += 1
            if z_st is not None:
                prev_z_stereo = z_st
            if tid is not None:
                prev_tid = tid
        oak = tk.get("oak") if isinstance(tk.get("oak"), dict) else None
        if oak is not None and ("det_fps" in oak or "depth_fps" in oak):
            saw_fps = True
        if oak is not None:
            df = _num(oak.get("det_fps"))
            dpf = _num(oak.get("depth_fps"))
            if df is not None:
                det_fps.append(df)
            if dpf is not None:
                depth_fps.append(dpf)
    return {
        "status_counts": status_counts if saw_status else None,
        "n_z_stereo_jumps_same_track": n_z_jumps if saw_status else None,
        "median_depth_valid_px": _median(valid_pxs),
        "median_det_fps": _median(det_fps) if saw_fps else None,
        "median_depth_fps": _median(depth_fps) if saw_fps else None,
    }


def analyze(paths, bin_s: float = DEFAULT_BIN_S, start_s: float | None = None,
            end_s: float | None = None, direct_cap: float | None = None) -> dict:
    """Compute every report metric from one or more JSON run logs."""
    header, ticks = _load_records([str(p) for p in paths])
    fm_all = [tk for tk in ticks if tk.get("mode") == "FOLLOW_ME"]
    t0 = next((ts for ts in (_ts(tk) for tk in fm_all) if ts is not None), None)
    lo = (t0 + (start_s or 0.0)) if t0 is not None else None
    hi = (t0 + end_s) if (t0 is not None and end_s is not None) else None

    def _in_window(tk) -> bool:
        ts = _ts(tk)
        if ts is None:
            return False
        if lo is not None and ts < lo:
            return False
        if hi is not None and ts > hi:
            return False
        return True

    ticks_w = [tk for tk in ticks if _in_window(tk)] if t0 is not None else ticks
    segs = _segments(ticks_w)
    fm_ticks = [tk for seg in segs for tk in seg]
    cfg_fm = _get(header, "config", "follow_me") if header else None
    cfg_vesc = _get(header, "config", "vesc") if header else None
    cfg_imu = _get(header, "config", "imu_steering") if header else None
    header_cap = _num(_get(cfg_fm, "direct_mode_max_steer_byte"))
    if direct_cap is not None:
        used_direct_cap = direct_cap
    elif header_cap is not None:
        used_direct_cap = header_cap
    else:
        used_direct_cap = DEFAULT_DIRECT_CAP
    max_corr = _num(_get(cfg_fm, "speed_pid_max_correction_mps"))
    mps_per = _num(_get(cfg_fm, "speed_loop_mps_per_byte"))
    clamp_byte = (max_corr / mps_per) if (max_corr is not None and mps_per not in (None, 0.0)) else DEFAULT_CLAMP_BYTE
    max_spd = _num(_get(cfg_fm, "max_follow_speed_byte")) or DEFAULT_MAX_SPEED_BYTE
    dead_zone = _num(_get(cfg_fm, "speed_dead_zone_m"))
    if dead_zone is None:
        dead_zone = DEFAULT_DEAD_ZONE_M
    loop_dts = [v for v in (_num(tk.get("loop_dt_ms")) for tk in fm_ticks) if v is not None]
    seg_info = []
    for seg in segs:
        ts0, ts1 = _ts(seg[0]), _ts(seg[-1])
        seg_info.append({
            "start_ts_iso": seg[0].get("ts_iso") or "n/a", "start_ts": ts0,
            "duration_s": (ts1 - ts0) if (ts0 is not None and ts1 is not None) else None,
            "n_ticks": len(seg),
        })
    jump_overall = _jumpiness_block(fm_ticks, t0, dead_zone)
    jump_overall.update(_clamp_stats(fm_ticks, clamp_byte))
    windows_10s = []
    if t0 is not None and fm_ticks:
        for k, chunk in sorted(_group_bins(fm_ticks, t0, WINDOW_10S).items()):
            block = _jumpiness_block(chunk, t0, dead_zone)
            block["t"] = k * WINDOW_10S
            block.pop("surges", None)
            block.pop("cuts", None)
            windows_10s.append(block)
    jump_overall["windows_10s"] = windows_10s
    return {
        "header": {
            "git_sha": (header or {}).get("git_sha") if header else None,
            "config": {"follow_me": cfg_fm, "vesc": cfg_vesc, "imu_steering": cfg_imu},
            "n_segments": len(segs), "segments": seg_info,
            "median_loop_dt_ms": _median(loop_dts),
            "pursuit_mode_transitions": _mode_transitions(fm_ticks, t0),
            "n_follow_me_ticks": len(fm_ticks), "t0": t0,
            "dead_zone_m": dead_zone, "clamp_byte": clamp_byte,
        },
        "timeline": _section_timeline(fm_ticks, t0, bin_s) if t0 is not None else [],
        "jumpiness": jump_overall,
        "detection_filter": _section_detection_filter(fm_ticks, segs),
        "steering": _section_steering(fm_ticks, t0, used_direct_cap),
        "speed_cap": _section_speed_cap(fm_ticks, max_spd),
        "obstacle": _section_obstacle(fm_ticks, t0),
        "person_depth": _section_person_depth(fm_ticks),
    }


def _fmt_hist(name: str, bins: list[dict]) -> str:
    parts = [f"[{_f(b['lo'], '.2f')}-{_f(b['hi'], '.2f')}]={b['n']}" for b in bins]
    return f"  {name}: " + (" ".join(parts) if parts else "n/a")


def render_report(m: dict) -> str:
    h = m.get("header") or {}
    cfg = h.get("config") or {}
    lines = [
        "=== HEADER ===",
        f"git_sha: {h.get('git_sha') or 'n/a'}",
        _kv_line("follow_me", cfg.get("follow_me")),
        _kv_line("vesc", cfg.get("vesc")),
        _kv_line("imu_steering", cfg.get("imu_steering")),
        f"FOLLOW_ME segments: {h.get('n_segments', 0)}",
    ]
    for i, seg in enumerate(h.get("segments") or []):
        lines.append(
            f"  [{i}] start={seg.get('start_ts_iso') or 'n/a'}  "
            f"duration_s={_f(seg.get('duration_s'), '.2f')}  ticks={seg.get('n_ticks', 0)}"
        )
    lines.append(f"median loop_dt_ms: {_f(h.get('median_loop_dt_ms'), '.1f')}")
    trans = h.get("pursuit_mode_transitions") or []
    if not trans:
        lines.append("pursuit_mode transitions: (none)")
    else:
        lines.append("pursuit_mode transitions:")
        for tr in trans:
            lines.append(f"  t={_f(tr.get('t'), '.2f')} {tr.get('ts_iso')}: {tr.get('from')} -> {tr.get('to')}")

    lines += ["", "=== TIMELINE ==="]
    lines.append(
        f"{'t':>6} {'mode':>8} {'x':>6} {'z':>6} {'derr':>6} {'spd':>6} {'str':>6} "
        f"{'L':>6} {'R':>6} {'lrpm':>7} {'rrpm':>7} {'act':>6} {'olb':>6} {'corr':>6} "
        f"{'cls':>5} {'thr':>5} {'tmin':>5} {'dvp':>5} {'frsh':>5} {'trk':>5} {'ndet':>5}"
    )
    for row in m.get("timeline") or []:
        lines.append(
            f"{_f(row.get('t'), '.1f'):>6} {str(row.get('pursuit_mode') or 'n/a'):>8} "
            f"{_f(row.get('mean_target_x_m')):>6} {_f(row.get('mean_target_z_m')):>6} "
            f"{_f(row.get('mean_distance_error_m')):>6} {_f(row.get('mean_speed_offset'), '.1f'):>6} "
            f"{_f(row.get('mean_steer_offset'), '.1f'):>6} {_f(row.get('mean_motor_L'), '.1f'):>6} "
            f"{_f(row.get('mean_motor_R'), '.1f'):>6} {_f(row.get('mean_left_rpm'), '.0f'):>7} "
            f"{_f(row.get('mean_right_rpm'), '.0f'):>7} {_f(row.get('mean_actual_mps')):>6} "
            f"{_f(row.get('mean_open_loop_byte'), '.1f'):>6} {_f(row.get('mean_corr_byte'), '.1f'):>6} "
            f"{_f(row.get('frac_closed')):>5} {_f(row.get('mean_throttle_scale')):>5} "
            f"{_f(row.get('min_throttle_scale')):>5} {_f(row.get('mean_depth_valid_pct')):>5} "
            f"{_f(row.get('frac_fresh_detection')):>5} {_f(row.get('frac_tracking')):>5} "
            f"{_f(row.get('mean_n_detections')):>5}"
        )

    j = m.get("jumpiness") or {}
    lines += ["", "=== JUMPINESS ===", "overall:"]
    lines.append(
        f"  std speed_offset={_f(j.get('std_speed_offset'), '.2f')}  "
        f"mean|d speed_offset|={_f(j.get('mean_abs_delta_speed_offset'), '.2f')}  "
        f"SURGES={j.get('n_surges', 0)}  CUTS={j.get('n_cuts', 0)}"
    )
    for ev in j.get("cuts") or []:
        bb = ev.get("bbox") or {}
        lines.append(
            f"  CUT t={_f(ev.get('t'), '.2f')} z={_f(ev.get('z'))} x={_f(ev.get('x'))} "
            f"cause={ev.get('cause') or 'n/a'}  bbox w={_f(bb.get('width'), '.3f')} "
            f"xmin={_f(bb.get('xmin'), '.3f')} xmax={_f(bb.get('xmax'), '.3f')} "
            f"conf={_f(bb.get('conf'))} z_m={_f(bb.get('z_m'))}"
        )
    lines.append(
        f"  frac |corr_byte|>=0.95*clamp ({_f(j.get('clamp_byte'), '.1f')}): "
        f"{_f(j.get('frac_corr_clamped'))}  "
        f"of those wound-up (speed_offset < open_loop-5): {_f(j.get('frac_clamp_wound_up'))}"
    )
    lines.append(
        f"  frac throttle_scale<1.0: {_f(j.get('frac_throttle_lt_1'))}  "
        f"<0.5: {_f(j.get('frac_throttle_lt_0_5'))}"
    )
    lines.append("per 10 s window:")
    for w in j.get("windows_10s") or []:
        lines.append(
            f"  t={_f(w.get('t'), '.1f')}  std={_f(w.get('std_speed_offset'), '.2f')}  "
            f"mean|d|={_f(w.get('mean_abs_delta_speed_offset'), '.2f')}  "
            f"SURGES={w.get('n_surges', 0)}  CUTS={w.get('n_cuts', 0)}"
        )

    d = m.get("detection_filter") or {}
    lines += ["", "=== DETECTION FILTER ==="]
    lines.append(
        f"raw detection but fresh_detection=false: ticks={d.get('n_ticks', 0)}  "
        f"detections={d.get('n_detections', 0)}"
    )
    for key, label in (
        ("hist_width", "bbox_width"), ("hist_xmin", "xmin"), ("hist_xmax", "xmax"),
        ("hist_conf", "conf"), ("hist_z_m", "z_m"),
    ):
        lines.append(_fmt_hist(label, d.get(key) or []))
    lines.append(
        f"  width<0.09={d.get('n_width_lt_0_09', 0)}  <0.07={d.get('n_width_lt_0_07', 0)}  "
        f"<0.05={d.get('n_width_lt_0_05', 0)}  <0.04={d.get('n_width_lt_0_04', 0)}"
    )
    lines.append(
        f"  edge (xmin>0.85 or xmax<0.15)={d.get('n_edge', 0)}  "
        f"top-clipped ymin<=0.01={d.get('n_top_clipped', 0)}"
    )
    lines.append(
        f"track id changes on the followed target: {d.get('n_target_track_id_changes', 0)}"
    )

    s = m.get("steering") or {}
    lines += ["", "=== STEERING ==="]
    lines.append(
        f"|steer_offset| p50={_f(s.get('abs_steer_p50'), '.2f')}  "
        f"p90={_f(s.get('abs_steer_p90'), '.2f')}  max={_f(s.get('abs_steer_max'), '.2f')}"
    )
    lines.append(
        f"frac |steer|>=0.95*direct_cap({_f(s.get('direct_cap'), '.1f')}): "
        f"{_f(s.get('frac_at_direct_cap'))}"
    )
    lines.append(
        f"time |bearing|>15 deg: {_f(s.get('time_bearing_gt_15_s'), '.2f')} s   "
        f">25 deg: {_f(s.get('time_bearing_gt_25_s'), '.2f')} s"
    )
    evs = s.get("bearing_events") or []
    if not evs:
        lines.append("bearing events: (none)")
    else:
        lines.append("bearing events:")
        for ev in evs:
            lines.append(
                f"  t={_f(ev.get('t_start'), '.2f')}..{_f(ev.get('t_end'), '.2f')} "
                f"dur={_f(ev.get('duration_s'), '.2f')}s  end={ev.get('end_reason') or 'n/a'}  "
                f"bearing0={_f(ev.get('bearing_start_deg'), '.1f')} deg"
            )

    def _yaw_line(label: str, fit: dict) -> None:
        lag_ticks = fit.get("lag_ticks")
        lines.append(
            f"yaw vs (L-R) {label} from heading_deg: "
            f"slope={_f(fit.get('slope'), '.4f')} deg/s/byte  "
            f"lag={lag_ticks if lag_ticks is not None else 'n/a'} ticks "
            f"({_f(fit.get('lag_s'), '.2f')} s)  "
            f"int={_f(fit.get('intercept'), '.3f')}  r={_f(fit.get('r'), '.3f')}  "
            f"n={fit.get('n', 0)}"
        )
        bins = fit.get("bins") or []
        if not bins:
            lines.append("  bins: n/a")
        else:
            parts = [
                f"[{_f(b['lr_lo'], '.0f')},{_f(b['lr_hi'], '.0f')})="
                f"{_f(b.get('mean_yaw_dps'), '.2f')}n={b.get('n', 0)}"
                for b in bins
            ]
            lines.append("  bins: " + " ".join(parts))
        logged = fit.get("logged") or {}
        n_zero = logged.get("n_zero", 0)
        n_samp = logged.get("n_samples", logged.get("n", 0))
        lines.append(
            f"  logged yaw_rate_dps (unreliable: {n_zero} of {n_samp} samples "
            f"are 0.0): slope={_f(logged.get('slope'), '.4f')} deg/s/byte  "
            f"int={_f(logged.get('intercept'), '.3f')}  "
            f"r={_f(logged.get('r'), '.3f')}  n={logged.get('n', 0)}"
        )

    _yaw_line("moving (actual_mps>=0.3)", s.get("yaw_moving") or {})
    _yaw_line("pivot (actual_mps<0.15)", s.get("yaw_pivot") or {})

    c = m.get("speed_cap") or {}
    lines += ["", "=== SPEED CAP ==="]
    lines.append(
        f"frac speed_offset>=max_follow_speed_byte({_f(c.get('max_follow_speed_byte'), '.1f')})-0.5: "
        f"{_f(c.get('frac_at_speed_cap'))}"
    )
    lines.append(
        f"max left_rpm={_f(c.get('max_left_rpm'), '.0f')} l_duty={_f(c.get('l_duty_at_max_left_rpm'), '.3f')}  "
        f"max right_rpm={_f(c.get('max_right_rpm'), '.0f')} r_duty={_f(c.get('r_duty_at_max_right_rpm'), '.3f')}"
    )
    lines.append(
        f"mean eRPM per unit duty (|eRPM|>8000, n={c.get('n_erpm_gt_8000', 0)}): "
        f"{_f(c.get('mean_erpm_per_duty'), '.1f')}"
    )

    o = m.get("obstacle") or {}
    lines += ["", "=== OBSTACLE ==="]
    ticks_o = o.get("ticks") or []
    if not ticks_o:
        lines.append("throttle_scale<1.0 ticks: (none)")
    else:
        lines.append("throttle_scale<1.0 ticks:")
        for r in ticks_o:
            lines.append(
                f"  t={_f(r.get('t'), '.2f')} thr={_f(r.get('throttle_scale'))} "
                f"dist={_f(r.get('distance_m'))} p5={_f(r.get('depth_p5_mm'), '.1f')} "
                f"valid%={_f(r.get('depth_valid_pct'))} z={_f(r.get('target_z_m'))}"
            )
    lines.append(
        f"depth_valid_pct<8.0: {o.get('n_depth_valid_pct_lt_8', 0)}  "
        f"runs={o.get('n_runs', 0)}  longest={o.get('longest_run', 0)}"
    )

    pd = m.get("person_depth") or {}
    lines += ["", "=== PERSON DEPTH ==="]
    counts = pd.get("status_counts")
    if not counts:
        lines.append("depth_status: n/a")
    else:
        parts = [f"{k}={v}" for k, v in sorted(counts.items())]
        lines.append("depth_status: " + " ".join(parts))
    jumps = pd.get("n_z_stereo_jumps_same_track")
    lines.append(
        f"z_stereo_m jumps >1.0 m (same track_id): "
        f"{jumps if jumps is not None else 'n/a'}"
    )
    lines.append(f"median depth_valid_px: {_f(pd.get('median_depth_valid_px'), '.0f')}")
    lines.append(
        f"median det_fps: {_f(pd.get('median_det_fps'))}  "
        f"median depth_fps: {_f(pd.get('median_depth_fps'))}"
    )
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", help="Structured JSON log file(s)")
    ap.add_argument("--bin", type=float, default=DEFAULT_BIN_S, dest="bin_s",
                    metavar="SECONDS", help="Timeline bin width (default 2)")
    ap.add_argument("--start", type=float, default=None, dest="start_s",
                    metavar="S", help="Seconds from first FOLLOW_ME tick")
    ap.add_argument("--end", type=float, default=None, dest="end_s",
                    metavar="S", help="Seconds from first FOLLOW_ME tick")
    ap.add_argument("--json", type=Path, default=None, dest="json_path",
                    metavar="PATH", help="Write every computed metric as one JSON object")
    ap.add_argument("--direct-cap", type=float, default=None,
                    dest="direct_cap", metavar="BYTES",
                    help="Direct-mode steer cap in bytes (default: session "
                         "header follow_me.direct_mode_max_steer_byte, else 32)")
    args = ap.parse_args(argv)
    for p in args.logs:
        if not Path(p).is_file():
            print(f"File not found: {p}", file=sys.stderr)
            return 1
    metrics = analyze(
        args.logs, bin_s=args.bin_s, start_s=args.start_s,
        end_s=args.end_s, direct_cap=args.direct_cap,
    )
    if args.json_path is not None:
        args.json_path.parent.mkdir(parents=True, exist_ok=True)
        args.json_path.write_text(json.dumps(metrics, indent=2, default=str) + "\n", encoding="utf-8")
    print(render_report(metrics), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
