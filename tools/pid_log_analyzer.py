#!/usr/bin/env python3
"""
Compact PID-focused log analyzer for WALL-E Mini.

Design goals:
- Produce a tiny, LLM-ready NDJSON in logs/ filtering only when PID is active.
- Ignore BT content entirely.
- Use short, integer-encoded columns with a single header describing schema/scales.
- Add a small summary line with key metrics at the end.
- Do not modify existing project files; can be called after main exits.

Output formats (compact):
1) json (default): single JSON object {meta, rows, summary}
2) ndjson: header line, rows per line, final summary line

Where row columns are:
- dt: elapsed milliseconds since first included sample
- e: heading error (deg) × scale["e"] (int)
- r: yaw rate (deg/s) × scale["r"] (int)
- p,i,d: PID component contributions (byte-equivalent), already ints in logs
- u: applied correction (after clamp/invert), int bytes
- s: steering input (normalized −1..1) × scale["s"] (int)
- sat: 1 if PID output saturated/clamped, else 0
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional, Tuple
import sys


def _repo_root_from_this_file() -> Path:
    return Path(__file__).resolve().parents[1]


def _import_config():
    # Allow running from repo root or from anywhere
    root = _repo_root_from_this_file()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    try:
        from config import config  # type: ignore
        return config
    except Exception:
        return None


@dataclass
class Scales:
    e: int = 10   # 0.1 deg
    r: int = 10   # 0.1 deg/s
    s: int = 100  # 0.01 normalized

    def to_dict(self) -> dict:
        return {"e": self.e, "r": self.r, "s": self.s}


def _parse_ts_iso(ts_iso: str) -> Optional[datetime]:
    try:
        return datetime.strptime(ts_iso, "%Y-%m-%d %H:%M:%S.%f")
    except Exception:
        try:
            # Fallback without fractional part
            return datetime.strptime(ts_iso, "%Y-%m-%d %H:%M:%S")
        except Exception:
            return None


def _angle_wrap_deg(a: float) -> float:
    x = (a + 180.0) % 360.0 - 180.0
    # Map -180 exclusive edge back to 180 for stability
    return 180.0 if x == -180.0 else x


def _is_num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def _pid_active(obj: dict) -> bool:
    # Active when IMU available and a correction is actually applied
    imu = obj.get("imu") or {}
    steer = obj.get("imu_steering") or {}
    if not isinstance(imu, dict) or not imu.get("is_available", False):
        return False
    u = steer.get("correction_applied")
    return _is_num(u)


def _sat_flag(steer: dict, max_corr: Optional[int]) -> int:
    raw = steer.get("correction_raw")
    app = steer.get("correction_applied")
    if _is_num(raw) and _is_num(app) and int(raw) != int(app):
        return 1
    if max_corr is not None and _is_num(raw) and abs(float(raw)) >= float(max_corr):
        return 1
    return 0


def _derive_row(obj: dict, scales: Scales, max_corr: Optional[int]) -> Optional[Tuple[int, int, int, int, int, int, int, int, int]]:
    if not _pid_active(obj):
        return None
    ts_iso = obj.get("ts_iso")
    imu = obj.get("imu") or {}
    steer = obj.get("imu_steering") or {}
    pid = obj.get("pid") or {}

    # Error in degrees: prefer logged pid.error_deg; fall back to target-heading
    e = pid.get("error_deg")
    if not _is_num(e):
        hd = imu.get("heading_deg")
        td = imu.get("target_heading_deg")
        if _is_num(hd) and _is_num(td):
            e = _angle_wrap_deg(float(td) - float(hd))
        else:
            return None
    r = imu.get("yaw_rate_dps", 0.0)
    p = pid.get("p", 0.0)
    i = pid.get("i", 0.0)
    d = pid.get("d", 0.0)
    u = steer.get("correction_applied")
    s = obj.get("imu_steering", {}).get("steering_input", 0.0)
    sat = _sat_flag(steer, max_corr)

    # Quantize
    e_q = int(round(float(e) * scales.e))
    r_q = int(round(float(r) * scales.r))
    p_q = int(round(float(p)))
    i_q = int(round(float(i)))
    d_q = int(round(float(d)))
    u_q = int(round(float(u)))
    s_q = int(round(float(s) * scales.s))

    # Return with placeholder dt (0). Caller fills dt sequence.
    return (0, e_q, r_q, p_q, i_q, d_q, u_q, s_q, int(sat))


def _zero_crossings_per_second(times_ms: Iterable[int], errors_q: Iterable[int], scale_e: int) -> float:
    # Count zero-crossings of error to estimate oscillation frequency
    times = list(times_ms)
    errs = [x / float(scale_e) for x in errors_q]
    if len(times) < 3:
        return 0.0
    crossings = []
    prev_sign = 1 if errs[0] > 0 else (-1 if errs[0] < 0 else 0)
    prev_t = times[0]
    for t, e in zip(times[1:], errs[1:]):
        sgn = 1 if e > 0 else (-1 if e < 0 else 0)
        if sgn != 0 and prev_sign != 0 and sgn != prev_sign:
            crossings.append(t)
        if sgn != 0:
            prev_sign = sgn
        prev_t = t
    if len(crossings) < 2:
        return 0.0
    # Period estimated by differences between crossings (half-cycles)
    diffs_ms = [b - a for a, b in zip(crossings, crossings[1:])]
    if not diffs_ms:
        return 0.0
    mean_ms = sum(diffs_ms) / len(diffs_ms)
    if mean_ms <= 0:
        return 0.0
    # Two crossings ~ one full period
    period_s = (mean_ms * 2.0) / 1000.0
    return 1.0 / period_s if period_s > 0 else 0.0


def analyze(log_path: Path, out_path: Optional[Path] = None, fmt: str = "json") -> Optional[Path]:
    cfg = _import_config()
    pid_debug_enabled = bool(getattr(getattr(cfg, "imu_steering", cfg), "log_steering_corrections", False) or getattr(cfg, "log_steering_corrections", False))
    if not pid_debug_enabled:
        # Respect request to run only when PID debugging is enabled
        return None

    scales = Scales()
    max_corr = None
    kp = ki = kd = None
    if cfg is not None and getattr(cfg, "imu_steering", None) is not None:
        max_corr = int(getattr(cfg.imu_steering, "max_correction", 200))
        kp = float(getattr(cfg.imu_steering, "kp", 0.0))
        ki = float(getattr(cfg.imu_steering, "ki", 0.0))
        kd = float(getattr(cfg.imu_steering, "kd", 0.0))

    if not log_path.exists():
        return None

    # Prepare output path
    if out_path is None:
        name = log_path.name
        stem = name
        if name.startswith("run_") and name.endswith(".log"):
            stem = name[len("run_") : -len(".log")]
        suffix = ".json" if fmt == "json" else ".ndjson"
        out_path = log_path.with_name(f"pid_{stem}{suffix}")

    # Stream parse and filter
    rows: list[Tuple[int, int, int, int, int, int, int, int, int]] = []
    t0 = None
    times_ms: list[int] = []
    errors_q: list[int] = []
    with log_path.open("r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            # Only consider JSON lines; logs are JSONL
            if not (line.startswith("{") and line.endswith("}")):
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue
            row = _derive_row(obj, scales, max_corr)
            if row is None:
                continue
            ts_iso = obj.get("ts_iso")
            dt_ms = None
            if isinstance(ts_iso, str):
                ts = _parse_ts_iso(ts_iso)
                if ts is not None:
                    if t0 is None:
                        t0 = ts
                    dt_ms = int(round((ts - t0).total_seconds() * 1000))
            if dt_ms is None:
                # Fallback: grow by 100 ms increments if timestamp missing
                dt_ms = (times_ms[-1] + 100) if times_ms else 0
            # Fill dt component
            row = (dt_ms,) + row[1:]
            rows.append(row)
            times_ms.append(dt_ms)
            errors_q.append(row[1])

    if not rows:
        return None

    # Compute compact summary metrics
    abs_errors = [abs(x / float(scales.e)) for x in errors_q]
    mean_abs_e = sum(abs_errors) / len(abs_errors)
    rms_e = math.sqrt(sum((x * x) for x in abs_errors) / len(abs_errors))
    max_abs_e = max(abs_errors)
    sat_ratio = sum(r[-1] for r in rows) / float(len(rows))
    zc_hz = _zero_crossings_per_second(times_ms, errors_q, scales.e)

    meta_obj = {
        "columns": ["dt","e","r","p","i","d","u","s","sat"],
        "units": {"dt": "ms", "e": "deg", "r": "deg/s", "p": "byte", "i": "byte", "d": "byte", "u": "byte", "s": "norm", "sat": "0/1"},
        "scale": scales.to_dict(),
        "pid": {"kp": kp, "ki": ki, "kd": kd},
        "hint": "LLM: dt(ms), e(deg), r(deg/s), p,i,d(terms), u(applied), s(steer -1..1), sat(1 if clamped)."
    }
    summary_obj = {
        "n": len(rows),
        "rms_e": round(rms_e, 3),
        "mean_abs_e": round(mean_abs_e, 3),
        "max_abs_e": round(max_abs_e, 3),
        "sat_ratio": round(sat_ratio, 3),
        "zc_per_s": round(zc_hz, 3),
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as out:
        if fmt == "json":
            payload = {
                "meta": meta_obj,
                "rows": rows,
                "summary": summary_obj,
            }
            out.write(json.dumps(payload, separators=(",", ":")))
        else:
            out.write(json.dumps({"meta": meta_obj}, separators=(",", ":")) + "\n")
            for r in rows:
                out.write("[" + ",".join(str(x) for x in r) + "]\n")
            out.write(json.dumps({"summary": summary_obj}, separators=(",", ":")) + "\n")

    return out_path


def main():
    root = _repo_root_from_this_file()
    logs_dir = root / "logs"
    parser = argparse.ArgumentParser(description="Compact PID log analyzer (WALL-E Mini)")
    parser.add_argument("--log", type=str, default=str(logs_dir / "latest.log"), help="Path to input JSONL log (default: logs/latest.log)")
    parser.add_argument("--out", type=str, default=None, help="Output path (default by format)")
    parser.add_argument("--format", choices=["json", "ndjson"], default="json", help="Output format (default: json)")
    args = parser.parse_args()
    log_path = Path(args.log)
    out_path = Path(args.out) if args.out else None

    res = analyze(log_path, out_path, fmt=args.format)
    if res is None:
        # Either pid debugging disabled or no data
        return 0
    print(str(res))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
