#!/usr/bin/env python3
"""Cost function for a single Follow-Me steering trial.

Reads a per-tick trial-recorder JSONL file (one file per FM engagement, written
by ``FollowMeController._record_tick`` to ``/tmp/fm_trials/<engagement_ts>.jsonl``)
and reduces it to a single scalar cost ``J``. Lower is better. The GP-BO
auto-tuner (``tools/fm_autotune.py``) minimises this.

Each JSONL line is a tick:
    {t, x_raw, x_filt, x_err, steer, speed, mode, track_id, depth, conf}
where ``mode`` is one of "direct" (lateral-PID pursuit), "pp" (pure-pursuit
trail following), "search", or "lost". ``steer`` is in motor-byte units, range
±STEER_MAX_BYTE.

The cost is::

    J = W_ERROR  * sum(x_err^2 * dt)                            # tracking error
      + W_THRASH * sum((d_steer_norm / dt)^2 * dt)              # steering effort
      + W_OSC    * max(0, zero_crossings - direction_changes)   # hunting / wobble
      + W_LOST   * lost_track                                    # lost the person

Usage:
    python tools/fm_score.py <trial.jsonl> [--json]

Dependencies: stdlib + numpy only (numpy is already a service dependency).
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Cost weights (rationale)
# ─────────────────────────────────────────────────────────────────────────────
# W_ERROR (1.0)  — baseline. Integrated squared lateral error (normalised offset,
#                  -1..+1) is the thing we actually care about: keep the person
#                  centred. Everything else is scaled relative to this.
# W_THRASH (0.1) — penalise steering effort / jerk. d_steer is normalised to
#                  [-1,1] (÷STEER_MAX_BYTE) so a full-scale slam per second costs
#                  ~0.1 — about a tenth of holding 1.0 full-scale error for a
#                  second. Discourages buzzy actuator chatter without dominating.
# W_OSC (0.5)    — penalise *hunting*: every zero-crossing of the filtered error
#                  beyond the ones explained by the person genuinely changing
#                  direction (the 1 s-smoothed signal) is overshoot oscillation.
#                  0.5 per excess crossing makes a visibly wobbly run clearly
#                  worse than a smooth one.
# W_LOST (50.0)  — dominating penalty. Losing the target (mode entered "lost" or
#                  "search") is a hard failure of the controller; one such trial
#                  should always score worse than any merely-imperfect one.
W_ERROR = 1.0
W_THRASH = 0.1
W_OSC = 0.5
W_LOST = 50.0

# steer JSONL field is in motor-byte units spanning ±this (config.max_steer_offset_byte).
STEER_MAX_BYTE = 25.0

# A gap between consecutive ticks longer than this is treated as a discontinuity
# (recorder paused, control loop stalled) and clamped, so one big dt can't swamp
# the integral. Nominal FM output cadence is ~15 Hz (~0.067 s).
DT_MAX_S = 1.0
DT_NOMINAL_S = 1.0 / 15.0

# Modes that count as "still controlling toward the person".
_LOST_MODES = ("lost", "search")


def parse_jsonl(path: str) -> tuple[list[dict], int]:
    """Parse a trial JSONL file.

    Returns (records, n_malformed). Lines that are blank, non-JSON, not an
    object, or missing the timestamp ``t`` are skipped and counted as malformed
    rather than raising — partial/corrupt trial files must still score.
    """
    records: list[dict] = []
    malformed = 0
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except (ValueError, json.JSONDecodeError):
                malformed += 1
                continue
            if not isinstance(obj, dict) or "t" not in obj:
                malformed += 1
                continue
            try:
                float(obj["t"])
            except (TypeError, ValueError):
                malformed += 1
                continue
            records.append(obj)
    return records, malformed


def _count_sign_changes(values: list[float]) -> int:
    """Count sign changes in a sequence, ignoring exact zeros.

    A run of zeros does not count; only a flip between a nonzero negative and a
    nonzero positive (in either order) increments the count.
    """
    last_sign = 0
    changes = 0
    for v in values:
        if v > 0:
            sign = 1
        elif v < 0:
            sign = -1
        else:
            sign = 0
        if sign != 0:
            if last_sign != 0 and sign != last_sign:
                changes += 1
            last_sign = sign
    return changes


def _smooth_1s(times: list[float], values: list[float]) -> list[float]:
    """Centred ~1 s moving average of ``values`` over the timeline ``times``.

    For each sample, average all samples within ±0.5 s. Used to recover the
    *intended* direction of the target (slow trend) so genuine direction
    reversals can be told apart from fast wobble.
    """
    t = np.asarray(times, dtype=float)
    v = np.asarray(values, dtype=float)
    out: list[float] = []
    for ti in t:
        mask = np.abs(t - ti) <= 0.5
        out.append(float(np.mean(v[mask])))
    return out


def score_trial(records: list[dict], n_malformed: int = 0) -> dict:
    """Compute the cost breakdown for a list of parsed trial ticks.

    Terms 1 & 2 (tracking error, steering thrash) are scored over **direct-mode
    ticks only** — we don't score the pure-pursuit (pp) path. Term 3 (hunting)
    uses the filtered error across the whole trial; term 4 (lost) flags whether
    the target was ever lost during the trial.

    Returns a dict with each term, the total ``J``, and diagnostic counts.
    """
    # Sort defensively by timestamp (recorder appends in order, but be safe).
    recs = sorted(records, key=lambda r: float(r["t"]))
    n = len(recs)

    times = [float(r["t"]) for r in recs]
    modes = [r.get("mode") for r in recs]

    # Representative forward dt per tick: duration this sample "held" until the
    # next. Clamp gaps; last sample gets the nominal cadence.
    diffs = [t2 - t1 for t1, t2 in zip(times, times[1:])]
    positive = [d for d in diffs if d > 0]
    dt_nom = float(np.median(positive)) if positive else DT_NOMINAL_S
    dt_fwd: list[float] = []
    for i in range(n):
        if i < n - 1:
            d = times[i + 1] - times[i]
            if d <= 0 or d > DT_MAX_S:
                d = dt_nom
        else:
            d = dt_nom
        dt_fwd.append(d)

    # ── Term 1: integrated squared tracking error (direct ticks only) ──
    # x_err is the post-deadband error when the recorder populated it; fall back
    # to x_filt (filtered offset) when x_err is absent (None).
    error_term = 0.0
    n_direct = 0
    for i in range(n):
        if modes[i] != "direct":
            continue
        n_direct += 1
        err = recs[i].get("x_err")
        if err is None:
            err = recs[i].get("x_filt")
        if err is None:
            continue
        try:
            err = float(err)
        except (TypeError, ValueError):
            continue
        error_term += err * err * dt_fwd[i]

    # ── Term 2: steering thrash, (d_steer_norm/dt)^2 * dt, between consecutive
    # direct ticks (so we measure effort within direct control, not pp seams) ──
    thrash_term = 0.0
    for i in range(n - 1):
        if modes[i] != "direct" or modes[i + 1] != "direct":
            continue
        s0 = recs[i].get("steer")
        s1 = recs[i + 1].get("steer")
        if s0 is None or s1 is None:
            continue
        try:
            ds = (float(s1) - float(s0)) / STEER_MAX_BYTE
        except (TypeError, ValueError):
            continue
        dt = times[i + 1] - times[i]
        if dt <= 0 or dt > DT_MAX_S:
            dt = dt_nom
        # (ds/dt)^2 * dt == ds^2 / dt
        thrash_term += (ds * ds) / dt

    # ── Term 3: hunting / wobble (whole trial) ──
    # Zero-crossings of the filtered error beyond those explained by the
    # 1 s-smoothed signal's genuine direction reversals.
    xf_times: list[float] = []
    xf_vals: list[float] = []
    for i in range(n):
        xf = recs[i].get("x_filt")
        if xf is None:
            continue
        try:
            xf_vals.append(float(xf))
            xf_times.append(times[i])
        except (TypeError, ValueError):
            continue
    if len(xf_vals) >= 2:
        zero_crossings = _count_sign_changes(xf_vals)
        direction_changes = _count_sign_changes(_smooth_1s(xf_times, xf_vals))
    else:
        zero_crossings = 0
        direction_changes = 0
    excess_crossings = max(0, zero_crossings - direction_changes)
    osc_term = float(excess_crossings)

    # ── Term 4: lost target (whole trial) ──
    lost_track = 1 if any(m in _LOST_MODES for m in modes) else 0

    total = (
        W_ERROR * error_term
        + W_THRASH * thrash_term
        + W_OSC * osc_term
        + W_LOST * lost_track
    )

    return {
        "J": total,
        "terms": {
            "error": W_ERROR * error_term,
            "thrash": W_THRASH * thrash_term,
            "oscillation": W_OSC * osc_term,
            "lost": W_LOST * lost_track,
        },
        "raw": {
            "error_integral": error_term,
            "thrash_integral": thrash_term,
            "zero_crossings": zero_crossings,
            "direction_changes": direction_changes,
            "excess_crossings": excess_crossings,
            "lost_track": lost_track,
        },
        "diagnostics": {
            "n_records": n,
            "n_direct": n_direct,
            "n_malformed": n_malformed,
            "duration_s": (times[-1] - times[0]) if n >= 2 else 0.0,
        },
    }


def score_file(path: str) -> dict:
    """Parse + score a trial file in one call (used by the auto-tuner)."""
    records, malformed = parse_jsonl(path)
    return score_trial(records, n_malformed=malformed)


def _format_human(result: dict, path: str) -> str:
    t = result["terms"]
    raw = result["raw"]
    d = result["diagnostics"]
    lines = [
        f"Follow-Me trial score — {path}",
        "─" * 56,
        f"  ticks: {d['n_records']}  (direct: {d['n_direct']}, "
        f"malformed skipped: {d['n_malformed']}, dur: {d['duration_s']:.1f}s)",
        "",
        f"  1. tracking error   J_err    = {t['error']:.4f}   "
        f"(∫x_err² dt = {raw['error_integral']:.4f}, w={W_ERROR})",
        f"  2. steering thrash  J_thrash = {t['thrash']:.4f}   "
        f"(∫(dṡ)² dt = {raw['thrash_integral']:.4f}, w={W_THRASH})",
        f"  3. hunting/wobble   J_osc    = {t['oscillation']:.4f}   "
        f"(excess crossings = {raw['excess_crossings']} "
        f"[zc={raw['zero_crossings']} - dc={raw['direction_changes']}], w={W_OSC})",
        f"  4. lost target      J_lost   = {t['lost']:.4f}   "
        f"(lost_track = {raw['lost_track']}, w={W_LOST})",
        "─" * 56,
        f"  TOTAL  J = {result['J']:.4f}   (lower is better)",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Score a Follow-Me trial JSONL file (lower J is better)."
    )
    parser.add_argument("trial", help="path to a trial JSONL file")
    parser.add_argument("--json", action="store_true",
                        help="emit machine-readable JSON instead of a report")
    args = parser.parse_args(argv)

    try:
        records, malformed = parse_jsonl(args.trial)
    except FileNotFoundError:
        print(f"error: no such file: {args.trial}", file=sys.stderr)
        return 2

    result = score_trial(records, n_malformed=malformed)

    if args.json:
        print(json.dumps(result))
    else:
        print(_format_human(result, args.trial))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
