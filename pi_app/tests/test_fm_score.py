"""Tests for tools/fm_score.py — the Follow-Me trial cost function."""

import json
import math
import os
import sys

import pytest

# tools/ is at the repo root, two levels up from pi_app/tests/.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "tools"))

import fm_score  # noqa: E402


def _write_jsonl(tmp_path, lines):
    p = tmp_path / "trial.jsonl"
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def _tick(t, x, steer, mode="direct", **extra):
    rec = {
        "t": t, "x_raw": x, "x_filt": round(x, 4), "x_err": round(x, 4),
        "steer": round(steer, 3), "speed": 40.0, "mode": mode,
        "track_id": 1, "depth": 1.5, "conf": 0.9,
    }
    rec.update(extra)
    return json.dumps(rec)


def _converging_lines(n=90, dt=1 / 15):
    """Smooth exponential decay to centre, steer tracks error — no oscillation."""
    out = []
    t = 0.0
    for i in range(n):
        x = 0.5 * math.exp(-i / 20.0)
        out.append(_tick(t, x, x * fm_score.STEER_MAX_BYTE))
        t += dt
    return out


def _oscillating_lines(n=90, dt=1 / 15):
    """Fast sine wobble about centre with thrashing steer — lots of crossings."""
    out = []
    t = 0.0
    for i in range(n):
        x = 0.5 * math.sin(2 * math.pi * i / 8.0)  # ~2 Hz hunting
        out.append(_tick(t, x, x * fm_score.STEER_MAX_BYTE))
        t += dt
    return out


def test_oscillating_costs_at_least_3x_converging(tmp_path):
    pa = tmp_path / "conv.jsonl"
    pa.write_text("\n".join(_converging_lines()) + "\n")
    pb = tmp_path / "osc.jsonl"
    pb.write_text("\n".join(_oscillating_lines()) + "\n")

    Ja = fm_score.score_file(str(pa))["J"]
    Jb = fm_score.score_file(str(pb))["J"]

    assert Ja > 0
    assert Jb >= 3 * Ja, f"expected J_osc >= 3*J_conv, got {Jb:.4f} vs {Ja:.4f}"


def test_search_mode_adds_lost_penalty(tmp_path):
    lines = _converging_lines(n=30)
    # Inject a tick where the controller entered search.
    lines.append(_tick(30 / 15, 0.0, 0.0, mode="search"))
    path = _write_jsonl(tmp_path, lines)

    result = fm_score.score_file(path)
    assert result["raw"]["lost_track"] == 1
    assert result["terms"]["lost"] == pytest.approx(fm_score.W_LOST)
    assert result["J"] >= fm_score.W_LOST


def test_lost_mode_adds_lost_penalty(tmp_path):
    lines = _converging_lines(n=20)
    lines.append(_tick(20 / 15, 0.0, 0.0, mode="lost"))
    path = _write_jsonl(tmp_path, lines)
    assert fm_score.score_file(path)["raw"]["lost_track"] == 1


def test_malformed_lines_skipped_without_crash(tmp_path):
    good = _converging_lines(n=10)
    lines = []
    lines.append(good[0])
    lines.append("{ this is not json")          # malformed
    lines.append("")                             # blank (ignored, not counted)
    lines.append("[1, 2, 3]")                    # JSON but not an object
    lines.append(json.dumps({"x_filt": 0.1}))    # object missing "t"
    lines.extend(good[1:])
    p = tmp_path / "messy.jsonl"
    p.write_text("\n".join(lines) + "\n")

    result = fm_score.score_file(str(p))
    # 3 malformed counted: bad-json, non-object [1,2,3], object-missing-"t".
    # The blank line is ignored, not counted.
    assert result["diagnostics"]["n_malformed"] == 3
    assert result["diagnostics"]["n_records"] == 10
    assert math.isfinite(result["J"])


def test_empty_file(tmp_path):
    p = tmp_path / "empty.jsonl"
    p.write_text("")
    result = fm_score.score_file(str(p))
    assert result["diagnostics"]["n_records"] == 0
    assert result["J"] == 0.0


def test_single_line(tmp_path):
    p = tmp_path / "one.jsonl"
    p.write_text(_tick(0.0, 0.3, 7.5) + "\n")
    result = fm_score.score_file(str(p))
    assert result["diagnostics"]["n_records"] == 1
    assert math.isfinite(result["J"])
    assert result["raw"]["lost_track"] == 0


def test_pp_ticks_not_scored_for_error_term(tmp_path):
    """Terms 1-2 use direct ticks only; a pp-mode run scores ~0 error/thrash."""
    lines = []
    t = 0.0
    for i in range(40):
        lines.append(_tick(t, 0.4, 10.0, mode="pp"))  # large constant error, but pp
        t += 1 / 15
    path = _write_jsonl(tmp_path, lines)
    result = fm_score.score_file(path)
    assert result["raw"]["error_integral"] == 0.0
    assert result["raw"]["thrash_integral"] == 0.0
    assert result["diagnostics"]["n_direct"] == 0


def test_x_err_fallback_to_x_filt(tmp_path):
    """When x_err is absent, the error term falls back to x_filt."""
    lines = []
    t = 0.0
    for i in range(20):
        rec = {"t": t, "x_filt": 0.3, "steer": 7.5, "mode": "direct"}  # no x_err
        lines.append(json.dumps(rec))
        t += 1 / 15
    path = _write_jsonl(tmp_path, lines)
    result = fm_score.score_file(path)
    # 0.3^2 * ~(20 * 1/15) ≈ 0.12, definitely > 0
    assert result["raw"]["error_integral"] > 0.0
