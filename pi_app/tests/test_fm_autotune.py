"""Tests for tools/fm_autotune.py — GP-BO ask-tell plumbing.

The optimiser is mocked, so scikit-optimize is NOT required to run these. The
real fm_score scoring, state persistence, trial discovery, and seed/resume
control flow are all exercised.
"""

import json
import math
import os
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "tools"))

import fm_autotune as at  # noqa: E402


# ── helpers ──────────────────────────────────────────────────────────────────

class MockOptimizer:
    """Records ask()/tell() so the loop's control flow can be asserted."""
    def __init__(self, asks):
        self._asks = [list(a) for a in asks]
        self.ask_calls = 0
        self.told = []  # list of (x, J)

    def ask(self):
        x = self._asks[self.ask_calls]
        self.ask_calls += 1
        return list(x)

    def tell(self, x, J):
        self.told.append((list(x), J))


def _write_converging_trial(path):
    lines = []
    t = 0.0
    for i in range(40):
        x = 0.5 * math.exp(-i / 20.0)
        lines.append(json.dumps({"t": t, "x_filt": round(x, 4), "x_err": round(x, 4),
                                 "steer": round(x * 25, 3), "mode": "direct"}))
        t += 1 / 15
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def _write_lost_trial(path):
    lines = [json.dumps({"t": 0.0, "x_filt": 0.1, "x_err": 0.1, "steer": 2.0, "mode": "direct"}),
             json.dumps({"t": 0.1, "x_filt": None, "x_err": None, "steer": 0.0, "mode": "lost"})]
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


# ── vector <-> params helpers ─────────────────────────────────────────────────

def test_params_list_roundtrip():
    params = dict(at.SHIPPED_VALUES)
    x = at.list_from_params(params)
    assert x == [params[k] for k in at.PARAM_ORDER]
    assert at.params_from_list(x) == params


# ── trial discovery ───────────────────────────────────────────────────────────

def test_find_newest_trial_picks_newest_after_ts(tmp_path):
    old = tmp_path / "100.jsonl"
    old.write_text("{}\n")
    os.utime(str(old), (100, 100))
    new = tmp_path / "200.jsonl"
    new.write_text("{}\n")
    os.utime(str(new), (200, 200))

    # after_ts between the two → only the newer qualifies.
    assert at.find_newest_trial(str(tmp_path), after_ts=150) == str(new)
    # after_ts before both → newest of the two.
    assert at.find_newest_trial(str(tmp_path), after_ts=50) == str(new)
    # after_ts after both → nothing.
    assert at.find_newest_trial(str(tmp_path), after_ts=250) is None


def test_find_newest_trial_empty_dir(tmp_path):
    assert at.find_newest_trial(str(tmp_path), after_ts=0) is None


# ── scoring / penalties ────────────────────────────────────────────────────────

def test_score_or_penalty_missing_file():
    J, result, warn = at.score_or_penalty(None)
    assert J == at.MISSING_TRIAL_PENALTY
    assert result is None
    assert warn


def test_score_or_penalty_empty_file(tmp_path):
    p = tmp_path / "empty.jsonl"
    p.write_text("")
    J, result, warn = at.score_or_penalty(str(p))
    assert J == at.MISSING_TRIAL_PENALTY
    assert "empty" in warn


def test_score_or_penalty_lost_trial_gets_50(tmp_path):
    p = tmp_path / "lost.jsonl"
    _write_lost_trial(str(p))
    J, result, warn = at.score_or_penalty(str(p))
    assert warn is None
    assert J >= 50.0  # lost-track term
    assert result["raw"]["lost_track"] == 1


# ── state persistence ───────────────────────────────────────────────────────────

def test_state_save_load_roundtrip(tmp_path):
    p = str(tmp_path / "state.json")
    state = [{"iteration": 1, "x": [0.4, 0.2, 0.5, 0.04, 0.1], "params": {}, "J": 1.2}]
    at.save_state(p, state)
    assert at.load_state(p) == state


def test_load_state_missing_returns_empty(tmp_path):
    assert at.load_state(str(tmp_path / "nope.json")) == []


# ── the ask-tell loop ───────────────────────────────────────────────────────────

def _run_loop(tmp_path, n, asks, trial_writers, resume=False, state_path=None):
    """Drive autotune() with a mock optimiser and prepared trial files."""
    trials_dir = tmp_path / "trials"
    trials_dir.mkdir(exist_ok=True)
    state_path = state_path or str(tmp_path / "state.json")

    opt = MockOptimizer(asks)
    posted = []
    # Each trial: write its file, then hand its path to the loop.
    counter = {"i": 0}

    def fake_find(_dir, _after):
        i = counter["i"]
        counter["i"] += 1
        writer = trial_writers[i]
        if writer is None:
            return None
        path = str(trials_dir / f"trial_{i}.jsonl")
        writer(path)
        return path

    def fake_post(robot, params):
        posted.append(dict(params))
        return {"ok": True, "applied": params}

    times = iter(range(1000))
    best = at.autotune(
        opt, n=n, robot="x:8080", trials_dir=str(trials_dir),
        state_path=state_path, resume=resume,
        prompt_fn=lambda *a, **k: None,
        find_trial_fn=fake_find,
        post_fn=fake_post,
        now_fn=lambda: next(times),
        out=lambda *a, **k: None,
    )
    return opt, posted, best, state_path


def test_loop_seeds_trial1_with_shipped_then_asks(tmp_path):
    asks = [[0.5, 0.3, 0.6, 0.05, 0.2], [0.6, 0.1, 0.7, 0.02, 0.15]]
    writers = [_write_converging_trial] * 3
    opt, posted, best, state_path = _run_loop(tmp_path, n=3, asks=asks,
                                              trial_writers=writers)

    # Trial 1 used the shipped seed (not ask); ask() called only for 2 & 3.
    assert opt.ask_calls == 2
    assert opt.told[0][0] == at.list_from_params(at.SHIPPED_VALUES)
    assert posted[0] == at.SHIPPED_VALUES
    # All three trials told.
    assert len(opt.told) == 3
    # State persisted with 3 entries.
    state = at.load_state(state_path)
    assert len(state) == 3
    assert [e["iteration"] for e in state] == [1, 2, 3]
    # Best is the minimum-J entry.
    assert best["J"] == min(e["J"] for e in state)


def test_loop_missing_trial_uses_penalty(tmp_path):
    asks = [[0.5, 0.3, 0.6, 0.05, 0.2]]
    # Trial 1 (seed) produces no file → penalty.
    writers = [None, _write_converging_trial]
    opt, posted, best, state_path = _run_loop(tmp_path, n=2, asks=asks,
                                              trial_writers=writers)
    state = at.load_state(state_path)
    assert state[0]["J"] == at.MISSING_TRIAL_PENALTY
    # The optimiser was told the penalty for the seed point.
    assert opt.told[0][1] == at.MISSING_TRIAL_PENALTY


def test_resume_retells_prior_points_then_continues(tmp_path):
    state_path = str(tmp_path / "state.json")
    # Pre-existing 2-trial history.
    prior = [
        {"iteration": 1, "x": at.list_from_params(at.SHIPPED_VALUES),
         "params": dict(at.SHIPPED_VALUES), "J": 5.0, "trial_file": "a", "ts": 0},
        {"iteration": 2, "x": [0.5, 0.3, 0.6, 0.05, 0.2],
         "params": at.params_from_list([0.5, 0.3, 0.6, 0.05, 0.2]), "J": 2.0,
         "trial_file": "b", "ts": 1},
    ]
    at.save_state(state_path, prior)

    asks = [[0.6, 0.1, 0.7, 0.02, 0.15], [0.45, 0.25, 0.55, 0.06, 0.12]]
    writers = [_write_converging_trial, _write_converging_trial]
    opt, posted, best, _ = _run_loop(tmp_path, n=4, asks=asks,
                                     trial_writers=writers, resume=True,
                                     state_path=state_path)

    # First two tells re-play the prior points, in order.
    assert opt.told[0] == (prior[0]["x"], 5.0)
    assert opt.told[1] == (prior[1]["x"], 2.0)
    # Then two fresh asks → two fresh tells (total 4).
    assert opt.ask_calls == 2
    assert len(opt.told) == 4
    # State now has 4 entries.
    assert len(at.load_state(state_path)) == 4


# ── real-skopt integration (runs only where scikit-optimize is installed) ──────

def test_integration_real_optimizer_3_trials(tmp_path):
    pytest.importorskip("skopt")
    trials_dir = tmp_path / "trials"
    trials_dir.mkdir()
    state_path = str(tmp_path / "state.json")

    counter = {"i": 0}

    def fake_find(_dir, _after):
        i = counter["i"]
        counter["i"] += 1
        path = str(trials_dir / f"t{i}.jsonl")
        _write_converging_trial(path)
        return path

    posted = []
    times = iter(range(1000))
    opt = at.make_optimizer(seed=1)
    best = at.autotune(
        opt, n=3, robot="x:8080", trials_dir=str(trials_dir),
        state_path=state_path, resume=False,
        prompt_fn=lambda *a, **k: None,
        find_trial_fn=fake_find,
        post_fn=lambda r, p: posted.append(dict(p)),
        now_fn=lambda: next(times),
        out=lambda *a, **k: None,
    )
    # Real optimiser proposed valid in-bounds points for trials 2 & 3.
    assert len(posted) == 3
    assert posted[0] == at.SHIPPED_VALUES
    for p in posted[1:]:
        for k, (lo, hi) in at.PARAM_BOUNDS.items():
            assert lo <= p[k] <= hi
    assert len(at.load_state(state_path)) == 3
    assert best is not None


# ── bounds-consistency with the live controller (if importable) ─────────────────

def test_bounds_match_follow_me():
    pytest.importorskip("numpy")
    try:
        from pi_app.control.follow_me import TUNABLE_PARAM_BOUNDS
    except Exception:
        pytest.skip("follow_me not importable in this environment")
    assert at.PARAM_BOUNDS == TUNABLE_PARAM_BOUNDS
    # Shipped seed values lie within their bounds.
    for k, (lo, hi) in at.PARAM_BOUNDS.items():
        assert lo <= at.SHIPPED_VALUES[k] <= hi
