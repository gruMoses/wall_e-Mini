#!/usr/bin/env python3
"""Gaussian-process Bayesian-optimisation auto-tuner for Follow-Me steering.

Closed-loop, human-in-the-loop ask-tell tuner. Each iteration:
  1. ask() the GP optimiser for a candidate point in the 5-D param space;
  2. POST those params to the robot's runtime override endpoint
     (``/api/follow_me/params``) — this sets gains/filters ONLY, never motion;
  3. prompt you to engage Follow-Me, walk the standard pattern, and disengage;
  4. find the trial-recorder JSONL the engagement just produced;
  5. score it with the same cost function the tuner minimises (``fm_score``);
  6. tell() the optimiser the score and persist it for crash-resume;
  7. report best-so-far.

Trial 1 is *seeded* with the currently-shipped config values so the GP always
has the known-decent baseline in its model.

SAFETY: this tool only ever (a) POSTs the five whitelisted, bounds-checked
steering params and (b) reads files. It never touches arm/drive/teleop/
calibration endpoints — it cannot command motion.

WHERE TO RUN: on the Pi itself (default ``--trials-dir /tmp/fm_trials`` is the
Pi-local path the recorder writes to). ``--robot`` defaults to the Pi's LAN
address so it also works from another host on the LAN, but the trial files are
only readable where the recorder wrote them.

scikit-optimize is required and imported lazily — install it with
``pip install scikit-optimize`` (a CLI/dev dependency, not part of the service).

Usage:
    python tools/fm_autotune.py [--n 20] [--robot 192.168.86.54:8080] \\
        [--trials-dir /tmp/fm_trials] [--state fm_autotune_state.json] [--resume]
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

# fm_score lives alongside this file in tools/.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fm_score import parse_jsonl, score_trial  # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
# Param space — MUST match pi_app.control.follow_me.TUNABLE_PARAM_BOUNDS and the
# bounds enforced by the /api/follow_me/params endpoint. Kept as a local literal
# (rather than importing follow_me) so this CLI stays import-light and runnable
# without the robot's heavy hardware deps. test_fm_autotune asserts the two
# agree whenever follow_me is importable.
# ─────────────────────────────────────────────────────────────────────────────
PARAM_BOUNDS: dict[str, tuple[float, float]] = {
    "pid_lateral_kp": (0.1, 0.8),
    "pid_lateral_kd": (0.0, 0.5),
    "target_ema_alpha": (0.2, 0.9),
    "steer_deadband_norm": (0.0, 0.1),
    "steer_slew_per_tick": (0.03, 0.4),
}
PARAM_ORDER: list[str] = list(PARAM_BOUNDS)

# Currently-shipped FollowMeConfig values — seed for trial 1.
SHIPPED_VALUES: dict[str, float] = {
    "pid_lateral_kp": 0.4,
    "pid_lateral_kd": 0.2,
    "target_ema_alpha": 0.5,
    "steer_deadband_norm": 0.04,
    "steer_slew_per_tick": 0.1,
}

# A trial that produced no usable file (recorder never ran / empty) gets this
# fixed penalty — worse than a lost-track trial (50) so the GP steers away from
# whatever made the engagement unrecordable, but not so huge it warps the model.
MISSING_TRIAL_PENALTY = 100.0

DEFAULT_ROBOT = "192.168.86.54:8080"
DEFAULT_TRIALS_DIR = "/tmp/fm_trials"
DEFAULT_STATE = "fm_autotune_state.json"
DEFAULT_N = 20


# ── param <-> vector helpers ─────────────────────────────────────────────────

def params_from_list(x: list[float]) -> dict:
    """Map an optimiser vector (PARAM_ORDER) to a named-params dict."""
    return {k: float(v) for k, v in zip(PARAM_ORDER, x)}


def list_from_params(params: dict) -> list[float]:
    """Map a named-params dict to an optimiser vector (PARAM_ORDER)."""
    return [float(params[k]) for k in PARAM_ORDER]


# ── state persistence ────────────────────────────────────────────────────────

def load_state(path: str) -> list[dict]:
    """Load the recorded (params, J) history; [] if absent/unreadable."""
    if not os.path.exists(path):
        return []
    try:
        with open(path) as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return []
    return data if isinstance(data, list) else []


def save_state(path: str, state: list[dict]) -> None:
    """Atomically rewrite the full history (tmp + os.replace)."""
    tmp = f"{path}.tmp"
    with open(tmp, "w") as fh:
        json.dump(state, fh, indent=2)
    os.replace(tmp, path)


# ── trial discovery + scoring ────────────────────────────────────────────────

def find_newest_trial(trials_dir: str, after_ts: float) -> str | None:
    """Return the newest ``*.jsonl`` in ``trials_dir`` modified after ``after_ts``.

    The recorder opens/writes the engagement's file only once the operator
    engages Follow-Me, which happens strictly after we stamp ``after_ts``, so a
    fresh trial always sorts newer. Returns None if nothing newer is found.
    """
    newest: tuple[float, str] | None = None
    for p in glob.glob(os.path.join(trials_dir, "*.jsonl")):
        try:
            mtime = os.path.getmtime(p)
        except OSError:
            continue
        if mtime > after_ts and (newest is None or mtime > newest[0]):
            newest = (mtime, p)
    return newest[1] if newest is not None else None


def score_or_penalty(trial_path: str | None) -> tuple[float, dict | None, str | None]:
    """Score a trial file, or return the fixed penalty for missing/empty.

    Lost/failed trials are NOT special-cased here: their JSONL exists, so they
    score normally and pick up the +50 lost-track term inside ``score_trial``.
    Only a genuinely absent or empty file gets ``MISSING_TRIAL_PENALTY``.
    """
    if trial_path is None:
        return MISSING_TRIAL_PENALTY, None, "no trial file found"
    try:
        records, malformed = parse_jsonl(trial_path)
    except OSError as exc:
        return MISSING_TRIAL_PENALTY, None, f"unreadable trial file: {exc}"
    if not records:
        return MISSING_TRIAL_PENALTY, None, "empty trial file"
    result = score_trial(records, n_malformed=malformed)
    return result["J"], result, None


# ── robot endpoint ───────────────────────────────────────────────────────────

def post_params(robot: str, params: dict) -> dict:
    """POST the five params to the robot override endpoint (stdlib urllib)."""
    import urllib.error
    import urllib.request

    url = f"http://{robot}/api/follow_me/params"
    data = json.dumps(params).encode("utf-8")
    req = urllib.request.Request(
        url, data=data, method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise RuntimeError(f"endpoint rejected params ({exc.code}): {body}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"could not reach {url}: {exc.reason}") from exc


# ── default interactive prompt ───────────────────────────────────────────────

def _interactive_prompt(trial_num: int, params: dict, seeded: bool) -> None:
    tag = "  (shipped seed)" if seeded else ""
    pretty = ", ".join(f"{k}={v:.4g}" for k, v in params.items())
    print(f"\nTRIAL {trial_num} READY{tag}")
    print(f"  params applied: {pretty}")
    print("  engage Follow-Me, walk the pattern, disengage, then press Enter")
    input()


# ── core loop (fully injectable for testing) ─────────────────────────────────

def autotune(
    optimizer,
    *,
    n: int,
    robot: str,
    trials_dir: str,
    state_path: str,
    resume: bool = False,
    prompt_fn=_interactive_prompt,
    find_trial_fn=find_newest_trial,
    post_fn=post_params,
    now_fn=time.time,
    out=print,
) -> dict | None:
    """Run the ask-tell loop. Returns the best recorded entry (or None).

    All side-effecting collaborators (optimizer, prompt, trial discovery, POST,
    clock) are injectable so the plumbing can be unit-tested without skopt, a
    robot, or real trial files.
    """
    state = load_state(state_path) if resume else []
    if resume and state:
        for e in state:
            optimizer.tell(e["x"], e["J"])
        out(f"resumed: re-told {len(state)} prior point(s) to the optimiser")

    completed = len(state)
    for i in range(completed, n):
        trial_num = i + 1
        seeded = (trial_num == 1 and not state)
        if seeded:
            # Seed the model with the known-decent shipped baseline.
            x = list_from_params(SHIPPED_VALUES)
        else:
            x = list(optimizer.ask())
        params = params_from_list(x)

        try:
            post_fn(robot, params)
        except Exception as exc:  # noqa: BLE001 — surface, let operator decide
            out(f"WARNING: failed to apply params for trial {trial_num}: {exc}")

        start_ts = now_fn()
        prompt_fn(trial_num, params, seeded)

        trial_path = find_trial_fn(trials_dir, start_ts)
        J, _result, warn = score_or_penalty(trial_path)
        if warn:
            out(f"WARNING: trial {trial_num}: {warn} -> penalty J={J}")

        optimizer.tell(x, J)
        state.append({
            "iteration": trial_num,
            "x": list(x),
            "params": params,
            "J": J,
            "trial_file": trial_path,
            "ts": start_ts,
        })
        save_state(state_path, state)

        best = min(state, key=lambda e: e["J"])
        best_pretty = ", ".join(f"{k}={v:.4g}" for k, v in best["params"].items())
        out(f"[trial {trial_num}/{n}] J={J:.4f}   "
            f"best so far: J={best['J']:.4f}  ({best_pretty})")

    return min(state, key=lambda e: e["J"]) if state else None


# ── skopt optimiser construction (lazy import) ───────────────────────────────

def make_optimizer(seed: int | None = 42):
    """Build a skopt GP Optimizer over PARAM_BOUNDS, importing skopt lazily."""
    try:
        from skopt import Optimizer
        from skopt.space import Real
    except ImportError:
        sys.exit(
            "scikit-optimize is required for fm_autotune but is not installed.\n"
            "  It is a CLI/tooling dependency, NOT part of the wall-e service.\n"
            "  Install it on the Pi with:\n\n"
            "      pip install scikit-optimize\n"
        )
    dims = [Real(lo, hi, name=k) for k, (lo, hi) in PARAM_BOUNDS.items()]
    return Optimizer(
        dimensions=dims,
        base_estimator="GP",
        acq_func="EI",
        random_state=seed,
    )


def _print_recommendation(best: dict | None, out=print) -> None:
    if best is None:
        out("\nNo trials completed — nothing to recommend.")
        return
    out("\n" + "=" * 60)
    out(f"RECOMMENDED PARAMS  (best J={best['J']:.4f}, trial {best['iteration']})")
    out("=" * 60)
    for k, v in best["params"].items():
        out(f"  {k} = {v:.4f}")
    out("\nTo make permanent, set these in config.py (class FollowMeConfig):\n")
    for k, v in best["params"].items():
        out(f"    {k}: float = {v:.4f}")
    out("")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="GP-BO auto-tuner for Follow-Me steering (human-in-the-loop)."
    )
    parser.add_argument("--n", type=int, default=DEFAULT_N,
                        help=f"number of trials (default {DEFAULT_N})")
    parser.add_argument("--robot", default=DEFAULT_ROBOT,
                        help=f"robot host:port (default {DEFAULT_ROBOT})")
    parser.add_argument("--trials-dir", default=DEFAULT_TRIALS_DIR,
                        help=f"trial JSONL dir, Pi-local (default {DEFAULT_TRIALS_DIR})")
    parser.add_argument("--state", default=DEFAULT_STATE,
                        help=f"state file for resume (default {DEFAULT_STATE})")
    parser.add_argument("--resume", action="store_true",
                        help="resume from --state, re-telling all recorded points")
    parser.add_argument("--seed", type=int, default=42,
                        help="GP random_state for reproducibility (default 42)")
    args = parser.parse_args(argv)

    if not args.resume and os.path.exists(args.state):
        sys.exit(
            f"state file {args.state!r} already exists.\n"
            "  Use --resume to continue it, or --state <newpath> / remove it to start fresh."
        )

    optimizer = make_optimizer(seed=args.seed)
    best = autotune(
        optimizer,
        n=args.n,
        robot=args.robot,
        trials_dir=args.trials_dir,
        state_path=args.state,
        resume=args.resume,
    )
    _print_recommendation(best)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
