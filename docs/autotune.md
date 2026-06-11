# Follow-Me steering auto-tune

A human-in-the-loop Gaussian-process Bayesian-optimisation (GP-BO) rig for
tuning the five Follow-Me lateral-steering parameters against real walked
trials, instead of hand-guessing gains.

The loop: the tuner proposes a parameter set → pushes it to the live robot →
you engage Follow-Me and walk a fixed pattern → it scores the recorded trial →
it proposes the next, smarter set. After ~20 trials it recommends the best set
and prints the exact `config.py` lines to make it permanent.

It tunes **only** these five (all bounds enforced both client- and server-side):

| param                 | bounds       | shipped |
|-----------------------|--------------|---------|
| `pid_lateral_kp`      | [0.1, 0.8]   | 0.4     |
| `pid_lateral_kd`      | [0.0, 0.5]   | 0.2     |
| `target_ema_alpha`    | [0.2, 0.9]   | 0.5     |
| `steer_deadband_norm` | [0.0, 0.1]   | 0.04    |
| `steer_slew_per_tick` | [0.03, 0.4]  | 0.1     |

**Safety:** the rig only POSTs the five whitelisted gains/filters and reads
trial files. It can never command arm/drive/teleop/calibration — it cannot move
the robot. Changes are *volatile* (lost on service restart) by design; nothing
is persisted to the robot until you edit `config.py` yourself.

## Prerequisites

- The wall-e service running on the Pi (serves `:8080`), armed, with a person
  present so Follow-Me can engage.
- `scikit-optimize` installed **on the Pi** (CLI-only dependency, *not* a
  service dependency):

  ```bash
  pip install scikit-optimize
  ```

- Run the tuner **on the Pi**: the per-tick trial recorder writes to the
  Pi-local path `/tmp/fm_trials/<engagement_ts>.jsonl`, and the tuner reads
  those files directly.

## The 30-second walk pattern

Walk the **same** pattern every trial — consistency is what lets the optimiser
compare parameter sets fairly. Roughly 30 s end to end:

1. **Stand 2.5 m directly ahead** of the robot and let it **lock on (~2 s)**.
2. **4 paced steps left** (~1.5 m of lateral travel), then **hold 2 s**.
3. **8 steps right**, back through centre and out the other side (~3 m total),
   then **hold 2 s**.
4. **4 steps left**, back to centre, then **hold 3 s**.
5. Disengage Follow-Me.

This exercises a left step, a long sweep through centre (catches overshoot and
hunting), and a settle — the things the cost function measures.

## Running

```bash
# on the Pi, from the repo root
python tools/fm_autotune.py
```

Common flags:

```
--n 20                       number of trials (default 20)
--robot 192.168.86.54:8080   robot host:port (default)
--trials-dir /tmp/fm_trials  where the recorder writes (default, Pi-local)
--state fm_autotune_state.json   resume/history file (default)
--resume                     continue an interrupted session
--seed 42                    GP random seed (reproducibility)
```

Each trial it prints `TRIAL N READY`, the params it just applied, and waits for
you to engage → walk → disengage → press **Enter**. It then finds the newest
trial file, scores it, and prints the running best.

- **Trial 1 is seeded** with the currently-shipped values, so the GP always has
  the known-decent baseline in its model.
- A **lost/failed** trial scores normally and picks up the +50 lost-track
  penalty. A **missing or empty** trial file gets a fixed `J=100` penalty and a
  warning (so a fumbled engagement doesn't silently look great).

## The cost function

`tools/fm_score.py` reduces a trial to a single scalar `J` (lower = better):

```
J = 1.0  * ∫ x_err² dt                              # tracking error
  + 0.1  * ∫ (d_steer_norm / dt)² dt                # steering thrash / effort
  + 0.5  * max(0, zero_crossings − direction_changes)  # hunting / wobble
  + 50.0 * lost_track                               # lost the person (hard fail)
```

Tracking-error and thrash terms are scored over **direct-pursuit ticks only**
(pure-pursuit trail-following is not scored). You can score any trial by hand:

```bash
python tools/fm_score.py /tmp/fm_trials/<ts>.jsonl          # human report
python tools/fm_score.py /tmp/fm_trials/<ts>.jsonl --json   # machine-readable
```

## Resuming a crashed session

State is appended to `--state` (default `fm_autotune_state.json`) after every
trial. To continue:

```bash
python tools/fm_autotune.py --resume
```

It re-tells every recorded `(params, J)` to a fresh optimiser, then continues
from where it stopped until `--n` total trials are done. (Starting a *fresh*
run refuses to clobber an existing state file — use `--resume`, or point
`--state` at a new path.)

## Making the result permanent

On completion the tuner prints the recommended params and the exact lines to
paste into `class FollowMeConfig` in `config.py`, e.g.:

```python
    pid_lateral_kp: float = 0.37
    pid_lateral_kd: float = 0.21
    target_ema_alpha: float = 0.58
    steer_deadband_norm: float = 0.05
    steer_slew_per_tick: float = 0.12
```

Edit `config.py`, commit, and push to `main` — the Pi auto-deploys. (The
runtime override endpoint is volatile and only used during tuning; the
committed `config.py` values are what survive a restart.)
