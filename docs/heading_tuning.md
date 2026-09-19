# Heading Tuning (OAK-D IMU)

Field-proven procedure for diagnosing and calibrating OAK-D Lite BMI270 yaw after
chalk tests. Production defaults were **field-validated 2026-07-12**: pin
`gyro_y` at scale `1.0`. **Never restore `auto` / `0.46`** — that pair hid
host-side sample loss behind an empirical multiplier.

## Sign convention (read this first)

The canonical statement lives in the docstring of `OakImuReader.read`
(`pi_app/hardware/oak_imu.py`). Everything else in the stack refers to it:

- `heading_deg` is compass-style, **CLOCKWISE-POSITIVE** viewed from above,
  relative to boot orientation, in `[0, 360)`.
- `yaw_rate_world_dps` (published as `gz_dps` for `ImuReader` compatibility)
  == `d(heading_deg)/dt`. **A right turn is a positive rate.**
- Every yaw channel selectable in `OakImuReader` / `ImuYawProducer` is
  "rotation about the body-**DOWN** axis, CW-positive":
  - `gyro_y` **already is**. The BMI270 Y axis points down on this mounting
    (the accelerometer reads about −1 g on Y at rest, which is why
    `oak_imu.py` computes `roll_acc = atan2(ax, -ay)`), and a right-handed
    frame with +Y down makes `+gy` a clockwise turn.
  - `gravity_projected` projects onto the accelerometer **UP**-vector
    (specific force), so it is **NEGATED** to land in this convention.
  - `gyro_x` / `gyro_z` are diagnostics; their sign depends on mounting.

WARNING: the 2026-07-12 chalk validation was **magnitude-only**, so the sign was
never checked. It was wrong for two months. Field measurement 2026-09-19 (robot
on `3923f23`): hand-turning the robot ~90° **RIGHT** moved the dashboard heading
241.4 → 152.8 (−88.6°); turning back **LEFT** moved it 162.9 → 255.0 (+92°).
Magnitude right, sign inverted. The inversion was `heading = -yaw` inside the
reader; `ImuSteeringConfig.invert_output = True` and the negated waypoint ALIGN
pivot were **compensators** for it, not independent facts. Under that double
flip the steering D-term (`d_term = -kd * yaw_rate`) was **anti-damping**.

Fixed 2026-09-19: the reader integrates `+gyro_y` into heading,
`invert_output` defaults to `False`, ALIGN commands `+pivot_yaw_cmd` for a
positive heading error, and the chalk harness prints a **SIGN** PASS/FAIL
separate from the magnitude band. Guard tests:
`pi_app/tests/test_heading_sign_closed_loop.py` (closed loop around a kinematic
plant), plus sign cases in `test_oak_imu.py` and `test_waypoint_nav.py`.

If steering or ALIGN turns the wrong way, **fix the sign at the reader and
re-chalk**. Never add a second inversion downstream.

## Architecture (lossless producer yaw)

```
BMI270 @ 100 Hz ──► DepthAI host IMU queue (maxSize=512 msgs, nonblocking)
                         │
                         ▼
                    OakDepthReader._poll_imu (shared vision thread)
                         │
                         ├─ drain ALL queue messages (consumed/batched, not dropped)
                         ├─ sort packets by device timestamp
                         └─ ImuYawProducer.ingest(every packet; cap=512 packets)
                                │
                                ├─ cum_yaw_x / y / z / gravity (unscaled rad)
                                ├─ latest raw body rates (diagnostics)
                                └─ counters: integrated / duplicate / gap / backlog
                                         │
                                         ▼
                              OakImuReader.read()
                                │
                                ├─ select axis (gyro_x/y/z / auto / gravity)
                                ├─ Δcum × scale  (scale applied exactly once)
                                ├─ preserve unread Δcum across generation bumps
                                │    (incl. +turn then -turn returning cum≈0)
                                └─ reseed without jump only on counter rewind
```

### Why this exists

1. **Host-side packet loss (primary under-report)** — OAK requests 100 Hz, but
   the shared vision loop historically drained the IMU queue and kept only the
   newest packet (`latest` mode). `OakImuReader` then reconstructed angle from
   sparse snapshots; intervals >0.15 s **gap-froze** and lost rotation. Direct
   light-pipeline harness at fixed `gyro_y` scale=1 was accurate (~90→92,
   ~180→180–183) while full service scale=1 read ~73/81 for physical 90°.
2. **Host queue depth** — DepthAI default `createOutputQueue()` is
   `maxSize=16` nonblocking. Full-vision stalls can overwrite samples before
   the host ever sees them. Production uses **maxSize=512 message slots**,
   nonblocking. At batch threshold ≈1 this is multi-second headroom, but it is
   **not** a hard 5.1 s wall-clock guarantee: one queue message may hold
   multiple packets. **Nonblocking overwrite loss is not directly observable**
   through `tryGet` — never invent occupancy or drop counts from drain size.
3. **Wrong / unstable axis (`auto`)** — can pick X/Z under vibration.
4. **Phantom yaw from duplicate samples** — fixed earlier (6950e14).
5. **Generation-reseed loss** — a single jittered/regressed device timestamp
   bumps producer generation; older consumers discarded all unread cum (~27°
   in simulation). Consumer now applies continuous cum deltas across generation
   bumps. True freeze only on **integrated-packet counter rewind** (producer
   replacement) — never a near-zero cum magnitude heuristic (a legitimate
   +turn then -turn returns channels to ~0).

## Current production baseline (config.py)

| Key | Value | Notes |
| --- | --- | --- |
| `oak_yaw_rate_source` | `"gyro_y"` | Field-validated 2026-07-12; **never restore `auto`** |
| `oak_yaw_rate_scale` | `1.0` | Neutral; **never restore `0.46`** as a loss mask |
| `oak_use_gravity_projected_yaw_rate` | `False` | |
| `oak_nmni_enabled` | `True` | Applied per-packet on the producer |
| `oak_nmni_threshold_dps` | `0.3` | |
| `oak_stationary_bias_tracking_enabled` | `True` | Added 2026-09-19; replaced `oak_bias_adapt_*` |
| `oak_zupt_enabled` | `True` | Freeze yaw integration while provably still |
| `oak_stationary_window_s` | `1.0` | |
| `oak_stationary_gyro_std_dps` | `0.3` | Per-axis raw-gyro std gate |
| `oak_stationary_accel_std_g` | `0.03` | **Tune this one on the robot** |
| `oak_stationary_max_rate_dps` | `2.0` | Must stay above the hot bias (~1.3 dps) |
| `oak_stationary_bias_tau_s` | `15.0` | Bias relaxation time constant |
| IMU host queue | `maxSize=512` msgs, `blocking=False` | message slots, not seconds |
| Producer drain cap | `max_packets_per_drain=512` | aligned with host msg capacity @ ~1 pkt/msg |

Legacy `auto`+`0.46` is retired. If chalk later shows systematic magnitude error
*after* lossless proof, fit a new scale for the **pinned** axis only — do not
bring back auto axis switching without new field evidence.

## Stationary bias tracking and ZUPT

### The field measurement (2026-09-19)

Parked, the heading wound **+0.9 deg/s**. The robot JSON log shows `gy_body_dps`
(the bias-subtracted body rate about Y) after a **good** 3 s boot calibration:

| Time | `gy_body_dps` |
| --- | --- |
| 11:12 (just after calibration) | −0.03 |
| 11:16 | −0.46 |
| 11:20 | −0.83 |
| 11:36 | −1.07 |
| 11:49 | −1.25 |
| later | plateau ≈ −1.2 |

That is **thermal gyro-bias drift**, and before 2026-09-19 nothing tracked it:

- NMNI at 0.3 deg/s cannot gate a 1.2 deg/s rate.
- The reader-side `bias_adapt` only ran when every axis was below the NMNI
  threshold, so it could **never engage** once the drift exceeded 0.3 deg/s —
  exactly when it was needed. It has been removed
  (`oak_bias_adapt_enabled` / `oak_bias_adapt_alpha` are gone), because leaving
  it would have meant two bias integrators fighting.

### Motion witness: an IMU cannot vouch for its own stillness (2026-09-19)

Two independent reviewers found the same BLOCKER in the detector below: a
genuine slow steady rotation (a cross-slope creep, a gentle arc) satisfies the
raw gyro/accel window's std and rate gates exactly as well as a truly parked
robot — measured, a true 1.5 deg/s turn was frozen 39 of 40 s, and a
0.08 deg/s² ramp ratcheted the bias to 70 deg/s. Gyro and accel alone cannot
tell the difference; the fix is an independent signal.

`ImuYawProducer.set_motion_witness(still, host_ts)` records a wheels-stopped
witness pushed once per control-loop tick from `pi_app.app.main`, derived from
VESC RPM / commanded drive bytes via
`pi_app.control.rpm_plausibility.wheels_stopped()` (prefers RPM readback;
falls back to commanded bytes when RPM is unavailable or the plausibility gate
has already flagged it dead). The robot is **stationary** only when the window
below is quiet **and** the witness is fresh and says `still=True`
(`oak_witness_timeout_s`, default 1.0 s). No witness ever received, a stale
one, or one that says the wheels are turning: never stationary — no bias
tracking, no ZUPT, regardless of how quiet the gyro looks. When tracking is
enabled but no fresh witness has been seen for over 5 s, a rate-limited (one
per 60 s) WARNING says so: `stationary bias tracking idle: no motion witness
(wheels-stopped signal) received`.

### How it works now (producer side)

`ImuYawProducer` keeps a rolling window of the last `oak_stationary_window_s`
of **raw** samples, with running sums so mean/std are O(1) per packet (no numpy
on the Pi). The window looks **quiet** only when **all** of these hold:

1. the window is full (spans the configured length, ≥ 3 samples);
2. every raw gyro axis has std < `oak_stationary_gyro_std_dps`;
3. the accel norm has std < `oak_stationary_accel_std_g`;
4. every axis RAW mean has absolute value < `oak_stationary_max_rate_dps`.

The std gates do the real work. The max-rate bound is an **absolute** bound on
the raw mean (not a residual against the current bias estimate — residual-to-
bias was chicken-and-egg: it could never learn a bias larger than the bound
itself). Raised 2.0 → 5.0 deg/s on 2026-09-19: a raw mean above 5 deg/s while
the wheels are stopped and the sensor is quiet is not bias; a hot bias
(measured up to ~2.6 deg/s) must stay learnable. **`stationary` = window quiet
AND fresh witness** — see the subsection above; the window alone is
necessary but no longer sufficient.

While stationary and tracking is enabled, each packet relaxes the bias toward
the window mean: `bias += (dt / oak_stationary_bias_tau_s) * (mean - bias)`.
With `oak_zupt_enabled` the packet is also **not** added to `cum_x/y/z/grav`
(the heading is frozen), while the packet counters still advance so the
consumer's producer-replacement detector keeps working.

NOTE: a first-order tracker lags a ramp by `slope * tau`, so the heading error
that leaks through a bias change is about `delta_bias * tau` — roughly 18° for
the measured 1.2 deg/s change at `tau = 15 s`, independent of how long the drift
takes. ZUPT is what removes it: while the robot is provably still, nothing
integrates at all. If a future field run shows wind-up **while moving**, shorten
`oak_stationary_bias_tau_s` rather than widening the std gates.

`OakImuReader` reports `yaw_rate_world_dps = 0.0` and `integrate_status =
"zupt"` whenever the snapshot says `zupt_active`, so the PID cannot damp a turn
the frozen heading does not see.

### Log lines to look for

All at WARNING (the app installs no logging handler, so INFO is dropped):

```
OAK IMU gyro bias measured over 3.00s from N samples: x=... y=... z=... dps
OAK IMU gyro bias: no fresh IMU samples in 3.00s: keeping prior bias (...)
OAK IMU zupt_engage: bias=(...) dps delta_since_last_log=(...) dps bias_updates=N ...
OAK IMU zupt_disengage: bias=(...) dps delta_since_last_log=(...) dps ...
OAK IMU samples stale (age=...s > ...s): heading frozen, yaw rate reported as 0 ...
```

The ZUPT lines are rate-limited to at most one per 10 s per event type; the
counters (`zupt_engage_count`, `bias_updates`) carry the full history.

Healthy parked signature: `zupt_active=true`, `window_gyro_std_dps` ≈ 0.1,
`window_accel_std_g` ≈ 0.01, `bias_gy_dps` drifting slowly toward −1.2, and
`producer_cum_yaw_y_deg` **not moving**.

## Observability

`OakImuReader.get_health()` and extra `read()` keys expose:

- Body triad: `gx_body_dps`, `gy_body_dps`, `gz_body_dps`
- Selected path: `yaw_rate_source_cfg`, `yaw_rate_source_selected`, `yaw_rate_scale`, `yaw_rate_sign`
- Stationary / ZUPT: `stationary`, `zupt_active`, `zupt_engage_count`,
  `bias_updates`, `bias_gx_dps` / `gy` / `gz`, `gyro_bias_dps` (boot
  calibration copy), `tracked_bias_dps` (the producer's live tracked bias —
  what body-rate diagnostics actually subtract), `window_gyro_std_dps`,
  `window_accel_std_g`, `last_bias_update_host_ts`,
  `stationary_tracking_enabled`, `zupt_enabled`
- Motion witness: `witness_still`, `witness_age_s` (in `get_imu_metrics()`)
- Integration path: `integration_path` (`producer` vs `legacy_snapshot`)
- Sample identity: `sample_age_s`, `device_timestamp_s`, `last_dt_s`
- Consumer counters: `count_duplicate`, `count_stale`, `count_regressed`,
  `count_restart`, `count_gap_freeze`, `count_integrated`,
  `count_producer_packets`, `count_generation_change`, `count_cum_reset`
- Producer / host counters (**truthful loss accounting**):
  - `producer_packets_received` / `drained` / `parsed` / `integrated`
  - `producer_packets_duplicate` / `gap_freeze` / `regressed` / `restart` / `backlog_dropped`
  - `packets_coalesced` — legacy selection drops; stays 0 **by structure** on
    the lossless path (never force-assigned as a proof)
  - `producer_cadence_avg_s` / `producer_cadence_max_s` (healthy ~0.01 s avg)
  - `producer_cum_yaw_x_deg` / `y` / `z` / `grav` (unscaled free yaw)
  - `last_batch_packets`, `queue_drain_count`
  - `queue_msgs_received` / `queue_msgs_consumed` — successfully drained
    messages are **consumed/batched**, never counted as dropped
  - `queue_msgs_dropped` — always 0 (overwrite loss not observable)
  - `queue_msgs_overwrite_observable` — always `false`
  - Host config: `host_queue_max_size`, `host_queue_blocking`, `max_packets_per_drain`
  - **Drain-batch observability** (messages drained per poll — **not** occupancy):
    `drain_batch_high_water_msgs`, `drain_batch_large_events`,
    `drain_batch_full_size_events`
- OAK pipeline: `oak_connected`, `oak_reconnect_count`, `oak_pipeline_running`

Live service: `controller.get_imu_status()["oak_imu"]` (logged under `imu` in arm logs).

### Metric invariants (loss proof)

During a clean chalk turn, **do not** treat a hard-coded zero as proof. Require:

| Invariant | Healthy |
| --- | --- |
| `integrated ≈ received − restart_seeds − duplicate − gap_freeze − …` | within a few packets |
| `packets_backlog_dropped` | 0 (full configured backlog must integrate) |
| `packets_gap_freeze` | rare (cadence_max mostly ≪ 0.15 s) |
| `queue_msgs_dropped` | 0 (drained ≠ dropped; overwrite not observable) |
| `drain_batch_high_water_msgs` | informative only — large drain ≠ queue full |
| scale=1 best axis `\|Δ\|` | within pass band of chalk angle |
| `count_cum_reset` | 0 unless integrated-counter rewind (true replacement) |

`packets_coalesced` remaining 0 only means selection mode is not dropping.
**Never** infer host-queue overflow from drain-batch high-water / large / full-size
events — those measure messages successfully retrieved in one poll, not remaining
queue depth or overwritten samples.

### Residual limitations

- **DepthAI nonblocking overwrite loss is not directly observable** through this
  API. If the device overwrites unread host-queue messages during a long stall,
  those samples never appear in `tryGet` and cannot be counted as drops.
- Host queue is a **message-slot** budget. At batch≈1 / 100 Hz it is roughly
  multi-second, but multi-packet messages make wall-clock headroom shorter than
  `maxSize/rate`. Do not claim a hard 5.1 s guarantee.
- Producer cap is in **packets** (aligned to 512 at ~1 pkt/msg). Pathological
  multi-packet bursts above the cap soft-drop oldest samples
  (`packets_backlog_dropped`) to keep CPU/memory bounded.

## Safe chalk harness (disarmed)

Stop the service so it does not own the camera, then:

```bash
# on the Pi
sudo systemctl stop wall-e    # unit name may vary
cd /home/pi/wall_e-Mini
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --stream
# then
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 180 --stream
# optional NMNI override for this run only (does not write config):
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --stream --no-nmni
# stationary / expected-zero check (scale fit is n/a; no div-by-zero):
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 0 --stream --bias-s 10
```

The harness uses the **full shared OakDepthReader pipeline** (same producer path as
service) and prints producer + drain-batch metrics.

### Exact procedure

1. Flat ground. Mark chalk **0°** on floor and a matching mark on the chassis.
2. Mark chalk **90°** and **180°** CW (looking down) with a square/protractor or known board.
3. Robot **disarmed**, no motor drive from this tool (harness never commands motors).
4. Start harness; wait for bias collection and `READY`.
5. Align chassis to 0°. Enter → **MARK START**.
6. Rotate slowly by hand (or carefully with RC only if you accept extra vibration) to the
   chalk target. Prefer hand-rotate for axis ID.
   - **A clean 90° turn should take roughly 5–10 seconds** (slow enough for
     clean integration, fast enough that residual bias does not dominate).
   - **The camera must rotate rigidly with the chassis** — no flexible mount,
     hand-hold on the camera alone, or relative slip between OAK and body.
7. Enter → **MARK END**.
8. Read the report:
   - triad from **producer cum** at scale=1 (`gyro_x/y/z`)
   - production path (`gyro_y` × 1.0); production heading Δ should ≈ **+**producer gyro_y Δ
   - exact cumulative start/end, bias at 6 decimals, raw per-axis rate stats
   - bias-corrected gyro_y fraction below NMNI threshold
   - generation / restart / regression / gap / backlog metrics
   - producer `recv` / `integrated` / `gap` / `backlog_drop` / cadence
   - drain-batch high-water (msgs per poll) vs maxSize — not occupancy

### Baseline sync (stream and non-stream)

The harness **polls continuously** while waiting for MARK START and MARK END in
both stream and non-stream modes, then captures producer / triad / production
baselines from the **same** poll (triad re-anchored at MARK START). Do not use
a blocking `input()` without polling — that freezes `TriadIntegrator` while
producer cum advances, so a later production refresh desynchronizes start
baselines (field symptom: start prod heading ≠ triad / producer Y, production
Δ mismatch).

### Choosing axis and scale (from evidence only)

1. Among `gyro_x`, `gyro_y`, `gyro_z` at scale=1, pick the axis whose **|Δ|** is closest to
   the chalk angle and whose **sign is consistent** CW vs CCW.
2. Production is already pinned to `gyro_y` @ 1.0 from 2026-07-12 field validation.
3. Only if magnitude is systematically off **after** lossless proof:
   `new_scale = expected_deg / |measured_deg|` for that pinned axis.
4. Re-run 90° and 180° both CW and CCW with the candidate settings via:
   ```bash
   python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 \
     --production-source gyro_y --production-scale 1.0
   ```
5. Only then edit `config.py` and redeploy. **Do not reintroduce auto/0.46.**

### Pass criteria

For a single chalk turn after bias, with the **chosen pinned axis** at its fitted scale
(or scale=1 if magnitude already matches):

| Check | Pass |
| --- | --- |
| Magnitude | `||measured| − expected| ≤ max(8°, 10% of expected)` |
| 90° and 180° | Both pass; 180° error should not be ~2× worse than 90° (rules out wrong axis) |
| CW vs CCW | |Δ| within the same band; opposite signs from each other |
| **SIGN** | **cw → production heading Δ > 0; ccw → < 0.** Own PASS/FAIL row; magnitude alone is not enough |
| No packet loss | integrated tracks received; backlog_drop=0 |
| No jumps | Heading freezes on stale/duplicate; counter-rewind freeze only on true replacement |
| Integration | During the turn, `count_integrated` / `count_producer_packets` increase |

Fail examples from the pre-producer chalk run: 90° CW → ~72° / ~69° (wrong axis and/or
sparse snapshot gap-freeze). Service scale=1 at ~73/81 for physical 90° with load while
light pipeline was accurate — classic host-side loss.

## Exact hardware validation (post-fix)

On the robot, with service path (or chalk harness using full OakDepthReader):

1. `sudo systemctl stop wall-e`
2. `python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --production-source gyro_y --production-scale 1.0 --stream`
3. Hand-rotate 90° CW; confirm:
   - triad `gyro_y` PASS at scale=1
   - production free-yaw ≈ triad for gyro_y
   - backlog_drop=0, cadence_avg≈0.01 s; drain-batch stats are informational only
4. Repeat 180° CW and 90° CCW.
5. Restart service under normal vision load; arm; log `imu.oak_imu` during a slow pivot:
   - `integration_path=producer`
   - `yaw_rate_source_selected=gyro_y`
   - heading tracks physical turn without a scale fudge
6. Only if magnitude still systematically off after lossless proof, fit scale from chalk
   and keep the axis pinned — never “guess” a multiplier to hide packet loss.

## Field results after lossless producer (commit 4ca1ccc baseline)

Chalk harness, full `OakDepthReader` path, production `gyro_y` × `1.0`, NMNI on
(threshold 0.3 dps). Loss accounting was clean on every run below: zero
duplicate/gap/backlog/queue loss, cadence ≈ 0.01007 s, no reconnect, producer
`gyro_y` Δ exact opposite-sign match of production free-yaw Δ.

CAUTION: the production Δ column below is in the **pre-2026-09-19** (negated)
sign. Read it as magnitude evidence only: a 90° CW turn printed −89.73°, which
is exactly the inverted sign this table failed to catch. Under the current
convention the same turn prints **+89.73°**.

| Run | Physical | Production free-yaw Δ | Result |
| --- | --- | --- | --- |
| 90° CW (precise) | 90° CW | **−89.728842°** | PASS |
| 180° CW (slight physical over-rotate) | ~180°+ CW | **+188.870381°** magnitude band | PASS |
| 90° CCW attempt 1 (precise) | 90° CCW | **+109.088079°** | fail (~+19°) |
| 90° CCW attempt 2 (precise) | 90° CCW | **+112.239724°** | fail (~+22°) |

Both CCW runs printed bias collection as **exactly** `x=y=z=0.000000`. During the
marked turns only **0.4%–1.1%** of bias-corrected `gyro_y` samples were below the
0.3 dps NMNI threshold (i.e. almost the entire window was above deadband — a
turn with a persistent rate, not a sparse/gappy integrate).

### Root-cause analysis (calibration vs lossless path)

Lossless proof holds: under-report from host coalescing is **not** this bug.
CW at scale=1 is already accurate; CCW overshoots with matching producer/production
sign-pair → residual **rate offset** in the integrate path, not a second scale or
packet drop.

**Hypothesis checked:** `OakImuReader.__init__` enables producer NMNI (bias=0)
before `calibrate_gyro`. If calibration averaged already bias/NMNI-corrected host
rates, sub-threshold residual bias would collapse to exact zero → circular
calibration → residual integrates for the whole turn and skews CW vs CCW.

**What the code actually exposed (pre-fix):**
- `ImuYawProducer` already published **raw** latest `gx/gy/gz` (bias/NMNI applied
  only as integrate-path locals into cum).
- So `get_imu_data()` did **not** expose corrected rates — the circular path was a
  real footgun if that contract ever flipped, and field `bias=0.000000` still
  matches “calibration saw zeros / no-op bias,” whether from true near-zero
  stationary raw, empty/fresh-sample issues, or a corrected snapshot.

**Fix (no scale fudge):**
1. Document and keep latest `g*_rads` as **raw** forever; add
   `OakDepthReader.get_imu_raw_gyro_dps()` as the explicit cal contract.
2. `calibrate_gyro` temporarily forces producer **bias=0** and **NMNI=off**,
   samples raw rates, then restores NMNI with the measured bias. If a host
   snapshot were integrate-path corrected, pausing those transforms makes
   samples raw-equivalent; if already raw, sample values are unchanged.
3. Empty collection keeps prior bias (does not claim all-zero success).
4. Unit regressions: sub-threshold residual recovered with NMNI on; corrected
   `get_imu_data` footgun still calibrates; CW/CCW `|Δ|` symmetric after cal.

### Hardware retest required?

**Yes — one short chalk pass on the robot after deploy of this fix** (not done in
this worktree; no commit/deploy from the agent session):

```bash
sudo systemctl stop wall-e
cd /home/pi/wall_e-Mini   # or this worktree path on the Pi
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --direction cw --stream
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --direction ccw --stream
# optional:
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 180 --direction cw --stream
```

Pass when both CW and CCW 90° sit in the existing band, bias print is **non-zero
if residual exists** (or honestly near-zero with symmetric CW/CCW), and loss
invariants stay clean. If CCW still overshoots with a truthful non-zero bias and
zero loss, reopen as mount/g-sensitivity — **do not** reintroduce scale `0.46`.

## If regression happens in service

1. Confirm IMU source: startup log `source: oak_d`.
2. Inspect arm log `imu.oak_imu` (or status JSON):
   - `integration_path` is `producer`?
   - `yaw_rate_source_selected` is `gyro_y` (not auto flapping)?
   - `packets_backlog_dropped` / `gap_freeze` rising during motion
   - large `drain_batch_*` alone is **not** overflow proof (overwrite not observable)
   - `count_cum_reset` during motion without USB reconnect → investigate counter rewind
   - `count_duplicate` rising while idle is expected; rising `count_stale` during motion is not
   - `sample_age_s` should stay well under 0.5 s when healthy
3. Re-run chalk harness before changing scale again.

## Unit tests

```bash
python3 -m unittest pi_app.tests.test_oak_imu_yaw_producer pi_app.tests.test_oak_imu -v
python3 -m unittest discover -s pi_app/tests -p "test_*.py"
```

Coverage includes: sparse undercount reproduction, full-batch accuracy under delayed
consumer reads, full 512-packet backlog without drop, duplicates, ordering,
generation-jitter 27° preservation, exact +10°/−10° with generation (no false
cum_reset), true counter-rewind reset freeze, reconnect, bounded soft backlog cap,
scale-once, host-queue / producer-cap alignment asserts, axis isolation,
controller health embedding, **NMNI+bias non-circular calibrate** (sub-threshold
residual recovered with NMNI on), corrected-snapshot footgun calibrate, and
**CW/CCW free-yaw symmetry after bias cal**.

## Related docs

- `pi_app/hardware/oak_imu_yaw_producer.py` — pure integrator
- `docs/imu_tuning_session_20260308.md` — mount / gravity-projection history
- `docs/IMU_TROUBLESHOOTING.md` — external IMU paths
- `docs/gps_heading_alignment.md` — absolute north alignment (separate from relative gyro yaw)
