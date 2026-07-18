# Drivetrain Gearing Analysis — Follow-Me Jerkiness

Question: is gear ratio/ERPM headroom limiting low-speed control during person-following,
or is this a control-loop problem? Evidence pulled from this repo only.

## a) Drivetrain as-built

- Two VESCs over CAN, left ID `2`, right ID `1` (`config.py:135-136`, `docs/can_vesc.md:19-25`).
  `CAN_PACKET_SET_RPM`, arb id `0x300 + can_id`, signed 32-bit BE RPM payload
  (`pi_app/hardware/vesc.py:700-702`).
- 13S Li-ion, confirmed via BMS cell count, cutoff 39.0 V = 3.0 V/cell (`config.py:146-147`).
- Motor: **14 poles → 7 pole pairs** (`config.py:162,166`). No kV/current/wattage rating
  recorded anywhere in the repo.
- Drivetrain: **34.2857:1** overall = 6:1 gearbox × (20/14) chain × (40/10) chain
  (`config.py:163-167`, comment). Only recorded as this one comment — no parts list to
  cross-check the individual sprocket counts.
- Wheel radius **0.18415 m** (14.5" dia, axle-to-contact-patch; `config.py:161,165`) —
  corrected from a placeholder `0.085` in commit `dbef18a` (2026-04-19).
- Command ceiling `max_erpm = 15000` (`config.py:131-132`) — a config value, not verified
  against a documented motor/controller limit.
- Byte protocol: `0..254`, neutral `126`, ±128 span (`pi_app/control/mapping.py:19-21`);
  `_byte_to_rpm()` linearly maps offset-from-neutral to `±max_erpm`
  (`pi_app/hardware/vesc.py:736-747`).
- Track width **0.28 m** matches physical (`config.py:354`); a test note flags *effective*
  skid-steer width may need 0.30–0.40 m for tread scrub (`docs/trail_follow_testing_plan.md:884-888`)
  — a steering-kinematics caveat, not a gearing one.
- Weight: one-sheet says **~40 kg** (`docs/walle_one_sheet.html:200`), matching the task
  brief; an older test doc's "~15 kg" (`docs/trail_follow_testing_plan.md:110`) is stale.

**Cross-check:** at full-scale command (byte 254, offset 128 → `eRPM=15000`), the recorded
chain gives `mech_rpm=15000/7=2142.9` → `wheel_rpm=2142.9/34.2857=62.5` → wheel speed
`=62.5/60×2π×0.18415=1.205 m/s` — matching the one-sheet's **"~1.2 m/s" top speed**
(`docs/walle_one_sheet.html:204`) almost exactly. Same formula reimplemented for live
telemetry in `pi_app/control/controller.py:578-587` with identical constants.

## b) Control problem at walking speed

Follow-Me's cap `max_follow_speed_byte=110` (`config.py:202`), used by both direct and trail
pursuit (`pi_app/control/follow_me.py:818,970-971`) → `110/128×15000≈12,891 eRPM`, **~86% of
`max_erpm`**. Two independent m/s estimates: kinematic `110/128×1.205≈1.04 m/s`; empirical
`trail_speed_scale_mps_per_byte=0.0075` (RTK GPS on gravel, offsets 67–123, 2026-03-28,
`config.py:347`) → `110×0.0075=0.825 m/s`. Measured ≈80% of kinematic — normal track
scrub, and it reinforces the cross-check above (constants aren't obviously wrong).

**Resolution:** open-loop gain `=110/1.5≈73.3 bytes/m` (`pi_app/control/follow_me.py:818-822`);
with a 0.2 m dead zone (`config.py:263`), steady following-error (0.2–0.7 m) commands
**~15–51 bytes** (~1,800–3,800 eRPM, ~0.11–0.38 m/s). Each byte ≈0.0075–0.0094 m/s
(~1–1.3% of top speed) — integer resolution is not the bottleneck here.

What *is* a live, diagnosed problem at this speed range: the velocity PID that closes the
loop on measured wheel speed is disabled — gains zeroed 2026-06-11 because "the wheel-RPM
telemetry feeding this loop is dead/unreliable, so the PID integrates garbage and drives the
lunge/stall cycle" (`config.py:270-274`). Slip compensation was separately disabled
2026-06-13 after a runaway the first time RPM telemetry went live (`config.py:313-320`).
Both are feedback-reliability issues, not torque/ratio issues. No repo evidence on current
draw/torque/stall at low RPM (`left/right_current_a` exist and log,
`pi_app/control/controller.py:571-572`, but no baseline recorded) — see §e.

## c) Is gearing the limiter?

**No — evidence points at the control loop.**

1. eRPM/gearing math reproduces the documented top speed almost exactly (§a) — strong
   evidence the ratio and wheel size are recorded correctly.
2. `max_erpm=15000` leaves headroom above the current cap (86% used), but that ceiling has
   no documented motor-capability backing (missing fact, below).
3. `max_follow_speed_byte` has bounced 60→100→115→80→110 across 5 commits, Feb 28–Apr 17
   2026, unchanged since (`git log -p -G"max_follow_speed_byte: int = " config.py`) — a
   tuning search, not evidence of a hard mechanical ceiling.
4. Every jerkiness mechanism actually diagnosed here (velocity-PID lunge/stall, slip-comp
   runaway) is a feedback-reliability problem, already root-caused and mitigated by
   disabling the loop — not by any gearing change.

Still open per `TODO.md:10-15`: RPM telemetry is "resolved bench-verified" but followups are
"Confirm nonzero RPM readback during a real drive (bench verification only so far)" and
"Decide whether to re-enable the velocity PID." That's the real next step.

## d) Options

**1. Regear** — Pros: helps *if* low-speed stall torque on soft ground is real, but nothing
in the repo shows that today. Cons: unjustified by evidence; cuts the already-modest ~1.2 m/s
ceiling further; a full gearbox+chain rebuild to chase a problem currently explained by
disabled feedback loops. **Not recommended without new evidence (§e).**

**2. Stay + finish velocity PID (recommended)** — Pros: targets the diagnosed lunge/stall
directly; telemetry (`left/right_rpm`, `actual_speed_mps`, `current_a`) is already wired to
logs (`pi_app/control/controller.py:1235-1237`, `pi_app/app/main.py:997-1000`); prior-tuned
gains preserved in comment (`config.py:276`, "0.8/0.2/0.05"); freshness gating already exists
(>0.5s per-motor staleness → open-loop fallback, `pi_app/control/controller.py:611-619`).
Cons: needs the field verification `TODO.md:14` still flags open.

**3. Stay + open-loop/duty-floor tuning** — Pros: cheap, no hardware/telemetry risk;
re-validate `trail_speed_scale_mps_per_byte` (calibrated 2026-03-28 at offsets 67–123,
predating the current 110 cap set 2026-04-17) and `pursuit_min_speed_byte=15.0`
(`config.py:372`). Cons: doesn't touch the root cause — a band-aid on the open-loop fallback.

## e) Measurements for the next drive

| # | Measurement | Already captured? |
|---|---|---|
| 1 | eRPM during an actual slow follow drive (not just bench) | **Yes** — logged when fresh (`controller.py:1235-1236`, `main.py:997-998`); this is exactly what `TODO.md:14` flags unconfirmed. |
| 2 | `actual_speed_mps` vs. commanded speed at walking pace (is the lunge/stall from `config.py:270-274` still present?) | **Yes** — `controller.py:1237,1308`, `main.py:924,999-1000`. |
| 3 | Duty cycle at slow follow pace | **No** — parsed from CAN STATUS(9) (`vesc.py:69,461`) but never copied into the `VescTelemetry` snapshot (`vesc.py:79-105`) or forwarded onward. Needs a small wiring addition first. |
| 4 | Motor current during slow follow + a deliberate stall (grass/obstacle) — the only way to test torque headroom, which nothing here documents | **Yes, fields log** (`controller.py:571-572,1235`) — but no baseline exists; this drive would be the first data point. |
| 5 | Re-check `trail_speed_scale_mps_per_byte` near byte 90–110 (today's range) | **Partially** — m/s is logged (#2), but the last GPS calibration only covered offsets 67–123, before the current 110 cap. |

## Missing facts

- No motor kV, current rating, or wattage — torque headroom can't be assessed statically.
- No confirmation `max_erpm=15000` reflects a real motor limit vs. an arbitrary cap.
- No individually-verified sprocket tooth counts (only the combined-ratio comment).
- No backlash measurement — still an unstarted roadmap item
  (`docs/performance_optimization_todo.md:60-61`).
- No stall-current or grass/soil traction data.

## Bottom line

Gearing math checks out (near-exact match to documented top speed) with real eRPM headroom
above the current follow cap — nothing suggests under-gearing. Every diagnosed jerkiness
mechanism (velocity-PID lunge/stall, slip-comp runaway) is a disabled/broken feedback loop,
not a torque or ratio shortfall — highest-value next step is finishing the velocity-PID
re-enable (option 2) after capturing §e, not regearing.
