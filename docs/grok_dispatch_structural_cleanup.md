# Triage of wave5/wave3-4 structural review + Grok dispatch

**Reviewer inputs:** `docs/code-review-wave5-debug-tab.md`, `docs/code-review-wave3-wave4.md`
**Verified against HEAD:** `1727931` (`main` == `origin/main` at review time)
**Triage lens:** safety code (RC ch3/ch5 authority, charger_inhibit, pack_low latch, UPS
detect/debounce/grace/shutdown, detect-only suppression) was hardware-validated repeatedly
on **2026-07-10**. Prefer refactors that are provably behavior-identical (text relocation,
grep-verified renames) over control-flow rewrites of validated paths. Any control-flow refactor
that ships must be gated on a characterization test that pins byte-identical behavior first.

Every finding below was read against the actual code — line numbers re-derived, not trusted
from the review.

---

## Triage table

| # | Finding (source) | Verdict | One-line rationale |
|---|------------------|---------|--------------------|
| W5-1 | BLOCKER: `oak_viewer.py` grew to 1839 lines with a 2nd mega-HTML blob (`_DEBUG_HTML`) | **AGREE** | Pure text relocation of `_DEBUG_HTML`/`_DASHBOARD_HTML` to template files; `send_from_directory` already imported; served bytes stay identical — highest value, lowest risk. |
| W5-2 | STRONG: telemetry field-by-field spaghetti through flat `RecordingTelemetry` | **AGREE-DIFFERENT** | Adopt option A (dedicated read-path debug snapshot) as a *going-forward* pattern only; option B (nest into `RecordingTelemetry`) rewrites the MCAP jsonschema topic — reject. Not a mechanical diff; low priority. |
| W5-3 | STRONG: duplicated `UPS_STATUS_FILE` default + 9-key payload across daemon and web | **AGREE (gated)** | Real dedup, but collides with the in-flight `wave5/i2c-politeness` branch and the daemon has an import-path wrinkle (standalone, no `pi_app` import). Do last, after that branch lands; contract-test fallback if a shared import can't be made clean. |
| W5-4 | MEDIUM: `_publish_status` re-reads full UPS snapshot each second | **REJECT** | Owned by the `wave5/i2c-politeness` branch (that branch *is* the `_publish_status` cadence/gating fix). Direct collision — Grok must not touch it. |
| W5-5 | MEDIUM: controller grows as telemetry mirror (3 new age/thread fields) | **REJECT** | The 3 fields already landed, are read for real staleness fallback, and live in the MCAP schema. Removing them is churn for zero behavior gain. Folded into W5-2 forward-looking guidance. |
| W5-6 | LOW: `getattr` soup in SSE serializer | **AGREE-optional** | Cosmetic; typed access is nicer but the `getattr(...,default)` is currently fail-soft. Very low value; only if trivially cheap, and keep `_finite_or_none` rounding. |
| W34-1 | BLOCKER: triple motor-cutout clone in `controller.py` (disarmed/charger/pack_low) | **AGREE (char-test gated)** | Confirmed 3 byte-identical bodies at 1003–1037; collapsing to one `drive_blocked = (a or b or c)` is behavior-identical *because the bodies are identical*. Require a characterization test first. Reject the "DriveInhibit flags enum" stronger form as over-engineering. |
| W34-2 | BLOCKER: stale `*voltage_shutdown*` naming after OS-shutdown removal | **AGREE** | Mechanical, grep-verifiable rename across `config.py`, `vesc.py`, `main.py` banner, `controller.py` comment, `test_vesc_telemetry.py`, `TODO.md`. Config field renames = public surface, so grep-proof must return zero. |
| W34-3 | STRONG: `detect_only` as N× `if detect_only` vs `ShutdownActions` policy object | **REJECT** | Headline rejection. Per-site guards are hardware-validated and the code's own comment marks them a *deliberate* safety design. The Protocol rewrite relocates the "forgot a guard" failure mode (arguably worse) and collides with i2c-politeness. Churn on validated safety > benefit. |
| W34-4 | STRONG: log edge-state open-coded in `main.py` (`LogGate`) | **AGREE (char-test gated)** | Contained extraction next to the already-tested `should_log_tick`; also drop the redundant `getattr(config,"log_disarmed_heartbeat_s",5.0)` (field exists = 5.0). Require a unit test proving identical gate decisions over a scripted tick sequence. |
| W34-5 | MEDIUM: soft `getattr` telemetry boundary for pack-low | **AGREE-optional** | Low value; only worth doing while already in `controller.py` for W34-1. Keep dual enforcement (driver hard-stop + controller slew-pin). |

**Dispatched to Grok:** W5-1, W34-2, W34-1, W34-4, W5-3 (gated), plus optional W34-5/W5-6.
**Explicitly out of scope:** W5-4 (i2c-politeness owns it), W5-5 (don't remove landed fields),
W5-2 option B (don't restructure `RecordingTelemetry`), W34-3 (ShutdownActions rewrite rejected).

---

## Rejections, in plain terms

- **W34-3 (ShutdownActions policy object) — REJECTED.** `run_shutdown_sequence` and the two
  startup config-write sites already suppress every hardware/OS effect *inline, per site*, and
  the code comment (daemon lines 541–547) explicitly calls this a deliberate safety choice: not
  a single early return, so no refactor can silently re-arm one action in test mode. Replacing
  that with a `RealShutdownActions` / `DetectOnlyShutdownActions` pair does not eliminate the
  reviewer's stated failure mode ("next action added without a guard re-arms") — it *relocates*
  it to "next method added to `RealShutdownActions` and not overridden in the detect-only class,"
  which silently inherits the real action unless the base is abstract. That is arguably a worse
  trap on a path validated on real hardware today. It also lands on the exact file the
  `wave5/i2c-politeness` branch is editing. Net: control-flow rewrite of validated safety for a
  linear-path aesthetic. Not worth it. Leave the guards.

- **W5-4 (`_publish_status` redundant snapshot) — REJECTED (ownership).** Not "not worth it,"
  just not Grok's to touch: the `wave5/i2c-politeness` branch is the cadence/gating fix for this
  exact function. Dispatching it here guarantees a merge collision.

- **W5-5 (controller telemetry-mirror fields) — REJECTED.** The three fields
  (`_vesc_left_status_age_s`, `_vesc_right_status_age_s`, `_vesc_rx_thread_alive`) already shipped,
  are read for the open-loop staleness fallback anyway, and are serialized into the MCAP telemetry
  schema. Ripping them back out is pure churn with a schema blast radius and no behavior benefit.

- **W5-2 option B (nest VESC diag into `RecordingTelemetry`) — REJECTED.** `RecordingTelemetry`
  *is* the MCAP `/oak/telemetry` jsonschema (registered in `oak_recorder._make_mcap_writer`).
  Restructuring it into nested objects changes the recording schema and can break replay/analysis
  tooling. Only the read-only debug-snapshot idea (option A) is endorsed, and only as a pattern
  for future pills — not a rewrite of what's there.

## Where I propose a better path than the reviewer

- **W34-1:** endorse only the boolean-union `drive_blocked` (collapse three identical bodies),
  **not** the reviewer's optional `DriveInhibit` flags enum. The enum buys nothing here and adds a
  type; the three telemetry keys already stay independent with the simple union.
- **W5-3:** the reviewer wants a shared importable module. The daemon (`scripts/upsPlus_power_daemon.py`)
  is standalone — it imports neither `pi_app` nor root `config`, and its systemd launch context puts
  `scripts/` on `sys.path[0]`, not the repo root. A naive `import pi_app.ups_status_bridge` from the
  daemon can fail at runtime even though the test harness (which injects `scripts/` on the path) is
  green. So: Grok must prove the daemon can import the shared module *in its real launch context*
  (not just under pytest). If it can't be made clean, the fallback is a single **contract test** that
  asserts the writer payload keys == the reader keys and both defaults match — catching drift without
  forcing a runtime import that breaks the daemon.
- **W5-2:** downgraded from "STRONG, fix now" to a forward-looking guideline. There is no clean
  mechanical diff that both stops the bleed and leaves the MCAP schema untouched, so the actionable
  part is "next debug pill uses a read-path snapshot, not a new flat field," not a rewrite this pass.

---

## THE GROK PROMPT

```
You are GROK, executing an agreed, pre-triaged structural cleanup on WALL-E — a Raspberry Pi 5
robot with a SAFETY-RELEVANT control stack. A senior human reviewer has already decided what to do
and what NOT to do. Do exactly the tasks below, in order. Do not add scope.

REPO / BRANCH / MERGE POLICY
- Repo: gruMoses/wall_e-Mini
- Cut a NEW branch named `grok/structural-cleanup` from the LATEST `origin/main`
  (`git fetch origin && git checkout -b grok/structural-cleanup origin/main`).
- DO NOT commit to `main`. DO NOT push to `main`. NO deploys, no ssh, no service restarts.
  A human merges `grok/structural-cleanup` after review + hardware QA. Push only your own branch.
- One commit per task (per finding). Keep each diff independently reviewable.

REBASE CAVEAT — READ FIRST
- A separate branch `wave5/i2c-politeness` is landing on `main` and rewrites
  `scripts/upsPlus_power_daemon.py` (`_publish_status` cadence/gating) and possibly
  `pi_app/web/oak_viewer.py`'s `/api/ups` rendering. You MUST branch from `origin/main` AFTER that
  branch has merged. If it has not merged yet, STOP and report — do not start Task 5, and expect
  the daemon / `/api/ups` code to have moved. Re-derive line numbers from the code you see, never
  from this prompt.

HARD CONSTRAINTS (non-negotiable — this is safety code, hardware-validated 2026-07-10)
- ZERO behavior change. The following must remain byte-identical in behavior:
  RC ch3/ch5 authority, charger_inhibit cutout, pack_low latch, UPS AC detection / debounce /
  grace / shutdown sequence, and detect-only suppression of every hardware/OS side effect.
- For ANY task that touches control flow (Tasks 3 and 4), you MUST first add a characterization
  test that pins the CURRENT behavior byte-for-byte, prove it passes on unmodified code, THEN
  refactor and prove the same test still passes. No characterization test => do not refactor.
- Do NOT rename `VescCanDriver.shutdown()` (lifecycle close) or any UPS "shutdown-countdown"
  register concept — those are real shutdowns.
- Do NOT restructure the `RecordingTelemetry` dataclass and do NOT remove any of its fields
  (it is the MCAP `/oak/telemetry` jsonschema).
- Do NOT convert the daemon's per-site `if detect_only:` guards into a policy/strategy object.
  They are a deliberate, validated safety design. Leave them exactly as they are.
- Do NOT touch `scripts/upsPlus_power_daemon.py._publish_status` cadence/gating (i2c-politeness owns it).
- No new third-party dependencies. Stdlib only for any new shared/helper module.

BASELINE / TEST COMMAND (run from repo root)
- .venv/bin/python -m pytest pi_app/tests/ -q
- Baseline at the review HEAD: 613 passed / 4 skipped / 5 subtests. The full suite must stay
  green at >= 613 passed / 4 skipped after every commit (new tests you add push the count up;
  it must never drop). Paste the pytest summary line into each commit's verification.

──────────────────────────────────────────────────────────────────────────────
TASK 1 — Extract dashboard + debug HTML out of pi_app/web/oak_viewer.py  (finding W5-1, BLOCKER)
- Scope: move the two giant Python string literals `_DASHBOARD_HTML` (~line 101) and `_DEBUG_HTML`
  (~line 940) out of oak_viewer.py into standalone template files, e.g.
  `pi_app/web/templates/dashboard.html` and `pi_app/web/templates/debug.html`.
- Approach: read each file at request time (or module-load time) and serve the SAME bytes with the
  SAME `content_type="text/html"`. `send_from_directory` is already imported; you may use it or a
  plain file read + `Response(...)`. Python keeps owning routes and data — only the markup moves.
- Files: pi_app/web/oak_viewer.py, new pi_app/web/templates/*.html (and package data wiring if the
  service runs from an installed path — verify the file is found in the real run layout, not only tests).
- Acceptance:
  * The bytes served by `GET /` and `GET /debug` are IDENTICAL to before (diff the response body
    against the pre-refactor string — include the check in a test).
  * oak_viewer.py line count drops materially (target: back toward or below its pre-wave5 ~1384).
  * Existing tests that assert read-only page markers (no POST/teleop/follow_me in `/debug` source)
    still pass unchanged.
  * Full suite green (paste summary).

TASK 2 — Rename stale *voltage_shutdown* identifiers  (finding W34-2, BLOCKER)
- Scope: the VESC pack-low watchdog no longer shuts the OS down; the `voltage_shutdown` names lie.
  Rename (behavior-preserving) across the whole tree:
    _check_voltage_shutdown            -> _check_pack_low_watchdog
    _trigger_low_voltage_shutdown      -> _engage_pack_low_latch
    voltage_shutdown_threshold_v       -> voltage_cutoff_threshold_v
    voltage_shutdown_floor_v           -> voltage_cutoff_floor_v
    voltage_shutdown_delay_s           -> voltage_cutoff_sustain_s
- Files (grep first, fix all): config.py (3 fields + comments), pi_app/hardware/vesc.py (methods +
  field reads + comments), pi_app/app/main.py (startup banner ~lines 361–362), pi_app/control/controller.py
  (comment ~line 1024 referencing `_check_voltage_shutdown`), pi_app/tests/test_vesc_telemetry.py
  (all refs), docs/TODO.md (line ~47).
- Approach: pure identifier rename, no logic change. These are `Config` dataclass fields (public
  surface) — the rename must be exhaustive or construction breaks.
- Acceptance:
  * `rg 'voltage_shutdown|_check_voltage_shutdown|_trigger_low_voltage_shutdown' --glob '*.py' --glob '*.md'`
    returns ZERO hits. Paste the (empty) grep output.
  * `VescCanDriver.shutdown()` and UPS shutdown-countdown names are untouched (grep-prove they still exist).
  * Full suite green (paste summary).

TASK 3 — Collapse the triple motor-cutout clone in controller.py  (finding W34-1, BLOCKER)
- Scope: pi_app/control/controller.py ~1003–1037 has three byte-identical branch bodies for
  `not self._safety_state.is_armed` / `self._charger_inhibit` / `self._vesc_pack_low_latched`,
  each doing: force neutral -> `_motor.stop()` -> `_reset_slew_state(mono_now)` -> mirror slew
  telemetry (slew_out_*, slew_delta_*). Collapse to a single guarded path:
      drive_blocked = (not self._safety_state.is_armed) or self._charger_inhibit or self._vesc_pack_low_latched
      if drive_blocked: <the one shared body>  else: <existing slew/set_tracks path>
- Do NOT introduce a `DriveInhibit` flags enum. Just the boolean union. Keep the independent
  telemetry keys `charger_inhibit` and `vesc_pack_low_latched` exactly as they are set today.
- CHARACTERIZATION TEST REQUIRED FIRST (commit it, prove green on UNMODIFIED code, then refactor):
  drive the controller `process()` for each blocked condition individually AND in combination
  (disarmed+charger, charger+pack_low, disarmed+pack_low, all three), plus one non-blocked control
  case, and assert byte-identical left/right outputs, that `_motor.stop()` was called, slew was
  reset, and every slew_* telemetry key matches. The refactor must not change a single asserted value.
- Keep `TestControllerPackLowLatch` (lurch-on-release) green.
- Acceptance: three identical neutral bodies gone (one body); characterization test green before AND
  after; full suite green (paste summary).

TASK 4 — Extract a LogGate for the disarmed-log edge state  (finding W34-4, STRONG)
- Scope: pi_app/app/main.py carries `_prev_tick_mode` / `_prev_tick_charger_inhibit` + the
  mode-changed / charger-inhibit-changed edge bools inline (~lines 427–431, ~828–850) and a
  redundant `getattr(config, "log_disarmed_heartbeat_s", 5.0)` (~line 429). The pure decision fn
  `should_log_tick` already lives in pi_app/app/log_gating.py.
- Approach: add a small stateful `LogGate` class next to `should_log_tick` in log_gating.py that owns
  last-write timestamp + previous mode/inhibit and exposes one `should_write(...)` call wrapping the
  existing `should_log_tick` logic. Main loop becomes one call. Drop the getattr default — use
  `config.log_disarmed_heartbeat_s` directly (the field exists, default 5.0).
- CHARACTERIZATION TEST REQUIRED FIRST: extend pi_app/tests/test_log_gating.py with a scripted tick
  sequence (armed/disarmed transitions, mode flips, charger flips, heartbeat-interval expiry, events)
  and assert `LogGate` yields the identical write/skip decision the current open-coded logic produces.
  Keep the existing pure `should_log_tick` tests passing (or fold them into the class tests without
  weakening assertions).
- Acceptance: no `_prev_tick_*` edge state open-coded in main.py; no `getattr(..., 5.0)` default;
  LogGate tests green; full suite green (paste summary).

TASK 5 — Single shared UPS status contract  (finding W5-3, STRONG) — GATED
- PRECONDITION: only start this if `wave5/i2c-politeness` has ALREADY merged into `origin/main` and
  you branched after it. If not, skip Task 5 and report.
- Scope: `UPS_STATUS_FILE` default (`/tmp/ups_status.json`) and the 9-key status payload
  (ts, ac_present, typec_mv, microusb_mv, batt_v, batt_ma, protect_mv, detect_only,
  seconds_without_charge) are duplicated in scripts/upsPlus_power_daemon.py and
  pi_app/web/oak_viewer.py with no single source of truth.
- Approach (in priority order):
  (a) PREFERRED: a single stdlib-only module (no Flask, no smbus) that both sides import — holding the
      default path/env resolution and the payload field names (a TypedDict/dataclass or a KEYS tuple)
      and MAX_AGE_S. BUT the daemon is a standalone script that today imports neither `pi_app` nor root
      `config`; its systemd launch context puts `scripts/` on sys.path, not the repo root. You MUST
      prove the daemon can import the shared module in its REAL launch context (simulate the actual
      `python scripts/upsPlus_power_daemon.py` invocation / systemd WorkingDirectory), not just under
      pytest. If a clean import isn't achievable without fragile sys.path hacks, do NOT force it.
  (b) FALLBACK (if (a) can't be made clean): keep the two constants where they are but add a CONTRACT
      TEST asserting the writer payload keys == the reader-consumed keys and the two `UPS_STATUS_FILE`
      defaults are equal — so drift fails CI without a runtime import that could break the daemon.
- Do NOT touch `_publish_status` cadence/gating (i2c-politeness owns it). Only the shared
  constant/schema is in scope.
- Acceptance: exactly one definition of the default path + payload field set (or a contract test that
  fails on drift); daemon still starts in its real launch context (prove it); full suite green (paste summary).

TASK 6 — OPTIONAL, low priority, only if cheap  (findings W34-5 + W5-6)
- W34-5: replace `getattr(_telem, "pack_low_latched", False)` in controller.py with an explicit typed
  field access on the telemetry object the controller already consumes (default False). Keep BOTH
  enforcement points (driver set_tracks hard-stop + controller slew-pin).
- W5-6: in oak_viewer.py's SSE serializer (~1491–1552), prefer direct/typed attribute access over
  `getattr(t, "...", default)` for fields `RecordingTelemetry` already declares — but KEEP the
  `_finite_or_none` numeric rounding. Skip any field that legitimately may be absent on partial objects.
- Only do Task 6 if it's a small, obviously-safe diff. If it grows or risks a KeyError on older
  telemetry objects, drop it and say so. Full suite green (paste summary).

FINAL REPORT (back to the human, do not merge)
- Branch name + commit SHAs (one per task).
- For Tasks 3 & 4: state that the characterization test was committed and green BEFORE the refactor.
- Task 2 grep-proof (empty output). Task 1 byte-identical proof. Task 5 import proof or contract-test.
- Final pytest summary line (>= 613 passed / 4 skipped).
- Anything you skipped (Task 5 if i2c-politeness hadn't landed; Task 6 if not cheap) and why.
```

---

## Verification gates baked into the prompt (summary)

- **Every task:** `.venv/bin/python -m pytest pi_app/tests/ -q` stays green at >= 613 passed / 4 skipped;
  pytest summary pasted per commit.
- **Task 1:** response-body byte diff vs the pre-refactor HTML string; oak_viewer.py line count drops.
- **Task 2:** `rg 'voltage_shutdown|_check_voltage_shutdown|_trigger_low_voltage_shutdown'` over `*.py`/`*.md`
  returns zero; `VescCanDriver.shutdown()` + UPS countdown names grep-proven intact.
- **Tasks 3 & 4:** characterization test committed, green on unmodified code first, still green after refactor.
- **Task 5:** exactly one path/schema definition (or a drift-catching contract test); daemon starts in its
  real launch context; gated behind the `wave5/i2c-politeness` merge.
