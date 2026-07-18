# Charger-inhibit fail-open policy — decision memo

Owner: Kevin. No code changed by this memo. Cites against `main` @ `fc1ef31`.

## 1. Current behavior

**Signal.** `BmsService` polls the Daly BMS over BLE every `bms_poll_interval_s`
(8.0s); `bms_timeout_s` (30.0s) is both the BLE connect timeout and the
fail-open staleness threshold (`config.py:536-537`). Pack current and
charge-FET state come from the `soc`/`mosfet_status` polls
(`pi_app/hardware/bms.py:337-343,363-367`).

**Debounce (ON transition only).** A poll "looks like charging" iff
`charge_fet_on` AND `pack_current_a > charge_detect_min_current_a` (2.0A)
(`bms.py:410-416`). A streak counter increments on such polls, resets to 0 on
any other poll (`bms.py:418-424`). `is_charging()` reports confirmed-True
only once the streak hits `charge_detect_min_consecutive_polls` (2)
(`bms.py:195-224`, `config.py:548-549`). OFF is never debounced.

**Fail-open timer.** `is_charging()` checks `elapsed = now -
last_success_monotonic` *before* reading the debounced verdict: if never
polled or `elapsed > bms_timeout_s`, returns `False` unconditionally
(`bms.py:218-221`) — a stale/dropped link always wins over last-known state.

**Application.** `main.py:520-525` calls `controller.set_charger_inhibit(
bms_service.is_charging() or _bms_safety_stopped, ...)` every tick.
`Controller.set_charger_inhibit()` (`controller.py:222-246`) stores the flag,
logs a WARNING on each ENGAGE/RELEASE edge. In `process()`, enforced after
the armed-check, before the VESC-pack-low check: if armed and
`_charger_inhibit`, outputs forced to neutral + `motor.stop()` every tick
(`controller.py:1089-1108`), overriding RC/BT input.

`_bms_safety_stopped` (`main.py:416-573`) is a related, separate latch: if
`discharge_fet_on` reads False >`bms_fet_safety_timeout_s` (2s) after a 20s
post-arm grace window, it also forces neutral via the same call. That path is
fail-*closed* by construction (unreachable BMS can't fake `True` either) —
out of scope here, but on the same wire.

**RC/web e-stops, independent of all of the above.** `charger_inhibit`
enforcement is gated behind `is_armed`, but arm/disarm/e-stop are computed
earlier in `process()` and never read BMS state:
- RC ch5 rising edge latches `emergency_active`, force-disarms
  (`safety.py:70-80`).
- RC ch3 arm/disarm, 0.3s debounce (`safety.py:88-101`).
- RC-stale watchdog: no packet >1.0s force-disarms (`controller.py:688-708`).
- Web/teleop e-stop (`teleop.py:199-217`) — a separate latch at the phone
  session layer, accepted from any client, gates BT bytes before they reach
  the controller.

None consult BMS reachability — a strict superset of "can stop," never
weakened by charger-inhibit logic either way.

## 2. Failure-mode table

| Charging? | BLE | Policy | Outcome | Severity |
|---|---|---|---|---|
| No | up/down | either | Correct or coincidentally-correct: no inhibit. | — |
| Yes | up | either | Same: inhibit engages within ≤2 polls (~16s). | — |
| Yes | down >30s | **fail-open (today)** | **Robot on the charger drives off** — cable damage / dragged-charger risk on a 40kg platform. Invisible unless the owner notices or (d) below fires. | **High — the actual gap** |
| Yes | down >30s | fail-closed | Robot stays parked correctly until link recovers or overridden. If the BMS/radio simply died and nothing is charging, robot is stuck until intervention (mitigated by (b)). | Low (availability, not safety) |

**Real incident (2026-06-13, fixed by the debounce above):** BLE was healthy
throughout — orthogonal to the fail-open/closed axis. One poll read
`pack_current_a=+1.3A` (regen/noise blip, near-full pack, full-speed
FOLLOW_ME). Pre-debounce, that single sample flipped `is_charging()` True and
held it ~8s (one poll interval), forcing neutral mid-chase
(commit `de51ff5`, `bms.py:196-209`). A false-positive-charging bug, not a
BLE-loss bug — relevant to (c) below as a reminder this codebase has already
shipped one over-trusting-a-single-signal failure.

## 3. Alternatives

**(a) Keep fail-open.** Pro: simplest; matches the "never brick the robot"
philosophy already used for VESC low-voltage (delegated to BMS hard-cut, not
software — `vesc.py:508-513`). Con: the one real hazard here is completely
unmitigated and invisible — `get_state().connected` reaches `/api/state`
(`main.py:981-995`) but nothing alarms on it.

**(b) Fail-closed with RC-override.** Hold last-known state on timeout;
clear only on an RC ch3 arm-cycle (mirrors teleop's existing physical-rearm
gate, `teleop.py:219-232` — pattern already exists in this codebase). Pro:
closes the high-severity cell; recovery is a one-button action any
RC-holding operator already knows. Con: same gap if BLE is already down at
power-on/first 30s; widens "won't move" surface given the link is documented
flaky (8s poll/30s timeout already tuned around that, plus known 3-9min slow
BLE association). Needs new state threaded through `BmsService`.

**(c) Corroborate via VESC voltage / UPS AC.** VESC exposes pack voltage
independent of BLE (`CAN_PACKET_STATUS_5`, both controllers, CAN-bus rate —
`vesc.py:29-31,478-489,495-505`); same signal already backs the low-voltage
watchdog (`voltage_shutdown_threshold_v=39.0V`, `config.py:147`). Pro: wired,
not BLE, available exactly when BMS isn't. Con: voltage *rising* isn't a
clean charging signal here — it also rises the instant drive current drops,
indistinguishable from "charger connected" without correlating against
near-zero current over a sustained window (new logic, new edge cases, on a
codebase that already shipped one single-signal false positive). The UPS
`ac_present` field (`oak_viewer.py:86`, from `upsPlus_power_daemon.py`) is
**not verified to be electrically tied to the pack charger** — it's the Pi's
own 5V UPS rail, a separate power path per `docs/ups_shutdown_bench_test.md`
and `docs/ups_detect_only_test.md`. Using it without confirming the two plugs
are physically ganged risks a confidently-wrong signal, worse than none.

**(d) Fail-open + loud banner on link-loss-cleared-inhibit.** No control-flow
change: `is_charging()` + `get_state().connected` already distinguish
"not charging" from "not charging because unreachable." Surface on `/debug`
(SSE plumbing exists, `main.py:650`) plus a persistent banner and a distinct
journald line (transition-log pattern already at `controller.py:239-245`).
Pro: cheapest, adds no new way to strand the robot, targets the actual
pattern (charger overnight, BLE drops silently) by making it loud. Con:
doesn't stop the drive-off by itself if no one's watching — mitigation, not
fix.

## 4. Recommendation

Ship **(d) now, (b) as a follow-up** — not mutually exclusive, and (d) is
near-free. Reasoning: (a) leaves the only real hazard open and invisible.
(c) trades a well-understood BLE-flakiness problem for a harder-to-validate
voltage-inference problem, on a robot that already shipped one
single-signal false-positive bug; its UPS-AC variant is unverified wiring
and shouldn't be used until Kevin confirms the plugs are ganged. (b) is the
correct end state — it closes the high-severity cell and its failure mode
has a trivial, already-idiomatic recovery — but it's a real change to a
safety path and deserves its own review, not a rider here. (d) ships in an
afternoon, makes the gap observable today, and doesn't foreclose (b) later.
