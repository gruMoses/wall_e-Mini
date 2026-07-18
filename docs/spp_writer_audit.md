# Bluetooth SPP control-writer audit & retirement

**Backlog item:** "spp_server third-writer retirement."
**Status:** Retired (gated OFF by default). Branch `fix/spp-writer-retirement`.

## Question

The robot's motion inputs should have exactly two writers:

1. **RC transmitter** via the Arduino — the safety authority (arm/disarm ch3,
   latched e-stop ch5).
2. **Web teleop** (`pi_app/web/teleop.py`) — its own arming ceremony, latched
   session e-stop, 250 ms server-side deadman, speed cap, and stale guard.

Could data arriving over the Bluetooth SPP server (`wall-e-spp.service` →
`pi_app/cli/spp_server.py`) also inject control/motion input — an unaudited
third writer bypassing the web session-safety work?

## Verdict: YES — SPP was a genuine control-input writer (not telemetry-only)

Confirmed end-to-end path (pre-fix):

| # | Location | What happens |
|---|----------|--------------|
| 1 | `pi_app/cli/spp_server.py` (V2 `parse_cmd2` / V1 `parse_v1`) | Inbound RFCOMM bytes → `left_i/right_i` (or floats) → `ints_to_bytes`/`floats_to_bytes` → `{left_byte, right_byte, last_update_epoch_s}` written to `/tmp/wall_e_bt_latest.json`. |
| 2 | `pi_app/app/main.py:435,479-488` | Main loop stats + reads that file; if `age <= 0.6 s`, sets `bt_override = (left_byte, right_byte)`. |
| 3 | `pi_app/app/main.py:527` | `controller.process(rc, bt_override_bytes=bt_override)`. |
| 4 | `pi_app/control/controller.py:827-828` | `elif bt_override_bytes is not None: left, right = bt_override_bytes` — the override becomes the track command directly (also branches at 850, 898-902, 915-920). |

So Bluetooth SPP data **could set the left/right track outputs**, via the exact
same override channel as web teleop, but **without any of the web-session
protections** — no arming ceremony, no session e-stop latch, no 250 ms deadman,
no speed cap, no stale guard. Its only safety was (a) the 600 ms file-freshness
fallback in `main.py` and (b) RC arming.

## What SPP could NOT do

- **Arm.** SPP explicitly ignores `ARM:` commands —
  `spp_server.py` V1 branch: `... startswith("ARM:") ... (ignored for safety)`.
  Arming is RC-only (`controller.update_safety`, ch3/ch5).
- **Bypass RC arming.** The final gate `controller.py:1090`
  (`if not self._safety_state.is_armed: left = right = CENTER_OUTPUT_VALUE`)
  forces neutral when RC has not armed. SPP could only command motion while RC
  held the robot armed.
- **Select mode or navigate.** SPP writes only `left_byte`/`right_byte`; nothing
  in the SPP path touches mode, waypoints, or Follow-Me.
- **Feed any e-stop or safety-critical path.** SPP only injects *positive*
  motion. No safety interlock depends on it, so retiring the channel is purely
  safety-additive (strictly fewer ways to command motion).

## Fix

Retire the SPP → motor-override channel behind an explicit opt-in env var,
**default OFF**, gated on the **writer side** (the standalone SPP process), so
`main.py` and the audited web-teleop path are untouched:

- `spp_server.control_output_enabled()` reads `WALL_E_SPP_CONTROL_ENABLED`
  (truthy: `1/true/yes/on`; default absent = OFF).
- `spp_server.write_control_override(...)` no-ops (writes nothing, returns
  `False`) when the gate is OFF; the SPP server still accepts connections and
  parses / ACKs / logs commands for bench diagnostics.
- The pybluez `import bluetooth` was deferred from module scope into
  `run_server()` so the gate (and its tests) run on hosts without a Bluetooth
  stack.

With the gate OFF, no fresh override file is produced by SPP, so
`controller.process` receives `bt_override=None` from the SPP path and the
command cannot move the tracks. To re-enable the legacy Android/BT bench-drive
path for **supervised** testing, set `WALL_E_SPP_CONTROL_ENABLED=1` on the
`wall-e-spp.service` unit.

## Tests

`pi_app/tests/test_spp_control_gate.py` proves: default/explicit-disable writes
nothing; a gated write never becomes a `main.py` `bt_override` (mirrors the
`main.py:479-488` freshness read); a gated write does not clobber a legitimate
teleop-written file; the opt-in path still writes the consumable schema; and the
module imports without pybluez.

## Notes

- `pi_app/cli/spp_server_backup.py` is a dead backup (referenced by no service
  or script) and still contains the old unconditional writes. Left as-is; it is
  not executed. Delete it in a later cleanup if desired.
- This changes **production BT-control default behavior**: the Android BT app no
  longer drives the robot unless the env var is set on the Pi's service unit.
  Not deployed by this branch.
