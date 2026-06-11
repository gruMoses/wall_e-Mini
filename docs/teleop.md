# Phone teleop — fail-safe `/drive`

A phone-browser driver for WALL-E that is **fail-safe by construction**: if the
internet/wifi drops, the phone sleeps, the tab crashes, or the WebSocket handler
thread dies, the robot stops. Stopping is enforced **server-side, in a watchdog
loop** — never by the browser and never by the WS handler.

Tranche 1 delivered the safety core; **tranche 2 (this doc) delivered the polished
UI.** Tranche 3 = auth/network hardening. See the bottom of this file.

## Where the code lives

- `pi_app/web/teleop.py` — everything:
  - `TeleopSession` — transport-agnostic, Flask-free safety state machine.
  - `TeleopSession._watchdog_loop` / `.tick()` — the server-side deadman.
  - `FileCommandSink` — production sink (writes the override file).
  - `register_teleop(app, session, …)` — `/drive` page + WebSocket + REST.
- `pi_app/web/oak_viewer.py` — **one additive block** in `OakWebViewer._run`
  instantiates the session, registers the routes, and starts the watchdog. No
  existing route or behavior is changed.
- `pi_app/tests/test_teleop.py` — mocked-motor unit tests.

## How it slots UNDER existing RC authority (RC stays boss)

The real motor path is the main loop in `pi_app/app/main.py`:

```
rc = rc_reader.get_state()
bt_override = <read /tmp/wall_e_bt_latest.json if fresh within 600 ms>
cmd = controller.process(rc, bt_override_bytes=bt_override)   # -> motor.set_tracks
```

`controller.process` already gates **everything** through RC: an RC-stale
watchdog (>1 s → disarm), `update_safety` (ch3 disarm, ch5 e-stop latch), and a
final `if not is_armed: left = right = NEUTRAL` (`controller.py:898`). Phone
teleop physically cannot move the robot unless RC keeps it armed.

Teleop does **not** touch the motor or `controller.process`. It writes the same
`/tmp/wall_e_bt_latest.json` override file the existing `/api/teleop` endpoint
already uses, so it rides the already-RC-gated path. On top of that file channel
it adds a tighter 250 ms deadman, an arming ceremony, a latched e-stop, a speed
cap, and a stale-command guard.

RC authority is enforced twice in teleop, independent of the downstream gate:
- `TeleopSession.tick()` reads `rc_state_provider()` first, every tick; if RC is
  not armed or RC e-stop is active it force-disarms the phone session.
- `notify_rc_state()` can force-disarm synchronously from any thread.

### RC ch5 e-stop path into teleop (DEFECT-2)
The teleop session has no direct view of the RC receiver; it learns RC state
from the OAK recorder's latest telemetry via `make_recorder_rc_state_provider`.
The provider returns `(rc_armed, rc_estop)` where:
- `rc_armed` ← `RecordingTelemetry.is_armed` (controller armed state), and
- `rc_estop` ← `RecordingTelemetry.emergency_active` (the latched **RC ch5**
  e-stop, mirrored from `controller.emergency_active`).

`emergency_active` is populated in `pi_app/app/main.py` at the existing
`RecordingTelemetry(...)` construction site from `cmd.emergency_active`. Before
this fix the provider read a field that did not exist, so an RC ch5 e-stop never
reached the phone session — it now force-disarms / handles the e-stop on the next
watchdog tick.

### Telemetry staleness fail-safe (DEFECT-3)
`RecordingTelemetry.ts_mono` carries a `time.monotonic()` stamp set at
construction. The provider fail-safes on a frozen telemetry object:
- **Never seen any telemetry** (recorder `None`, or no snapshot yet) → returns
  `None`: RC state is genuinely unknown, so the session is **not** force-disarmed
  (bench-mode behavior unchanged).
- **Has seen telemetry, but the newest is older than 1.0 s** → returns
  `(False, False)`, which the rc gate treats as "RC not armed" and force-disarms
  the phone session. A frozen telemetry object can never keep the session armable
  forever.

## Deadman enforcement path (exact)

1. The phone holds a WebSocket open and sends a heartbeat at 10 Hz
   (`{"type":"hb","t":<ms>}`) plus drive commands at ~15 Hz (which also count as
   heartbeats).
2. The **WS / REST handler only mutates session state** — it records the last
   heartbeat time, the drive intent, and the sequence number. It never writes to
   the motor sink.
3. A dedicated daemon thread, `TeleopSession._watchdog_loop`, calls
   `tick()` at 50 Hz (20 ms). `tick()` is the **sole writer** of the command
   sink. Every tick, if the session is armed and
   `now - last_heartbeat > 250 ms`, it:
   - sets `armed = False`, `tripped_reason = "deadman"`, and
   - writes **neutral** (`126, 126`) to the override file (belt), then stops
     writing so the file goes stale and the main loop drops `bt_override` to
     `None` within 600 ms → RC regains authority (suspenders).

Because only the watchdog writes the sink, a killed WS handler thread, a socket
close, a wifi drop, or a slept phone **all** converge on the same trip path. The
handler deliberately does **not** disarm on disconnect — proving the watchdog is
what stops the robot.

### Idle does not clobber RC
The watchdog writes the override file **only while actively driving** and emits a
single neutral on the driving→stopped edge. It does not stamp the file every idle
tick, which would otherwise make the main loop treat teleop as "fresh" forever and
suppress the RC sticks.

## Arm ceremony (the chosen gate)

A phone on the LAN must never arm-and-drive with zero ceremony. Arming requires:

1. **Press-and-hold "ARM" ≥ 500 ms** in the UI (`hold_ms` is sent and checked
   server-side), AND
2. **RC must currently be armed**, OR the operator checks **"RC is in my hand"**
   (`rc_in_hand: true`).

Rationale: the robot only moves under teleop if RC keeps it armed anyway
(downstream gate), so requiring RC-armed at the phone-arm step is natural and the
"RC in my hand" checkbox covers the arm-phone-then-arm-RC order. After **any**
deadman trip, RC override, or disconnect, reconnecting does **not** re-arm — an
explicit ARM hold is always required.

## E-stop

The UI's big red button sends `{"type":"estop"}`. Server-side this latches:
neutral + disarm + `estop_latched = True` + driver-lock released. E-stop is
accepted from **any** authenticated client — a non-driver phone can always stop
the robot. While latched, drive commands are ignored even with fresh heartbeats,
and arming is refused. The latch survives reconnects. This is independent of —
and does not interfere with — the RC ch5 e-stop.

### Clearing the latch: physical re-arm gate (DEFECT-4 / spec deviation)
The spec is "latched until physical re-arm." `clear_estop` returns `(ok, reason)`
and behaves by whether RC state is available to the session:

- **RC present** (the provider has ever delivered data): the latch is cleared
  **only after an RC ch3 cycle** — `rc_armed` observed going **False then True**
  *since* the e-stop fired (a physical re-arm on the transmitter). A browser-only
  clear before that cycle is **refused** with reason `cycle_rc_ch3`; the UI shows
  "CYCLE RC CH3 TO CLEAR" and `status.estop_clear_gated` is `true`. The ch3 edge
  is observed by the watchdog (`tick()`) and by `notify_rc_state()`.
- **Bench mode** (no RC state ever observed): the existing **two-tap browser
  clear** remains — first tap shows "TAP AGAIN TO CLEAR", second sends
  `clear_estop`, which succeeds.

A cleared latch never re-arms on its own — a fresh ARM hold is always required.

## Single-driver lock (DEFECT-1)

Only one client may hold an armed session at a time. The WS server issues a
per-connection `client_id` (a nonce, sent in a `{"type":"hello","client_id":…}`
frame on connect); REST callers may pass `?cid=`/`X-Teleop-Client`. The client
that completes the arm ceremony becomes the **driver**:

- `arm` and `drive` from any **other** client are refused (`reason:"not_driver"`
  / `result:"not_driver"`). Only the driver's `heartbeat` refreshes the deadman.
- **`estop` is accepted from ANY client** — a stop is never locked out.
- The lock is **released** on driver disconnect (`release_driver`, which also
  disarms the now-ownerless session), deadman trip, `disarm`, e-stop, or RC
  override.

`status` reports `has_driver` (someone holds the lock) and a per-client
`is_driver`. A spectator phone's UI shows "ANOTHER DRIVER ACTIVE" and disables the
ARM hold. A `None` client id (bench / single-client) is always treated as the
driver, so the existing single-phone and unit-test paths are unchanged.

## Speed cap

Server-side scale factor applied to the normalized track values before they are
converted to motor bytes: `slow=0.3`, `normal=0.6`, `fast=1.0`. Default **slow**.
Selectable from the UI; a change applies on the next watchdog tick.

## Protocol (WebSocket `/ws/drive`)

Client → server (JSON, one object per frame):

| type          | fields                                  | meaning |
|---------------|-----------------------------------------|---------|
| `arm`         | `hold_ms`, `rc_in_hand`                  | arm if ceremony satisfied |
| `disarm`      | —                                       | disarm session |
| `estop`       | —                                       | latched e-stop |
| `clear_estop` | —                                       | dismiss latch (does not arm) |
| `speed`       | `level` ∈ {slow,normal,fast}            | set speed cap |
| `drive`       | `seq` (monotonic int), `t` (client ms), `left`, `right` ∈ [-1,1] | drive intent + heartbeat |
| `hb`          | `t` (client ms)                         | bare heartbeat |

Server → client, ~5 Hz and on demand:

```
{"type":"status", "armed":bool, "estop_latched":bool, "speed_level":str,
 "speed_cap":float, "tripped_reason":str|null, "deadman_ms":250,
 "heartbeat_age_ms":int|null, "rc_state":[rc_armed,rc_estop]|null,
 "echo_ts":<last client t>, "battery_v":float|null, "server_t":epoch}
```

Round-trip latency: the client computes `performance.now() - echo_ts` from the
status frame.

**Stale-command guard:** a `drive` whose `seq` is not strictly greater than the
last accepted `seq` is dropped (out-of-order); a `drive` whose `t` is older than
the newest seen `t` by more than 300 ms is dropped (stale).

A REST mirror exists for every message (`POST /api/teleop/session/{arm,disarm,
estop,clear_estop,speed,drive}`, `GET /api/teleop/session/status`) as a fallback
if `flask-sock` is unavailable. The deadman is watchdog-enforced either way.

## Auth (minimum) and the `/api/teleop` exposure audit

- `/drive`, the WS upgrade, and the REST mirror honor an **optional** shared
  token from the `WALL_E_TELEOP_TOKEN` env var (`?token=…` or `X-Teleop-Token`).
  If unset (default), they are open on the trusted LAN — **tranche-3 gap**.
- **Audit finding:** the pre-existing `POST /api/teleop` (and `/api/teleop/stop`)
  endpoints have **no auth** — any host on the LAN can POST and write the
  override file. The blast radius is bounded by RC authority (the controller
  forces neutral unless RC-armed) and the 600 ms freshness window, but it is a
  real unauthenticated motor-command surface. Left intact (not in scope to
  change); flagged here for tranche 3.

## Dependencies

Adds `flask-sock>=0.7` (pulls in `simple-websocket`) to `requirements.txt`. It is
small, well-maintained, and works with Flask's threaded dev server (`app.run(
threaded=True)`) without eventlet/gevent — the WS runs on its own request thread,
matching the existing MJPEG/SSE streaming model. If it is missing at runtime the
code degrades to the REST fallback rather than crashing.

## UI layout (tranche 2, portrait iPhone)

```
┌─────────────────────────────────────┐
│  ● LINK UP   RTT 18ms  ARMED  BATT  │  HUD bar (2 rows of chips)
│  CAP 30%    HB 45ms   RC ARMED     │
├─────────────────────────────────────┤
│  [ SLOW ]   [ NORMAL ]   [ FAST ]  │  Speed segmented control
├─────────────────────────────────────┤
│  ╔═══════════════════════════════╗  │
│  ║          E — STOP             ║  │  Big red button (56 px tall)
│  ╚═══════════════════════════════╝  │
├─────────────────────────────────────┤
│  [ HOLD TO ARM   ◎ ]  ☐ RC in hand │  ARM button + ring + checkbox
├─────────────────────────────────────┤
│                                     │
│  ┌───────────┐    ┌───────────┐    │
│  │  ↑ fwd    │    │  ↑ fwd    │    │  Joystick pads
│  │   (knob)  │    │   (knob)  │    │  (flex-1, thumb zones)
│  │  ↓ rev    │    │  ↓ rev    │    │
│  └───────────┘    └───────────┘    │
│  L TRACK          R TRACK          │
└─────────────────────────────────────┘
```

### Joysticks
Tank-drive mapping: left pad = left track, right pad = right track. Vertical
axis only — drag up = forward (+1), drag down = reverse (−1), release =
spring-return to center (CSS `cubic-bezier(.34,1.56,.64,1)` spring). Each pad
uses Pointer Events with `setPointerCapture` for reliable multi-touch; knob
glows blue (forward) or red (reverse).

### ARM ceremony
Press-and-hold renders a radial progress ring (rAF, 520 ms fill) around the
button. On completion the arm message is sent with `hold_ms` + `rc_in_hand`.
The server still enforces ≥ 500 ms hold; the UI ring is purely cosmetic. While
`armed`, a single tap disarms. The `justArmed` flag prevents the click event
that ends the hold from immediately disarming.

### E-STOP
Immediate on `pointerdown` (not click): zeroes sticks, sends `{type:"estop"}`,
triggers haptic. Latched state pulses red. Two-step dismiss: first tap shows
"TAP AGAIN TO CLEAR" (3 s timeout), second tap sends `{type:"clear_estop"}`.
Re-arm still requires the full hold ceremony.

### Disconnect / deadman overlay
Full-screen blurred overlay is shown when:
- WS drops (auto-reconnect in background after 800 ms), or
- Deadman trips while armed (robot was moving).

The overlay must be explicitly dismissed before re-arming; it never auto-dismisses
or auto-re-arms — the server ceremony remains the only gate.

### Client-side safety (belt, in addition to server deadman)
- `visibilitychange` hidden: zeroes sticks + sends `drive` with 0/0 immediately.
- `pagehide`: same.
- `pointercancel` on each pad: zeroes that track immediately.

### PWA
- `/drive/manifest.json` — name "WALL-E Drive", standalone, portrait, dark
  `theme_color: #080a0f`.
- `/drive/icon.svg` — WALL-E glyph (boxy head, binocular eyes, twin track pods).
- `/drive/icon-{180,192,512}.png` — generated on first request by pure Python
  (numpy fast path, stdlib fallback); no extra pip deps.
- iOS Add-to-Home-Screen hint banner shown in mobile Safari when not in standalone
  mode; dismissed via `localStorage`.

## What tranche 3 will add

- **Tranche 3 (hardening):** real auth (per-device tokens / session cookies)
  closing the `WALL_E_TELEOP_TOKEN`-unset gap and the unauthenticated
  `/api/teleop`; rate limiting; TLS/`wss`; optional single-active-driver lock.
