# Phone teleop — fail-safe `/drive` (Tranche 1)

A phone-browser driver for WALL-E that is **fail-safe by construction**: if the
internet/wifi drops, the phone sleeps, the tab crashes, or the WebSocket handler
thread dies, the robot stops. Stopping is enforced **server-side, in a watchdog
loop** — never by the browser and never by the WS handler.

This is tranche 1 (the safety core). Tranche 2 = the "sexy" UI. Tranche 3 =
auth/network hardening. See the bottom of this file.

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
neutral + disarm + `estop_latched = True`. While latched, drive commands are
ignored even with fresh heartbeats, and arming is refused. The latch survives
reconnects. Clearing requires an explicit UI dismiss (`clear_estop`) **and** a
fresh ARM hold. This is independent of — and does not interfere with — the RC ch5
e-stop.

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

## What tranche 2 / 3 will add

- **Tranche 2 (UI):** the polished, "sexy" phone UI — proper joysticks/haptics,
  big legible status, camera/telemetry overlay, theming. The current `/drive`
  page is intentionally minimal.
- **Tranche 3 (hardening):** real auth (per-device tokens / session cookies)
  closing the `WALL_E_TELEOP_TOKEN`-unset gap and the unauthenticated
  `/api/teleop`; rate limiting; TLS/`wss`; optional single-active-driver lock.
