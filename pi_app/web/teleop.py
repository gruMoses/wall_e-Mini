"""Phone teleop: fail-safe session manager + server-side deadman watchdog.

TRANCHE 1 — the fail-safe core (not the pretty UI; that's tranche 2).

Kevin's requirement: a phone-browser driver that is *fail-safe* — if internet/
wifi drop, the robot stops. The whole point of this module is that stopping is
enforced by the **server**, in a dedicated watchdog loop, NOT by anything the
phone or the WebSocket handler does. A dead WS handler thread, a slept phone, a
dropped wifi link, a crashed browser tab — all converge on the same code path:
no fresh heartbeat → motor neutral → teleop session disarmed.

Architecture / how this slots UNDER existing RC authority
---------------------------------------------------------
The real motor command path is the main control loop in ``pi_app/app/main.py``:

    rc = rc_reader.get_state()
    bt_override = <read /tmp/wall_e_bt_latest.json if fresh within 600 ms>
    cmd = controller.process(rc, bt_override_bytes=bt_override)  # -> set_tracks

``controller.process`` already gates EVERYTHING through RC: an RC-stale watchdog,
``update_safety`` (ch3 disarm / ch5 e-stop latch), and a final
``if not is_armed: left = right = NEUTRAL`` (controller.py:898). So phone teleop
physically cannot move the robot unless RC keeps it armed. We do not change that.

This module is purely an additional, *tighter* fail-safe ON TOP of that existing
file channel:

  * The existing ``/api/teleop`` endpoint writes the same override file with a
    600 ms freshness window. That is a crude deadman (fall back to RC after
    600 ms) with no arming ceremony, no e-stop latch, no speed cap, no stale
    guard, and no explicit re-arm. We leave it untouched but supersede it for
    the ``/drive`` page.
  * ``TeleopSession`` owns a 250 ms server-side deadman, an arming ceremony, a
    latched e-stop, a speed cap, and a stale-command guard.
  * The **watchdog thread is the sole writer** to the command sink. The WS / REST
    handlers only mutate session state (last heartbeat, intent, seq). If the
    handler thread dies, the watchdog still runs and trips the deadman.

Defense in depth on a deadman trip:
  1. Belt: the watchdog writes NEUTRAL bytes to the override file immediately.
  2. Suspenders: it then stops writing, so the file goes stale and the main loop
     drops ``bt_override`` to ``None`` within 600 ms → RC regains authority.
Both independently bring the robot to a stop.

The core ``TeleopSession`` has **no Flask dependency** so it is unit-testable with
a mock motor sink and an injected clock. The Flask/WebSocket layer below is a
thin adapter.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Tuple

from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.io.bt_proto import floats_to_bytes

logger = logging.getLogger(__name__)

# Default speed caps applied server-side to the normalized [-1, 1] track values.
DEFAULT_SPEED_CAPS = {"slow": 0.3, "normal": 0.6, "fast": 1.0}
DEFAULT_SPEED_LEVEL = "slow"

# Fail-safe timing.
DEADMAN_TIMEOUT_S = 0.25   # >250 ms without a heartbeat while armed -> trip
STALE_COMMAND_S = 0.30     # drive commands older than this (by client clock) dropped
ARM_HOLD_MIN_MS = 500      # UI press-and-hold floor before an arm message is honored
WATCHDOG_HZ = 50.0         # watchdog tick rate (20 ms) — comfortably tighter than 250 ms


class TeleopSession:
    """Fail-safe state machine for one phone-teleop session.

    Thread-safe. All public methods take an optional ``now`` (monotonic seconds)
    so tests can drive the clock deterministically; production passes the real
    ``now_fn`` at construction and lets the watchdog supply ``now``.

    The sink ``command_sink(left_byte, right_byte)`` is the ONLY way motor
    commands leave this object. In production it writes the override file; in
    tests it is a mock motor's ``set_tracks``.
    """

    def __init__(
        self,
        *,
        command_sink: Callable[[int, int], None],
        rc_state_provider: Optional[Callable[[], Optional[Tuple[bool, bool]]]] = None,
        now_fn: Callable[[], float] = time.monotonic,
        deadman_timeout_s: float = DEADMAN_TIMEOUT_S,
        stale_command_s: float = STALE_COMMAND_S,
        arm_hold_min_ms: int = ARM_HOLD_MIN_MS,
        require_rc_arm: bool = True,
        speed_caps: Optional[dict] = None,
    ) -> None:
        self._sink = command_sink
        # rc_state_provider() -> (rc_armed: bool, rc_estop: bool) | None
        #   None  => RC state unknown (e.g. no telemetry yet) -> do not force-disarm.
        self._rc_state_provider = rc_state_provider
        self._now = now_fn
        self._deadman_s = float(deadman_timeout_s)
        self._stale_s = float(stale_command_s)
        self._arm_hold_min_ms = int(arm_hold_min_ms)
        self._require_rc_arm = bool(require_rc_arm)
        self._caps = dict(speed_caps or DEFAULT_SPEED_CAPS)

        self._lock = threading.RLock()
        # ---- session state ----
        self.armed: bool = False
        self.estop_latched: bool = False
        self.speed_level: str = DEFAULT_SPEED_LEVEL
        self.tripped_reason: Optional[str] = None
        self._last_hb: float = 0.0          # monotonic of last heartbeat/accepted drive
        self._last_seq: int = -1            # last accepted drive sequence number
        self._max_client_ts: float = float("-inf")  # newest client timestamp (ms) seen
        self._intent: Tuple[float, float] = (0.0, 0.0)  # last commanded (left_f, right_f)
        self._driving: bool = False         # are we actively writing drive bytes?
        self._last_echo_ts: Optional[float] = None  # last client ts, echoed for RTT calc
        # ---- watchdog thread ----
        self._wd_thread: Optional[threading.Thread] = None
        self._wd_stop = threading.Event()

    # -- arming ceremony ----------------------------------------------------

    def arm(self, *, rc_in_hand: bool = False, hold_ms: int = 0,
            now: Optional[float] = None) -> Tuple[bool, str]:
        """Attempt to arm the phone session.

        Refuses while the e-stop is latched. Requires the robot to be RC-armed,
        OR an explicit ``rc_in_hand`` confirmation ("RC is in my hand"). A phone
        on the LAN therefore can never arm-and-drive with zero ceremony: it needs
        the press-and-hold (``hold_ms``) AND either RC already armed or the
        explicit confirmation. (The downstream controller RC gate applies
        regardless — teleop still cannot move the robot unless RC keeps it armed.)
        """
        with self._lock:
            now = self._now() if now is None else now
            if self.estop_latched:
                return False, "estop_latched"
            if hold_ms < self._arm_hold_min_ms:
                return False, "hold_too_short"
            if self._require_rc_arm and not rc_in_hand:
                rc = self._read_rc_state()
                # rc is (rc_armed, rc_estop) or None (unknown).
                if rc is None or not rc[0] or rc[1]:
                    return False, "rc_not_armed"
            self.armed = True
            self.tripped_reason = None
            self._last_hb = now          # grace: first watchdog tick won't trip
            self._last_seq = -1
            self._max_client_ts = float("-inf")
            self._intent = (0.0, 0.0)
            return True, "armed"

    def disarm(self, reason: str = "user") -> None:
        with self._lock:
            self.armed = False
            self.tripped_reason = reason
            self._intent = (0.0, 0.0)
            self._stop_motor_if_driving()

    # -- e-stop (latched) ---------------------------------------------------

    def estop(self) -> None:
        """Server-side latched e-stop: neutral + disarm + latch.

        Independent of the RC ch5 e-stop. The latch survives reconnects and is
        cleared only by ``clear_estop`` followed by a fresh ``arm``.
        """
        with self._lock:
            self.estop_latched = True
            self.armed = False
            self.tripped_reason = "estop"
            self._intent = (0.0, 0.0)
            self._stop_motor_if_driving()

    def clear_estop(self) -> None:
        """Dismiss the latched e-stop. Does NOT re-arm — arm() must be called."""
        with self._lock:
            self.estop_latched = False
            if self.tripped_reason == "estop":
                self.tripped_reason = None

    # -- inputs from the phone ---------------------------------------------

    def heartbeat(self, *, client_ts: Optional[float] = None,
                  now: Optional[float] = None) -> None:
        """Liveness ping. Refreshes the deadman only while armed.

        After a trip the session is disarmed; heartbeats keep arriving but do
        NOT re-arm — re-arm requires an explicit ``arm`` message.
        """
        with self._lock:
            now = self._now() if now is None else now
            if client_ts is not None:
                self._last_echo_ts = client_ts
            if self.armed and not self.estop_latched:
                self._last_hb = now

    def drive(self, *, left_f: float, right_f: float, seq: Optional[int] = None,
              client_ts: Optional[float] = None, now: Optional[float] = None) -> str:
        """Apply a drive intent (also serves as a heartbeat when accepted).

        Stale-command guard: a command whose ``seq`` is not strictly greater than
        the last accepted seq is dropped (out-of-order). A command whose
        ``client_ts`` is older than the newest seen client timestamp by more than
        ``stale_command_s`` is dropped (stale). Commands are ignored entirely
        while e-stopped or not armed.

        Returns a short status string describing what happened.
        """
        with self._lock:
            now = self._now() if now is None else now
            if client_ts is not None:
                self._last_echo_ts = client_ts

            # Ordering / staleness guards run regardless of arm state so a stale
            # packet can never be "accepted" later.
            if seq is not None:
                if seq <= self._last_seq:
                    return "dropped_out_of_order"
            if client_ts is not None and self._max_client_ts != float("-inf"):
                # client_ts in ms; stale window in seconds.
                if client_ts < self._max_client_ts - (self._stale_s * 1000.0):
                    return "dropped_stale"

            if self.estop_latched:
                return "ignored_estop"
            if not self.armed:
                return "ignored_disarmed"

            # Accept: advance guards, refresh deadman, store intent.
            if seq is not None:
                self._last_seq = seq
            if client_ts is not None:
                self._max_client_ts = max(self._max_client_ts, client_ts)
            self._intent = (
                max(-1.0, min(1.0, float(left_f))),
                max(-1.0, min(1.0, float(right_f))),
            )
            self._last_hb = now
            return "accepted"

    def set_speed(self, level: str) -> bool:
        with self._lock:
            if level not in self._caps:
                return False
            self.speed_level = level
            return True

    def notify_rc_state(self, rc_armed: bool, rc_estop: bool) -> None:
        """RC authority hook. If RC is not armed or RC e-stop is active, force the
        phone session disarmed immediately — RC overrides phone state at all times.
        Safe to call from any thread; the watchdog also re-checks every tick.
        """
        with self._lock:
            if (not rc_armed) or rc_estop:
                if self.armed:
                    self.armed = False
                    self.tripped_reason = "rc_override"
                    self._intent = (0.0, 0.0)
                    self._stop_motor_if_driving()

    # -- the watchdog: server-side enforcement -----------------------------

    def tick(self, now: Optional[float] = None) -> str:
        """One watchdog iteration. THIS is where the deadman is enforced.

        Order of authority (highest first):
          1. RC override   — RC disarm / RC e-stop forces session disarm.
          2. Latched e-stop — UI big-red-button.
          3. Disarmed       — nothing to enforce; ensure a stop was emitted.
          4. Deadman        — armed but heartbeat older than 250 ms → trip.
          5. Driving        — armed + fresh → write the capped intent.

        Returns a short status for logging/telemetry. The watchdog thread calls
        this ~50 Hz; tests call it directly with a controlled ``now``.
        """
        with self._lock:
            now = self._now() if now is None else now

            # 1. RC authority — always wins.
            rc = self._read_rc_state()
            if rc is not None:
                rc_armed, rc_estop = rc
                if (not rc_armed) or rc_estop:
                    if self.armed:
                        self.armed = False
                        self.tripped_reason = "rc_override"
                    self._stop_motor_if_driving()
                    return "rc_override"

            # 2. Latched e-stop.
            if self.estop_latched:
                self._stop_motor_if_driving()
                return "estop"

            # 3. Disarmed: ensure we emitted a final neutral, then stay quiet so we
            #    don't suppress RC by continuously stamping the override file.
            if not self.armed:
                self._stop_motor_if_driving()
                return "idle"

            # 4. Deadman: armed but no fresh heartbeat.
            if (now - self._last_hb) > self._deadman_s:
                self.armed = False
                self.tripped_reason = "deadman"
                self._stop_motor_if_driving()
                return "deadman_trip"

            # 5. Armed + fresh: drive the capped intent.
            cap = self._caps.get(self.speed_level, self._caps[DEFAULT_SPEED_LEVEL])
            lf, rf = self._intent
            left_byte, right_byte = floats_to_bytes(lf * cap, rf * cap)
            self._sink(left_byte, right_byte)
            self._driving = True
            return "driving"

    def start_watchdog(self) -> None:
        """Spawn the daemon watchdog thread (production)."""
        with self._lock:
            if self._wd_thread is not None and self._wd_thread.is_alive():
                return
            self._wd_stop.clear()
            self._wd_thread = threading.Thread(
                target=self._watchdog_loop, name="TeleopWatchdog", daemon=True
            )
            self._wd_thread.start()
            logger.info("Teleop watchdog started (deadman=%dms, %.0f Hz)",
                        int(self._deadman_s * 1000), WATCHDOG_HZ)

    def stop_watchdog(self) -> None:
        self._wd_stop.set()
        with self._lock:
            self._stop_motor_if_driving()

    def _watchdog_loop(self) -> None:
        period = 1.0 / WATCHDOG_HZ
        while not self._wd_stop.is_set():
            try:
                self.tick()
            except Exception:  # never let the safety loop die
                logger.exception("Teleop watchdog tick failed")
            time.sleep(period)

    # -- status -------------------------------------------------------------

    def status(self, *, now: Optional[float] = None) -> dict:
        with self._lock:
            now = self._now() if now is None else now
            hb_age_ms = None
            if self.armed and self._last_hb > 0.0:
                hb_age_ms = int(round((now - self._last_hb) * 1000.0))
            return {
                "armed": self.armed,
                "estop_latched": self.estop_latched,
                "speed_level": self.speed_level,
                "speed_cap": self._caps.get(self.speed_level),
                "tripped_reason": self.tripped_reason,
                "deadman_ms": int(self._deadman_s * 1000),
                "heartbeat_age_ms": hb_age_ms,
                "rc_state": self._read_rc_state(),
                "echo_ts": self._last_echo_ts,
            }

    # -- internals ----------------------------------------------------------

    def _read_rc_state(self) -> Optional[Tuple[bool, bool]]:
        if self._rc_state_provider is None:
            return None
        try:
            return self._rc_state_provider()
        except Exception:
            logger.exception("rc_state_provider raised")
            return None

    def _stop_motor_if_driving(self) -> None:
        """Emit ONE neutral command if we were driving, then stop writing.

        Writing neutral only on the driving->stopped edge (rather than every idle
        tick) is deliberate: continuously stamping the override file would make
        the main loop treat teleop as 'fresh' forever and suppress the RC sticks.
        On a real stop we emit the immediate neutral (belt) and then let the file
        go stale so RC regains authority (suspenders).
        """
        if self._driving:
            try:
                self._sink(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
            except Exception:
                logger.exception("Teleop neutral write failed")
            self._driving = False


# ---------------------------------------------------------------------------
# Production command sink: write the shared override file the main loop reads.
# ---------------------------------------------------------------------------

class FileCommandSink:
    """Writes ``/tmp/wall_e_bt_latest.json`` in the schema ``main.py`` consumes.

    Same file + schema as the existing ``/api/teleop`` endpoint, so this rides
    the already-audited RC-gated override path rather than touching the motor.
    """

    def __init__(self, path: str = "/tmp/wall_e_bt_latest.json") -> None:
        self._path = Path(path)

    def __call__(self, left_byte: int, right_byte: int) -> None:
        self._path.write_text(
            json.dumps({
                "left_byte": int(left_byte),
                "right_byte": int(right_byte),
                "last_update_epoch_s": time.time(),
            }),
            encoding="utf-8",
        )


def make_recorder_rc_state_provider(recorder) -> Callable[[], Optional[Tuple[bool, bool]]]:
    """Build an rc_state_provider from the OAK recorder's latest telemetry.

    Returns (rc_armed, rc_estop) where rc_estop reflects the latched RC ch5
    emergency. Returns None when telemetry is unavailable so the session does not
    force-disarm on a transient read miss.
    """
    def _provider() -> Optional[Tuple[bool, bool]]:
        if recorder is None:
            return None
        try:
            t = recorder.get_latest_telemetry()
        except Exception:
            return None
        if t is None:
            return None
        rc_armed = bool(getattr(t, "is_armed", False))
        rc_estop = bool(getattr(t, "emergency_active", False))
        return rc_armed, rc_estop
    return _provider


def make_recorder_battery_provider(recorder) -> Callable[[], Optional[float]]:
    """Best-effort pack voltage from telemetry for the UI status line."""
    def _provider() -> Optional[float]:
        if recorder is None:
            return None
        try:
            t = recorder.get_latest_telemetry()
        except Exception:
            return None
        if t is None:
            return None
        v = getattr(t, "bms_voltage_v", None)
        return float(v) if isinstance(v, (int, float)) else None
    return _provider


# ---------------------------------------------------------------------------
# Flask / WebSocket adapter (thin — all enforcement lives in TeleopSession)
# ---------------------------------------------------------------------------

def register_teleop(app, session: "TeleopSession", *, token: str = "",
                    battery_provider: Optional[Callable[[], Optional[float]]] = None) -> None:
    """Register the /drive page, the drive WebSocket, and REST fallbacks on ``app``.

    Additive only: it adds new routes and does not modify any existing route.
    ``token`` (if non-empty) is required as ``?token=`` on /drive and on the WS
    upgrade. The WS / REST handlers ONLY mutate session state — the watchdog
    thread is what reaches the motor, so a dying handler cannot keep the robot
    driving.
    """
    from flask import Blueprint, Response, request

    bp = Blueprint("teleop", __name__)

    def _auth_ok() -> bool:
        if not token:
            return True  # open on the trusted LAN (see docs/teleop.md — tranche-3 gap)
        supplied = request.args.get("token") or request.headers.get("X-Teleop-Token")
        return supplied == token

    def _json(obj, status=200):
        return Response(json.dumps(obj), status=status, content_type="application/json")

    def _battery_v():
        if battery_provider is None:
            return None
        try:
            return battery_provider()
        except Exception:
            return None

    def _status_payload():
        st = session.status()
        st["battery_v"] = _battery_v()
        st["server_t"] = time.time()
        return st

    # -- page ---------------------------------------------------------------

    @bp.route("/drive")
    def drive_page():
        if not _auth_ok():
            return Response("unauthorized", status=401)
        return Response(_DRIVE_HTML, content_type="text/html")

    # -- REST fallbacks (mirror the WS messages; deadman still watchdog-enforced)

    @bp.route("/api/teleop/session/arm", methods=["POST"])
    def rest_arm():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        p = request.get_json(silent=True) or {}
        ok, reason = session.arm(rc_in_hand=bool(p.get("rc_in_hand", False)),
                                 hold_ms=int(p.get("hold_ms", 0)))
        return _json({"ok": ok, "reason": reason, "status": _status_payload()})

    @bp.route("/api/teleop/session/disarm", methods=["POST"])
    def rest_disarm():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        session.disarm()
        return _json({"ok": True, "status": _status_payload()})

    @bp.route("/api/teleop/session/estop", methods=["POST"])
    def rest_estop():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        session.estop()
        return _json({"ok": True, "status": _status_payload()})

    @bp.route("/api/teleop/session/clear_estop", methods=["POST"])
    def rest_clear_estop():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        session.clear_estop()
        return _json({"ok": True, "status": _status_payload()})

    @bp.route("/api/teleop/session/speed", methods=["POST"])
    def rest_speed():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        p = request.get_json(silent=True) or {}
        ok = session.set_speed(str(p.get("level", "")))
        return _json({"ok": ok, "status": _status_payload()})

    @bp.route("/api/teleop/session/drive", methods=["POST"])
    def rest_drive():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        p = request.get_json(silent=True) or {}
        result = session.drive(
            left_f=float(p.get("left", 0.0)), right_f=float(p.get("right", 0.0)),
            seq=p.get("seq"), client_ts=p.get("t"),
        )
        return _json({"result": result, "status": _status_payload()})

    @bp.route("/api/teleop/session/status")
    def rest_status():
        if not _auth_ok():
            return _json({"error": "unauthorized"}, 401)
        return _json(_status_payload())

    # -- PWA assets (additive; no auth — manifest/icons are public) ----------

    @bp.route("/drive/manifest.json")
    def drive_manifest():
        return Response(_MANIFEST_JSON, content_type="application/manifest+json")

    @bp.route("/drive/icon.svg")
    def drive_icon_svg():
        return Response(_ROBOT_SVG, content_type="image/svg+xml")

    @bp.route("/drive/icon-<int:size>.png")
    def drive_icon_png(size):
        if size not in (180, 192, 512):
            from flask import abort as _abort
            _abort(404)
        return Response(_get_robot_png(size), content_type="image/png")

    app.register_blueprint(bp)

    # -- WebSocket ----------------------------------------------------------

    try:
        from flask_sock import Sock
    except Exception:
        logger.warning("flask-sock not installed — /drive WS disabled, REST fallback only")
        return

    sock = Sock(app)

    @sock.route("/ws/drive")
    def ws_drive(ws):  # pragma: no cover - exercised via integration, not unit tests
        from flask import request as _req
        if token:
            supplied = _req.args.get("token")
            if supplied != token:
                try:
                    ws.send(json.dumps({"type": "error", "error": "unauthorized"}))
                finally:
                    ws.close()
                return

        stop = threading.Event()

        def _pump_status():
            # Push status ~5 Hz so the phone always has a fresh deadman/arm/latency view.
            while not stop.is_set():
                try:
                    ws.send(json.dumps({"type": "status", **_status_payload()}))
                except Exception:
                    break
                time.sleep(0.2)

        sender = threading.Thread(target=_pump_status, name="TeleopWsStatus", daemon=True)
        sender.start()
        try:
            while True:
                raw = ws.receive()  # blocks; returns None on close
                if raw is None:
                    break
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue
                _dispatch_ws(session, msg)
        finally:
            stop.set()
            # Intentionally NOT disarming here: the deadman watchdog trips within
            # 250 ms of the heartbeats stopping. Proving that path is the whole
            # point — a killed handler must not be what stops the robot.


def _dispatch_ws(session: "TeleopSession", msg: dict) -> None:
    mtype = msg.get("type")
    if mtype == "drive":
        session.drive(
            left_f=float(msg.get("left", 0.0)), right_f=float(msg.get("right", 0.0)),
            seq=msg.get("seq"), client_ts=msg.get("t"),
        )
    elif mtype == "hb":
        session.heartbeat(client_ts=msg.get("t"))
    elif mtype == "arm":
        session.arm(rc_in_hand=bool(msg.get("rc_in_hand", False)),
                    hold_ms=int(msg.get("hold_ms", 0)))
    elif mtype == "disarm":
        session.disarm()
    elif mtype == "estop":
        session.estop()
    elif mtype == "clear_estop":
        session.clear_estop()
    elif mtype == "speed":
        session.set_speed(str(msg.get("level", "")))


# ---------------------------------------------------------------------------
# PWA assets: manifest, SVG icon, and pure-Python PNG icon generator
# (no external image library required).
# ---------------------------------------------------------------------------

_MANIFEST_JSON = json.dumps({
    "name": "WALL-E Drive",
    "short_name": "W-E Drive",
    "description": "Fail-safe phone controller for WALL-E Mini",
    "start_url": "/drive",
    "display": "standalone",
    "orientation": "portrait",
    "theme_color": "#080a0f",
    "background_color": "#080a0f",
    "icons": [
        {"src": "/drive/icon.svg",     "sizes": "any",     "type": "image/svg+xml", "purpose": "any maskable"},
        {"src": "/drive/icon-180.png", "sizes": "180x180", "type": "image/png"},
        {"src": "/drive/icon-192.png", "sizes": "192x192", "type": "image/png"},
        {"src": "/drive/icon-512.png", "sizes": "512x512", "type": "image/png"},
    ],
}, separators=(",", ":"))

# Simple WALL-E robot glyph: binocular eyes, boxy head/body, twin track pods.
_ROBOT_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512">'
    '<rect width="512" height="512" rx="96" fill="#080a0f"/>'
    # tracks
    '<rect x="86" y="318" width="80" height="140" rx="40" fill="#1a1d27" stroke="#2563eb" stroke-width="8"/>'
    '<rect x="346" y="318" width="80" height="140" rx="40" fill="#1a1d27" stroke="#2563eb" stroke-width="8"/>'
    # body
    '<rect x="120" y="230" width="272" height="200" rx="32" fill="#1a1d27" stroke="#2563eb" stroke-width="8"/>'
    # head
    '<rect x="150" y="100" width="212" height="158" rx="24" fill="#1a1d27" stroke="#2563eb" stroke-width="8"/>'
    # eyes (binocular rings)
    '<circle cx="210" cy="172" r="44" fill="#080a0f" stroke="#2563eb" stroke-width="8"/>'
    '<circle cx="210" cy="172" r="26" fill="#2563eb"/>'
    '<circle cx="302" cy="172" r="44" fill="#080a0f" stroke="#2563eb" stroke-width="8"/>'
    '<circle cx="302" cy="172" r="26" fill="#2563eb"/>'
    # antenna
    '<line x1="256" y1="100" x2="256" y2="56" stroke="#2563eb" stroke-width="10" stroke-linecap="round"/>'
    '<circle cx="256" cy="48" r="18" fill="#2563eb"/>'
    '</svg>'
)

_PNG_CACHE: dict = {}


def _get_robot_png(size: int) -> bytes:
    if size not in _PNG_CACHE:
        _PNG_CACHE[size] = _robot_png_bytes(size)
    return _PNG_CACHE[size]


def _robot_png_bytes(size: int) -> bytes:
    """Generate a WALL-E glyph PNG icon.  Uses numpy when available (fast);
    falls back to pure Python without any extra deps."""
    import struct
    import zlib

    bg   = (8,   10,  15)   # #080a0f
    body = (26,  29,  39)   # #1a1d27
    blue = (37,  99,  235)  # #2563eb

    try:
        import numpy as np
        img = np.empty((size, size, 3), dtype=np.uint8)
        img[:, :] = bg

        Y, X = np.mgrid[0:size, 0:size]
        NY = Y / size
        NX = X / size

        head  = (NX >= 0.29) & (NX <= 0.71) & (NY >= 0.14) & (NY <= 0.45)
        bod   = (NX >= 0.23) & (NX <= 0.77) & (NY >= 0.43) & (NY <= 0.83)
        lt    = (NX >= 0.10) & (NX <= 0.26) & (NY >= 0.55) & (NY <= 0.92)
        rt    = (NX >= 0.74) & (NX <= 0.90) & (NY >= 0.55) & (NY <= 0.92)
        struct_m = head | bod | lt | rt

        le  = (NX - 0.41) ** 2 + (NY - 0.30) ** 2 < 0.070 ** 2
        re  = (NX - 0.59) ** 2 + (NY - 0.30) ** 2 < 0.070 ** 2
        ant = (NX - 0.50) ** 2 + (NY - 0.08) ** 2 < 0.050 ** 2
        stm = (NX >= 0.485) & (NX <= 0.515) & (NY >= 0.08) & (NY <= 0.17)
        accent_m = le | re | ant | stm

        img[struct_m]  = body
        img[accent_m]  = blue
        pixels = img

    except ImportError:
        rows_list = []
        for y in range(size):
            ny = y / size
            row = []
            for x in range(size):
                nx = x / size
                if ((nx - 0.41) ** 2 + (ny - 0.30) ** 2 < 0.070 ** 2 or
                        (nx - 0.59) ** 2 + (ny - 0.30) ** 2 < 0.070 ** 2 or
                        (nx - 0.50) ** 2 + (ny - 0.08) ** 2 < 0.050 ** 2 or
                        (0.485 <= nx <= 0.515 and 0.08 <= ny <= 0.17)):
                    row.append(blue)
                elif ((0.29 <= nx <= 0.71 and 0.14 <= ny <= 0.45) or
                      (0.23 <= nx <= 0.77 and 0.43 <= ny <= 0.83) or
                      (0.10 <= nx <= 0.26 and 0.55 <= ny <= 0.92) or
                      (0.74 <= nx <= 0.90 and 0.55 <= ny <= 0.92)):
                    row.append(body)
                else:
                    row.append(bg)
            rows_list.append(row)
        pixels = rows_list

    def png_chunk(tag: bytes, data: bytes) -> bytes:
        hdr = tag + data
        return struct.pack(">I", len(data)) + hdr + struct.pack(">I", zlib.crc32(hdr) & 0xFFFFFFFF)

    raw = bytearray()
    if hasattr(pixels, "flatten"):          # numpy path
        for row_arr in pixels:
            raw.append(0)                   # filter = None
            raw.extend(row_arr.flatten().tolist())
    else:                                   # pure-Python path
        for row_list in pixels:
            raw.append(0)
            for (r, g, b) in row_list:
                raw.extend((r, g, b))

    ihdr = png_chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
    idat = png_chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    iend = png_chunk(b"IEND", b"")
    return b"\x89PNG\r\n\x1a\n" + ihdr + idat + iend


# ---------------------------------------------------------------------------
# Polished phone-first /drive UI  (tranche 2 — the "sexy" UI)
# ---------------------------------------------------------------------------
_DRIVE_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="WALL-E Drive">
<meta name="theme-color" content="#080a0f">
<link rel="manifest" href="/drive/manifest.json">
<link rel="apple-touch-icon" href="/drive/icon.svg">
<title>WALL-E Drive</title>
<style>
:root{
  --bg:#080a0f;--s1:#111318;--s2:#1a1d27;--bd:#252836;
  --blue:#2563eb;--blue-lo:rgba(37,99,235,.13);--blue-md:rgba(37,99,235,.32);
  --green:#10b981;--green-lo:rgba(16,185,129,.13);--green-md:rgba(16,185,129,.3);
  --red:#ef4444;--red-lo:rgba(239,68,68,.13);--red-md:rgba(239,68,68,.32);
  --amber:#f59e0b;
  --text:#f1f5f9;--t2:#94a3b8;--t3:#475569;
  color-scheme:dark;
}
*,*::before,*::after{
  box-sizing:border-box;
  -webkit-user-select:none;user-select:none;
  -webkit-tap-highlight-color:transparent;
}
html,body{margin:0;height:100%;overflow:hidden;}
body{
  background:var(--bg);color:var(--text);
  font:700 13px/1 -apple-system,"SF Pro Display",system-ui,sans-serif;
  display:flex;flex-direction:column;
  padding-top:env(safe-area-inset-top);
  padding-bottom:env(safe-area-inset-bottom);
  padding-left:env(safe-area-inset-left);
  padding-right:env(safe-area-inset-right);
}
/* ---- overlay ---- */
#overlay{
  position:fixed;inset:0;z-index:100;
  background:rgba(8,10,15,.94);
  backdrop-filter:blur(14px);-webkit-backdrop-filter:blur(14px);
  display:flex;flex-direction:column;
  align-items:center;justify-content:center;gap:13px;
}
#overlay.hidden{display:none;}
.ov-icon{font-size:58px;line-height:1;animation:blink 1.4s ease-in-out infinite;}
.ov-title{font-size:24px;font-weight:800;color:var(--red);letter-spacing:.06em;}
.ov-sub{font-size:12px;color:var(--t2);text-align:center;max-width:270px;line-height:1.5;}
.ov-btn{
  margin-top:8px;padding:13px 44px;
  border:1.5px solid var(--bd);border-radius:12px;
  background:var(--s2);color:var(--text);
  font:inherit;font-size:14px;letter-spacing:.08em;
  transition:background .12s;
}
.ov-btn:active{background:var(--s1);}
/* ---- A2HS banner ---- */
#a2hs{
  background:var(--s1);border-bottom:1px solid var(--bd);
  padding:9px 14px;display:flex;align-items:center;
  gap:10px;font-size:11px;color:var(--t2);flex-shrink:0;
}
#a2hs.hidden{display:none;}
#a2hs-msg{flex:1;line-height:1.4;}
#a2hs-dismiss{
  border:1px solid var(--bd);border-radius:6px;
  background:var(--s2);color:var(--t2);
  font:inherit;padding:4px 9px;flex-shrink:0;
}
/* ---- HUD ---- */
.hud{
  background:var(--s1);border-bottom:1px solid var(--bd);
  padding:8px 14px;flex-shrink:0;
}
.hud-r{display:flex;align-items:center;gap:6px;}
.hud-r+.hud-r{margin-top:5px;}
.sp{flex:1;}
.dot{
  width:8px;height:8px;border-radius:50%;background:var(--t3);flex-shrink:0;
  transition:background .3s,box-shadow .3s;
}
.dot.live{background:var(--green);box-shadow:0 0 0 3px rgba(16,185,129,.18);animation:hbp 1.2s ease-in-out infinite;}
.dot.dead{background:var(--red);box-shadow:0 0 0 3px rgba(239,68,68,.18);}
.lnk{font-size:10px;letter-spacing:.06em;color:var(--t2);transition:color .3s;}
.lnk.live{color:var(--green);}.lnk.dead{color:var(--red);}
.chip{
  font-size:10px;letter-spacing:.05em;
  border:1px solid var(--bd);border-radius:6px;
  padding:3px 7px;background:var(--s2);color:var(--t2);white-space:nowrap;
  transition:all .2s;
}
.chip.ok{background:var(--green-lo);border-color:var(--green);color:var(--green);}
.chip.warn{border-color:var(--amber);color:var(--amber);}
.chip.bad{background:var(--red-lo);border-color:var(--red);color:var(--red);animation:blink .9s infinite;}
/* ---- speed seg ---- */
.spd-seg{
  display:flex;flex-shrink:0;
  margin:7px 12px 0;
  background:var(--s1);border:1px solid var(--bd);
  border-radius:10px;padding:3px;gap:2px;
}
.seg{
  flex:1;padding:8px 2px;border:none;border-radius:7px;
  background:transparent;color:var(--t3);
  font:700 10px/1 inherit;letter-spacing:.1em;
  transition:background .14s,color .14s,box-shadow .14s;
}
.seg.on{background:var(--blue);color:#fff;box-shadow:0 2px 10px var(--blue-md);}
/* ---- e-stop ---- */
.estop-wrap{padding:8px 12px 4px;flex-shrink:0;}
.estop{
  width:100%;height:56px;border:none;border-radius:14px;
  background:var(--red);color:#fff;
  font:800 17px/1 inherit;letter-spacing:.14em;
  box-shadow:0 4px 22px var(--red-md),0 2px 4px rgba(0,0,0,.4);
  transition:transform .08s,box-shadow .08s;
  position:relative;overflow:hidden;
}
.estop::after{
  content:'';position:absolute;inset:0;
  background:rgba(255,255,255,0);transition:background .1s;
}
.estop:active{transform:scaleY(.96);box-shadow:0 2px 10px var(--red-md);}
.estop:active::after{background:rgba(255,255,255,.08);}
.estop.latched{
  background:#fff;color:var(--red);
  box-shadow:0 0 0 3px var(--red),0 4px 22px var(--red-md);
  animation:latch 1.1s ease-in-out infinite;
}
.estop.dim1{
  background:rgba(239,68,68,.15);color:#fff;
  border:2px solid var(--red);box-shadow:none;animation:none;
}
/* ---- arm row ---- */
.arm-row{display:flex;align-items:center;gap:10px;padding:6px 12px;flex-shrink:0;}
.arm-outer{flex:1;position:relative;}
.arm-btn{
  width:100%;height:44px;border-radius:10px;
  border:1.5px solid var(--bd);background:var(--s2);
  color:var(--t2);font:700 12px/1 inherit;letter-spacing:.07em;
  transition:background .15s,color .15s,border-color .15s,box-shadow .15s;
}
.arm-btn.arming{border-color:var(--green);color:var(--green);background:rgba(16,185,129,.05);}
.arm-btn.armed{background:var(--green-lo);border-color:var(--green);color:var(--green);box-shadow:0 2px 14px var(--green-lo);}
.arm-btn:disabled{opacity:.38;pointer-events:none;}
/* radial progress ring */
.arm-ring{
  position:absolute;top:-7px;right:-7px;
  width:30px;height:30px;pointer-events:none;overflow:visible;
}
.r-bg{fill:none;stroke:var(--bd);stroke-width:2.5;}
.r-fg{
  fill:none;stroke:var(--green);stroke-width:2.5;
  stroke-linecap:round;
  stroke-dasharray:94.25;stroke-dashoffset:94.25;
}
.rc-lbl{
  display:flex;align-items:center;gap:6px;
  font-size:11px;color:var(--t2);flex-shrink:0;
  cursor:pointer;touch-action:manipulation;
}
.rc-lbl input{width:16px;height:16px;accent-color:var(--blue);cursor:pointer;}
/* ---- sticks ---- */
.sticks{flex:1;display:flex;gap:10px;padding:6px 12px 8px;min-height:0;}
.stick-col{flex:1;display:flex;flex-direction:column;gap:4px;min-height:0;}
.slbl{text-align:center;font-size:9px;font-weight:700;color:var(--t3);letter-spacing:.14em;flex-shrink:0;}
.pad{
  flex:1;position:relative;overflow:hidden;
  background:var(--s1);border:1px solid var(--bd);
  border-radius:18px;touch-action:none;
  box-shadow:inset 0 2px 12px rgba(0,0,0,.5);
  transition:border-color .15s,box-shadow .15s;
}
.pad::after{
  content:'';position:absolute;left:8px;right:8px;top:50%;height:1px;
  background:var(--bd);pointer-events:none;
}
.pad.fwd{border-color:rgba(37,99,235,.45);box-shadow:inset 0 2px 12px rgba(0,0,0,.5),0 0 0 1px rgba(37,99,235,.12);}
.pad.rev{border-color:rgba(239,68,68,.45);box-shadow:inset 0 2px 12px rgba(0,0,0,.5),0 0 0 1px rgba(239,68,68,.12);}
.pfill{
  position:absolute;left:5px;right:5px;
  border-radius:5px;pointer-events:none;
}
.pknob{
  position:absolute;width:52px;height:52px;border-radius:50%;
  background:radial-gradient(circle at 38% 32%,#3c4263,#1a1d27);
  border:1.5px solid #3a3f52;
  box-shadow:0 4px 14px rgba(0,0,0,.65),0 0 0 1px rgba(255,255,255,.04);
  transform:translate(-50%,-50%);
  left:50%;top:50%;
  pointer-events:none;
  transition:top .22s cubic-bezier(.34,1.56,.64,1);
}
.pknob::before{
  content:'';position:absolute;
  width:10px;height:10px;border-radius:50%;
  background:rgba(255,255,255,.07);
  top:10px;left:50%;transform:translateX(-50%);
}
.pknob::after{
  content:'';position:absolute;inset:-4px;
  border-radius:50%;border:2px solid transparent;transition:border-color .12s;
}
.pad.fwd .pknob::after{border-color:rgba(37,99,235,.5);}
.pad.rev .pknob::after{border-color:rgba(239,68,68,.5);}
.pknob.drag{transition:none;}
/* ---- keyframes ---- */
@keyframes blink{0%,100%{opacity:1;}50%{opacity:.55;}}
@keyframes hbp{
  0%,100%{opacity:1;box-shadow:0 0 0 3px rgba(16,185,129,.18);}
  50%{opacity:.7;box-shadow:0 0 0 5px rgba(16,185,129,.07);}
}
@keyframes latch{
  0%,100%{box-shadow:0 0 0 3px var(--red),0 4px 22px var(--red-md);}
  50%{box-shadow:0 0 0 6px var(--red),0 6px 30px var(--red-md);}
}
/* ---- landscape compact ---- */
@media(orientation:landscape)and(max-height:480px){
  .hud{padding:4px 14px;}
  .hud-r+.hud-r{display:none;}
  .spd-seg{margin:4px 12px 0;}
  .seg{padding:6px 2px;font-size:9px;}
  .estop-wrap{padding:4px 12px;}
  .estop{height:44px;font-size:14px;}
  .arm-row{padding:3px 12px;}
  .arm-btn{height:36px;}
  .sticks{padding:4px 12px 5px;}
}
</style>
</head>
<body>

<!-- disconnected / deadman overlay — requires explicit dismiss before re-arm -->
<div id="overlay" class="hidden">
  <div class="ov-icon">&#9888;</div>
  <div class="ov-title" id="ov-title">DISCONNECTED</div>
  <div class="ov-sub"  id="ov-sub">Robot stopped &middot; reconnecting&hellip;</div>
  <button class="ov-btn" id="ov-btn">DISMISS</button>
</div>

<!-- iOS Add-to-Home-Screen hint (dismissible, not shown in standalone mode) -->
<div id="a2hs" class="hidden">
  <span id="a2hs-msg">Tap <b>&#8679; Share</b> &rarr; <b>Add to Home Screen</b> for full-screen mode</span>
  <button id="a2hs-dismiss">&#x2715;</button>
</div>

<!-- HUD -->
<div class="hud">
  <div class="hud-r">
    <div class="dot" id="dot"></div>
    <span class="lnk" id="lnklbl">CONNECTING</span>
    <div class="sp"></div>
    <span class="chip" id="c-rtt">RTT &mdash;</span>
    <span class="chip" id="c-arm">DISARMED</span>
  </div>
  <div class="hud-r">
    <span class="chip" id="c-batt">BATT &mdash;</span>
    <span class="chip" id="c-cap">CAP &mdash;</span>
    <span class="chip" id="c-dm">DM &mdash;</span>
    <span class="chip" id="c-rc">RC &mdash;</span>
  </div>
</div>

<!-- Speed segmented control -->
<div class="spd-seg" id="spd-seg">
  <button class="seg on" data-l="slow">SLOW</button>
  <button class="seg"    data-l="normal">NORMAL</button>
  <button class="seg"    data-l="fast">FAST</button>
</div>

<!-- E-STOP -->
<div class="estop-wrap">
  <button class="estop" id="estop">E &mdash; STOP</button>
</div>

<!-- ARM -->
<div class="arm-row">
  <div class="arm-outer">
    <button class="arm-btn" id="arm-btn">HOLD TO ARM</button>
    <svg class="arm-ring" viewBox="0 0 32 32" aria-hidden="true">
      <circle class="r-bg" cx="16" cy="16" r="15"/>
      <circle class="r-fg" cx="16" cy="16" r="15" id="ring" transform="rotate(-90 16 16)"/>
    </svg>
  </div>
  <label class="rc-lbl">
    <input type="checkbox" id="rc-in-hand">
    <span>RC in hand</span>
  </label>
</div>

<!-- Joysticks — bottom half, thumb-zone -->
<div class="sticks">
  <div class="stick-col">
    <div class="pad" id="padL">
      <div class="pfill" id="fillL"></div>
      <div class="pknob" id="knobL"></div>
    </div>
    <div class="slbl">L TRACK</div>
  </div>
  <div class="stick-col">
    <div class="pad" id="padR">
      <div class="pfill" id="fillR"></div>
      <div class="pknob" id="knobR"></div>
    </div>
    <div class="slbl">R TRACK</div>
  </div>
</div>

<script>
'use strict';

/* ---- config ---- */
const QS       = new URLSearchParams(location.search);
const TOKEN    = QS.get('token');
const ARM_MS   = 520;     // >= 500 ms server minimum
const RING_C   = 94.25;   // 2*pi*15 — ring circumference for 32x32 SVG, r=15

/* ---- state ---- */
let ws = null, connected = false;
let rtt = null, lastSt = {}, seq = 1;
let leftV = 0, rightV = 0;

/* ---- WebSocket ---- */
function wsUrl() {
  var p = location.protocol === 'https:' ? 'wss' : 'ws';
  var u = p + '://' + location.host + '/ws/drive';
  return TOKEN ? u + '?token=' + encodeURIComponent(TOKEN) : u;
}
function connect() {
  ws = new WebSocket(wsUrl());
  ws.onopen  = function() { connected = true;  renderConn(); hideOverlay(); };
  ws.onclose = function() {
    connected = false; renderConn();
    showOverlay('disconnected');
    setTimeout(connect, 800);
  };
  ws.onerror = function() { try { ws.close(); } catch(_) {} };
  ws.onmessage = function(ev) {
    var m; try { m = JSON.parse(ev.data); } catch(_) { return; }
    if (m.type !== 'status') return;
    var wasArmed = lastSt.armed;
    lastSt = m;
    if (!m.armed) armPending = false;
    if (m.echo_ts != null) rtt = Math.max(0, Math.round(performance.now() - m.echo_ts));
    if (wasArmed && !m.armed && m.tripped_reason === 'deadman') showOverlay('deadman');
    renderAll();
  };
}
function send(o) {
  if (ws && ws.readyState === 1) { try { ws.send(JSON.stringify(o)); } catch(_) {} }
}

/* ---- drive loops — identical timing to tranche-1 minimal UI ---- */
/* heartbeat 10 Hz */
setInterval(function() { send({type:'hb', t: performance.now()}); }, 100);
/* drive 15 Hz, only when session is armed */
setInterval(function() {
  if (lastSt.armed) send({type:'drive', seq: seq++, t: performance.now(), left: leftV, right: rightV});
}, 66);

/* ---- client-side safety belt: zero sticks immediately on page hide ---- */
document.addEventListener('visibilitychange', function() {
  if (document.hidden) {
    leftV = rightV = 0;
    send({type:'drive', seq: seq++, t: performance.now(), left: 0, right: 0});
  }
});
window.addEventListener('pagehide', function() {
  leftV = rightV = 0;
  send({type:'drive', seq: seq++, t: performance.now(), left: 0, right: 0});
});

/* ---- overlay ---- */
function showOverlay(reason) {
  var titles = {disconnected:'DISCONNECTED', deadman:'LINK LOST'};
  var subs   = {
    disconnected: 'Robot stopped · reconnecting…',
    deadman:      'Deadman tripped — robot stopped · re-arm to resume',
  };
  document.getElementById('ov-title').textContent = titles[reason] || 'STOPPED';
  document.getElementById('ov-sub').textContent   = subs[reason]   || 'Robot stopped';
  document.getElementById('overlay').classList.remove('hidden');
}
function hideOverlay() { document.getElementById('overlay').classList.add('hidden'); }
document.getElementById('ov-btn').addEventListener('click', hideOverlay);

/* ---- iOS A2HS banner ---- */
(function() {
  var ios = /iphone|ipad|ipod/i.test(navigator.userAgent);
  if (ios && !window.navigator.standalone && !localStorage.getItem('walle-a2hs-v1')) {
    document.getElementById('a2hs').classList.remove('hidden');
  }
  document.getElementById('a2hs-dismiss').addEventListener('click', function() {
    localStorage.setItem('walle-a2hs-v1', '1');
    document.getElementById('a2hs').classList.add('hidden');
  });
})();

/* ---- speed segmented control ---- */
document.getElementById('spd-seg').addEventListener('click', function(e) {
  var b = e.target.closest('.seg');
  if (!b) return;
  send({type:'speed', level: b.dataset.l});
  document.querySelectorAll('.seg').forEach(function(x) { x.classList.toggle('on', x === b); });
});

/* ---- E-STOP ---- */
var estopEl    = document.getElementById('estop');
var dimStep    = 0;
var dimTmr     = null;

estopEl.addEventListener('pointerdown', function() {
  if (lastSt.estop_latched) return;   /* latched state handled by click */
  leftV = rightV = 0;
  send({type:'estop'});
  if (navigator.vibrate) navigator.vibrate(150);
});

estopEl.addEventListener('click', function() {
  if (!lastSt.estop_latched) return;
  if (dimStep === 0) {
    dimStep = 1;
    estopEl.classList.add('dim1');
    estopEl.textContent = 'TAP AGAIN TO CLEAR';
    dimTmr = setTimeout(function() {
      dimStep = 0; estopEl.classList.remove('dim1'); renderEstop();
    }, 3000);
  } else {
    clearTimeout(dimTmr); dimStep = 0;
    estopEl.classList.remove('dim1');
    send({type:'clear_estop'});
  }
});

function renderEstop() {
  if (dimStep) return;
  estopEl.classList.toggle('latched', !!lastSt.estop_latched);
  estopEl.textContent = lastSt.estop_latched ? 'E-STOP LATCHED' : 'E — STOP';
}

/* ---- ARM button with 500 ms hold + radial progress ring ---- */
var armEl      = document.getElementById('arm-btn');
var ring       = document.getElementById('ring');
var isHolding  = false;
var armPending = false;
var justArmed  = false;
var holdTmr    = null;
var holdStart  = 0;
var holdRAF    = null;

function setRing(frac) {
  ring.style.strokeDashoffset = (RING_C * (1 - Math.max(0, Math.min(1, frac)))).toFixed(2);
}
function animRing() {
  setRing((performance.now() - holdStart) / ARM_MS);
  holdRAF = requestAnimationFrame(animRing);
}
function cancelHold() {
  isHolding = false;
  if (holdTmr) { clearTimeout(holdTmr); holdTmr = null; }
  if (holdRAF) { cancelAnimationFrame(holdRAF); holdRAF = null; }
  setRing(0);
  armEl.classList.remove('arming');
  renderArm();
}

armEl.addEventListener('pointerdown', function(e) {
  e.preventDefault();
  if (lastSt.armed || lastSt.estop_latched || isHolding) return;
  isHolding = true;
  holdStart = performance.now();
  armEl.classList.add('arming');
  armEl.textContent = 'HOLD…';
  setRing(0);
  holdRAF = requestAnimationFrame(animRing);
  holdTmr = setTimeout(function() {
    holdTmr  = null;
    isHolding = false;
    justArmed = true;
    armPending = true;
    send({type:'arm',
          hold_ms: Math.round(performance.now() - holdStart),
          rc_in_hand: document.getElementById('rc-in-hand').checked});
    if (navigator.vibrate) navigator.vibrate([25, 15, 25]);
    if (holdRAF) { cancelAnimationFrame(holdRAF); holdRAF = null; }
    setRing(1);
    armEl.classList.remove('arming');
    armEl.textContent = 'ARMING…';
  }, ARM_MS);
});

armEl.addEventListener('pointerup',     cancelHold);
armEl.addEventListener('pointerleave',  cancelHold);
armEl.addEventListener('pointercancel', cancelHold);

armEl.addEventListener('click', function() {
  if (justArmed) { justArmed = false; return; }  /* swallow click after successful hold */
  if (lastSt.armed) send({type:'disarm'});
});

function renderArm() {
  if (isHolding) return;
  armEl.classList.toggle('armed', !!lastSt.armed);
  armEl.classList.remove('arming');
  if (lastSt.armed) {
    armPending = false;
    armEl.textContent = 'ARMED · TAP TO DISARM';
    armEl.disabled = false;
  } else if (armPending) {
    armEl.textContent = 'ARMING…';
    armEl.disabled = false;
  } else if (lastSt.estop_latched) {
    armEl.textContent = 'CLEAR E-STOP FIRST';
    armEl.disabled = true;
  } else {
    armEl.textContent = 'HOLD TO ARM';
    armEl.disabled = false;
    setRing(0);
  }
}

/* ---- joystick pads (tank drive — vertical axis only) ---- */
function bindPad(padId, fillId, knobId, setter) {
  var pad  = document.getElementById(padId);
  var fill = document.getElementById(fillId);
  var knob = document.getElementById(knobId);
  var ptr  = null;

  function calcV(e) {
    var r = pad.getBoundingClientRect();
    return Math.max(-1, Math.min(1, 1 - 2 * (e.clientY - r.top) / r.height));
  }
  function paint(v) {
    /* knob: 38 % travel from centre in each direction */
    knob.style.top = (50 - v * 38).toFixed(1) + '%';
    /* fill bar */
    var h = Math.abs(v) * 50;
    fill.style.top    = v >= 0 ? (50 - h).toFixed(1) + '%' : '50%';
    fill.style.height = h.toFixed(1) + '%';
    fill.style.background = v >= 0 ? 'var(--blue-lo)' : 'var(--red-lo)';
    pad.classList.toggle('fwd', v >  0.05);
    pad.classList.toggle('rev', v < -0.05);
  }
  function spring() {
    knob.classList.remove('drag');
    knob.style.top = '50%';           /* CSS spring transition fires here */
    fill.style.height = '0';
    pad.classList.remove('fwd', 'rev');
  }

  pad.addEventListener('pointerdown', function(e) {
    e.preventDefault();
    pad.setPointerCapture(e.pointerId);
    ptr = e.pointerId;
    knob.classList.add('drag');
    var v = calcV(e); setter(v); paint(v);
  });
  pad.addEventListener('pointermove', function(e) {
    if (ptr !== e.pointerId) return;
    var v = calcV(e); setter(v); paint(v);
  });
  function end(e) {
    if (ptr !== e.pointerId) return;
    ptr = null; setter(0); spring();
  }
  pad.addEventListener('pointerup',     end);
  pad.addEventListener('pointercancel', end);
  pad.addEventListener('pointerleave',  end);
}

bindPad('padL', 'fillL', 'knobL', function(v) { leftV  = v; });
bindPad('padR', 'fillR', 'knobR', function(v) { rightV = v; });

/* ---- HUD render ---- */
function renderConn() {
  var d = document.getElementById('dot');
  var l = document.getElementById('lnklbl');
  d.className = 'dot ' + (connected ? 'live' : 'dead');
  l.className = 'lnk ' + (connected ? 'live' : 'dead');
  l.textContent = connected ? 'LINK UP' : 'LINK DOWN';
}

function renderAll() {
  var s = lastSt;

  /* RTT */
  var rttEl = document.getElementById('c-rtt');
  rttEl.textContent = rtt != null ? 'RTT ' + rtt + ' ms' : 'RTT —';
  rttEl.className   = 'chip' + (rtt != null && rtt > 120 ? ' warn' : '');

  /* arm badge */
  var ac = document.getElementById('c-arm');
  if (s.armed) {
    ac.textContent = 'ARMED'; ac.className = 'chip ok';
  } else if (s.tripped_reason) {
    ac.textContent = s.tripped_reason.replace(/_/g,' ').toUpperCase();
    ac.className = 'chip bad';
  } else {
    ac.textContent = 'DISARMED'; ac.className = 'chip';
  }

  /* battery */
  var bc = document.getElementById('c-batt');
  if (s.battery_v != null) {
    bc.textContent = s.battery_v.toFixed(1) + ' V';
    bc.className   = 'chip' + (s.battery_v < 21.0 ? ' warn' : '');
  } else {
    bc.textContent = 'BATT —'; bc.className = 'chip';
  }

  /* speed cap */
  document.getElementById('c-cap').textContent =
    s.speed_cap != null ? 'CAP ' + Math.round(s.speed_cap * 100) + '%' : 'CAP —';

  /* deadman age */
  var dc = document.getElementById('c-dm');
  if (s.armed && s.heartbeat_age_ms != null) {
    var pct = s.heartbeat_age_ms / (s.deadman_ms || 250);
    dc.textContent = 'HB ' + s.heartbeat_age_ms + ' ms';
    dc.className   = 'chip' + (pct > 0.7 ? ' warn' : '');
  } else {
    dc.textContent = 'DM —'; dc.className = 'chip';
  }

  /* RC state */
  var rc = document.getElementById('c-rc');
  if (s.rc_state) {
    var ra = s.rc_state[0], re = s.rc_state[1];
    rc.textContent = 'RC ' + (re ? 'ESTOP' : ra ? 'ARMED' : 'DISARMD');
    rc.className   = 'chip' + (re ? ' bad' : ra ? ' ok' : '');
  } else {
    rc.textContent = 'RC —'; rc.className = 'chip';
  }

  /* sync speed seg to server-reported level */
  if (s.speed_level) {
    document.querySelectorAll('.seg').forEach(function(b) {
      b.classList.toggle('on', b.dataset.l === s.speed_level);
    });
  }

  renderArm();
  renderEstop();
}

/* ---- start ---- */
connect();
</script>
</body>
</html>"""

