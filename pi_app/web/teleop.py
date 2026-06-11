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


# Minimal phone-first /drive UI. Tranche 2 will replace this with the "sexy" UI;
# tranche 1 only needs functional, fail-safe controls.
_DRIVE_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>WALL-E Drive</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; -webkit-tap-highlight-color: transparent; }
  body { margin: 0; background: #0d0f14; color: #e6e6e6; font: 14px/1.4 system-ui, sans-serif;
         height: 100vh; display: flex; flex-direction: column; overflow: hidden; }
  #status { padding: 8px 10px; font-size: 12px; background: #14171f; border-bottom: 1px solid #222;
            white-space: pre-wrap; }
  #status b { color: #4a9eff; }
  .bad { color: #e63946; } .ok { color: #38b000; } .warn { color: #e6a239; }
  #bar { display: flex; gap: 8px; padding: 8px 10px; align-items: center; flex-wrap: wrap; }
  button { font: inherit; border: 1px solid #2a2d37; border-radius: 8px; background: #22252f;
           color: #e6e6e6; padding: 10px 12px; }
  #arm { flex: 1; min-width: 120px; }
  #arm.armed { background: #38b000; color: #061; border-color: #38b000; }
  #estop { background: #e63946; border-color: #e63946; color: #fff; font-weight: 700; min-width: 110px; }
  #estop.latched { background: #fff; color: #e63946; }
  .speed.sel { background: #4a9eff; color: #06121f; border-color: #4a9eff; }
  label.chk { font-size: 12px; color: #aaa; display: flex; align-items: center; gap: 4px; }
  #pads { flex: 1; display: flex; gap: 10px; padding: 10px; min-height: 0; }
  .pad { flex: 1; position: relative; background: #161922; border: 1px solid #2a2d37;
         border-radius: 12px; touch-action: none; overflow: hidden; }
  .pad .lbl { position: absolute; top: 6px; left: 0; right: 0; text-align: center; color: #667; font-size: 12px; }
  .pad .fill { position: absolute; left: 0; right: 0; background: #1f6feb55; }
  .pad .mid { position: absolute; left: 0; right: 0; top: 50%; height: 1px; background: #2a2d37; }
  .hint { padding: 4px 10px 10px; color: #556; font-size: 11px; }
</style>
</head>
<body>
<div id="status">connecting…</div>
<div id="bar">
  <button id="arm">HOLD TO ARM</button>
  <button id="estop">E-STOP</button>
  <button class="speed sel" data-l="slow">slow</button>
  <button class="speed" data-l="normal">normal</button>
  <button class="speed" data-l="fast">fast</button>
  <label class="chk"><input type="checkbox" id="rcinhand"> RC in my hand</label>
</div>
<div id="pads">
  <div class="pad" id="padL"><div class="mid"></div><div class="fill"></div><div class="lbl">LEFT TRACK</div></div>
  <div class="pad" id="padR"><div class="mid"></div><div class="fill"></div><div class="lbl">RIGHT TRACK</div></div>
</div>
<div class="hint">Drag up = forward, down = reverse. Release = stop. Loses link → robot stops (server deadman 250&nbsp;ms). RC always overrides.</div>
<script>
const qs = new URLSearchParams(location.search);
const token = qs.get('token');
let ws = null, connected = false;
let seq = 1;
let left = 0, right = 0;
let lastStatus = {};
let rtt = null;

function wsUrl() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  let u = proto + '://' + location.host + '/ws/drive';
  if (token) u += '?token=' + encodeURIComponent(token);
  return u;
}
function connect() {
  ws = new WebSocket(wsUrl());
  ws.onopen = () => { connected = true; render(); };
  ws.onclose = () => { connected = false; render(); setTimeout(connect, 600); };
  ws.onerror = () => { try { ws.close(); } catch(e){} };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch(e) { return; }
    if (m.type === 'status') {
      lastStatus = m;
      if (m.echo_ts != null) rtt = Math.max(0, Math.round(performance.now() - m.echo_ts));
      render();
    }
  };
}
function send(o) { if (ws && ws.readyState === 1) { try { ws.send(JSON.stringify(o)); } catch(e){} } }

// Heartbeat 10 Hz; drive 15 Hz coalesced with the heartbeat timestamp.
setInterval(() => send({type: 'hb', t: performance.now()}), 100);
setInterval(() => {
  if (lastStatus.armed) send({type: 'drive', seq: seq++, t: performance.now(), left, right});
}, 66);

// Arm = press and hold >= 500 ms.
const armBtn = document.getElementById('arm');
let holdStart = 0, holdTimer = null;
function armDown(e){ e.preventDefault(); holdStart = performance.now();
  armBtn.textContent = 'HOLD…';
  holdTimer = setTimeout(() => {
    send({type:'arm', hold_ms: Math.round(performance.now()-holdStart),
          rc_in_hand: document.getElementById('rcinhand').checked});
  }, 520);
}
function armUp(e){ e.preventDefault(); if (holdTimer){clearTimeout(holdTimer); holdTimer=null;}
  if (lastStatus.armed) { /* hold complete-> stays armed */ } else { armBtn.textContent='HOLD TO ARM'; }
}
armBtn.addEventListener('pointerdown', armDown);
armBtn.addEventListener('pointerup', armUp);
armBtn.addEventListener('pointerleave', armUp);
armBtn.addEventListener('click', (e)=>{ if (lastStatus.armed) { send({type:'disarm'}); } });

document.getElementById('estop').addEventListener('click', () => {
  if (lastStatus.estop_latched) { send({type:'clear_estop'}); }
  else { left = right = 0; send({type:'estop'}); }
});
document.querySelectorAll('.speed').forEach(b => b.addEventListener('click', () => {
  send({type:'speed', level: b.dataset.l});
  document.querySelectorAll('.speed').forEach(x=>x.classList.remove('sel'));
  b.classList.add('sel');
}));

// Touch zones: vertical drag -> throttle [-1,1].
function bindPad(padId, setFn) {
  const pad = document.getElementById(padId);
  const fill = pad.querySelector('.fill');
  let active = null;
  function val(ev) {
    const r = pad.getBoundingClientRect();
    let f = 1 - 2 * ((ev.clientY - r.top) / r.height); // top=+1, bottom=-1
    return Math.max(-1, Math.min(1, f));
  }
  function paint(v) {
    const r = pad.getBoundingClientRect();
    const mid = r.height/2, h = Math.abs(v)*mid;
    fill.style.height = h + 'px';
    fill.style.top = v >= 0 ? (mid - h) + 'px' : mid + 'px';
    fill.style.background = v >= 0 ? '#1f6feb66' : '#e6394666';
  }
  pad.addEventListener('pointerdown', e => { e.preventDefault(); active = e.pointerId;
    pad.setPointerCapture(e.pointerId); const v=val(e); setFn(v); paint(v); });
  pad.addEventListener('pointermove', e => { if (active!==e.pointerId) return; const v=val(e); setFn(v); paint(v); });
  function end(e){ if (active!==e.pointerId) return; active=null; setFn(0); paint(0); }
  pad.addEventListener('pointerup', end);
  pad.addEventListener('pointercancel', end);
  pad.addEventListener('pointerleave', end);
}
bindPad('padL', v => left = v);
bindPad('padR', v => right = v);

function render() {
  const s = lastStatus;
  armBtn.classList.toggle('armed', !!s.armed);
  armBtn.textContent = s.armed ? 'ARMED — tap to disarm' : 'HOLD TO ARM';
  const eb = document.getElementById('estop');
  eb.classList.toggle('latched', !!s.estop_latched);
  eb.textContent = s.estop_latched ? 'E-STOP LATCHED — tap to clear' : 'E-STOP';
  const armTxt = s.armed ? '<span class="ok">ARMED</span>' : '<span class="warn">disarmed</span>';
  const dm = s.armed
      ? ((s.heartbeat_age_ms!=null && s.heartbeat_age_ms <= s.deadman_ms) ? '<span class="ok">live</span>' : '<span class="bad">STALE</span>')
      : (s.tripped_reason ? '<span class="bad">'+s.tripped_reason+'</span>' : 'idle');
  const rc = s.rc_state ? ('rc_armed='+s.rc_state[0]+' rc_estop='+s.rc_state[1]) : 'rc=unknown';
  const batt = (s.battery_v!=null) ? s.battery_v.toFixed(1)+'V' : '—';
  document.getElementById('status').innerHTML =
    (connected ? '<b>link up</b>' : '<span class="bad">link down</span>') +
    '  arm: ' + armTxt + '  deadman: ' + dm +
    '  rtt: ' + (rtt!=null ? rtt+'ms' : '—') +
    '  speed: ' + (s.speed_level||'—') + '  batt: ' + batt + '\n' + rc;
}
connect();
</script>
</body>
</html>"""

