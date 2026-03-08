"""
Calibration wizard web UI — Flask Blueprint.

Provides guided, interactive calibration routines accessible from the
dashboard at /calibrate.  Each tool is a multi-step wizard with live
feedback, "I'm ready" buttons, and an optional "Apply to config" action.

Tools included:
  1. Heading PID Tune    — relay feedback Ziegler–Nichols PID
  2. Follow Me Steer Tune — relay feedback Z-N PD for lateral steering
  3. Straight Bias Tune  — per-throttle yaw correction
"""

from __future__ import annotations

import json
import math
import queue
import re
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from flask import Blueprint, Response, request
except ImportError:
    Blueprint = None  # type: ignore[assignment,misc]

import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT

_PROJECT_ROOT = Path(__file__).resolve().parents[2]

# ───────────────────────────── CalibrationManager ─────────────────────────────

class CalibrationManager:
    """Coordinates calibration routines with hardware and the control loop."""

    STATES = ("IDLE", "WAITING_USER", "RUNNING", "COMPLETE", "FAILED")

    def __init__(self, controller=None, oak_reader=None, motor_driver=None,
                 imu_reader=None) -> None:
        self._controller = controller
        self._oak = oak_reader
        self._motor = motor_driver
        self._imu = imu_reader
        self._state = "IDLE"
        self._tool: Optional[str] = None
        self._progress: queue.Queue = queue.Queue(maxsize=200)
        self._cancel_event = threading.Event()
        self._step_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._result: Optional[dict] = None
        self._lock = threading.Lock()

    @property
    def state(self) -> str:
        return self._state

    @property
    def tool(self) -> Optional[str]:
        return self._tool

    @property
    def result(self) -> Optional[dict]:
        return self._result

    def _emit(self, kind: str, **kw) -> None:
        msg = {"kind": kind, "ts": time.time(), **kw}
        try:
            self._progress.put_nowait(msg)
        except queue.Full:
            pass

    def drain_progress(self):
        items = []
        while True:
            try:
                items.append(self._progress.get_nowait())
            except queue.Empty:
                break
        return items

    # ── Lifecycle ──

    def start(self, tool: str, params: dict | None = None) -> bool:
        with self._lock:
            if self._state not in ("IDLE", "COMPLETE", "FAILED"):
                return False
            self._state = "RUNNING"
            self._tool = tool
            self._result = None
            self._cancel_event.clear()
            self._step_event.clear()
            while not self._progress.empty():
                try:
                    self._progress.get_nowait()
                except queue.Empty:
                    break
        if self._controller:
            self._controller.enter_calibration_mode()
        self._thread = threading.Thread(
            target=self._run, args=(tool, params or {}),
            name="CalibrationWorker", daemon=True,
        )
        self._thread.start()
        return True

    def step_done(self) -> None:
        self._step_event.set()

    def cancel(self) -> None:
        self._cancel_event.set()
        self._step_event.set()

    def _finish(self, result: dict | None, failed: bool = False) -> None:
        if self._motor:
            try:
                self._motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
            except Exception:
                pass
        if self._controller:
            self._controller.exit_calibration_mode()
        self._result = result
        self._state = "FAILED" if failed else "COMPLETE"
        self._emit("done", success=not failed, result=result)

    def _wait_for_user(self, instruction: str, timeout_s: float = 120.0) -> bool:
        self._state = "WAITING_USER"
        self._step_event.clear()
        self._emit("wait_user", instruction=instruction)
        deadline = time.monotonic() + timeout_s
        while not self._step_event.is_set():
            if self._cancel_event.is_set():
                return False
            if time.monotonic() > deadline:
                self._emit("log", msg="Timed out waiting for user.")
                return False
            time.sleep(0.1)
        self._state = "RUNNING"
        return True

    def _cancelled(self) -> bool:
        return self._cancel_event.is_set()

    # ── Dispatch ──

    def _run(self, tool: str, params: dict) -> None:
        try:
            handler = {
                "heading_pid": self._tool_heading_pid,
                "follow_me_steer": self._tool_follow_me_steer,
                "straight_bias": self._tool_straight_bias,
            }.get(tool)
            if handler is None:
                self._emit("log", msg=f"Unknown tool: {tool}")
                self._finish(None, failed=True)
                return
            handler(params)
        except Exception as exc:
            self._emit("log", msg=f"Error: {exc}")
            self._finish(None, failed=True)

    # ── Tool: Heading PID Auto-Tune ──

    def _tool_heading_pid(self, params: dict) -> None:
        imu = self._imu
        motor = self._motor
        if imu is None or motor is None:
            self._emit("log", msg="IMU or motor driver not available.")
            self._finish(None, failed=True)
            return

        relay_amp = int(params.get("relay_amplitude", 60))
        duration = float(params.get("duration", 10.0))
        base_duty = int(params.get("base_duty", 140))
        deadband_deg = float(params.get("deadband_deg", 2.0))

        ok = self._wait_for_user(
            "Clear the area around the rover — it will spin in place. "
            "ARM the rover via RC, then tap 'I'm Ready'."
        )
        if not ok:
            self._finish(None, failed=True)
            return

        self._emit("log", msg=f"Running relay test ({duration:.0f}s, amp=±{relay_amp})…")

        # Collect data
        data_imu = imu.read()
        target_heading = float(data_imu["heading_deg"])
        motor.set_tracks(base_duty, base_duty)
        time.sleep(0.3)

        direction = 1
        start = time.monotonic()
        sample_period = 1.0 / 60.0
        next_tick = start
        times, errors = [], []

        try:
            while time.monotonic() < start + duration:
                if self._cancelled():
                    self._finish(None, failed=True)
                    return
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(max(0, next_tick - now))
                next_tick += sample_period

                d = imu.read()
                heading = float(d["heading_deg"])
                error = ((target_heading - heading) + 180) % 360 - 180
                t_rel = now - start

                if abs(error) > deadband_deg and (error > 0) != (direction > 0):
                    direction = -direction

                l = base_duty + direction * relay_amp
                r = base_duty - direction * relay_amp
                motor.set_tracks(max(0, min(255, l)), max(0, min(255, r)))

                times.append(t_rel)
                errors.append(error)

                if len(times) % 60 == 0:
                    self._emit("log", msg=f"  t={t_rel:.1f}s  error={error:+.1f}°  dir={'R' if direction > 0 else 'L'}")
        finally:
            motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)

        # Analyze
        times_np = np.array(times)
        errors_np = np.array(errors)
        mask = times_np >= 1.5
        if not np.any(mask):
            mask = slice(None)
        times_np = times_np[mask]
        errors_np = errors_np[mask]

        signs = np.sign(errors_np)
        signs[signs == 0] = 1
        crossings = np.where(np.diff(signs) != 0)[0]
        if crossings.size < 3:
            self._emit("log", msg="Insufficient oscillations.")
            self._finish(None, failed=True)
            return

        Tu = float(np.mean(np.diff(times_np[crossings])) * 2.0)
        seg_amps = []
        for i in range(1, len(crossings)):
            seg = errors_np[crossings[i-1]:crossings[i]+1]
            if seg.size >= 2:
                a = (float(np.max(seg)) - float(np.min(seg))) / 2.0
                if a > 0.1:
                    seg_amps.append(a)
        if not seg_amps:
            self._emit("log", msg="Could not measure amplitude.")
            self._finish(None, failed=True)
            return
        osc_amp = float(np.median(seg_amps))

        d_eff = 2.0 * float(relay_amp)
        Ku = 4.0 * d_eff / (math.pi * max(osc_amp, 0.1))
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / max(Tu, 1e-3)
        Kd = 0.075 * Ku * Tu

        self._emit("log", msg=f"Tu={Tu:.3f}s  amplitude={osc_amp:.2f}°  Ku={Ku:.2f}")
        self._emit("log", msg=f"Tuned PID: Kp={Kp:.3f}  Ki={Ki:.3f}  Kd={Kd:.4f}")

        self._finish({
            "tool": "heading_pid",
            "Kp": round(Kp, 4), "Ki": round(Ki, 4), "Kd": round(Kd, 5),
            "Tu": round(Tu, 4), "amplitude_deg": round(osc_amp, 3),
        })

    # ── Tool: Follow Me Steering Auto-Tune ──

    def _tool_follow_me_steer(self, params: dict) -> None:
        oak = self._oak
        motor = self._motor
        if oak is None or motor is None:
            self._emit("log", msg="OAK-D or motor driver not available.")
            self._finish(None, failed=True)
            return

        relay_amp = int(params.get("relay_amplitude", 30))
        duration = float(params.get("duration", 8.0))
        deadband_m = float(params.get("deadband_m", 0.05))
        fm_cfg = config.follow_me

        ok = self._wait_for_user(
            "Stand 2–3 meters in front of the rover and stay still. "
            "The rover will swing left and right for about "
            f"{duration:.0f} seconds."
        )
        if not ok:
            self._finish(None, failed=True)
            return

        # Verify person is visible
        self._emit("log", msg="Looking for person…")
        found = False
        for _ in range(100):
            persons = oak.get_person_detections()
            for p in persons:
                if (p.confidence >= fm_cfg.detection_confidence
                        and fm_cfg.min_distance_m <= p.z_m <= fm_cfg.max_distance_m):
                    self._emit("log", msg=f"Person at z={p.z_m:.2f}m, x={p.x_m:.2f}m")
                    found = True
                    break
            if found:
                break
            time.sleep(0.1)

        if not found:
            self._emit("log", msg="No person detected. Make sure you're visible to the camera.")
            self._finish(None, failed=True)
            return

        self._emit("log", msg=f"Running relay test ({duration:.0f}s, amp=±{relay_amp})…")

        direction = 1
        start = time.monotonic()
        sample_period = 1.0 / 30.0
        next_tick = start
        times, x_vals = [], []
        missed = 0

        def _get_x_m():
            persons = oak.get_person_detections()
            best, best_score = None, -1.0
            for p in persons:
                if p.confidence < fm_cfg.detection_confidence:
                    continue
                if p.z_m < fm_cfg.min_distance_m or p.z_m > fm_cfg.max_distance_m:
                    continue
                cx = (p.bbox[0] + p.bbox[2]) / 2.0
                score = 1.0 - abs(cx - 0.5) * 2.0
                if score > best_score:
                    best_score = score
                    best = p
            return best.x_m if best else None

        try:
            while time.monotonic() < start + duration:
                if self._cancelled():
                    self._finish(None, failed=True)
                    return
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(max(0, next_tick - now))
                next_tick += sample_period

                x_m = _get_x_m()
                t_rel = time.monotonic() - start

                if x_m is None:
                    missed += 1
                    if missed > 30:
                        self._emit("log", msg="Lost person for >1s. Aborting.")
                        self._finish(None, failed=True)
                        return
                    continue
                missed = 0

                if abs(x_m) > deadband_m and (x_m > 0) != (direction > 0):
                    direction = -direction

                l = CENTER_OUTPUT_VALUE + direction * relay_amp
                r = CENTER_OUTPUT_VALUE - direction * relay_amp
                motor.set_tracks(max(0, min(255, l)), max(0, min(255, r)))

                times.append(t_rel)
                x_vals.append(x_m)

                if len(times) % 30 == 0:
                    self._emit("log", msg=f"  t={t_rel:.1f}s  x={x_m:+.2f}m  dir={'R' if direction > 0 else 'L'}")
        finally:
            motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)

        # Analyze
        if len(times) < 10:
            self._emit("log", msg="Not enough samples.")
            self._finish(None, failed=True)
            return

        times_np = np.array(times)
        x_np = np.array(x_vals)
        mask = times_np >= 1.5
        if not np.any(mask):
            mask = slice(None)
        times_np = times_np[mask]
        x_np = x_np[mask]

        signs = np.sign(x_np)
        signs[signs == 0] = 1
        crossings = np.where(np.diff(signs) != 0)[0]
        if crossings.size < 3:
            self._emit("log", msg="Insufficient oscillations. Try larger relay amplitude.")
            self._finish(None, failed=True)
            return

        Tu = float(np.mean(np.diff(times_np[crossings])) * 2.0)
        seg_amps = []
        for i in range(1, len(crossings)):
            seg = x_np[crossings[i-1]:crossings[i]+1]
            if seg.size >= 2:
                a = (float(np.max(seg)) - float(np.min(seg))) / 2.0
                if a > 0.01:
                    seg_amps.append(a)
        if not seg_amps:
            self._emit("log", msg="Could not measure oscillation amplitude.")
            self._finish(None, failed=True)
            return
        osc_amp = float(np.median(seg_amps))

        scale = fm_cfg.max_follow_speed_byte / max(fm_cfg.max_distance_m, 0.1)
        Ku_bytes = 4.0 * float(relay_amp) / (math.pi * max(osc_amp, 0.01))
        Ku = Ku_bytes / scale
        Kp = 0.8 * Ku
        Kd = 0.10 * Ku * Tu

        self._emit("log", msg=f"Tu={Tu:.3f}s  amplitude={osc_amp:.3f}m  Ku={Ku:.3f}")
        self._emit("log", msg=f"Tuned PD: steering_gain={Kp:.3f}  steering_derivative_gain={Kd:.3f}")

        self._finish({
            "tool": "follow_me_steer",
            "steering_gain": round(Kp, 4),
            "steering_derivative_gain": round(Kd, 4),
            "Tu": round(Tu, 4), "amplitude_m": round(osc_amp, 4),
        })

    # ── Tool: Straight Bias Auto-Tune ──

    def _tool_straight_bias(self, params: dict) -> None:
        imu = self._imu
        motor = self._motor
        if imu is None or motor is None:
            self._emit("log", msg="IMU or motor driver not available.")
            self._finish(None, failed=True)
            return

        bins = params.get("bins", [20, 40, 60])
        seg_s = float(params.get("seg_seconds", 3.0))
        test_delta = int(params.get("test_delta", 8))
        max_bias = int(params.get("max_bias", 15))

        ok = self._wait_for_user(
            "Clear the area — the rover will drive forward at several speeds. "
            "ARM via RC, then tap 'I'm Ready'."
        )
        if not ok:
            self._finish(None, failed=True)
            return

        results = []
        for off in bins:
            if self._cancelled():
                self._finish(None, failed=True)
                return
            self._emit("log", msg=f"Testing throttle offset +{off}…")
            base = CENTER_OUTPUT_VALUE + off
            base = max(0, min(254, base))

            # Segment 1: equal tracks
            motor.set_tracks(base, base)
            yaw1 = self._collect_yaw(seg_s)

            # Segment 2: test delta
            l2 = max(0, min(254, base + test_delta))
            r2 = max(0, min(254, base - test_delta))
            motor.set_tracks(l2, r2)
            yaw2 = self._collect_yaw(seg_s)

            motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
            time.sleep(0.3)

            d_eff = 2.0 * float(test_delta)
            slope = (yaw2 - yaw1) / d_eff if d_eff != 0 else 0.0
            delta = -yaw1 / (2.0 * slope) if abs(slope) > 1e-6 else 0.0
            delta_c = int(max(-max_bias, min(max_bias, round(delta))))
            results.append(delta_c)

            self._emit("log", msg=f"  yaw_equal={yaw1:.2f}  yaw_test={yaw2:.2f}  suggested={delta_c:+d}")

        motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)

        if not results:
            self._finish(None, failed=True)
            return
        agg = sorted(results)[len(results) // 2]
        bias_left, bias_right = agg, -agg

        self._emit("log", msg=f"Aggregate bias: left={bias_left:+d}  right={bias_right:+d}")

        self._finish({
            "tool": "straight_bias",
            "straight_bias_left_byte": bias_left,
            "straight_bias_right_byte": bias_right,
        })

    def _collect_yaw(self, duration_s: float) -> float:
        samples = []
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            if self._cancelled():
                return 0.0
            d = self._imu.read()
            samples.append(float(d["gz_dps"]))
            time.sleep(0.02)
        return sum(samples) / len(samples) if samples else 0.0

    # ── Config apply ──

    def apply_result(self) -> dict:
        """Write last result values into config.py. Returns {ok, msg}."""
        if self._result is None:
            return {"ok": False, "msg": "No result to apply."}

        cfg_path = _PROJECT_ROOT / "config.py"
        try:
            text = cfg_path.read_text(encoding="utf-8")
        except Exception as e:
            return {"ok": False, "msg": f"Cannot read config.py: {e}"}

        tool = self._result.get("tool")
        new_text = text

        if tool == "heading_pid":
            new_text = _re_replace(new_text, r"kp:\s*float\s*=\s*", self._result["Kp"])
            new_text = _re_replace(new_text, r"ki:\s*float\s*=\s*", self._result["Ki"])
            new_text = _re_replace(new_text, r"kd:\s*float\s*=\s*", self._result["Kd"])

        elif tool == "follow_me_steer":
            new_text = _re_replace(new_text, r"steering_gain:\s*float\s*=\s*",
                                   self._result["steering_gain"])
            new_text = _re_replace(new_text, r"steering_derivative_gain:\s*float\s*=\s*",
                                   self._result["steering_derivative_gain"])

        elif tool == "straight_bias":
            new_text = _re_replace(new_text, r"straight_bias_left_byte:\s*int\s*=\s*",
                                   self._result["straight_bias_left_byte"], is_int=True)
            new_text = _re_replace(new_text, r"straight_bias_right_byte:\s*int\s*=\s*",
                                   self._result["straight_bias_right_byte"], is_int=True)

        else:
            return {"ok": False, "msg": f"Unknown tool: {tool}"}

        if new_text == text:
            return {"ok": False, "msg": "No matching fields found in config.py."}

        try:
            cfg_path.write_text(new_text, encoding="utf-8")
        except Exception as e:
            return {"ok": False, "msg": f"Write failed: {e}"}

        return {"ok": True, "msg": "Config updated. Restart the service to use new values."}


def _re_replace(text: str, prefix_pattern: str, value, is_int: bool = False) -> str:
    if is_int:
        val_str = str(int(value))
        return re.sub(f"({prefix_pattern})-?[0-9]+", rf"\g<1>{val_str}", text, count=1)
    val_str = f"{float(value):.4f}".rstrip("0").rstrip(".")
    return re.sub(f"({prefix_pattern})[0-9.]+", rf"\g<1>{val_str}", text, count=1)


# ───────────────────────────── Flask Blueprint ────────────────────────────────

_TOOL_DEFS = [
    {
        "id": "heading_pid", "name": "Heading PID Auto-Tune",
        "category": "IMU", "duration": "~10s",
        "desc": "Relay feedback test to find optimal Kp, Ki, Kd for IMU heading control.",
        "steps": [
            "Clear the area — rover will spin in place",
        ],
    },
    {
        "id": "follow_me_steer", "name": "Follow Me Steering Tune",
        "category": "Steering", "duration": "~8s",
        "desc": "Relay feedback test to find optimal lateral steering gains for Follow Me.",
        "steps": [
            "Stand 2–3 m in front of the rover and stay still",
            "Rover swings left/right — keep still",
        ],
    },
    {
        "id": "straight_bias", "name": "Straight Bias Tune",
        "category": "Steering", "duration": "~18s",
        "desc": "Measure and correct left/right motor bias so the rover drives straight.",
        "steps": [
            "Clear the area — rover will drive forward at several speeds",
        ],
    },
]


def create_calibration_blueprint(cal_manager: CalibrationManager) -> Blueprint:
    bp = Blueprint("calibration", __name__)

    @bp.route("/calibrate")
    def cal_index():
        return Response(_INDEX_HTML, content_type="text/html")

    @bp.route("/calibrate/<tool_id>")
    def cal_wizard(tool_id: str):
        tool = next((t for t in _TOOL_DEFS if t["id"] == tool_id), None)
        if tool is None:
            return Response("Unknown tool", status=404)
        html = _WIZARD_HTML.replace("{{TOOL_JSON}}", json.dumps(tool))
        return Response(html, content_type="text/html")

    @bp.route("/api/cal/start", methods=["POST"])
    def cal_start():
        body = request.get_json(force=True, silent=True) or {}
        tool = body.get("tool", "")
        params = body.get("params", {})
        ok = cal_manager.start(tool, params)
        if ok:
            return _json_resp({"status": "started", "tool": tool})
        return _json_resp({"error": "Calibration already running"}, 409)

    @bp.route("/api/cal/step_done", methods=["POST"])
    def cal_step_done():
        cal_manager.step_done()
        return _json_resp({"status": "ok"})

    @bp.route("/api/cal/cancel", methods=["POST"])
    def cal_cancel():
        cal_manager.cancel()
        return _json_resp({"status": "cancelled"})

    @bp.route("/api/cal/apply", methods=["POST"])
    def cal_apply():
        result = cal_manager.apply_result()
        return _json_resp(result)

    @bp.route("/api/cal/status")
    def cal_status():
        return _json_resp({
            "state": cal_manager.state,
            "tool": cal_manager.tool,
            "result": cal_manager.result,
        })

    @bp.route("/api/cal/progress")
    def cal_progress_sse():
        def gen():
            while True:
                items = cal_manager.drain_progress()
                for item in items:
                    yield f"data: {json.dumps(item)}\n\n"
                st = cal_manager.state
                if st in ("COMPLETE", "FAILED", "IDLE") and not items:
                    yield f"data: {json.dumps({'kind': 'state', 'state': st})}\n\n"
                    return
                time.sleep(0.25)
        return Response(gen(), mimetype="text/event-stream")

    return bp


def _json_resp(obj, status=200):
    return Response(json.dumps(obj), status=status, content_type="application/json")


# ───────────────────────────── HTML Templates ─────────────────────────────────

_INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WALL-E Mini — Calibration Wizard</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
         background: #0f1117; color: #e0e0e0; }
  header { background: #1a1d27; padding: 12px 24px; display: flex;
           align-items: center; justify-content: space-between; border-bottom: 1px solid #2a2d37; }
  header h1 { font-size: 18px; font-weight: 600; color: #fff; }
  header a { color: #7aa2f7; text-decoration: none; font-size: 14px; }
  header a:hover { text-decoration: underline; }
  .container { max-width: 900px; margin: 0 auto; padding: 24px 16px; }
  h2 { font-size: 15px; color: #888; text-transform: uppercase; letter-spacing: 1px;
       margin: 24px 0 12px; }
  h2:first-child { margin-top: 0; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  .card { background: #1a1d27; border-radius: 10px; padding: 20px;
          cursor: pointer; transition: transform .15s, box-shadow .15s;
          border: 1px solid #2a2d37; }
  .card:hover { transform: translateY(-2px); box-shadow: 0 4px 20px rgba(0,0,0,.4); }
  .card h3 { font-size: 16px; color: #fff; margin-bottom: 6px; }
  .card p { font-size: 13px; color: #999; line-height: 1.5; }
  .card .meta { font-size: 12px; color: #666; margin-top: 10px; }
  @media (max-width: 600px) { .grid { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>Calibration Wizard</h1>
  <a href="/">← Dashboard</a>
</header>
<div class="container">
  <h2>IMU</h2>
  <div class="grid">
    <div class="card" onclick="location.href='/calibrate/heading_pid'">
      <h3>Heading PID Tune</h3>
      <p>Relay test to auto-tune Kp, Ki, Kd for heading control.</p>
      <div class="meta">~10 seconds</div>
    </div>
  </div>
  <h2>Steering</h2>
  <div class="grid">
    <div class="card" onclick="location.href='/calibrate/follow_me_steer'">
      <h3>Follow Me Steering</h3>
      <p>Auto-tune lateral steering gains for person following.</p>
      <div class="meta">~8 seconds</div>
    </div>
    <div class="card" onclick="location.href='/calibrate/straight_bias'">
      <h3>Straight Bias</h3>
      <p>Correct left/right motor bias so the rover drives straight.</p>
      <div class="meta">~18 seconds</div>
    </div>
  </div>
</div>
</body>
</html>"""


_WIZARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WALL-E Mini — Calibration</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
         background: #0f1117; color: #e0e0e0; }
  header { background: #1a1d27; padding: 12px 24px; display: flex;
           align-items: center; justify-content: space-between; border-bottom: 1px solid #2a2d37; }
  header h1 { font-size: 18px; font-weight: 600; color: #fff; }
  header a { color: #7aa2f7; text-decoration: none; font-size: 14px; }
  .container { max-width: 700px; margin: 0 auto; padding: 24px 16px; }
  .status-bar { background: #1a1d27; border-radius: 10px; padding: 16px 20px;
                margin-bottom: 16px; border: 1px solid #2a2d37; }
  .status-bar .state { font-size: 14px; font-weight: 600; }
  .state-IDLE { color: #888; } .state-WAITING_USER { color: #e6a239; }
  .state-RUNNING { color: #7aa2f7; } .state-COMPLETE { color: #4ec9b0; }
  .state-FAILED { color: #e63946; }
  .instruction { background: #1e2030; border-left: 3px solid #e6a239; padding: 16px 20px;
                 border-radius: 0 10px 10px 0; margin-bottom: 16px; font-size: 15px;
                 line-height: 1.6; display: none; }
  .btn-group { display: flex; gap: 10px; margin-bottom: 16px; }
  .btn { padding: 10px 24px; border-radius: 8px; border: none; font-size: 14px;
         font-weight: 600; cursor: pointer; transition: background .15s; }
  .btn-primary { background: #7aa2f7; color: #111; }
  .btn-primary:hover { background: #9bb8fa; }
  .btn-primary:disabled { background: #3a3d47; color: #666; cursor: default; }
  .btn-danger { background: #e63946; color: #fff; }
  .btn-danger:hover { background: #ff4d5a; }
  .btn-success { background: #4ec9b0; color: #111; }
  .btn-success:hover { background: #6edac4; }
  .btn-secondary { background: #2a2d37; color: #ccc; }
  .btn-secondary:hover { background: #3a3d47; }
  .log-box { background: #0a0c10; border-radius: 10px; padding: 14px 16px;
             font-family: 'SF Mono', 'Fira Code', monospace; font-size: 12px;
             line-height: 1.7; max-height: 300px; overflow-y: auto;
             border: 1px solid #1a1d27; margin-bottom: 16px; }
  .log-box .line { color: #999; }
  .log-box .line.result { color: #4ec9b0; font-weight: 600; }
  .result-box { background: #1a1d27; border-radius: 10px; padding: 16px 20px;
                border: 1px solid #2a2d37; margin-bottom: 16px; display: none; }
  .result-box h3 { font-size: 14px; color: #4ec9b0; margin-bottom: 10px; }
  .result-box table { width: 100%; }
  .result-box td { padding: 4px 8px; font-size: 13px; }
  .result-box td:first-child { color: #888; }
  .result-box td:last-child { color: #fff; font-family: monospace; }
  .camera-feed { margin-bottom: 16px; border-radius: 10px; overflow: hidden;
                 display: none; max-width: 100%; }
  .camera-feed img { width: 100%; display: block; }
</style>
</head>
<body>
<header>
  <h1 id="tool-name">Calibration</h1>
  <a href="/calibrate">← All Tools</a>
</header>
<div class="container">
  <div class="status-bar">
    State: <span id="state" class="state state-IDLE">IDLE</span>
  </div>

  <div class="camera-feed" id="camera-feed">
    <img src="/stream/rgb" alt="Camera">
  </div>

  <div class="instruction" id="instruction"></div>

  <div class="btn-group">
    <button class="btn btn-primary" id="btn-start">Start Calibration</button>
    <button class="btn btn-primary" id="btn-ready" style="display:none">I'm Ready</button>
    <button class="btn btn-danger" id="btn-cancel" style="display:none">Cancel</button>
  </div>

  <div class="log-box" id="log"></div>

  <div class="result-box" id="result-box">
    <h3>Results</h3>
    <table id="result-table"></table>
    <div class="btn-group" style="margin-top:14px">
      <button class="btn btn-success" id="btn-apply">Apply to Config</button>
      <button class="btn btn-secondary" onclick="location.href='/calibrate'">Done</button>
    </div>
    <div id="apply-msg" style="margin-top:8px;font-size:13px;color:#888;"></div>
  </div>
</div>

<script>
const TOOL = {{TOOL_JSON}};
document.getElementById('tool-name').textContent = TOOL.name;

const stateEl = document.getElementById('state');
const instrEl = document.getElementById('instruction');
const logEl = document.getElementById('log');
const resultBox = document.getElementById('result-box');
const resultTable = document.getElementById('result-table');
const btnStart = document.getElementById('btn-start');
const btnReady = document.getElementById('btn-ready');
const btnCancel = document.getElementById('btn-cancel');
const btnApply = document.getElementById('btn-apply');
const applyMsg = document.getElementById('apply-msg');
const cameraFeed = document.getElementById('camera-feed');

if (TOOL.id === 'follow_me_steer') cameraFeed.style.display = 'block';

let evtSource = null;

function addLog(msg, cls) {
  const d = document.createElement('div');
  d.className = 'line' + (cls ? ' ' + cls : '');
  d.textContent = msg;
  logEl.appendChild(d);
  logEl.scrollTop = logEl.scrollHeight;
}

function setState(s) {
  stateEl.textContent = s;
  stateEl.className = 'state state-' + s;
}

function showInstruction(text) {
  instrEl.textContent = text;
  instrEl.style.display = 'block';
  btnReady.style.display = '';
  btnStart.style.display = 'none';
}

function hideInstruction() {
  instrEl.style.display = 'none';
  btnReady.style.display = 'none';
}

function showResults(result) {
  resultBox.style.display = 'block';
  resultTable.innerHTML = '';
  const skip = ['tool'];
  for (const [k, v] of Object.entries(result)) {
    if (skip.includes(k) || v === null) continue;
    const tr = document.createElement('tr');
    const td1 = document.createElement('td');
    td1.textContent = k;
    const td2 = document.createElement('td');
    td2.textContent = typeof v === 'number' ? v.toFixed(4) : String(v);
    tr.appendChild(td1);
    tr.appendChild(td2);
    resultTable.appendChild(tr);
  }
}

function listenProgress() {
  if (evtSource) evtSource.close();
  evtSource = new EventSource('/api/cal/progress');
  evtSource.onmessage = function(e) {
    const msg = JSON.parse(e.data);
    if (msg.kind === 'log') {
      addLog(msg.msg);
    } else if (msg.kind === 'wait_user') {
      setState('WAITING_USER');
      showInstruction(msg.instruction);
    } else if (msg.kind === 'done') {
      hideInstruction();
      btnCancel.style.display = 'none';
      if (msg.success) {
        setState('COMPLETE');
        addLog('Calibration complete!', 'result');
        if (msg.result) showResults(msg.result);
      } else {
        setState('FAILED');
        addLog('Calibration failed.', 'result');
      }
      btnStart.style.display = '';
      btnStart.textContent = 'Run Again';
      evtSource.close();
    } else if (msg.kind === 'state') {
      setState(msg.state);
      if (msg.state === 'COMPLETE' || msg.state === 'FAILED' || msg.state === 'IDLE') {
        evtSource.close();
        btnStart.style.display = '';
        btnCancel.style.display = 'none';
      }
    }
  };
  evtSource.onerror = function() {
    evtSource.close();
  };
}

btnStart.addEventListener('click', function() {
  logEl.innerHTML = '';
  resultBox.style.display = 'none';
  applyMsg.textContent = '';
  setState('RUNNING');
  btnStart.style.display = 'none';
  btnCancel.style.display = '';
  hideInstruction();
  fetch('/api/cal/start', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({tool: TOOL.id, params: {}})
  }).then(r => r.json()).then(d => {
    if (d.error) { addLog('Error: ' + d.error); setState('FAILED'); return; }
    addLog('Started ' + TOOL.name + '...');
    listenProgress();
  });
});

btnReady.addEventListener('click', function() {
  hideInstruction();
  setState('RUNNING');
  fetch('/api/cal/step_done', {method: 'POST'});
});

btnCancel.addEventListener('click', function() {
  fetch('/api/cal/cancel', {method: 'POST'});
  addLog('Cancelling...');
});

btnApply.addEventListener('click', function() {
  btnApply.disabled = true;
  fetch('/api/cal/apply', {method: 'POST'})
    .then(r => r.json())
    .then(d => {
      applyMsg.textContent = d.msg || (d.ok ? 'Applied!' : 'Failed.');
      applyMsg.style.color = d.ok ? '#4ec9b0' : '#e63946';
      btnApply.disabled = false;
    });
});
</script>
</body>
</html>"""
