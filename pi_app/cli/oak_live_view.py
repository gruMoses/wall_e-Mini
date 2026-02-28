#!/usr/bin/env python3
"""
Dry-run test dashboard for the OAK-D Lite camera.

Runs the full ObstacleAvoidanceController and FollowMeController against live
camera data, displays what the motors *would* do, and walks the user through a
scripted test sequence via on-screen prompts.

Usage:
    python3 -m pi_app.cli.oak_live_view [--port 8080]
"""

import argparse
import json
import logging
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.follow_me import FollowMeController, PersonDetection

logger = logging.getLogger(__name__)

LABELS = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor",
]
PERSON_LABEL = 15
from pi_app.control.mapping import CENTER_OUTPUT_VALUE as NEUTRAL, MAX_OUTPUT, MIN_OUTPUT


def _scale_toward_neutral(byte_val: int, scale: float) -> int:
    result = NEUTRAL + (byte_val - NEUTRAL) * scale
    return max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(result))))


# ---------------------------------------------------------------------------
# Test sequence state machine
# ---------------------------------------------------------------------------

PHASES = [
    {
        "id": "baseline",
        "instruction": "Stand still, away from the camera. Calibrating baseline...",
        "duration": 5,
    },
    {
        "id": "approach",
        "instruction": "Walk SLOWLY toward the camera. Watch the throttle bar drop.",
        "duration": 25,
    },
    {
        "id": "close",
        "instruction": "STOP! Stay close (within 0.5m). Obstacle avoidance should show FULL STOP.",
        "duration": 5,
    },
    {
        "id": "back_up",
        "instruction": "Back up to about 2 meters from the camera.",
        "duration": 15,
    },
    {
        "id": "follow_left",
        "instruction": "Walk to the LEFT side of the camera's view.",
        "duration": 8,
    },
    {
        "id": "follow_right",
        "instruction": "Now walk to the RIGHT side.",
        "duration": 8,
    },
    {
        "id": "follow_close_far",
        "instruction": "Walk TOWARD camera, then BACK AWAY. Watch the motor speed change.",
        "duration": 12,
    },
    {
        "id": "done",
        "instruction": "All tests complete! See results below.",
        "duration": 9999,
    },
]


class TestSequence:
    def __init__(self):
        self._phase_idx = 0
        self._phase_start = time.monotonic()
        self._results: dict[str, dict] = {}
        self._observations: dict[str, list] = {p["id"]: [] for p in PHASES}

    @property
    def phase(self) -> dict:
        return PHASES[self._phase_idx]

    @property
    def results(self) -> dict[str, dict]:
        return dict(self._results)

    def tick(self, depth_m: float | None, scale: float,
             persons: int, person_x: float | None,
             motor_l: int, motor_r: int) -> None:
        pid = self.phase["id"]
        if pid == "done":
            return

        elapsed = time.monotonic() - self._phase_start
        self._observations[pid].append({
            "t": elapsed, "depth": depth_m, "scale": scale,
            "persons": persons, "px": person_x,
            "ml": motor_l, "mr": motor_r,
        })

        advance = False
        if pid == "baseline":
            advance = elapsed >= self.phase["duration"]
            if advance:
                scales = [o["scale"] for o in self._observations[pid]]
                self._results[pid] = {
                    "pass": all(s > 0.9 for s in scales[-5:]),
                    "detail": f"Scale stable at {scales[-1]:.2f}" if scales else "no data",
                }
        elif pid == "approach":
            if depth_m is not None and depth_m < 0.5:
                advance = True
            elif elapsed >= self.phase["duration"]:
                advance = True
            if advance:
                min_scale = min((o["scale"] for o in self._observations[pid]), default=1.0)
                self._results[pid] = {
                    "pass": min_scale < 0.3,
                    "detail": f"Throttle dropped to {min_scale:.2f}",
                }
        elif pid == "close":
            advance = elapsed >= self.phase["duration"]
            if advance:
                scales = [o["scale"] for o in self._observations[pid]]
                avg = sum(scales) / len(scales) if scales else 1.0
                self._results[pid] = {
                    "pass": avg < 0.15,
                    "detail": f"Avg scale while close: {avg:.2f}",
                }
        elif pid == "back_up":
            if depth_m is not None and depth_m > 1.5 and scale > 0.9:
                advance = True
            elif elapsed >= self.phase["duration"]:
                advance = True
            if advance:
                last_scale = self._observations[pid][-1]["scale"] if self._observations[pid] else 0
                self._results[pid] = {
                    "pass": last_scale > 0.8,
                    "detail": f"Scale recovered to {last_scale:.2f}",
                }
        elif pid == "follow_left":
            advance = elapsed >= self.phase["duration"]
            if advance:
                xs = [o["px"] for o in self._observations[pid] if o["px"] is not None]
                saw_left = any(x < -0.2 for x in xs)
                lefts = [o for o in self._observations[pid] if o["px"] is not None and o["px"] < -0.2]
                steered = any(o["ml"] < o["mr"] for o in lefts) if lefts else False
                self._results[pid] = {
                    "pass": saw_left and steered,
                    "detail": f"Saw left: {saw_left}, steered left: {steered}",
                }
        elif pid == "follow_right":
            advance = elapsed >= self.phase["duration"]
            if advance:
                xs = [o["px"] for o in self._observations[pid] if o["px"] is not None]
                saw_right = any(x > 0.2 for x in xs)
                rights = [o for o in self._observations[pid] if o["px"] is not None and o["px"] > 0.2]
                steered = any(o["ml"] > o["mr"] for o in rights) if rights else False
                self._results[pid] = {
                    "pass": saw_right and steered,
                    "detail": f"Saw right: {saw_right}, steered right: {steered}",
                }
        elif pid == "follow_close_far":
            advance = elapsed >= self.phase["duration"]
            if advance:
                speeds = [max(o["ml"], o["mr"]) - NEUTRAL for o in self._observations[pid]
                          if o["persons"] > 0]
                if speeds:
                    self._results[pid] = {
                        "pass": max(speeds) > 5,
                        "detail": f"Max speed offset: {max(speeds)}, range: {min(speeds)}-{max(speeds)}",
                    }
                else:
                    self._results[pid] = {"pass": False, "detail": "No person seen"}

        if advance:
            self._phase_idx = min(self._phase_idx + 1, len(PHASES) - 1)
            self._phase_start = time.monotonic()


# ---------------------------------------------------------------------------
# Dashboard HTML
# ---------------------------------------------------------------------------

_HTML = r"""<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WALL-E Mini — Dry-Run Test</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:system-ui,sans-serif;background:#0f1117;color:#e0e0e0}
  header{background:#1a1d27;padding:12px 24px;border-bottom:1px solid #2a2d37;
         display:flex;align-items:center;justify-content:space-between}
  header h1{font-size:18px;color:#fff}
  .phase-badge{padding:3px 10px;border-radius:12px;font-size:12px;font-weight:600;
               text-transform:uppercase;letter-spacing:.5px;background:#4a9eff;color:#fff}
  .container{max-width:1300px;margin:0 auto;padding:16px}

  /* Instruction banner */
  .instruction{background:#1a1d27;border-radius:10px;padding:24px;margin-bottom:16px;
               text-align:center;border:2px solid #2a2d37}
  .instruction .text{font-size:28px;font-weight:700;line-height:1.3}

  /* Gauges row */
  .gauges{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;margin-bottom:16px}
  .gauge{background:#1a1d27;border-radius:8px;padding:16px}
  .gauge .label{font-size:11px;color:#888;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px}
  .bar-wrap{height:28px;background:#22252f;border-radius:4px;overflow:hidden;position:relative}
  .bar-fill{height:100%;border-radius:4px;transition:width .15s}
  .bar-text{position:absolute;top:0;left:0;right:0;bottom:0;display:flex;
            align-items:center;justify-content:center;font-size:13px;font-weight:600;color:#fff;
            text-shadow:0 1px 2px rgba(0,0,0,.5)}

  /* Motor bars: centered at 50% = neutral */
  .motor-bar{position:relative;height:28px;background:#22252f;border-radius:4px;overflow:hidden}
  .motor-bar .center{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#555;z-index:2}
  .motor-bar .fill{position:absolute;top:0;height:100%;border-radius:4px;transition:left .15s,width .15s}
  .motor-bar .fill.fwd{background:#4ecdc4}
  .motor-bar .fill.rev{background:#e63946}
  .motor-bar .fill.scaled{opacity:.4}
  .motor-bar .lbl{position:absolute;top:0;left:0;right:0;bottom:0;display:flex;
                   align-items:center;justify-content:center;font-size:13px;font-weight:600;
                   color:#fff;text-shadow:0 1px 2px rgba(0,0,0,.5);z-index:3}

  /* Person X indicator */
  .person-track{position:relative;height:28px;background:#22252f;border-radius:4px;overflow:hidden}
  .person-track .center{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#555}
  .person-track .dot{position:absolute;top:4px;width:20px;height:20px;border-radius:50%;
                     background:#4a9eff;transition:left .15s;margin-left:-10px}
  .person-track .no-person{position:absolute;top:0;left:0;right:0;bottom:0;display:flex;
                           align-items:center;justify-content:center;font-size:12px;color:#555}

  .streams{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:16px}
  .card{background:#1a1d27;border-radius:8px;overflow:hidden}
  .card h2{font-size:13px;padding:8px 12px;background:#22252f;color:#aaa;
           text-transform:uppercase;letter-spacing:.5px}
  .card img{width:100%;display:block;background:#111;min-height:180px}

  /* Results */
  .results{background:#1a1d27;border-radius:8px;padding:16px;margin-bottom:16px}
  .results h2{font-size:14px;margin-bottom:12px;color:#ccc}
  .result-row{display:flex;align-items:center;padding:6px 0;border-bottom:1px solid #22252f}
  .result-row:last-child{border-bottom:none}
  .result-icon{width:24px;font-size:18px;margin-right:10px;text-align:center}
  .result-id{font-family:monospace;font-size:13px;color:#aaa;width:140px}
  .result-detail{font-size:13px;color:#888;flex:1}
  .pass{color:#4ecdc4} .fail{color:#e63946}

  /* Raw stats */
  .raw-stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:6px;margin-bottom:16px}
  .rs{background:#1a1d27;border-radius:6px;padding:8px 10px}
  .rs .k{font-size:10px;color:#555;text-transform:uppercase} .rs .v{font-size:15px;font-weight:600}

  @media(max-width:800px){
    .streams,.gauges{grid-template-columns:1fr}
    .instruction .text{font-size:20px}
  }
</style></head><body>
<header>
  <h1>WALL-E Mini — Dry-Run Test</h1>
  <span class="phase-badge" id="phase-badge">INIT</span>
</header>
<div class="container">
  <div class="instruction" id="instr-box">
    <div class="text" id="instr-text">Initializing camera...</div>
  </div>

  <div class="gauges">
    <div class="gauge">
      <div class="label">Throttle Scale (obstacle avoidance)</div>
      <div class="bar-wrap"><div class="bar-fill" id="throttle-fill" style="width:100%;background:#4ecdc4"></div>
        <div class="bar-text" id="throttle-text">1.00</div></div>
    </div>
    <div class="gauge">
      <div class="label">Motor Left / Right (Follow Me)</div>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px">
        <div>
          <div style="font-size:10px;color:#666;margin-bottom:3px">LEFT</div>
          <div class="motor-bar"><div class="center"></div><div class="fill fwd" id="ml-fill"></div>
            <div class="fill fwd scaled" id="ml-sfill"></div><div class="lbl" id="ml-lbl">126</div></div>
        </div>
        <div>
          <div style="font-size:10px;color:#666;margin-bottom:3px">RIGHT</div>
          <div class="motor-bar"><div class="center"></div><div class="fill fwd" id="mr-fill"></div>
            <div class="fill fwd scaled" id="mr-sfill"></div><div class="lbl" id="mr-lbl">126</div></div>
        </div>
      </div>
    </div>
    <div class="gauge">
      <div class="label">Person Position (lateral)</div>
      <div class="person-track" id="person-track">
        <div class="center"></div>
        <div class="no-person" id="no-person">no person</div>
        <div class="dot" id="person-dot" style="display:none;left:50%"></div>
      </div>
    </div>
  </div>

  <div class="streams">
    <div class="card"><h2>RGB + Detections</h2><img src="/stream/rgb" alt="RGB"></div>
    <div class="card"><h2>Depth Map</h2><img src="/stream/depth" alt="Depth"></div>
  </div>

  <div class="results" id="results-box" style="display:none">
    <h2>Test Results</h2>
    <div id="results-list"></div>
  </div>

  <div class="raw-stats" id="raw-stats"></div>
</div>
<script>
function motorBar(id, sfid, lblId, val, sval) {
  const pct = (v) => Math.min(Math.abs(v - 126) / 126 * 50, 50);
  const el = document.getElementById(id);
  const sel = document.getElementById(sfid);
  const lbl = document.getElementById(lblId);
  const fwd = val >= 126;
  const sfwd = sval >= 126;
  el.className = 'fill ' + (fwd ? 'fwd' : 'rev');
  sel.className = 'fill ' + (sfwd ? 'fwd' : 'rev') + ' scaled';
  if (fwd) { el.style.left = '50%'; el.style.width = pct(val) + '%'; }
  else { const w = pct(val); el.style.left = (50 - w) + '%'; el.style.width = w + '%'; }
  if (sfwd) { sel.style.left = '50%'; sel.style.width = pct(sval) + '%'; }
  else { const w = pct(sval); sel.style.left = (50 - w) + '%'; sel.style.width = w + '%'; }
  lbl.textContent = val + ' (' + sval + ')';
}

const es = new EventSource('/events');
es.onmessage = function(e) {
  const d = JSON.parse(e.data);

  // Instruction
  document.getElementById('instr-text').textContent = d.test_instruction || '';
  document.getElementById('phase-badge').textContent = (d.test_phase || 'init').toUpperCase();

  // Throttle bar
  const s = d.throttle_scale != null ? d.throttle_scale : 1;
  const tf = document.getElementById('throttle-fill');
  tf.style.width = (s * 100) + '%';
  tf.style.background = s > 0.7 ? '#4ecdc4' : s > 0.3 ? '#e6a239' : '#e63946';
  document.getElementById('throttle-text').textContent = s.toFixed(2);

  // Motor bars
  motorBar('ml-fill', 'ml-sfill', 'ml-lbl', d.motor_left || 126, d.scaled_left || 126);
  motorBar('mr-fill', 'mr-sfill', 'mr-lbl', d.motor_right || 126, d.scaled_right || 126);

  // Person dot
  const dot = document.getElementById('person-dot');
  const np = document.getElementById('no-person');
  if (d.person_x_m != null) {
    dot.style.display = 'block'; np.style.display = 'none';
    const pct = 50 + (d.person_x_m / 3) * 50;
    dot.style.left = Math.max(2, Math.min(98, pct)) + '%';
  } else { dot.style.display = 'none'; np.style.display = 'flex'; }

  // Instruction box border color
  const box = document.getElementById('instr-box');
  if (d.test_phase === 'done') box.style.borderColor = '#4ecdc4';
  else if (d.test_phase === 'close') box.style.borderColor = '#e63946';
  else box.style.borderColor = '#4a9eff';

  // Results
  if (d.test_results && Object.keys(d.test_results).length) {
    const rb = document.getElementById('results-box');
    rb.style.display = 'block';
    const rl = document.getElementById('results-list');
    rl.innerHTML = Object.entries(d.test_results).map(function(kv) {
      const k = kv[0], r = kv[1];
      const cls = r.pass ? 'pass' : 'fail';
      const icon = r.pass ? '&#10003;' : '&#10007;';
      return '<div class="result-row"><div class="result-icon ' + cls + '">' + icon + '</div>' +
             '<div class="result-id">' + k + '</div><div class="result-detail">' + r.detail + '</div></div>';
    }).join('');
  }

  // Raw stats
  const raw = document.getElementById('raw-stats');
  const skip = new Set(['test_instruction','test_phase','test_results','motor_left','motor_right',
                        'scaled_left','scaled_right','throttle_scale','person_x_m']);
  raw.innerHTML = Object.entries(d).filter(function(kv){ return !skip.has(kv[0]); }).map(function(kv){
    return '<div class="rs"><div class="k">' + kv[0] + '</div><div class="v">' + kv[1] + '</div></div>';
  }).join('');
};
</script></body></html>"""


# ---------------------------------------------------------------------------
# Live viewer with control logic
# ---------------------------------------------------------------------------

class LiveViewer:
    def __init__(self, port: int = 8080):
        self.port = port
        self._rgb_jpeg: bytes = b""
        self._depth_jpeg: bytes = b""
        self._stats: dict = {}
        self._lock = threading.Lock()

    def run(self):
        cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        cam_thread.start()
        self._serve()

    def _camera_loop(self):
        import depthai as dai

        oa = ObstacleAvoidanceController(config.obstacle_avoidance)
        fm = FollowMeController(config.follow_me)
        seq = TestSequence()

        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setOutputSize(640, 400)
        mono_left.requestOutput((640, 400)).link(stereo.left)
        mono_right.requestOutput((640, 400)).link(stereo.right)

        model_desc = dai.NNModelDescription(model="mobilenet-ssd", platform="RVC2")
        spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
            cam_rgb, stereo, model_desc,
        )
        spatial_nn.input.setBlocking(False)
        spatial_nn.setBoundingBoxScaleFactor(0.5)
        spatial_nn.setDepthLowerThreshold(300)
        spatial_nn.setDepthUpperThreshold(10000)
        spatial_nn.setConfidenceThreshold(0.3)

        det_q = spatial_nn.out.createOutputQueue()
        depth_q = spatial_nn.passthroughDepth.createOutputQueue()
        rgb_q = spatial_nn.passthrough.createOutputQueue()

        pipeline.start()
        print(f"OAK-D pipeline started. Open http://0.0.0.0:{self.port}/")

        detections = []
        depth_ts = 0.0
        min_depth_m: float | None = None

        while pipeline.isRunning():
            now = time.monotonic()

            # --- RGB ---
            rgb_msg = rgb_q.tryGet()
            if rgb_msg is not None:
                img = rgb_msg.getCvFrame()
                h, w = img.shape[:2]
                for dd in detections:
                    lbl = LABELS[dd.label] if dd.label < len(LABELS) else "?"
                    x1, y1 = int(dd.xmin * w), int(dd.ymin * h)
                    x2, y2 = int(dd.xmax * w), int(dd.ymax * h)
                    z = dd.spatialCoordinates.z / 1000
                    is_person = dd.label == PERSON_LABEL
                    color = (0, 255, 0) if is_person else (0, 140, 255)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(img, f"{lbl} {dd.confidence:.0%} {z:.1f}m",
                                (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with self._lock:
                    self._rgb_jpeg = buf.tobytes()

            # --- Depth ---
            depth_msg = depth_q.tryGet()
            if depth_msg is not None:
                frame = depth_msg.getFrame()
                valid = frame[frame > 0]
                if valid.size > 0:
                    min_depth_m = round(float(np.percentile(valid, 5)) / 1000, 2)
                    depth_ts = now
                d = frame.astype(float)
                d[d == 0] = np.nan
                mn, mx = np.nanpercentile(d, 2), np.nanpercentile(d, 98)
                norm = np.clip((frame.astype(float) - mn) / (mx - mn + 1), 0, 1)
                cmap = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                cmap[frame == 0] = 0
                _, buf = cv2.imencode(".jpg", cmap, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with self._lock:
                    self._depth_jpeg = buf.tobytes()

            # --- Detections ---
            det_msg = det_q.tryGet()
            if det_msg is not None:
                detections = list(det_msg.detections)

            # --- Control logic ---
            depth_age = now - depth_ts if depth_ts > 0 else 999.0
            scale = oa.compute_throttle_scale(
                min_depth_m if min_depth_m is not None else 99.0,
                depth_age,
            )

            persons = []
            for dd in detections:
                if dd.label != PERSON_LABEL:
                    continue
                if dd.confidence < config.follow_me.detection_confidence:
                    continue
                persons.append(PersonDetection(
                    x_m=dd.spatialCoordinates.x / 1000.0,
                    z_m=dd.spatialCoordinates.z / 1000.0,
                    confidence=dd.confidence,
                    bbox=(dd.xmin, dd.ymin, dd.xmax, dd.ymax),
                ))

            motor_l, motor_r = fm.compute(persons)
            scaled_l = _scale_toward_neutral(motor_l, scale)
            scaled_r = _scale_toward_neutral(motor_r, scale)

            best_person_x = None
            best_person_z = None
            if persons:
                best = max(persons, key=lambda p: p.confidence)
                best_person_x = round(best.x_m, 2)
                best_person_z = round(best.z_m, 2)

            # --- Test sequence ---
            seq.tick(
                depth_m=min_depth_m, scale=scale,
                persons=len(persons), person_x=best_person_x,
                motor_l=motor_l, motor_r=motor_r,
            )

            # --- Pack stats ---
            stats = {
                "test_phase": seq.phase["id"],
                "test_instruction": seq.phase["instruction"],
                "test_results": seq.results,
                "throttle_scale": round(scale, 2),
                "motor_left": motor_l,
                "motor_right": motor_r,
                "scaled_left": scaled_l,
                "scaled_right": scaled_r,
                "person_x_m": best_person_x,
                "min_depth": f"{min_depth_m}m" if min_depth_m else "---",
                "depth_age": f"{depth_age:.1f}s",
                "persons": len(persons),
                "person_z_m": f"{best_person_z}m" if best_person_z else "---",
                "all_detections": len(detections),
            }
            with self._lock:
                self._stats = stats

            time.sleep(0.03)

    def _serve(self):
        from flask import Flask, Response

        app = Flask(__name__)
        logging.getLogger("werkzeug").setLevel(logging.WARNING)

        @app.route("/")
        def index():
            return Response(_HTML, content_type="text/html")

        @app.route("/stream/rgb")
        def stream_rgb():
            return Response(self._mjpeg("rgb"),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/stream/depth")
        def stream_depth():
            return Response(self._mjpeg("depth"),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/events")
        def events():
            def gen():
                while True:
                    with self._lock:
                        data = dict(self._stats)
                    yield f"data: {json.dumps(data)}\n\n"
                    time.sleep(0.25)
            return Response(gen(), mimetype="text/event-stream")

        app.run(host="0.0.0.0", port=self.port, threaded=True, use_reloader=False)

    def _mjpeg(self, which: str):
        while True:
            with self._lock:
                frame = self._rgb_jpeg if which == "rgb" else self._depth_jpeg
            if frame:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="OAK-D dry-run test dashboard")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    LiveViewer(port=args.port).run()


if __name__ == "__main__":
    main()
