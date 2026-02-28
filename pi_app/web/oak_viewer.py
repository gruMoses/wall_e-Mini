"""
Live web viewer for OAK-D Lite camera feeds and recordings.

Serves a dashboard at http://<pi-ip>:8080 with:
  - Live annotated RGB camera feed (MJPEG stream)
  - Live colorized depth map (MJPEG stream)
  - Real-time telemetry (Server-Sent Events)
  - Recordings browser with download links for MCAP and MP4 files

Runs on a background daemon thread — does not block the main control loop.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

try:
    from flask import Flask, Response, send_from_directory, abort
except ImportError:
    Flask = None  # type: ignore[assignment,misc]

import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import OakWebViewerConfig


# ---------------------------------------------------------------------------
# Dashboard HTML (self-contained, no external dependencies)
# ---------------------------------------------------------------------------

_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WALL-E Mini — OAK-D Viewer</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
         background: #0f1117; color: #e0e0e0; }
  header { background: #1a1d27; padding: 12px 24px; display: flex;
           align-items: center; justify-content: space-between; border-bottom: 1px solid #2a2d37; }
  header h1 { font-size: 18px; font-weight: 600; color: #fff; }
  .rec-badge { padding: 3px 10px; border-radius: 12px; font-size: 12px; font-weight: 600;
               text-transform: uppercase; letter-spacing: 0.5px; }
  .rec-idle { background: #2a2d37; color: #888; }
  .rec-recording { background: #e63946; color: #fff; animation: pulse 1.2s infinite; }
  .rec-lingering { background: #e6a239; color: #111; }
  @keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: 0.6; } }
  .container { max-width: 1200px; margin: 0 auto; padding: 16px; }
  .streams { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 16px; }
  .stream-card { background: #1a1d27; border-radius: 8px; overflow: hidden; }
  .stream-card h2 { font-size: 13px; padding: 8px 12px; background: #22252f;
                     color: #aaa; font-weight: 500; text-transform: uppercase; letter-spacing: 0.5px; }
  .stream-card img { width: 100%; display: block; background: #111; min-height: 200px; }
  .stream-card .stream-status { font-size: 10px; padding: 2px 8px; color: #555; text-align: right; }
  .telemetry { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
               gap: 8px; margin-bottom: 16px; }
  .telem-card { background: #1a1d27; border-radius: 8px; padding: 12px; }
  .telem-card .label { font-size: 11px; color: #888; text-transform: uppercase;
                        letter-spacing: 0.5px; margin-bottom: 4px; }
  .telem-card .value { font-size: 22px; font-weight: 600; font-variant-numeric: tabular-nums; }
  .telem-card .value.green { color: #4ecdc4; }
  .telem-card .value.red { color: #e63946; }
  .telem-card .value.yellow { color: #e6a239; }
  .telem-card .value.blue { color: #4a9eff; }
  .recordings { background: #1a1d27; border-radius: 8px; padding: 16px; }
  .recordings h2 { font-size: 14px; margin-bottom: 12px; color: #ccc; }
  .rec-list { list-style: none; }
  .rec-list li { padding: 8px 0; border-bottom: 1px solid #22252f; display: flex;
                 align-items: center; justify-content: space-between; }
  .rec-list li:last-child { border-bottom: none; }
  .rec-list .session-name { font-family: monospace; font-size: 13px; color: #aaa; }
  .rec-list .links a { color: #4a9eff; text-decoration: none; font-size: 13px;
                        margin-left: 12px; }
  .rec-list .links a:hover { text-decoration: underline; }
  .empty { color: #555; font-size: 13px; font-style: italic; }
  .follow-btn { padding: 10px 24px; border: none; border-radius: 6px; font-size: 15px;
                 font-weight: 700; cursor: pointer; letter-spacing: 0.5px; transition: all 0.2s; }
  .follow-btn.activate { background: #4ecdc4; color: #111; }
  .follow-btn.activate:hover { background: #3dbdb4; }
  .follow-btn.deactivate { background: #e63946; color: #fff; }
  .follow-btn.deactivate:hover { background: #d32836; }
  .follow-btn:disabled { background: #333; color: #666; cursor: not-allowed; }
  .follow-btn-row { display: flex; align-items: center; gap: 12px; margin-bottom: 12px; }
  .follow-btn-hint { font-size: 11px; color: #555; }
  .tracking-panel { background: #1a1d27; border-radius: 8px; padding: 16px; margin-bottom: 16px;
                     border: 2px solid #2a2d37; transition: border-color 0.3s; }
  .tracking-panel.tracking-locked { border-color: #4ecdc4; }
  .tracking-panel.tracking-follow { border-color: #e6a239; animation: pulse 1.2s infinite; }
  .tracking-panel.tracking-off { border-color: #2a2d37; }
  .tracking-header { display: flex; align-items: center; gap: 10px; margin-bottom: 12px; }
  .tracking-dot { width: 14px; height: 14px; border-radius: 50%; flex-shrink: 0;
                   background: #555; transition: background 0.3s; }
  .tracking-locked .tracking-dot { background: #4ecdc4; box-shadow: 0 0 8px #4ecdc4; }
  .tracking-follow .tracking-dot { background: #e6a239; box-shadow: 0 0 8px #e6a239; }
  .tracking-label { font-size: 14px; font-weight: 600; letter-spacing: 0.5px; color: #aaa; }
  .tracking-locked .tracking-label { color: #4ecdc4; }
  .tracking-follow .tracking-label { color: #e6a239; }
  .tracking-details { display: flex; gap: 16px; align-items: flex-start; }
  .tracking-radar { background: #111; border-radius: 6px; padding: 4px; }
  .tracking-radar canvas { display: block; }
  .tracking-stats { font-size: 13px; color: #aaa; line-height: 1.8; font-variant-numeric: tabular-nums; }
  .tracking-stats .stat-val { color: #e0e0e0; font-weight: 600; }
  @media (max-width: 700px) { .streams { grid-template-columns: 1fr; }
    .tracking-details { flex-direction: column; } }
</style>
</head>
<body>
<header>
  <h1>WALL-E Mini — OAK-D Live</h1>
  <span id="rec-badge" class="rec-badge rec-idle">IDLE</span>
</header>
<div class="container">
  <div class="streams">
    <div class="stream-card">
      <h2>RGB Camera (annotated)</h2>
      <img id="rgb-stream" src="/stream/rgb" alt="RGB stream">
      <div class="stream-status" id="rgb-status"></div>
    </div>
    <div class="stream-card">
      <h2>Depth Map (colorized)</h2>
      <img id="depth-stream" src="/stream/depth" alt="Depth stream">
      <div class="stream-status" id="depth-status"></div>
    </div>
  </div>
  <div class="telemetry">
    <div class="telem-card">
      <div class="label">Mode</div>
      <div class="value blue" id="t-mode">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Obstacle Distance</div>
      <div class="value green" id="t-dist">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Throttle Scale</div>
      <div class="value" id="t-scale">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Motor L / R</div>
      <div class="value" id="t-motors">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Persons Detected</div>
      <div class="value blue" id="t-persons">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Armed</div>
      <div class="value" id="t-armed">—</div>
    </div>
  </div>
  <div id="tracking-panel" class="tracking-panel tracking-off">
    <div class="follow-btn-row">
      <button id="follow-btn" class="follow-btn activate" onclick="toggleFollowMe()">ACTIVATE FOLLOW ME</button>
      <span class="follow-btn-hint" id="follow-hint">System must be armed</span>
    </div>
    <div class="tracking-header">
      <span class="tracking-dot" id="tracking-dot"></span>
      <span class="tracking-label" id="tracking-label">FOLLOW ME — NO TARGET</span>
    </div>
    <div class="tracking-details" id="tracking-details">
      <div class="tracking-radar">
        <canvas id="radar" width="280" height="200"></canvas>
      </div>
      <div class="tracking-stats" id="tracking-stats"></div>
    </div>
  </div>
  <div class="recordings">
    <h2>Recordings</h2>
    <ul class="rec-list" id="rec-list">
      <li class="empty">Loading...</li>
    </ul>
  </div>
</div>
<script>
const sse = new EventSource('/api/telemetry');
const radarCanvas = document.getElementById('radar');
const radarCtx = radarCanvas.getContext('2d');
function drawRadar(detections, targetX, targetZ, tracking, mode) {
  const W = radarCanvas.width, H = radarCanvas.height;
  const cx = W / 2, maxZ = 5.0, maxX = 3.0;
  radarCtx.fillStyle = '#111'; radarCtx.fillRect(0, 0, W, H);
  radarCtx.strokeStyle = '#222'; radarCtx.lineWidth = 1;
  for (let r = 1; r <= 5; r++) {
    const y = H - (r / maxZ) * H;
    radarCtx.beginPath(); radarCtx.moveTo(0, y); radarCtx.lineTo(W, y); radarCtx.stroke();
    radarCtx.fillStyle = '#333'; radarCtx.font = '10px monospace';
    radarCtx.fillText(r + 'm', 2, y - 2);
  }
  radarCtx.strokeStyle = '#222';
  radarCtx.beginPath(); radarCtx.moveTo(cx, 0); radarCtx.lineTo(cx, H); radarCtx.stroke();
  radarCtx.fillStyle = '#4a9eff'; radarCtx.font = '9px sans-serif';
  radarCtx.fillText('CAM', cx - 10, H - 2);
  const dets = detections || [];
  for (const det of dets) {
    const px = cx + (det.x_m / maxX) * (W / 2);
    const py = H - (det.z_m / maxZ) * H;
    const isTarget = tracking && targetX != null && Math.abs(det.x_m - targetX) < 0.1 && Math.abs(det.z_m - targetZ) < 0.2;
    radarCtx.beginPath(); radarCtx.arc(px, py, isTarget ? 8 : 5, 0, Math.PI * 2);
    radarCtx.fillStyle = isTarget ? (mode === 'FOLLOW_ME' ? '#e6a239' : '#4ecdc4') : '#888';
    radarCtx.fill();
    if (isTarget) {
      radarCtx.strokeStyle = isTarget ? '#fff' : '#666'; radarCtx.lineWidth = 2;
      radarCtx.stroke();
      radarCtx.fillStyle = '#fff'; radarCtx.font = 'bold 10px monospace';
      radarCtx.fillText(det.z_m.toFixed(1) + 'm', px + 12, py + 4);
    }
  }
}
sse.onmessage = function(e) {
  const d = JSON.parse(e.data);
  document.getElementById('t-mode').textContent = d.mode || '—';
  const dist = d.obstacle_distance_m;
  document.getElementById('t-dist').textContent = dist != null ? dist.toFixed(2) + 'm' : '—';
  const scl = d.throttle_scale;
  const sclEl = document.getElementById('t-scale');
  sclEl.textContent = scl != null ? scl.toFixed(2) : '—';
  sclEl.className = 'value ' + (scl >= 0.8 ? 'green' : scl >= 0.3 ? 'yellow' : 'red');
  document.getElementById('t-motors').textContent =
    d.motor_left + ' / ' + d.motor_right;
  document.getElementById('t-persons').textContent = d.num_persons ?? '—';
  const armedEl = document.getElementById('t-armed');
  armedEl.textContent = d.is_armed ? 'YES' : 'NO';
  armedEl.className = 'value ' + (d.is_armed ? 'green' : 'red');
  const badge = document.getElementById('rec-badge');
  const rs = d.recording_state || 'IDLE';
  badge.textContent = rs;
  badge.className = 'rec-badge rec-' + rs.toLowerCase();
  const panel = document.getElementById('tracking-panel');
  const label = document.getElementById('tracking-label');
  const stats = document.getElementById('tracking-stats');
  const isFollowMode = d.mode === 'FOLLOW_ME';
  const tracking = d.follow_tracking;
  if (isFollowMode && tracking) {
    panel.className = 'tracking-panel tracking-follow';
    label.textContent = 'FOLLOW ME — LOCKED ON';
  } else if (tracking) {
    panel.className = 'tracking-panel tracking-locked';
    label.textContent = 'TARGET LOCKED — ' + d.num_persons + ' person(s)';
  } else if (d.num_persons > 0) {
    panel.className = 'tracking-panel tracking-locked';
    label.textContent = 'PERSON DETECTED — not tracking';
  } else {
    panel.className = 'tracking-panel tracking-off';
    label.textContent = 'NO PERSON DETECTED';
  }
  let statsHtml = '';
  if (d.follow_target_z_m != null) {
    statsHtml += 'Target distance: <span class="stat-val">' + d.follow_target_z_m.toFixed(2) + ' m</span><br>';
  }
  if (d.follow_target_x_m != null) {
    const dir = d.follow_target_x_m > 0.1 ? ' (right)' : d.follow_target_x_m < -0.1 ? ' (left)' : ' (center)';
    statsHtml += 'Target lateral: <span class="stat-val">' + d.follow_target_x_m.toFixed(2) + ' m' + dir + '</span><br>';
  }
  statsHtml += 'Persons seen: <span class="stat-val">' + (d.num_persons ?? 0) + '</span><br>';
  statsHtml += 'Mode: <span class="stat-val">' + d.mode + '</span>';
  if (isFollowMode) statsHtml += '<br>Motors: <span class="stat-val">L=' + d.motor_left + ' R=' + d.motor_right + '</span>';
  stats.innerHTML = statsHtml;
  drawRadar(d.detections, d.follow_target_x_m, d.follow_target_z_m, tracking, d.mode);
  const btn = document.getElementById('follow-btn');
  const hint = document.getElementById('follow-hint');
  if (isFollowMode) {
    btn.textContent = 'STOP FOLLOWING';
    btn.className = 'follow-btn deactivate';
    btn.disabled = false;
    hint.textContent = 'Currently following';
  } else if (d.is_armed) {
    btn.textContent = 'ACTIVATE FOLLOW ME';
    btn.className = 'follow-btn activate';
    btn.disabled = false;
    hint.textContent = d.num_persons > 0 ? 'Person detected — ready' : 'Waiting for person';
  } else {
    btn.textContent = 'ACTIVATE FOLLOW ME';
    btn.className = 'follow-btn activate';
    btn.disabled = true;
    hint.textContent = 'System must be armed (ch3 high)';
  }
};
function toggleFollowMe() {
  const btn = document.getElementById('follow-btn');
  btn.disabled = true;
  fetch('/api/follow_me', {method: 'POST'})
    .then(r => r.json())
    .then(d => {
      if (d.error) { document.getElementById('follow-hint').textContent = d.error; }
    })
    .catch(e => { document.getElementById('follow-hint').textContent = 'Error: ' + e; })
    .finally(() => { setTimeout(() => { btn.disabled = false; }, 500); });
}
function loadRecordings() {
  fetch('/api/recordings').then(r => r.json()).then(sessions => {
    const ul = document.getElementById('rec-list');
    if (!sessions.length) { ul.innerHTML = '<li class="empty">No recordings yet</li>'; return; }
    ul.innerHTML = sessions.map(s => {
      const links = s.files.map(f =>
        '<a href="/recordings/' + s.name + '/' + f + '" target="_blank">' + f + '</a>'
      ).join('');
      return '<li><span class="session-name">' + s.name + '</span><span class="links">' + links + '</span></li>';
    }).join('');
  }).catch(() => {});
}
loadRecordings();
setInterval(loadRecordings, 15000);
function keepAlive(imgId, statusId, url) {
  const img = document.getElementById(imgId);
  const status = document.getElementById(statusId);
  let cooldown = false;
  img.onerror = function() {
    if (cooldown) return;
    cooldown = true;
    setTimeout(function() {
      img.src = url + '?t=' + Date.now();
      if (status) status.textContent = 'reconnected ' + new Date().toLocaleTimeString();
      cooldown = false;
    }, 3000);
  };
}
keepAlive('rgb-stream', 'rgb-status', '/stream/rgb');
keepAlive('depth-stream', 'depth-status', '/stream/depth');
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Placeholder JPEG (shown when camera has no frames yet)
# ---------------------------------------------------------------------------

def _placeholder_jpeg() -> bytes:
    try:
        import cv2
        import numpy as np
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        cv2.putText(img, "Waiting for camera...", (30, 155),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1, cv2.LINE_AA)
        _, buf = cv2.imencode(".jpg", img)
        return buf.tobytes()
    except Exception:
        return b""


# ---------------------------------------------------------------------------
# Flask app factory
# ---------------------------------------------------------------------------

def create_app(recorder, config: OakWebViewerConfig, controller=None) -> Flask:
    """Create the Flask app wired to the given OakRecorder and Controller instances."""
    if Flask is None:
        raise ImportError("Flask is required for the web viewer: pip install flask")

    app = Flask(__name__)
    app.config["PROPAGATE_EXCEPTIONS"] = False
    placeholder = _placeholder_jpeg()

    # -- Dashboard -----------------------------------------------------------

    @app.route("/")
    def index():
        return Response(_DASHBOARD_HTML, content_type="text/html")

    # -- MJPEG streams -------------------------------------------------------

    def _mjpeg_generator(get_jpeg_fn, fps: float = 10.0):
        interval = 1.0 / fps
        while True:
            jpeg = get_jpeg_fn()
            frame = jpeg if jpeg else placeholder
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            time.sleep(interval)

    @app.route("/stream/rgb")
    def stream_rgb():
        return Response(
            _mjpeg_generator(recorder.get_latest_annotated_jpeg, fps=10),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/stream/depth")
    def stream_depth():
        return Response(
            _mjpeg_generator(recorder.get_latest_depth_jpeg, fps=5),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    # -- Telemetry SSE -------------------------------------------------------

    def _sse_generator():
        while True:
            t = recorder.get_latest_telemetry()
            if t is not None:
                obj = {
                    "mode": t.mode,
                    "throttle_scale": round(t.throttle_scale, 3),
                    "obstacle_distance_m": round(t.obstacle_distance_m, 3) if t.obstacle_distance_m else None,
                    "motor_left": t.motor_left,
                    "motor_right": t.motor_right,
                    "is_armed": t.is_armed,
                    "num_persons": len(t.person_detections),
                    "recording_state": recorder.recording_state,
                    "follow_tracking": t.follow_tracking,
                    "follow_target_x_m": round(t.follow_target_x_m, 2) if t.follow_target_x_m is not None else None,
                    "follow_target_z_m": round(t.follow_target_z_m, 2) if t.follow_target_z_m is not None else None,
                    "detections": [
                        {"x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
                         "conf": round(d.confidence, 2)}
                        for d in t.person_detections
                    ],
                }
                yield f"data: {json.dumps(obj)}\n\n"
            time.sleep(0.2)

    @app.route("/api/telemetry")
    def api_telemetry():
        return Response(_sse_generator(), mimetype="text/event-stream")

    # -- Follow Me toggle API ------------------------------------------------

    @app.route("/api/follow_me", methods=["POST"])
    def api_follow_me_toggle():
        if controller is None:
            return Response(json.dumps({"error": "no controller"}), status=503,
                            content_type="application/json")
        t = recorder.get_latest_telemetry()
        mode = t.mode if t else "MANUAL"
        if mode == "FOLLOW_ME":
            controller.deactivate_follow_me()
            return Response(json.dumps({"mode": "MANUAL", "action": "deactivated"}),
                            content_type="application/json")
        else:
            ok = controller.activate_follow_me()
            if ok:
                return Response(json.dumps({"mode": "FOLLOW_ME", "action": "activated"}),
                                content_type="application/json")
            return Response(json.dumps({"error": "must be armed", "mode": mode}),
                            status=400, content_type="application/json")

    # -- Recordings API ------------------------------------------------------

    @app.route("/api/recordings")
    def api_recordings():
        rec_dir = recorder.recordings_dir
        sessions = []
        if rec_dir.exists():
            for d in sorted(rec_dir.iterdir(), reverse=True):
                if not d.is_dir():
                    continue
                files = sorted(f.name for f in d.iterdir() if f.is_file())
                if files:
                    sessions.append({"name": d.name, "files": files})
        return Response(json.dumps(sessions), content_type="application/json")

    @app.route("/recordings/<session>/<filename>")
    def serve_recording(session: str, filename: str):
        rec_dir = recorder.recordings_dir / session
        if not rec_dir.exists():
            abort(404)
        fpath = rec_dir / filename
        if not fpath.exists() or not fpath.is_file():
            abort(404)
        return send_from_directory(str(rec_dir), filename)

    return app


# ---------------------------------------------------------------------------
# Background server thread
# ---------------------------------------------------------------------------

class OakWebViewer:
    """Runs the Flask web viewer on a daemon thread."""

    def __init__(self, config: OakWebViewerConfig, recorder, controller=None) -> None:
        self._config = config
        self._recorder = recorder
        self._controller = controller
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if Flask is None:
            logger.warning("Flask not installed — web viewer disabled")
            return
        self._thread = threading.Thread(
            target=self._run, name="OakWebViewer", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        try:
            app = create_app(self._recorder, self._config, controller=self._controller)
            app.run(
                host=self._config.host,
                port=self._config.port,
                threaded=True,
                use_reloader=False,
            )
        except Exception:
            logger.exception("Web viewer failed to start")
