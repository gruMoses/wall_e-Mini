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
from copy import deepcopy

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
  header .nav-link { color: #7aa2f7; text-decoration: none; font-size: 13px; font-weight: 500; }
  header .nav-link:hover { text-decoration: underline; }
  .hdr-right { display: flex; align-items: center; gap: 14px; }
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
  .stream-card img { width: 100%; display: block; background: #111; aspect-ratio: 640/352; }
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
  <div class="hdr-right">
    <a href="/map" class="nav-link">Map</a>
    <a href="/calibrate" class="nav-link">Calibrate</a>
    <span id="rec-badge" class="rec-badge rec-idle">IDLE</span>
  </div>
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
    <div class="telem-card">
      <div class="label">GPS Fix</div>
      <div class="value" id="t-gps-fix">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Satellites</div>
      <div class="value blue" id="t-gps-sats">—</div>
    </div>
    <div class="telem-card">
      <div class="label">HDOP</div>
      <div class="value" id="t-gps-hdop">—</div>
    </div>
    <div class="telem-card">
      <div class="label">GPS Position</div>
      <div class="value" id="t-gps-pos" style="font-size:14px;">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Altitude</div>
      <div class="value" id="t-gps-alt">—</div>
    </div>
    <div class="telem-card">
      <div class="label">LoRa Link</div>
      <div class="value" id="t-lora-link">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Camera Pipeline</div>
      <div class="value" id="t-cam-pipeline">—</div>
    </div>
    <div class="telem-card">
      <div class="label">RGB Age</div>
      <div class="value" id="t-cam-rgb-age">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Depth Age</div>
      <div class="value" id="t-cam-depth-age">—</div>
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
  const fixNames = {0:'None',1:'GPS',2:'DGPS',4:'RTK Fix',5:'RTK Float'};
  const fixEl = document.getElementById('t-gps-fix');
  if (d.gps_fix != null) {
    fixEl.textContent = (fixNames[d.gps_fix] || d.gps_fix);
    fixEl.className = 'value ' + (d.gps_fix >= 4 ? 'green' : d.gps_fix >= 1 ? 'yellow' : 'red');
  } else { fixEl.textContent = '—'; fixEl.className = 'value'; }
  document.getElementById('t-gps-sats').textContent = d.gps_sats != null ? d.gps_sats : '—';
  const hdopEl = document.getElementById('t-gps-hdop');
  if (d.gps_hdop != null) {
    hdopEl.textContent = d.gps_hdop.toFixed(2);
    hdopEl.className = 'value ' + (d.gps_hdop <= 1.0 ? 'green' : d.gps_hdop <= 2.0 ? 'yellow' : 'red');
  } else { hdopEl.textContent = '—'; hdopEl.className = 'value'; }
  document.getElementById('t-gps-pos').textContent =
    d.gps_lat != null ? d.gps_lat.toFixed(7) + ', ' + d.gps_lon.toFixed(7) : '—';
  document.getElementById('t-gps-alt').textContent =
    d.gps_alt_m != null ? d.gps_alt_m.toFixed(1) + ' m' : '—';
  const loraEl = document.getElementById('t-lora-link');
  if (d.gps_diff_age_s != null) {
    const da = d.gps_diff_age_s;
    const fix = d.gps_fix || 0;
    let txt = da.toFixed(1) + 's';
    let cls;
    if (da < 2 && fix >= 4) { cls = 'green'; }
    else if (da < 5) { cls = 'yellow'; }
    else if (da < 15) { cls = 'yellow'; }
    else { cls = 'red'; txt = 'Lost'; }
    if (d.gps_station_id) txt += ' #' + d.gps_station_id;
    loraEl.textContent = txt;
    loraEl.className = 'value ' + cls;
  } else {
    loraEl.textContent = (d.gps_fix != null && d.gps_fix < 4) ? 'No Link' : '—';
    loraEl.className = 'value ' + (d.gps_fix != null && d.gps_fix < 4 ? 'red' : '');
  }
  const cam = d.camera_health || {};
  const pipeEl = document.getElementById('t-cam-pipeline');
  const rgbAgeEl = document.getElementById('t-cam-rgb-age');
  const depthAgeEl = document.getElementById('t-cam-depth-age');
  if (cam.pipeline_running === true) {
    pipeEl.textContent = cam.is_stale ? 'STALE' : 'RUNNING';
    pipeEl.className = 'value ' + (cam.is_stale ? 'yellow' : 'green');
  } else if (cam.pipeline_running === false) {
    pipeEl.textContent = 'STOPPED';
    pipeEl.className = 'value red';
  } else {
    pipeEl.textContent = '—';
    pipeEl.className = 'value';
  }
  const rgbAge = cam.rgb_age_s;
  if (rgbAge != null) {
    rgbAgeEl.textContent = rgbAge.toFixed(2) + 's';
    rgbAgeEl.className = 'value ' + (rgbAge <= 1.5 ? 'green' : rgbAge <= 3.0 ? 'yellow' : 'red');
  } else {
    rgbAgeEl.textContent = '—';
    rgbAgeEl.className = 'value';
  }
  const depthAge = cam.depth_age_s;
  if (depthAge != null) {
    depthAgeEl.textContent = depthAge.toFixed(2) + 's';
    depthAgeEl.className = 'value ' + (depthAge <= 1.0 ? 'green' : depthAge <= 2.5 ? 'yellow' : 'red');
  } else {
    depthAgeEl.textContent = '—';
    depthAgeEl.className = 'value';
  }
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
        img = np.zeros((352, 640, 3), dtype=np.uint8)
        cv2.putText(img, "Waiting for camera...", (200, 185),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 1, cv2.LINE_AA)
        _, buf = cv2.imencode(".jpg", img)
        return buf.tobytes()
    except Exception:
        return b""


# ---------------------------------------------------------------------------
# Flask app factory
# ---------------------------------------------------------------------------

def create_app(recorder, config: OakWebViewerConfig, controller=None, oak_reader=None) -> Flask:
    """Create the Flask app wired to the given OakRecorder and Controller instances."""
    if Flask is None:
        raise ImportError("Flask is required for the web viewer: pip install flask")

    app = Flask(__name__)
    app.config["PROPAGATE_EXCEPTIONS"] = False
    placeholder = _placeholder_jpeg()
    rec_cache_lock = threading.Lock()
    rec_cache: list[dict] = []
    rec_cache_ts = 0.0
    rec_cache_ttl_s = 5.0

    # -- Dashboard -----------------------------------------------------------

    @app.route("/")
    def index():
        return Response(_DASHBOARD_HTML, content_type="text/html")

    # -- MJPEG streams -------------------------------------------------------

    def _mjpeg_generator(get_jpeg_fn, fps: float = 10.0, stream_name: str | None = None):
        interval = 1.0 / fps
        try:
            if stream_name is not None and hasattr(recorder, "set_stream_client_connected"):
                recorder.set_stream_client_connected(stream_name, True)
            while True:
                jpeg = get_jpeg_fn()
                frame = jpeg if jpeg else placeholder
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
                time.sleep(interval)
        finally:
            if stream_name is not None and hasattr(recorder, "set_stream_client_connected"):
                recorder.set_stream_client_connected(stream_name, False)

    @app.route("/stream/rgb")
    def stream_rgb():
        return Response(
            _mjpeg_generator(
                recorder.get_latest_annotated_jpeg,
                fps=max(0.5, float(getattr(config, "rgb_stream_fps", 6.0))),
                stream_name="rgb",
            ),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/stream/depth")
    def stream_depth():
        return Response(
            _mjpeg_generator(
                recorder.get_latest_depth_jpeg,
                fps=max(0.5, float(getattr(config, "depth_stream_fps", 3.0))),
                stream_name="depth",
            ),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    # -- Telemetry SSE -------------------------------------------------------

    def _finite_or_none(v, ndigits=3):
        """Round a float for JSON; replace inf/nan with None."""
        if v is None:
            return None
        import math
        if not math.isfinite(v):
            return None
        return round(v, ndigits)

    def _sse_generator():
        while True:
            t = recorder.get_latest_telemetry()
            if t is not None:
                obj = {
                    "mode": t.mode,
                    "throttle_scale": _finite_or_none(t.throttle_scale, 3),
                    "obstacle_distance_m": _finite_or_none(t.obstacle_distance_m, 3),
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
                    "gps_lat": round(t.gps_lat, 7) if t.gps_lat is not None else None,
                    "gps_lon": round(t.gps_lon, 7) if t.gps_lon is not None else None,
                    "gps_alt_m": round(t.gps_alt_m, 1) if t.gps_alt_m is not None else None,
                    "gps_fix": t.gps_fix,
                    "gps_sats": t.gps_sats,
                    "gps_hdop": round(t.gps_hdop, 2) if t.gps_hdop is not None else None,
                    "gps_diff_age_s": round(t.gps_diff_age_s, 2) if t.gps_diff_age_s is not None else None,
                    "gps_station_id": t.gps_station_id,
                    "imu_heading_deg": round(t.imu_heading_deg, 1) if t.imu_heading_deg is not None else None,
                }
                if oak_reader is not None:
                    try:
                        get_health = getattr(oak_reader, "get_health", None)
                        if callable(get_health):
                            obj["camera_health"] = get_health()
                    except Exception:
                        obj["camera_health"] = None
                yield f"data: {json.dumps(obj)}\n\n"
            telemetry_hz = max(0.5, float(getattr(config, "telemetry_hz", 4.0)))
            time.sleep(1.0 / telemetry_hz)

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
        nonlocal rec_cache_ts, rec_cache
        now = time.monotonic()
        with rec_cache_lock:
            if now - rec_cache_ts >= rec_cache_ttl_s:
                rec_dir = recorder.recordings_dir
                sessions = []
                if rec_dir.exists():
                    for d in sorted(rec_dir.iterdir(), reverse=True):
                        if not d.is_dir():
                            continue
                        files = sorted(f.name for f in d.iterdir() if f.is_file())
                        if files:
                            sessions.append({"name": d.name, "files": files})
                rec_cache = sessions
                rec_cache_ts = now
            payload = deepcopy(rec_cache)
        return Response(json.dumps(payload), content_type="application/json")

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

    def __init__(self, config: OakWebViewerConfig, recorder, controller=None,
                 oak_reader=None, motor_driver=None, imu_reader=None) -> None:
        self._config = config
        self._recorder = recorder
        self._controller = controller
        self._oak_reader = oak_reader
        self._motor_driver = motor_driver
        self._imu_reader = imu_reader
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
            app = create_app(
                self._recorder,
                self._config,
                controller=self._controller,
                oak_reader=self._oak_reader,
            )

            try:
                from pi_app.web.calibration_wizard import (
                    CalibrationManager, create_calibration_blueprint,
                )
                cal_mgr = CalibrationManager(
                    controller=self._controller,
                    oak_reader=self._oak_reader,
                    motor_driver=self._motor_driver,
                    imu_reader=self._imu_reader,
                )
                bp = create_calibration_blueprint(cal_mgr)
                app.register_blueprint(bp)
                logger.info("Calibration wizard registered at /calibrate")
            except Exception:
                logger.exception("Calibration wizard failed to load — skipping")

            try:
                from pi_app.web.property_map import create_map_blueprint
                map_bp = create_map_blueprint(
                    recorder=self._recorder,
                    controller=self._controller,
                )
                app.register_blueprint(map_bp)
                logger.info("Property map registered at /map")
            except Exception:
                logger.exception("Property map failed to load — skipping")

            app.run(
                host=self._config.host,
                port=self._config.port,
                threaded=True,
                use_reloader=False,
            )
        except Exception:
            logger.exception("Web viewer failed to start")
