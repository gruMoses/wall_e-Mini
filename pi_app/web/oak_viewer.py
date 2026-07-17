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
    from flask import Flask, Response, send_from_directory, abort, request
except ImportError:
    Flask = None  # type: ignore[assignment,misc]

import os
import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import OakWebViewerConfig


# ---------------------------------------------------------------------------
# UPS status bridge (read side)
# ---------------------------------------------------------------------------
#
# The UPSPlus HAT is watched by a *separate* process (scripts/upsPlus_power_daemon.py)
# that has the I2C bus; WALL-E's control loop never touches it. That daemon
# atomically publishes a small JSON snapshot to UPS_STATUS_FILE each poll; this
# reader lets the /debug board show live UPS state without giving the web app
# any hardware access. Both sides read the same env var so they stay in sync.

UPS_STATUS_FILE = os.environ.get("UPS_STATUS_FILE", "/tmp/ups_status.json")
UPS_STATUS_MAX_AGE_S = 10.0


def read_ups_status(path: str = UPS_STATUS_FILE,
                    max_age_s: float = UPS_STATUS_MAX_AGE_S,
                    now: float | None = None) -> dict:
    """Read the UPS status JSON published by the power daemon.

    Always returns a dict with at least ``available`` and ``stale`` bools —
    a missing file, unreadable path, corrupt/undecodable JSON, or a non-dict
    payload all yield ``{"available": False, ...}`` instead of raising, so the
    /api/ups route can never 500 on a bad/absent file. A readable file whose
    ``ts`` is older than ``max_age_s`` (or has no usable ``ts``) is reported
    ``available=True, stale=True`` so the page can grey out last-known values
    rather than hide them.
    """
    now = time.time() if now is None else now
    try:
        with open(path, "r") as f:
            raw = f.read()
    except (FileNotFoundError, NotADirectoryError, IsADirectoryError, OSError):
        return {"available": False, "stale": True, "age_s": None, "reason": "missing"}
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        return {"available": False, "stale": True, "age_s": None, "reason": "corrupt"}
    if not isinstance(data, dict):
        return {"available": False, "stale": True, "age_s": None, "reason": "corrupt"}

    ts = data.get("ts")
    age_s = None
    if isinstance(ts, (int, float)):
        age_s = max(0.0, float(now) - float(ts))
    stale = (age_s is None) or (age_s > max_age_s)
    return {
        "available": True,
        "stale": bool(stale),
        "age_s": round(age_s, 2) if age_s is not None else None,
        "ts": ts,
        "ac_present": data.get("ac_present"),
        "typec_mv": data.get("typec_mv"),
        "microusb_mv": data.get("microusb_mv"),
        "batt_v": data.get("batt_v"),
        "batt_ma": data.get("batt_ma"),
        "protect_mv": data.get("protect_mv"),
        # Battery/protect fields refresh at a slower cadence than the whole file
        # and are suppressed entirely during power-transfer windows (the daemon
        # minimizes I2C to the UPS MCU then). ts_batt is when they were last
        # actually read; batt_stale flags that they are last-known, not fresh —
        # the page greys them so the owner isn't misled during an outage.
        "ts_batt": data.get("ts_batt"),
        "batt_stale": data.get("batt_stale"),
        "detect_only": data.get("detect_only"),
        "seconds_without_charge": data.get("seconds_without_charge"),
    }


# ---------------------------------------------------------------------------
# Pi CPU temperature (sysfs) — debug board only, never on the safety path
# ---------------------------------------------------------------------------

_PI_CPU_THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"
_pi_cpu_temp_cache: tuple[float, float | None] | None = None  # (mono_ts, °C|None)
_PI_CPU_TEMP_CACHE_S = 1.0


def read_pi_cpu_temp_c(now: float | None = None,
                       path: str = _PI_CPU_THERMAL_PATH) -> float | None:
    """Read Raspberry Pi SoC temperature from thermal_zone0 (°C).

    Cached for ``_PI_CPU_TEMP_CACHE_S`` so a 4 Hz SSE client does not thrash
    sysfs. Returns None on any OS/parse error (desktop dev machines, missing
    thermal zone). Pure diagnostic — never used for control decisions.
    """
    global _pi_cpu_temp_cache
    mono = time.monotonic() if now is None else now
    if _pi_cpu_temp_cache is not None:
        cached_ts, cached_val = _pi_cpu_temp_cache
        if (mono - cached_ts) < _PI_CPU_TEMP_CACHE_S:
            return cached_val
    val: float | None = None
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
        milli = int(raw)
        # thermal_zone reports millidegrees C on Linux; reject nonsense.
        # Typical Pi idle ~40–55 °C; 125 °C is the SoC hard limit band.
        c = milli / 1000.0
        if -40.0 <= c <= 125.0:
            val = round(c, 1)
        # else: leave val None (out-of-range / garbage sysfs)
    except (OSError, ValueError, TypeError):
        val = None
    _pi_cpu_temp_cache = (mono, val)
    return val


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
  .rtk-panel { background: #1a1d27; border-radius: 8px; padding: 12px; margin-bottom: 16px; }
  .rtk-panel h2 { font-size: 13px; color: #bbb; margin-bottom: 10px; letter-spacing: 0.5px; text-transform: uppercase; }
  .rtk-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 8px; }
  .rtk-card { background: #161923; border: 1px solid #22252f; border-radius: 8px; padding: 10px; }
  .rtk-card .label { font-size: 11px; color: #7f8798; text-transform: uppercase; letter-spacing: 0.4px; margin-bottom: 4px; }
  .rtk-card .value { font-size: 20px; font-weight: 600; font-variant-numeric: tabular-nums; }
  .rtk-card .value.small { font-size: 15px; font-weight: 500; }
  .rtk-note { margin-top: 8px; font-size: 12px; color: #7f8798; }
  .recordings { background: #1a1d27; border-radius: 8px; padding: 16px; }
  .recordings h2 { font-size: 14px; margin-bottom: 12px; color: #ccc; }
  .teleop { background: #1a1d27; border-radius: 8px; padding: 14px; margin-bottom: 16px; }
  .teleop h2 { font-size: 14px; margin-bottom: 10px; color: #ccc; }
  .teleop-hint { margin-top: 8px; font-size: 11px; color: #666; }
  .teleop-hint a { color: #4a9eff; }
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
  /* ── Follow-Me Status Panel ─────────────────────────────────────── */
  .fm-panel { background: #1a1d27; border-radius: 8px; padding: 16px; margin-bottom: 16px; }
  .fm-panel h2 { font-size: 14px; color: #ccc; margin-bottom: 12px; display: flex;
                  align-items: center; gap: 10px; }
  .fm-badge { display: inline-block; padding: 4px 14px; border-radius: 12px; font-size: 12px;
              font-weight: 700; text-transform: uppercase; letter-spacing: 0.5px;
              background: #2a2d37; color: #888; transition: all 0.3s; }
  .fm-badge-green { background: #4ecdc4; color: #111; }
  .fm-badge-blue { background: #4a9eff; color: #fff; }
  .fm-badge-orange { background: #e6a239; color: #111; }
  .fm-badge-yellow { background: #f0e68c; color: #111; }
  .fm-badge-red { background: #e63946; color: #fff; }
  .fm-body { display: grid; grid-template-columns: 258px 1fr; gap: 16px; margin-top: 12px; }
  .fm-trail-wrap { background: #1a1a2e; border-radius: 6px; padding: 4px; }
  .fm-trail-wrap canvas { display: block; }
  .fm-section { margin-bottom: 8px; }
  .fm-section-title { font-size: 11px; color: #666; text-transform: uppercase;
                      letter-spacing: 0.5px; margin-bottom: 4px; }
  .fm-trail-health { font-size: 13px; color: #aaa; margin: 4px 0; font-variant-numeric: tabular-nums; }
  .fm-trouble-grid { display: grid; grid-template-columns: auto 1fr; gap: 2px 12px; font-size: 13px;
                     font-variant-numeric: tabular-nums; }
  .fm-tl { color: #888; }
  .fm-tv { color: #e0e0e0; font-weight: 600; }
  .fm-target-info { font-size: 13px; color: #aaa; margin: 4px 0; font-variant-numeric: tabular-nums; }
  .fm-confirmation { padding: 12px 16px; border-radius: 6px; font-size: 15px; font-weight: 600;
                     text-align: center; margin-top: 12px; background: transparent; transition: all 0.3s; }
  .fm-reset-btn { background: none; border: 1px solid #444; color: #888; font-size: 10px;
                  padding: 1px 6px; border-radius: 4px; cursor: pointer; margin-left: 6px; }
  .fm-reset-btn:hover { border-color: #888; color: #ccc; }
  @media (max-width: 700px) { .streams { grid-template-columns: 1fr; }
    .tracking-details { flex-direction: column; }
    .fm-body { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>WALL-E Mini — OAK-D Live</h1>
  <div class="hdr-right">
    <a href="/map" class="nav-link">Map</a>
    <a href="/navigate" class="nav-link">Navigate</a>
    <a href="/calibrate" class="nav-link">Calibrate</a>
    <a href="/debug" class="nav-link">Debug</a>
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
  <div class="teleop">
    <h2>Web Remote Drive (Manual Mode)</h2>
    <div class="teleop-hint">
      Manual drive moved to the fail-safe session at <a href="/drive">/drive</a>
      (deadman watchdog, single-driver lock, e-stop, speed caps). The old
      unauthenticated W/A/S/D controls here have been retired.
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
      <div class="label">Heading</div>
      <div class="value" id="t-imu-heading">—</div>
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
    <div class="telem-card">
      <div class="label">Battery SOC</div>
      <div class="value" id="t-bms-soc">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Battery V / A</div>
      <div class="value" id="t-bms-va">—</div>
    </div>
    <div class="telem-card">
      <div class="label">BMS Status</div>
      <div class="value" id="t-bms-status">—</div>
    </div>
    <div class="telem-card">
      <div class="label">Cell Delta</div>
      <div class="value" id="t-bms-delta">—</div>
    </div>
  </div>
  <div class="rtk-panel">
    <h2>RTK Diagnostics</h2>
    <div class="rtk-grid">
      <div class="rtk-card">
        <div class="label">RTK State</div>
        <div class="value" id="t-rtk-state">—</div>
      </div>
      <div class="rtk-card">
        <div class="label">Correction Age</div>
        <div class="value" id="t-rtk-corr-age">—</div>
      </div>
      <div class="rtk-card">
        <div class="label">Base Station</div>
        <div class="value small" id="t-rtk-base">—</div>
      </div>
      <div class="rtk-card">
        <div class="label">Float Duration</div>
        <div class="value" id="t-rtk-float-age">—</div>
      </div>
      <div class="rtk-card">
        <div class="label">Fix Duration</div>
        <div class="value" id="t-rtk-fix-age">—</div>
      </div>
      <div class="rtk-card">
        <div class="label">Quality Hint</div>
        <div class="value small" id="t-rtk-quality">—</div>
      </div>
    </div>
    <div class="rtk-note" id="t-rtk-note">Looking for stable corrections...</div>
  </div>
  <div id="fm-panel" class="fm-panel">
    <h2>Follow-Me Status <span id="fm-mode-badge" class="fm-badge">&mdash;</span></h2>
    <div class="fm-section">
      <div class="fm-section-title">Trail Health</div>
      <div id="fm-trail-health" class="fm-trail-health">&mdash;</div>
    </div>
    <div class="fm-body">
      <div class="fm-trail-wrap">
        <canvas id="trail-canvas" width="250" height="250"></canvas>
      </div>
      <div class="fm-info">
        <div class="fm-section">
          <div class="fm-section-title">Troubleshooting</div>
          <div class="fm-trouble-grid">
            <span class="fm-tl">Rejected jumps:</span>
            <span class="fm-tv"><span id="fm-rejected-jumps">&mdash;</span><button class="fm-reset-btn" onclick="resetCounter('jumps')">reset</button></span>
            <span class="fm-tl">Rejected speeds:</span>
            <span class="fm-tv"><span id="fm-rejected-speeds">&mdash;</span><button class="fm-reset-btn" onclick="resetCounter('speeds')">reset</button></span>
            <span class="fm-tl">Hysteresis:</span>
            <span class="fm-tv" id="fm-hysteresis">&mdash;</span>
            <span class="fm-tl">Extrapolation:</span>
            <span class="fm-tv" id="fm-extrapolation">&mdash;</span>
            <span class="fm-tl">Curvature &#127919;:</span>
            <span class="fm-tv" id="fm-curvature">&mdash;</span>
          </div>
        </div>
        <div class="fm-section">
          <div class="fm-section-title">Target</div>
          <div id="fm-target-info" class="fm-target-info">&mdash;</div>
        </div>
      </div>
    </div>
    <div id="fm-confirmation" class="fm-confirmation">&mdash;</div>
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
let rtkModeName = null;
let rtkModeSinceMs = Date.now();

function formatDuration(sec) {
  if (!(sec >= 0)) return '—';
  if (sec < 60) return sec.toFixed(0) + 's';
  const m = Math.floor(sec / 60);
  const s = Math.floor(sec % 60);
  if (m < 60) return m + 'm ' + String(s).padStart(2, '0') + 's';
  const h = Math.floor(m / 60);
  const mm = m % 60;
  return h + 'h ' + String(mm).padStart(2, '0') + 'm';
}

function updateRtkPanel(d) {
  const fix = d.gps_fix;
  const corrAge = d.gps_diff_age_s;
  const sats = d.gps_sats;
  const hdop = d.gps_hdop;
  const station = d.gps_station_id;
  const state = (fix === 4) ? 'RTK FIXED' : (fix === 5) ? 'RTK FLOAT' : (fix >= 1) ? 'GPS ONLY' : 'NO FIX';
  const nowMs = Date.now();
  if (rtkModeName !== state) {
    rtkModeName = state;
    rtkModeSinceMs = nowMs;
  }
  const modeAgeSec = (nowMs - rtkModeSinceMs) / 1000.0;
  const stateEl = document.getElementById('t-rtk-state');
  stateEl.textContent = state;
  stateEl.className = 'value ' + (
    state === 'RTK FIXED' ? 'green' :
    state === 'RTK FLOAT' ? 'yellow' :
    state === 'GPS ONLY' ? 'blue' : 'red'
  );
  const corrEl = document.getElementById('t-rtk-corr-age');
  if (corrAge != null) {
    corrEl.textContent = corrAge.toFixed(1) + 's';
    corrEl.className = 'value ' + (corrAge < 2 ? 'green' : corrAge < 5 ? 'yellow' : 'red');
  } else {
    corrEl.textContent = '—';
    corrEl.className = 'value';
  }
  document.getElementById('t-rtk-base').textContent = station ? ('#' + station) : 'none';
  document.getElementById('t-rtk-float-age').textContent = (state === 'RTK FLOAT') ? formatDuration(modeAgeSec) : '0s';
  document.getElementById('t-rtk-fix-age').textContent = (state === 'RTK FIXED') ? formatDuration(modeAgeSec) : '0s';
  const qEl = document.getElementById('t-rtk-quality');
  const noteEl = document.getElementById('t-rtk-note');
  if (state === 'RTK FIXED' && corrAge != null && corrAge < 2) {
    qEl.textContent = 'Locked';
    qEl.className = 'value small green';
    noteEl.textContent = 'Integer ambiguity solved. This is your highest-quality state.';
  } else if (state === 'RTK FLOAT' && corrAge != null && corrAge < 2) {
    qEl.textContent = 'Corrections OK';
    qEl.className = 'value small yellow';
    noteEl.textContent = 'Receiving corrections, but ambiguity is unresolved (float).';
  } else if (state === 'GPS ONLY') {
    qEl.textContent = 'No RTK';
    qEl.className = 'value small blue';
    noteEl.textContent = 'Rover has GNSS fix but no RTK solution.';
  } else {
    qEl.textContent = 'Degraded';
    qEl.className = 'value small red';
    noteEl.textContent = 'Correction stream missing/stale or rover has no valid fix.';
  }
  if (sats != null && hdop != null) {
    noteEl.textContent += ' Sats=' + sats + ', HDOP=' + hdop.toFixed(2) + '.';
  }
}

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
  document.getElementById('t-imu-heading').textContent =
    (d.corrected_heading_deg != null && d.heading_offset_locked && d.imu_heading_deg != null)
      ? d.imu_heading_deg.toFixed(1) + '° → ' + d.corrected_heading_deg.toFixed(1) + '°'
        + (d.heading_offset_frozen ? ' (frozen)' : '')
      : (d.imu_heading_deg != null ? d.imu_heading_deg.toFixed(1) + '°' : '—');
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
  updateRtkPanel(d);
  const cam = d.camera_health || {};
  const pipeEl = document.getElementById('t-cam-pipeline');
  const rgbAgeEl = document.getElementById('t-cam-rgb-age');
  const depthAgeEl = document.getElementById('t-cam-depth-age');
  if (cam.pipeline_running === true) {
    if (cam.critical_stale) {
      pipeEl.textContent = 'STALE';
      pipeEl.className = 'value yellow';
    } else if (cam.is_stale) {
      pipeEl.textContent = 'DEGRADED';
      pipeEl.className = 'value yellow';
    } else {
      pipeEl.textContent = 'RUNNING';
      pipeEl.className = 'value green';
    }
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
  const socEl = document.getElementById('t-bms-soc');
  const vaEl = document.getElementById('t-bms-va');
  const statusEl = document.getElementById('t-bms-status');
  const deltaEl = document.getElementById('t-bms-delta');
  const bmsConnected = d.bms_connected === true;
  const bmsSoc = d.bms_soc_pct;
  if (bmsConnected && bmsSoc != null) {
    socEl.textContent = bmsSoc.toFixed(1) + '%';
    socEl.className = 'value ' + (bmsSoc >= 40 ? 'green' : bmsSoc >= 20 ? 'yellow' : 'red');
  } else {
    socEl.textContent = '—';
    socEl.className = 'value';
  }
  if (bmsConnected && d.bms_voltage_v != null) {
    const aStr = d.bms_current_a != null ? d.bms_current_a.toFixed(1) + 'A' : '—';
    vaEl.textContent = d.bms_voltage_v.toFixed(2) + 'V / ' + aStr;
    vaEl.className = 'value';
  } else {
    vaEl.textContent = '—';
    vaEl.className = 'value';
  }
  if (!bmsConnected) {
    statusEl.textContent = 'OFFLINE';
    statusEl.className = 'value red';
  } else if (d.bms_charging === true) {
    statusEl.textContent = 'CHARGING';
    statusEl.className = 'value blue';
  } else if (d.bms_discharge_fet_on === false) {
    statusEl.textContent = 'DISCHARGE OFF';
    statusEl.className = 'value yellow';
  } else {
    statusEl.textContent = 'ONLINE';
    statusEl.className = 'value green';
  }
  if (bmsConnected && d.bms_cell_delta_mv != null) {
    const dv = d.bms_cell_delta_mv;
    deltaEl.textContent = dv + 'mV';
    deltaEl.className = 'value ' + (dv <= 20 ? 'green' : dv <= 40 ? 'yellow' : 'red');
  } else {
    deltaEl.textContent = '—';
    deltaEl.className = 'value';
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
  if ((d.detections || []).length > 0) {
    const trackList = d.detections.map(function(det) {
      return 'id:' + (det.track_id != null ? det.track_id : '—') + ' @ ' + det.z_m.toFixed(1) + 'm';
    }).join(', ');
    statsHtml += 'Tracks: <span class="stat-val">' + trackList + '</span><br>';
  }
  statsHtml += 'Mode: <span class="stat-val">' + d.mode + '</span>';
  if (isFollowMode) statsHtml += '<br>Motors: <span class="stat-val">L=' + d.motor_left + ' R=' + d.motor_right + '</span>';
  stats.innerHTML = statsHtml;
  drawRadar(d.detections, d.follow_target_x_m, d.follow_target_z_m, tracking, d.mode);
  try { updateFollowPanel(d); } catch(e) {}
  try { drawTrailCanvas(d); } catch(e) {}
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

/* ── Follow-Me Status Panel + Trail Canvas ───────────────────────── */
var trailCanvas = document.getElementById('trail-canvas');
var trailCtx = trailCanvas ? trailCanvas.getContext('2d') : null;

function updateFollowPanel(d) {
  try {
    var fm = d.follow_mode || '';
    var tracking = d.follow_tracking;
    var badge = document.getElementById('fm-mode-badge');
    var modeText = '\u2014', badgeCls = '';
    if (fm === 'DIRECT_PID') { modeText = 'DIRECT PID'; badgeCls = 'fm-badge-green'; }
    else if (fm === 'TRAIL_PURSUIT') { modeText = 'TRAIL PURSUIT'; badgeCls = 'fm-badge-blue'; }
    else if (fm === 'LOST_BLIND_TRAIL') { modeText = 'LOST \u2014 BLIND TRAIL'; badgeCls = 'fm-badge-orange'; }
    else if (fm === 'SEARCH_ROTATE') { modeText = 'SEARCH ROTATE'; badgeCls = 'fm-badge-yellow'; }
    else if (fm === 'TRAIL_EXHAUSTED') { modeText = 'TRAIL EXHAUSTED'; badgeCls = 'fm-badge-red'; }
    else if (fm === 'IDLE') { modeText = 'IDLE'; }
    else if (fm) { modeText = fm.replace(/_/g, ' '); }
    badge.textContent = modeText;
    badge.className = 'fm-badge ' + badgeCls;

    /* trail health */
    var ptCount = d.trail_length != null ? d.trail_length : '\u2014';
    var remaining = d.trail_distance_m != null ? d.trail_distance_m.toFixed(1) : '\u2014';
    var laM = '\u2014';
    var la = d.lookahead_point_xy;
    var rx = d.robot_pose_x, ry = d.robot_pose_y;
    if (la && rx != null) {
      var dx = la[0] - rx, dy = la[1] - (ry || 0);
      laM = Math.sqrt(dx * dx + dy * dy).toFixed(2);
    }
    document.getElementById('fm-trail-health').textContent =
      ptCount + ' pts \u2022 ' + remaining + ' m remaining \u2022 lookahead ' + laM + ' m';

    /* troubleshooting */
    document.getElementById('fm-rejected-jumps').textContent =
      d.trail_rejected_jump_count != null ? d.trail_rejected_jump_count : '\u2014';
    document.getElementById('fm-rejected-speeds').textContent =
      d.trail_rejected_speed_count != null ? d.trail_rejected_speed_count : '\u2014';
    var hc = d.hysteresis_count || 0, hm = d.hysteresis_max || 3;
    var hystEl = document.getElementById('fm-hysteresis');
    hystEl.textContent = hc + '/' + hm;
    hystEl.style.color = hc >= hm ? '#e63946' : '#e0e0e0';
    var ea = d.extrapolation_active, ec = d.extrapolation_count || 0;
    document.getElementById('fm-extrapolation').textContent =
      ea ? '\u2713 +' + ec + ' virtual points' : '\u2717';
    document.getElementById('fm-curvature').textContent =
      d.trail_curvature_at_lookahead != null ? d.trail_curvature_at_lookahead.toFixed(4) : '\u2014';

    /* target info */
    var dist = d.follow_target_z_m, off = d.target_lateral_offset;
    var conf = d.target_confidence, tid = d.target_track_id;
    document.getElementById('fm-target-info').textContent =
      'Distance: ' + (dist != null ? dist.toFixed(2) + 'm' : '\u2014') + ' | ' +
      'Offset: ' + (off != null ? off.toFixed(2) : '\u2014') + ' | ' +
      'Confidence: ' + (conf != null ? (conf * 100).toFixed(0) + '%' : '\u2014') + ' | ' +
      'Track: ' + (tid != null ? '#' + tid : '\u2014');

    /* confirmation indicator */
    var confEl = document.getElementById('fm-confirmation');
    if (fm === 'DIRECT_PID' && tracking) {
      confEl.innerHTML = '<span style="color:#4ecdc4;">\u2705 TRACKING + TRAIL ACTIVE</span>';
      confEl.style.background = 'rgba(78,205,196,0.1)';
    } else if (fm === 'TRAIL_PURSUIT' || fm === 'LOST_BLIND_TRAIL' || fm === 'SEARCH_ROTATE') {
      confEl.innerHTML = '<span style="color:#e6a239;">\u26a0\ufe0f Target lost \u2014 following trail</span>';
      confEl.style.background = 'rgba(230,162,57,0.1)';
    } else if (fm === 'TRAIL_EXHAUSTED') {
      confEl.innerHTML = '<span style="color:#e63946;">\u274c Trail exhausted \u2014 stopping</span>';
      confEl.style.background = 'rgba(230,57,70,0.1)';
    } else if (d.mode === 'FOLLOW_ME' && tracking) {
      confEl.innerHTML = '<span style="color:#4ecdc4;">\u2705 TRACKING</span>';
      confEl.style.background = 'rgba(78,205,196,0.1)';
    } else {
      confEl.innerHTML = '<span style="color:#555;">\u2014 Follow Me inactive</span>';
      confEl.style.background = 'transparent';
    }
  } catch(e) {}
}

function drawTrailCanvas(d) {
  if (!trailCtx) return;
  try {
    var W = trailCanvas.width, H = trailCanvas.height;
    trailCtx.fillStyle = '#1a1a2e'; trailCtx.fillRect(0, 0, W, H);
    var pts = d.trail_points_xy || [];
    var ox = d.robot_pose_x, oy = d.robot_pose_y, otheta = d.robot_pose_theta;
    if (ox == null || oy == null) {
      trailCtx.fillStyle = '#555'; trailCtx.font = '12px sans-serif';
      trailCtx.textAlign = 'center'; trailCtx.fillText('No odometry', W / 2, H / 2);
      trailCtx.textAlign = 'start'; return;
    }
    var allX = [ox], allY = [oy];
    for (var i = 0; i < pts.length; i++) { allX.push(pts[i][0]); allY.push(pts[i][1]); }
    var la = d.lookahead_point_xy;
    if (la) { allX.push(la[0]); allY.push(la[1]); }
    var minX = Math.min.apply(null, allX), maxX = Math.max.apply(null, allX);
    var minY = Math.min.apply(null, allY), maxY = Math.max.apply(null, allY);
    var pad = 0.8; minX -= pad; maxX += pad; minY -= pad; maxY += pad;
    var rX = maxX - minX || 1, rY = maxY - minY || 1;
    var sc = Math.min(W / rX, H / rY);
    var oXpx = (W - rX * sc) / 2, oYpx = (H - rY * sc) / 2;
    function tx(x) { return (x - minX) * sc + oXpx; }
    function ty(y) { return H - ((y - minY) * sc + oYpx); }

    /* grid */
    trailCtx.strokeStyle = '#252547'; trailCtx.lineWidth = 0.5;
    var gs = (rX > 10 || rY > 10) ? 2.0 : 1.0;
    for (var gx = Math.ceil(minX / gs) * gs; gx <= maxX; gx += gs) {
      var px = tx(gx);
      trailCtx.beginPath(); trailCtx.moveTo(px, 0); trailCtx.lineTo(px, H); trailCtx.stroke();
    }
    for (var gy = Math.ceil(minY / gs) * gs; gy <= maxY; gy += gs) {
      var py = ty(gy);
      trailCtx.beginPath(); trailCtx.moveTo(0, py); trailCtx.lineTo(W, py); trailCtx.stroke();
    }

    /* consume radius */
    var cr = d.consume_radius_m;
    if (cr) {
      trailCtx.beginPath();
      trailCtx.arc(tx(ox), ty(oy), cr * sc, 0, Math.PI * 2);
      trailCtx.strokeStyle = 'rgba(230,57,70,0.3)'; trailCtx.lineWidth = 1; trailCtx.stroke();
    }

    /* trail line + dots */
    if (pts.length > 1) {
      trailCtx.beginPath();
      trailCtx.moveTo(tx(pts[0][0]), ty(pts[0][1]));
      for (var i = 1; i < pts.length; i++) trailCtx.lineTo(tx(pts[i][0]), ty(pts[i][1]));
      trailCtx.strokeStyle = '#4ecdc4'; trailCtx.lineWidth = 2; trailCtx.stroke();
    }
    for (var i = 0; i < pts.length; i++) {
      trailCtx.beginPath();
      trailCtx.arc(tx(pts[i][0]), ty(pts[i][1]), 3, 0, Math.PI * 2);
      trailCtx.fillStyle = '#4ecdc4'; trailCtx.fill();
    }

    /* lookahead carrot */
    if (la) {
      trailCtx.beginPath();
      trailCtx.arc(tx(la[0]), ty(la[1]), 6, 0, Math.PI * 2);
      trailCtx.fillStyle = '#e6a239'; trailCtx.fill();
      trailCtx.strokeStyle = '#fff'; trailCtx.lineWidth = 1.5; trailCtx.stroke();
    }

    /* robot arrow */
    var rpx = tx(ox), rpy = ty(oy);
    var theta = otheta != null ? -otheta : 0;
    trailCtx.save(); trailCtx.translate(rpx, rpy);
    trailCtx.rotate(theta - Math.PI / 2);
    trailCtx.beginPath();
    trailCtx.moveTo(0, -10); trailCtx.lineTo(-6, 6); trailCtx.lineTo(6, 6);
    trailCtx.closePath(); trailCtx.fillStyle = '#fff'; trailCtx.fill();
    trailCtx.restore();

    /* scale bar */
    trailCtx.fillStyle = '#555'; trailCtx.font = '10px monospace';
    var scaleM = Math.min(rX, rY) * 0.25;
    var barPx = scaleM * sc;
    trailCtx.fillRect(10, H - 16, barPx, 2);
    trailCtx.fillText(scaleM.toFixed(1) + 'm', 10, H - 20);
  } catch(e) {}
}

function resetCounter(type) {
  fetch('/api/follow_me/reset_counter', {method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({type: type})}).catch(function(){});
}
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Debug status board (self-contained, no external assets) — READ ONLY.
#
# Bench/diagnostic page: subscribes to the existing /api/telemetry SSE and
# polls /api/ups, rendering every safety + power status as a big pill that
# visibly flips as the owner physically toggles switches / pulls plugs. There
# are NO control elements here — nothing on this page can command the robot.
# ---------------------------------------------------------------------------

_DEBUG_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WALL-E Mini — Debug Status Board</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
         background: #0f1117; color: #e0e0e0; padding-bottom: 40px; }
  header { background: #1a1d27; padding: 12px 24px; display: flex; align-items: center;
           justify-content: space-between; border-bottom: 1px solid #2a2d37;
           position: sticky; top: 0; z-index: 10; }
  header h1 { font-size: 17px; font-weight: 600; color: #fff; }
  header .sub { font-size: 11px; color: #7f8798; margin-top: 2px; }
  header .nav-link { color: #7aa2f7; text-decoration: none; font-size: 13px; font-weight: 500; }
  header .nav-link:hover { text-decoration: underline; }
  .hdr-right { display: flex; align-items: center; gap: 14px; }
  .conn { font-size: 11px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.5px;
          padding: 3px 10px; border-radius: 12px; background: #2a2d37; color: #888; }
  .conn.live { background: #143d33; color: #4ecdc4; }
  .conn.dead { background: #3d1a1f; color: #e63946; }
  .container { max-width: 1100px; margin: 0 auto; padding: 16px; }
  .panel { background: #1a1d27; border-radius: 10px; padding: 16px; margin-bottom: 16px;
           border: 1px solid #22252f; }
  .panel h2 { font-size: 12px; color: #9aa2b1; margin-bottom: 12px; text-transform: uppercase;
              letter-spacing: 0.8px; font-weight: 700; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; }
  .cell { background: #12151d; border: 1px solid #22252f; border-radius: 8px; padding: 10px 12px; }
  .cell .k { font-size: 10px; color: #7f8798; text-transform: uppercase; letter-spacing: 0.5px;
             margin-bottom: 6px; }
  .pill { display: inline-block; font-size: 17px; font-weight: 800; letter-spacing: 0.3px;
          padding: 5px 12px; border-radius: 8px; font-variant-numeric: tabular-nums;
          background: #262a36; color: #c8ccd6; transition: background 0.15s, color 0.15s;
          min-width: 54px; text-align: center; }
  .pill.flash { animation: flashpill 0.9s ease-out; }
  @keyframes flashpill { 0% { box-shadow: 0 0 0 2px #fff; } 100% { box-shadow: 0 0 0 0 transparent; } }
  .pill.green { background: #143d33; color: #4ecdc4; }
  .pill.red   { background: #3d1a1f; color: #ff6b74; }
  .pill.amber { background: #3a2f15; color: #f0b344; }
  .pill.blue  { background: #14263d; color: #6bb6ff; }
  .pill.grey  { background: #262a36; color: #9aa2b1; }
  .val { font-size: 17px; font-weight: 700; font-variant-numeric: tabular-nums; color: #e0e0e0; }
  .val.green { color: #4ecdc4; } .val.red { color: #ff6b74; }
  .val.amber { color: #f0b344; } .val.blue { color: #6bb6ff; } .val.grey { color: #7f8798; }
  .panel.stale { opacity: 0.5; }
  .stale-tag { font-size: 10px; font-weight: 700; color: #f0b344; margin-left: 8px;
               text-transform: uppercase; letter-spacing: 0.5px; }
  .log { background: #0b0d13; border: 1px solid #22252f; border-radius: 8px; padding: 10px;
         font-family: 'SFMono-Regular', Menlo, Consolas, monospace; font-size: 12px;
         line-height: 1.7; height: 260px; overflow-y: auto; }
  .log .row { white-space: pre; }
  .log .ts { color: #5b6472; }
  .log .up { color: #4ecdc4; } .log .down { color: #ff6b74; } .log .neu { color: #f0b344; }
  .log .empty { color: #444; font-style: italic; }
  .hint { font-size: 11px; color: #5b6472; margin-top: 10px; }
  /* ── Temperature history ─────────────────────────────────────────── */
  .temp-toolbar { display: flex; flex-wrap: wrap; align-items: center;
                  justify-content: space-between; gap: 10px; margin-bottom: 10px; }
  .temp-legend { display: flex; flex-wrap: wrap; gap: 6px 10px; }
  .temp-chip { display: inline-flex; align-items: center; gap: 6px; cursor: pointer;
               user-select: none; background: #12151d; border: 1px solid #2a2d37;
               border-radius: 999px; padding: 5px 10px 5px 8px; font-size: 11px;
               color: #c8ccd6; transition: opacity 0.12s, border-color 0.12s; }
  .temp-chip:focus-visible { outline: 2px solid #6bb6ff; outline-offset: 2px; }
  .temp-chip.off { opacity: 0.45; }
  .temp-chip.stale { border-color: #8a7020; }
  .temp-chip .swatch { width: 10px; height: 10px; border-radius: 50%;
                       box-shadow: 0 0 0 1px rgba(255,255,255,0.12); flex-shrink: 0; }
  .temp-chip .cname { font-weight: 600; letter-spacing: 0.2px; }
  .temp-chip .cval { font-variant-numeric: tabular-nums; font-weight: 700;
                     color: #e8eaf0; min-width: 3.6em; text-align: right; }
  /* Secondary text: #8b93a3 on #12151d ≈ 5:1 contrast (was #5b6472, too dim). */
  .temp-chip .cmm { color: #8b93a3; font-variant-numeric: tabular-nums; font-size: 10px; }
  .temp-meta { font-size: 11px; color: #8b93a3; white-space: nowrap; }
  .temp-canvas-wrap { position: relative; width: 100%; height: 280px;
                      background: #0b0d13; border: 1px solid #22252f; border-radius: 8px;
                      overflow: hidden; }
  .temp-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
  .temp-empty { position: absolute; inset: 0; display: flex; align-items: center;
                justify-content: center; color: #8b93a3; font-size: 13px;
                pointer-events: none; }
  .temp-empty.hidden { display: none; }
  .temp-sr { position: absolute; width: 1px; height: 1px; padding: 0; margin: -1px;
             overflow: hidden; clip: rect(0,0,0,0); white-space: nowrap; border: 0; }
  @media (max-width: 640px) {
    .temp-canvas-wrap { height: 220px; }
    .temp-chip .cmm { display: none; }
  }
</style>
</head>
<body>
<header>
  <div>
    <h1>WALL-E Mini — Debug Status Board</h1>
    <div class="sub">Read-only bench diagnostics · flip a switch and watch it react</div>
  </div>
  <div class="hdr-right">
    <a href="/" class="nav-link">&larr; Dashboard</a>
    <span id="conn" class="conn">connecting</span>
  </div>
</header>
<div class="container">

  <div class="panel" id="p-safety">
    <h2>Safety</h2>
    <div class="grid">
      <div class="cell"><div class="k">Armed</div><span id="s-armed" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Emergency Stop</div><span id="s-estop" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Mode</div><span id="s-mode" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Charger Inhibit</div><span id="s-chgi" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Pack Low Latched</div><span id="s-packlow" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Throttle Scale</div><span id="s-throttle" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Obstacle Distance</div><span id="s-dist" class="val grey">&mdash;</span></div>
    </div>
  </div>

  <div class="panel" id="p-vesc">
    <h2>VESC / CAN</h2>
    <div class="grid">
      <div class="cell"><div class="k">Left RPM</div><span id="v-lrpm" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Right RPM</div><span id="v-rrpm" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Motor Bytes L / R</div><span id="v-bytes" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">RX Frames</div><span id="v-frames" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Frames / sec</div><span id="v-fps" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Last Frame Age</div><span id="v-age" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">RX Thread</div><span id="v-thread" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Left Status Age</div><span id="v-lage" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Right Status Age</div><span id="v-rage" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Parse / Recv / Reopen</div><span id="v-errs" class="val grey">&mdash;</span></div>
    </div>
  </div>

  <div class="panel" id="p-bms">
    <h2>BMS &mdash; Main Pack</h2>
    <div class="grid">
      <div class="cell"><div class="k">Connected</div><span id="b-conn" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Voltage</div><span id="b-volt" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Current</div><span id="b-curr" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">SOC</div><span id="b-soc" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Cell Delta</div><span id="b-delta" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Temp Min / Max</div><span id="b-temp" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Charging</div><span id="b-chg" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Discharge FET</div><span id="b-dfet" class="pill grey">&mdash;</span></div>
    </div>
  </div>

  <div class="panel" id="p-ups">
    <h2>UPS <span id="ups-tag" class="stale-tag"></span></h2>
    <div class="grid">
      <div class="cell"><div class="k">AC Power</div><span id="u-ac" class="pill grey">&mdash;</span></div>
      <div class="cell"><div class="k">Type-C In</div><span id="u-typec" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Micro-USB In</div><span id="u-microusb" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Battery Voltage</div><span id="u-battv" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Battery Current</div><span id="u-battma" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Protect Threshold</div><span id="u-protect" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Secs Without Charge</div><span id="u-swc" class="val grey">&mdash;</span></div>
      <div class="cell"><div class="k">Daemon Mode</div><span id="u-mode" class="pill grey">&mdash;</span></div>
    </div>
    <div class="hint" id="u-hint">Waiting for /tmp/ups_status.json from upsPlus_power_daemon.py &hellip;</div>
  </div>

  <div class="panel" id="p-temp">
    <h2>Temperature History <span style="font-weight:400;color:#8b93a3;">(°C · last 10 min)</span></h2>
    <div class="temp-toolbar">
      <div class="temp-legend" id="temp-legend" role="group" aria-label="Temperature series toggles"></div>
      <div class="temp-meta" id="temp-meta">waiting for samples&hellip;</div>
    </div>
    <div class="temp-canvas-wrap">
      <canvas id="temp-canvas" width="1000" height="280"
              role="img" aria-label="Temperature history graph"
              aria-describedby="temp-sr"></canvas>
      <div class="temp-empty" id="temp-empty">No temperature samples yet</div>
      <div class="temp-sr" id="temp-sr" aria-live="polite">No temperature samples yet</div>
    </div>
    <div class="hint">Click or press a series to toggle &middot; null / disconnected sensors leave gaps &middot;
      in-memory ring buffer (no disk) &middot; Pi CPU from thermal_zone0</div>
  </div>

  <div class="panel">
    <h2>Event Log <span style="font-weight:400;color:#5b6472;">(status transitions)</span></h2>
    <div class="log" id="log"><div class="row empty">Watching for changes&hellip;</div></div>
    <div class="hint">Newest at top &middot; last 50 transitions &middot; no journalctl required.</div>
  </div>

</div>
<script>
/* ── Event log ─────────────────────────────────────────────────────── */
var LOG_MAX = 50;
var logEl = document.getElementById('log');
var logRows = [];
var prev = {};   // last seen value per watched key
function nowStr() {
  var d = new Date();
  function p(n){ return String(n).padStart(2,'0'); }
  return p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds());
}
function esc(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
function logLine(label, oldV, newV, dir) {
  var cls = dir === 'up' ? 'up' : dir === 'down' ? 'down' : 'neu';
  var row = '<div class="row"><span class="ts">' + nowStr() + '</span>  ' +
            esc(label) + ': <span class="' + cls + '">' + esc(oldV) + ' → ' + esc(newV) +
            '</span></div>';
  logRows.unshift(row);
  if (logRows.length > LOG_MAX) logRows.length = LOG_MAX;
  logEl.innerHTML = logRows.join('');
}
/* Compare a watched value against its previous; log on change.
   dirFn(newV) -> 'up' | 'down' | 'neu' picks the color. */
function watch(key, label, value, fmt, dirFn) {
  if (value === undefined) return;
  var shown = fmt ? fmt(value) : String(value);
  if (!(key in prev)) { prev[key] = shown; return; }   // seed silently, no log
  if (prev[key] !== shown) {
    var dir = dirFn ? dirFn(value, prev[key]) : 'neu';
    logLine(label, prev[key], shown, dir);
    prev[key] = shown;
  }
}
var boolStr = function(v){ return v ? 'true' : 'false'; };

/* ── Pill / value helpers ──────────────────────────────────────────── */
function pill(id, text, cls) {
  var el = document.getElementById(id);
  if (!el) return;
  if (el.textContent !== text) {
    el.classList.remove('flash'); void el.offsetWidth; el.classList.add('flash');
  }
  el.textContent = text;
  el.className = 'pill ' + cls;
}
function val(id, text, cls) {
  var el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  el.className = 'val ' + (cls || 'grey');
}
function num(v, digits, suffix) {
  if (v === null || v === undefined) return '—';
  return v.toFixed(digits === undefined ? 0 : digits) + (suffix || '');
}

/* ── Telemetry SSE (robot safety + VESC + BMS) ─────────────────────── */
var connEl = document.getElementById('conn');
var lastFrameCount = null, lastFrameTs = null, fps = null;

var sse = new EventSource('/api/telemetry');
sse.onopen = function(){ connEl.textContent = 'live'; connEl.className = 'conn live'; };
sse.onerror = function(){ connEl.textContent = 'reconnecting'; connEl.className = 'conn dead'; };
sse.onmessage = function(e) {
  var d;
  try { d = JSON.parse(e.data); } catch(_) { return; }
  connEl.textContent = 'live'; connEl.className = 'conn live';

  /* ---- SAFETY ---- */
  pill('s-armed', d.is_armed ? 'ARMED' : 'DISARMED', d.is_armed ? 'green' : 'red');
  var es = d.emergency_active;
  pill('s-estop', es == null ? '—' : (es ? 'E-STOP' : 'clear'),
       es == null ? 'grey' : (es ? 'red' : 'green'));
  pill('s-mode', d.mode || '—', d.mode === 'FOLLOW_ME' ? 'amber' : 'blue');
  var ci = d.charger_inhibit;
  pill('s-chgi', ci == null ? '—' : (ci ? 'INHIBIT' : 'clear'),
       ci == null ? 'grey' : (ci ? 'red' : 'green'));
  var pl = d.vesc_pack_low_latched;
  pill('s-packlow', pl ? 'LATCHED' : 'clear', pl ? 'red' : 'green');
  var sc = d.throttle_scale;
  var scCls = sc == null ? 'grey' : sc >= 0.8 ? 'green' : sc >= 0.3 ? 'amber' : 'red';
  pill('s-throttle', sc == null ? '—' : sc.toFixed(2), scCls);
  var dist = d.obstacle_distance_m;
  val('s-dist', dist == null ? '—' : dist.toFixed(2) + ' m',
      dist == null ? 'grey' : dist < 0.5 ? 'red' : dist < 1.2 ? 'amber' : 'green');

  /* ---- VESC / CAN ---- */
  val('v-lrpm', d.vesc_left_rpm == null ? '—' : d.vesc_left_rpm,
      d.vesc_left_rpm == null ? 'grey' : '');
  val('v-rrpm', d.vesc_right_rpm == null ? '—' : d.vesc_right_rpm,
      d.vesc_right_rpm == null ? 'grey' : '');
  var mbl = d.motor_left, mbr = d.motor_right;
  val('v-bytes', (mbl == null || mbr == null) ? '—' : mbl + ' / ' + mbr,
      (mbl == null || mbr == null) ? 'grey' : '');
  var fc = d.vesc_rx_frame_count;
  val('v-frames', fc == null ? '—' : fc, '');
  var tnow = performance.now();
  if (fc != null && lastFrameCount != null && lastFrameTs != null) {
    var dt = (tnow - lastFrameTs) / 1000.0;
    if (dt > 0.05) { fps = Math.max(0, (fc - lastFrameCount) / dt); lastFrameCount = fc; lastFrameTs = tnow; }
  } else if (fc != null) { lastFrameCount = fc; lastFrameTs = tnow; }
  pill('v-fps', fps == null ? '—' : fps.toFixed(1),
       fps == null ? 'grey' : fps >= 20 ? 'green' : fps > 0 ? 'amber' : 'red');
  var age = d.vesc_rx_last_frame_age_s;
  pill('v-age', age == null ? '—' : age.toFixed(2) + 's',
       age == null ? 'grey' : age > 1.0 ? 'red' : age > 0.3 ? 'amber' : 'green');
  var th = d.vesc_rx_thread_alive;
  pill('v-thread', th == null ? 'n/a' : (th ? 'ALIVE' : 'DEAD'),
       th == null ? 'grey' : (th ? 'green' : 'red'));
  var la = d.vesc_left_status_age_s, ra = d.vesc_right_status_age_s;
  pill('v-lage', la == null ? 'n/a' : la.toFixed(2) + 's',
       la == null ? 'grey' : la > 0.5 ? 'red' : 'green');
  pill('v-rage', ra == null ? 'n/a' : ra.toFixed(2) + 's',
       ra == null ? 'grey' : ra > 0.5 ? 'red' : 'green');
  val('v-errs', (d.vesc_rx_parse_error_count ?? 0) + ' / ' +
                (d.vesc_rx_recv_error_count ?? 0) + ' / ' +
                (d.vesc_rx_reopen_count ?? 0),
      ((d.vesc_rx_parse_error_count || d.vesc_rx_recv_error_count || d.vesc_rx_reopen_count)) ? 'amber' : '');

  /* ---- BMS ---- */
  var bc = d.bms_connected;
  pill('b-conn', bc == null ? '—' : (bc ? 'ONLINE' : 'OFFLINE'),
       bc == null ? 'grey' : (bc ? 'green' : 'red'));
  val('b-volt', d.bms_voltage_v == null ? '—' : d.bms_voltage_v.toFixed(2) + ' V',
      d.bms_voltage_v == null ? 'grey' : '');
  var ba = d.bms_current_a;
  if (ba == null) { val('b-curr', '—', 'grey'); }
  else {
    var lbl = ba > 0.05 ? ' (charging)' : ba < -0.05 ? ' (discharging)' : '';
    val('b-curr', ba.toFixed(1) + ' A' + lbl, ba > 0.05 ? 'green' : ba < -0.05 ? 'amber' : '');
  }
  var soc = d.bms_soc_pct;
  val('b-soc', soc == null ? '—' : soc.toFixed(1) + '%',
      soc == null ? 'grey' : soc >= 40 ? 'green' : soc >= 20 ? 'amber' : 'red');
  var cd = d.bms_cell_delta_mv;
  val('b-delta', cd == null ? '—' : cd + ' mV',
      cd == null ? 'grey' : cd <= 20 ? 'green' : cd <= 40 ? 'amber' : 'red');
  var tp = d.bms_temp_max_c;
  var tmin = d.bms_temp_min_c;
  if (tp == null && tmin == null) {
    val('b-temp', '—', 'grey');
  } else if (tmin != null && tp != null && tmin !== tp) {
    val('b-temp', tmin.toFixed(1) + '–' + tp.toFixed(1) + '°C',
        tp >= 50 ? 'red' : tp >= 40 ? 'amber' : 'green');
  } else {
    var tshow = tp != null ? tp : tmin;
    val('b-temp', tshow.toFixed(1) + '°C',
        tshow >= 50 ? 'red' : tshow >= 40 ? 'amber' : 'green');
  }
  var chg = d.bms_charging;
  pill('b-chg', chg == null ? '—' : (chg ? 'YES' : 'no'),
       chg == null ? 'grey' : (chg ? 'blue' : 'grey'));
  var df = d.bms_discharge_fet_on;
  pill('b-dfet', df == null ? '—' : (df ? 'ON' : 'OFF'),
       df == null ? 'grey' : (df ? 'green' : 'amber'));

  /* ---- event log (robot) ---- */
  watch('armed', 'is_armed', d.is_armed, boolStr, function(v){ return v ? 'up' : 'down'; });
  watch('estop', 'emergency', d.emergency_active, boolStr, function(v){ return v ? 'down' : 'up'; });
  watch('mode', 'mode', d.mode, null, null);
  watch('chgi', 'charger_inhibit', d.charger_inhibit, boolStr, function(v){ return v ? 'down' : 'up'; });
  watch('packlow', 'pack_low_latched', d.vesc_pack_low_latched, boolStr, function(v){ return v ? 'down' : 'up'; });
  watch('rxthread', 'vesc_rx_thread', d.vesc_rx_thread_alive, function(v){ return v ? 'alive' : 'dead'; }, function(v){ return v ? 'up' : 'down'; });
  watch('bmsconn', 'bms_connected', d.bms_connected, boolStr, function(v){ return v ? 'up' : 'down'; });
  watch('bmschg', 'bms_charging', d.bms_charging, boolStr, null);
  watch('bmsdfet', 'bms_discharge_fet', d.bms_discharge_fet_on, function(v){ return v ? 'on' : 'off'; }, function(v){ return v ? 'up' : 'down'; });
  var band = dist == null ? 'unknown' : dist < 0.5 ? 'STOP' : dist < 1.2 ? 'slow' : 'clear';
  watch('obsband', 'obstacle_band', band, null, function(v){ return v === 'clear' ? 'up' : v === 'STOP' ? 'down' : 'neu'; });

  /* ---- Temperature history sample ---- */
  pushTempSample(d, Date.now());
};

/* ── UPS poll (separate daemon via /api/ups) ───────────────────────── */
function applyUps(u) {
  var panel = document.getElementById('p-ups');
  var tag = document.getElementById('ups-tag');
  var hint = document.getElementById('u-hint');
  if (!u || !u.available) {
    panel.classList.add('stale');
    tag.textContent = 'UNAVAILABLE';
    var reason = u && u.reason ? u.reason : 'no data';
    hint.textContent = 'UPS status unavailable (' + reason + '). Is upsPlus_power_daemon.py running?';
    ['u-ac','u-mode'].forEach(function(id){ pill(id, '—', 'grey'); });
    ['u-typec','u-microusb','u-battv','u-battma','u-protect','u-swc'].forEach(function(id){ val(id,'—','grey'); });
    watch('ac', 'ups_ac_present', undefined, null, null);
    return;
  }
  if (u.stale) {
    panel.classList.add('stale');
    tag.textContent = 'STALE ' + (u.age_s != null ? u.age_s + 's' : '');
    hint.textContent = 'Last UPS update ' + (u.age_s != null ? u.age_s + 's ago' : '(unknown age)') +
                       ' — older than freshness window; values may be out of date.';
  } else {
    panel.classList.remove('stale');
    tag.textContent = '';
    hint.textContent = 'Live — updated ' + (u.age_s != null ? u.age_s + 's ago' : 'just now') + '.';
  }
  var ac = u.ac_present;
  pill('u-ac', ac == null ? '—' : (ac ? 'PRESENT' : 'MISSING'),
       ac == null ? 'grey' : (ac ? 'green' : 'red'));
  val('u-typec', num(u.typec_mv, 0, ' mV'), u.typec_mv == null ? 'grey' : '');
  val('u-microusb', num(u.microusb_mv, 0, ' mV'), u.microusb_mv == null ? 'grey' : '');
  /* Battery/protect come from the daemon's supplemental read, which is
     suppressed during power-transfer windows and throttled to a slow cadence
     to keep I2C off the UPS MCU. When batt_stale, these are last-known (not
     fresh this poll) — grey them and tag with age so the owner isn't misled. */
  var bstale = !!u.batt_stale;
  var bage = (u.ts_batt != null) ? Math.max(0, Math.round(Date.now()/1000 - u.ts_batt)) : null;
  var bsuffix = (bstale && bage != null) ? ' (' + bage + 's old)' : bstale ? ' (stale)' : '';
  val('u-battv', num(u.batt_v, 3, ' V') + (u.batt_v == null ? '' : bsuffix),
      (u.batt_v == null || bstale) ? 'grey' : '');
  if (u.batt_ma == null) { val('u-battma', '—', 'grey'); }
  else if (bstale) { val('u-battma', u.batt_ma.toFixed(0) + ' mA' + bsuffix, 'grey'); }
  else {
    var mlbl = u.batt_ma > 20 ? ' (charging)' : u.batt_ma < -20 ? ' (discharging)' : '';
    val('u-battma', u.batt_ma.toFixed(0) + ' mA' + mlbl,
        u.batt_ma > 20 ? 'green' : u.batt_ma < -20 ? 'amber' : '');
  }
  val('u-protect', num(u.protect_mv, 0, ' mV'),
      (u.protect_mv == null || bstale) ? 'grey' : '');
  var swc = u.seconds_without_charge;
  val('u-swc', swc == null ? '—' : swc + ' s',
      swc == null ? 'grey' : swc > 0 ? 'amber' : 'green');
  /* Older daemons may not publish detect_only at all — render unknown as
     '—', never as ARMED (a false "armed" claim about power protection). */
  var dm = u.detect_only;
  pill('u-mode', dm == null ? '—' : (dm ? 'DETECT-ONLY' : 'ARMED'),
       dm == null ? 'grey' : (dm ? 'amber' : 'green'));

  watch('ac', 'ups_ac_present', ac, boolStr, function(v){ return v ? 'up' : 'down'; });
  watch('upsmode', 'ups_daemon_mode',
        dm == null ? undefined : (dm ? 'detect-only' : 'armed'), null, null);
}
function pollUps() {
  fetch('/api/ups', {cache: 'no-store'})
    .then(function(r){ return r.json(); })
    .then(applyUps)
    .catch(function(){ applyUps({available: false, reason: 'fetch failed'}); });
}
pollUps();
setInterval(pollUps, 1000);

/* ── Temperature history (canvas, bounded ring buffer, no deps) ─────── */
var TEMP_WINDOW_MS = 10 * 60 * 1000;   // 10 minutes of history
var TEMP_MAX_POINTS = 720;             // hard cap (~1.2 Hz × 10 min)
var TEMP_MIN_SAMPLE_MS = 900;          // throttle samples for Pi CPU
// Break stroke when consecutive samples are farther apart than this so an
// SSE drop / tab freeze does not draw a misleading diagonal across the gap.
var TEMP_GAP_MS = 5000;
// Match controller open-loop fallback: any STATUS(9) age > 0.5 s is stale.
var TEMP_VESC_STALE_S = 0.5;
var TEMP_PAD = { l: 44, r: 12, t: 14, b: 28 };
var TEMP_AXIS_COLOR = '#8b93a3';
var TEMP_GRID_COLOR = '#1c2030';
// Distinct hues (not two golds) so BMS min/max stay separable at a glance.
var TEMP_SERIES = [
  { key: 'bms_temp_max_c',          label: 'BMS Max',    color: '#f0b344', source: 'bms'  },
  { key: 'bms_temp_min_c',          label: 'BMS Min',    color: '#86efac', source: 'bms'  },
  { key: 'vesc_left_temp_c',        label: 'VESC L FET', color: '#6bb6ff', source: 'vesc' },
  { key: 'vesc_right_temp_c',       label: 'VESC R FET', color: '#4ecdc4', source: 'vesc' },
  { key: 'vesc_left_motor_temp_c',  label: 'VESC L Mot', color: '#a78bfa', source: 'vesc' },
  { key: 'vesc_right_motor_temp_c', label: 'VESC R Mot', color: '#f472b6', source: 'vesc' },
  { key: 'pi_cpu_temp_c',           label: 'Pi CPU',     color: '#ff8a65', source: 'pi'   }
];
var tempState = {
  t: [],                               // sample times (Date.now ms)
  series: {},                          // key -> { enabled, values:[], cur, min, max, stale }
  lastSampleAt: 0,
  drawPending: false,
  legendBuilt: false
};
TEMP_SERIES.forEach(function(s) {
  tempState.series[s.key] = {
    enabled: true, values: [], cur: null, min: null, max: null, stale: false
  };
});

var tempCanvas = document.getElementById('temp-canvas');
var tempEmpty = document.getElementById('temp-empty');
var tempMeta = document.getElementById('temp-meta');
var tempLegend = document.getElementById('temp-legend');
var tempSr = document.getElementById('temp-sr');
var tempCtx = tempCanvas ? tempCanvas.getContext('2d') : null;

function finiteOrNull(v) {
  return (typeof v === 'number' && isFinite(v)) ? v : null;
}
function fmtTemp(v) {
  return v == null ? '—' : v.toFixed(1) + '°';
}
function seriesTitle(s, st) {
  return 'Toggle ' + s.label + (st.stale ? ' (stale/disconnected)' : '');
}
function seriesAria(s, st) {
  var state = st.enabled ? 'shown' : 'hidden';
  var val = st.cur == null ? 'no reading' : (st.cur.toFixed(1) + ' degrees C');
  var stale = st.stale ? ', stale' : '';
  return s.label + ', ' + state + ', ' + val + stale;
}
function chipClass(st) {
  return 'temp-chip' + (st.enabled ? '' : ' off') + (st.stale ? ' stale' : '');
}
function mmText(st) {
  if (st.min == null || st.max == null) return '';
  return ' · ' + st.min.toFixed(1) + '–' + st.max.toFixed(1);
}
/** Build legend buttons once; later samples only patch values (keeps focus). */
function ensureTempLegend() {
  if (!tempLegend || tempState.legendBuilt) return;
  var html = '';
  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    html += '<button type="button" class="' + chipClass(st) + '" data-key="' + s.key + '" ' +
            'aria-pressed="' + (st.enabled ? 'true' : 'false') + '" ' +
            'title="' + esc(seriesTitle(s, st)) + '" ' +
            'aria-label="' + esc(seriesAria(s, st)) + '">' +
            '<span class="swatch" style="background:' + s.color + '" aria-hidden="true"></span>' +
            '<span class="cname">' + esc(s.label) + '</span>' +
            '<span class="cval">' + fmtTemp(st.cur) + '</span>' +
            '<span class="cmm">' + esc(mmText(st)) + '</span></button>';
  });
  tempLegend.innerHTML = html;
  tempState.legendBuilt = true;
}
function updateTempLegend() {
  if (!tempLegend) return;
  if (!tempState.legendBuilt) { ensureTempLegend(); return; }
  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    var btn = tempLegend.querySelector('[data-key="' + s.key + '"]');
    if (!btn) return;
    btn.className = chipClass(st);
    btn.setAttribute('aria-pressed', st.enabled ? 'true' : 'false');
    btn.title = seriesTitle(s, st);
    btn.setAttribute('aria-label', seriesAria(s, st));
    var cval = btn.querySelector('.cval');
    var cmm = btn.querySelector('.cmm');
    if (cval) cval.textContent = fmtTemp(st.cur);
    if (cmm) cmm.textContent = mmText(st);
  });
}
if (tempLegend) {
  tempLegend.addEventListener('click', function(ev) {
    var btn = ev.target.closest('[data-key]');
    if (!btn) return;
    var key = btn.getAttribute('data-key');
    var st = tempState.series[key];
    if (!st) return;
    st.enabled = !st.enabled;
    updateTempLegend();
    scheduleTempDraw();
  });
}
ensureTempLegend();

function trimTempHistory(now) {
  var cutoff = now - TEMP_WINDOW_MS;
  var t = tempState.t;
  var drop = 0;
  while (drop < t.length && t[drop] < cutoff) drop++;
  // Also enforce hard point cap from the left.
  var over = t.length - drop - TEMP_MAX_POINTS;
  if (over > 0) drop += over;
  if (drop <= 0) return;
  tempState.t = t.slice(drop);
  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    st.values = st.values.slice(drop);
  });
}
function recomputeSeriesStats() {
  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    var mn = null, mx = null;
    for (var i = 0; i < st.values.length; i++) {
      var v = st.values[i];
      if (v == null) continue;
      if (mn == null || v < mn) mn = v;
      if (mx == null || v > mx) mx = v;
    }
    st.min = mn;
    st.max = mx;
    // Live current only — stale/disconnected shows "—" (not a frozen last value).
    st.cur = null;
    if (!st.stale) {
      for (var j = st.values.length - 1; j >= 0; j--) {
        if (st.values[j] != null) { st.cur = st.values[j]; break; }
      }
    }
  });
}
function vescTempsStale(d) {
  // Mirror controller: RX dead OR any STATUS age > 0.5 s clears temps.
  if (d.vesc_rx_thread_alive === false) return true;
  if (d.vesc_left_status_age_s != null && d.vesc_left_status_age_s > TEMP_VESC_STALE_S) return true;
  if (d.vesc_right_status_age_s != null && d.vesc_right_status_age_s > TEMP_VESC_STALE_S) return true;
  return false;
}
function pushTempSample(d, now) {
  if (!tempCtx) return;
  if (now - tempState.lastSampleAt < TEMP_MIN_SAMPLE_MS) return;
  tempState.lastSampleAt = now;

  var bmsOffline = d.bms_connected === false;
  var bmsStale = bmsOffline || (d.bms_connected != null && d.bms_connected !== true);
  var vescStale = vescTempsStale(d);

  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    var raw = finiteOrNull(d[s.key]);
    // When source is known-dead/stale, force a gap rather than hold a frozen value.
    if (s.source === 'bms' && bmsOffline) raw = null;
    if (s.source === 'vesc' && vescStale) raw = null;
    st.values.push(raw);
    st.stale = (s.source === 'bms' && bmsStale) ||
               (s.source === 'vesc' && vescStale) ||
               (s.source === 'pi' && raw == null);
  });
  tempState.t.push(now);
  trimTempHistory(now);
  recomputeSeriesStats();
  updateTempLegend();

  var n = tempState.t.length;
  var spanMin = n > 1 ? ((tempState.t[n - 1] - tempState.t[0]) / 60000) : 0;
  var meta = n + ' sample' + (n === 1 ? '' : 's') +
    (spanMin > 0.05 ? (' · ' + spanMin.toFixed(1) + ' min window') : '') +
    ' · °C';
  if (tempMeta) tempMeta.textContent = meta;
  if (tempEmpty) tempEmpty.classList.toggle('hidden', n > 0);
  if (tempSr) {
    var parts = [];
    TEMP_SERIES.forEach(function(s) {
      var st = tempState.series[s.key];
      if (!st.enabled || st.cur == null) return;
      parts.push(s.label + ' ' + st.cur.toFixed(1) + '°C');
    });
    tempSr.textContent = parts.length
      ? ('Temperatures: ' + parts.join(', ') + '. ' + meta)
      : (n ? 'Temperature samples recorded but no live readings.' : 'No temperature samples yet');
  }
  scheduleTempDraw();
}

function scheduleTempDraw() {
  if (tempState.drawPending || !tempCtx) return;
  tempState.drawPending = true;
  requestAnimationFrame(function() {
    tempState.drawPending = false;
    drawTempChart();
  });
}
function resizeTempCanvas() {
  if (!tempCanvas || !tempCtx) return { w: 300, h: 220 };
  var wrap = tempCanvas.parentElement;
  var cssW = Math.max(200, (wrap && wrap.clientWidth) || 300);
  var cssH = Math.max(160, (wrap && wrap.clientHeight) || 220);
  // Cap DPR at 2 — Retina-sharp without 3× canvas cost on Pi browsers.
  var dpr = Math.min(window.devicePixelRatio || 1, 2);
  var w = Math.round(cssW * dpr);
  var h = Math.round(cssH * dpr);
  if (tempCanvas.width !== w || tempCanvas.height !== h) {
    tempCanvas.width = w;
    tempCanvas.height = h;
  }
  tempCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: cssW, h: cssH };
}
function drawTempChart() {
  if (!tempCtx || !tempCanvas) return;
  var size = resizeTempCanvas();
  var W = size.w, H = size.h;
  var pad = TEMP_PAD;
  var plotW = Math.max(1, W - pad.l - pad.r);
  var plotH = Math.max(1, H - pad.t - pad.b);
  tempCtx.clearRect(0, 0, W, H);

  var times = tempState.t;
  if (!times.length) return;

  var tMin = times[0];
  var tMax = times[times.length - 1];
  if (tMax <= tMin) tMax = tMin + 1000;
  // Keep a fixed rolling window feel once we have enough history.
  var windowStart = Math.max(tMin, tMax - TEMP_WINDOW_MS);
  var yMin = null, yMax = null;
  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    if (!st.enabled) return;
    // Scale Y from points actually inside the visible window.
    for (var i = 0; i < times.length; i++) {
      if (times[i] < windowStart) continue;
      var v = st.values[i];
      if (v == null) continue;
      if (yMin == null || v < yMin) yMin = v;
      if (yMax == null || v > yMax) yMax = v;
    }
  });
  if (yMin == null || yMax == null) {
    tempCtx.fillStyle = TEMP_AXIS_COLOR;
    tempCtx.font = '12px -apple-system, BlinkMacSystemFont, sans-serif';
    tempCtx.textAlign = 'left';
    tempCtx.textBaseline = 'middle';
    tempCtx.fillText('All series hidden or null', pad.l, H / 2);
    return;
  }
  // Comfortable padding so lines don't hug the frame.
  var padY = Math.max(2, (yMax - yMin) * 0.12);
  if (yMax - yMin < 1) { padY = 2; }
  yMin = Math.floor(yMin - padY);
  yMax = Math.ceil(yMax + padY);
  if (yMax <= yMin) yMax = yMin + 1;

  function xOf(t) { return pad.l + ((t - windowStart) / (tMax - windowStart)) * plotW; }
  function yOf(v) { return pad.t + (1 - (v - yMin) / (yMax - yMin)) * plotH; }

  // Grid + axes
  tempCtx.strokeStyle = TEMP_GRID_COLOR;
  tempCtx.lineWidth = 1;
  tempCtx.fillStyle = TEMP_AXIS_COLOR;
  tempCtx.font = '10px -apple-system, BlinkMacSystemFont, sans-serif';
  tempCtx.textAlign = 'right';
  tempCtx.textBaseline = 'middle';
  var yTicks = 4;
  for (var gi = 0; gi <= yTicks; gi++) {
    var gv = yMin + (gi / yTicks) * (yMax - yMin);
    var gy = yOf(gv);
    tempCtx.beginPath();
    tempCtx.moveTo(pad.l, gy);
    tempCtx.lineTo(pad.l + plotW, gy);
    tempCtx.stroke();
    tempCtx.fillText(gv.toFixed(0) + '°', pad.l - 6, gy);
  }
  tempCtx.textAlign = 'center';
  tempCtx.textBaseline = 'top';
  var xTicks = 4;
  for (var xi = 0; xi <= xTicks; xi++) {
    var tt = windowStart + (xi / xTicks) * (tMax - windowStart);
    var gx = xOf(tt);
    tempCtx.strokeStyle = TEMP_GRID_COLOR;
    tempCtx.beginPath();
    tempCtx.moveTo(gx, pad.t);
    tempCtx.lineTo(gx, pad.t + plotH);
    tempCtx.stroke();
    var ageS = Math.max(0, (tMax - tt) / 1000);
    var label = ageS < 60 ? ('-' + Math.round(ageS) + 's')
                          : ('-' + (ageS / 60).toFixed(ageS < 600 ? 1 : 0) + 'm');
    if (xi === xTicks) label = 'now';
    tempCtx.fillStyle = TEMP_AXIS_COLOR;
    tempCtx.fillText(label, gx, pad.t + plotH + 8);
  }
  // Plot border
  tempCtx.strokeStyle = '#2a2d37';
  tempCtx.strokeRect(pad.l + 0.5, pad.t + 0.5, plotW - 1, plotH - 1);

  // Series lines (gap on null AND on large time jumps / SSE reconnect)
  TEMP_SERIES.forEach(function(s) {
    var st = tempState.series[s.key];
    if (!st.enabled) return;
    tempCtx.strokeStyle = s.color;
    tempCtx.lineWidth = 1.8;
    tempCtx.lineJoin = 'round';
    tempCtx.lineCap = 'round';
    tempCtx.globalAlpha = st.stale ? 0.45 : 1.0;
    tempCtx.beginPath();
    var penDown = false;
    var prevT = null;
    for (var i = 0; i < times.length; i++) {
      var tv = times[i];
      if (tv < windowStart) { penDown = false; prevT = null; continue; }
      var val = st.values[i];
      if (val == null) { penDown = false; prevT = tv; continue; }
      if (prevT != null && (tv - prevT) > TEMP_GAP_MS) penDown = false;
      var px = xOf(tv), py = yOf(val);
      if (!penDown) { tempCtx.moveTo(px, py); penDown = true; }
      else tempCtx.lineTo(px, py);
      prevT = tv;
    }
    tempCtx.stroke();
    // Current point marker (last non-null in window, even if series flagged stale)
    if (times.length) {
      var lastIdx = -1;
      for (var k = st.values.length - 1; k >= 0; k--) {
        if (st.values[k] != null && times[k] >= windowStart) { lastIdx = k; break; }
      }
      if (lastIdx >= 0) {
        tempCtx.fillStyle = s.color;
        tempCtx.beginPath();
        tempCtx.arc(xOf(times[lastIdx]), yOf(st.values[lastIdx]), 2.6, 0, Math.PI * 2);
        tempCtx.fill();
      }
    }
    tempCtx.globalAlpha = 1.0;
  });
}

var _tempResizeObs = null;
if (typeof ResizeObserver !== 'undefined' && tempCanvas && tempCanvas.parentElement) {
  _tempResizeObs = new ResizeObserver(function() { scheduleTempDraw(); });
  _tempResizeObs.observe(tempCanvas.parentElement);
} else if (tempCanvas) {
  window.addEventListener('resize', scheduleTempDraw);
}
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


def _depth_frame_to_jpeg(depth_frame) -> bytes | None:
    """Best-effort depth colorization for web preview fallback."""
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    if depth_frame is None:
        return None
    try:
        valid_mask = depth_frame > 0
        if not valid_mask.any():
            return None
        max_mm = min(float(np.max(depth_frame[valid_mask])), 10000.0)
        norm = np.zeros_like(depth_frame, dtype=np.uint8)
        if max_mm > 0:
            norm[valid_mask] = (
                np.clip(depth_frame[valid_mask].astype(np.float32) / max_mm, 0, 1) * 255
            ).astype(np.uint8)
        colorized = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        colorized = cv2.resize(colorized, (320, 240), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", colorized, [cv2.IMWRITE_JPEG_QUALITY, 50])
        return buf.tobytes() if ok else None
    except Exception:
        return None


def sse_detection_dict(d) -> dict:
    """Serialize one PersonDetection for the /api/telemetry SSE `detections` array.

    Pure function extracted so tests can exercise the exact production
    serialization instead of a hand-maintained mirror.
    """
    return {
        "x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
        "conf": round(d.confidence, 2), "track_id": d.track_id,
    }


# ---------------------------------------------------------------------------
# Flask app factory
# ---------------------------------------------------------------------------

def create_app(recorder, config: OakWebViewerConfig, controller=None, oak_reader=None,
               estop_check=None) -> Flask:  # recorder may be None
    """Create the Flask app wired to the given OakRecorder and Controller instances.

    ``estop_check``: optional zero-arg callable returning True while the
    /drive TeleopSession's e-stop is latched. When it returns True, autonomy
    activation (``/api/follow_me``) refuses with 409 rather than silently
    fighting a driver who just hit the physical e-stop. ``None`` (the
    default, e.g. when /drive fails to load) means "no session gate" —
    behavior is unchanged from before this check existed.
    """
    if Flask is None:
        raise ImportError("Flask is required for the web viewer: pip install flask")

    app = Flask(__name__)
    app.config["PROPAGATE_EXCEPTIONS"] = False
    placeholder = _placeholder_jpeg()
    rec_cache_lock = threading.Lock()
    rec_cache: list[dict] = []
    rec_cache_ts = 0.0
    rec_cache_ttl_s = 5.0

    def _teleop_estopped() -> bool:
        if estop_check is None:
            return False
        try:
            return bool(estop_check())
        except Exception:
            return False

    # -- Dashboard -----------------------------------------------------------

    @app.route("/")
    def index():
        return Response(_DASHBOARD_HTML, content_type="text/html")

    # -- Debug status board (read-only) --------------------------------------

    @app.route("/debug")
    def debug_board():
        return Response(_DEBUG_HTML, content_type="text/html")

    @app.route("/api/ups")
    def api_ups():
        """UPS snapshot from the separate power daemon's status file.

        Read-only: reads /tmp/ups_status.json (via read_ups_status, which never
        raises). Returns 200 even when the file is missing/stale/corrupt so the
        debug page can render an UNAVAILABLE/STALE state instead of erroring.
        """
        return Response(json.dumps(read_ups_status()), content_type="application/json")

    # -- MJPEG streams -------------------------------------------------------

    def _mjpeg_generator(get_jpeg_fn, fps: float = 10.0, stream_name: str | None = None):
        interval = 1.0 / fps
        try:
            if recorder is not None and stream_name is not None and hasattr(recorder, "set_stream_client_connected"):
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
            if recorder is not None and stream_name is not None and hasattr(recorder, "set_stream_client_connected"):
                recorder.set_stream_client_connected(stream_name, False)

    @app.route("/stream/rgb")
    def stream_rgb():
        if recorder is None:
            return Response(placeholder, mimetype="image/jpeg")
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
        def _get_depth_jpeg():
            jpeg = recorder.get_latest_depth_jpeg() if recorder is not None else None
            if jpeg:
                return jpeg
            # Fallback: colorize latest raw depth frame directly from the OAK reader.
            if oak_reader is not None:
                try:
                    return _depth_frame_to_jpeg(oak_reader.get_latest_depth_frame())
                except Exception:
                    return None
            return None
        return Response(
            _mjpeg_generator(
                _get_depth_jpeg,
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
            t = recorder.get_latest_telemetry() if recorder is not None else None
            if t is not None:
                obj = {
                    "mode": t.mode,
                    "throttle_scale": _finite_or_none(t.throttle_scale, 3),
                    "obstacle_distance_m": _finite_or_none(t.obstacle_distance_m, 3),
                    "motor_left": t.motor_left,
                    "motor_right": t.motor_right,
                    "is_armed": t.is_armed,
                    "emergency_active": getattr(t, "emergency_active", None),
                    "charger_inhibit": t.charger_inhibit,
                    "num_persons": len(t.person_detections),
                    "recording_state": recorder.recording_state if recorder is not None else "disabled",
                    "follow_tracking": t.follow_tracking,
                    "follow_target_x_m": round(t.follow_target_x_m, 2) if t.follow_target_x_m is not None else None,
                    "follow_target_z_m": round(t.follow_target_z_m, 2) if t.follow_target_z_m is not None else None,
                    "detections": [
                        sse_detection_dict(d) for d in t.person_detections
                    ],
                    "gps_lat": round(t.gps_lat, 7) if t.gps_lat is not None else None,
                    "gps_lon": round(t.gps_lon, 7) if t.gps_lon is not None else None,
                    "gps_alt_m": round(t.gps_alt_m, 1) if t.gps_alt_m is not None else None,
                    "gps_fix": t.gps_fix,
                    "gps_sats": t.gps_sats,
                    "gps_hdop": round(t.gps_hdop, 2) if t.gps_hdop is not None else None,
                    "gps_diff_age_s": round(t.gps_diff_age_s, 2) if t.gps_diff_age_s is not None else None,
                    "gps_station_id": t.gps_station_id,
                    "bms_voltage_v": _finite_or_none(getattr(t, "bms_voltage_v", None), 2),
                    "bms_current_a": _finite_or_none(getattr(t, "bms_current_a", None), 1),
                    "bms_soc_pct": _finite_or_none(getattr(t, "bms_soc_pct", None), 1),
                    "bms_cell_delta_mv": getattr(t, "bms_cell_delta_mv", None),
                    "bms_temp_max_c": _finite_or_none(getattr(t, "bms_temp_max_c", None), 1),
                    "bms_temp_min_c": _finite_or_none(getattr(t, "bms_temp_min_c", None), 1),
                    "bms_connected": getattr(t, "bms_connected", None),
                    "bms_charging": getattr(t, "bms_charging", None),
                    "bms_discharge_fet_on": getattr(t, "bms_discharge_fet_on", None),
                    "imu_heading_deg": round(t.imu_heading_deg, 1) if t.imu_heading_deg is not None else None,
                    "vesc_left_rpm": getattr(t, "vesc_left_rpm", None),
                    "vesc_right_rpm": getattr(t, "vesc_right_rpm", None),
                    "vesc_actual_speed_mps": _finite_or_none(getattr(t, "vesc_actual_speed_mps", None), 3),
                    "vesc_rx_frame_count": getattr(t, "vesc_rx_frame_count", 0),
                    "vesc_rx_parse_error_count": getattr(t, "vesc_rx_parse_error_count", 0),
                    "vesc_rx_recv_error_count": getattr(t, "vesc_rx_recv_error_count", 0),
                    "vesc_rx_reopen_count": getattr(t, "vesc_rx_reopen_count", 0),
                    "vesc_rx_last_frame_age_s": _finite_or_none(getattr(t, "vesc_rx_last_frame_age_s", None), 3),
                    "vesc_pack_low_latched": bool(getattr(t, "vesc_pack_low_latched", False)),
                    "vesc_left_status_age_s": _finite_or_none(getattr(t, "vesc_left_status_age_s", None), 3),
                    "vesc_right_status_age_s": _finite_or_none(getattr(t, "vesc_right_status_age_s", None), 3),
                    "vesc_rx_thread_alive": getattr(t, "vesc_rx_thread_alive", None),
                    "vesc_left_temp_c": _finite_or_none(getattr(t, "vesc_left_temp_c", None), 1),
                    "vesc_right_temp_c": _finite_or_none(getattr(t, "vesc_right_temp_c", None), 1),
                    "vesc_left_motor_temp_c": _finite_or_none(getattr(t, "vesc_left_motor_temp_c", None), 1),
                    "vesc_right_motor_temp_c": _finite_or_none(getattr(t, "vesc_right_motor_temp_c", None), 1),
                    "pi_cpu_temp_c": read_pi_cpu_temp_c(),
                    "wp_index": getattr(t, "wp_index", None),
                    "wp_total": getattr(t, "wp_total", None),
                    "wp_name": getattr(t, "wp_name", None),
                    "wp_bearing_deg": _finite_or_none(getattr(t, "wp_bearing_deg", None), 1),
                    "wp_distance_m": _finite_or_none(getattr(t, "wp_distance_m", None), 2),
                    "wp_completed": getattr(t, "wp_completed", None),
                    "heading_offset_deg": _finite_or_none(getattr(t, "heading_offset_deg", None), 1),
                    "heading_offset_locked": getattr(t, "heading_offset_locked", None),
                    "heading_offset_frozen": getattr(t, "heading_offset_frozen", None),
                    "heading_offset_refining": getattr(t, "heading_offset_refining", None),
                    "corrected_heading_deg": _finite_or_none(
                        getattr(t, "corrected_heading_deg", None), 1
                    ),
                    "heading_align": {
                        "enabled": bool(getattr(t, "heading_align_enabled", False)),
                        "locked": bool(getattr(t, "heading_offset_locked", False)),
                        "frozen": bool(getattr(t, "heading_offset_frozen", False)),
                        "refining": bool(getattr(t, "heading_offset_refining", False)),
                        "offset_deg": _finite_or_none(getattr(t, "heading_offset_deg", None), 1),
                        "corrected_heading_deg": _finite_or_none(
                            getattr(t, "corrected_heading_deg", None), 1
                        ),
                        "last_cog_deg": _finite_or_none(
                            getattr(t, "heading_align_last_cog_deg", None), 1
                        ),
                    },
                }
                # Follow-Me visualization fields
                obj["follow_mode"] = getattr(t, "follow_mode", None)
                obj["pursuit_mode"] = getattr(t, "pursuit_mode", None)
                obj["trail_length"] = getattr(t, "trail_length", None)
                obj["trail_distance_m"] = _finite_or_none(getattr(t, "trail_distance_m", None), 2)
                obj["trail_rejected_jump_count"] = getattr(t, "trail_rejected_jump_count", None)
                obj["trail_rejected_speed_count"] = getattr(t, "trail_rejected_speed_count", None)
                obj["trail_curvature_at_lookahead"] = _finite_or_none(getattr(t, "trail_curvature_at_lookahead", None), 4)
                obj["target_confidence"] = _finite_or_none(getattr(t, "target_confidence", None), 2)
                obj["target_lateral_offset"] = _finite_or_none(getattr(t, "target_lateral_offset", None), 3)
                obj["target_track_id"] = getattr(t, "target_track_id", None)
                obj["hysteresis_count"] = getattr(t, "hysteresis_count", 0)
                obj["hysteresis_max"] = getattr(t, "hysteresis_max", 3)
                obj["extrapolation_active"] = getattr(t, "extrapolation_active", False)
                obj["extrapolation_count"] = getattr(t, "extrapolation_count", 0)
                obj["trail_points_xy"] = getattr(t, "trail_points_xy", None)
                obj["lookahead_point_xy"] = getattr(t, "lookahead_point_xy", None)
                obj["robot_pose_x"] = _finite_or_none(getattr(t, "robot_pose_x", None), 3)
                obj["robot_pose_y"] = _finite_or_none(getattr(t, "robot_pose_y", None), 3)
                obj["robot_pose_theta"] = _finite_or_none(getattr(t, "robot_pose_theta", None), 4)
                obj["consume_radius_m"] = _finite_or_none(getattr(t, "consume_radius_m", None), 3)
                obj["odom_x"] = _finite_or_none(getattr(t, "odom_x", None), 3)
                obj["odom_y"] = _finite_or_none(getattr(t, "odom_y", None), 3)
                obj["odom_theta_deg"] = _finite_or_none(getattr(t, "odom_theta_deg", None), 1)
                obj["speed_offset"] = _finite_or_none(getattr(t, "speed_offset", None), 1)
                obj["steer_offset"] = _finite_or_none(getattr(t, "steer_offset", None), 1)
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
        if _teleop_estopped():
            return Response(
                json.dumps({"error": "teleop session e-stop latched", "mode": "MANUAL"}),
                status=409, content_type="application/json")
        t = recorder.get_latest_telemetry() if recorder is not None else None
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
            return Response(json.dumps({"error": "must be armed with target present", "mode": mode}),
                            status=400, content_type="application/json")

    @app.route("/api/follow_me/reset_counter", methods=["POST"])
    def api_follow_me_reset_counter():
        if controller is None:
            return Response(json.dumps({"error": "no controller"}), status=503,
                            content_type="application/json")
        try:
            fm = getattr(controller, "_follow_me", None)
            if fm is not None and hasattr(fm, "reset_debug_counters"):
                fm.reset_debug_counters()
            return Response(json.dumps({"ok": True}), content_type="application/json")
        except Exception as exc:
            return Response(json.dumps({"error": str(exc)}), status=500,
                            content_type="application/json")

    @app.route("/api/follow_me/params", methods=["GET"])
    def api_follow_me_params_get():
        """Return the five runtime-tunable Follow-Me steering params."""
        if controller is None:
            return Response(json.dumps({"error": "no controller"}), status=503,
                            content_type="application/json")
        fm = getattr(controller, "_follow_me", None)
        if fm is None or not hasattr(fm, "get_tunable_params"):
            return Response(json.dumps({"error": "follow-me unavailable"}), status=503,
                            content_type="application/json")
        return Response(json.dumps(fm.get_tunable_params()),
                        content_type="application/json")

    @app.route("/api/follow_me/params", methods=["POST"])
    def api_follow_me_params_set():
        """Apply a subset of the five tunable params to the live controller.

        Validates hard bounds; rejects any out-of-range value or unknown key
        with 400 and applies NOTHING (all-or-nothing). Changes are volatile
        (lost on restart) — intended for tuning trials. Sets gains/filters
        only; cannot command motion.
        """
        if controller is None:
            return Response(json.dumps({"error": "no controller"}), status=503,
                            content_type="application/json")
        fm = getattr(controller, "_follow_me", None)
        if fm is None or not hasattr(fm, "apply_tunable_params"):
            return Response(json.dumps({"error": "follow-me unavailable"}), status=503,
                            content_type="application/json")
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict) or not payload:
            return Response(
                json.dumps({"error": "JSON body with at least one param required"}),
                status=400, content_type="application/json")
        try:
            applied = fm.apply_tunable_params(payload)
        except ValueError as exc:
            return Response(json.dumps({"error": str(exc)}), status=400,
                            content_type="application/json")
        return Response(json.dumps({"ok": True, "applied": applied}),
                        content_type="application/json")

    # -- Legacy teleop (RETIRED) ----------------------------------------------
    #
    # These endpoints used to write /tmp/wall_e_bt_latest.json directly, with
    # NO auth/arm/e-stop/single-driver-lock checks — they routed around every
    # safety guard the /drive TeleopSession enforces (250ms watchdog deadman,
    # single-driver lock, latched e-stop, speed caps) while sharing the exact
    # same override file the /drive watchdog thread writes. Last-writer-wins
    # meant this endpoint could fight /drive, or drive unsupervised while a
    # /drive e-stop was latched. Retired in favor of the session-gated REST
    # mirror at /api/teleop/session/* (see pi_app/web/teleop.py) or the /drive
    # page directly. Kept as a 410 Gone stub (not a plain 404) so any stale
    # client gets an unambiguous "this is gone for good" signal.
    @app.route("/api/teleop", methods=["POST"])
    def api_teleop():
        return Response(
            json.dumps({"error": "retired — use /drive or /api/teleop/session/*"}),
            status=410, content_type="application/json")

    @app.route("/api/teleop/stop", methods=["POST"])
    def api_teleop_stop():
        return Response(
            json.dumps({"error": "retired — use /drive or /api/teleop/session/*"}),
            status=410, content_type="application/json")

    # -- Recordings API ------------------------------------------------------

    @app.route("/api/recordings")
    def api_recordings():
        nonlocal rec_cache_ts, rec_cache
        now = time.monotonic()
        with rec_cache_lock:
            if now - rec_cache_ts >= rec_cache_ttl_s:
                rec_dir = recorder.recordings_dir if recorder is not None else Path("/nonexistent")
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
        rec_dir = (recorder.recordings_dir if recorder is not None else Path("/nonexistent")) / session
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
        self._teleop_session = None  # set in _run once the Flask app exists

    def start(self) -> None:
        if Flask is None:
            logger.warning("Flask not installed — web viewer disabled")
            return
        self._thread = threading.Thread(
            target=self._run, name="OakWebViewer", daemon=True
        )
        self._thread.start()

    def _session_estopped(self) -> bool:
        """True while the /drive TeleopSession's e-stop is latched.

        Read-only gate used by autonomy-activation endpoints (follow-me,
        waypoint nav) so they refuse to arm while a driver has hit e-stop.
        Safe to call before the session exists (startup ordering below
        creates it last) — returns False ("not gated") until then.
        """
        session = self._teleop_session
        return bool(session is not None and session.estop_latched)

    def _run(self) -> None:
        try:
            app = create_app(
                self._recorder,
                self._config,
                controller=self._controller,
                oak_reader=self._oak_reader,
                estop_check=self._session_estopped,
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
                bp = create_calibration_blueprint(cal_mgr, estop_check=self._session_estopped)
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

            try:
                from pi_app.web.waypoint_nav_ui import create_nav_blueprint
                nav_bp = create_nav_blueprint(
                    controller=self._controller,
                    estop_check=self._session_estopped,
                )
                app.register_blueprint(nav_bp)
                logger.info("Waypoint nav UI registered at /navigate")
            except Exception:
                logger.exception("Waypoint nav UI failed to load — skipping")

            try:
                import os
                from pi_app.web.teleop import (
                    TeleopSession, FileCommandSink, register_teleop,
                    make_recorder_rc_state_provider, make_recorder_battery_provider,
                )
                self._teleop_session = TeleopSession(
                    command_sink=FileCommandSink(),
                    rc_state_provider=make_recorder_rc_state_provider(self._recorder),
                )
                register_teleop(
                    app, self._teleop_session,
                    token=os.environ.get("WALL_E_TELEOP_TOKEN", ""),
                    battery_provider=make_recorder_battery_provider(self._recorder),
                    frame_source=(self._recorder.get_latest_annotated_jpeg
                                  if self._recorder is not None else None),
                    frame_client_hook=(
                        (lambda c: self._recorder.set_stream_client_connected("rgb", c))
                        if self._recorder is not None else None
                    ),
                )
                self._teleop_session.start_watchdog()
                logger.info("Fail-safe teleop registered at /drive (deadman watchdog running)")
            except Exception:
                logger.exception("Teleop /drive failed to load — skipping")

            app.run(
                host=self._config.host,
                port=self._config.port,
                threaded=True,
                use_reloader=False,
            )
        except Exception:
            logger.exception("Web viewer failed to start")
