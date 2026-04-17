"""
Waypoint navigation web UI — Flask Blueprint.

Provides an interactive Leaflet-based map for creating, editing, and
executing GPS waypoint routes on the WALL-E Mini robot.

Routes:
  GET  /navigate              — full-screen waypoint navigation page
  GET  /api/nav/waypoints     — list saved waypoint routes
  POST /api/nav/waypoints     — save current waypoints as named route
  POST /api/nav/load          — load a saved route by name
  POST /api/nav/start         — activate waypoint nav with current waypoints
  POST /api/nav/stop          — deactivate waypoint nav
  POST /api/nav/skip          — skip to next waypoint
  DELETE /api/nav/waypoints/<name> — delete saved route
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path

try:
    from flask import Blueprint, Response, request, jsonify
except ImportError:
    Blueprint = None  # type: ignore[assignment,misc]

import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.control.geo_transform import (
    load_calibration,
    pixel_to_gps_pt,
    gps_to_pixel_pt,
)
from pi_app.control.waypoint_nav import Waypoint
from config import config

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_log = logging.getLogger(__name__)
_ROUTES_DIR = _PROJECT_ROOT / "data" / "routes"


def _json_resp(data, status=200):
    return Response(
        json.dumps(data), status=status, content_type="application/json"
    )


# ---------------------------------------------------------------------------
# Inline HTML / JS / CSS  (matches project pattern — no template files)
# ---------------------------------------------------------------------------

_NAV_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<title>WALL-E Mini — Navigate</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { height: 100%; overflow: hidden; touch-action: manipulation;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    background: #0f1117; color: #c0caf5; -webkit-tap-highlight-color: transparent; }

  /* ── Map ── */
  #map { position: absolute; top: 0; left: 0; width: 100%; height: 100%;
    z-index: 0; background: #0f1117; }
  .leaflet-container { background: #0f1117 !important; }

  /* ── Mode Toolbar ── */
  .toolbar { position: fixed; top: env(safe-area-inset-top, 0px); left: 0; right: 0;
    z-index: 800; display: flex; padding: 8px 8px 6px; gap: 6px;
    background: rgba(15,17,23,0.92); backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px); border-bottom: 1px solid #2a2d37; }
  .toolbar .back-btn { width: 40px; min-width: 40px; height: 40px; display: flex;
    align-items: center; justify-content: center; background: #1a1d27;
    border: 1px solid #2a2d37; border-radius: 8px; color: #7aa2f7;
    font-size: 20px; cursor: pointer; text-decoration: none; }
  .toolbar .mode-btn { flex: 1; height: 40px; border: 1px solid #2a2d37;
    background: #1a1d27; color: #787c99; border-radius: 8px; font-size: 12px;
    font-weight: 600; cursor: pointer; touch-action: manipulation;
    transition: all .15s; white-space: nowrap; padding: 0 6px; }
  .toolbar .mode-btn.active { background: #7aa2f7; color: #0f1117;
    border-color: #7aa2f7; }
  .toolbar .mode-btn svg { width: 14px; height: 14px; vertical-align: -2px;
    margin-right: 3px; fill: currentColor; }

  /* ── GPS Quality Badge ── */
  .gps-badge { position: fixed; top: calc(env(safe-area-inset-top, 0px) + 56px);
    right: 8px; z-index: 800; padding: 4px 10px; border-radius: 12px;
    font-size: 11px; font-weight: 700; letter-spacing: 0.5px;
    background: rgba(15,17,23,0.85); border: 1px solid #2a2d37; }
  .gps-badge.fix-rtk { color: #9ece6a; border-color: #9ece6a; }
  .gps-badge.fix-float { color: #e0af68; border-color: #e0af68; }
  .gps-badge.fix-gps { color: #f7768e; border-color: #f7768e; }
  .gps-badge.fix-none { color: #565a6e; border-color: #565a6e; }

  /* ── Bottom Sheet ── */
  .bottom-sheet { position: fixed; bottom: 0; left: 0; right: 0;
    z-index: 900; background: #1a1d27; border-top: 1px solid #2a2d37;
    border-radius: 16px 16px 0 0; transition: transform 0.3s ease;
    max-height: 85vh; display: flex; flex-direction: column;
    padding-bottom: env(safe-area-inset-bottom, 0px); }
  .sheet-handle { width: 36px; height: 4px; background: #3b3f54;
    border-radius: 2px; margin: 8px auto 4px; flex-shrink: 0; cursor: grab; }
  .sheet-status { display: flex; align-items: center; gap: 10px;
    padding: 6px 16px 8px; font-size: 13px; color: #787c99; flex-shrink: 0; }
  .sheet-status .mode-label { font-weight: 700; color: #7aa2f7; }
  .sheet-status .wp-count { margin-left: auto; }

  .sheet-body { padding: 0 12px 12px; overflow-y: auto; flex: 1;
    -webkit-overflow-scrolling: touch; display: none; }
  .bottom-sheet.half .sheet-body,
  .bottom-sheet.full .sheet-body { display: block; }

  .sheet-settings { display: none; padding: 8px 0; }
  .bottom-sheet.full .sheet-settings { display: block; }

  /* ── Waypoint List ── */
  .wp-list { list-style: none; margin: 0 0 10px; }
  .wp-list li { display: flex; align-items: center; gap: 8px;
    padding: 10px 8px; background: #0f1117; border-radius: 8px;
    margin-bottom: 4px; font-size: 13px; touch-action: manipulation; }
  .wp-list li .wp-num { width: 26px; height: 26px; border-radius: 50%;
    background: #7aa2f7; color: #0f1117; font-weight: 700; font-size: 12px;
    display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
  .wp-list li .wp-coords { flex: 1; color: #787c99; font-size: 11px;
    font-family: 'SF Mono', 'Fira Code', monospace; }
  .wp-list li .wp-dist { color: #565a6e; font-size: 11px; white-space: nowrap; }
  .wp-list li .wp-del { width: 28px; height: 28px; border: none;
    background: transparent; color: #f7768e; font-size: 16px; cursor: pointer;
    border-radius: 6px; display: flex; align-items: center; justify-content: center; }
  .wp-list li .wp-del:active { background: rgba(247,118,142,0.15); }
  .wp-list li.completed .wp-num { background: #9ece6a; }
  .wp-list li.active .wp-num { background: #ff9e64;
    animation: pulse-wp 1.2s ease-in-out infinite; }
  .wp-list li .wp-drag { cursor: grab; color: #565a6e; font-size: 16px;
    padding: 0 2px; touch-action: none; user-select: none; }

  @keyframes pulse-wp { 0%,100% { transform: scale(1); } 50% { transform: scale(1.15); } }

  /* ── Action Buttons ── */
  .action-row { display: flex; gap: 8px; margin-top: 4px; }
  .action-row button { flex: 1; height: 48px; border: none; border-radius: 10px;
    font-size: 14px; font-weight: 700; cursor: pointer; touch-action: manipulation;
    transition: all .15s; }
  .btn-go { background: #2d4a22; color: #9ece6a; position: relative; overflow: hidden; }
  .btn-go .go-fill { position: absolute; left: 0; top: 0; bottom: 0; width: 0;
    background: rgba(158,206,106,0.2); transition: width 0.05s linear; pointer-events: none; }
  .btn-go:disabled { opacity: 0.4; cursor: not-allowed; }
  .btn-go.armed-required { background: #3b2020; color: #f7768e; }
  .btn-stop { background: #4a2222; color: #f7768e; display: none; }
  .btn-stop:active { background: #5a2a2a; }
  .btn-skip { background: #1a1d27; color: #e0af68; border: 1px solid #2a2d37 !important; display: none; }
  .btn-clear { background: #1a1d27; color: #787c99; border: 1px solid #2a2d37 !important; }

  /* ── Save/Load Row ── */
  .route-row { display: flex; gap: 6px; margin-bottom: 10px; }
  .route-row button, .route-row select { height: 38px; border: 1px solid #2a2d37;
    background: #0f1117; color: #c0caf5; border-radius: 8px; font-size: 12px;
    cursor: pointer; padding: 0 10px; }
  .route-row select { flex: 1; appearance: none;
    background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='10' height='6'%3E%3Cpath d='M0 0l5 6 5-6z' fill='%23787c99'/%3E%3C/svg%3E");
    background-repeat: no-repeat; background-position: right 10px center;
    padding-right: 26px; }
  .route-row input[type=text] { flex: 1; height: 38px; border: 1px solid #2a2d37;
    background: #0f1117; color: #c0caf5; border-radius: 8px; font-size: 12px;
    padding: 0 10px; outline: none; }
  .route-row input[type=text]:focus { border-color: #7aa2f7; }

  /* ── Settings ── */
  .setting-group { margin-bottom: 12px; }
  .setting-group label { display: block; font-size: 11px; color: #787c99;
    margin-bottom: 4px; font-weight: 600; text-transform: uppercase;
    letter-spacing: 0.5px; }
  .setting-group input[type=range] { width: 100%; accent-color: #7aa2f7; }
  .setting-group .setting-val { font-size: 12px; color: #c0caf5;
    float: right; font-weight: 600; }

  /* ── Waypoint Markers (Leaflet divIcons) ── */
  .wp-marker { width: 32px; height: 32px; border-radius: 50%;
    background: #7aa2f7; color: #0f1117; font-weight: 800; font-size: 13px;
    display: flex; align-items: center; justify-content: center;
    border: 2px solid #c0caf5; box-shadow: 0 2px 8px rgba(0,0,0,0.4);
    transition: transform 0.15s; cursor: grab;
    /* 48px touch target via padding */
    padding: 8px; margin: -8px; }
  .wp-marker.completed { background: #9ece6a; }
  .wp-marker.active-target { background: #ff9e64;
    animation: pulse-marker 1.2s ease-in-out infinite; }
  .wp-marker.dragging { transform: scale(1.3); z-index: 9000 !important; }
  @keyframes pulse-marker { 0%,100% { box-shadow: 0 0 0 0 rgba(255,158,100,0.5); }
    50% { box-shadow: 0 0 0 10px rgba(255,158,100,0); } }

  /* ── Robot Marker ── */
  .robot-icon { background: transparent; border: none; }
  .robot-arrow { width: 28px; height: 28px; transition: transform 0.25s linear;
    transform-origin: 50% 50%; }
  .robot-arrow svg { width: 28px; height: 28px; display: block;
    filter: drop-shadow(0 2px 4px rgba(0,0,0,0.5)); }

  /* ── Distance Labels ── */
  .dist-label { background: none !important; border: none !important;
    box-shadow: none !important; color: #565a6e; font-size: 10px;
    font-weight: 600; white-space: nowrap; text-shadow: 0 1px 3px #0f1117; }

  /* ── Toast ── */
  .toast { position: fixed; top: calc(env(safe-area-inset-top, 0px) + 56px);
    left: 50%; transform: translateX(-50%); z-index: 1200;
    background: rgba(15,17,23,0.95); border: 1px solid #2a2d37;
    border-radius: 10px; padding: 8px 18px; font-size: 13px;
    color: #c0caf5; opacity: 0; transition: opacity 0.3s;
    pointer-events: none; }
  .toast.show { opacity: 1; }

  /* ── Nav Progress Line ── */
  .nav-progress-line { stroke: #ff9e64; stroke-width: 2; stroke-dasharray: 8 4; }

  /* hide Leaflet attribution on mobile for space */
  .leaflet-control-attribution { display: none !important; }
</style>
</head>
<body>

<!-- Mode Toolbar -->
<div class="toolbar">
  <a href="/" class="back-btn" title="Dashboard">&larr;</a>
  <button class="mode-btn active" data-mode="navigate" onclick="setMode('navigate')">
    <svg viewBox="0 0 24 24"><path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7z"/></svg>
    Navigate
  </button>
  <button class="mode-btn" data-mode="waypoint" onclick="setMode('waypoint')">
    <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"/></svg>
    Waypoint
  </button>
  <button class="mode-btn" data-mode="draw" onclick="setMode('draw')">
    <svg viewBox="0 0 24 24"><path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04a1 1 0 000-1.41l-2.34-2.34a1 1 0 00-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z"/></svg>
    Draw
  </button>
  <button class="mode-btn" data-mode="edit" onclick="setMode('edit')">
    <svg viewBox="0 0 24 24"><path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04a1 1 0 000-1.41l-2.34-2.34a1 1 0 00-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z"/></svg>
    Edit
  </button>
</div>

<!-- GPS Badge -->
<div class="gps-badge fix-none" id="gpsBadge">NO FIX</div>

<!-- Toast -->
<div class="toast" id="toast"></div>

<!-- Map -->
<div id="map"></div>

<!-- Bottom Sheet -->
<div class="bottom-sheet half" id="sheet">
  <div class="sheet-handle" id="sheetHandle"></div>
  <div class="sheet-status">
    <span class="mode-label" id="sheetMode">NAVIGATE</span>
    <span id="sheetNavStatus"></span>
    <span class="wp-count" id="wpCount">0 waypoints</span>
  </div>
  <div class="sheet-body">
    <!-- Route save/load -->
    <div class="route-row" id="routeRow">
      <select id="routeSelect"><option value="">— Load Route —</option></select>
      <button onclick="loadRoute()">Load</button>
      <input type="text" id="routeName" placeholder="Route name…" />
      <button onclick="saveRoute()">Save</button>
    </div>

    <!-- Waypoint list -->
    <ul class="wp-list" id="wpList"></ul>

    <!-- Path distance -->
    <div style="font-size:12px;color:#787c99;margin-bottom:8px;text-align:right" id="totalDist"></div>

    <!-- Actions -->
    <div class="action-row">
      <button class="btn-go" id="btnGo" onpointerdown="goDown(event)" onpointerup="goUp()" onpointerleave="goUp()">
        <span class="go-fill" id="goFill"></span>
        HOLD TO GO
      </button>
      <button class="btn-stop" id="btnStop" onclick="stopNav()">STOP</button>
      <button class="btn-skip" id="btnSkip" onclick="skipWp()">SKIP &raquo;</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" onclick="clearAll()">Clear All</button>
    </div>

    <!-- Settings (visible in full mode) -->
    <div class="sheet-settings">
      <div class="setting-group">
        <label>Cruise Speed <span class="setting-val" id="speedVal">40</span></label>
        <input type="range" id="speedSlider" min="10" max="80" value="40" oninput="document.getElementById('speedVal').textContent=this.value">
      </div>
      <div class="setting-group">
        <label>Arrival Radius <span class="setting-val" id="radiusVal">0.5 m</span></label>
        <input type="range" id="radiusSlider" min="0.3" max="3.0" step="0.1" value="0.5" oninput="document.getElementById('radiusVal').textContent=parseFloat(this.value).toFixed(1)+' m'">
      </div>
    </div>
  </div>
</div>

<script>
// ════════════════════════════════════════════════════════════════════════════
// State
// ════════════════════════════════════════════════════════════════════════════
let mode = 'navigate';           // navigate | waypoint | draw | edit
let waypoints = [];              // [{px, py, lat, lon, marker, num}]
let polyline = null;             // L.Polyline connecting waypoints
let distLabels = [];             // L.Marker distance labels
let drawVertices = [];           // temporary draw-path vertices
let drawPolyline = null;
let calibration = null;          // {pixel_to_gps, gps_to_pixel, calibrated}
let map, imageOverlay;
let imgW = 4000, imgH = 3000;   // updated from calibration
let robotMarker = null;
let robotLat = 0, robotLon = 0, robotHeading = 0;
let robotPxX = 0, robotPxY = 0;
let targetPxX = 0, targetPxY = 0;
let isAnimating = false;
let progressLine = null;         // robot → current target line
let isNavigating = false;
let isArmed = false;
let gpsFix = 0;
let gpsSats = 0;
let navWpIndex = 0, navWpTotal = 0;
let goTimer = null;
let goStart = 0;

// ════════════════════════════════════════════════════════════════════════════
// Coordinate transforms (matching geo_transform.py logic)
// ════════════════════════════════════════════════════════════════════════════
function pixelToGps(px, py) {
  if (!calibration || !calibration.calibrated) return {lat: 0, lon: 0};
  const m = calibration.pixel_to_gps; // [a,b,tx, c,d,ty] lat-row first, lon-row second
  return {
    lat: m[0]*px + m[1]*py + m[2],
    lon: m[3]*px + m[4]*py + m[5]
  };
}
function gpsToPixel(lat, lon) {
  if (!calibration || !calibration.calibrated) return {x: 0, y: 0};
  const m = calibration.gps_to_pixel; // [a,b,tx, c,d,ty]
  return {
    x: m[0]*lat + m[1]*lon + m[2],
    y: m[3]*lat + m[4]*lon + m[5]
  };
}

// Haversine distance in metres
function haversineM(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const toRad = Math.PI / 180;
  const dLat = (lat2 - lat1) * toRad;
  const dLon = (lon2 - lon1) * toRad;
  const a = Math.sin(dLat/2)**2 + Math.cos(lat1*toRad)*Math.cos(lat2*toRad)*Math.sin(dLon/2)**2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

function fmtDist(m) {
  return m < 1 ? (m*100).toFixed(0) + ' cm' : m.toFixed(1) + ' m';
}

// ════════════════════════════════════════════════════════════════════════════
// Map Initialization
// ════════════════════════════════════════════════════════════════════════════
async function initMap() {
  // Fetch calibration
  try {
    const resp = await fetch('/api/map/calibration');
    calibration = await resp.json();
  } catch(e) {
    showToast('Calibration fetch failed');
    calibration = {calibrated: false};
  }

  // Fetch map image via fetch() so we can read X-Original-* headers.
  // The endpoint downsamples large source images for bandwidth, but the
  // calibration affine is in ORIGINAL-image pixel space — Leaflet bounds
  // must match that space or gpsToPixel() lands the robot marker in the
  // wrong place.
  let imageUrl = '/map/image';
  try {
    const resp = await fetch('/map/image');
    if (!resp.ok) throw new Error('image fetch failed: ' + resp.status);
    const origW = parseInt(resp.headers.get('X-Original-Width')) || 0;
    const origH = parseInt(resp.headers.get('X-Original-Height')) || 0;
    const blob = await resp.blob();
    imageUrl = URL.createObjectURL(blob);
    const img = new Image();
    await new Promise((resolve) => {
      img.onload = resolve;
      img.onerror = resolve;
      img.src = imageUrl;
    });
    imgW = origW || img.naturalWidth;
    imgH = origH || img.naturalHeight;
  } catch(e) {
    showToast('Map image failed to load');
  }
  setupLeaflet(imageUrl);
}

function setupLeaflet(imageUrl) {
  // CRS.Simple: bounds are [[SW lat, SW lng], [NE lat, NE lng]] = [[0,0], [height, width]]
  const bounds = [[0, 0], [imgH, imgW]];

  // minZoom must be negative enough for fitBounds() to shrink a 9512x15530
  // image into a phone viewport — at minZoom=-3 (scale 1/8) the image stays
  // 1189x1941, which on a 400x700 screen shows only a center crop (and the
  // robot marker at ~21% up from the bottom falls outside that crop, making
  // it appear both "rotated" and "missing").
  map = L.map('map', {
    crs: L.CRS.Simple,
    minZoom: -6,
    maxZoom: 3,
    zoomSnap: 0.1,
    zoomDelta: 0.5,
    maxBounds: bounds,
    maxBoundsViscosity: 0.7,
    attributionControl: false,
    zoomControl: false,
  });

  imageOverlay = L.imageOverlay(imageUrl, bounds).addTo(map);
  map.fitBounds(bounds, {padding: [10, 10]});

  // Polyline connecting waypoints
  polyline = L.polyline([], {color: '#7aa2f7', weight: 2, opacity: 0.7}).addTo(map);

  // Progress line (robot → target during nav)
  progressLine = L.polyline([], {color: '#ff9e64', weight: 2, dashArray: '8 4', opacity: 0.8}).addTo(map);

  // Click handler
  map.on('click', onMapClick);

  // Create robot marker
  createRobotMarker();

  // Start SSE
  connectSSE();

  // Load route list
  refreshRouteList();
}

// ════════════════════════════════════════════════════════════════════════════
// Robot Marker
// ════════════════════════════════════════════════════════════════════════════
function createRobotMarker() {
  const icon = L.divIcon({
    className: 'robot-icon',
    html: `<div class="robot-arrow" id="robotArrow"><svg viewBox="0 0 28 28"><polygon points="14,2 24,24 14,18 4,24" fill="#7aa2f7" stroke="#c0caf5" stroke-width="1.5"/></svg></div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
  });
  robotMarker = L.marker([imgH/2, imgW/2], {icon, interactive: false, zIndexOffset: 5000}).addTo(map);
}

function updateRobotPosition(lat, lon, heading) {
  robotLat = lat; robotLon = lon; robotHeading = heading;
  const px = gpsToPixel(lat, lon);
  targetPxX = px.x; targetPxY = px.y;
  if (!isAnimating) {
    robotPxX = targetPxX;
    robotPxY = targetPxY;
    isAnimating = true;
    animateRobot();
  }
}

function animateRobot() {
  // Smooth interpolation
  const alpha = 0.2;
  robotPxX += (targetPxX - robotPxX) * alpha;
  robotPxY += (targetPxY - robotPxY) * alpha;

  if (robotMarker) {
    robotMarker.setLatLng([robotPxY, robotPxX]); // CRS.Simple: [y, x]
    const arrow = document.getElementById('robotArrow');
    if (arrow) arrow.style.transform = `rotate(${robotHeading}deg)`;
  }

  // Update progress line
  if (isNavigating && navWpIndex < waypoints.length) {
    const wp = waypoints[navWpIndex];
    progressLine.setLatLngs([[robotPxY, robotPxX], [wp.py, wp.px]]);
  } else {
    progressLine.setLatLngs([]);
  }

  if (Math.abs(targetPxX - robotPxX) > 0.1 || Math.abs(targetPxY - robotPxY) > 0.1) {
    requestAnimationFrame(animateRobot);
  } else {
    isAnimating = false;
  }
}

// ════════════════════════════════════════════════════════════════════════════
// SSE Telemetry
// ════════════════════════════════════════════════════════════════════════════
function connectSSE() {
  const evtSrc = new EventSource('/api/telemetry');
  evtSrc.onmessage = function(e) {
    try {
      const d = JSON.parse(e.data);
      const lat = d.gps_lat || 0;
      const lon = d.gps_lon || 0;
      const heading = d.imu_heading_deg || 0;
      gpsFix = d.gps_fix || 0;
      gpsSats = d.gps_sats || 0;
      isArmed = !!d.is_armed;
      const navMode = d.mode || 'MANUAL';
      isNavigating = navMode === 'WAYPOINT_NAV';
      navWpIndex = d.wp_index || 0;
      navWpTotal = d.wp_total || 0;

      if (lat !== 0 && lon !== 0) {
        updateRobotPosition(lat, lon, heading);
      }

      updateGpsBadge();
      updateNavUI();
    } catch(err) {}
  };
  evtSrc.onerror = function() {
    setTimeout(connectSSE, 3000);
    evtSrc.close();
  };
}

// ════════════════════════════════════════════════════════════════════════════
// GPS Badge
// ════════════════════════════════════════════════════════════════════════════
function updateGpsBadge() {
  const badge = document.getElementById('gpsBadge');
  badge.className = 'gps-badge';
  if (gpsFix >= 4) {
    badge.textContent = 'RTK FIXED · ' + gpsSats + ' sats';
    badge.classList.add('fix-rtk');
  } else if (gpsFix === 5 || gpsFix === 3) {
    badge.textContent = 'RTK FLOAT · ' + gpsSats + ' sats';
    badge.classList.add('fix-float');
  } else if (gpsFix >= 1) {
    badge.textContent = 'GPS · ' + gpsSats + ' sats';
    badge.classList.add('fix-gps');
  } else {
    badge.textContent = 'NO FIX';
    badge.classList.add('fix-none');
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Mode Switching
// ════════════════════════════════════════════════════════════════════════════
function setMode(m) {
  // Finalize draw path if leaving draw mode
  if (mode === 'draw' && m !== 'draw') finalizeDrawPath();

  mode = m;
  document.querySelectorAll('.mode-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.mode === m);
  });
  document.getElementById('sheetMode').textContent = m.toUpperCase();

  // Toggle waypoint dragging
  waypoints.forEach(wp => {
    if (wp.marker) wp.marker.dragging[m === 'edit' ? 'enable' : 'disable']();
  });

  // Change map cursor
  const mapEl = document.getElementById('map');
  if (m === 'waypoint' || m === 'draw') {
    mapEl.style.cursor = 'crosshair';
  } else {
    mapEl.style.cursor = '';
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Map Click → Place Waypoint / Draw Vertex
// ════════════════════════════════════════════════════════════════════════════
function onMapClick(e) {
  if (mode === 'waypoint') {
    addWaypoint(e.latlng.lng, e.latlng.lat);  // CRS.Simple: lng=x, lat=y
  } else if (mode === 'draw') {
    addDrawVertex(e.latlng.lng, e.latlng.lat);
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Waypoint Management
// ════════════════════════════════════════════════════════════════════════════
function addWaypoint(px, py) {
  const gps = pixelToGps(px, py);
  const idx = waypoints.length;
  const num = idx + 1;

  const icon = L.divIcon({
    className: '',
    html: `<div class="wp-marker" data-idx="${idx}">${num}</div>`,
    iconSize: [48, 48],
    iconAnchor: [24, 24],
  });

  const marker = L.marker([py, px], {icon, draggable: mode === 'edit', zIndexOffset: 2000}).addTo(map);

  marker.on('dragstart', function() {
    const el = marker.getElement();
    if (el) el.querySelector('.wp-marker').classList.add('dragging');
  });
  marker.on('dragend', function(ev) {
    const el = marker.getElement();
    if (el) el.querySelector('.wp-marker').classList.remove('dragging');
    const pos = ev.target.getLatLng();
    const wp = waypoints[idx];
    wp.px = pos.lng;  // CRS.Simple
    wp.py = pos.lat;
    const g = pixelToGps(wp.px, wp.py);
    wp.lat = g.lat;
    wp.lon = g.lon;
    updatePolyline();
    updateWpList();
  });

  waypoints.push({px, py, lat: gps.lat, lon: gps.lon, marker, num});
  updatePolyline();
  updateWpList();
}

function removeWaypoint(idx) {
  if (idx < 0 || idx >= waypoints.length) return;
  const wp = waypoints[idx];
  map.removeLayer(wp.marker);
  waypoints.splice(idx, 1);
  // Re-number
  waypoints.forEach((w, i) => {
    w.num = i + 1;
    const el = w.marker.getElement();
    if (el) {
      const mkr = el.querySelector('.wp-marker');
      if (mkr) { mkr.textContent = w.num; mkr.dataset.idx = i; }
    }
  });
  // Rebind drag events with correct indices
  rebindDragEvents();
  updatePolyline();
  updateWpList();
}

function rebindDragEvents() {
  waypoints.forEach((wp, idx) => {
    wp.marker.off('dragend');
    wp.marker.on('dragend', function(ev) {
      const el = wp.marker.getElement();
      if (el) el.querySelector('.wp-marker').classList.remove('dragging');
      const pos = ev.target.getLatLng();
      wp.px = pos.lng;
      wp.py = pos.lat;
      const g = pixelToGps(wp.px, wp.py);
      wp.lat = g.lat;
      wp.lon = g.lon;
      updatePolyline();
      updateWpList();
    });
  });
}

function clearAll() {
  if (waypoints.length === 0) return;
  if (!confirm('Clear all waypoints?')) return;
  waypoints.forEach(wp => map.removeLayer(wp.marker));
  waypoints = [];
  updatePolyline();
  updateWpList();
}

function updatePolyline() {
  const pts = waypoints.map(wp => [wp.py, wp.px]);
  polyline.setLatLngs(pts);
  // Distance labels
  distLabels.forEach(l => map.removeLayer(l));
  distLabels = [];
  for (let i = 0; i < waypoints.length - 1; i++) {
    const a = waypoints[i], b = waypoints[i+1];
    const d = haversineM(a.lat, a.lon, b.lat, b.lon);
    const midY = (a.py + b.py) / 2;
    const midX = (a.px + b.px) / 2;
    const label = L.marker([midY, midX], {
      icon: L.divIcon({className: 'dist-label', html: fmtDist(d), iconSize: [60, 16], iconAnchor: [30, 8]}),
      interactive: false,
    }).addTo(map);
    distLabels.push(label);
  }
  // Total distance
  let total = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    total += haversineM(waypoints[i].lat, waypoints[i].lon, waypoints[i+1].lat, waypoints[i+1].lon);
  }
  document.getElementById('totalDist').textContent = waypoints.length > 1 ? 'Total: ' + fmtDist(total) : '';
}

// ════════════════════════════════════════════════════════════════════════════
// Draw Path Mode
// ════════════════════════════════════════════════════════════════════════════
function addDrawVertex(px, py) {
  drawVertices.push({px, py});
  if (!drawPolyline) {
    drawPolyline = L.polyline([], {color: '#bb9af7', weight: 3, dashArray: '6 4'}).addTo(map);
  }
  drawPolyline.addLatLng([py, px]);

  // Place temporary circle marker at vertex
  const circ = L.circleMarker([py, px], {radius: 5, color: '#bb9af7', fillColor: '#bb9af7',
    fillOpacity: 1, weight: 0}).addTo(map);
  drawVertices[drawVertices.length - 1].circle = circ;

  if (drawVertices.length >= 2) {
    showToast('Tap another point or switch mode to finalize');
  }
}

function finalizeDrawPath() {
  if (drawVertices.length < 2) {
    // Clean up
    drawVertices.forEach(v => { if (v.circle) map.removeLayer(v.circle); });
    drawVertices = [];
    if (drawPolyline) { map.removeLayer(drawPolyline); drawPolyline = null; }
    return;
  }

  // Interpolate waypoints at ~1.5m intervals along the path
  const INTERVAL_M = 1.5;
  const pathGps = drawVertices.map(v => pixelToGps(v.px, v.py));

  // Always add the first point
  addWaypoint(drawVertices[0].px, drawVertices[0].py);

  for (let seg = 0; seg < pathGps.length - 1; seg++) {
    const aGps = pathGps[seg];
    const bGps = pathGps[seg + 1];
    const aPx = drawVertices[seg];
    const bPx = drawVertices[seg + 1];
    const segDist = haversineM(aGps.lat, aGps.lon, bGps.lat, bGps.lon);
    const steps = Math.max(1, Math.floor(segDist / INTERVAL_M));

    for (let i = 1; i <= steps; i++) {
      const t = i / steps;
      const px = aPx.px + (bPx.px - aPx.px) * t;
      const py = aPx.py + (bPx.py - aPx.py) * t;
      // Don't duplicate last point if it's close to next segment start
      if (seg < pathGps.length - 2 && i === steps) continue;
      addWaypoint(px, py);
    }
  }
  // Always add the last point
  const last = drawVertices[drawVertices.length - 1];
  addWaypoint(last.px, last.py);

  // Cleanup draw artifacts
  drawVertices.forEach(v => { if (v.circle) map.removeLayer(v.circle); });
  drawVertices = [];
  if (drawPolyline) { map.removeLayer(drawPolyline); drawPolyline = null; }

  showToast(waypoints.length + ' waypoints generated');
}

// ════════════════════════════════════════════════════════════════════════════
// Waypoint List UI
// ════════════════════════════════════════════════════════════════════════════
function updateWpList() {
  const list = document.getElementById('wpList');
  list.innerHTML = '';
  waypoints.forEach((wp, i) => {
    const li = document.createElement('li');
    if (isNavigating && i < navWpIndex) li.classList.add('completed');
    if (isNavigating && i === navWpIndex) li.classList.add('active');

    const dist = i < waypoints.length - 1
      ? haversineM(wp.lat, wp.lon, waypoints[i+1].lat, waypoints[i+1].lon) : 0;

    li.innerHTML = `
      <span class="wp-drag">&#x2630;</span>
      <span class="wp-num">${wp.num}</span>
      <span class="wp-coords">${wp.lat.toFixed(7)}, ${wp.lon.toFixed(7)}</span>
      ${dist > 0 ? '<span class="wp-dist">' + fmtDist(dist) + '</span>' : ''}
      <button class="wp-del" onclick="removeWaypoint(${i})">&times;</button>
    `;
    list.appendChild(li);
  });
  document.getElementById('wpCount').textContent = waypoints.length + ' waypoint' + (waypoints.length !== 1 ? 's' : '');
  updateGoButton();

  // Enable list drag reorder
  enableListDragReorder();
}

function enableListDragReorder() {
  const list = document.getElementById('wpList');
  let dragIdx = null;

  list.querySelectorAll('.wp-drag').forEach((handle, idx) => {
    handle.addEventListener('pointerdown', function(e) {
      e.preventDefault();
      dragIdx = idx;
      const li = handle.parentElement;
      li.style.opacity = '0.5';

      function onMove(ev) {
        // Simple swap based on pointer Y vs list items
        const items = list.querySelectorAll('li');
        for (let i = 0; i < items.length; i++) {
          const rect = items[i].getBoundingClientRect();
          if (ev.clientY < rect.top + rect.height / 2 && i < dragIdx) {
            reorderWaypoint(dragIdx, i);
            dragIdx = i;
            break;
          } else if (ev.clientY > rect.top + rect.height / 2 && i > dragIdx) {
            reorderWaypoint(dragIdx, i);
            dragIdx = i;
            break;
          }
        }
      }
      function onUp() {
        li.style.opacity = '';
        document.removeEventListener('pointermove', onMove);
        document.removeEventListener('pointerup', onUp);
      }
      document.addEventListener('pointermove', onMove);
      document.addEventListener('pointerup', onUp);
    });
  });
}

function reorderWaypoint(fromIdx, toIdx) {
  const wp = waypoints.splice(fromIdx, 1)[0];
  waypoints.splice(toIdx, 0, wp);
  waypoints.forEach((w, i) => {
    w.num = i + 1;
    const el = w.marker.getElement();
    if (el) {
      const mkr = el.querySelector('.wp-marker');
      if (mkr) { mkr.textContent = w.num; mkr.dataset.idx = i; }
    }
  });
  rebindDragEvents();
  updatePolyline();
  updateWpList();
}

// ════════════════════════════════════════════════════════════════════════════
// Navigation Controls
// ════════════════════════════════════════════════════════════════════════════
function updateGoButton() {
  const btn = document.getElementById('btnGo');
  btn.classList.remove('armed-required');

  if (isNavigating) {
    btn.style.display = 'none';
    document.getElementById('btnStop').style.display = '';
    document.getElementById('btnSkip').style.display = '';
    return;
  }

  btn.style.display = '';
  document.getElementById('btnStop').style.display = 'none';
  document.getElementById('btnSkip').style.display = 'none';

  if (waypoints.length < 1) {
    btn.disabled = true;
    btn.innerHTML = '<span class="go-fill" id="goFill"></span>TAP TO ADD WAYPOINT';
    return;
  }
  if (!isArmed) {
    btn.disabled = true;
    btn.classList.add('armed-required');
    btn.innerHTML = '<span class="go-fill" id="goFill"></span>ARM REQUIRED';
    return;
  }
  if (gpsFix < 4) {
    btn.disabled = false;  // Allow but warn
    btn.innerHTML = '<span class="go-fill" id="goFill"></span>&#x26a0; HOLD TO GO (LOW GPS)';
    return;
  }
  btn.disabled = false;
  btn.innerHTML = '<span class="go-fill" id="goFill"></span>HOLD TO GO';
}

function updateNavUI() {
  updateGoButton();
  const status = document.getElementById('sheetNavStatus');
  if (isNavigating) {
    status.textContent = `WP ${navWpIndex + 1}/${navWpTotal}`;
    // Update waypoint marker styles
    waypoints.forEach((wp, i) => {
      const el = wp.marker.getElement();
      if (!el) return;
      const mkr = el.querySelector('.wp-marker');
      if (!mkr) return;
      mkr.classList.remove('completed', 'active-target');
      if (i < navWpIndex) mkr.classList.add('completed');
      if (i === navWpIndex) mkr.classList.add('active-target');
    });
  } else {
    status.textContent = '';
  }
  updateWpList();
}

function goDown(e) {
  e.preventDefault();
  const btn = document.getElementById('btnGo');
  if (btn.disabled) return;
  goStart = Date.now();
  const fill = document.getElementById('goFill');

  goTimer = setInterval(() => {
    const elapsed = Date.now() - goStart;
    const pct = Math.min(100, (elapsed / 2000) * 100);
    if (fill) fill.style.width = pct + '%';
    if (elapsed >= 2000) {
      clearInterval(goTimer);
      goTimer = null;
      if (fill) fill.style.width = '0';
      startNav();
    }
  }, 30);
}

function goUp() {
  if (goTimer) {
    clearInterval(goTimer);
    goTimer = null;
  }
  const fill = document.getElementById('goFill');
  if (fill) fill.style.width = '0';
}

async function startNav() {
  const wpData = waypoints.map(wp => ({px: wp.px, py: wp.py, lat: wp.lat, lon: wp.lon}));
  try {
    const resp = await fetch('/api/nav/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        waypoints: wpData,
        cruise_speed: parseInt(document.getElementById('speedSlider').value),
        arrival_radius: parseFloat(document.getElementById('radiusSlider').value),
      })
    });
    const data = await resp.json();
    if (data.ok) {
      showToast('Navigation started!');
    } else {
      showToast('Error: ' + (data.error || 'unknown'));
    }
  } catch(err) {
    showToast('Failed to start navigation');
  }
}

async function stopNav() {
  try {
    await fetch('/api/nav/stop', {method: 'POST'});
    showToast('Navigation stopped');
  } catch(err) {
    showToast('Failed to stop');
  }
}

async function skipWp() {
  try {
    await fetch('/api/nav/skip', {method: 'POST'});
    showToast('Skipping to next waypoint');
  } catch(err) {
    showToast('Failed to skip');
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Route Save/Load
// ════════════════════════════════════════════════════════════════════════════
async function refreshRouteList() {
  try {
    const resp = await fetch('/api/nav/waypoints');
    const data = await resp.json();
    const sel = document.getElementById('routeSelect');
    sel.innerHTML = '<option value="">— Load Route —</option>';
    (data.routes || []).forEach(name => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      sel.appendChild(opt);
    });
  } catch(e) {}
}

async function saveRoute() {
  const name = document.getElementById('routeName').value.trim();
  if (!name) { showToast('Enter a route name'); return; }
  if (waypoints.length === 0) { showToast('No waypoints to save'); return; }

  const wpData = waypoints.map(wp => ({px: wp.px, py: wp.py, lat: wp.lat, lon: wp.lon}));
  try {
    const resp = await fetch('/api/nav/waypoints', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name, waypoints: wpData}),
    });
    const data = await resp.json();
    if (data.ok) {
      showToast('Route "' + name + '" saved');
      refreshRouteList();
      document.getElementById('routeName').value = '';
    } else {
      showToast('Error: ' + (data.error || 'unknown'));
    }
  } catch(err) {
    showToast('Failed to save route');
  }
}

async function loadRoute() {
  const name = document.getElementById('routeSelect').value;
  if (!name) { showToast('Select a route'); return; }

  try {
    const resp = await fetch('/api/nav/load', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name}),
    });
    const data = await resp.json();
    if (data.ok && data.waypoints) {
      // Clear existing
      waypoints.forEach(wp => map.removeLayer(wp.marker));
      waypoints = [];
      distLabels.forEach(l => map.removeLayer(l));
      distLabels = [];

      data.waypoints.forEach(wp => addWaypoint(wp.px, wp.py));
      showToast('Route "' + name + '" loaded (' + waypoints.length + ' waypoints)');

      // Zoom to fit
      if (waypoints.length > 0) {
        const pts = waypoints.map(wp => [wp.py, wp.px]);
        map.fitBounds(pts, {padding: [60, 60]});
      }
    } else {
      showToast('Error: ' + (data.error || 'not found'));
    }
  } catch(err) {
    showToast('Failed to load route');
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Bottom Sheet Drag
// ════════════════════════════════════════════════════════════════════════════
(function() {
  const sheet = document.getElementById('sheet');
  const handle = document.getElementById('sheetHandle');
  let startY = 0, startState = '';
  const states = ['collapsed', 'half', 'full'];

  handle.addEventListener('pointerdown', function(e) {
    e.preventDefault();
    startY = e.clientY;
    startState = states.find(s => sheet.classList.contains(s)) || 'half';
    document.addEventListener('pointermove', onMove);
    document.addEventListener('pointerup', onUp);
  });

  function onMove(e) {
    // We just detect direction; snap on release
  }

  function onUp(e) {
    document.removeEventListener('pointermove', onMove);
    document.removeEventListener('pointerup', onUp);
    const dy = e.clientY - startY;

    sheet.classList.remove('collapsed', 'half', 'full');
    if (dy > 40) {
      // Swiped down
      if (startState === 'full') sheet.classList.add('half');
      else sheet.classList.add('collapsed');
    } else if (dy < -40) {
      // Swiped up
      if (startState === 'collapsed') sheet.classList.add('half');
      else sheet.classList.add('full');
    } else {
      sheet.classList.add(startState);
    }
  }
})();

// ════════════════════════════════════════════════════════════════════════════
// Toast
// ════════════════════════════════════════════════════════════════════════════
let toastTimeout;
function showToast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimeout);
  toastTimeout = setTimeout(() => el.classList.remove('show'), 2500);
}

// ════════════════════════════════════════════════════════════════════════════
// Init
// ════════════════════════════════════════════════════════════════════════════
initMap();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Flask Blueprint
# ---------------------------------------------------------------------------

def create_nav_blueprint(controller=None) -> Blueprint:
    """Create and return the waypoint-nav Flask blueprint."""
    bp = Blueprint("waypoint_nav_ui", __name__)
    cal_path = _PROJECT_ROOT / config.property_map.calibration_path

    # Ensure routes directory exists
    _ROUTES_DIR.mkdir(parents=True, exist_ok=True)

    # ── Page ──

    @bp.route("/navigate")
    def navigate_page():
        return Response(_NAV_HTML, content_type="text/html")

    # ── List saved routes ──

    @bp.route("/api/nav/waypoints")
    def list_routes():
        routes = []
        if _ROUTES_DIR.exists():
            for f in sorted(_ROUTES_DIR.glob("*.json")):
                routes.append(f.stem)
        return _json_resp({"routes": routes})

    # ── Save route ──

    @bp.route("/api/nav/waypoints", methods=["POST"])
    def save_waypoints():
        data = request.get_json(force=True)
        name = data.get("name", "").strip()
        wps = data.get("waypoints", [])
        if not name:
            return _json_resp({"ok": False, "error": "name required"}, 400)
        if not wps:
            return _json_resp({"ok": False, "error": "no waypoints"}, 400)

        # Sanitize filename
        safe_name = "".join(c for c in name if c.isalnum() or c in '-_ ').strip()
        if not safe_name:
            return _json_resp({"ok": False, "error": "invalid name"}, 400)

        path = _ROUTES_DIR / f"{safe_name}.json"
        path.write_text(json.dumps(wps, indent=2))
        _log.info("Saved route '%s' with %d waypoints", safe_name, len(wps))
        return _json_resp({"ok": True, "name": safe_name})

    # ── Load route ──

    @bp.route("/api/nav/load", methods=["POST"])
    def load_route():
        data = request.get_json(force=True)
        name = data.get("name", "").strip()
        if not name:
            return _json_resp({"ok": False, "error": "name required"}, 400)

        safe_name = "".join(c for c in name if c.isalnum() or c in '-_ ').strip()
        path = _ROUTES_DIR / f"{safe_name}.json"
        if not path.exists():
            return _json_resp({"ok": False, "error": "route not found"}, 404)

        wps = json.loads(path.read_text())
        return _json_resp({"ok": True, "waypoints": wps})

    # ── Delete route ──

    @bp.route("/api/nav/waypoints/<name>", methods=["DELETE"])
    def delete_route(name):
        safe_name = "".join(c for c in name if c.isalnum() or c in '-_ ').strip()
        path = _ROUTES_DIR / f"{safe_name}.json"
        if path.exists():
            path.unlink()
            _log.info("Deleted route '%s'", safe_name)
            return _json_resp({"ok": True})
        return _json_resp({"ok": False, "error": "not found"}, 404)

    # ── Start navigation ──

    @bp.route("/api/nav/start", methods=["POST"])
    def start_nav():
        if controller is None:
            return _json_resp({"ok": False, "error": "no controller"}, 503)

        data = request.get_json(force=True)
        wp_list = data.get("waypoints", [])
        cruise_speed = data.get("cruise_speed", 40)
        arrival_radius = data.get("arrival_radius", 0.5)

        if len(wp_list) < 1:
            return _json_resp({"ok": False, "error": "need at least 1 waypoint"}, 400)

        # Calibration is only required if any waypoint is missing lat/lon
        # (i.e. must be re-derived from pixel coords). The UI normally sends lat/lon.
        cal = None
        needs_cal = any(wp.get("lat") is None or wp.get("lon") is None for wp in wp_list)
        if needs_cal:
            cal = load_calibration(cal_path)
            if cal is None:
                return _json_resp({"ok": False, "error": "map not calibrated"}, 400)

        # Build Waypoint list — use GPS coords (from client or re-derive from pixel)
        gps_waypoints = []
        for i, wp in enumerate(wp_list):
            lat = wp.get("lat")
            lon = wp.get("lon")
            if lat is None or lon is None:
                lat, lon = pixel_to_gps_pt(cal.pixel_to_gps, wp["px"], wp["py"])
            gps_waypoints.append(Waypoint(lat=lat, lon=lon, name=f"WP{i+1}"))

        # Configure and activate
        try:
            nav = controller._waypoint_nav
            if nav is None:
                return _json_resp({"ok": False, "error": "waypoint nav not initialized"}, 503)

            nav._cfg.cruise_speed_byte = int(cruise_speed)
            nav._cfg.arrival_radius_m = float(arrival_radius)
            nav.set_waypoints(gps_waypoints)
            controller.activate_waypoint_nav()
            _log.info("Waypoint nav started with %d waypoints, speed=%d, radius=%.1f",
                       len(gps_waypoints), cruise_speed, arrival_radius)
            return _json_resp({"ok": True, "waypoint_count": len(gps_waypoints)})
        except Exception as exc:
            _log.exception("Failed to start waypoint nav")
            return _json_resp({"ok": False, "error": str(exc)}, 500)

    # ── Stop navigation ──

    @bp.route("/api/nav/stop", methods=["POST"])
    def stop_nav():
        if controller is None:
            return _json_resp({"ok": False, "error": "no controller"}, 503)
        try:
            controller.deactivate_waypoint_nav()
            _log.info("Waypoint nav stopped")
            return _json_resp({"ok": True})
        except Exception as exc:
            _log.exception("Failed to stop waypoint nav")
            return _json_resp({"ok": False, "error": str(exc)}, 500)

    # ── Skip waypoint ──

    @bp.route("/api/nav/skip", methods=["POST"])
    def skip_waypoint():
        if controller is None:
            return _json_resp({"ok": False, "error": "no controller"}, 503)
        try:
            nav = controller._waypoint_nav
            if nav is None:
                return _json_resp({"ok": False, "error": "waypoint nav not initialized"}, 503)
            if nav._index < len(nav._waypoints) - 1:
                nav._index += 1
                _log.info("Skipped to waypoint %d", nav._index)
                return _json_resp({"ok": True, "index": nav._index})
            else:
                return _json_resp({"ok": False, "error": "already at last waypoint"})
        except Exception as exc:
            _log.exception("Failed to skip waypoint")
            return _json_resp({"ok": False, "error": str(exc)}, 500)

    return bp
