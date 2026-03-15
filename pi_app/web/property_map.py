"""
Property map web UI — Flask Blueprint.

Provides an interactive satellite/aerial-image overlay with live GPS tracking,
pan/zoom, breadcrumb trails, and a calibration panel for setting up the
pixel ↔ GPS affine transform.

Routes:
  GET  /map                 — main interactive map page
  GET  /map/image           — serve the property JPEG (downsampled, cached)
  GET  /api/map/calibration — current calibration JSON
  POST /api/map/calibration — compute + save affine from control points
  POST /api/map/record_gps  — snapshot current GPS reading
  POST /api/map/goto        — (v2 stub) pixel→GPS navigate
"""

from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from typing import Optional

try:
    from flask import Blueprint, Response, request, send_file
except ImportError:
    Blueprint = None  # type: ignore[assignment,misc]

import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.control.geo_transform import (
    ControlPoint,
    MapCalibration,
    compute_affine,
    save_calibration,
    load_calibration,
    pixel_to_gps_pt,
    gps_to_pixel_pt,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_log = logging.getLogger(__name__)


# ───────────────────────────── Flask Blueprint ────────────────────────────────

def create_map_blueprint(recorder=None, controller=None) -> Blueprint:
    """Create and return the property-map Flask blueprint."""
    bp = Blueprint("property_map", __name__)

    image_path = _PROJECT_ROOT / config.property_map.image_path
    cal_path = _PROJECT_ROOT / config.property_map.calibration_path
    web_cache = _PROJECT_ROOT / ".map_web.jpg"
    max_w = config.property_map.max_serve_width

    # ── Page ──

    @bp.route("/map")
    def map_index():
        return Response(_MAP_HTML, content_type="text/html")

    # ── Image serving ──

    @bp.route("/map/image")
    def map_image():
        if not image_path.exists():
            return Response("Property map image not found", status=404)

        try:
            from PIL import Image as PILImage
        except ImportError:
            return Response("Pillow not installed", status=500)

        # Rebuild cache if stale or missing
        rebuild = False
        if not web_cache.exists():
            rebuild = True
        else:
            src_mtime = os.path.getmtime(str(image_path))
            cache_mtime = os.path.getmtime(str(web_cache))
            if cache_mtime < src_mtime:
                rebuild = True

        if rebuild:
            try:
                img = PILImage.open(str(image_path))
                orig_w, orig_h = img.size
                if orig_w > max_w:
                    ratio = max_w / orig_w
                    new_h = int(orig_h * ratio)
                    img = img.resize((max_w, new_h), PILImage.LANCZOS)
                img.save(str(web_cache), "JPEG", quality=85)
            except Exception as exc:
                _log.error("Failed to build map web cache: %s", exc)
                return Response(f"Image processing error: {exc}", status=500)

        # Read original dimensions (from source) for header
        try:
            orig_img = PILImage.open(str(image_path))
            orig_w, orig_h = orig_img.size
            orig_img.close()
        except Exception:
            orig_w, orig_h = 0, 0

        resp = send_file(str(web_cache), mimetype="image/jpeg")
        resp.headers["X-Original-Width"] = str(orig_w)
        resp.headers["X-Original-Height"] = str(orig_h)
        resp.headers["Access-Control-Expose-Headers"] = "X-Original-Width, X-Original-Height"
        return resp

    # ── Calibration GET ──

    @bp.route("/api/map/calibration")
    def get_calibration():
        cal = load_calibration(cal_path)
        if cal is None:
            return _json_resp({"calibrated": False})
        return _json_resp({
            "calibrated": True,
            "pixel_to_gps": list(cal.pixel_to_gps),
            "gps_to_pixel": list(cal.gps_to_pixel),
            "residuals": cal.residuals,
            "rms_error_m": cal.rms_error_m,
            "control_points": [
                {"pixel_x": p.pixel_x, "pixel_y": p.pixel_y,
                 "lat": p.lat, "lon": p.lon}
                for p in cal.control_points
            ],
        })

    # ── Calibration POST ──

    @bp.route("/api/map/calibration", methods=["POST"])
    def post_calibration():
        data = request.get_json(force=True, silent=True)
        if not data or "points" not in data:
            return _json_resp({"error": "Missing 'points' array"}, 400)

        try:
            points = [ControlPoint(**p) for p in data["points"]]
        except (TypeError, KeyError) as exc:
            return _json_resp({"error": f"Invalid control point: {exc}"}, 400)

        if len(points) < 3:
            return _json_resp({"error": "Need at least 3 control points"}, 400)

        try:
            cal = compute_affine(points)
        except ValueError as exc:
            return _json_resp({"error": str(exc)}, 400)

        # Get image dimensions for saving
        img_w, img_h = 0, 0
        try:
            from PIL import Image as PILImage
            img = PILImage.open(str(image_path))
            img_w, img_h = img.size
            img.close()
        except Exception:
            pass

        try:
            save_calibration(cal_path, cal, config.property_map.image_path,
                             img_w, img_h)
        except Exception as exc:
            _log.error("Failed to save calibration: %s", exc)
            return _json_resp({"error": f"Save failed: {exc}"}, 500)

        return _json_resp({
            "pixel_to_gps": list(cal.pixel_to_gps),
            "gps_to_pixel": list(cal.gps_to_pixel),
            "residuals": cal.residuals,
            "rms_error_m": cal.rms_error_m,
        })

    # ── Record GPS ──

    @bp.route("/api/map/record_gps", methods=["POST"])
    def record_gps():
        t = recorder.get_latest_telemetry() if recorder else None
        if t is None or t.gps_lat is None:
            return _json_resp({"error": "No GPS reading available"}, 503)

        result = {
            "lat": t.gps_lat,
            "lon": t.gps_lon,
            "fix_quality": t.gps_fix,
            "hdop": t.gps_hdop,
            "sats": t.gps_sats,
        }

        if t.gps_fix is not None and t.gps_fix < 4:
            result["warning"] = "GPS quality below RTK fixed"

        return _json_resp(result)

    # ── Go-to (v2 stub) ──

    @bp.route("/api/map/goto", methods=["POST"])
    def goto_point():
        data = request.get_json(force=True, silent=True)
        if not data or "pixel_x" not in data or "pixel_y" not in data:
            return _json_resp({"error": "Missing pixel_x/pixel_y"}, 400)

        cal = load_calibration(cal_path)
        if cal is None:
            return _json_resp({"error": "Map not calibrated"}, 400)

        lat, lon = pixel_to_gps_pt(cal.pixel_to_gps,
                                   float(data["pixel_x"]),
                                   float(data["pixel_y"]))
        return _json_resp({
            "lat": lat,
            "lon": lon,
            "status": "not_implemented",
        })

    return bp


def _json_resp(obj, status=200):
    return Response(json.dumps(obj), status=status,
                    content_type="application/json")


# ───────────────────────────── HTML / JS ──────────────────────────────────────

_MAP_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>WALL-E Mini — Property Map</title>
<style>
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
html, body { width: 100%; height: 100%; overflow: hidden; }
body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    background: #1a1b26; color: #c0caf5;
}

/* ── Top bar ── */
#topbar {
    position: fixed; top: 0; left: 0; right: 0; z-index: 100;
    height: 44px; background: rgba(26,27,38,0.92); backdrop-filter: blur(8px);
    display: flex; align-items: center; justify-content: space-between;
    padding: 0 16px; border-bottom: 1px solid #292e42;
}
#topbar h1 { font-size: 15px; font-weight: 600; color: #c0caf5; white-space: nowrap; }
#topbar a { color: #7aa2f7; text-decoration: none; font-size: 13px; margin-right: 12px; }
#topbar a:hover { text-decoration: underline; }
.topbar-left { display: flex; align-items: center; gap: 12px; }
.topbar-right { display: flex; align-items: center; gap: 10px; }

/* ── GPS status chip ── */
#gps-chip {
    font-size: 11px; font-weight: 600; padding: 3px 10px; border-radius: 10px;
    display: flex; align-items: center; gap: 6px; background: #292e42;
}
#gps-dot { width: 8px; height: 8px; border-radius: 50%; background: #f7768e; flex-shrink: 0; }
#gps-text { white-space: nowrap; }

.btn-topbar {
    font-size: 12px; font-weight: 600; padding: 4px 12px; border-radius: 6px;
    border: 1px solid #3b4261; background: #292e42; color: #7aa2f7;
    cursor: pointer; transition: background .15s;
}
.btn-topbar:hover { background: #3b4261; }
.btn-topbar.active { background: #7aa2f7; color: #1a1b26; border-color: #7aa2f7; }

/* ── Canvas ── */
#mapCanvas {
    position: fixed; top: 44px; left: 0; right: 0; bottom: 0;
    display: block; cursor: grab;
}
#mapCanvas.crosshair { cursor: crosshair; }
#mapCanvas:active { cursor: grabbing; }

/* ── Calibration panel ── */
#calPanel {
    position: fixed; top: 44px; right: -340px; bottom: 0; width: 330px;
    background: rgba(26,27,38,0.96); backdrop-filter: blur(10px);
    border-left: 1px solid #292e42; z-index: 90;
    transition: right .25s ease; overflow-y: auto;
    padding: 16px; font-size: 13px;
}
#calPanel.open { right: 0; }
#calPanel h3 { font-size: 14px; color: #7aa2f7; margin-bottom: 10px; }
#calPanel .section { margin-bottom: 16px; }
#calPanel .instructions {
    font-size: 12px; color: #787c99; line-height: 1.6; margin-bottom: 12px;
}

.btn {
    display: inline-block; padding: 7px 16px; border-radius: 6px; border: none;
    font-size: 12px; font-weight: 600; cursor: pointer; transition: background .15s;
}
.btn-primary { background: #7aa2f7; color: #1a1b26; }
.btn-primary:hover { background: #89b4fa; }
.btn-primary:disabled { background: #3b4261; color: #565f89; cursor: default; }
.btn-danger { background: #f7768e; color: #1a1b26; }
.btn-danger:hover { background: #ff9e9e; }
.btn-small { padding: 3px 8px; font-size: 11px; }
.btn-group { display: flex; gap: 8px; margin-bottom: 10px; flex-wrap: wrap; }

#gpsResult {
    background: #292e42; border-radius: 6px; padding: 8px 10px; margin: 8px 0;
    font-family: 'SF Mono', 'Fira Code', monospace; font-size: 11px;
    display: none; line-height: 1.5;
}
#gpsResult .warn { color: #e0af68; }

/* ── Control point list ── */
#cpList { list-style: none; margin: 8px 0; }
#cpList li {
    background: #292e42; border-radius: 6px; padding: 6px 10px; margin-bottom: 4px;
    display: flex; justify-content: space-between; align-items: center;
    font-family: monospace; font-size: 11px;
}
#cpList li .cp-info { flex: 1; }
#cpList li .cp-del { cursor: pointer; color: #f7768e; font-weight: 700; padding: 0 4px; }
#cpList li .cp-del:hover { color: #ff9e9e; }

#calResult {
    background: #292e42; border-radius: 6px; padding: 10px; margin-top: 8px;
    font-family: monospace; font-size: 11px; line-height: 1.6; display: none;
}
#calResult .rms { font-weight: 700; }
#calResult .rms.good { color: #9ece6a; }
#calResult .rms.warn { color: #e0af68; }
#calResult .rms.bad { color: #f7768e; }
</style>
</head>
<body>

<!-- ── Top bar ── -->
<div id="topbar">
  <div class="topbar-left">
    <a href="/">&larr; Dashboard</a>
    <h1>Property Map</h1>
  </div>
  <div class="topbar-right">
    <div id="gps-chip"><span id="gps-dot"></span><span id="gps-text">No Fix</span></div>
    <button class="btn-topbar" id="btnCalToggle">Calibrate</button>
  </div>
</div>

<!-- ── Canvas ── -->
<canvas id="mapCanvas"></canvas>

<!-- ── Calibration panel ── -->
<div id="calPanel">
  <h3>Map Calibration</h3>
  <div class="instructions">
    For each control point:<br>
    1. Click <b>Record GPS</b> to capture the rover's current position.<br>
    2. Click the corresponding spot on the map image.<br>
    3. Repeat for at least 3 points (4+ recommended).<br>
    4. Click <b>Compute</b> to calculate the transform.
  </div>

  <div class="section">
    <div class="btn-group">
      <button class="btn btn-primary" id="btnRecordGps">Record GPS</button>
    </div>
    <div id="gpsResult"></div>
  </div>

  <div class="section">
    <h3>Control Points</h3>
    <ul id="cpList"></ul>
  </div>

  <div class="btn-group">
    <button class="btn btn-primary" id="btnCompute" disabled>Compute</button>
    <button class="btn btn-danger btn-small" id="btnClearAll">Clear All</button>
  </div>
  <div id="calResult"></div>
</div>

<script>
(function() {
"use strict";

/* ── Config ── */
const MAX_TRAIL = """ + str(config.property_map.trail_max_points) + r""";

/* ── State ── */
let mapImg = null;
let imgNatW = 0, imgNatH = 0;       // original full-res pixel dimensions
let imgDispW = 0, imgDispH = 0;     // served (possibly downsampled) dimensions
let calibration = null;              // {gps_to_pixel: [6], pixel_to_gps: [6]}
let wallePos = null;                 // {px, py, heading}  in original-image pixels
let trail = [];
let gpsStatus = {fix: null, hdop: null, sats: null};

// Pan / zoom
let zoom = 1;
let panX = 0, panY = 0;

// Calibration mode
let calOpen = false;
let pendingGps = null;       // {lat, lon} waiting for pixel click
let controlPoints = [];      // [{lat, lon, pixel_x, pixel_y}]
let awaitingPixelClick = false;

/* ── DOM ── */
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const calPanel = document.getElementById('calPanel');
const btnCalToggle = document.getElementById('btnCalToggle');
const btnRecordGps = document.getElementById('btnRecordGps');
const btnCompute = document.getElementById('btnCompute');
const btnClearAll = document.getElementById('btnClearAll');
const gpsResultEl = document.getElementById('gpsResult');
const cpListEl = document.getElementById('cpList');
const calResultEl = document.getElementById('calResult');
const gpsDot = document.getElementById('gps-dot');
const gpsTextEl = document.getElementById('gps-text');

/* ── Canvas sizing ── */
function resizeCanvas() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight - 44;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

/* ── Load image ── */
function loadMapImage() {
    fetch('/map/image').then(resp => {
        if (!resp.ok) return null;
        imgNatW = parseInt(resp.headers.get('X-Original-Width')) || 0;
        imgNatH = parseInt(resp.headers.get('X-Original-Height')) || 0;
        return resp.blob();
    }).then(blob => {
        if (!blob) return;
        const url = URL.createObjectURL(blob);
        const img = new Image();
        img.onload = function() {
            mapImg = img;
            imgDispW = img.naturalWidth;
            imgDispH = img.naturalHeight;
            if (!imgNatW) imgNatW = imgDispW;
            if (!imgNatH) imgNatH = imgDispH;
            fitToViewport();
        };
        img.src = url;
    }).catch(() => {});
}

function fitToViewport() {
    if (!mapImg) return;
    zoom = Math.min(canvas.width / imgDispW, canvas.height / imgDispH);
    panX = (canvas.width - imgDispW * zoom) / 2;
    panY = (canvas.height - imgDispH * zoom) / 2;
}

/* ── Load calibration ── */
function loadCalibration() {
    fetch('/api/map/calibration').then(r => r.json()).then(d => {
        if (d.calibrated) {
            calibration = {
                gps_to_pixel: d.gps_to_pixel,
                pixel_to_gps: d.pixel_to_gps,
            };
            if (d.control_points) {
                controlPoints = d.control_points.map(p => ({
                    lat: p.lat, lon: p.lon,
                    pixel_x: p.pixel_x, pixel_y: p.pixel_y,
                }));
                renderCpList();
            }
        }
    }).catch(() => {});
}

/* ── Affine helpers ── */
function gpsToPixel(lat, lon) {
    // Returns position in ORIGINAL image pixel space
    const m = calibration.gps_to_pixel;
    return [m[0]*lat + m[1]*lon + m[2], m[3]*lat + m[4]*lon + m[5]];
}

function origToDisp(ox, oy) {
    // Original image pixels -> displayed (possibly downsampled) pixels
    const sx = imgDispW / imgNatW;
    const sy = imgDispH / imgNatH;
    return [ox * sx, oy * sy];
}

function dispToOrig(dx, dy) {
    const sx = imgNatW / imgDispW;
    const sy = imgNatH / imgDispH;
    return [dx * sx, dy * sy];
}

function dispToCanvas(dx, dy) {
    return [dx * zoom + panX, dy * zoom + panY];
}

function canvasToDisp(cx, cy) {
    return [(cx - panX) / zoom, (cy - panY) / zoom];
}

/* ── SSE Telemetry ── */
let sse = null;
function connectSSE() {
    if (sse) sse.close();
    sse = new EventSource('/api/telemetry');
    sse.onmessage = function(e) {
        let d;
        try { d = JSON.parse(e.data); } catch(_) { return; }

        if (calibration && d.gps_lat != null && d.gps_lon != null) {
            const [opx, opy] = gpsToPixel(d.gps_lat, d.gps_lon);
            wallePos = {px: opx, py: opy, heading: d.imu_heading_deg || 0};
            trail.push({px: opx, py: opy});
            if (trail.length > MAX_TRAIL) trail.shift();
        }
        updateGpsStatus(d);
    };
    sse.onerror = function() {
        sse.close();
        setTimeout(connectSSE, 3000);
    };
}

function updateGpsStatus(d) {
    const fix = d.gps_fix != null ? d.gps_fix : null;
    const hdop = d.gps_hdop != null ? d.gps_hdop : null;
    const sats = d.gps_sats != null ? d.gps_sats : null;
    gpsStatus = {fix, hdop, sats};

    let label, color;
    if (fix === null || fix === 0) {
        label = 'No Fix'; color = '#f7768e';
    } else if (fix >= 4) {
        label = fix === 5 ? 'RTK Float' : 'RTK Fixed'; color = '#9ece6a';
        if (fix === 4) { label = 'RTK Fixed'; color = '#9ece6a'; }
        if (fix === 5) { label = 'RTK Float'; color = '#e0af68'; }
    } else {
        label = 'GPS'; color = '#e0af68';
    }
    if (hdop != null) label += '  HDOP:' + hdop.toFixed(1);
    if (sats != null) label += '  Sats:' + sats;

    gpsDot.style.background = color;
    gpsTextEl.textContent = label;
}

/* ── Render loop ── */
function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw map image
    if (mapImg) {
        ctx.save();
        ctx.translate(panX, panY);
        ctx.scale(zoom, zoom);
        ctx.drawImage(mapImg, 0, 0, imgDispW, imgDispH);
        ctx.restore();
    }

    // Draw calibration points
    if (calOpen && controlPoints.length > 0) {
        controlPoints.forEach((cp, i) => {
            const [dx, dy] = origToDisp(cp.pixel_x, cp.pixel_y);
            const [cx, cy] = dispToCanvas(dx, dy);
            drawCalibrationMarker(cx, cy, i + 1);
        });
    }

    // Draw trail
    if (trail.length > 1) {
        ctx.save();
        ctx.strokeStyle = 'rgba(80, 250, 123, 0.4)';
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let i = 0; i < trail.length; i++) {
            const [dx, dy] = origToDisp(trail[i].px, trail[i].py);
            const [cx, cy] = dispToCanvas(dx, dy);
            if (i === 0) ctx.moveTo(cx, cy);
            else ctx.lineTo(cx, cy);
        }
        ctx.stroke();
        ctx.restore();
    }

    // Draw Wall-E marker
    if (wallePos) {
        const [dx, dy] = origToDisp(wallePos.px, wallePos.py);
        const [cx, cy] = dispToCanvas(dx, dy);
        drawWalleMarker(cx, cy, wallePos.heading);
    }

    requestAnimationFrame(render);
}

function drawCalibrationMarker(cx, cy, num) {
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, 10, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(122,162,247,0.7)';
    ctx.fill();
    ctx.strokeStyle = '#1a1b26';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(num), cx, cy);
    ctx.restore();
}

function drawWalleMarker(cx, cy, headingDeg) {
    // heading: 0=north(up), 90=east(right), CW positive
    // Canvas: 0 rad = right, so rotate by heading - 90 degrees
    const rad = (headingDeg - 90) * Math.PI / 180;
    const size = 20;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(rad);

    // Arrow / triangle pointing right (will be rotated)
    ctx.beginPath();
    ctx.moveTo(size * 0.6, 0);              // tip
    ctx.lineTo(-size * 0.4, -size * 0.35);  // top-left
    ctx.lineTo(-size * 0.2, 0);             // notch
    ctx.lineTo(-size * 0.4, size * 0.35);   // bottom-left
    ctx.closePath();

    ctx.fillStyle = '#50fa7b';
    ctx.fill();
    ctx.strokeStyle = '#1a1b26';
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.restore();

    // GPS quality dot
    const dotColor = gpsStatus.fix === null || gpsStatus.fix === 0 ? '#f7768e'
                   : gpsStatus.fix >= 4 ? '#9ece6a' : '#e0af68';
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fillStyle = dotColor;
    ctx.fill();
    ctx.restore();
}

/* ── Pan / Zoom (mouse) ── */
let dragging = false;
let dragStartX = 0, dragStartY = 0;
let panStartX = 0, panStartY = 0;

canvas.addEventListener('mousedown', function(e) {
    if (awaitingPixelClick) return; // handled in click
    dragging = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
    panStartX = panX;
    panStartY = panY;
});

window.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    panX = panStartX + (e.clientX - dragStartX);
    panY = panStartY + (e.clientY - dragStartY);
});

window.addEventListener('mouseup', function() {
    dragging = false;
});

canvas.addEventListener('wheel', function(e) {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const oldZoom = zoom;
    const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
    zoom = Math.max(0.1, Math.min(10, zoom * factor));
    // Zoom around cursor
    panX = mx - (mx - panX) * (zoom / oldZoom);
    panY = my - (my - panY) * (zoom / oldZoom);
}, {passive: false});

/* ── Pan / Zoom (touch) ── */
let touches = {};
let lastPinchDist = 0;
let lastTouchPanX = 0, lastTouchPanY = 0;

canvas.addEventListener('touchstart', function(e) {
    e.preventDefault();
    for (let t of e.changedTouches) touches[t.identifier] = {x: t.clientX, y: t.clientY};
    const ids = Object.keys(touches);
    if (ids.length === 1) {
        const t = touches[ids[0]];
        lastTouchPanX = t.x; lastTouchPanY = t.y;
    } else if (ids.length >= 2) {
        const a = touches[ids[0]], b = touches[ids[1]];
        lastPinchDist = Math.hypot(a.x - b.x, a.y - b.y);
    }
}, {passive: false});

canvas.addEventListener('touchmove', function(e) {
    e.preventDefault();
    for (let t of e.changedTouches) {
        if (touches[t.identifier]) touches[t.identifier] = {x: t.clientX, y: t.clientY};
    }
    const ids = Object.keys(touches);
    if (ids.length === 1) {
        const t = touches[ids[0]];
        panX += t.x - lastTouchPanX;
        panY += t.y - lastTouchPanY;
        lastTouchPanX = t.x; lastTouchPanY = t.y;
    } else if (ids.length >= 2) {
        const a = touches[ids[0]], b = touches[ids[1]];
        const dist = Math.hypot(a.x - b.x, a.y - b.y);
        if (lastPinchDist > 0) {
            const midX = (a.x + b.x) / 2;
            const midY = (a.y + b.y) / 2 - 44;
            const oldZoom = zoom;
            zoom = Math.max(0.1, Math.min(10, zoom * (dist / lastPinchDist)));
            panX = midX - (midX - panX) * (zoom / oldZoom);
            panY = midY - (midY - panY) * (zoom / oldZoom);
        }
        lastPinchDist = dist;
    }
}, {passive: false});

canvas.addEventListener('touchend', function(e) {
    for (let t of e.changedTouches) delete touches[t.identifier];
    if (Object.keys(touches).length < 2) lastPinchDist = 0;
});

/* ── Canvas click (calibration pixel pick) ── */
canvas.addEventListener('click', function(e) {
    if (!awaitingPixelClick || !pendingGps) return;

    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    // Canvas coords -> display-image coords -> original-image coords
    const [dx, dy] = canvasToDisp(cx, cy);
    const [ox, oy] = dispToOrig(dx, dy);

    // Bounds check
    if (ox < 0 || oy < 0 || ox > imgNatW || oy > imgNatH) return;

    controlPoints.push({
        lat: pendingGps.lat,
        lon: pendingGps.lon,
        pixel_x: Math.round(ox * 100) / 100,
        pixel_y: Math.round(oy * 100) / 100,
    });

    pendingGps = null;
    awaitingPixelClick = false;
    canvas.classList.remove('crosshair');
    renderCpList();
    updateComputeBtn();

    gpsResultEl.innerHTML += '<br><span style="color:#9ece6a">Pixel recorded. Point added.</span>';
});

/* ── Calibration panel toggle ── */
btnCalToggle.addEventListener('click', function() {
    calOpen = !calOpen;
    calPanel.classList.toggle('open', calOpen);
    btnCalToggle.classList.toggle('active', calOpen);
});

/* ── Record GPS ── */
btnRecordGps.addEventListener('click', function() {
    btnRecordGps.disabled = true;
    gpsResultEl.style.display = 'block';
    gpsResultEl.textContent = 'Reading GPS...';

    fetch('/api/map/record_gps', {method: 'POST'})
    .then(r => r.json())
    .then(d => {
        btnRecordGps.disabled = false;
        if (d.error) {
            gpsResultEl.innerHTML = '<span style="color:#f7768e">' + d.error + '</span>';
            return;
        }
        let html = 'Lat: ' + d.lat.toFixed(8) + '<br>Lon: ' + d.lon.toFixed(8);
        html += '<br>Fix: ' + (d.fix_quality != null ? d.fix_quality : '?');
        if (d.hdop != null) html += '  HDOP: ' + d.hdop;
        if (d.sats != null) html += '  Sats: ' + d.sats;
        if (d.warning) html += '<br><span class="warn">' + d.warning + '</span>';
        html += '<br><br><b>Now click on the map where the rover is.</b>';
        gpsResultEl.innerHTML = html;

        pendingGps = {lat: d.lat, lon: d.lon};
        awaitingPixelClick = true;
        canvas.classList.add('crosshair');
    })
    .catch(() => {
        btnRecordGps.disabled = false;
        gpsResultEl.innerHTML = '<span style="color:#f7768e">Request failed</span>';
    });
});

/* ── Control point list rendering ── */
function renderCpList() {
    cpListEl.innerHTML = '';
    controlPoints.forEach((cp, i) => {
        const li = document.createElement('li');
        const info = document.createElement('span');
        info.className = 'cp-info';
        info.textContent = (i+1) + '. (' + cp.lat.toFixed(6) + ', ' + cp.lon.toFixed(6)
            + ') \u2192 px(' + cp.pixel_x.toFixed(0) + ', ' + cp.pixel_y.toFixed(0) + ')';
        const del = document.createElement('span');
        del.className = 'cp-del';
        del.textContent = '\u2715';
        del.title = 'Delete';
        del.addEventListener('click', function() {
            controlPoints.splice(i, 1);
            renderCpList();
            updateComputeBtn();
        });
        li.appendChild(info);
        li.appendChild(del);
        cpListEl.appendChild(li);
    });
}

function updateComputeBtn() {
    btnCompute.disabled = controlPoints.length < 3;
}

/* ── Compute calibration ── */
btnCompute.addEventListener('click', function() {
    btnCompute.disabled = true;
    calResultEl.style.display = 'block';
    calResultEl.textContent = 'Computing...';

    const pts = controlPoints.map(p => ({
        pixel_x: p.pixel_x, pixel_y: p.pixel_y,
        lat: p.lat, lon: p.lon,
    }));

    fetch('/api/map/calibration', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({points: pts}),
    })
    .then(r => r.json())
    .then(d => {
        btnCompute.disabled = false;
        updateComputeBtn();
        if (d.error) {
            calResultEl.innerHTML = '<span style="color:#f7768e">' + d.error + '</span>';
            return;
        }

        calibration = {
            gps_to_pixel: d.gps_to_pixel,
            pixel_to_gps: d.pixel_to_gps,
        };
        trail = [];

        let html = '<b>Calibration saved.</b><br><br>';
        const rms = d.rms_error_m;
        const rmsCls = rms < 1 ? 'good' : rms < 3 ? 'warn' : 'bad';
        html += 'RMS error: <span class="rms ' + rmsCls + '">'
              + rms.toFixed(3) + ' m</span><br><br>';

        html += 'Per-point residuals:<br>';
        d.residuals.forEach((r, i) => {
            html += '  #' + (i+1) + ': ' + r.toFixed(3) + ' m<br>';
        });

        calResultEl.innerHTML = html;
    })
    .catch(() => {
        btnCompute.disabled = false;
        updateComputeBtn();
        calResultEl.innerHTML = '<span style="color:#f7768e">Request failed</span>';
    });
});

/* ── Clear all ── */
btnClearAll.addEventListener('click', function() {
    controlPoints = [];
    pendingGps = null;
    awaitingPixelClick = false;
    canvas.classList.remove('crosshair');
    renderCpList();
    updateComputeBtn();
    gpsResultEl.style.display = 'none';
    calResultEl.style.display = 'none';
});

/* ── Init ── */
loadMapImage();
loadCalibration();
connectSSE();
requestAnimationFrame(render);

})();
</script>
</body>
</html>"""
