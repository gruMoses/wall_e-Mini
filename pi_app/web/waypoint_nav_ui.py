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
import math
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

  .mission-strip { display: flex; gap: 6px; padding: 0 12px 8px; flex-wrap: wrap; }
  .mission-chip { border: 1px solid #2a2d37; border-radius: 999px; padding: 3px 8px;
    font-size: 10px; font-weight: 700; letter-spacing: 0.4px; color: #787c99;
    background: #0f1117; text-transform: uppercase; }
  .mission-chip.ok { color: #9ece6a; border-color: #9ece6a; }
  .mission-chip.warn { color: #e0af68; border-color: #e0af68; }
  .mission-chip.err { color: #f7768e; border-color: #f7768e; }
  .mission-timeline { padding: 0 12px 8px; display: flex; align-items: center; gap: 4px;
    overflow-x: auto; scrollbar-width: thin; }
  .timeline-node { min-width: 18px; height: 18px; border-radius: 999px;
    border: 1px solid #2a2d37; background: #0f1117; color: #787c99;
    font-size: 10px; font-weight: 700; display: inline-flex; align-items: center;
    justify-content: center; }
  .timeline-node.done { border-color: #9ece6a; color: #9ece6a; }
  .timeline-node.active { border-color: #ff9e64; color: #ff9e64; background: rgba(255,158,100,0.12); }
  .timeline-node.pending { border-color: #3b3f54; color: #565a6e; }
  .timeline-link { min-width: 14px; height: 2px; background: #3b3f54; border-radius: 1px; }
  .timeline-link.done { background: #9ece6a; }

  .sheet-body { padding: 0 12px 12px; overflow-y: auto; flex: 1;
    -webkit-overflow-scrolling: touch; display: none; }
  .bottom-sheet.half .sheet-body,
  .bottom-sheet.full .sheet-body { display: block; }

  .sheet-settings { display: none; padding: 8px 0; }
  .bottom-sheet.full .sheet-settings { display: block; }

  /* ── Waypoint List ── */
  .wp-list { list-style: none; margin: 0 0 10px; }
  .wp-list li { display: flex; align-items: center; gap: 8px;
    padding: 10px 8px; background: #0f1117; border: 1px solid #252a36; border-radius: 8px;
    margin-bottom: 4px; font-size: 13px; touch-action: manipulation; }
  .wp-main { flex: 1; display: flex; flex-direction: column; gap: 2px; min-width: 0; }
  .wp-main-top { display: flex; align-items: center; gap: 8px; }
  .wp-main-bottom { display: flex; align-items: center; gap: 8px; color: #565a6e; font-size: 10px;
    font-family: 'SF Mono', 'Fira Code', monospace; }
  .wp-list li .wp-num { width: 26px; height: 26px; border-radius: 50%;
    background: #7aa2f7; color: #0f1117; font-weight: 700; font-size: 12px;
    display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
  .wp-list li .wp-coords { flex: 1; color: #787c99; font-size: 11px;
    font-family: 'SF Mono', 'Fira Code', monospace; }
  .wp-list li .wp-dist { color: #565a6e; font-size: 11px; white-space: nowrap; min-width: 48px; text-align: right; }
  .wp-list li .wp-state { color: #787c99; border: 1px solid #2a2d37;
    border-radius: 10px; font-size: 9px; font-weight: 700; letter-spacing: 0.3px;
    padding: 2px 6px; text-transform: uppercase; }
  .wp-list li .wp-state.active { color: #ff9e64; border-color: #ff9e64; }
  .wp-list li .wp-state.done { color: #9ece6a; border-color: #9ece6a; }
  .wp-list li .wp-del { width: 28px; height: 28px; border: none;
    background: transparent; color: #f7768e; font-size: 16px; cursor: pointer;
    border-radius: 6px; display: flex; align-items: center; justify-content: center; }
  .wp-list li .wp-del:active { background: rgba(247,118,142,0.15); }
  .wp-list li.completed .wp-num { background: #9ece6a; }
  .wp-list li.active .wp-num { background: #ff9e64;
    animation: pulse-wp 1.2s ease-in-out infinite; }
  .wp-list li .wp-drag { cursor: grab; color: #565a6e; font-size: 17px;
    width: 24px; height: 24px; display: inline-flex; align-items: center; justify-content: center;
    border-radius: 6px; background: #171b24; border: 1px solid #2a2d37;
    touch-action: none; user-select: none; }
  .wp-list li .wp-drag:active { background: #222838; }
  .wp-list li .wp-drag.disabled { opacity: 0.4; cursor: not-allowed; }
  .wp-list li .wp-del[disabled] { opacity: 0.35; cursor: not-allowed; }
  .wp-row-actions { display: inline-flex; align-items: center; gap: 6px; }
  .wp-more { width: 24px; height: 24px; border-radius: 6px; border: 1px solid #2a2d37;
    background: #171b24; color: #787c99; font-size: 14px; display: inline-flex;
    align-items: center; justify-content: center; cursor: pointer; }
  .wp-more:active { background: #222838; }
  .wp-more[disabled] { opacity: 0.35; cursor: not-allowed; }
  .wp-editor { margin-top: 6px; padding: 8px; border-radius: 8px; background: #131722;
    border: 1px solid #2a2d37; display: grid; grid-template-columns: 1fr 70px; gap: 6px; }
  .wp-editor input { height: 30px; border-radius: 6px; border: 1px solid #2a2d37;
    background: #0f1117; color: #c0caf5; font-size: 11px; padding: 0 8px; }
  .wp-editor label { font-size: 9px; color: #565a6e; text-transform: uppercase; letter-spacing: 0.3px; }
  .wp-editor-actions { grid-column: 1 / -1; display: flex; gap: 6px; }
  .wp-editor-actions button { height: 28px; border-radius: 6px; border: 1px solid #2a2d37;
    background: #0f1117; color: #787c99; font-size: 10px; font-weight: 700; cursor: pointer; }

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
  .btn-pause { background: #2e2a1a; color: #e0af68; display: none; border: 1px solid #5a4a2a !important; }
  .btn-resume { background: #1f3a2a; color: #9ece6a; display: none; border: 1px solid #2f5a3a !important; }
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
  .validation-panel { border: 1px solid #2a2d37; background: #0f1117; border-radius: 10px;
    padding: 8px; margin: 10px 0; }
  .validation-title { font-size: 11px; color: #787c99; font-weight: 700;
    text-transform: uppercase; letter-spacing: 0.4px; margin-bottom: 6px; }
  .validation-list { list-style: none; margin: 0; padding: 0; display: flex;
    flex-direction: column; gap: 4px; }
  .validation-list li { font-size: 11px; color: #787c99; }
  .validation-list li.ok { color: #9ece6a; }
  .validation-list li.warn { color: #e0af68; }
  .validation-list li.err { color: #f7768e; }
  .event-log { list-style: none; margin: 0; padding: 0; max-height: 120px; overflow-y: auto;
    display: flex; flex-direction: column; gap: 4px; }
  .event-log li { font-size: 11px; color: #787c99; border: 1px solid #252a36; border-radius: 6px;
    background: #111722; padding: 4px 6px; display: flex; justify-content: space-between; gap: 8px; }
  .event-log .t { color: #565a6e; min-width: 44px; text-align: right; }
  .route-summary { border: 1px solid #2a2d37; background: #0f1117; border-radius: 10px;
    padding: 8px; margin: 8px 0 10px; display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; }
  .route-metric { border: 1px solid #252a36; border-radius: 8px; padding: 6px; background: #141925; }
  .route-metric .k { font-size: 9px; color: #565a6e; text-transform: uppercase; letter-spacing: 0.3px; }
  .route-metric .v { font-size: 12px; font-weight: 700; color: #c0caf5; margin-top: 3px; }
  .dryrun-row { display: flex; gap: 6px; align-items: center; margin-top: 8px; }
  .dryrun-row input[type=range] { flex: 1; accent-color: #bb9af7; }
  .dryrun-chip { border: 1px solid #2a2d37; border-radius: 999px; padding: 3px 8px;
    font-size: 10px; color: #bb9af7; background: #111722; min-width: 78px; text-align: center; }

  @media (min-width: 1100px) {
    .bottom-sheet {
      top: calc(env(safe-area-inset-top, 0px) + 56px);
      left: auto;
      right: 0;
      bottom: 0;
      width: min(430px, 36vw);
      max-height: none;
      border-radius: 0;
      border-top: none;
      border-left: 1px solid #2a2d37;
      padding-bottom: 0;
    }
    .sheet-handle { display: none; }
    .sheet-body { display: block !important; }
    .sheet-settings { display: block; }
  }
  .wp-list li.selected { border-color: #7aa2f7; background: #121826; }

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

  .help-overlay { position: fixed; inset: 0; background: rgba(8,10,16,0.78); z-index: 1300;
    display: none; align-items: center; justify-content: center; padding: 16px; }
  .help-overlay.show { display: flex; }
  .help-card { width: min(560px, 100%); background: #111521; border: 1px solid #2a2d37;
    border-radius: 12px; box-shadow: 0 10px 30px rgba(0,0,0,0.45); padding: 12px; color: #c0caf5; }
  .help-title { font-size: 13px; font-weight: 800; margin-bottom: 8px; color: #7aa2f7; }
  .help-grid { display: grid; grid-template-columns: 130px 1fr; gap: 6px 10px; font-size: 12px; }
  .help-grid kbd { display: inline-block; border: 1px solid #3b3f54; border-bottom-width: 2px;
    border-radius: 6px; padding: 2px 6px; background: #0f1117; color: #9ece6a; font-size: 11px; }
  .help-close { margin-top: 10px; width: 100%; height: 34px; border-radius: 8px; border: 1px solid #2a2d37;
    background: #171b24; color: #c0caf5; cursor: pointer; }

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
  <button class="back-btn" title="Help / Shortcuts" onclick="toggleHelp(true)">?</button>
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
<div class="help-overlay" id="helpOverlay" onclick="if(event.target===this)toggleHelp(false)">
  <div class="help-card">
    <div class="help-title">Navigation Shortcuts</div>
    <div class="help-grid">
      <div><kbd>Ctrl/Cmd + Z</kbd></div><div>Undo route edit</div>
      <div><kbd>Ctrl/Cmd + Shift + Z</kbd></div><div>Redo route edit</div>
      <div><kbd>Ctrl/Cmd + Y</kbd></div><div>Redo route edit</div>
      <div><kbd>F</kbd></div><div>Fit map to route</div>
      <div><kbd>C</kbd></div><div>Center on robot</div>
      <div><kbd>?</kbd></div><div>Toggle this help panel</div>
      <div><kbd>Esc</kbd></div><div>Close help panel</div>
    </div>
    <button class="help-close" onclick="toggleHelp(false)">Close</button>
  </div>
</div>

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
  <div class="mission-strip">
    <span class="mission-chip" id="missionDirty">Saved</span>
    <span class="mission-chip" id="missionGate">No Waypoints</span>
    <span class="mission-chip" id="missionRun">Idle</span>
  </div>
  <div class="mission-timeline" id="missionTimeline"></div>
  <div class="sheet-body">
    <!-- Route save/load -->
    <div class="route-row" id="routeRow">
      <select id="routeSelect"><option value="">— Load Route —</option></select>
      <button onclick="loadRoute()">Load</button>
      <input type="text" id="routeName" placeholder="Route name…" />
      <button onclick="saveRoute()">Save</button>
    </div>
    <div class="route-row">
      <button onclick="exportRoute()">Export JSON</button>
      <button onclick="triggerImportRoute()">Import JSON</button>
      <input id="routeImportInput" type="file" accept="application/json,.json" style="display:none" onchange="importRouteFile(event)" />
    </div>

    <!-- Waypoint list -->
    <ul class="wp-list" id="wpList"></ul>

    <!-- Path distance -->
    <div style="font-size:12px;color:#787c99;margin-bottom:8px;text-align:right" id="totalDist"></div>
    <div class="route-summary" id="routeSummary">
      <div class="route-metric"><div class="k">Waypoints</div><div class="v" id="metricWp">0</div></div>
      <div class="route-metric"><div class="k">Distance</div><div class="v" id="metricDist">0 m</div></div>
      <div class="route-metric"><div class="k">ETA</div><div class="v" id="metricEta">--</div></div>
    </div>

    <!-- Actions -->
    <div class="action-row">
      <button class="btn-go" id="btnGo" onpointerdown="goDown(event)" onpointerup="goUp()" onpointerleave="goUp()">
        <span class="go-fill" id="goFill"></span>
        HOLD TO GO
      </button>
      <button class="btn-stop" id="btnStop" onclick="stopNav()">STOP</button>
      <button class="btn-pause" id="btnPause" onclick="pauseNav()">PAUSE</button>
      <button class="btn-resume" id="btnResume" onclick="resumeNav()">RESUME</button>
      <button class="btn-skip" id="btnSkip" onclick="skipWp()">SKIP &raquo;</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" onclick="centerRobot()">Center Robot</button>
      <button class="btn-clear" onclick="fitRoute()">Fit Route</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" onclick="focusActiveWaypoint()">Focus Active WP</button>
      <button class="btn-clear" onclick="addRobotWaypoint()">Add Robot WP</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" id="btnDryRun" onclick="toggleDryRun()">Play Dry Run</button>
    </div>
    <div class="dryrun-row">
      <span class="dryrun-chip" id="dryRunState">Dry Run: Off</span>
      <input id="dryRunSlider" type="range" min="0" max="0" step="1" value="0" oninput="setDryRunIndex(parseInt(this.value,10), true)">
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" id="btnUndo" onclick="undoRoute()">Undo</button>
      <button class="btn-clear" id="btnRedo" onclick="redoRoute()">Redo</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" onclick="reverseRoute()">Reverse Route</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" onclick="cleanCloseWaypoints()">Clean Close</button>
      <button class="btn-clear" onclick="simplifyRouteSpacing()">Simplify 1.5m</button>
    </div>
    <div class="action-row" style="margin-top:6px">
      <button class="btn-clear" onclick="clearAll()">Clear All</button>
    </div>

    <!-- Settings (visible in full mode) -->
    <div class="sheet-settings">
      <div class="validation-panel">
        <div class="validation-title">Pre-run Validation</div>
        <ul class="validation-list" id="validationList"></ul>
      </div>
      <div class="validation-panel">
        <div class="validation-title">Mission Log</div>
        <ul class="event-log" id="eventLog"></ul>
      </div>
      <div class="setting-group">
        <label>Cruise Speed <span class="setting-val" id="speedVal">40</span></label>
        <input type="range" id="speedSlider" min="10" max="80" value="40" oninput="document.getElementById('speedVal').textContent=this.value; refreshRouteSummary()">
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
let navState = 'IDLE';
let navHeadingErrorDeg = 0;
let goTimer = null;
let goStart = 0;
let missionDirty = false;
let insertGhosts = [];
let lastListRenderMs = 0;
let pausedMission = null;
let expandedWpIdx = -1;
let routeHistory = [];
let routeFuture = [];
let suspendRouteTracking = false;
let selectedWpIdx = -1;
let lastValidationWarnKey = '';
let eventLog = [];
let dryRunActiveIdx = -1;
let dryRunTimer = null;
let dryRunPlaying = false;

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

// Convert image-file pixel (px, py) — py=0 at top — to Leaflet CRS.Simple
// latlng [lat, lng]. With bounds [[0,0],[imgH,imgW]], the image is drawn
// with its top-left pixel at NW = (lat=imgH, lng=0) and bottom-right at
// SE = (lat=0, lng=imgW), so lat = imgH - py, lng = px.
function pxToLatLng(px, py) { return [imgH - py, px]; }
// Inverse: Leaflet latlng → image-file pixel (px, py) with py=0 at top.
function latLngToPx(lat, lng) { return {px: lng, py: imgH - lat}; }

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

function setMissionDirty(dirty) {
  missionDirty = !!dirty;
  if (missionDirty) {
    pausedMission = null;
    stopDryRun(false);
  }
  updateMissionStrip();
}

function toggleHelp(show) {
  const ov = document.getElementById('helpOverlay');
  if (!ov) return;
  const next = (show === undefined) ? !ov.classList.contains('show') : !!show;
  ov.classList.toggle('show', next);
}

function getRouteSnapshot() {
  return waypoints.map(wp => ({
    px: wp.px,
    py: wp.py,
    lat: wp.lat,
    lon: wp.lon,
    name: wp.name || '',
    holdS: Number.isFinite(wp.holdS) ? wp.holdS : 0,
  }));
}

function updateUndoRedoButtons() {
  const u = document.getElementById('btnUndo');
  const r = document.getElementById('btnRedo');
  if (!u || !r) return;
  u.disabled = routeHistory.length === 0 || isNavigating;
  r.disabled = routeFuture.length === 0 || isNavigating;
}

function pushUndoState() {
  if (suspendRouteTracking) return;
  routeHistory.push(getRouteSnapshot());
  if (routeHistory.length > 40) routeHistory.shift();
  routeFuture = [];
  updateUndoRedoButtons();
}

function restoreRouteSnapshot(snapshot, markDirty=true) {
  suspendRouteTracking = true;
  waypoints.forEach(wp => map.removeLayer(wp.marker));
  waypoints = [];
  distLabels.forEach(l => map.removeLayer(l));
  distLabels = [];
  (snapshot || []).forEach(wp => addWaypoint(wp.px, wp.py, {name: wp.name, holdS: wp.holdS}));
  suspendRouteTracking = false;
  expandedWpIdx = -1;
  selectedWpIdx = -1;
  setMissionDirty(!!markDirty);
  updatePolyline();
  updateWpList();
  updateUndoRedoButtons();
}

function undoRoute() {
  if (isNavigating) return;
  if (routeHistory.length === 0) return;
  routeFuture.push(getRouteSnapshot());
  const prev = routeHistory.pop();
  restoreRouteSnapshot(prev, true);
}

function redoRoute() {
  if (isNavigating) return;
  if (routeFuture.length === 0) return;
  routeHistory.push(getRouteSnapshot());
  const next = routeFuture.pop();
  restoreRouteSnapshot(next, true);
}

function updateMissionStrip() {
  const dirty = document.getElementById('missionDirty');
  const gate = document.getElementById('missionGate');
  const run = document.getElementById('missionRun');
  if (!dirty || !gate || !run) return;

  dirty.className = 'mission-chip';
  dirty.textContent = missionDirty ? 'Unsaved' : 'Saved';
  dirty.classList.add(missionDirty ? 'warn' : 'ok');

  gate.className = 'mission-chip';
  if (waypoints.length < 1) {
    gate.textContent = 'No Waypoints';
    gate.classList.add('err');
  } else if (!isArmed) {
    gate.textContent = 'Arm Required';
    gate.classList.add('err');
  } else if (gpsFix < 4) {
    gate.textContent = 'Low GPS';
    gate.classList.add('warn');
  } else {
    gate.textContent = 'Ready';
    gate.classList.add('ok');
  }

  run.className = 'mission-chip';
  if (!isNavigating) {
    if (dryRunActiveIdx >= 0) {
      run.textContent = dryRunPlaying ? 'DryRun Play' : 'DryRun Pause';
      run.classList.add('warn');
    } else if (pausedMission && pausedMission.length > 0) {
      run.textContent = 'Paused';
      run.classList.add('warn');
    } else {
      run.textContent = 'Idle';
      run.classList.add('warn');
    }
  } else {
    let label = 'Running';
    if (navState === 'ALIGN') label = 'Aligning';
    if (navState === 'DRIVE') label = 'Driving';
    if (navState === 'ARRIVE') label = 'Arrived';
    run.textContent = label;
    run.classList.add('ok');
  }
  updateValidationPanel();
  updateMissionTimeline();
}

function updateMissionTimeline() {
  const el = document.getElementById('missionTimeline');
  if (!el) return;
  if (waypoints.length === 0) {
    el.innerHTML = '<span style="font-size:11px;color:#565a6e">No mission timeline yet</span>';
    return;
  }
  let html = '';
  const displayActive = isNavigating ? navWpIndex : dryRunActiveIdx;
  for (let i = 0; i < waypoints.length; i++) {
    let cls = 'pending';
    if (displayActive >= 0 && i < displayActive) cls = 'done';
    else if (displayActive >= 0 && i === displayActive) cls = 'active';
    html += `<span class="timeline-node ${cls}" title="Waypoint ${i + 1}">${i + 1}</span>`;
    if (i < waypoints.length - 1) {
      const linkCls = (displayActive >= 0 && i < displayActive) ? 'done' : '';
      html += `<span class="timeline-link ${linkCls}"></span>`;
    }
  }
  el.innerHTML = html;
}

function getRouteValidation() {
  const checks = [];
  checks.push({
    label: waypoints.length > 0 ? 'Route has waypoints' : 'Add at least one waypoint',
    status: waypoints.length > 0 ? 'ok' : 'err',
  });
  checks.push({
    label: isArmed ? 'Robot armed' : 'Robot not armed',
    status: isArmed ? 'ok' : 'err',
  });
  checks.push({
    label: gpsFix >= 4 ? 'RTK fixed GPS ready' : 'GPS fix below RTK fixed',
    status: gpsFix >= 4 ? 'ok' : 'warn',
  });
  checks.push({
    label: calibration && calibration.calibrated ? 'Map calibration loaded' : 'Map calibration unavailable',
    status: calibration && calibration.calibrated ? 'ok' : 'warn',
  });

  let shortSegments = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const d = haversineM(waypoints[i].lat, waypoints[i].lon, waypoints[i + 1].lat, waypoints[i + 1].lon);
    if (d < 0.5) shortSegments += 1;
  }
  checks.push({
    label: shortSegments > 0 ? `${shortSegments} short segment(s) < 0.5 m` : 'Waypoint spacing looks reasonable',
    status: shortSegments > 0 ? 'warn' : 'ok',
  });

  let longSegments = 0;
  let sharpTurns = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const d = haversineM(waypoints[i].lat, waypoints[i].lon, waypoints[i + 1].lat, waypoints[i + 1].lon);
    if (d > 25.0) longSegments += 1;
  }
  for (let i = 1; i < waypoints.length - 1; i++) {
    const a = waypoints[i - 1], b = waypoints[i], c = waypoints[i + 1];
    const v1x = b.lon - a.lon, v1y = b.lat - a.lat;
    const v2x = c.lon - b.lon, v2y = c.lat - b.lat;
    const n1 = Math.hypot(v1x, v1y), n2 = Math.hypot(v2x, v2y);
    if (n1 < 1e-9 || n2 < 1e-9) continue;
    const dot = (v1x * v2x + v1y * v2y) / (n1 * n2);
    const clamped = Math.max(-1, Math.min(1, dot));
    const ang = Math.acos(clamped) * 180 / Math.PI;
    if (ang > 120) sharpTurns += 1;
  }
  checks.push({
    label: longSegments > 0 ? `${longSegments} long segment(s) > 25 m` : 'Segment lengths are manageable',
    status: longSegments > 0 ? 'warn' : 'ok',
  });
  checks.push({
    label: sharpTurns > 0 ? `${sharpTurns} sharp turn(s) > 120°` : 'Turn geometry looks smooth',
    status: sharpTurns > 0 ? 'warn' : 'ok',
  });
  return checks;
}

function getValidationWarnings() {
  const checks = getRouteValidation();
  return checks.filter(c => c.status === 'warn' || c.status === 'err');
}

function updateValidationPanel() {
  const el = document.getElementById('validationList');
  if (!el) return;
  const checks = getRouteValidation();
  el.innerHTML = checks.map(c => `<li class="${c.status}">${c.label}</li>`).join('');
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
  updateMissionStrip();
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
  robotMarker = L.marker(pxToLatLng(imgW/2, imgH/2), {icon, interactive: false, zIndexOffset: 5000}).addTo(map);
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
    robotMarker.setLatLng(pxToLatLng(robotPxX, robotPxY));
    const arrow = document.getElementById('robotArrow');
    if (arrow) arrow.style.transform = `rotate(${robotHeading}deg)`;
  }

  // Update progress line
  if (isNavigating && navWpIndex < waypoints.length) {
    const wp = waypoints[navWpIndex];
    progressLine.setLatLngs([pxToLatLng(robotPxX, robotPxY), pxToLatLng(wp.px, wp.py)]);
  } else {
    progressLine.setLatLngs([]);
  }

  if (Math.abs(targetPxX - robotPxX) > 0.1 || Math.abs(targetPxY - robotPxY) > 0.1) {
    requestAnimationFrame(animateRobot);
  } else {
    isAnimating = false;
  }
}

function centerRobot() {
  if (!map) return;
  map.panTo(pxToLatLng(robotPxX, robotPxY), {animate: true, duration: 0.4});
}

function fitRoute() {
  if (!map || waypoints.length === 0) return;
  const pts = waypoints.map(wp => pxToLatLng(wp.px, wp.py));
  if (robotPxX !== 0 || robotPxY !== 0) pts.push(pxToLatLng(robotPxX, robotPxY));
  map.fitBounds(pts, {padding: [50, 50]});
}

function focusWaypointByIndex(idx) {
  if (!map) return;
  if (idx < 0 || idx >= waypoints.length) return;
  selectedWpIdx = idx;
  const wp = waypoints[idx];
  map.panTo(pxToLatLng(wp.px, wp.py), {animate: true, duration: 0.35});
  updateWpList();
}

function focusActiveWaypoint() {
  if (!map || waypoints.length === 0) return;
  const idx = isNavigating ? Math.max(0, Math.min(navWpIndex, waypoints.length - 1)) : 0;
  focusWaypointByIndex(idx);
}

function addRobotWaypoint() {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (robotLat === 0 && robotLon === 0) {
    showToast('No robot GPS position yet');
    return;
  }
  addWaypoint(robotPxX, robotPxY, {name: 'Robot', holdS: 0});
  focusWaypointByIndex(waypoints.length - 1);
}

// ════════════════════════════════════════════════════════════════════════════
// SSE Telemetry
// ════════════════════════════════════════════════════════════════════════════
let sseMsgCount = 0;
function connectSSE() {
  if (window.__sse) { try { window.__sse.close(); } catch(_) {} }
  const evtSrc = new EventSource('/api/telemetry');
  window.__sse = evtSrc;
  evtSrc.onopen = function() { console.log('[nav] SSE open'); };
  evtSrc.onmessage = function(e) {
    try {
      const d = JSON.parse(e.data);
      if (sseMsgCount++ === 0) console.log('[nav] first SSE msg', d);
      const lat = d.gps_lat || 0;
      const lon = d.gps_lon || 0;
      const heading = d.imu_heading_deg || 0;
      gpsFix = d.gps_fix || 0;
      gpsSats = d.gps_sats || 0;
      isArmed = !!d.is_armed;
      const navMode = d.mode || 'MANUAL';
      isNavigating = navMode === 'WAYPOINT_NAV';
      if (isNavigating) {
        pausedMission = null;
        stopDryRun(false);
      }
      navWpIndex = d.wp_index || 0;
      navWpTotal = d.wp_total || 0;
      navState = d.nav_state || 'IDLE';
      navHeadingErrorDeg = (typeof d.wp_heading_error_deg === 'number') ? d.wp_heading_error_deg : 0;

      if (lat !== 0 && lon !== 0) {
        updateRobotPosition(lat, lon, heading);
      }

      updateGpsBadge();
      updateNavUI();
    } catch(err) { console.warn('[nav] SSE parse err', err); }
  };
  evtSrc.onerror = function() {
    console.warn('[nav] SSE error, reconnecting in 3s');
    try { evtSrc.close(); } catch(_) {}
    setTimeout(connectSSE, 3000);
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
  if (isNavigating && (m === 'draw' || m === 'waypoint' || m === 'edit')) {
    showToast('Editing modes are disabled while navigating');
    m = 'navigate';
  }
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
  updateInsertHandles();
}

// ════════════════════════════════════════════════════════════════════════════
// Map Click → Place Waypoint / Draw Vertex
// ════════════════════════════════════════════════════════════════════════════
function onMapClick(e) {
  if (isNavigating) return;
  // Leaflet latlng -> image-file pixel (px, py) with py=0 at top
  const p = latLngToPx(e.latlng.lat, e.latlng.lng);
  if (mode === 'waypoint') {
    addWaypoint(p.px, p.py);
  } else if (mode === 'draw') {
    addDrawVertex(p.px, p.py);
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Waypoint Management
// ════════════════════════════════════════════════════════════════════════════
function addWaypoint(px, py, meta=null) {
  if (!suspendRouteTracking) pushUndoState();
  const gps = pixelToGps(px, py);
  const wp = {
    px, py, lat: gps.lat, lon: gps.lon, marker: null, num: waypoints.length + 1,
    name: (meta && meta.name) ? String(meta.name) : '',
    holdS: (meta && Number.isFinite(Number(meta.holdS))) ? Number(meta.holdS) : 0,
  };

  const icon = L.divIcon({
    className: '',
    html: `<div class="wp-marker">${wp.num}</div>`,
    iconSize: [48, 48],
    iconAnchor: [24, 24],
  });

  const marker = L.marker(pxToLatLng(px, py), {icon, draggable: mode === 'edit', zIndexOffset: 2000}).addTo(map);
  wp.marker = marker;

  marker.on('dragstart', function() {
    const el = marker.getElement();
    if (el) el.querySelector('.wp-marker').classList.add('dragging');
  });
  marker.on('dragend', function(ev) {
    if (!suspendRouteTracking) pushUndoState();
    const el = marker.getElement();
    if (el) el.querySelector('.wp-marker').classList.remove('dragging');
    const pos = ev.target.getLatLng();
    const p = latLngToPx(pos.lat, pos.lng);
    wp.px = p.px;
    wp.py = p.py;
    const g = pixelToGps(wp.px, wp.py);
    wp.lat = g.lat;
    wp.lon = g.lon;
    setMissionDirty(true);
    updatePolyline();
    updateWpList();
  });

  waypoints.push(wp);
  renumberWaypoints();
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

function insertWaypointAt(insertIdx, px, py, meta=null) {
  if (!suspendRouteTracking) pushUndoState();
  const gps = pixelToGps(px, py);
  const wp = {
    px, py, lat: gps.lat, lon: gps.lon, marker: null, num: 0,
    name: (meta && meta.name) ? String(meta.name) : '',
    holdS: (meta && Number.isFinite(Number(meta.holdS))) ? Number(meta.holdS) : 0,
  };
  const icon = L.divIcon({
    className: '',
    html: `<div class="wp-marker"></div>`,
    iconSize: [48, 48],
    iconAnchor: [24, 24],
  });
  const marker = L.marker(pxToLatLng(px, py), {icon, draggable: mode === 'edit', zIndexOffset: 2000}).addTo(map);
  wp.marker = marker;
  marker.on('dragstart', function() {
    const el = marker.getElement();
    if (el) el.querySelector('.wp-marker').classList.add('dragging');
  });
  marker.on('dragend', function(ev) {
    if (!suspendRouteTracking) pushUndoState();
    const el = marker.getElement();
    if (el) el.querySelector('.wp-marker').classList.remove('dragging');
    const pos = ev.target.getLatLng();
    const p = latLngToPx(pos.lat, pos.lng);
    wp.px = p.px;
    wp.py = p.py;
    const g = pixelToGps(wp.px, wp.py);
    wp.lat = g.lat;
    wp.lon = g.lon;
    setMissionDirty(true);
    updatePolyline();
    updateWpList();
  });
  waypoints.splice(insertIdx, 0, wp);
  renumberWaypoints();
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

function renumberWaypoints() {
  if (selectedWpIdx >= waypoints.length) selectedWpIdx = waypoints.length - 1;
  waypoints.forEach((w, i) => {
    w.num = i + 1;
    const el = w.marker && w.marker.getElement ? w.marker.getElement() : null;
    if (el) {
      const mkr = el.querySelector('.wp-marker');
      if (mkr) mkr.textContent = w.num;
    }
  });
  rebindDragEvents();
}

function removeWaypoint(idx) {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (idx < 0 || idx >= waypoints.length) return;
  pushUndoState();
  const wp = waypoints[idx];
  map.removeLayer(wp.marker);
  waypoints.splice(idx, 1);
  renumberWaypoints();
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

function rebindDragEvents() {
  waypoints.forEach((wp, idx) => {
    wp.marker.off('dragend');
    wp.marker.on('dragend', function(ev) {
      if (!suspendRouteTracking) pushUndoState();
      const el = wp.marker.getElement();
      if (el) el.querySelector('.wp-marker').classList.remove('dragging');
      const pos = ev.target.getLatLng();
      const p = latLngToPx(pos.lat, pos.lng);
      wp.px = p.px;
      wp.py = p.py;
      const g = pixelToGps(wp.px, wp.py);
      wp.lat = g.lat;
      wp.lon = g.lon;
      setMissionDirty(true);
      updatePolyline();
      updateWpList();
    });
  });
}

function clearAll() {
  if (waypoints.length === 0) return;
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (!confirm('Clear all waypoints?')) return;
  pushUndoState();
  waypoints.forEach(wp => map.removeLayer(wp.marker));
  waypoints = [];
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

function undoLastWaypoint() {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (waypoints.length === 0) return;
  removeWaypoint(waypoints.length - 1);
}

function reverseRoute() {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (waypoints.length < 2) return;
  pushUndoState();
  waypoints.reverse();
  renumberWaypoints();
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

function duplicateWaypoint(idx) {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (idx < 0 || idx >= waypoints.length) return;
  const wp = waypoints[idx];
  insertWaypointAt(idx + 1, wp.px, wp.py, {
    name: (wp.name && wp.name.trim()) ? `${wp.name} copy` : '',
    holdS: wp.holdS || 0,
  });
}

function moveWaypointTo(idx, target) {
  if (isNavigating) return;
  if (idx < 0 || idx >= waypoints.length) return;
  const to = target === 'top' ? 0 : (waypoints.length - 1);
  if (to === idx) return;
  pushUndoState();
  const wp = waypoints.splice(idx, 1)[0];
  waypoints.splice(to, 0, wp);
  renumberWaypoints();
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

function cleanCloseWaypoints(minSpacingM=0.4) {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (waypoints.length < 3) return;
  if (!confirm(`Remove waypoints closer than ${minSpacingM.toFixed(1)} m?`)) return;
  pushUndoState();
  const snap = getRouteSnapshot();
  const kept = [snap[0]];
  for (let i = 1; i < snap.length - 1; i++) {
    const prev = kept[kept.length - 1];
    const cur = snap[i];
    const d = haversineM(prev.lat, prev.lon, cur.lat, cur.lon);
    if (d >= minSpacingM) kept.push(cur);
  }
  kept.push(snap[snap.length - 1]);
  const removed = snap.length - kept.length;
  if (removed <= 0) {
    routeHistory.pop();
    updateUndoRedoButtons();
    showToast('No close waypoints to clean');
    return;
  }
  restoreRouteSnapshot(kept, true);
  showToast(`Removed ${removed} close waypoint${removed === 1 ? '' : 's'}`);
}

function simplifyRouteSpacing(defaultSpacingM=1.5) {
  if (isNavigating) { showToast('Stop navigation before editing route'); return; }
  if (waypoints.length < 2) return;
  const input = prompt('Target spacing (meters)', String(defaultSpacingM));
  if (input === null) return;
  const spacing = Math.max(0.4, Math.min(15.0, Number(input) || defaultSpacingM));
  if (!Number.isFinite(spacing)) return;
  if (!confirm(`Resample route to approximately ${spacing.toFixed(1)} m waypoint spacing?`)) return;

  const snap = getRouteSnapshot();
  let total = 0;
  const cumulative = [0];
  for (let i = 1; i < snap.length; i++) {
    total += haversineM(snap[i - 1].lat, snap[i - 1].lon, snap[i].lat, snap[i].lon);
    cumulative.push(total);
  }
  if (total < spacing * 1.2) {
    showToast('Route too short to simplify spacing');
    return;
  }

  pushUndoState();
  const out = [snap[0]];
  for (let s = spacing; s < total; s += spacing) {
    let seg = 1;
    while (seg < cumulative.length && cumulative[seg] < s) seg += 1;
    if (seg >= snap.length) break;
    const a = snap[seg - 1];
    const b = snap[seg];
    const span = Math.max(1e-6, cumulative[seg] - cumulative[seg - 1]);
    const t = (s - cumulative[seg - 1]) / span;
    out.push({
      px: a.px + (b.px - a.px) * t,
      py: a.py + (b.py - a.py) * t,
      lat: a.lat + (b.lat - a.lat) * t,
      lon: a.lon + (b.lon - a.lon) * t,
      name: '',
      holdS: 0,
    });
  }
  out.push(snap[snap.length - 1]);
  restoreRouteSnapshot(out, true);
  showToast(`Simplified route to ${out.length} waypoints`);
}

function updatePolyline() {
  const pts = waypoints.map(wp => pxToLatLng(wp.px, wp.py));
  polyline.setLatLngs(pts);
  // Distance labels
  distLabels.forEach(l => map.removeLayer(l));
  distLabels = [];
  for (let i = 0; i < waypoints.length - 1; i++) {
    const a = waypoints[i], b = waypoints[i+1];
    const d = haversineM(a.lat, a.lon, b.lat, b.lon);
    const midPy = (a.py + b.py) / 2;
    const midPx = (a.px + b.px) / 2;
    const label = L.marker(pxToLatLng(midPx, midPy), {
      icon: L.divIcon({className: 'dist-label', html: fmtDist(d), iconSize: [60, 16], iconAnchor: [30, 8]}),
      interactive: false,
    }).addTo(map);
    distLabels.push(label);
  }
  // Total distance
  const total = getTotalRouteDistanceM();
  document.getElementById('totalDist').textContent = waypoints.length > 1 ? 'Total: ' + fmtDist(total) : '';
  updateRouteSummary(total);
  updateInsertHandles();
}

function getTotalRouteDistanceM() {
  let total = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    total += haversineM(waypoints[i].lat, waypoints[i].lon, waypoints[i + 1].lat, waypoints[i + 1].lon);
  }
  return total;
}

function updateRouteSummary(totalDistanceM) {
  const wpEl = document.getElementById('metricWp');
  const dEl = document.getElementById('metricDist');
  const eEl = document.getElementById('metricEta');
  if (!wpEl || !dEl || !eEl) return;
  wpEl.textContent = String(waypoints.length);
  dEl.textContent = waypoints.length > 1 ? fmtDist(totalDistanceM) : '0 m';

  // Heuristic conversion for operator planning only.
  const speedByte = parseInt(document.getElementById('speedSlider').value || '40', 10);
  const estMps = Math.max(0.1, speedByte * 0.02);
  if (totalDistanceM <= 0.0) {
    eEl.textContent = '--';
    return;
  }
  const seconds = Math.round(totalDistanceM / estMps);
  const mins = Math.floor(seconds / 60);
  const secs = seconds % 60;
  eEl.textContent = mins > 0 ? `${mins}m ${secs}s` : `${secs}s`;
}

function refreshRouteSummary() {
  updateRouteSummary(getTotalRouteDistanceM());
}

function updateDryRunControls() {
  const slider = document.getElementById('dryRunSlider');
  const chip = document.getElementById('dryRunState');
  const btn = document.getElementById('btnDryRun');
  if (!slider || !chip || !btn) return;
  const maxIdx = Math.max(0, waypoints.length - 1);
  slider.max = String(maxIdx);
  if (dryRunActiveIdx < 0) slider.value = '0';
  else slider.value = String(Math.max(0, Math.min(maxIdx, dryRunActiveIdx)));
  chip.textContent = dryRunActiveIdx < 0
    ? 'Dry Run: Off'
    : `Dry Run: WP ${Math.min(maxIdx, dryRunActiveIdx) + 1}`;
  btn.textContent = dryRunPlaying ? 'Stop Dry Run' : 'Play Dry Run';
  btn.disabled = waypoints.length < 2 || isNavigating;
}

function setDryRunIndex(idx, fromSlider=false) {
  if (isNavigating || waypoints.length === 0) return;
  const clamped = Math.max(0, Math.min(waypoints.length - 1, Number(idx) || 0));
  dryRunActiveIdx = clamped;
  if (!fromSlider) {
    const slider = document.getElementById('dryRunSlider');
    if (slider) slider.value = String(clamped);
  }
  selectedWpIdx = clamped;
  updateMissionStrip();
  updateWpList();
}

function stopDryRun(withToast=true) {
  if (dryRunTimer) {
    clearInterval(dryRunTimer);
    dryRunTimer = null;
  }
  const wasOn = dryRunActiveIdx >= 0 || dryRunPlaying;
  dryRunPlaying = false;
  dryRunActiveIdx = -1;
  updateMissionStrip();
  updateWpList();
  if (withToast && wasOn) showToast('Dry run stopped');
}

function toggleDryRun() {
  if (isNavigating) return;
  if (waypoints.length < 2) {
    showToast('Need at least two waypoints for dry run');
    return;
  }
  if (dryRunPlaying) {
    stopDryRun(true);
    return;
  }
  if (dryRunActiveIdx < 0) setDryRunIndex(0);
  dryRunPlaying = true;
  showToast('Dry run started');
  dryRunTimer = setInterval(() => {
    if (isNavigating || waypoints.length < 2) {
      stopDryRun(false);
      return;
    }
    if (dryRunActiveIdx >= waypoints.length - 1) {
      stopDryRun(false);
      showToast('Dry run complete');
      return;
    }
    setDryRunIndex(dryRunActiveIdx + 1);
  }, 800);
  updateDryRunControls();
}

function clearInsertHandles() {
  insertGhosts.forEach(h => map.removeLayer(h));
  insertGhosts = [];
}

function updateInsertHandles() {
  clearInsertHandles();
  if (!map) return;
  if (isNavigating) return;
  if (!(mode === 'waypoint' || mode === 'edit')) return;
  if (waypoints.length < 2) return;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const a = waypoints[i];
    const b = waypoints[i + 1];
    const midPx = (a.px + b.px) / 2;
    const midPy = (a.py + b.py) / 2;
    const ghost = L.circleMarker(pxToLatLng(midPx, midPy), {
      radius: 7,
      color: '#7aa2f7',
      weight: 2,
      fillColor: '#0f1117',
      fillOpacity: 0.95,
      interactive: true,
      pane: 'markerPane',
    }).addTo(map);
    ghost.bindTooltip('+', {permanent: true, direction: 'center', className: 'dist-label'});
    ghost.on('click', function() {
      insertWaypointAt(i + 1, midPx, midPy);
    });
    insertGhosts.push(ghost);
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Draw Path Mode
// ════════════════════════════════════════════════════════════════════════════
function addDrawVertex(px, py) {
  drawVertices.push({px, py});
  if (!drawPolyline) {
    drawPolyline = L.polyline([], {color: '#bb9af7', weight: 3, dashArray: '6 4'}).addTo(map);
  }
  drawPolyline.addLatLng(pxToLatLng(px, py));

  // Place temporary circle marker at vertex
  const circ = L.circleMarker(pxToLatLng(px, py), {radius: 5, color: '#bb9af7', fillColor: '#bb9af7',
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
  pushUndoState();
  suspendRouteTracking = true;

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
  suspendRouteTracking = false;
  setMissionDirty(true);

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
    const displayActive = isNavigating ? navWpIndex : dryRunActiveIdx;
    if (displayActive >= 0 && i < displayActive) li.classList.add('completed');
    if (displayActive >= 0 && i === displayActive) li.classList.add('active');
    if (i === selectedWpIdx) li.classList.add('selected');
    li.dataset.idx = String(i);

    const dist = i < waypoints.length - 1
      ? haversineM(wp.lat, wp.lon, waypoints[i+1].lat, waypoints[i+1].lon) : 0;
    const fromRobot = (robotLat !== 0 || robotLon !== 0)
      ? haversineM(robotLat, robotLon, wp.lat, wp.lon)
      : null;
    let state = 'Pending';
    let stateCls = '';
    if (displayActive >= 0 && i < displayActive) {
      state = 'Done';
      stateCls = 'done';
    } else if (displayActive >= 0 && i === displayActive) {
      state = isNavigating ? (navState === 'ALIGN' ? 'Align' : 'Active') : 'Preview';
      stateCls = 'active';
    }

    const dragCls = isNavigating ? 'disabled' : '';
    const dragSym = isNavigating ? '&#x1f512;' : '&#x2630;';

    const name = wp.name && wp.name.trim().length ? wp.name.trim() : `WP ${wp.num}`;
    const editor = expandedWpIdx === i ? `
      <div class="wp-editor">
        <div>
          <label>Name</label>
          <input type="text" value="${name.replace(/"/g, '&quot;')}" onclick="event.stopPropagation()" oninput="updateWaypointMeta(${i}, 'name', this.value)">
        </div>
        <div>
          <label>Hold (s)</label>
          <input type="number" min="0" max="120" step="1" value="${Number.isFinite(wp.holdS) ? wp.holdS : 0}" onclick="event.stopPropagation()" oninput="updateWaypointMeta(${i}, 'holdS', this.value)">
        </div>
        <div class="wp-editor-actions">
          <button onclick="event.stopPropagation(); duplicateWaypoint(${i})">Duplicate</button>
          <button onclick="event.stopPropagation(); moveWaypointTo(${i}, 'top')">Move Top</button>
          <button onclick="event.stopPropagation(); moveWaypointTo(${i}, 'bottom')">Move Bottom</button>
        </div>
      </div>` : '';

    li.innerHTML = `
      <span class="wp-drag ${dragCls}">${dragSym}</span>
      <span class="wp-num">${wp.num}</span>
      <div class="wp-main">
        <div class="wp-main-top">
          <span class="wp-coords">${name}</span>
          <span class="wp-state ${stateCls}">${state}</span>
          ${dist > 0 ? '<span class="wp-dist">' + fmtDist(dist) + '</span>' : '<span class="wp-dist">--</span>'}
        </div>
        <div class="wp-main-bottom">
          <span>${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}</span>
          <span>|</span>
          <span>R:${fromRobot !== null ? fmtDist(fromRobot) : '--'}</span>
          <span>|</span>
          <span>${i < waypoints.length - 1 ? 'seg '+fmtDist(dist) : 'final waypoint'}${(wp.holdS||0) > 0 ? ' | hold '+wp.holdS+'s' : ''}</span>
        </div>
        ${editor}
      </div>
      <div class="wp-row-actions">
        <button class="wp-more" ${isNavigating ? 'disabled' : ''} onclick="event.stopPropagation(); toggleWpDetails(${i})">&#8942;</button>
        <button class="wp-del" ${isNavigating ? 'disabled' : ''} onclick="event.stopPropagation(); removeWaypoint(${i})">&times;</button>
      </div>
    `;
    li.onclick = () => focusWaypointByIndex(i);
    list.appendChild(li);
  });
  document.getElementById('wpCount').textContent = waypoints.length + ' waypoint' + (waypoints.length !== 1 ? 's' : '');
  updateGoButton();
  updateMissionStrip();

  // Enable list drag reorder
  enableListDragReorder();
  updateUndoRedoButtons();
  updateDryRunControls();
  if (isNavigating) {
    const active = list.querySelector(`li[data-idx="${navWpIndex}"]`);
    if (active && typeof active.scrollIntoView === 'function') {
      active.scrollIntoView({block: 'nearest', behavior: 'smooth'});
    }
  }
}

function toggleWpDetails(idx) {
  if (isNavigating) return;
  expandedWpIdx = expandedWpIdx === idx ? -1 : idx;
  updateWpList();
}

function updateWaypointMeta(idx, key, value) {
  if (idx < 0 || idx >= waypoints.length) return;
  const wp = waypoints[idx];
  if (key === 'name') {
    wp.name = String(value || '').slice(0, 32);
  } else if (key === 'holdS') {
    const v = Math.max(0, Math.min(120, Number(value) || 0));
    wp.holdS = Math.round(v);
  }
  setMissionDirty(true);
  updateWpList();
}

function enableListDragReorder() {
  if (isNavigating) return;
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
  if (isNavigating) return;
  pushUndoState();
  const wp = waypoints.splice(fromIdx, 1)[0];
  waypoints.splice(toIdx, 0, wp);
  renumberWaypoints();
  setMissionDirty(true);
  updatePolyline();
  updateWpList();
}

// ════════════════════════════════════════════════════════════════════════════
// Navigation Controls
// ════════════════════════════════════════════════════════════════════════════
function updateGoButton() {
  const btn = document.getElementById('btnGo');
  const btnPause = document.getElementById('btnPause');
  const btnResume = document.getElementById('btnResume');
  const btnStop = document.getElementById('btnStop');
  const btnSkip = document.getElementById('btnSkip');
  btn.classList.remove('armed-required');
  btnPause.style.display = 'none';
  btnResume.style.display = 'none';
  btnStop.style.display = 'none';
  btnSkip.style.display = 'none';

  if (isNavigating) {
    btn.style.display = 'none';
    btnPause.style.display = '';
    btnStop.style.display = '';
    btnSkip.style.display = '';
    return;
  }

  btn.style.display = '';
  if (pausedMission && pausedMission.length > 0) {
    btnResume.style.display = '';
  }

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
    btn.disabled = true;
    btn.innerHTML = '<span class="go-fill" id="goFill"></span>LOW GPS FIX REQUIRED';
    return;
  }
  btn.disabled = false;
  btn.innerHTML = '<span class="go-fill" id="goFill"></span>HOLD TO GO';
  updateUndoRedoButtons();
}

function updateNavUI() {
  updateGoButton();
  const status = document.getElementById('sheetNavStatus');
  if (isNavigating) {
    const herr = Number.isFinite(navHeadingErrorDeg) ? ` | err ${Math.abs(navHeadingErrorDeg).toFixed(1)}°` : '';
    status.textContent = `WP ${navWpIndex + 1}/${navWpTotal} | ${navState}${herr}`;
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
  updateMissionStrip();
  const now = Date.now();
  if (now - lastListRenderMs > 400) {
    lastListRenderMs = now;
    updateWpList();
  }
  updateInsertHandles();
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

async function startNavWithWaypoints(wpData, isResume=false) {
  if (!wpData || wpData.length < 1) {
    showToast('No waypoints to run');
    return;
  }
  const issues = getValidationWarnings();
  if (issues.length > 0) {
    const key = issues.map(i => i.label).join('|');
    if (key !== lastValidationWarnKey) {
      const msg = 'Validation warnings:\n- ' + issues.map(i => i.label).join('\n- ') + '\n\nProceed anyway?';
      if (!confirm(msg)) return;
      lastValidationWarnKey = key;
    }
  } else {
    lastValidationWarnKey = '';
  }
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
      pausedMission = null;
      showToast(isResume ? 'Navigation resumed' : 'Navigation started!');
    } else {
      showToast('Error: ' + (data.error || 'unknown'));
    }
  } catch(err) {
    showToast(isResume ? 'Failed to resume navigation' : 'Failed to start navigation');
  }
}

async function startNav() {
  const wpData = waypoints.map(wp => ({px: wp.px, py: wp.py, lat: wp.lat, lon: wp.lon, name: wp.name || '', holdS: wp.holdS || 0}));
  await startNavWithWaypoints(wpData, false);
}

async function stopNav(showMsg=true) {
  try {
    await fetch('/api/nav/stop', {method: 'POST'});
    if (showMsg) showToast('Navigation stopped');
  } catch(err) {
    if (showMsg) showToast('Failed to stop');
  }
}

async function pauseNav() {
  const idx = Math.max(0, Math.min(navWpIndex, waypoints.length - 1));
  pausedMission = waypoints.slice(idx).map(wp => ({px: wp.px, py: wp.py, lat: wp.lat, lon: wp.lon, name: wp.name || '', holdS: wp.holdS || 0}));
  await stopNav(false);
  showToast(pausedMission.length > 0 ? 'Navigation paused' : 'Paused');
  updateGoButton();
}

async function resumeNav() {
  if (!pausedMission || pausedMission.length < 1) {
    showToast('No paused mission');
    return;
  }
  await startNavWithWaypoints(pausedMission, true);
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
function exportRoute() {
  if (waypoints.length === 0) { showToast('No route to export'); return; }
  const payload = {
    exported_at: new Date().toISOString(),
    waypoints: waypoints.map(wp => ({
      px: wp.px, py: wp.py, lat: wp.lat, lon: wp.lon, name: wp.name || '', holdS: wp.holdS || 0,
    })),
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], {type: 'application/json'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `route-${Date.now()}.json`;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
  showToast('Route exported');
}

function triggerImportRoute() {
  if (isNavigating) { showToast('Stop navigation before import'); return; }
  if (waypoints.length > 0 && !confirm('Importing will replace current unsaved route. Continue?')) return;
  const input = document.getElementById('routeImportInput');
  if (!input) return;
  input.value = '';
  input.click();
}

async function importRouteFile(e) {
  if (isNavigating) { showToast('Stop navigation before import'); return; }
  const file = e.target && e.target.files ? e.target.files[0] : null;
  if (!file) return;
  try {
    const text = await file.text();
    const parsed = JSON.parse(text);
    const raw = Array.isArray(parsed) ? parsed : parsed.waypoints;
    if (!Array.isArray(raw) || raw.length < 1) {
      showToast('Invalid route file');
      return;
    }
    routeHistory = [];
    routeFuture = [];
    suspendRouteTracking = true;
    waypoints.forEach(wp => map.removeLayer(wp.marker));
    waypoints = [];
    distLabels.forEach(l => map.removeLayer(l));
    distLabels = [];
    raw.forEach(wp => addWaypoint(wp.px, wp.py, {name: wp.name || '', holdS: wp.holdS || 0}));
    suspendRouteTracking = false;
    setMissionDirty(true);
    expandedWpIdx = -1;
    selectedWpIdx = -1;
    updatePolyline();
    updateWpList();
    showToast(`Imported ${waypoints.length} waypoint${waypoints.length === 1 ? '' : 's'}`);
  } catch(err) {
    showToast('Failed to import route JSON');
  }
}

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
  const sel = document.getElementById('routeSelect');
  const existing = sel ? Array.from(sel.options).some(o => o.value === name) : false;
  if (existing && !confirm(`Overwrite existing route "${name}"?`)) return;

  const wpData = waypoints.map(wp => ({px: wp.px, py: wp.py, lat: wp.lat, lon: wp.lon, name: wp.name || '', holdS: wp.holdS || 0}));
  try {
    const resp = await fetch('/api/nav/waypoints', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name, waypoints: wpData}),
    });
    const data = await resp.json();
    if (data.ok) {
      showToast('Route "' + name + '" saved');
      setMissionDirty(false);
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
  if (missionDirty && waypoints.length > 0) {
    if (!confirm('Discard unsaved route changes and load selected route?')) return;
  }
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
      routeHistory = [];
      routeFuture = [];
      suspendRouteTracking = true;
      data.waypoints.forEach(wp => addWaypoint(wp.px, wp.py, {name: wp.name || '', holdS: wp.holdS || 0}));
      suspendRouteTracking = false;
      setMissionDirty(false);
      expandedWpIdx = -1;
      showToast('Route "' + name + '" loaded (' + waypoints.length + ' waypoints)');

      // Zoom to fit
      if (waypoints.length > 0) {
        const pts = waypoints.map(wp => pxToLatLng(wp.px, wp.py));
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
function logEvent(msg) {
  const ts = new Date();
  const stamp = ts.toTimeString().slice(0, 8);
  eventLog.unshift({t: stamp, msg: String(msg)});
  if (eventLog.length > 12) eventLog = eventLog.slice(0, 12);
  const el = document.getElementById('eventLog');
  if (!el) return;
  el.innerHTML = eventLog.map(e => `<li><span>${e.msg}</span><span class="t">${e.t}</span></li>`).join('');
}

function showToast(msg) {
  logEvent(msg);
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimeout);
  toastTimeout = setTimeout(() => el.classList.remove('show'), 2500);
}

// ════════════════════════════════════════════════════════════════════════════
// Init
// ════════════════════════════════════════════════════════════════════════════
document.addEventListener('keydown', function(e) {
  const tag = (e.target && e.target.tagName) ? e.target.tagName.toLowerCase() : '';
  if (e.key === 'Escape') {
    toggleHelp(false);
    return;
  }
  if (e.key === '?' || (e.shiftKey && e.key === '/')) {
    e.preventDefault();
    toggleHelp();
    return;
  }
  if (tag === 'input' || tag === 'textarea') return;
  const helpOpen = document.getElementById('helpOverlay')?.classList.contains('show');
  if (helpOpen) return;
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
    e.preventDefault();
    if (e.shiftKey) redoRoute();
    else undoRoute();
    return;
  }
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'y') {
    e.preventDefault();
    redoRoute();
    return;
  }
  if (e.key.toLowerCase() === 'f') {
    e.preventDefault();
    fitRoute();
    return;
  }
  if (e.key.toLowerCase() === 'c') {
    e.preventDefault();
    centerRobot();
  }
});

window.addEventListener('beforeunload', function(e) {
  if (!missionDirty) return;
  e.preventDefault();
  e.returnValue = '';
});

document.addEventListener('visibilitychange', function() {
  if (document.hidden && dryRunPlaying) stopDryRun(false);
});

logEvent('Waypoint UI ready');
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

    # ── Current GPS position ──

    @bp.route("/api/gps")
    def get_gps():
        """Return the robot's latest GPS fix.

        Response fields:
          lat, lon          – WGS-84 decimal degrees (0.0 when no fix)
          fix_quality       – 0=none 1=GPS 2=DGPS 4=RTK-fix 5=RTK-float
          satellites        – satellites used
          altitude_m        – altitude in metres
          hdop              – horizontal dilution of precision
          diff_age_s        – seconds since last RTK correction (null if N/A)
          ok                – false only when controller is unavailable
        """
        if controller is None:
            return _json_resp({"ok": False, "error": "no controller"}, 503)
        r = controller._gps_reading
        if r is None:
            return _json_resp({
                "ok": True,
                "lat": 0.0,
                "lon": 0.0,
                "fix_quality": 0,
                "satellites": 0,
                "altitude_m": 0.0,
                "hdop": 99.9,
                "diff_age_s": None,
            })
        return _json_resp({
            "ok": True,
            "lat": r.latitude,
            "lon": r.longitude,
            "fix_quality": r.fix_quality,
            "satellites": r.satellites_used,
            "altitude_m": r.altitude_m,
            "hdop": r.hdop,
            "diff_age_s": r.diff_age_s if r.diff_age_s >= 0 else None,
        })

    # ── Go to GPS waypoint (absolute or relative) ──

    @bp.route("/api/nav/go", methods=["POST"])
    def nav_go():
        """Send the robot to a GPS waypoint without the map UI.

        Accepts JSON with one of two forms:

        Absolute GPS:
            {"lat": 30.123, "lon": -95.456}
            {"lat": 30.123, "lon": -95.456, "cruise_speed": 40, "arrival_radius": 0.5}

        Relative offset (uses current GPS position as origin):
            {"direction": "north", "distance_ft": 10}
            {"direction": "southwest", "distance_ft": 25}
            {"bearing_deg": 270, "distance_ft": 15}
            Any of the above may include optional cruise_speed / arrival_radius.

        Valid direction strings: north, south, east, west,
            northeast, northwest, southeast, southwest.

        NOTE: the robot must already be armed via the RC transmitter
        (ch3 / throttle stick high) before it will move. This endpoint
        queues the waypoint and activates nav mode; motion starts as
        soon as the arm condition is satisfied.

        Response:
            {"ok": true, "target_lat": x, "target_lon": y}
        """
        if controller is None:
            return _json_resp({"ok": False, "error": "no controller"}, 503)

        data = request.get_json(force=True) or {}
        cruise_speed = int(data.get("cruise_speed", 40))
        arrival_radius = float(data.get("arrival_radius", 0.5))

        _DIRECTION_BEARING = {
            "north":     0.0,
            "northeast": 45.0,
            "east":      90.0,
            "southeast": 135.0,
            "south":     180.0,
            "southwest": 225.0,
            "west":      270.0,
            "northwest": 315.0,
        }

        # ── Resolve target lat/lon ──
        if "lat" in data and "lon" in data:
            target_lat = float(data["lat"])
            target_lon = float(data["lon"])
        elif "direction" in data or "bearing_deg" in data:
            r = controller._gps_reading
            if r is None or r.fix_quality == 0:
                return _json_resp(
                    {"ok": False, "error": "no GPS fix — cannot compute relative position"},
                    400,
                )
            distance_ft = data.get("distance_ft")
            if distance_ft is None:
                return _json_resp({"ok": False, "error": "distance_ft required for relative mode"}, 400)
            distance_m = float(distance_ft) * 0.3048

            if "bearing_deg" in data:
                brng = float(data["bearing_deg"]) % 360.0
            else:
                direction = str(data["direction"]).lower().strip()
                if direction not in _DIRECTION_BEARING:
                    return _json_resp(
                        {"ok": False, "error": f"unknown direction '{direction}'; "
                         f"valid: {', '.join(_DIRECTION_BEARING)}"},
                        400,
                    )
                brng = _DIRECTION_BEARING[direction]

            target_lat, target_lon = _offset_gps(r.latitude, r.longitude, brng, distance_m)
        else:
            return _json_resp(
                {"ok": False,
                 "error": "provide either {lat, lon} or {direction/bearing_deg, distance_ft}"},
                400,
            )

        # ── Activate nav ──
        try:
            nav = controller._waypoint_nav
            if nav is None:
                return _json_resp({"ok": False, "error": "waypoint nav not initialized"}, 503)

            nav._cfg.cruise_speed_byte = cruise_speed
            nav._cfg.arrival_radius_m = arrival_radius
            nav.set_waypoints([Waypoint(lat=target_lat, lon=target_lon, name="GO")])
            controller.activate_waypoint_nav()
            _log.info("nav/go: target=(%.7f, %.7f) speed=%d radius=%.1f",
                      target_lat, target_lon, cruise_speed, arrival_radius)
            return _json_resp({"ok": True, "target_lat": target_lat, "target_lon": target_lon})
        except Exception as exc:
            _log.exception("nav/go failed")
            return _json_resp({"ok": False, "error": str(exc)}, 500)

    return bp


def _offset_gps(lat: float, lon: float, bearing_deg: float, distance_m: float):
    """Return (lat2, lon2) that is distance_m away from (lat, lon) at bearing_deg."""
    R = 6_371_000.0
    d = distance_m / R
    brng = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d) +
                     math.cos(lat1) * math.sin(d) * math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d) * math.cos(lat1),
                              math.cos(d) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)
