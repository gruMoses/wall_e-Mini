"""
Activity-triggered OAK-D Lite recorder.

Records H.265 video and MCAP data (annotated RGB snapshots, colorized depth,
telemetry) whenever the robot is actively engaged — obstacle avoidance is
slowing/stopping, or Follow Me mode is active.

A pre-buffer ring keeps the last N seconds of frames in memory so the recording
captures the moment *before* the trigger, not just after.
"""

from __future__ import annotations

import logging
import math
import shutil
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]

try:
    from mcap.writer import Writer as McapWriter
except ImportError:
    McapWriter = None  # type: ignore[assignment,misc]

import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import OakRecordingConfig, ObstacleAvoidanceConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.hardware.oak_depth import DepthStats


def mcap_detection_dict(d: PersonDetection) -> dict:
    """Serialize one PersonDetection for the MCAP telemetry `detections` array.

    Pure function extracted so tests can exercise the exact production
    serialization instead of a hand-maintained mirror.
    """
    return {
        "x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
        "conf": round(d.confidence, 2), "track_id": d.track_id,
        "bbox": [round(b, 3) for b in d.bbox],
    }


# ---------------------------------------------------------------------------
# Recording state machine
# ---------------------------------------------------------------------------

class _RecState:
    IDLE = "IDLE"
    RECORDING = "RECORDING"
    LINGERING = "LINGERING"


# ---------------------------------------------------------------------------
# Telemetry snapshot fed by main loop each tick
# ---------------------------------------------------------------------------

@dataclass
class RecordingTelemetry:
    """Lightweight struct passed from main loop to recorder each tick."""
    timestamp: float  # time.time()
    mode: str  # "MANUAL" or "FOLLOW_ME"
    throttle_scale: float  # 0.0-1.0 from obstacle avoidance
    obstacle_distance_m: float | None
    motor_left: int
    motor_right: int
    is_armed: bool
    depth_stats: DepthStats | None
    person_detections: list[PersonDetection]
    follow_tracking: bool = False
    follow_target_x_m: float | None = None
    follow_target_z_m: float | None = None
    heading_deg: float | None = None
    yaw_rate_dps: float | None = None
    pursuit_mode: str | None = None
    trail_length: int | None = None
    trail_distance_m: float | None = None
    trail_rejected_jump_count: int | None = None
    trail_rejected_speed_count: int | None = None
    follow_target_world_x: float | None = None
    follow_target_world_y: float | None = None
    odom_x: float | None = None
    odom_y: float | None = None
    odom_theta_deg: float | None = None
    speed_offset: float | None = None
    steer_offset: float | None = None
    distance_error_m: float | None = None
    gps_lat: float | None = None
    gps_lon: float | None = None
    gps_alt_m: float | None = None
    gps_fix: int | None = None
    gps_sats: int | None = None
    gps_hdop: float | None = None
    gps_diff_age_s: float | None = None
    gps_station_id: int | None = None
    bms_voltage_v: float | None = None
    bms_current_a: float | None = None
    bms_soc_pct: float | None = None
    bms_cell_delta_mv: int | None = None
    bms_temp_max_c: float | None = None
    bms_connected: bool | None = None
    bms_charging: bool | None = None
    bms_discharge_fet_on: bool | None = None
    # Controller charger_inhibit state this tick (2026-06-13 motor-cutout bug —
    # this was surfaced nowhere before, so a mid-drive stop looked like a
    # follow-me/obstacle bug instead of the real charger-inhibit cause).
    charger_inhibit: bool | None = None
    imu_heading_deg: float | None = None
    vesc_left_rpm: int | None = None
    vesc_right_rpm: int | None = None
    vesc_actual_speed_mps: float | None = None
    vesc_rx_frame_count: int = 0
    vesc_rx_parse_error_count: int = 0
    vesc_rx_recv_error_count: int = 0
    vesc_rx_reopen_count: int = 0
    vesc_rx_last_frame_age_s: float | None = None
    # VESC low-voltage watchdog latch (early-warning + motor-cutoff; recoverable).
    vesc_pack_low_latched: bool = False
    # Per-motor STATUS(9) frame ages + CAN RX-thread liveness (diagnostics for
    # the /debug board — >0.5s on a motor means its RPM is not trustworthy).
    vesc_left_status_age_s: float | None = None
    vesc_right_status_age_s: float | None = None
    vesc_rx_thread_alive: bool | None = None
    # ── Follow-Me visualization fields (Items 2 + 4) ─────────────────
    follow_mode: str | None = None
    trail_points_xy: list | None = None       # [[x,y], ...] last 20
    lookahead_point_xy: list | None = None     # [x, y] or None
    robot_pose_x: float | None = None
    robot_pose_y: float | None = None
    robot_pose_theta: float | None = None
    hysteresis_count: int = 0
    hysteresis_max: int = 3
    extrapolation_active: bool = False
    extrapolation_count: int = 0
    consume_radius_m: float = 0.4
    target_confidence: float | None = None
    target_lateral_offset: float | None = None
    # ── Waypoint-nav progress ─────────────────────────────────────────
    wp_index: int | None = None
    wp_total: int | None = None
    wp_name: str | None = None
    wp_bearing_deg: float | None = None
    wp_distance_m: float | None = None
    wp_completed: bool | None = None
    heading_offset_deg: float | None = None
    heading_offset_locked: bool | None = None
    corrected_heading_deg: float | None = None
    heading_align_last_cog_deg: float | None = None
    heading_align_enabled: bool | None = None
    # ── Safety / liveness (consumed by phone-teleop rc_state_provider) ──
    # Latched RC ch5 emergency-stop state, mirrored from the controller so the
    # teleop session sees the real RC e-stop (it has no other view of ch5).
    emergency_active: bool = False
    # Monotonic timestamp (time.monotonic()) of when this snapshot was built.
    # Lets the rc_state_provider fail-safe when telemetry stops updating: a
    # frozen object older than the staleness bound forces teleop disarm.
    ts_mono: float = 0.0


# ---------------------------------------------------------------------------
# MCAP helpers
# ---------------------------------------------------------------------------

MCAP_IMAGE_TOPIC = "/oak/rgb_annotated"
MCAP_DEPTH_TOPIC = "/oak/depth_colorized"
MCAP_TELEMETRY_TOPIC = "/oak/telemetry"

# Foxglove well-known schema for compressed images
_COMPRESSED_IMAGE_SCHEMA = "foxglove.CompressedImage"
_JSON_SCHEMA = "foxglove.RawJSON"


def _make_mcap_writer(path: Path) -> tuple | None:
    """Open an MCAP file and register channels. Returns (writer, fh, channel_ids) or None."""
    if McapWriter is None:
        return None
    try:
        fh = open(path, "wb")
        writer = McapWriter(fh)
        writer.start(profile="", library="wall-e-mini-recorder")

        img_schema_id = writer.register_schema(
            name=_COMPRESSED_IMAGE_SCHEMA,
            encoding="jsonschema",
            data=b'{"type":"object","properties":{"timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}}},"frame_id":{"type":"string"},"format":{"type":"string"},"data":{"type":"string","contentEncoding":"base64"}}}',
        )
        json_schema_id = writer.register_schema(
            name=_JSON_SCHEMA,
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )

        rgb_ch = writer.register_channel(
            topic=MCAP_IMAGE_TOPIC,
            message_encoding="json",
            schema_id=img_schema_id,
        )
        depth_ch = writer.register_channel(
            topic=MCAP_DEPTH_TOPIC,
            message_encoding="json",
            schema_id=img_schema_id,
        )
        telem_ch = writer.register_channel(
            topic=MCAP_TELEMETRY_TOPIC,
            message_encoding="json",
            schema_id=json_schema_id,
        )
        return writer, fh, {"rgb": rgb_ch, "depth": depth_ch, "telemetry": telem_ch}
    except Exception:
        logger.exception("Failed to create MCAP writer")
        return None


# ---------------------------------------------------------------------------
# OpenCV annotation helpers
# ---------------------------------------------------------------------------

_COLOR_TARGET = (0, 255, 0)     # green
_COLOR_OTHER = (160, 160, 160)  # gray
_COLOR_ROI = (255, 200, 0)      # cyan-ish
_COLOR_TEXT_BG = (0, 0, 0)


def _world_to_pixel(
    wx, wy, robot_x, robot_y, robot_theta,
    img_w, img_h,
    camera_height_m=0.497, hfov_deg=81.0,
):
    """Project a world-coordinate ground point to image pixel coords.

    Returns (px, py) or None if behind camera / off-screen.
    """
    dx = wx - robot_x
    dy = wy - robot_y
    cos_t = math.cos(robot_theta)
    sin_t = math.sin(robot_theta)
    z_cam = dx * cos_t + dy * sin_t      # forward distance
    x_cam = dx * sin_t - dy * cos_t      # rightward offset
    if z_cam < 0.15:
        return None
    hfov_rad = math.radians(hfov_deg)
    fx = (img_w / 2.0) / math.tan(hfov_rad / 2.0)
    vfov_rad = 2.0 * math.atan(img_h / img_w * math.tan(hfov_rad / 2.0))
    fy = (img_h / 2.0) / math.tan(vfov_rad / 2.0)
    px = int(img_w / 2.0 + fx * x_cam / z_cam)
    py = int(img_h / 2.0 + fy * camera_height_m / z_cam)
    if 0 <= px < img_w and 0 <= py < img_h:
        return (px, py)
    return None


def _annotate_rgb(
    frame: np.ndarray,
    detections: list[PersonDetection],
    telemetry: RecordingTelemetry,
    obs_cfg: ObstacleAvoidanceConfig | None = None,
) -> np.ndarray:
    """Draw bounding boxes, trail overlay, and enhanced status text."""
    if cv2 is None or np is None:
        return frame
    img = frame.copy()
    h, w = img.shape[:2]

    if obs_cfg is None:
        obs_cfg = ObstacleAvoidanceConfig()

    rw = obs_cfg.roi_width_pct
    rh = obs_cfg.roi_height_pct
    rv = getattr(obs_cfg, "roi_vertical_offset_pct", 0.0)
    cy_norm = max(rh / 2.0, min(1.0 - rh / 2.0, 0.5 + rv))
    y0 = int(h * (cy_norm - rh / 2))
    y1 = int(h * (cy_norm + rh / 2))

    robot_w = getattr(obs_cfg, "robot_width_m", 0.0)
    cam_h = getattr(obs_cfg, "camera_height_m", 0.0)

    if robot_w > 0 and cam_h > 0:
        cx_img = int(w / 2.0)
        cy_horizon = int(h / 2.0)
        pts = np.array([
            [0, h - 1],
            [w - 1, h - 1],
            [cx_img, cy_horizon],
        ], np.int32)
        cv2.polylines(img, [pts], isClosed=True, color=_COLOR_ROI, thickness=1)
        roi_color = (0, 200, 255)
        dash_len = 12
        for y_line in (y0, y1):
            for x_start in range(0, w, dash_len * 2):
                x_end = min(x_start + dash_len, w)
                cv2.line(img, (x_start, y_line), (x_end, y_line), roi_color, 1)
        cv2.putText(img, "ROI", (4, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, roi_color, 1, cv2.LINE_AA)
    else:
        x0r = int(w * (0.5 - rw / 2))
        x1r = int(w * (0.5 + rw / 2))
        cv2.rectangle(img, (x0r, y0), (x1r, y1), _COLOR_ROI, 1)

    # Detection bounding boxes
    best_idx = -1
    if detections:
        best_score = -1.0
        for i, d in enumerate(detections):
            cx = (d.bbox[0] + d.bbox[2]) / 2.0
            center_close = 1.0 - abs(cx - 0.5) * 2.0
            depth_close = 1.0 - min(d.z_m / 5.0, 1.0)
            score = 0.6 * center_close + 0.4 * depth_close
            if score > best_score:
                best_score = score
                best_idx = i

    for i, d in enumerate(detections):
        bx0, by0 = int(d.bbox[0] * w), int(d.bbox[1] * h)
        bx1, by1 = int(d.bbox[2] * w), int(d.bbox[3] * h)
        color = _COLOR_TARGET if i == best_idx else _COLOR_OTHER
        thickness = 2 if i == best_idx else 1
        cv2.rectangle(img, (bx0, by0), (bx1, by1), color, thickness)
        label = f"{d.confidence:.0%} {d.z_m:.1f}m"
        cv2.putText(img, label, (bx0, max(by0 - 4, 10)),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)

    # ── Item 2: Trail visualization on RGB feed ──────────────────────────
    try:
        _draw_trail_overlay(img, telemetry, cam_h)
    except Exception:
        pass  # never crash the video stream for overlay bugs

    # ── Item 4: Enhanced status text ─────────────────────────────────────
    try:
        _draw_status_overlay(img, telemetry)
    except Exception:
        _draw_basic_status(img, telemetry)

    return img


def _draw_trail_overlay(img, telemetry, camera_height_m):
    """Item 2: Draw trail points, lookahead carrot, and consume radius."""
    h, w = img.shape[:2]
    trail_pts = getattr(telemetry, "trail_points_xy", None)
    robot_x = getattr(telemetry, "robot_pose_x", None)
    robot_y = getattr(telemetry, "robot_pose_y", None)
    robot_theta = getattr(telemetry, "robot_pose_theta", None)

    if not trail_pts or robot_x is None or robot_y is None or robot_theta is None:
        return
    if camera_height_m <= 0:
        camera_height_m = 0.497

    n_pts = len(trail_pts)
    extrap_active = getattr(telemetry, "extrapolation_active", False)
    extrap_count = getattr(telemetry, "extrapolation_count", 0)
    extrap_start = max(0, n_pts - extrap_count) if extrap_active else n_pts

    # Project each trail point to image pixels
    projected = []
    for idx, pt in enumerate(trail_pts):
        pix = _world_to_pixel(
            pt[0], pt[1], robot_x, robot_y, robot_theta, w, h,
            camera_height_m=camera_height_m,
        )
        projected.append(pix)
        if pix is not None:
            if extrap_active and idx >= extrap_start:
                color = (255, 255, 0)   # cyan (BGR)
            elif idx >= n_pts - 3:
                color = (0, 255, 0)     # bright green
            else:
                color = (0, 180, 0)     # dim green
            cv2.circle(img, pix, 3, color, -1, cv2.LINE_AA)

    # Polyline connecting valid consecutive points
    current_seg = []
    for pix in projected:
        if pix is not None:
            current_seg.append(pix)
        else:
            if len(current_seg) >= 2:
                arr = np.array(current_seg, np.int32).reshape((-1, 1, 2))
                cv2.polylines(img, [arr], False, (0, 180, 0), 1, cv2.LINE_AA)
            current_seg = []
    if len(current_seg) >= 2:
        arr = np.array(current_seg, np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [arr], False, (0, 180, 0), 1, cv2.LINE_AA)

    # Lookahead carrot (orange circle with label)
    la_pt = getattr(telemetry, "lookahead_point_xy", None)
    if la_pt is not None:
        la_pix = _world_to_pixel(
            la_pt[0], la_pt[1], robot_x, robot_y, robot_theta, w, h,
            camera_height_m=camera_height_m,
        )
        if la_pix is not None:
            cv2.circle(img, la_pix, 6, (0, 140, 255), 2, cv2.LINE_AA)
            la_dist = math.hypot(la_pt[0] - robot_x, la_pt[1] - robot_y)
            cv2.putText(img, f"LA {la_dist:.1f}m",
                        (la_pix[0] + 8, la_pix[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30,
                        (0, 140, 255), 1, cv2.LINE_AA)

    # Consume radius (faint red arc at bottom of frame)
    consume_r = getattr(telemetry, "consume_radius_m", 0.4)
    if consume_r > 0:
        hfov_rad = math.radians(81.0)
        fx = (w / 2.0) / math.tan(hfov_rad / 2.0)
        radius_px = max(3, int(fx * consume_r / 1.0))
        cx = w // 2
        cy = h - 10
        cv2.ellipse(img, (cx, cy), (radius_px, max(2, radius_px // 3)),
                    0, 180, 360, (0, 0, 160), 1, cv2.LINE_AA)


def _draw_status_overlay(img, telemetry):
    """Item 4: Enhanced status text with color-coded mode and trail info."""
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    lines = []  # (text, color_bgr, scale)

    # Mode line — bigger and color-coded
    follow_mode = getattr(telemetry, "follow_mode", None) or telemetry.mode
    _mode_colors = {
        "DIRECT_PID":       (0, 220, 0),      # green
        "TRAIL_PURSUIT":    (255, 180, 0),     # blue-ish
        "LOST_BLIND_TRAIL": (0, 165, 255),     # orange
        "SEARCH_ROTATE":    (0, 255, 255),     # yellow
        "TRAIL_EXHAUSTED":  (0, 0, 255),       # red
        "FOLLOW_ME":        (0, 220, 0),
        "MANUAL":           (200, 200, 200),
        "IDLE":             (140, 140, 140),
    }
    mode_color = _mode_colors.get(follow_mode, (200, 200, 200))
    mode_display = follow_mode.replace("_", " ")
    lines.append((mode_display, mode_color, 0.40))

    # Basic info
    dist_str = (f"{telemetry.obstacle_distance_m:.2f}m"
                if telemetry.obstacle_distance_m else "?")
    arm_str = "ARM" if telemetry.is_armed else "DISARM"
    lines.append((
        f"scl={telemetry.throttle_scale:.2f} dist={dist_str}  "
        f"M L={telemetry.motor_left} R={telemetry.motor_right} {arm_str}",
        (255, 255, 255), 0.28))

    # Trail info (only when trail active)
    trail_len = getattr(telemetry, "trail_length", None)
    trail_dist = getattr(telemetry, "trail_distance_m", None)
    extrap_active = getattr(telemetry, "extrapolation_active", False)
    extrap_count = getattr(telemetry, "extrapolation_count", 0)
    if trail_len is not None and trail_len > 0:
        trail_text = f"TRAIL: {trail_len}pts"
        if trail_dist is not None:
            trail_text += f" {trail_dist:.1f}m"
        if extrap_active and extrap_count > 0:
            trail_text += f" +{extrap_count}ext"
        lines.append((trail_text, (0, 220, 0), 0.30))

    # Hysteresis (only when count > 0)
    hyst_count = getattr(telemetry, "hysteresis_count", 0)
    hyst_max = getattr(telemetry, "hysteresis_max", 3)
    if hyst_count > 0:
        hyst_text = f"HYST: {hyst_count}/{hyst_max}"
        if hyst_count >= hyst_max:
            hyst_color = (0, 0, 255)       # red
        elif hyst_count >= hyst_max - 1:
            hyst_color = (0, 100, 255)     # orange
        else:
            hyst_color = (0, 200, 255)     # yellow
        lines.append((hyst_text, hyst_color, 0.30))

    # Draw lines bottom-up
    y_pos = h - 6
    for text, color, scale in reversed(lines):
        cv2.putText(img, text, (4, y_pos), font, scale, color, 1, cv2.LINE_AA)
        y_pos -= int(16 * (scale / 0.32))


def _draw_basic_status(img, telemetry):
    """Fallback basic status (original behavior)."""
    h, w = img.shape[:2]
    dist_str = (f"{telemetry.obstacle_distance_m:.2f}m"
                if telemetry.obstacle_distance_m else "?")
    text_lines = [
        f"[{telemetry.mode}] scl={telemetry.throttle_scale:.2f} dist={dist_str}",
        f"M L={telemetry.motor_left} R={telemetry.motor_right} "
        f"{'ARMED' if telemetry.is_armed else 'DISARMED'}",
    ]
    for li, text in enumerate(text_lines):
        y_pos = h - 8 - (len(text_lines) - 1 - li) * 14
        cv2.putText(img, text, (4, y_pos),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.32,
                     (255, 255, 255), 1, cv2.LINE_AA)



def _colorize_depth(depth_frame: np.ndarray) -> np.ndarray | None:
    """Apply a JET colormap to a uint16 depth frame (mm). Returns BGR."""
    if cv2 is None or np is None:
        return None
    valid_mask = depth_frame > 0
    if not valid_mask.any():
        return None
    max_mm = min(float(np.max(depth_frame[valid_mask])), 10000.0)
    norm = np.zeros_like(depth_frame, dtype=np.uint8)
    if max_mm > 0:
        norm[valid_mask] = (
            np.clip(depth_frame[valid_mask].astype(np.float32) / max_mm, 0, 1) * 255
        ).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


# ---------------------------------------------------------------------------
# Main recorder class
# ---------------------------------------------------------------------------

class OakRecorder:
    """Activity-triggered recorder for OAK-D Lite streams.

    Lifecycle:
        recorder = OakRecorder(config)
        recorder.start(oak_reader)   # starts background threads
        ...
        recorder.update(telemetry)   # called each main-loop tick
        ...
        recorder.stop()              # flushes and closes files
    """

    def __init__(
        self,
        config: OakRecordingConfig,
        roi_vertical_offset_pct: float = 0.0,
        obstacle_config: ObstacleAvoidanceConfig | None = None,
    ) -> None:
        self._cfg = config
        self._roi_v_offset = float(roi_vertical_offset_pct)
        self._obs_cfg = obstacle_config
        rec_dir = Path(config.recording_dir)
        if not rec_dir.is_absolute():
            rec_dir = Path(__file__).resolve().parents[2] / rec_dir
        self._base_dir = rec_dir
        self._state = _RecState.IDLE
        self._last_trigger_time = 0.0
        self._session_start: str | None = None

        # Pre-buffer: ring of (timestamp, rgb_bytes, depth_frame_bytes)
        buf_size = int(config.pre_buffer_s * 30)  # ~30fps max
        self._pre_buffer: deque[tuple[float, bytes | None]] = deque(maxlen=max(buf_size, 1))

        # File handles for current session
        self._h265_fh = None
        self._mcap_ctx = None  # (writer, fh, channel_ids)

        # Background threads
        self._h265_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._h265_queue: deque | None = None

        # Rate limiters for MCAP snapshots
        self._last_mcap_rgb_ts = 0.0
        self._last_mcap_depth_ts = 0.0
        self._last_mcap_telem_ts = 0.0

        # Latest telemetry (set by main loop, read by recorder)
        self._telemetry: RecordingTelemetry | None = None
        self._latest_depth_frame: np.ndarray | None = None

        self._recording_queues: dict | None = None

        # Always-on preview frame for the web viewer (rate-limited to avoid excessive CPU)
        self._latest_annotated_jpeg: bytes | None = None
        self._latest_depth_jpeg: bytes | None = None
        self._preview_lock = threading.Lock()
        self._last_rgb_preview_ts = 0.0
        self._last_depth_preview_ts = 0.0
        rgb_fps = max(0.5, float(getattr(config, "preview_rgb_fps", 6.0)))
        depth_fps = max(0.5, float(getattr(config, "preview_depth_fps", 3.0)))
        self._rgb_preview_interval = 1.0 / rgb_fps
        self._depth_preview_interval = 1.0 / depth_fps
        self._rgb_stream_clients = 0
        self._depth_stream_clients = 0

    # -- Public preview accessors (for web viewer) ---------------------------

    def get_latest_annotated_jpeg(self) -> bytes | None:
        """Return the most recent annotated RGB JPEG, or None. Thread-safe."""
        with self._preview_lock:
            return self._latest_annotated_jpeg

    def get_latest_depth_jpeg(self) -> bytes | None:
        """Return the most recent colorized depth JPEG, or None. Thread-safe."""
        with self._preview_lock:
            return self._latest_depth_jpeg

    def get_latest_telemetry(self) -> RecordingTelemetry | None:
        return self._telemetry

    def wants_rgb_preview(self) -> bool:
        """Whether caller should fetch an RGB frame this tick."""
        return self._needs_rgb_preview()

    def wants_depth_preview(self) -> bool:
        """Whether caller should fetch a depth frame this tick."""
        return self._needs_depth_preview()

    def set_stream_client_connected(self, stream: str, connected: bool) -> None:
        """Track active web-stream clients to avoid unnecessary preview work."""
        with self._preview_lock:
            if stream == "rgb":
                if connected:
                    self._rgb_stream_clients += 1
                elif self._rgb_stream_clients > 0:
                    self._rgb_stream_clients -= 1
            elif stream == "depth":
                if connected:
                    self._depth_stream_clients += 1
                elif self._depth_stream_clients > 0:
                    self._depth_stream_clients -= 1

    @property
    def recording_state(self) -> str:
        return self._state

    @property
    def recordings_dir(self) -> Path:
        return self._base_dir

    # -- Lifecycle -----------------------------------------------------------

    def start(self, oak_reader) -> None:
        """Begin monitoring. Call after oak_reader.start()."""
        self._stop_event.clear()
        try:
            self._base_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            logger.warning("Cannot create recording dir %s", self._base_dir)
        self._cleanup_storage()

        if self._cfg.video_enabled:
            self._h265_queue = deque(maxlen=300)
            self._h265_thread = threading.Thread(
                target=self._h265_writer_loop, name="OakH265Writer", daemon=True
            )
            self._h265_thread.start()

        # Get recording queues from the reader (blocks until device ready)
        self._recording_queues = oak_reader.get_recording_queues(timeout_s=15.0)
        if self._recording_queues:
            logger.info("OAK recorder connected to device queues")
        else:
            logger.warning("OAK recorder: no recording queues available")

    def stop(self) -> None:
        """Flush and close all open files."""
        self._stop_event.set()
        self._end_session()
        if self._h265_thread is not None:
            self._h265_thread.join(timeout=5.0)
            self._h265_thread = None

    def update(self, telemetry: RecordingTelemetry, depth_frame=None, rgb_frame=None) -> None:
        """Called each main-loop tick. Drives the state machine and writes data."""
        self._telemetry = telemetry
        if depth_frame is not None:
            self._latest_depth_frame = depth_frame

        triggered = self._should_record(telemetry)
        now = time.monotonic()

        if self._state == _RecState.IDLE:
            if triggered:
                self._begin_session()
                self._flush_pre_buffer()
                self._state = _RecState.RECORDING
                self._last_trigger_time = now

        elif self._state == _RecState.RECORDING:
            if triggered:
                self._last_trigger_time = now
            else:
                self._state = _RecState.LINGERING

        elif self._state == _RecState.LINGERING:
            if triggered:
                self._state = _RecState.RECORDING
                self._last_trigger_time = now
            elif now - self._last_trigger_time > self._cfg.post_event_linger_s:
                self._end_session()
                self._state = _RecState.IDLE

        # Always poll device queues for fresh frames (recording H.265 etc.)
        self._poll_device_queues()

        # Only generate preview frames when they are actually needed.
        if rgb_frame is not None and self._needs_rgb_preview():
            self._update_rgb_preview(rgb_frame, telemetry)
        if self._needs_depth_preview():
            self._poll_depth_preview()

        # Write data if recording
        if self._state in (_RecState.RECORDING, _RecState.LINGERING):
            self._write_mcap_tick(telemetry)

    def _needs_rgb_preview(self) -> bool:
        with self._preview_lock:
            has_web_client = self._rgb_stream_clients > 0
        needs_mcap_rgb = (
            self._mcap_ctx is not None
            and self._cfg.mcap_enabled
            and self._cfg.mcap_image_fps > 0
            and self._state in (_RecState.RECORDING, _RecState.LINGERING)
            and self._mcap_images_allowed()
        )
        return has_web_client or needs_mcap_rgb

    def _needs_depth_preview(self) -> bool:
        with self._preview_lock:
            has_web_client = self._depth_stream_clients > 0
        needs_mcap_depth = (
            self._mcap_ctx is not None
            and self._cfg.mcap_enabled
            and self._cfg.mcap_depth_fps > 0
            and self._state in (_RecState.RECORDING, _RecState.LINGERING)
            and self._mcap_images_allowed()
        )
        return has_web_client or needs_mcap_depth

    def _mcap_images_allowed(self) -> bool:
        if not bool(getattr(self._cfg, "mcap_images_follow_only", True)):
            return True
        t = self._telemetry
        if t is None:
            return False
        return t.mode == "FOLLOW_ME" or len(t.person_detections) > 0

    # -- Trigger logic -------------------------------------------------------

    def _should_record(self, t: RecordingTelemetry) -> bool:
        if t.mode == "FOLLOW_ME":
            return True
        if not t.is_armed:
            return False
        return t.throttle_scale < float(getattr(self._cfg, "obstacle_trigger_scale", 1.0))

    # -- Session management --------------------------------------------------

    def _begin_session(self) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_start = ts
        session_dir = self._base_dir / ts
        try:
            session_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            logger.warning("Cannot create session dir %s", session_dir)
            return

        if self._cfg.video_enabled:
            try:
                self._h265_fh = open(session_dir / "video.h265", "wb")
            except Exception:
                logger.warning("Cannot open H.265 file", exc_info=True)

        if self._cfg.mcap_enabled:
            self._mcap_ctx = _make_mcap_writer(session_dir / "session.mcap")

        logger.info("Recording session started: %s", ts)

    def _end_session(self) -> None:
        if self._h265_fh is not None:
            try:
                self._h265_fh.flush()
                self._h265_fh.close()
            except Exception:
                pass
            self._h265_fh = None
            self._convert_h265_to_mp4()

        if self._mcap_ctx is not None:
            writer, fh, _ = self._mcap_ctx
            try:
                writer.finish()
                fh.close()
            except Exception:
                pass
            self._mcap_ctx = None

        if self._session_start:
            logger.info("Recording session ended: %s", self._session_start)
        self._session_start = None

    def _convert_h265_to_mp4(self) -> None:
        """Best-effort post-session conversion via ffmpeg."""
        if self._session_start is None:
            return
        h265_path = self._base_dir / self._session_start / "video.h265"
        mp4_path = h265_path.with_suffix(".mp4")
        if not h265_path.exists() or h265_path.stat().st_size == 0:
            return
        try:
            import subprocess
            subprocess.run(
                ["ffmpeg", "-y", "-framerate", "30", "-i", str(h265_path),
                 "-c", "copy", str(mp4_path)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=60, check=False,
            )
            if mp4_path.exists() and mp4_path.stat().st_size > 0:
                h265_path.unlink(missing_ok=True)
        except FileNotFoundError:
            pass  # ffmpeg not installed
        except Exception:
            logger.debug("H.265→MP4 conversion failed", exc_info=True)

    # -- Pre-buffer ----------------------------------------------------------

    def _flush_pre_buffer(self) -> None:
        """Write buffered H.265 packets to the newly opened file."""
        if self._h265_fh is None:
            return
        for _ts, data in self._pre_buffer:
            if data is not None:
                try:
                    self._h265_fh.write(data)
                except Exception:
                    break
        self._pre_buffer.clear()

    # -- Device queue polling ------------------------------------------------

    def _poll_device_queues(self) -> None:
        if self._recording_queues is None:
            return

        # H.265 encoded packets
        h265_q = self._recording_queues.get("h265")
        if h265_q is not None:
            try:
                pkt = h265_q.tryGet()
                if pkt is not None:
                    data = bytes(pkt.getData())
                    if self._state in (_RecState.RECORDING, _RecState.LINGERING):
                        if self._h265_queue is not None:
                            self._h265_queue.append(data)
                    else:
                        self._pre_buffer.append((time.monotonic(), data))
            except Exception:
                pass

        # (RGB and depth previews are now handled directly in update())

    def _update_rgb_preview(self, frame, telemetry: RecordingTelemetry) -> None:
        """Annotate an RGB frame and store JPEG for web viewer (rate-limited)."""
        if cv2 is None or frame is None or telemetry is None:
            return
        now = time.monotonic()
        if now - self._last_rgb_preview_ts < self._rgb_preview_interval:
            return
        self._last_rgb_preview_ts = now
        try:
            annotated = _annotate_rgb(
                frame,
                telemetry.person_detections,
                telemetry,
                obs_cfg=self._obs_cfg,
            )
            annotated = cv2.resize(annotated, (320, 240), interpolation=cv2.INTER_AREA)
            _, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 50])
            jpeg_bytes = jpeg.tobytes()
            with self._preview_lock:
                self._latest_annotated_jpeg = jpeg_bytes
        except Exception:
            logger.debug("RGB preview error", exc_info=True)

    def _poll_depth_preview(self) -> None:
        """Colorize the latest depth frame and store JPEG for web viewer (rate-limited)."""
        if cv2 is None or self._latest_depth_frame is None:
            return
        now = time.monotonic()
        if now - self._last_depth_preview_ts < self._depth_preview_interval:
            return
        self._last_depth_preview_ts = now
        try:
            colorized = _colorize_depth(self._latest_depth_frame)
            if colorized is None:
                return
            colorized = cv2.resize(colorized, (320, 240), interpolation=cv2.INTER_AREA)
            _, jpeg = cv2.imencode(".jpg", colorized, [cv2.IMWRITE_JPEG_QUALITY, 50])
            with self._preview_lock:
                self._latest_depth_jpeg = jpeg.tobytes()
        except Exception:
            pass

    # -- H.265 writer thread -------------------------------------------------

    def _h265_writer_loop(self) -> None:
        """Drain the H.265 queue and write to disk on a background thread."""
        while not self._stop_event.is_set():
            if self._h265_queue and self._h265_fh is not None:
                while self._h265_queue:
                    try:
                        data = self._h265_queue.popleft()
                        self._h265_fh.write(data)
                    except IndexError:
                        break
                    except Exception:
                        break
            time.sleep(0.01)
        # Drain remaining
        if self._h265_queue and self._h265_fh is not None:
            while self._h265_queue:
                try:
                    self._h265_fh.write(self._h265_queue.popleft())
                except Exception:
                    break

    # -- MCAP writing --------------------------------------------------------

    def _write_mcap_tick(self, telemetry: RecordingTelemetry) -> None:
        if self._mcap_ctx is None:
            return
        writer, _fh, channels = self._mcap_ctx
        now = time.monotonic()
        images_allowed = self._mcap_images_allowed()

        # Annotated RGB snapshot at configured fps
        if images_allowed and now - self._last_mcap_rgb_ts >= 1.0 / self._cfg.mcap_image_fps:
            self._last_mcap_rgb_ts = now
            self._write_mcap_rgb(writer, channels["rgb"], telemetry)

        # Colorized depth snapshot at configured fps
        if images_allowed and now - self._last_mcap_depth_ts >= 1.0 / self._cfg.mcap_depth_fps:
            self._last_mcap_depth_ts = now
            self._write_mcap_depth(writer, channels["depth"])

        # Telemetry at configured rate to avoid per-tick serialization overhead.
        telem_hz = max(0.5, float(getattr(self._cfg, "mcap_telemetry_hz", 10.0)))
        if now - self._last_mcap_telem_ts >= 1.0 / telem_hz:
            self._last_mcap_telem_ts = now
            self._write_mcap_telemetry(writer, channels["telemetry"], telemetry)

    def _write_mcap_rgb(self, writer, channel_id: int, telemetry: RecordingTelemetry) -> None:
        """Write the latest annotated JPEG (already produced by _poll_rgb_preview) to MCAP."""
        with self._preview_lock:
            jpeg_bytes = self._latest_annotated_jpeg
        if jpeg_bytes is None:
            return
        try:
            self._write_mcap_image(writer, channel_id, jpeg_bytes, telemetry.timestamp)
        except Exception:
            logger.debug("MCAP RGB write error", exc_info=True)

    def _write_mcap_depth(self, writer, channel_id: int) -> None:
        """Write the latest colorized depth JPEG (already produced by _poll_depth_preview) to MCAP."""
        with self._preview_lock:
            jpeg_bytes = self._latest_depth_jpeg
        if jpeg_bytes is None:
            return
        try:
            self._write_mcap_image(writer, channel_id, jpeg_bytes, time.time())
        except Exception:
            logger.debug("MCAP depth write error", exc_info=True)

    def _write_mcap_image(self, writer, channel_id: int, jpeg_bytes: bytes, ts: float) -> None:
        import base64
        import json
        sec = int(ts)
        nsec = int((ts - sec) * 1e9)
        msg = json.dumps({
            "timestamp": {"sec": sec, "nsec": nsec},
            "frame_id": "oak",
            "format": "jpeg",
            "data": base64.b64encode(jpeg_bytes).decode("ascii"),
        }).encode("utf-8")
        log_ns = int(ts * 1e9)
        writer.add_message(
            channel_id=channel_id,
            log_time=log_ns,
            data=msg,
            publish_time=log_ns,
        )

    def _write_mcap_telemetry(self, writer, channel_id: int, t: RecordingTelemetry) -> None:
        import json
        obj = {
            "timestamp": t.timestamp,
            "mode": t.mode,
            "throttle_scale": round(t.throttle_scale, 3),
            "obstacle_distance_m": round(t.obstacle_distance_m, 3) if t.obstacle_distance_m else None,
            "motor_left": t.motor_left,
            "motor_right": t.motor_right,
            "is_armed": t.is_armed,
            "num_persons": len(t.person_detections),
        }
        if t.depth_stats is not None:
            obj["depth_p5_mm"] = round(t.depth_stats.p5_mm, 1)
            obj["depth_p50_mm"] = round(t.depth_stats.p50_mm, 1)
            obj["depth_valid_pct"] = t.depth_stats.valid_pixel_pct
        if t.person_detections:
            obj["detections"] = [
                mcap_detection_dict(d) for d in t.person_detections
            ]
        if t.heading_deg is not None:
            obj["heading_deg"] = round(t.heading_deg, 1)
        if t.yaw_rate_dps is not None:
            obj["yaw_rate_dps"] = round(t.yaw_rate_dps, 1)
        if t.pursuit_mode is not None:
            obj["pursuit_mode"] = t.pursuit_mode
            obj["trail_length"] = t.trail_length or 0
        if t.trail_distance_m is not None:
            obj["trail_distance_m"] = round(t.trail_distance_m, 2)
        if t.trail_rejected_jump_count is not None:
            obj["trail_rejected_jump_count"] = int(t.trail_rejected_jump_count)
        if t.trail_rejected_speed_count is not None:
            obj["trail_rejected_speed_count"] = int(t.trail_rejected_speed_count)
        if t.follow_target_world_x is not None and t.follow_target_world_y is not None:
            obj["follow_target_world"] = {
                "x": round(t.follow_target_world_x, 3),
                "y": round(t.follow_target_world_y, 3),
            }
        if t.odom_x is not None:
            obj["odom"] = {
                "x": round(t.odom_x, 3),
                "y": round(t.odom_y or 0, 3),
                "theta_deg": round(t.odom_theta_deg or 0, 1),
            }
        if t.speed_offset is not None:
            obj["speed_offset"] = round(t.speed_offset, 1)
        if t.steer_offset is not None:
            obj["steer_offset"] = round(t.steer_offset, 1)
        if t.distance_error_m is not None:
            obj["distance_error_m"] = round(t.distance_error_m, 2)
        data = json.dumps(obj).encode("utf-8")
        log_ns = int(t.timestamp * 1e9)
        writer.add_message(
            channel_id=channel_id,
            log_time=log_ns,
            data=data,
            publish_time=log_ns,
        )

    # -- Storage cleanup -----------------------------------------------------

    def _cleanup_storage(self) -> None:
        """Delete old sessions exceeding age or total size limits."""
        if not self._base_dir.exists():
            return
        try:
            sessions = sorted(
                [d for d in self._base_dir.iterdir() if d.is_dir()],
                key=lambda d: d.name,
            )
        except Exception:
            return

        cutoff_s = time.time() - self._cfg.max_age_days * 86400
        for d in sessions:
            try:
                if d.stat().st_mtime < cutoff_s:
                    shutil.rmtree(d, ignore_errors=True)
            except Exception:
                pass

        # Enforce total size cap
        try:
            sessions = sorted(
                [d for d in self._base_dir.iterdir() if d.is_dir()],
                key=lambda d: d.name,
            )
            total_mb = 0.0
            for d in reversed(sessions):
                try:
                    size = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
                    total_mb += size / (1024 * 1024)
                except Exception:
                    pass
            if total_mb > self._cfg.max_total_mb:
                for d in sessions:  # oldest first
                    if total_mb <= self._cfg.max_total_mb:
                        break
                    try:
                        size = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
                        shutil.rmtree(d, ignore_errors=True)
                        total_mb -= size / (1024 * 1024)
                    except Exception:
                        pass
        except Exception:
            pass
