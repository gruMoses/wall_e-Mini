"""
Threaded OAK-D Lite depth and spatial person-detection reader.

Runs a DepthAI pipeline on a background daemon thread. Exposes thread-safe
APIs consumed by the main control loop:
  - get_min_distance()      -> (distance_m, age_s)  for obstacle avoidance
  - get_person_detections() -> list[PersonDetection]  for Follow Me
  - get_depth_stats()       -> DepthStats              for enriched telemetry
  - get_hand_data()         -> HandData | None          for gesture control

When recording is enabled, exposes additional queues for the recorder:
  - get_recording_queues()  -> dict of XLinkOut queue names
"""

from __future__ import annotations

import logging
import marshal  # kept for _build_hand_tracker_script compatibility
import math
import re
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ObstacleAvoidanceConfig, FollowMeConfig, GestureConfig, OakRecordingConfig, OakDetectionConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.control.gesture_control import HandData
from pi_app.hardware.oak_imu_yaw_producer import ImuPacket, ImuYawProducer

logger = logging.getLogger(__name__)

PERSON_LABEL = 15       # MobileNet-SSD VOC label index for "person"
YOLO_PERSON_LABEL = 0   # YOLOv8 COCO label index for "person"

_HAND_MODELS_DIR = Path(__file__).resolve().parent.parent / "models" / "hand"
_HAND_SCRIPT_TEMPLATE = _HAND_MODELS_DIR / "hand_tracker_script.py"

# DepthAI host IMU output queue (message slots, not wall-clock seconds).
# BMI270 is requested at 100 Hz with batch threshold 1 → ~1 packet/message.
# Default DepthAI maxSize=16 overflows during ordinary full-vision stalls
# (depth/YOLO/hand). 512 message slots give multi-second headroom at batch≈1,
# but this is NOT a hard 5.1 s guarantee: one queue message may hold multiple
# packets, and nonblocking overwrite loss is not observable via tryGet.
# Nonblocking so a stalled host never wedges the device.
IMU_HOST_QUEUE_MAX_SIZE = 512
IMU_HOST_QUEUE_BLOCKING = False
# Producer packet cap per drain (packets, not messages). Align with host
# message capacity at ~1 pkt/msg so a full host backlog is not silently
# halved. Keeps sort+fold memory O(cap). Raise only with measured multi-packet
# need — not a free pass to unbounded CPU.
IMU_MAX_PACKETS_PER_DRAIN = IMU_HOST_QUEUE_MAX_SIZE
# Drain-batch observability thresholds (NOT host-queue occupancy).
# A large drain means many messages were successfully retrieved in one poll;
# it does not measure remaining queue depth or overwritten samples.
IMU_DRAIN_BATCH_LARGE_FRAC = 0.75


@dataclass
class DepthStats:
    """Rich depth ROI statistics exposed for telemetry and recorder."""
    min_distance_m: float = float("inf")
    p5_mm: float = 0.0
    p50_mm: float = 0.0
    valid_pixel_pct: float = 0.0
    timestamp: float = 0.0


@dataclass
class _DepthState:
    min_distance_m: float = float("inf")
    timestamp: float = 0.0
    stats: DepthStats = field(default_factory=DepthStats)
    raw_frame: object = None  # numpy uint16 array or None


@dataclass
class _DetectionState:
    persons: list[PersonDetection]
    timestamp: float = 0.0


@dataclass
class _RgbState:
    frame: object = None  # numpy BGR array or None
    timestamp: float = 0.0


@dataclass
class _HandState:
    hand_data: HandData | None = None
    timestamp: float = 0.0


@dataclass
class _ImuState:
    ax_mss: float = 0.0  # m/s²
    ay_mss: float = 0.0
    az_mss: float = 0.0
    gx_rads: float = 0.0  # rad/s
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    timestamp: float = 0.0
    device_timestamp_s: float = 0.0
    # Producer-side unscaled cumulative free yaw (rad). Consumers apply scale once.
    cum_yaw_x_rad: float = 0.0
    cum_yaw_y_rad: float = 0.0
    cum_yaw_z_rad: float = 0.0
    cum_yaw_grav_rad: float = 0.0
    yaw_generation: int = 0
    producer_packets_integrated: int = 0
    producer_integrated_time_s: float = 0.0
    last_integrated_device_ts_s: float = 0.0


@dataclass
class _ImuMetrics:
    """Lightweight IMU observability counters.

    Packet-loss proof uses integrated vs received, gap/backlog drops, and
    drain-batch size stats — never a manufactured coalesced=0, and never
    claims of DepthAI host-queue occupancy or overwrite count (those are
    not exposed by nonblocking tryGet).
    """
    # Successfully tryGet'd host queue messages (all are consumed/batched).
    queue_msgs_received: int = 0
    queue_msgs_consumed: int = 0
    # Always 0: drained messages are not dropped. Nonblocking DepthAI overwrite
    # loss is not observable through this API — do not invent a drop count.
    queue_msgs_dropped: int = 0
    queue_drain_count: int = 0
    packets_received: int = 0  # drained/parsed from host queue (batch total)
    packets_parsed: int = 0    # successfully converted to ImuPacket
    packets_consumed: int = 0  # alias of integrated delta (compat)
    # Selection-mode drops (legacy "latest"/"bounded"). Lossless path never
    # increments this — it stays 0 by structure, not by forcing assignment.
    packets_coalesced: int = 0
    packets_integrated: int = 0
    packets_duplicate: int = 0
    packets_regressed: int = 0
    packets_restart: int = 0
    packets_gap_freeze: int = 0
    packets_invalid_ts: int = 0
    packets_backlog_dropped: int = 0
    last_batch_packets: int = 0
    last_drain_msgs: int = 0
    host_queue_max_size: int = IMU_HOST_QUEUE_MAX_SIZE
    host_queue_blocking: bool = IMU_HOST_QUEUE_BLOCKING
    # Drain-batch observability: messages successfully drained per poll.
    # NOT host-queue occupancy, NOT overwrite count, NOT overflow proof.
    drain_batch_high_water_msgs: int = 0
    drain_batch_large_events: int = 0      # drain size >= large frac of maxSize
    drain_batch_full_size_events: int = 0  # drain size >= maxSize (interesting only)
    cadence_samples: int = 0
    cadence_last_s: float = 0.0
    cadence_min_s: float = 0.0
    cadence_max_s: float = 0.0
    cadence_avg_s: float = 0.0
    error_count: int = 0
    first_error_ts: float = 0.0
    last_error_ts: float = 0.0
    last_error_msg: str = ""
    warning_emits: int = 0


@dataclass
class ObjectDetection:
    """Single detection from YOLOv8 (or MobileNet) with spatial coordinates."""
    label: int
    label_name: str
    confidence: float
    x_m: float        # lateral offset in metres (positive = right of robot)
    z_m: float        # forward distance in metres
    bbox: tuple       # (xmin, ymin, xmax, ymax) normalised 0-1
    safety_tier: str  # "stop", "slow", or "log"


@dataclass
class _AllDetsState:
    detections: list  # list[ObjectDetection]
    timestamp: float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Host-side tracklet layer (IoU + constant-velocity Kalman)
#
# The OAK device's on-device ObjectTracker emits stable track_ids, but the
# host-side YOLOv8 NeuralNetwork parse path and the raw SpatialDetectionNetwork
# path do not. This pure-numpy layer assigns stable, monotonic track_ids to
# person detections on those paths so follow_me can lock onto one person across
# frames / brief occlusions instead of falling back to "closest by depth".
# Zero new dependencies — hand-rolled constant-velocity Kalman.
# ─────────────────────────────────────────────────────────────────────────────

# Monotonic, process-global track-id source. Never reused, even across tracker
# resets, so a re-acquired target always gets a fresh id (matches the "new id
# after target absent > max_age" contract).
_track_id_lock = threading.Lock()
_track_id_counter = 0


def _next_track_id() -> int:
    global _track_id_counter
    with _track_id_lock:
        _track_id_counter += 1
        return _track_id_counter


def _iou(a, b) -> float:
    """Intersection-over-union of two (xmin, ymin, xmax, ymax) boxes."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


class Tracklet:
    """One tracked person — constant-velocity Kalman over bbox state.

    State vector ``x = [cx, cy, w, h, vx, vy]`` (bbox centre, size, centre
    velocity). Measurements observe ``[cx, cy, w, h]``; ``vx, vy`` are inferred.
    """

    def __init__(self, track_id: int, bbox) -> None:
        import numpy as np
        cx, cy, w, h = self._bbox_to_z(bbox)
        self.track_id = track_id
        self.x = np.array([cx, cy, w, h, 0.0, 0.0], dtype=float)
        self.P = np.eye(6, dtype=float) * 10.0
        # Constant-velocity transition: centre advances by its velocity each step.
        self.F = np.eye(6, dtype=float)
        self.F[0, 4] = 1.0
        self.F[1, 5] = 1.0
        # Measurement matrix: observe [cx, cy, w, h].
        self.H = np.zeros((4, 6), dtype=float)
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = self.H[3, 3] = 1.0
        self.Q = np.eye(6, dtype=float) * 1e-3   # process noise
        self.R = np.eye(4, dtype=float) * 1e-2   # measurement noise
        self.hits = 1
        self.age = 1
        self.time_since_update = 0

    @staticmethod
    def _bbox_to_z(bbox):
        x1, y1, x2, y2 = bbox
        return (x1 + x2) * 0.5, (y1 + y2) * 0.5, (x2 - x1), (y2 - y1)

    def predicted_bbox(self):
        """Bbox implied by the current (already-predicted) state."""
        cx, cy, w, h = self.x[0], self.x[1], self.x[2], self.x[3]
        w = max(0.0, w); h = max(0.0, h)
        return (cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5)

    def predict(self) -> None:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1

    def update(self, bbox) -> None:
        import numpy as np
        z = np.array(self._bbox_to_z(bbox), dtype=float)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.hits += 1
        self.time_since_update = 0


class TrackletTracker:
    """Greedy-IoU + Kalman multi-object tracker for person bboxes.

    ``update(detections)`` takes a list of ``(xmin, ymin, xmax, ymax)`` bboxes
    (normalised 0-1) and returns a parallel list of ``Optional[int]`` track_ids:
    the confirmed id for a detection matched to (or that grew into) a confirmed
    tracklet, else ``None``. A tracklet is confirmed only after ``min_hits``
    hits, so transient / first-seen detections report ``None`` — keeping the
    None path identical to today's untracked behaviour.
    """

    def __init__(self, iou_threshold: float = 0.3, min_hits: int = 3, max_age: int = 15) -> None:
        self._iou_threshold = float(iou_threshold)
        self._min_hits = int(min_hits)
        self._max_age = int(max_age)
        self._tracklets: list[Tracklet] = []

    @property
    def tracklets(self) -> list:
        return self._tracklets

    def update(self, detections):
        """Advance the tracker one frame; return parallel list of Optional[int]."""
        # 1. Predict every existing tracklet forward one frame.
        for t in self._tracklets:
            t.predict()

        n_det = len(detections)
        assigned = [None] * n_det  # track_id (or None) per detection

        # 2. Greedy IoU matching: predicted tracklet bbox vs new detections.
        unmatched_tracklets = set(range(len(self._tracklets)))
        unmatched_dets = set(range(n_det))
        if self._tracklets and n_det:
            pairs = []
            for ti, t in enumerate(self._tracklets):
                pb = t.predicted_bbox()
                for di in range(n_det):
                    iou = _iou(pb, detections[di])
                    if iou >= self._iou_threshold:
                        pairs.append((iou, ti, di))
            pairs.sort(reverse=True)  # highest IoU first
            for iou, ti, di in pairs:
                if ti in unmatched_tracklets and di in unmatched_dets:
                    self._tracklets[ti].update(detections[di])
                    unmatched_tracklets.discard(ti)
                    unmatched_dets.discard(di)
                    t = self._tracklets[ti]
                    if t.hits >= self._min_hits:
                        assigned[di] = t.track_id

        # 3. Unmatched detections become new tentative tracklets (report None
        #    until they reach min_hits).
        for di in unmatched_dets:
            t = Tracklet(_next_track_id(), detections[di])
            self._tracklets.append(t)
            if t.hits >= self._min_hits:  # only true if min_hits <= 1
                assigned[di] = t.track_id

        # 4. Age out tracklets unmatched for longer than max_age.
        self._tracklets = [
            t for t in self._tracklets if t.time_since_update <= self._max_age
        ]

        return assigned


class OakDepthReader:
    """Background OAK-D Lite reader following the ArduinoRCReader thread pattern."""

    def __init__(
        self,
        obstacle_config: ObstacleAvoidanceConfig,
        follow_me_config: FollowMeConfig,
        recording_config: OakRecordingConfig | None = None,
        gesture_config: GestureConfig | None = None,
        imu_poll_hz: float = 60.0,
        imu_packet_mode: str = "latest",
        imu_max_packets_per_poll: int = 4,
        detection_config: OakDetectionConfig | None = None,
    ) -> None:
        self._obs_cfg = obstacle_config
        self._fm_cfg = follow_me_config
        self._rec_cfg = recording_config
        self._gesture_cfg = gesture_config
        self._det_cfg = detection_config

        self._depth_state = _DepthState()
        self._det_state = _DetectionState(persons=[])
        self._all_dets_state = _AllDetsState(detections=[])
        self._rgb_state = _RgbState()
        self._hand_state = _HandState()
        # Host-side MediaPipe Hands is the most expensive per-frame CPU consumer
        # in this loop. Gate it (see set_hand_poll_enabled) so it does not run
        # while no gesture can have any effect. Defaults True so standalone /
        # CLI users keep the previous behavior.
        self._hand_poll_enabled = True
        # Factory camera calibration for the socket the depth frame lives in.
        # Set at session start, cleared on teardown. _intrinsics_for()
        # caches the resolution-specific (fx, cx) derived from it.
        self._calib = None
        self._calib_socket = None
        self._intrinsics_cache: dict[tuple[int, int], tuple[float, float]] = {}
        self._intrinsics_warned = False
        self._lm_net = None  # lazy-loaded OpenCV DNN for host-side LM
        self._imu_state = _ImuState()
        self._imu_metrics = _ImuMetrics()
        # Lossless producer-side yaw: every drained packet is integrated here
        # independent of depth/YOLO/RGB/hand load and of consumer poll rate.
        # Packet cap aligns with host message capacity at ~1 pkt/msg.
        self._imu_yaw_producer = ImuYawProducer(
            max_packets_per_drain=int(IMU_MAX_PACKETS_PER_DRAIN),
        )
        # Host output queue depth (also stamped into metrics for field tools).
        self._imu_host_queue_max_size = int(IMU_HOST_QUEUE_MAX_SIZE)
        self._imu_host_queue_blocking = bool(IMU_HOST_QUEUE_BLOCKING)
        self._imu_max_packets_per_drain = int(IMU_MAX_PACKETS_PER_DRAIN)
        self._imu_metrics.host_queue_max_size = self._imu_host_queue_max_size
        self._imu_metrics.host_queue_blocking = self._imu_host_queue_blocking
        # Person label index depends on the active model
        _model_type = detection_config.model_type if detection_config is not None else "mobilenet-ssd"
        self._person_label: int = YOLO_PERSON_LABEL if _model_type == "yolov8n" else PERSON_LABEL
        # Host-side tracklet layer for parse paths the OAK device doesn't track
        # (YOLOv8 NeuralNetwork parse + raw SpatialDetectionNetwork). Knobs from
        # FollowMeConfig; getattr defaults keep old configs working.
        self._tracklet_tracker = TrackletTracker(
            iou_threshold=getattr(self._fm_cfg, "tracklet_iou_threshold", 0.3),
            min_hits=getattr(self._fm_cfg, "tracklet_min_hits", 3),
            max_age=getattr(self._fm_cfg, "tracklet_max_age", 15),
        )
        self._imu_prev_consumed_ts = 0.0
        self._imu_last_warn_ts = 0.0
        self._lock = threading.Lock()

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Populated by _run_pipeline when recording is enabled; consumed by OakRecorder
        self._device = None
        self._recording_queues: dict | None = None
        self._device_ready = threading.Event()
        self._depth_stats_decimation = 3
        self._depth_stats_counter = 0
        self._rgb_poll_enabled = False
        self._rgb_always_poll = False  # set True for YOLO (passthrough monitoring)
        self._imu_poll_interval_s = 1.0 / max(1.0, float(imu_poll_hz))
        mode = str(imu_packet_mode or "latest").strip().lower()
        self._imu_packet_mode = mode if mode in ("latest", "bounded") else "latest"
        self._imu_max_packets_per_poll = max(1, int(imu_max_packets_per_poll))
        self._pipeline_running = False
        self._pipeline_dead = False  # True when pipeline has crashed and not yet recovered
        # USB disconnect / auto-reconnect supervisor state.
        self._connected = False           # True while a device session is live
        self._reconnect_count = 0         # number of recoveries since start()
        self._last_disconnect_ts = 0.0    # monotonic ts of the most recent drop
        self._last_pipeline_loop_ts = 0.0
        self._last_depth_poll_ts = 0.0
        self._last_depth_recv_ts = 0.0
        self._depth_recv_count = 0
        self._depth_quality_reject_count = 0
        self._last_detection_poll_ts = 0.0
        self._last_rgb_poll_ts = 0.0
        self._last_pipeline_error_msg = ""
        self._last_depth_error_msg = ""
        self._last_detection_error_msg = ""
        self._last_rgb_error_msg = ""
        self._last_imu_error_msg = ""

    # -- Public API ----------------------------------------------------------

    @staticmethod
    def detect() -> bool:
        """Return True if an OAK-D Lite (Myriad X) is visible on USB."""
        try:
            import depthai as dai
            if hasattr(dai.Device, "getAllAvailableDevices"):
                return len(dai.Device.getAllAvailableDevices()) > 0
            # depthai v3: use DeviceDiscovery
            return len(dai.DeviceDiscovery.getAllAvailableDevices()) > 0
        except Exception:
            return False

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_pipeline, name="OakDepthReader", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def get_min_distance(self) -> tuple[float, float]:
        """Return (min_distance_m, age_s). Thread-safe."""
        with self._lock:
            age = time.monotonic() - self._depth_state.timestamp
            return self._depth_state.min_distance_m, age

    def get_person_detections(self) -> list[PersonDetection]:
        """Return latest person detections. Thread-safe."""
        with self._lock:
            return list(self._det_state.persons)

    def get_depth_stats(self) -> DepthStats:
        """Return rich depth ROI statistics. Thread-safe."""
        with self._lock:
            return self._depth_state.stats

    def get_latest_depth_frame(self):
        """Return the most recent raw depth frame (uint16 mm) or None. Thread-safe."""
        with self._lock:
            return self._depth_state.raw_frame

    def get_latest_rgb_frame(self):
        """Return the most recent RGB preview frame (BGR numpy) or None. Thread-safe."""
        with self._lock:
            return self._rgb_state.frame

    def get_hand_data(self) -> HandData | None:
        """Return latest hand landmark data or None. Thread-safe."""
        with self._lock:
            return self._hand_state.hand_data

    def get_all_detections(self) -> list:
        """Return all latest object detections (all classes) as list[ObjectDetection].

        Each entry includes a safety_tier field ("stop", "slow", "log") based on
        OakDetectionConfig.stop_class_ids / slow_class_ids.  Useful for detection-based
        obstacle awareness beyond the depth-ROI approach.  Thread-safe.
        """
        with self._lock:
            return list(self._all_dets_state.detections)

    def set_rgb_poll_enabled(self, enabled: bool) -> None:
        """Enable/disable host RGB preview polling to reduce host copy load."""
        with self._lock:
            self._rgb_poll_enabled = bool(enabled)

    def set_hand_poll_enabled(self, enabled: bool) -> None:
        """Enable/disable host-side MediaPipe Hands inference.

        MediaPipe runs on the Pi CPU every poll iteration and competes with the
        ~30 Hz control loop. No gesture can produce an effect while the robot is
        disarmed: ACTIVATE requires ``safety_state.is_armed`` and DEACTIVATE only
        applies in FOLLOW_ME, which itself requires armed (see
        ``Controller.compute``). Disarmed is most of WALL-E's uptime, so gating
        on arm state is free CPU with no behavior change.

        Disabling clears the cached hand data, so a stale half-finished gesture
        sequence cannot survive across an arm cycle. ``GestureStateMachine.update``
        treats ``None`` as "no hand" and resets its hold counter.

        NOTE: while gestures are configured, the hand queue IS the RGB preview
        queue. The poll loop therefore keeps polling it for RGB when hand
        inference is off, so camera liveness and the web stream do not age out.
        """
        with self._lock:
            changed = bool(enabled) != self._hand_poll_enabled
            self._hand_poll_enabled = bool(enabled)
            if changed and not enabled:
                self._hand_state.hand_data = None
                self._hand_state.timestamp = time.monotonic()

    def get_imu_data(self) -> tuple[_ImuState, float]:
        """Return (imu_state_copy, age_s). Thread-safe.

        Includes producer cumulative free-yaw channels (unscaled rad). Consumers
        must apply ``oak_yaw_rate_scale`` once and must not re-integrate gyro×dt
        from the sparse latest snapshot for heading.

        ``gx_rads`` / ``gy_rads`` / ``gz_rads`` are the latest **raw** body rates
        (not bias-subtracted, not NMNI-gated). Bias and NMNI apply only inside
        the producer integrator that advances cum free-yaw. Gyro calibration
        must sample these raw rates (see ``get_imu_raw_gyro_dps``).
        """
        with self._lock:
            age = time.monotonic() - self._imu_state.timestamp if self._imu_state.timestamp else float("inf")
            return _ImuState(
                ax_mss=self._imu_state.ax_mss,
                ay_mss=self._imu_state.ay_mss,
                az_mss=self._imu_state.az_mss,
                gx_rads=self._imu_state.gx_rads,
                gy_rads=self._imu_state.gy_rads,
                gz_rads=self._imu_state.gz_rads,
                timestamp=self._imu_state.timestamp,
                device_timestamp_s=self._imu_state.device_timestamp_s,
                cum_yaw_x_rad=self._imu_state.cum_yaw_x_rad,
                cum_yaw_y_rad=self._imu_state.cum_yaw_y_rad,
                cum_yaw_z_rad=self._imu_state.cum_yaw_z_rad,
                cum_yaw_grav_rad=self._imu_state.cum_yaw_grav_rad,
                yaw_generation=self._imu_state.yaw_generation,
                producer_packets_integrated=self._imu_state.producer_packets_integrated,
                producer_integrated_time_s=self._imu_state.producer_integrated_time_s,
                last_integrated_device_ts_s=self._imu_state.last_integrated_device_ts_s,
            ), age

    def get_imu_raw_gyro_dps(self) -> tuple[tuple[float, float, float], float]:
        """Return ((gx, gy, gz) dps, age_s) of latest **raw** body gyro.

        Explicit calibration contract: never bias-subtracted or NMNI-gated.
        Same underlying sample as ``get_imu_data().g*_rads``; separate method
        so callers cannot confuse integrate-path rates with raw rates if the
        snapshot shape gains corrected fields later.
        """
        with self._lock:
            age = (
                time.monotonic() - self._imu_state.timestamp
                if self._imu_state.timestamp
                else float("inf")
            )
            return (
                (
                    math.degrees(self._imu_state.gx_rads),
                    math.degrees(self._imu_state.gy_rads),
                    math.degrees(self._imu_state.gz_rads),
                ),
                age,
            )

    def set_imu_gyro_bias_dps(self, gx_dps: float, gy_dps: float, gz_dps: float) -> None:
        """Push gyro bias into the producer integrator (applied before cum yaw)."""
        with self._lock:
            self._imu_yaw_producer.set_gyro_bias_dps(gx_dps, gy_dps, gz_dps)

    def set_imu_nmni(self, enabled: bool, threshold_dps: float = 0.3) -> None:
        """Configure per-packet NMNI on the producer integrator."""
        with self._lock:
            self._imu_yaw_producer.set_nmni(enabled, threshold_dps)

    def get_imu_metrics(self) -> dict:
        """Return a thread-safe snapshot of IMU observability counters."""
        now = time.monotonic()
        with self._lock:
            m = self._imu_metrics
            last_ts = self._imu_state.timestamp
            snap = self._imu_yaw_producer.snapshot()
            # Truthful accounting: not all received packets integrate (restart
            # seeds, duplicates, gap freezes). Selection coalescing is separate.
            packets_not_integrated = max(
                0,
                int(m.packets_parsed)
                - int(m.packets_integrated)
                - int(m.packets_duplicate)
                - int(m.packets_gap_freeze)
                - int(m.packets_regressed)
                - int(m.packets_restart)
                - int(m.packets_invalid_ts)
                - int(m.packets_backlog_dropped),
            )
            return {
                "queue_msgs_received": m.queue_msgs_received,
                "queue_msgs_consumed": m.queue_msgs_consumed,
                # Always 0 — drained messages are consumed/batched, not dropped.
                # DepthAI nonblocking overwrite loss is not observable here.
                "queue_msgs_dropped": m.queue_msgs_dropped,
                "queue_msgs_overwrite_observable": False,
                "queue_drain_count": m.queue_drain_count,
                "packets_received": m.packets_received,
                "packets_parsed": m.packets_parsed,
                "packets_drained": m.packets_received,
                "packets_consumed": m.packets_consumed,
                "packets_coalesced": m.packets_coalesced,
                "packets_integrated": m.packets_integrated,
                "packets_not_integrated_residual": packets_not_integrated,
                "packets_duplicate": m.packets_duplicate,
                "packets_regressed": m.packets_regressed,
                "packets_restart": m.packets_restart,
                "packets_gap_freeze": m.packets_gap_freeze,
                "packets_invalid_ts": m.packets_invalid_ts,
                "packets_backlog_dropped": m.packets_backlog_dropped,
                "last_batch_packets": m.last_batch_packets,
                "last_drain_msgs": m.last_drain_msgs,
                "host_queue_max_size": m.host_queue_max_size,
                "host_queue_blocking": m.host_queue_blocking,
                "max_packets_per_drain": int(self._imu_max_packets_per_drain),
                # Drain-batch observability (messages drained per poll).
                # Never infer host-queue occupancy or overwrite overflow from these.
                "drain_batch_high_water_msgs": m.drain_batch_high_water_msgs,
                "drain_batch_large_events": m.drain_batch_large_events,
                "drain_batch_full_size_events": m.drain_batch_full_size_events,
                "cadence_samples": m.cadence_samples,
                "cadence_last_s": m.cadence_last_s,
                "cadence_min_s": m.cadence_min_s,
                "cadence_max_s": m.cadence_max_s,
                "cadence_avg_s": m.cadence_avg_s,
                "error_count": m.error_count,
                "first_error_ts": m.first_error_ts,
                "last_error_ts": m.last_error_ts,
                "last_error_msg": m.last_error_msg,
                "warning_emits": m.warning_emits,
                "last_sample_timestamp": last_ts,
                "last_sample_age_s": (now - last_ts) if last_ts else float("inf"),
                # Producer yaw channels (unscaled rad / deg helpers).
                "producer_cum_yaw_x_deg": math.degrees(snap.cum_yaw_x_rad),
                "producer_cum_yaw_y_deg": math.degrees(snap.cum_yaw_y_rad),
                "producer_cum_yaw_z_deg": math.degrees(snap.cum_yaw_z_rad),
                "producer_cum_yaw_grav_deg": math.degrees(snap.cum_yaw_grav_rad),
                "producer_generation": snap.generation,
                "producer_integrated_time_s": snap.integrated_time_s,
                "producer_last_status": snap.last_status,
            }

    def get_health(self) -> dict:
        """Return OAK pipeline health snapshot for observability."""
        now = time.monotonic()
        hz = max(1.0, float(getattr(self._obs_cfg, "update_rate_hz", 10.0)))
        loop_stale_s = max(0.5, 5.0 / hz)
        depth_stale_s = max(1.0, 12.0 / hz)
        det_stale_s = max(1.0, 12.0 / hz)
        rgb_stale_s = 3.0

        with self._lock:
            running = bool(self._pipeline_running)
            connected = bool(self._connected)
            reconnect_count = self._reconnect_count
            last_disconnect_ts = self._last_disconnect_ts
            loop_ts = self._last_pipeline_loop_ts
            depth_ts = self._last_depth_poll_ts
            depth_recv_ts = self._last_depth_recv_ts
            depth_recv_count = self._depth_recv_count
            depth_reject_count = self._depth_quality_reject_count
            det_ts = self._last_detection_poll_ts
            rgb_ts = self._last_rgb_poll_ts
            depth_state_ts = self._depth_state.timestamp
            rgb_state_ts = self._rgb_state.timestamp
            pipe_err = self._last_pipeline_error_msg
            depth_err = self._last_depth_error_msg
            det_err = self._last_detection_error_msg
            rgb_err = self._last_rgb_error_msg
            imu_err = self._last_imu_error_msg
            rgb_expected = bool(self._rgb_poll_enabled or self._rgb_always_poll)

        loop_age_s = (now - loop_ts) if loop_ts > 0.0 else float("inf")
        depth_age_s = (now - depth_ts) if depth_ts > 0.0 else float("inf")
        depth_recv_age_s = (now - depth_recv_ts) if depth_recv_ts > 0.0 else float("inf")
        det_age_s = (now - det_ts) if det_ts > 0.0 else float("inf")
        rgb_age_s = (now - rgb_ts) if rgb_ts > 0.0 else float("inf")
        depth_frame_age_s = (now - depth_state_ts) if depth_state_ts > 0.0 else float("inf")
        rgb_frame_age_s = (now - rgb_state_ts) if rgb_state_ts > 0.0 else float("inf")
        last_disconnect_age_s = (now - last_disconnect_ts) if last_disconnect_ts > 0.0 else None

        loop_stale = (not running) or (loop_age_s > loop_stale_s)
        depth_stale = depth_recv_age_s > depth_stale_s
        det_stale = det_age_s > det_stale_s
        rgb_stale = rgb_expected and (rgb_age_s > rgb_stale_s)
        # "Critical" stale means obstacle/depth safety path is unhealthy.
        # Detection/RGB stalls are degraded but should not hard-mark pipeline stale.
        critical_stale = loop_stale or depth_stale

        return {
            "pipeline_running": running,
            "connected": connected,
            "reconnect_count": reconnect_count,
            "last_disconnect_ts": last_disconnect_ts if last_disconnect_ts > 0.0 else None,
            "last_disconnect_age_s": round(last_disconnect_age_s, 3) if last_disconnect_age_s is not None else None,
            "loop_age_s": round(loop_age_s, 3) if loop_age_s != float("inf") else None,
            "depth_age_s": round(depth_age_s, 3) if depth_age_s != float("inf") else None,
            "depth_recv_age_s": round(depth_recv_age_s, 3) if depth_recv_age_s != float("inf") else None,
            "depth_recv_count": depth_recv_count,
            "depth_quality_reject_count": depth_reject_count,
            "detections_age_s": round(det_age_s, 3) if det_age_s != float("inf") else None,
            "rgb_age_s": round(rgb_age_s, 3) if rgb_age_s != float("inf") else None,
            "depth_frame_age_s": round(depth_frame_age_s, 3) if depth_frame_age_s != float("inf") else None,
            "rgb_frame_age_s": round(rgb_frame_age_s, 3) if rgb_frame_age_s != float("inf") else None,
            "loop_stale": loop_stale,
            "depth_stale": depth_stale,
            "detections_stale": det_stale,
            "rgb_stale": rgb_stale,
            "rgb_expected": rgb_expected,
            "pipeline_dead": self._pipeline_dead,
            "critical_stale": critical_stale,
            "is_stale": loop_stale or depth_stale or det_stale or rgb_stale,
            "last_pipeline_error": pipe_err or None,
            "last_depth_error": depth_err or None,
            "last_detection_error": det_err or None,
            "last_rgb_error": rgb_err or None,
            "last_imu_error": imu_err or None,
        }

    @property
    def pipeline_dead(self) -> bool:
        """True when the OAK-D pipeline has crashed and is attempting recovery.

        Obstacle avoidance should treat depth as unavailable when this is True.
        """
        with self._lock:
            return self._pipeline_dead

    # Substrings that mark a fatal device/communication failure (USB drop,
    # XLink teardown, device closed). When a per-poll call raises one of these
    # the whole session is unrecoverable and must be rebuilt by the supervisor.
    _FATAL_COMM_MARKERS = (
        "x_link",
        "xlink",
        "communication",
        "device",
        "closed",
        "disconnect",
        "no available",
        "not running",
        "usb",
    )

    @classmethod
    def _is_fatal_comm_error(cls, exc: BaseException) -> bool:
        """Heuristically classify an exception as a fatal device/comm failure.

        depthai surfaces USB drops as plain RuntimeError with messages like
        "Communication exception - possible device error/misconfiguration" or
        "X_LINK_ERROR". We treat any such message (or an explicitly-named
        device/comm error class) as fatal so the supervisor rebuilds the device.
        Non-fatal, frame-level glitches keep their existing swallow-and-continue
        behaviour inside the individual _poll_* methods.
        """
        name = exc.__class__.__name__.lower()
        if "xlink" in name or "communication" in name:
            return True
        msg = str(exc).lower()
        return any(marker in msg for marker in cls._FATAL_COMM_MARKERS)

    @staticmethod
    def _format_err(prefix: str, exc: Exception) -> str:
        msg = str(exc).strip()
        if len(msg) > 220:
            msg = msg[:220] + "..."
        if msg:
            return f"{prefix}: {exc.__class__.__name__}: {msg}"
        return f"{prefix}: {exc.__class__.__name__}"

    @property
    def recording_enabled(self) -> bool:
        return self._rec_cfg is not None and self._rec_cfg.enabled

    def get_recording_queues(self, timeout_s: float = 10.0) -> dict | None:
        """Wait for the pipeline to be ready, then return output queue dict.

        Returns None if pipeline never became ready or recording is disabled.
        """
        if not self.recording_enabled:
            return None
        if not self._device_ready.wait(timeout=timeout_s):
            return None
        with self._lock:
            if self._recording_queues is None:
                return None
            return dict(self._recording_queues)

    # -- Pipeline Construction & Run (depthai v3 API) -----------------------

    # Reconnect backoff schedule (seconds): quick first retries, then steady
    # 10 s polling for the device to re-enumerate. The final value repeats.
    _RECONNECT_BACKOFF_S = (2.0, 5.0, 10.0)

    def _run_pipeline(self) -> None:
        """Supervisor loop: owns one device session at a time and re-establishes
        it after a USB disconnect / re-enumeration without a service restart.

        A "session" is build-pipeline -> open-device -> run poll loops. When the
        OAK USB device drops mid-run a queue ``get`` raises an X_LINK /
        communication error; ``_run_pipeline_once`` detects that, closes the
        device defensively, and returns. This loop then marks health as
        disconnected, backs off (interruptibly), optionally waits for the device
        to re-enumerate, and rebuilds — repeating until ``stop()`` is called.
        """
        try:
            import depthai as dai
            import numpy as np
        except ImportError:
            logger.error("depthai or numpy not installed -- OAK-D reader cannot start")
            return

        attempt = 0  # consecutive failed/ended sessions since last clean run
        while not self._stop_event.is_set():
            session_connected = False
            try:
                # Returns True if the device session actually opened (ran the
                # poll loop) before ending; False if the build/open failed.
                session_connected = bool(self._run_pipeline_once(dai, np))
            except Exception:
                logger.exception("OAK-D pipeline crashed -- will retry")
            if self._stop_event.is_set():
                break

            # The session has ended (fault or device drop). Mark disconnected so
            # consumers stop trusting depth, and count a reconnect attempt iff a
            # previously-live device went away (not a build failure on a missing
            # camera). We do NOT touch _depth_state.timestamp here: its age must
            # keep growing while the device is down so the staleness fail-safe in
            # compute_throttle_scale() stops autonomous motion.
            now = time.monotonic()
            with self._lock:
                self._pipeline_dead = True
                self._pipeline_running = False
                self._connected = False
                self._last_disconnect_ts = now
                if session_connected:
                    self._reconnect_count += 1

            # A live device that just dropped restarts the fast 2s->5s->10s ramp;
            # consecutive build/open failures (device truly absent) keep climbing
            # toward the steady 10s poll for re-enumeration.
            if session_connected:
                attempt = 0
            backoff_s = self._RECONNECT_BACKOFF_S[
                min(attempt, len(self._RECONNECT_BACKOFF_S) - 1)
            ]
            attempt += 1
            logger.warning(
                "OAK-D reconnect: device session ended (connected=%s); "
                "retry in %.1fs (reconnects=%d)",
                session_connected, backoff_s, self._reconnect_count,
            )
            # Interruptible backoff sleep — stop() fires the event and we exit
            # promptly instead of finishing the full backoff window.
            if self._stop_event.wait(timeout=backoff_s):
                break
            # Optionally wait for the device to re-enumerate before rebuilding so
            # we don't burn cycles building a pipeline against an absent device.
            # Bounded so a permanently-missing device still loops (and so a test
            # without the helper present is unaffected).
            self._wait_for_device(timeout_s=backoff_s)
        with self._lock:
            self._pipeline_dead = False
            self._connected = False

    def _wait_for_device(self, timeout_s: float) -> bool:
        """Poll the availability helper until a device appears or timeout/stop.

        Best-effort: any error in the helper is swallowed and treated as
        "unknown / proceed", so a missing or mocked-out ``detect()`` never blocks
        the supervisor. Returns True if a device was seen, else False.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while not self._stop_event.is_set():
            try:
                if self.detect():
                    return True
            except Exception:
                return False
            if time.monotonic() >= deadline:
                return False
            if self._stop_event.wait(timeout=0.25):
                return False
        return False

    def _run_pipeline_once(self, dai, np) -> bool:
        """Build + open one device session and run the poll loops.

        Returns True if the device session actually opened and ran the poll loop
        (i.e. a live device went away or stop() was requested), False if the
        build/open failed before the device was ever live. The supervisor uses
        this to decide whether an ended session counts as a reconnect.
        """

        try:
            pipeline = dai.Pipeline()

            # Determine model type early — affects stereo and NN config.
            _det_cfg = self._det_cfg
            _use_yolo = (
                _det_cfg is not None and _det_cfg.model_type == "yolov8n"
            )

            cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
            stereo.setLeftRightCheck(True)
            stereo.setOutputSize(640, 400)
            if _use_yolo:
                # Extended disparity halves the minimum detectable range (0.35 m vs 0.7 m)
                # which helps catch close-range obstacles and improves near-field Follow Me.
                stereo.setExtendedDisparity(True)
                # Align depth map to RGB camera so bounding-box spatial positions are correct.
                stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            mono_left.requestOutput((640, 400)).link(stereo.left)
            mono_right.requestOutput((640, 400)).link(stereo.right)

            # Device-side ROI spatial depth for obstacle distance + median depth.
            spatial_calc = pipeline.create(dai.node.SpatialLocationCalculator)
            rw = self._obs_cfg.roi_width_pct
            rh = self._obs_cfg.roi_height_pct
            rv = getattr(self._obs_cfg, "roi_vertical_offset_pct", 0.0)
            cy = max(rh / 2.0, min(1.0 - rh / 2.0, 0.5 + rv))
            roi_rect = dai.Rect(
                dai.Point2f(0.5 - rw / 2.0, cy - rh / 2.0),
                dai.Point2f(0.5 + rw / 2.0, cy + rh / 2.0),
            )

            spatial_min_cfg = dai.SpatialLocationCalculatorConfigData()
            spatial_min_cfg.depthThresholds.lowerThreshold = 400
            spatial_min_cfg.depthThresholds.upperThreshold = 10000
            spatial_min_cfg.roi = roi_rect
            spatial_min_cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
            spatial_calc.initialConfig.addROI(spatial_min_cfg)

            spatial_med_cfg = dai.SpatialLocationCalculatorConfigData()
            spatial_med_cfg.depthThresholds.lowerThreshold = 400
            spatial_med_cfg.depthThresholds.upperThreshold = 10000
            spatial_med_cfg.roi = roi_rect
            spatial_median_algo = getattr(
                dai.SpatialLocationCalculatorAlgorithm,
                "MEDIAN",
                dai.SpatialLocationCalculatorAlgorithm.AVERAGE,
            )
            spatial_med_cfg.calculationAlgorithm = spatial_median_algo
            spatial_calc.initialConfig.addROI(spatial_med_cfg)
            stereo.depth.link(spatial_calc.inputDepth)

            if _use_yolo:
                _blob = _det_cfg.model_path
                if _blob:
                    # NNModelDescription.model is Hub-slug only; resolve to absolute path for
                    # local blobs and use setBlobPath() + manual input wiring instead.
                    _blob_abs = (Path(__file__).resolve().parents[2] / _blob).resolve()
                    if not _blob_abs.exists():
                        logger.error(
                            "YOLOv8n blob not found at %s — falling back to MobileNet-SSD. "
                            "Run scripts/convert_yolov8n.py on the Pi to generate it.",
                            _blob_abs,
                        )
                        _use_yolo = False
                        with self._lock:
                            self._person_label = PERSON_LABEL
                else:
                    logger.warning(
                        "YOLOv8n requested but model_path is empty; "
                        "falling back to MobileNet-SSD. "
                        "Run scripts/convert_yolov8n.py on the Pi to generate a local blob."
                    )
                    _use_yolo = False
                    with self._lock:
                        self._person_label = PERSON_LABEL

            # Check if hand tracking will be enabled (before NN setup).
            gesture_enabled = (
                self._gesture_cfg is not None
                and self._gesture_cfg.enabled
                and _HAND_MODELS_DIR.is_dir()
            )

            det_q = None
            tracker_enabled = False
            if _use_yolo:
                # Use plain NeuralNetwork — the ultralytics ONNX export produces
                # pixel-space (cx, cy, w, h) in output0 (values 0..input_w/h).
                # DetectionParser / SpatialDetectionNetwork expect normalised [0,1]
                # coords and silently reject every box, yielding zero detections.
                # Parse the raw NNData tensor on the host and compute spatial
                # coordinates from the stereo depth frame instead.
                yolo_nn = pipeline.create(dai.node.NeuralNetwork)
                yolo_nn.setBlobPath(str(_blob_abs))
                yolo_nn.setNumInferenceThreads(2)
                yolo_nn.setNumShavesPerInferenceThread(4)
                yolo_nn.input.setBlocking(False)
                _yolo_cam_out = cam_rgb.requestOutput(
                    (_det_cfg.input_width, _det_cfg.input_height),
                    dai.ImgFrame.Type.BGR888p,
                )
                _yolo_cam_out.link(yolo_nn.input)
                det_q = yolo_nn.out.createOutputQueue(maxSize=1, blocking=False)
                # NeuralNetwork has no passthroughDepth; tap stereo.depth directly.
                depth_q = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
                logger.info(
                    "OAK-D: using YOLOv8n (NeuralNetwork+host-NMS: conf=%.2f nms=%.2f, %s, input=%dx%d)",
                    _det_cfg.confidence_threshold, _det_cfg.nms_threshold, _blob_abs,
                    _det_cfg.input_width, _det_cfg.input_height,
                )
            else:
                model_desc = dai.NNModelDescription(
                    model="luxonis/mobilenet-ssd:300x300", platform="RVC2"
                )
                logger.info(
                    "OAK-D: using MobileNet-SSD (%s)",
                    "yolov8n blob unavailable"
                    if _det_cfg is not None and _det_cfg.model_type == "yolov8n"
                    else "legacy",
                )
                spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
                    cam_rgb, stereo, model_desc,
                )
                spatial_nn.setConfidenceThreshold(self._fm_cfg.detection_confidence)
                spatial_nn.setNumInferenceThreads(1)
                spatial_nn.setNumShavesPerInferenceThread(4)
                spatial_nn.input.setBlocking(False)
                spatial_nn.setBoundingBoxScaleFactor(0.5)
                spatial_nn.setDepthLowerThreshold(int(self._fm_cfg.min_distance_m * 1000))
                spatial_nn.setDepthUpperThreshold(int(self._fm_cfg.max_distance_m * 1000))

                if gesture_enabled:
                    # ObjectTracker's passthrough fan-out stalls the pipeline when
                    # PD is also consuming camera frames.  Fall back to raw
                    # detections so both person-following and hand gestures work.
                    det_q = spatial_nn.out.createOutputQueue(maxSize=1, blocking=False)
                    logger.warning(
                        "ObjectTracker skipped (hand-tracking active); using raw detections",
                    )
                else:
                    try:
                        object_tracker = pipeline.create(dai.node.ObjectTracker)
                        object_tracker.setDetectionLabelsToTrack([self._person_label])
                        object_tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
                        object_tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

                        spatial_nn.out.link(object_tracker.inputDetections)
                        spatial_nn.passthrough.link(object_tracker.inputTrackerFrame)
                        spatial_nn.passthrough.link(object_tracker.inputDetectionFrame)
                        det_q = object_tracker.out.createOutputQueue(maxSize=1, blocking=False)
                        tracker_enabled = True
                    except Exception:
                        logger.warning(
                            "ObjectTracker init failed; falling back to raw detections",
                            exc_info=True,
                        )
                        det_q = spatial_nn.out.createOutputQueue(maxSize=1, blocking=False)

                depth_q = spatial_nn.passthroughDepth.createOutputQueue(maxSize=1, blocking=False)

            # Spatial ROI depth is also control-critical, so prefer newest sample.
            spatial_depth_q = spatial_calc.out.createOutputQueue(maxSize=1, blocking=False)

            # Preview feed -- when hand tracking is active the preview queue
            # comes from _build_hand_tracking_nodes (shared camera output).
            # For YOLO (NeuralNetwork), request a separate camera output at a
            # different resolution so depthai v3 does not de-duplicate the
            # requestOutput() call and return the NN-only internal output.
            if not gesture_enabled:
                if _use_yolo:
                    # Use a different resolution than the NN input so depthai v3
                    # does not de-duplicate the requestOutput() call and hand us
                    # back the same on-device-only output that feeds the NN.
                    _yolo_rgb_diag = cam_rgb.requestOutput((640, 480))
                    rgb_preview_q = _yolo_rgb_diag.createOutputQueue(maxSize=1, blocking=False)
                else:
                    rgb_preview_out = cam_rgb.requestOutput((640, 480))
                    rgb_preview_q = rgb_preview_out.createOutputQueue(maxSize=1, blocking=False)
            else:
                rgb_preview_q = None  # set below after hand pipeline build

            # Always poll RGB when YOLO is active so rgb_stale reflects camera
            # liveness regardless of whether gesture mode is also enabled.
            # When gesture_enabled=True, rgb_preview_q is reassigned to
            # hand_rgb_q below; _rgb_always_poll ensures _poll_rgb is called.
            if _use_yolo:
                with self._lock:
                    self._rgb_always_poll = True

            # IMU node (BMI270: accel + gyro at 100 Hz)
            imu_node = pipeline.create(dai.node.IMU)
            imu_node.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
            imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
            imu_node.setBatchReportThreshold(1)
            imu_node.setMaxBatchReports(10)
            # Explicit depth: default maxSize=16 overflows under full-vision stalls.
            # Nonblocking preserves device liveness if the host stalls longer.
            imu_q = imu_node.out.createOutputQueue(
                maxSize=int(self._imu_host_queue_max_size),
                blocking=bool(self._imu_host_queue_blocking),
            )
            assert int(self._imu_host_queue_max_size) >= 256, (
                "IMU host queue must buffer multiple seconds at 100 Hz "
                f"(got maxSize={self._imu_host_queue_max_size})"
            )
            assert self._imu_host_queue_blocking is False, (
                "IMU host queue must be nonblocking so a stalled host cannot wedge the device"
            )

            # -- Hand tracking pipeline (host-orchestrated) --
            hand_queues = None
            logger.warning("HAND_INIT: gesture_enabled=%s", gesture_enabled)
            if gesture_enabled:
                try:
                    hand_rgb_q = self._build_hand_tracking_nodes(
                        pipeline, dai, cam_rgb,
                    )
                    hand_queues = hand_rgb_q
                    rgb_preview_q = hand_rgb_q
                    logger.warning("Hand-tracking pipeline nodes created OK (host-side MediaPipe)")
                except Exception:
                    logger.warning(
                        "Hand-tracking pipeline init failed; gestures disabled",
                        exc_info=True,
                    )
                    hand_queues = None

            # Optional recording queues (v3: create from node outputs, no XLinkOut)
            # Disable recording when hand tracking is active to stay within ISP limits
            h265_q = None
            if self._rec_cfg is not None and self._rec_cfg.enabled and hand_queues is None:
                if self._rec_cfg.video_enabled:
                    encoder = pipeline.create(dai.node.VideoEncoder)
                    encoder.setDefaultProfilePreset(
                        30, dai.VideoEncoderProperties.Profile.H265_MAIN,
                    )
                    encoder.setBitrateKbps(self._rec_cfg.video_bitrate_kbps)
                    cam_rgb.requestOutput((1920, 1080), dai.ImgFrame.Type.NV12).link(encoder.input)
                    # Recording path should stay buffered to absorb disk/encoder jitter.
                    h265_q = encoder.bitstream.createOutputQueue(maxSize=30, blocking=False)

            rec_queues = {}
            if h265_q is not None:
                rec_queues["h265"] = h265_q
            with self._lock:
                self._recording_queues = rec_queues if rec_queues else None

            pipeline.start()
            logger.info("OAK-D pipeline started (depthai v3)")
            with self._lock:
                self._pipeline_dead = False
            if tracker_enabled:
                logger.info("OAK ObjectTracker enabled for person detections")
            if hand_queues is not None:
                logger.info("Hand-gesture tracking enabled (PD on-device, LM on host)")
            # Load the device's factory calibration for the socket the depth
            # frame is expressed in. Every OAK ships per-device intrinsics in
            # EEPROM; deriving fx from a hand-entered FOV constant is strictly
            # worse and is currently wrong (see _depth_intrinsics_for).
            self._load_calibration(pipeline, dai, _use_yolo)

            self._device_ready.set()
            with self._lock:
                self._pipeline_running = True
                self._connected = True
                self._last_pipeline_loop_ts = time.monotonic()
                self._last_pipeline_error_msg = ""
                # Device clock restarts with a new session — reseed yaw clocks
                # without jumping cumulative free-yaw.
                self._imu_yaw_producer.note_pipeline_restart()
                self._imu_prev_consumed_ts = 0.0

        except Exception:
            logger.exception("Failed to build/start OAK-D pipeline")
            with self._lock:
                self._pipeline_running = False
                self._connected = False
                self._last_pipeline_error_msg = "pipeline_start_failed"
            # Device never became live — report "not connected" so the
            # supervisor backs off without counting a reconnect.
            return False

        try:
            next_imu_poll = time.monotonic()
            while not self._stop_event.is_set() and pipeline.isRunning():
                with self._lock:
                    self._last_pipeline_loop_ts = time.monotonic()
                self._poll_depth(depth_q, spatial_depth_q, np)
                self._poll_detections(det_q)
                with self._lock:
                    hand_enabled = self._hand_poll_enabled
                if hand_queues is not None and hand_enabled:
                    self._poll_hand(hand_queues)
                with self._lock:
                    rgb_enabled = self._rgb_poll_enabled or self._rgb_always_poll
                if hand_queues is not None and not hand_enabled:
                    # The hand queue and the RGB preview queue are the same object
                    # when gestures are configured. _poll_hand normally refreshes
                    # RGB liveness; with it gated off, poll RGB instead so camera
                    # health and the web stream do not falsely age out.
                    rgb_enabled = True
                if rgb_enabled and rgb_preview_q is not None:
                    self._poll_rgb(rgb_preview_q)
                # IMU polling can run at a different cadence than depth polling.
                # Cap catch-up polls per outer loop to avoid runaway CPU under load.
                imu_polls = 0
                max_catchup = 4
                now = time.monotonic()
                while now >= next_imu_poll and imu_polls < max_catchup:
                    self._poll_imu(imu_q)
                    next_imu_poll += self._imu_poll_interval_s
                    imu_polls += 1
                    now = time.monotonic()
                if now - next_imu_poll > (self._imu_poll_interval_s * max_catchup):
                    next_imu_poll = now
                time.sleep(1.0 / self._obs_cfg.update_rate_hz)
        except Exception as e:
            # A fatal device/communication error (USB drop, XLink teardown) ends
            # the session; the supervisor will close, back off, and rebuild. Any
            # other escaped error is logged the same way — either way the session
            # is over. We deliberately do NOT refresh _depth_state.timestamp, so
            # its age keeps growing and the staleness fail-safe stops motion.
            if self._is_fatal_comm_error(e):
                logger.warning("OAK-D device communication lost: %s", e)
            else:
                logger.exception("OAK-D pipeline error")
            with self._lock:
                self._last_pipeline_error_msg = self._format_err("pipeline_loop", e)
        finally:
            # Defensive close — swallow any teardown error from an already-dead
            # device so the supervisor can always proceed to rebuild.
            try:
                pipeline.stop()
            except Exception:
                pass
            self._device = None
            # Drop calibration with the session: a reconnect re-reads it, and a
            # stale handle must never outlive the device it came from.
            with self._lock:
                self._calib = None
                self._calib_socket = None
                self._intrinsics_cache = {}
            with self._lock:
                self._pipeline_running = False
                self._connected = False
                self._recording_queues = None
            self._device_ready.clear()
        # The device session opened and ran (then ended via drop or stop()).
        # Report connected=True so the supervisor counts this as a reconnect
        # cycle (unless stop() was requested, which it checks separately).
        return True

    # -- Hand-tracking pipeline helpers -----------------------------------------

    def _load_calibration(self, pipeline, dai, use_yolo: bool) -> None:
        """Cache the device CalibrationHandler for the depth frame's socket.

        Which socket the depth frame is expressed in depends on alignment:

          * ``setDepthAlign(CAM_A)`` is applied on the YOLO path, so depth is
            warped into the RGB frame and carries CAM_A intrinsics.
          * Otherwise StereoDepth outputs in the rectified right mono frame,
            which is CAM_C on the OAK-D Lite.

        Failure is non-fatal: the callers fall back to the configured HFOV.
        """
        try:
            device = pipeline.getDefaultDevice()
            socket = (
                dai.CameraBoardSocket.CAM_A if use_yolo
                else dai.CameraBoardSocket.CAM_C
            )
            calib = device.readCalibration()
            with self._lock:
                self._calib = calib
                self._calib_socket = socket
                self._intrinsics_cache = {}
                self._intrinsics_warned = False
            # WARNING level on purpose: the root logger has no handler config,
            # so it sits at WARNING and INFO is dropped. This line is the only
            # runtime proof of where the corridor's focal length came from, and
            # a silent fallback to the config constant changes safety geometry.
            # Once per device session, not a hot path.
            logger.warning(
                "OAK calibration loaded from EEPROM for %s (depth %s RGB)",
                socket, "aligned to" if use_yolo else "not aligned to",
            )
        except Exception:
            with self._lock:
                self._calib = None
                self._calib_socket = None
            logger.warning(
                "Could not read OAK factory calibration; falling back to "
                "configured camera_hfov_deg", exc_info=True,
            )

    def get_intrinsics(self, width: int, height: int) -> tuple[float, float, float, float]:
        """Public: (fx, fy, cx, cy) in pixels for a CAM_A frame of this size.

        THE single source of camera geometry for the whole stack. Anything that
        projects between world and image — the obstacle corridor, person
        position, the recorder's overlay — must come through here rather than
        re-deriving a focal length from a hand-entered field of view. Deriving
        it separately is exactly how the overlay ended up drawing 81 deg
        geometry while the corridor ran on 70 deg.
        """
        return self._intrinsics_for(width, height)

    def _intrinsics_for(self, width: int, height: int) -> tuple[float, float, float, float]:
        """Return (fx, fy, cx, cy) in pixels for a frame of this size.

        Prefers the device's factory calibration, which is per-unit and already
        accounts for the crop/scale between the sensor's native resolution and
        the requested one. Falls back to trig on ``camera_hfov_deg``.

        NOTE: the fallback is a last resort and is strictly worse — it assumes a
        centred principal point, which is wrong by ~15 px on this unit. It
        exists only so a failed EEPROM read degrades instead of crashing; the
        failure is logged loudly because it silently changes safety geometry.
        """
        key = (int(width), int(height))
        with self._lock:
            cached = self._intrinsics_cache.get(key)
            calib = self._calib
            socket = self._calib_socket
        if cached is not None:
            return cached

        if calib is not None and socket is not None:
            try:
                intr = calib.getCameraIntrinsics(
                    socket, resizeWidth=int(width), resizeHeight=int(height),
                )
                fx = float(intr[0][0])
                fy = float(intr[1][1])
                cx = float(intr[0][2])
                cy = float(intr[1][2])
                if fx > 0.0 and fy > 0.0:
                    with self._lock:
                        self._intrinsics_cache[key] = (fx, fy, cx, cy)
                    # Once per resolution per session — records the numbers the
                    # corridor is actually running on, so a field check can be
                    # tied to real intrinsics instead of assumed ones.
                    logger.warning(
                        "OAK intrinsics %dx%d: fx=%.2f cx=%.2f (implied HFOV %.2f deg, "
                        "principal point %+.1f px off centre) — config fallback was %.1f deg",
                        width, height, fx, cx,
                        math.degrees(2.0 * math.atan((width / 2.0) / fx)),
                        cx - width / 2.0,
                        float(getattr(self._obs_cfg, "camera_hfov_deg", 70.0)),
                    )
                    return fx, fy, cx, cy
            except Exception:
                with self._lock:
                    already = self._intrinsics_warned
                    self._intrinsics_warned = True
                if not already:
                    logger.warning(
                        "getCameraIntrinsics failed for %dx%d; using "
                        "camera_hfov_deg fallback", width, height, exc_info=True,
                    )

        hfov = getattr(self._obs_cfg, "camera_hfov_deg", 70.0)
        fx = (width / 2.0) / math.tan(math.radians(hfov / 2.0))
        # Square pixels assumed in the fallback: fy == fx. Principal point taken
        # as frame centre, which is measurably wrong (+14.9 px on this unit) but
        # is the only guess available without calibration.
        return fx, fx, width / 2.0, height / 2.0

    def _build_hand_tracking_nodes(self, pipeline, dai, cam_rgb):
        """Create a camera output for host-side hand tracking via MediaPipe.

        All hand detection and landmark inference runs on the Pi CPU using
        MediaPipe Hands.  The OAK-D only provides an RGB stream.

        Returns rgb_q (host output queue for camera frames).
        """
        hand_cam_out = cam_rgb.requestOutput((640, 480))
        rgb_q = hand_cam_out.createOutputQueue(maxSize=1, blocking=False)
        return rgb_q

    @staticmethod
    def _build_hand_tracker_script(
        pad_h: int,
        img_h: int,
        img_w: int,
        frame_size: int,
        crop_w: int,
        pd_score_thresh: float = 0.5,
        lm_score_thresh: float = 0.5,
    ) -> str:
        """Read the Script node template and substitute geometry tokens."""
        template = _HAND_SCRIPT_TEMPLATE.read_text()
        replacements = {
            "_PAD_H": str(pad_h),
            "_IMG_H": str(img_h),
            "_IMG_W": str(img_w),
            "_FRAME_SIZE": str(frame_size),
            "_CROP_W": str(crop_w),
            "_PD_SCORE_THRESH": str(pd_score_thresh),
            "_LM_SCORE_THRESH": str(lm_score_thresh),
        }
        for token, value in replacements.items():
            template = template.replace(token, value)
        # Strip the module docstring (triple-quoted block at top)
        template = re.sub(r'^""".*?"""', '', template, count=1, flags=re.DOTALL)
        # Strip comments (the Script node's Python 3.9 runtime is limited)
        template = re.sub(r'#.*', '', template)
        template = re.sub(r'\n\s*\n', '\n', template)
        return template

    def _ensure_mp_hands(self):
        """Lazy-load the MediaPipe Hands solution."""
        if self._lm_net is not None:
            return self._lm_net
        import mediapipe as _mp
        self._lm_net = _mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=0,
        )
        logger.warning("Host-side MediaPipe Hands model loaded")
        return self._lm_net

    def _poll_hand(self, rgb_q) -> None:
        """Full hand tracking on host via MediaPipe Hands."""
        import cv2 as _cv2
        try:
            cam_msg = rgb_q.tryGet()
            if cam_msg is None:
                return

            while True:
                newer = rgb_q.tryGet()
                if newer is None:
                    break
                cam_msg = newer

            frame = cam_msg.getCvFrame()
            if frame is None:
                return

            # In gesture mode, hand tracking consumes the same RGB queue used for
            # diagnostics/preview. Update RGB liveness here so camera health and
            # web RGB stream do not falsely age out when _poll_rgb sees no frame.
            now = time.monotonic()
            with self._lock:
                self._rgb_state.frame = frame
                self._rgb_state.timestamp = now
                self._last_rgb_poll_ts = now
                self._last_rgb_error_msg = ""

            rgb_frame = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
            hands = self._ensure_mp_hands()
            results = hands.process(rgb_frame)

            if not results.multi_hand_landmarks:
                with self._lock:
                    self._hand_state.hand_data = None
                    self._hand_state.timestamp = time.monotonic()
                return

            hand_lm = results.multi_hand_landmarks[0]
            landmarks = []
            for lm in hand_lm.landmark:
                landmarks.append([lm.x, lm.y, lm.z])

            hd = HandData(norm_landmarks=landmarks, lm_score=1.0)
            with self._lock:
                self._hand_state.hand_data = hd
                self._hand_state.timestamp = time.monotonic()

        except Exception as e:
            with self._lock:
                self._last_rgb_error_msg = self._format_err("poll_hand_rgb", e)
            if self._is_fatal_comm_error(e):
                raise
            logger.warning("Hand poll error", exc_info=True)

    # -- Depth / detection / RGB / IMU polling --------------------------------

    def _poll_depth(self, depth_q, spatial_depth_q, np) -> None:
        """Extract minimum distance using trapezoidal corridor masking.

        Instead of a fixed-width rectangular ROI, each depth pixel is checked
        against the robot's physical width projected at that pixel's depth.
        Pixels whose real-world X offset exceeds half the robot width are
        excluded, producing a depth-dependent (trapezoidal) check region.
        """
        try:
            in_spatial = spatial_depth_q.tryGet()
            while True:
                newer_spatial = spatial_depth_q.tryGet()
                if newer_spatial is None:
                    break
                in_spatial = newer_spatial
            in_depth = depth_q.tryGet()
            if in_depth is None:
                return
            while True:
                newer_depth = depth_q.tryGet()
                if newer_depth is None:
                    break
                in_depth = newer_depth
            frame = in_depth.getFrame()  # uint16, millimetres
            recv_now = time.monotonic()
            with self._lock:
                self._last_depth_recv_ts = recv_now
                self._depth_recv_count += 1
                self._depth_state.raw_frame = frame
            h, w = frame.shape

            rh = self._obs_cfg.roi_height_pct
            rv = getattr(self._obs_cfg, "roi_vertical_offset_pct", 0.0)
            cy_norm = max(rh / 2.0, min(1.0 - rh / 2.0, 0.5 + rv))
            y0 = int(h * (cy_norm - rh / 2))
            y1 = int(h * (cy_norm + rh / 2))
            band = frame[y0:y1, :]

            # Mask out detected person bounding boxes so the depth corridor
            # measures obstacles AROUND/BEHIND the followed person, not the
            # person themselves.  Uses previous-frame detections (one-frame lag
            # is acceptable — bbox won't have moved significantly).
            with self._lock:
                person_bboxes = [p.bbox for p in self._det_state.persons if p.bbox]
            if person_bboxes:
                band = band.copy()  # avoid mutating the shared raw_frame
                band_h = band.shape[0]
                for bbox in person_bboxes:
                    bx1, by1, bx2, by2 = bbox
                    px1 = max(0, int(bx1 * w))
                    px2 = min(w, int(bx2 * w))
                    py1 = max(0, int(by1 * h) - y0)
                    py2 = min(band_h, int(by2 * h) - y0)
                    if py1 < py2 and px1 < px2:
                        band[py1:py2, px1:px2] = 0  # zeroed pixels fail > min_depth_mm

            robot_half_mm = getattr(self._obs_cfg, "robot_width_m", 0.0) * 500.0
            min_depth_mm = int(getattr(self._obs_cfg, "min_depth_mm", 600))
            min_valid_pct = float(getattr(self._obs_cfg, "min_valid_pct", 8.0))
            p50 = None
            valid_pct = None

            if robot_half_mm > 0:
                import math
                # Query with the FULL depth-frame height, not band.shape[0].
                # `band` is a vertical CROP of the frame (frame[y0:y1, :]), so
                # asking for intrinsics at the band height would describe a
                # differently-cropped sensor readout. Only fx/cx are used here
                # and both are horizontal, so the distinction does not change
                # today's numbers — but it would silently corrupt fy/cy for any
                # future caller.
                fx, _fy, cx, _cy = self._intrinsics_for(w, frame.shape[0])
                threshold = fx * robot_half_mm

                x_offsets = np.abs(np.arange(w, dtype=np.float32) - cx)
                depths_f = band.astype(np.float32)
                in_corridor = (depths_f * x_offsets[np.newaxis, :]) <= threshold
                valid_mask = (band > min_depth_mm) & in_corridor
                valid_depths = band[valid_mask]
            else:
                rw = self._obs_cfg.roi_width_pct
                x0 = int(w * (0.5 - rw / 2))
                x1 = int(w * (0.5 + rw / 2))
                roi = band[:, x0:x1]
                valid_mask = roi > min_depth_mm
                valid_depths = roi[valid_mask]

            # Use device-side spatial calculator for median only.
            if in_spatial is not None:
                try:
                    spatial_locations = in_spatial.getSpatialLocations()
                    if len(spatial_locations) > 1:
                        med_z_mm = float(spatial_locations[1].spatialCoordinates.z)
                        if med_z_mm > 0:
                            p50 = med_z_mm
                except Exception:
                    pass

            # Corridor distance — may be inf when person bbox fills the ROI.
            corridor_p5_mm = float("inf")
            corridor_rejected = False
            if valid_depths.size == 0:
                corridor_rejected = True
            else:
                corridor_pixel_count = int(in_corridor.sum())
                if corridor_pixel_count > 0:
                    corridor_valid_pct = (valid_depths.size / corridor_pixel_count) * 100.0
                else:
                    corridor_valid_pct = 0.0
                if corridor_valid_pct < min_valid_pct:
                    corridor_rejected = True
                else:
                    corridor_p5_mm = float(np.percentile(valid_depths, 5))

            # Canonical YOLO person/animal stop tier — see
            # _apply_safety_tier_override. A "stop"-tier detection within
            # safety_stop_radius_m forces the effective obstacle distance to 0,
            # regardless of what the depth corridor reports.
            safety_stop_radius = float(getattr(self._obs_cfg, "safety_stop_radius_m", 0.8))
            with self._lock:
                all_dets = list(self._all_dets_state.detections)
            effective_min_mm, _stop_det = self._apply_safety_tier_override(
                all_dets, corridor_p5_mm, safety_stop_radius
            )
            if _stop_det is not None:
                logger.info(
                    "Safety STOP: %s at %.2fm (< %.1fm radius)",
                    _stop_det.label_name, _stop_det.z_m, safety_stop_radius,
                )

            if effective_min_mm == float("inf"):
                # Fresh frame, corridor genuinely empty, no safety-tier trigger.
                # Update the depth-state timestamp so get_min_distance() reports a
                # fresh age (not stale) and consumers treat this as "corridor clear".
                # Do NOT skip the timestamp update — that would make a live but
                # empty corridor indistinguishable from a sensor failure.
                now_clear = time.monotonic()
                with self._lock:
                    self._depth_quality_reject_count += 1
                    self._depth_state.min_distance_m = float("inf")
                    self._depth_state.timestamp = now_clear
                    self._last_depth_poll_ts = now_clear
                    self._last_depth_error_msg = ""
                return

            p5 = effective_min_mm if corridor_rejected else corridor_p5_mm

            self._depth_stats_counter += 1
            if self._depth_stats_counter >= self._depth_stats_decimation:
                self._depth_stats_counter = 0
                band_small = band[::4, ::4]
                if robot_half_mm > 0:
                    x_off_small = np.abs(np.arange(band_small.shape[1], dtype=np.float32) - band_small.shape[1] / 2.0)
                    fx_small = fx * (band_small.shape[1] / w)
                    thr_small = fx_small * robot_half_mm
                    d_small = band_small.astype(np.float32)
                    corr_mask = (d_small * x_off_small[np.newaxis, :]) <= thr_small
                    stat_valid = band_small[(band_small > 0) & corr_mask]
                else:
                    stat_valid = band_small[band_small > 0]
                total_pixels = band_small.size
                if stat_valid.size > 0:
                    if p50 is None:
                        p50 = float(np.median(stat_valid))
                    valid_pct = (stat_valid.size / total_pixels) * 100.0 if total_pixels > 0 else 0.0

            now = time.monotonic()
            with self._lock:
                prev_stats = self._depth_state.stats
            if p50 is None:
                p50 = prev_stats.p50_mm if prev_stats.p50_mm > 0 else p5
            if valid_pct is None:
                valid_pct = prev_stats.valid_pixel_pct
            effective_min_m = effective_min_mm / 1000.0
            stats = DepthStats(
                min_distance_m=effective_min_m,
                p5_mm=p5,
                p50_mm=p50,
                valid_pixel_pct=round(valid_pct, 1),
                timestamp=now,
            )
            with self._lock:
                self._depth_state.min_distance_m = effective_min_m
                self._depth_state.timestamp = now
                self._depth_state.stats = stats
                self._last_depth_poll_ts = now
                self._last_depth_error_msg = ""
        except Exception as e:
            with self._lock:
                self._last_depth_error_msg = self._format_err("poll_depth", e)
            # A fatal device/comm failure (USB drop) must end the session so the
            # supervisor rebuilds — re-raise it. Frame-level glitches stay
            # swallowed so a single bad frame never tears the pipeline down.
            if self._is_fatal_comm_error(e):
                raise
            logger.debug("Depth poll error", exc_info=True)

    def _get_label_name(self, label: int) -> str:
        """Map a detection label index to a human-readable name."""
        det_cfg = self._det_cfg
        if det_cfg is not None and det_cfg.model_type == "yolov8n":
            names = det_cfg.coco_classes
            if 0 <= label < len(names):
                return names[label]
        return str(label)

    def _get_safety_tier(self, label: int) -> str:
        """Return 'stop', 'slow', or 'log' for a detection label."""
        det_cfg = self._det_cfg
        if det_cfg is None or det_cfg.model_type != "yolov8n":
            return "log"
        if label in det_cfg.stop_class_ids:
            return "stop"
        if label in det_cfg.slow_class_ids:
            return "slow"
        return "log"

    @staticmethod
    def _apply_safety_tier_override(detections, corridor_p5_mm: float,
                                    safety_stop_radius_m: float):
        """Canonical YOLO person/animal stop tier (pure, hardware-free).

        Lowers the effective forward obstacle distance (mm) for "stop"-tier
        detections:
          * a stop-tier detection within ``safety_stop_radius_m`` forces a hard
            stop — returns (0.0, that detection);
          * a more-distant stop-tier detection nearer than the corridor reading
            pulls the effective distance down to it.

        The returned distance is what poll_depth stores as ``min_distance_m``,
        so it flows through get_min_distance() ->
        ObstacleAvoidanceController.compute_throttle_scale(); 0.0 mm yields
        throttle scale 0.0 (full stop). This is the single, canonical stop tier.

        Returns (effective_min_mm, stop_det) where ``stop_det`` is the detection
        that forced the hard stop, else ``None``.
        """
        effective_min_mm = corridor_p5_mm
        for det in detections:
            if getattr(det, "safety_tier", "") == "stop" and getattr(det, "z_m", 0) > 0:
                if det.z_m < safety_stop_radius_m:
                    return 0.0, det
                det_mm = det.z_m * 1000.0
                if det_mm < effective_min_mm:
                    effective_min_mm = det_mm
        return effective_min_mm, None

    @staticmethod
    def _parse_yolo_tensor(
        tensor,
        conf_thresh: float,
        nms_thresh: float,
        input_w: int,
        input_h: int,
    ) -> list:
        """Parse a YOLOv8 NNData raw tensor to detected boxes.

        The ultralytics ONNX export produces output0 shape (1, 84, N):
          rows 0-3: cx, cy, w, h in PIXEL coordinates for the NN input image
          rows 4-83: 80 COCO class scores (post-sigmoid)

        Returns list of (label_int, conf, xmin_norm, ymin_norm, xmax_norm, ymax_norm).
        """
        import cv2
        import numpy as np

        arr = np.array(tensor)
        if arr.ndim == 3:
            arr = arr[0]   # (84, N)
        arr = arr.T        # (N, 84)

        scores = arr[:, 4:]               # (N, 80)
        max_scores = scores.max(axis=1)   # (N,)
        mask = max_scores >= conf_thresh
        arr_f = arr[mask]
        if arr_f.shape[0] == 0:
            return []

        cls_ids = scores[mask].argmax(axis=1)   # (M,)
        cls_scores = scores[mask].max(axis=1)   # (M,)

        cx = arr_f[:, 0]; cy = arr_f[:, 1]
        bw = arr_f[:, 2]; bh = arr_f[:, 3]
        x1 = cx - bw / 2.0; y1 = cy - bh / 2.0
        x2 = cx + bw / 2.0; y2 = cy + bh / 2.0

        results = []
        for c in np.unique(cls_ids):
            idxs = np.where(cls_ids == c)[0]
            boxes_px = [
                [float(x1[i]), float(y1[i]),
                 float(x2[i] - x1[i]), float(y2[i] - y1[i])]
                for i in idxs
            ]
            confs = [float(cls_scores[i]) for i in idxs]
            keep = cv2.dnn.NMSBoxes(boxes_px, confs, conf_thresh, nms_thresh)
            if keep is None:
                continue
            keep = keep.flatten() if hasattr(keep, "flatten") else list(keep)
            for ki in keep:
                i = idxs[int(ki)]
                x1n = max(0.0, float(x1[i]) / input_w)
                y1n = max(0.0, float(y1[i]) / input_h)
                x2n = min(1.0, float(x2[i]) / input_w)
                y2n = min(1.0, float(y2[i]) / input_h)
                results.append((int(c), float(cls_scores[i]), x1n, y1n, x2n, y2n))
        return results

    def _compute_spatial_from_depth(
        self,
        x1_n: float,
        y1_n: float,
        x2_n: float,
        y2_n: float,
        depth_frame,
    ) -> tuple:
        """Sample the stereo depth frame at a bbox to get (x_m, z_m).

        Uses the inner 50% of the bounding box to avoid background contamination,
        then derives lateral x_m via pinhole projection with the camera's HFoV.
        Returns (0.0, 0.0) when no valid depth pixels are found.
        """
        import math
        import numpy as np

        if depth_frame is None:
            return 0.0, 0.0

        dh, dw = depth_frame.shape
        min_depth_mm = int(self._fm_cfg.min_distance_m * 1000)
        max_depth_mm = int(self._fm_cfg.max_distance_m * 1000)

        cx_d = int(((x1_n + x2_n) / 2.0) * dw)
        cy_d = int(((y1_n + y2_n) / 2.0) * dh)
        bw_half = max(1, int((x2_n - x1_n) * 0.5 * dw / 2))
        bh_half = max(1, int((y2_n - y1_n) * 0.5 * dh / 2))
        x0d = max(0, cx_d - bw_half); x1d = min(dw, cx_d + bw_half)
        y0d = max(0, cy_d - bh_half); y1d = min(dh, cy_d + bh_half)

        roi = depth_frame[y0d:y1d, x0d:x1d]
        valid = roi[(roi > min_depth_mm) & (roi < max_depth_mm)]
        if valid.size == 0:
            return 0.0, 0.0

        z_m = float(np.median(valid)) / 1000.0
        fx, _fy, cx_principal, _cy = self._intrinsics_for(dw, depth_frame.shape[0])
        x_m = ((cx_d - cx_principal) / fx) * z_m
        return x_m, z_m

    def _assign_track_ids(self, persons: list) -> list:
        """Run the host-side tracklet layer over person detections.

        Returns a new list of PersonDetection with ``track_id`` populated
        (confirmed tracklets only; None otherwise). Used by the parse paths the
        OAK device doesn't track itself. Order is preserved.
        """
        if not persons:
            # Still advance the tracker so unmatched tracklets age out correctly.
            self._tracklet_tracker.update([])
            return persons
        track_ids = self._tracklet_tracker.update([p.bbox for p in persons])
        return [replace(p, track_id=tid) for p, tid in zip(persons, track_ids)]

    def _poll_detections(self, det_q) -> None:
        """Extract person detections with spatial coordinates.

        For YOLOv8 (80 COCO classes):
          - Parses raw NNData tensor on the host (NeuralNetwork node).
          - Applies host-side NMS; computes spatial coords from depth frame.
          - Builds PersonDetection list for Follow Me from class-0 detections.
          - Builds ObjectDetection list for all classes for safety-tier awareness.

        For MobileNet-SSD (backward-compat):
          - Behaviour is identical to the original implementation.
        """
        try:
            in_det = det_q.tryGet()
            if in_det is None:
                return
            while True:
                newer = det_q.tryGet()
                if newer is None:
                    break
                in_det = newer

            person_label = self._person_label
            fm_conf = self._fm_cfg.detection_confidence
            persons: list[PersonDetection] = []
            all_dets: list[ObjectDetection] = []

            # --- YOLOv8 NeuralNetwork path: raw tensor output ---
            if hasattr(in_det, "getAllLayerNames"):
                det_cfg = self._det_cfg
                conf_thresh = det_cfg.confidence_threshold if det_cfg else 0.45
                nms_thresh = det_cfg.nms_threshold if det_cfg else 0.5
                input_w = det_cfg.input_width if det_cfg else 640
                input_h = det_cfg.input_height if det_cfg else 352

                try:
                    raw = in_det.getTensor("output0")
                except Exception:
                    layer_names = in_det.getAllLayerNames()
                    raw = in_det.getTensor(layer_names[0]) if layer_names else None
                if raw is None:
                    return

                parsed = self._parse_yolo_tensor(raw, conf_thresh, nms_thresh, input_w, input_h)
                with self._lock:
                    depth_frame = self._depth_state.raw_frame

                for (cls_id, conf, x1n, y1n, x2n, y2n) in parsed:
                    x_m, z_m = self._compute_spatial_from_depth(x1n, y1n, x2n, y2n, depth_frame)
                    label_name = self._get_label_name(cls_id)
                    bbox = (x1n, y1n, x2n, y2n)
                    tier = self._get_safety_tier(cls_id)
                    all_dets.append(ObjectDetection(
                        label=cls_id, label_name=label_name, confidence=conf,
                        x_m=x_m, z_m=z_m, bbox=bbox, safety_tier=tier,
                    ))
                    if cls_id == person_label and conf >= fm_conf and z_m > 0.0:
                        persons.append(PersonDetection(
                            x_m=x_m, z_m=z_m, confidence=conf, bbox=bbox,
                        ))

                persons = self._assign_track_ids(persons)
                with self._lock:
                    self._det_state.persons = persons
                    self._det_state.timestamp = time.monotonic()
                    self._all_dets_state.detections = all_dets
                    self._all_dets_state.timestamp = self._det_state.timestamp
                    self._last_detection_poll_ts = self._det_state.timestamp
                    self._last_detection_error_msg = ""
                return

            tracklets = getattr(in_det, "tracklets", None)
            if tracklets is not None:
                # ObjectTracker output: tracklets already filtered to person_label
                for trk in tracklets:
                    src = getattr(trk, "srcImgDetection", None)
                    if src is None:
                        continue
                    is_person = (
                        getattr(src, "label", None) == person_label
                        or getattr(src, "labelName", "") == "person"
                    )
                    if not is_person:
                        continue
                    conf = float(getattr(src, "confidence", 0.0))
                    if conf < fm_conf:
                        continue
                    st = getattr(trk, "status", None)
                    st_name = getattr(st, "name", str(st)).upper() if st is not None else ""
                    if "LOST" in st_name or "REMOVED" in st_name:
                        continue
                    spatial = getattr(trk, "spatialCoordinates", None)
                    if spatial is None:
                        spatial = getattr(src, "spatialCoordinates", None)
                    if spatial is None:
                        continue
                    track_id = None
                    if hasattr(trk, "id"):
                        try:
                            tid = int(getattr(trk, "id"))
                            track_id = tid if tid >= 0 else None
                        except Exception:
                            track_id = None
                    x_m = float(getattr(spatial, "x", 0.0)) / 1000.0
                    z_m = float(getattr(spatial, "z", 0.0)) / 1000.0
                    bbox = (
                        float(getattr(src, "xmin", 0.0)),
                        float(getattr(src, "ymin", 0.0)),
                        float(getattr(src, "xmax", 0.0)),
                        float(getattr(src, "ymax", 0.0)),
                    )
                    persons.append(PersonDetection(
                        x_m=x_m, z_m=z_m, confidence=conf, bbox=bbox, track_id=track_id,
                    ))
                    all_dets.append(ObjectDetection(
                        label=person_label,
                        label_name="person",
                        confidence=conf,
                        x_m=x_m,
                        z_m=z_m,
                        bbox=bbox,
                        safety_tier=self._get_safety_tier(person_label),
                    ))
            else:
                # Raw SpatialDetectionNetwork output (all classes for YOLO, person-only for MobileNet)
                for det in getattr(in_det, "detections", []):
                    label = int(getattr(det, "label", -1))
                    label_name = getattr(det, "labelName", None) or self._get_label_name(label)
                    conf = float(getattr(det, "confidence", 0.0))
                    spatial = getattr(det, "spatialCoordinates", None)
                    if spatial is None:
                        continue
                    x_m = float(getattr(spatial, "x", 0.0)) / 1000.0
                    z_m = float(getattr(spatial, "z", 0.0)) / 1000.0
                    bbox = (
                        float(getattr(det, "xmin", 0.0)),
                        float(getattr(det, "ymin", 0.0)),
                        float(getattr(det, "xmax", 0.0)),
                        float(getattr(det, "ymax", 0.0)),
                    )
                    tier = self._get_safety_tier(label)
                    all_dets.append(ObjectDetection(
                        label=label, label_name=label_name, confidence=conf,
                        x_m=x_m, z_m=z_m, bbox=bbox, safety_tier=tier,
                    ))
                    # Person filter: Follow Me only cares about the person class
                    is_person = label == person_label or label_name == "person"
                    if is_person and conf >= fm_conf:
                        persons.append(PersonDetection(
                            x_m=x_m, z_m=z_m, confidence=conf, bbox=bbox,
                        ))
                    elif tier in ("stop", "slow") and label != person_label:
                        logger.debug(
                            "Safety-tier detection: %s (label=%d conf=%.2f z=%.1fm tier=%s)",
                            label_name, label, conf, z_m, tier,
                        )
                # Raw SpatialDetectionNetwork carries no track ids — assign host-side.
                persons = self._assign_track_ids(persons)

            with self._lock:
                self._det_state.persons = persons
                self._det_state.timestamp = time.monotonic()
                self._all_dets_state.detections = all_dets
                self._all_dets_state.timestamp = self._det_state.timestamp
                self._last_detection_poll_ts = self._det_state.timestamp
                self._last_detection_error_msg = ""
        except Exception as e:
            with self._lock:
                self._last_detection_error_msg = self._format_err("poll_detections", e)
            if self._is_fatal_comm_error(e):
                raise
            logger.debug("Detection poll error", exc_info=True)

    def _poll_rgb(self, rgb_q) -> None:
        """Grab the latest RGB preview frame from the NN passthrough."""
        try:
            msg = rgb_q.tryGet()
            if msg is None:
                return
            frame = msg.getCvFrame()
            with self._lock:
                self._rgb_state.frame = frame
                self._rgb_state.timestamp = time.monotonic()
                self._last_rgb_poll_ts = self._rgb_state.timestamp
                self._last_rgb_error_msg = ""
        except Exception as e:
            with self._lock:
                self._last_rgb_error_msg = self._format_err("poll_rgb", e)
            if self._is_fatal_comm_error(e):
                raise
            logger.debug("RGB poll error", exc_info=True)

    @staticmethod
    def _extract_imu_device_ts_s(pkt) -> float:
        """Best-effort device timestamp (seconds) from a depthai IMU packet."""
        device_ts_s = 0.0
        gyro = getattr(pkt, "gyroscope", None)
        accel = getattr(pkt, "acceleroMeter", None)
        for ts_src in (gyro, accel, pkt):
            if ts_src is None:
                continue
            ts = None
            try:
                getter = getattr(ts_src, "getTimestampDevice", None)
                if callable(getter):
                    ts = getter()
                elif hasattr(ts_src, "timestampDevice"):
                    ts = getattr(ts_src, "timestampDevice")
            except Exception:
                ts = None
            if ts is None:
                try:
                    getter = getattr(ts_src, "getTimestamp", None)
                    if callable(getter):
                        ts = getter()
                    elif hasattr(ts_src, "timestamp"):
                        ts = getattr(ts_src, "timestamp")
                except Exception:
                    ts = None
            if ts is None:
                continue
            try:
                if hasattr(ts, "total_seconds"):
                    device_ts_s = float(ts.total_seconds())
                elif hasattr(ts, "timestamp"):
                    device_ts_s = float(ts.timestamp())
                elif isinstance(ts, (int, float)):
                    device_ts_s = float(ts)
            except Exception:
                device_ts_s = 0.0
            if device_ts_s > 0.0:
                break
        return device_ts_s

    def _note_drain_batch(self, m: _ImuMetrics, drain_msg_count: int) -> None:
        """Update drain-batch observability (caller holds lock).

        Records how many host-queue messages were successfully drained in this
        poll. This is **not** host-queue occupancy, remaining depth, or proof of
        nonblocking overwrite loss (DepthAI does not expose those via tryGet).
        """
        if drain_msg_count > m.drain_batch_high_water_msgs:
            m.drain_batch_high_water_msgs = int(drain_msg_count)
        max_sz = max(1, int(m.host_queue_max_size or self._imu_host_queue_max_size))
        if drain_msg_count >= max_sz:
            m.drain_batch_full_size_events += 1
        elif drain_msg_count >= int(max_sz * IMU_DRAIN_BATCH_LARGE_FRAC):
            m.drain_batch_large_events += 1

    def _poll_imu(self, imu_q) -> None:
        """Drain IMU queue and integrate every packet into producer free-yaw.

        Depth/YOLO/RGB/hand load may delay this poll, so many packets can arrive
        in one drain. Historically only the newest (or a small tail) was kept
        and OakImuReader under-reported chalk turns. Yaw is now integrated here
        for **all** drained packets in timestamp order; the latest sample is
        still published for body-axis diagnostics.

        All successfully tryGet'd messages are consumed/batched — never counted
        as dropped. Nonblocking DepthAI overwrite loss is not observable here.
        """
        try:
            imu_data = imu_q.tryGet()
            if imu_data is None:
                return
            drained_extra = 0
            all_packets = list(getattr(imu_data, "packets", []) or [])
            while True:
                newer = imu_q.tryGet()
                if newer is None:
                    break
                drained_extra += 1
                newer_packets = getattr(newer, "packets", None)
                if newer_packets:
                    all_packets.extend(list(newer_packets))
            # First message + extras drained this poll (all successfully retrieved).
            drain_msg_count = 1 + drained_extra
            if not all_packets:
                with self._lock:
                    m = self._imu_metrics
                    m.queue_msgs_received += drain_msg_count
                    m.queue_msgs_consumed += drain_msg_count
                    # queue_msgs_dropped stays 0: drained msgs are consumed.
                    if drain_msg_count > 0:
                        m.queue_drain_count += 1
                    m.last_drain_msgs = drain_msg_count
                    self._note_drain_batch(m, drain_msg_count)
                return

            total_packets = len(all_packets)
            now = time.monotonic()
            parsed: list[ImuPacket] = []
            for pkt in all_packets:
                try:
                    accel = pkt.acceleroMeter
                    gyro = pkt.gyroscope
                    device_ts_s = self._extract_imu_device_ts_s(pkt)
                    # 0.0 from extractor means "not found" → NaN sentinel for producer.
                    dev_for_pkt = device_ts_s if device_ts_s > 0.0 else float("nan")
                    parsed.append(
                        ImuPacket(
                            device_ts_s=dev_for_pkt,
                            host_ts_s=now,
                            gx_rads=float(gyro.x),
                            gy_rads=float(gyro.y),
                            gz_rads=float(gyro.z),
                            ax_mss=float(accel.x),
                            ay_mss=float(accel.y),
                            az_mss=float(accel.z),
                        )
                    )
                except Exception:
                    # Skip malformed packet; continue draining the rest.
                    continue

            with self._lock:
                m = self._imu_metrics
                m.queue_msgs_received += drain_msg_count
                # Every successfully drained message is consumed into the batch.
                m.queue_msgs_consumed += drain_msg_count
                # Do not count drained messages as dropped. Overwrite loss from
                # a full nonblocking DepthAI queue is not observable via tryGet.
                if drain_msg_count > 0:
                    m.queue_drain_count += 1
                m.last_drain_msgs = drain_msg_count
                m.last_batch_packets = total_packets
                m.packets_received += total_packets
                m.packets_parsed += len(parsed)
                self._note_drain_batch(m, drain_msg_count)

                # Always integrate every successfully parsed packet (lossless yaw).
                # imu_packet_mode is retained for metrics compatibility only:
                # "latest"/"bounded" no longer drop samples from the yaw path, so
                # packets_coalesced is never incremented (stays 0 by structure).
                before = self._imu_yaw_producer.snapshot()
                snap = self._imu_yaw_producer.ingest(parsed, host_now_s=now)
                integrated_delta = snap.packets_integrated - before.packets_integrated
                m.packets_consumed += max(0, integrated_delta)
                m.packets_integrated = snap.packets_integrated
                m.packets_duplicate = snap.packets_duplicate
                m.packets_regressed = snap.packets_regressed
                m.packets_restart = snap.packets_restart
                m.packets_gap_freeze = snap.packets_gap_freeze
                m.packets_invalid_ts = snap.packets_invalid_ts
                m.packets_backlog_dropped = snap.packets_backlog_dropped
                m.cadence_samples = snap.cadence_samples
                m.cadence_last_s = snap.cadence_last_s
                m.cadence_min_s = snap.cadence_min_s
                m.cadence_max_s = snap.cadence_max_s
                m.cadence_avg_s = snap.cadence_avg_s

                self._imu_state.ax_mss = snap.ax_mss
                self._imu_state.ay_mss = snap.ay_mss
                self._imu_state.az_mss = snap.az_mss
                self._imu_state.gx_rads = snap.gx_rads
                self._imu_state.gy_rads = snap.gy_rads
                self._imu_state.gz_rads = snap.gz_rads
                self._imu_state.timestamp = snap.timestamp if snap.timestamp > 0.0 else now
                self._imu_state.device_timestamp_s = snap.device_timestamp_s
                self._imu_state.cum_yaw_x_rad = snap.cum_yaw_x_rad
                self._imu_state.cum_yaw_y_rad = snap.cum_yaw_y_rad
                self._imu_state.cum_yaw_z_rad = snap.cum_yaw_z_rad
                self._imu_state.cum_yaw_grav_rad = snap.cum_yaw_grav_rad
                self._imu_state.yaw_generation = snap.generation
                self._imu_state.producer_packets_integrated = snap.packets_integrated
                self._imu_state.producer_integrated_time_s = snap.integrated_time_s
                self._imu_state.last_integrated_device_ts_s = snap.last_integrated_device_ts_s

                sample_ts = (
                    snap.device_timestamp_s
                    if snap.device_timestamp_s > 0.0
                    else self._imu_state.timestamp
                )
                self._imu_prev_consumed_ts = sample_ts
        except Exception as e:
            # A fatal device/comm failure ends the session — propagate so the
            # supervisor rebuilds. Non-fatal errors keep the existing
            # count-and-warn behaviour below.
            if self._is_fatal_comm_error(e):
                raise
            now = time.monotonic()
            warn_summary = None
            with self._lock:
                m = self._imu_metrics
                m.error_count += 1
                if m.first_error_ts == 0.0:
                    m.first_error_ts = now
                m.last_error_ts = now
                m.last_error_msg = "IMU poll error"
                self._last_imu_error_msg = "poll_imu error"
                # Emit warning on first error, then at most every 30s.
                should_warn = (m.warning_emits == 0) or (now - getattr(self, "_imu_last_warn_ts", 0.0) >= 30.0)
                if should_warn:
                    self._imu_last_warn_ts = now
                    m.warning_emits += 1
                    warn_summary = (
                        m.error_count,
                        m.queue_msgs_received,
                        m.queue_msgs_consumed,
                        m.packets_received,
                        m.packets_integrated,
                        m.packets_backlog_dropped,
                    )
            if warn_summary is not None:
                err_count, q_recv, q_cons, p_recv, p_int, p_drop = warn_summary
                logger.warning(
                    "IMU poll path errors=%d q_recv=%d q_cons=%d p_recv=%d p_integrated=%d p_backlog_drop=%d",
                    err_count, q_recv, q_cons, p_recv, p_int, p_drop,
                )
            logger.debug("IMU poll error", exc_info=True)
