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
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ObstacleAvoidanceConfig, FollowMeConfig, GestureConfig, OakRecordingConfig, OakDetectionConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.control.gesture_control import HandData

logger = logging.getLogger(__name__)

PERSON_LABEL = 15       # MobileNet-SSD VOC label index for "person"
YOLO_PERSON_LABEL = 0   # YOLOv8 COCO label index for "person"

_HAND_MODELS_DIR = Path(__file__).resolve().parent.parent / "models" / "hand"
_HAND_SCRIPT_TEMPLATE = _HAND_MODELS_DIR / "hand_tracker_script.py"


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


@dataclass
class _ImuMetrics:
    """Lightweight IMU observability counters."""
    queue_msgs_received: int = 0
    queue_msgs_consumed: int = 0
    queue_msgs_dropped: int = 0
    queue_drain_count: int = 0
    packets_received: int = 0
    packets_consumed: int = 0
    packets_coalesced: int = 0
    last_batch_packets: int = 0
    last_drain_msgs: int = 0
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
        self._lm_net = None  # lazy-loaded OpenCV DNN for host-side LM
        self._imu_state = _ImuState()
        self._imu_metrics = _ImuMetrics()
        # Person label index depends on the active model
        _model_type = detection_config.model_type if detection_config is not None else "mobilenet-ssd"
        self._person_label: int = YOLO_PERSON_LABEL if _model_type == "yolov8n" else PERSON_LABEL
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

    def get_imu_data(self) -> tuple[_ImuState, float]:
        """Return (imu_state_copy, age_s). Thread-safe."""
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
            ), age

    def get_imu_metrics(self) -> dict:
        """Return a thread-safe snapshot of IMU observability counters."""
        now = time.monotonic()
        with self._lock:
            m = self._imu_metrics
            last_ts = self._imu_state.timestamp
            return {
                "queue_msgs_received": m.queue_msgs_received,
                "queue_msgs_consumed": m.queue_msgs_consumed,
                "queue_msgs_dropped": m.queue_msgs_dropped,
                "queue_drain_count": m.queue_drain_count,
                "packets_received": m.packets_received,
                "packets_consumed": m.packets_consumed,
                "packets_coalesced": m.packets_coalesced,
                "last_batch_packets": m.last_batch_packets,
                "last_drain_msgs": m.last_drain_msgs,
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

        loop_age_s = (now - loop_ts) if loop_ts > 0.0 else float("inf")
        depth_age_s = (now - depth_ts) if depth_ts > 0.0 else float("inf")
        depth_recv_age_s = (now - depth_recv_ts) if depth_recv_ts > 0.0 else float("inf")
        det_age_s = (now - det_ts) if det_ts > 0.0 else float("inf")
        rgb_age_s = (now - rgb_ts) if rgb_ts > 0.0 else float("inf")
        depth_frame_age_s = (now - depth_state_ts) if depth_state_ts > 0.0 else float("inf")
        rgb_frame_age_s = (now - rgb_state_ts) if rgb_state_ts > 0.0 else float("inf")

        loop_stale = (not running) or (loop_age_s > loop_stale_s)
        depth_stale = depth_recv_age_s > depth_stale_s
        det_stale = det_age_s > det_stale_s
        rgb_stale = rgb_age_s > rgb_stale_s

        return {
            "pipeline_running": running,
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
            "is_stale": loop_stale or depth_stale or det_stale or rgb_stale,
            "last_pipeline_error": pipe_err or None,
            "last_depth_error": depth_err or None,
            "last_detection_error": det_err or None,
            "last_rgb_error": rgb_err or None,
            "last_imu_error": imu_err or None,
        }

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

    def _run_pipeline(self) -> None:
        try:
            import depthai as dai
            import numpy as np
        except ImportError:
            logger.error("depthai or numpy not installed — OAK-D reader cannot start")
            return

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

            if _use_yolo:
                # depthai v3: SpatialDetectionNetwork.detectionParser defaults to
                # NNFamily=YOLO but leaves NumClasses=0/CoordinateSize=0/IouThreshold=0.0
                # which silently produces zero detections. Must configure the parser.
                spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork)
                spatial_nn.setBlobPath(str(_blob_abs))
                dp = spatial_nn.detectionParser
                dp.setNumClasses(80)
                dp.setCoordinateSize(4)
                dp.setAnchors([])      # anchor-free (YOLOv8)
                dp.setAnchorMasks({})  # anchor-free (YOLOv8)
                dp.setIouThreshold(_det_cfg.nms_threshold)
                # BGR888p gives 3×W×H bytes matching the tensor's expected size.
                # The default format (NV12) only gives 1.5×W×H bytes and triggers
                # "exceeds available data range" on the Myriad X, causing inference
                # to be skipped every frame.
                _yolo_cam_out = cam_rgb.requestOutput(
                    (_det_cfg.input_width, _det_cfg.input_height),
                    dai.ImgFrame.Type.BGR888p,
                )
                _yolo_cam_out.link(spatial_nn.input)
                stereo.depth.link(spatial_nn.inputDepth)
                logger.info(
                    "OAK-D: using YOLOv8n (detectionParser: classes=80 coordSize=4 iou=%.2f, %s, conf=%.2f, input=%dx%d)",
                    _det_cfg.nms_threshold, _blob_abs,
                    _det_cfg.confidence_threshold, _det_cfg.input_width, _det_cfg.input_height,
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
            if _use_yolo:
                spatial_nn.setConfidenceThreshold(_det_cfg.confidence_threshold)
                # NMS threshold setter name varies across depthai releases
                for _nms_setter in ("setNMSThreshold", "setNmsThreshold", "setIouThreshold"):
                    if hasattr(spatial_nn, _nms_setter):
                        getattr(spatial_nn, _nms_setter)(_det_cfg.nms_threshold)
                        break
            else:
                spatial_nn.setConfidenceThreshold(self._fm_cfg.detection_confidence)
            spatial_nn.setNumInferenceThreads(1)
            spatial_nn.setNumShavesPerInferenceThread(4)
            spatial_nn.input.setBlocking(False)
            # Scale factor 0.5-0.7 avoids background depth contamination on bounding box edges
            spatial_nn.setBoundingBoxScaleFactor(0.5)
            spatial_nn.setDepthLowerThreshold(int(self._fm_cfg.min_distance_m * 1000))
            spatial_nn.setDepthUpperThreshold(int(self._fm_cfg.max_distance_m * 1000))

            # Check if hand tracking will be enabled (before tracker setup).
            gesture_enabled = (
                self._gesture_cfg is not None
                and self._gesture_cfg.enabled
                and _HAND_MODELS_DIR.is_dir()
            )

            det_q = None
            tracker_enabled = False
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
            # For YOLO, use a dedicated small camera preview for rgb_stale
            # tracking.  We cannot reuse spatial_nn.passthrough because it
            # already has two on-device consumers (ObjectTracker inputTrackerFrame
            # and inputDetectionFrame); adding a host queue as a third consumer
            # causes the passthrough fan-out to stop delivering to the host in
            # depthai v3.  A separate ISP output at the same resolution avoids
            # the conflict and still verifies the camera ISP is alive.
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
            imu_q = imu_node.out.createOutputQueue()

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
            if tracker_enabled:
                logger.info("OAK ObjectTracker enabled for person detections")
            if hand_queues is not None:
                logger.info("Hand-gesture tracking enabled (PD on-device, LM on host)")
            self._device_ready.set()
            with self._lock:
                self._pipeline_running = True
                self._last_pipeline_loop_ts = time.monotonic()
                self._last_pipeline_error_msg = ""

        except Exception:
            logger.exception("Failed to build/start OAK-D pipeline")
            with self._lock:
                self._pipeline_running = False
                self._last_pipeline_error_msg = "pipeline_start_failed"
            return

        try:
            next_imu_poll = time.monotonic()
            while not self._stop_event.is_set() and pipeline.isRunning():
                with self._lock:
                    self._last_pipeline_loop_ts = time.monotonic()
                self._poll_depth(depth_q, spatial_depth_q, np)
                self._poll_detections(det_q)
                if hand_queues is not None:
                    self._poll_hand(hand_queues)
                with self._lock:
                    rgb_enabled = self._rgb_poll_enabled or self._rgb_always_poll
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
            logger.exception("OAK-D pipeline error")
            with self._lock:
                self._last_pipeline_error_msg = self._format_err("pipeline_loop", e)
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            self._device = None
            with self._lock:
                self._pipeline_running = False
                self._recording_queues = None
            self._device_ready.clear()

    # -- Hand-tracking pipeline helpers -----------------------------------------

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

        except Exception:
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

            robot_half_mm = getattr(self._obs_cfg, "robot_width_m", 0.0) * 500.0
            min_depth_mm = int(getattr(self._obs_cfg, "min_depth_mm", 600))
            min_valid_pct = float(getattr(self._obs_cfg, "min_valid_pct", 8.0))
            p50 = None
            valid_pct = None

            if robot_half_mm > 0:
                import math
                hfov = getattr(self._obs_cfg, "camera_hfov_deg", 73.0)
                fx = (w / 2.0) / math.tan(math.radians(hfov / 2.0))
                cx = w / 2.0
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

            if valid_depths.size == 0:
                with self._lock:
                    self._depth_quality_reject_count += 1
                return

            total_corridor_pixels = band.shape[0] * band.shape[1]
            corridor_valid_pct = (valid_depths.size / total_corridor_pixels) * 100.0 if total_corridor_pixels > 0 else 0.0
            if corridor_valid_pct < min_valid_pct:
                with self._lock:
                    self._depth_quality_reject_count += 1
                return

            p5 = float(np.percentile(valid_depths, 5))

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
            stats = DepthStats(
                min_distance_m=p5 / 1000.0,
                p5_mm=p5,
                p50_mm=p50,
                valid_pixel_pct=round(valid_pct, 1),
                timestamp=now,
            )
            with self._lock:
                self._depth_state.min_distance_m = p5 / 1000.0
                self._depth_state.timestamp = now
                self._depth_state.stats = stats
                self._last_depth_poll_ts = now
                self._last_depth_error_msg = ""
        except Exception as e:
            with self._lock:
                self._last_depth_error_msg = self._format_err("poll_depth", e)
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

    def _poll_detections(self, det_q) -> None:
        """Extract person detections with spatial coordinates.

        For YOLOv8 (80 COCO classes):
          - Builds PersonDetection list for Follow Me from class-0 detections that
            meet the Follow Me confidence threshold.
          - Builds ObjectDetection list for all classes to power safety-tier obstacle
            awareness (get_all_detections()).

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
            logger.debug("RGB poll error", exc_info=True)

    def _poll_imu(self, imu_q) -> None:
        """Extract latest accelerometer and gyroscope data from the OAK-D IMU."""
        try:
            imu_data = imu_q.tryGet()
            if imu_data is None:
                return
            drained_msgs = 0
            all_packets = list(getattr(imu_data, "packets", []) or [])
            while True:
                newer = imu_q.tryGet()
                if newer is None:
                    break
                drained_msgs += 1
                newer_packets = getattr(newer, "packets", None)
                if newer_packets:
                    all_packets.extend(list(newer_packets))
            if not all_packets:
                with self._lock:
                    m = self._imu_metrics
                    m.queue_msgs_received += (1 + drained_msgs)
                    m.queue_msgs_dropped += drained_msgs
                    if drained_msgs > 0:
                        m.queue_drain_count += 1
                    m.last_drain_msgs = drained_msgs
                return
            total_packets = len(all_packets)
            if self._imu_packet_mode == "bounded":
                selected = all_packets[-self._imu_max_packets_per_poll:]
            else:
                selected = [all_packets[-1]]

            now = time.monotonic()
            with self._lock:
                m = self._imu_metrics
                m.queue_msgs_received += (1 + drained_msgs)
                m.queue_msgs_consumed += 1
                m.queue_msgs_dropped += drained_msgs
                if drained_msgs > 0:
                    m.queue_drain_count += 1
                m.last_drain_msgs = drained_msgs
                m.last_batch_packets = total_packets
                m.packets_received += total_packets
                m.packets_consumed += len(selected)
                m.packets_coalesced += max(0, total_packets - len(selected))

                for pkt in selected:
                    accel = pkt.acceleroMeter
                    gyro = pkt.gyroscope
                    device_ts_s = 0.0
                    # Prefer device timestamps when available; used downstream for dt integration.
                    for ts_src in (gyro, accel, pkt):
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

                    self._imu_state.ax_mss = accel.x
                    self._imu_state.ay_mss = accel.y
                    self._imu_state.az_mss = accel.z
                    self._imu_state.gx_rads = gyro.x
                    self._imu_state.gy_rads = gyro.y
                    self._imu_state.gz_rads = gyro.z
                    self._imu_state.timestamp = now
                    self._imu_state.device_timestamp_s = device_ts_s

                    sample_ts = device_ts_s if device_ts_s > 0.0 else now
                    prev_ts = self._imu_prev_consumed_ts
                    if prev_ts > 0.0:
                        dt = sample_ts - prev_ts
                        if dt <= 0.0:
                            dt = now - prev_ts
                        m.cadence_last_s = dt
                        if m.cadence_samples == 0:
                            m.cadence_min_s = dt
                            m.cadence_max_s = dt
                            m.cadence_avg_s = dt
                        else:
                            if dt < m.cadence_min_s:
                                m.cadence_min_s = dt
                            if dt > m.cadence_max_s:
                                m.cadence_max_s = dt
                            n = m.cadence_samples
                            m.cadence_avg_s += (dt - m.cadence_avg_s) / (n + 1)
                        m.cadence_samples += 1
                    self._imu_prev_consumed_ts = sample_ts
        except Exception:
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
                        m.packets_consumed,
                        m.packets_coalesced,
                    )
            if warn_summary is not None:
                err_count, q_recv, q_cons, p_recv, p_cons, p_coal = warn_summary
                logger.warning(
                    "IMU poll path errors=%d q_recv=%d q_cons=%d p_recv=%d p_cons=%d p_coalesced=%d",
                    err_count, q_recv, q_cons, p_recv, p_cons, p_coal,
                )
            logger.debug("IMU poll error", exc_info=True)
