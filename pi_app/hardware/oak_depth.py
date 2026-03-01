"""
Threaded OAK-D Lite depth and spatial person-detection reader.

Runs a DepthAI pipeline on a background daemon thread. Exposes thread-safe
APIs consumed by the main control loop:
  - get_min_distance()      -> (distance_m, age_s)  for obstacle avoidance
  - get_person_detections() -> list[PersonDetection]  for Follow Me
  - get_depth_stats()       -> DepthStats              for enriched telemetry

When recording is enabled, exposes additional queues for the recorder:
  - get_recording_queues()  -> dict of XLinkOut queue names
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ObstacleAvoidanceConfig, FollowMeConfig, OakRecordingConfig
from pi_app.control.follow_me import PersonDetection

logger = logging.getLogger(__name__)

PERSON_LABEL = 15  # MobileNet-SSD label index for "person"


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
class _ImuState:
    ax_mss: float = 0.0  # m/s²
    ay_mss: float = 0.0
    az_mss: float = 0.0
    gx_rads: float = 0.0  # rad/s
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    timestamp: float = 0.0


class OakDepthReader:
    """Background OAK-D Lite reader following the ArduinoRCReader thread pattern."""

    def __init__(
        self,
        obstacle_config: ObstacleAvoidanceConfig,
        follow_me_config: FollowMeConfig,
        recording_config: OakRecordingConfig | None = None,
    ) -> None:
        self._obs_cfg = obstacle_config
        self._fm_cfg = follow_me_config
        self._rec_cfg = recording_config

        self._depth_state = _DepthState()
        self._det_state = _DetectionState(persons=[])
        self._rgb_state = _RgbState()
        self._imu_state = _ImuState()
        self._lock = threading.Lock()

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Populated by _run_pipeline when recording is enabled; consumed by OakRecorder
        self._device = None
        self._recording_queues: dict | None = None
        self._device_ready = threading.Event()

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
            ), age

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
            spatial_nn.setConfidenceThreshold(self._fm_cfg.detection_confidence)
            spatial_nn.input.setBlocking(False)
            spatial_nn.setBoundingBoxScaleFactor(0.5)
            spatial_nn.setDepthLowerThreshold(int(self._fm_cfg.min_distance_m * 1000))
            spatial_nn.setDepthUpperThreshold(int(self._fm_cfg.max_distance_m * 1000))

            det_q = spatial_nn.out.createOutputQueue(maxSize=4, blocking=False)
            depth_q = spatial_nn.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)
            rgb_preview_q = cam_rgb.requestOutput((640, 480)).createOutputQueue(maxSize=2, blocking=False)

            # IMU node (BMI270: accel + gyro at 100 Hz)
            imu_node = pipeline.create(dai.node.IMU)
            imu_node.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
            imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
            imu_node.setBatchReportThreshold(1)
            imu_node.setMaxBatchReports(10)
            imu_q = imu_node.out.createOutputQueue()

            # Optional recording queues (v3: create from node outputs, no XLinkOut)
            h265_q = None
            if self._rec_cfg is not None and self._rec_cfg.enabled:
                if self._rec_cfg.video_enabled:
                    encoder = pipeline.create(dai.node.VideoEncoder)
                    encoder.setDefaultProfilePreset(
                        30, dai.VideoEncoderProperties.Profile.H265_MAIN,
                    )
                    encoder.setBitrateKbps(self._rec_cfg.video_bitrate_kbps)
                    cam_rgb.requestOutput((1920, 1080), dai.ImgFrame.Type.NV12).link(encoder.input)
                    h265_q = encoder.bitstream.createOutputQueue(maxSize=60, blocking=False)

            rec_queues = {}
            if h265_q is not None:
                rec_queues["h265"] = h265_q
            with self._lock:
                self._recording_queues = rec_queues if rec_queues else None

            pipeline.start()
            logger.info("OAK-D pipeline started (depthai v3)")
            self._device_ready.set()

        except Exception:
            logger.exception("Failed to build/start OAK-D pipeline")
            return

        try:
            while not self._stop_event.is_set() and pipeline.isRunning():
                self._poll_depth(depth_q, np)
                self._poll_detections(det_q)
                self._poll_rgb(rgb_preview_q)
                self._poll_imu(imu_q)
                time.sleep(1.0 / self._obs_cfg.update_rate_hz)
        except Exception:
            logger.exception("OAK-D pipeline error")
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            self._device = None
            with self._lock:
                self._recording_queues = None
            self._device_ready.clear()

    def _poll_depth(self, depth_q, np) -> None:
        """Extract minimum distance and rich stats from the center ROI."""
        try:
            in_depth = depth_q.tryGet()
            if in_depth is None:
                return
            while True:
                newer = depth_q.tryGet()
                if newer is None:
                    break
                in_depth = newer
            frame = in_depth.getFrame()  # uint16, millimetres
            h, w = frame.shape
            rw = self._obs_cfg.roi_width_pct
            rh = self._obs_cfg.roi_height_pct
            x0 = int(w * (0.5 - rw / 2))
            x1 = int(w * (0.5 + rw / 2))
            y0 = int(h * (0.5 - rh / 2))
            y1 = int(h * (0.5 + rh / 2))
            roi = frame[y0:y1, x0:x1]
            total_pixels = roi.size
            valid = roi[roi > 0]
            if valid.size == 0:
                return
            # Use partition-based quantiles to avoid full sorting each poll.
            n = int(valid.size)
            k5 = max(0, int(0.05 * (n - 1)))
            k50 = n // 2
            q = np.partition(valid, (k5, k50))
            p5 = float(q[k5])
            p50 = float(q[k50])
            valid_pct = (valid.size / total_pixels) * 100.0 if total_pixels > 0 else 0.0
            now = time.monotonic()
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
                self._depth_state.raw_frame = frame
        except Exception:
            logger.debug("Depth poll error", exc_info=True)

    def _poll_detections(self, det_q) -> None:
        """Extract person detections with spatial coordinates."""
        try:
            in_det = det_q.tryGet()
            if in_det is None:
                return
            while True:
                newer = det_q.tryGet()
                if newer is None:
                    break
                in_det = newer
            persons: list[PersonDetection] = []
            for det in in_det.detections:
                is_person = (det.label == PERSON_LABEL or
                             getattr(det, "labelName", "") == "person")
                if not is_person:
                    continue
                if det.confidence < self._fm_cfg.detection_confidence:
                    continue
                persons.append(
                    PersonDetection(
                        x_m=det.spatialCoordinates.x / 1000.0,
                        z_m=det.spatialCoordinates.z / 1000.0,
                        confidence=det.confidence,
                        bbox=(det.xmin, det.ymin, det.xmax, det.ymax),
                    )
                )
            with self._lock:
                self._det_state.persons = persons
                self._det_state.timestamp = time.monotonic()
        except Exception:
            logger.debug("Detection poll error", exc_info=True)

    def _poll_rgb(self, rgb_q) -> None:
        """Grab the latest RGB preview frame from the NN passthrough."""
        try:
            msg = rgb_q.tryGet()
            if msg is None:
                return
            while True:
                newer = rgb_q.tryGet()
                if newer is None:
                    break
                msg = newer
            frame = msg.getCvFrame()
            with self._lock:
                self._rgb_state.frame = frame
                self._rgb_state.timestamp = time.monotonic()
        except Exception:
            logger.debug("RGB poll error", exc_info=True)

    def _poll_imu(self, imu_q) -> None:
        """Extract latest accelerometer and gyroscope data from the OAK-D IMU."""
        try:
            imu_data = imu_q.tryGet()
            if imu_data is None:
                return
            packets = imu_data.packets
            if not packets:
                return
            pkt = packets[-1]
            accel = pkt.acceleroMeter
            gyro = pkt.gyroscope
            now = time.monotonic()
            with self._lock:
                self._imu_state.ax_mss = accel.x
                self._imu_state.ay_mss = accel.y
                self._imu_state.az_mss = accel.z
                self._imu_state.gx_rads = gyro.x
                self._imu_state.gy_rads = gyro.y
                self._imu_state.gz_rads = gyro.z
                self._imu_state.timestamp = now
        except Exception:
            logger.debug("IMU poll error", exc_info=True)
