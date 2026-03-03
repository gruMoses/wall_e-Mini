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


class OakDepthReader:
    """Background OAK-D Lite reader following the ArduinoRCReader thread pattern."""

    def __init__(
        self,
        obstacle_config: ObstacleAvoidanceConfig,
        follow_me_config: FollowMeConfig,
        recording_config: OakRecordingConfig | None = None,
        imu_poll_hz: float = 60.0,
        imu_packet_mode: str = "latest",
        imu_max_packets_per_poll: int = 4,
    ) -> None:
        self._obs_cfg = obstacle_config
        self._fm_cfg = follow_me_config
        self._rec_cfg = recording_config

        self._depth_state = _DepthState()
        self._det_state = _DetectionState(persons=[])
        self._rgb_state = _RgbState()
        self._imu_state = _ImuState()
        self._imu_metrics = _ImuMetrics()
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
        self._imu_poll_interval_s = 1.0 / max(1.0, float(imu_poll_hz))
        mode = str(imu_packet_mode or "latest").strip().lower()
        self._imu_packet_mode = mode if mode in ("latest", "bounded") else "latest"
        self._imu_max_packets_per_poll = max(1, int(imu_max_packets_per_poll))

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

            # Device-side ROI spatial depth for obstacle distance + median depth.
            spatial_calc = pipeline.create(dai.node.SpatialLocationCalculator)
            rw = self._obs_cfg.roi_width_pct
            rh = self._obs_cfg.roi_height_pct
            roi_rect = dai.Rect(
                dai.Point2f(0.5 - rw / 2.0, 0.5 - rh / 2.0),
                dai.Point2f(0.5 + rw / 2.0, 0.5 + rh / 2.0),
            )

            spatial_min_cfg = dai.SpatialLocationCalculatorConfigData()
            spatial_min_cfg.depthThresholds.lowerThreshold = 100
            spatial_min_cfg.depthThresholds.upperThreshold = 10000
            spatial_min_cfg.roi = roi_rect
            spatial_min_cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
            spatial_calc.initialConfig.addROI(spatial_min_cfg)

            spatial_med_cfg = dai.SpatialLocationCalculatorConfigData()
            spatial_med_cfg.depthThresholds.lowerThreshold = 100
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

            model_desc = dai.NNModelDescription(model="mobilenet-ssd", platform="RVC2")
            spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
                cam_rgb, stereo, model_desc,
            )
            spatial_nn.setConfidenceThreshold(self._fm_cfg.detection_confidence)
            spatial_nn.input.setBlocking(False)
            spatial_nn.setBoundingBoxScaleFactor(0.5)
            spatial_nn.setDepthLowerThreshold(int(self._fm_cfg.min_distance_m * 1000))
            spatial_nn.setDepthUpperThreshold(int(self._fm_cfg.max_distance_m * 1000))

            det_q = None
            tracker_enabled = False
            try:
                object_tracker = pipeline.create(dai.node.ObjectTracker)
                object_tracker.setDetectionLabelsToTrack([PERSON_LABEL])
                object_tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
                object_tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

                # Feed detections + RGB frames into device-side tracker.
                spatial_nn.out.link(object_tracker.inputDetections)
                spatial_nn.passthrough.link(object_tracker.inputTrackerFrame)
                spatial_nn.passthrough.link(object_tracker.inputDetectionFrame)
                # Control loop consumes only newest detections; drop stale packets for low latency.
                det_q = object_tracker.out.createOutputQueue(maxSize=1, blocking=False)
                tracker_enabled = True
            except Exception:
                logger.warning(
                    "ObjectTracker init failed; falling back to raw detections",
                    exc_info=True,
                )
                # Same low-latency policy in fallback path when tracker is unavailable.
                det_q = spatial_nn.out.createOutputQueue(maxSize=1, blocking=False)

            # Obstacle avoidance wants freshest depth only; avoid host backlog.
            depth_q = spatial_nn.passthroughDepth.createOutputQueue(maxSize=1, blocking=False)
            # Spatial ROI depth is also control-critical, so prefer newest sample.
            spatial_depth_q = spatial_calc.out.createOutputQueue(maxSize=1, blocking=False)
            # Preview feed is operator/control UX, so keep latest-frame semantics.
            rgb_preview_q = cam_rgb.requestOutput((640, 480)).createOutputQueue(maxSize=1, blocking=False)

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
            self._device_ready.set()

        except Exception:
            logger.exception("Failed to build/start OAK-D pipeline")
            return

        try:
            next_imu_poll = time.monotonic()
            while not self._stop_event.is_set() and pipeline.isRunning():
                self._poll_depth(depth_q, spatial_depth_q, np)
                self._poll_detections(det_q)
                with self._lock:
                    rgb_enabled = self._rgb_poll_enabled
                if rgb_enabled:
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

    def _poll_depth(self, depth_q, spatial_depth_q, np) -> None:
        """Extract minimum distance and rich stats from the center ROI."""
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
            h, w = frame.shape
            rw = self._obs_cfg.roi_width_pct
            rh = self._obs_cfg.roi_height_pct
            x0 = int(w * (0.5 - rw / 2))
            x1 = int(w * (0.5 + rw / 2))
            y0 = int(h * (0.5 - rh / 2))
            y1 = int(h * (0.5 + rh / 2))
            roi = frame[y0:y1, x0:x1]
            p50 = None
            valid_pct = None

            p5 = None
            if in_spatial is not None:
                try:
                    spatial_locations = in_spatial.getSpatialLocations()
                    if spatial_locations:
                        min_z_mm = float(spatial_locations[0].spatialCoordinates.z)
                        if min_z_mm > 0:
                            p5 = min_z_mm
                    if len(spatial_locations) > 1:
                        med_z_mm = float(spatial_locations[1].spatialCoordinates.z)
                        if med_z_mm > 0:
                            p50 = med_z_mm
                except Exception:
                    p5 = None
            if p5 is None:
                # Fallback when no fresh spatial packet is available.
                nonzero = roi[roi > 0]
                if nonzero.size == 0:
                    return
                p5 = float(np.min(nonzero))

            self._depth_stats_counter += 1
            if self._depth_stats_counter >= self._depth_stats_decimation:
                self._depth_stats_counter = 0
                # Downsample for telemetry-only stats to keep host-side overhead low.
                roi_small = roi[::4, ::4]
                total_pixels = roi_small.size
                valid = roi_small[roi_small > 0]
                if valid.size > 0:
                    if p50 is None:
                        p50 = float(np.median(valid))
                    valid_pct = (valid.size / total_pixels) * 100.0 if total_pixels > 0 else 0.0

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
            tracklets = getattr(in_det, "tracklets", None)
            if tracklets is not None:
                for trk in tracklets:
                    src = getattr(trk, "srcImgDetection", None)
                    if src is None:
                        continue
                    is_person = (
                        getattr(src, "label", None) == PERSON_LABEL
                        or getattr(src, "labelName", "") == "person"
                    )
                    if not is_person:
                        continue
                    conf = float(getattr(src, "confidence", 0.0))
                    if conf < self._fm_cfg.detection_confidence:
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
                    persons.append(
                        PersonDetection(
                            x_m=float(getattr(spatial, "x", 0.0)) / 1000.0,
                            z_m=float(getattr(spatial, "z", 0.0)) / 1000.0,
                            confidence=conf,
                            bbox=(
                                float(getattr(src, "xmin", 0.0)),
                                float(getattr(src, "ymin", 0.0)),
                                float(getattr(src, "xmax", 0.0)),
                                float(getattr(src, "ymax", 0.0)),
                            ),
                            track_id=track_id,
                        )
                    )
            else:
                for det in getattr(in_det, "detections", []):
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
