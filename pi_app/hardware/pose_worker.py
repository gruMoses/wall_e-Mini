"""Host-side MediaPipe Pose worker (daemon thread, not the vision loop).

Pulls the latest 640x480 RGB frame the OakDepthReader already has and the
latest person detections, crops the largest person, runs Pose, and publishes
a PoseSample in full-frame normalised coordinates.

MediaPipe is constructed once, at the start of the worker thread, and is
lazy-imported there so unit tests can load this module without mediapipe
installed. Each crop is a new image: static_image_mode, no cross-frame
tracking. If the model fails to load, the worker publishes nothing and
does not retry.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional, Sequence

from config import config
from pi_app.control.arms_up import PoseJoint, PoseSample

logger = logging.getLogger(__name__)

# MediaPipe Pose landmark indices.
_NOSE = 0
_L_SHOULDER, _R_SHOULDER = 11, 12
_L_ELBOW, _R_ELBOW = 13, 14
_L_WRIST, _R_WRIST = 15, 16

_PERSON_MIN_CONF = 0.5
_EXPAND_X = 0.15
_EXPAND_Y = 0.10
_TIMING_WINDOW_S = 2.0


def _is_person_detection(det: Any) -> bool:
    """True for an unlabelled detection, or one whose class says person.

    ``get_person_detections()`` already filtered. A label or class field,
    when the object has one, must still be a person (COCO class 0 or the
    name "person"). Other classes are ignored.
    """
    if hasattr(det, "label_name") or hasattr(det, "class_name"):
        name = getattr(det, "label_name", None)
        if name is None:
            name = getattr(det, "class_name", None)
        if isinstance(name, str):
            return name.strip().lower() == "person"
        return False
    for attr in ("class_id", "cls", "label"):
        if not hasattr(det, attr):
            continue
        val = getattr(det, attr)
        if isinstance(val, str):
            return val.strip().lower() == "person"
        try:
            return int(val) == 0
        except (TypeError, ValueError):
            return False
    return True


def select_person_bbox(
    detections: Sequence[Any],
    min_confidence: float = _PERSON_MIN_CONF,
) -> Optional[tuple[float, float, float, float]]:
    """Largest-area person bbox with confidence >= min_confidence, or None."""
    best = None
    best_area = -1.0
    for det in detections or ():
        try:
            if not _is_person_detection(det):
                continue
            conf = float(getattr(det, "confidence", 0.0))
            if conf < min_confidence:
                continue
            bbox = getattr(det, "bbox", None)
            if bbox is None or len(bbox) < 4:
                continue
            xmin, ymin, xmax, ymax = (float(bbox[0]), float(bbox[1]),
                                      float(bbox[2]), float(bbox[3]))
            area = max(0.0, xmax - xmin) * max(0.0, ymax - ymin)
            if area > best_area:
                best_area = area
                best = (xmin, ymin, xmax, ymax)
        except (TypeError, ValueError):
            continue
    return best


def person_crop_box(
    bbox: tuple[float, float, float, float],
    frame_w: int,
    frame_h: int,
    expand_x: float = _EXPAND_X,
    expand_y: float = _EXPAND_Y,
) -> tuple[int, int, int, int]:
    """Expand a normalised bbox and clamp to the frame as a pixel crop.

    ``expand_x`` / ``expand_y`` are fractional size increases (0.15 = 15 %
    wider). Returns ``(x0, y0, x1, y1)`` in pixel coordinates.
    """
    xmin, ymin, xmax, ymax = bbox
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    half_w = 0.5 * (xmax - xmin) * (1.0 + expand_x)
    half_h = 0.5 * (ymax - ymin) * (1.0 + expand_y)
    x0 = int(round((cx - half_w) * frame_w))
    y0 = int(round((cy - half_h) * frame_h))
    x1 = int(round((cx + half_w) * frame_w))
    y1 = int(round((cy + half_h) * frame_h))
    if frame_w < 1:
        frame_w = 1
    if frame_h < 1:
        frame_h = 1
    x0 = max(0, min(frame_w, x0))
    x1 = max(0, min(frame_w, x1))
    y0 = max(0, min(frame_h, y0))
    y1 = max(0, min(frame_h, y1))
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return x0, y0, x1, y1


def _intr_usable(intr: Any) -> bool:
    if intr is None:
        return False
    try:
        fx = float(intr[0])
        fy = float(intr[1])
    except (TypeError, ValueError, IndexError):
        return False
    return fx != 0.0 and fy != 0.0


def _fallback_bbox_to_frame(
    bbox: tuple[float, float, float, float],
    det_wh: tuple[int, int],
    frame_wh: tuple[int, int],
) -> tuple[float, float, float, float]:
    """Centred-crop ratio when intrinsics are unavailable.

    Both outputs keep the full sensor width and the NN frame is the
    vertical centre band of the preview:

        y_frame = 0.5 + (y_det - 0.5) * (det_h / det_w) / (frame_h / frame_w)
        x_frame = x_det
    """
    xmin, ymin, xmax, ymax = bbox
    det_w, det_h = det_wh
    frame_w, frame_h = frame_wh
    if det_w <= 0 or frame_w <= 0 or frame_h <= 0:
        return (xmin, ymin, xmax, ymax)
    scale = (float(det_h) / float(det_w)) / (float(frame_h) / float(frame_w))

    def map_y(y: float) -> float:
        return 0.5 + (y - 0.5) * scale

    return (xmin, map_y(ymin), xmax, map_y(ymax))


def det_bbox_to_frame(
    bbox: tuple[float, float, float, float],
    det_wh: tuple[int, int],
    det_intr: Any,
    frame_wh: tuple[int, int],
    frame_intr: Any,
) -> tuple[float, float, float, float]:
    """Map a normalised NN-frame bbox into the preview frame.

    Each edge: pixel in the NN frame, ray ``(u - c) / f``, pixel in the
    preview. ``det_intr`` and ``frame_intr`` are ``(fx, fy, cx, cy)`` in
    pixels. If either is missing, use the centred-crop ratio.
    """
    if not _intr_usable(det_intr) or not _intr_usable(frame_intr):
        return _fallback_bbox_to_frame(bbox, det_wh, frame_wh)
    xmin, ymin, xmax, ymax = bbox
    det_w, det_h = det_wh
    frame_w, frame_h = frame_wh
    if det_w <= 0 or det_h <= 0 or frame_w <= 0 or frame_h <= 0:
        return _fallback_bbox_to_frame(bbox, det_wh, frame_wh)
    dfx, dfy, dcx, dcy = (float(det_intr[0]), float(det_intr[1]),
                          float(det_intr[2]), float(det_intr[3]))
    ffx, ffy, fcx, fcy = (float(frame_intr[0]), float(frame_intr[1]),
                          float(frame_intr[2]), float(frame_intr[3]))

    def map_x(x_norm: float) -> float:
        u = x_norm * float(det_w)
        ray = (u - dcx) / dfx
        return (ray * ffx + fcx) / float(frame_w)

    def map_y(y_norm: float) -> float:
        v = y_norm * float(det_h)
        ray = (v - dcy) / dfy
        return (ray * ffy + fcy) / float(frame_h)

    return (map_x(xmin), map_y(ymin), map_x(xmax), map_y(ymax))


def expanded_person_crop(
    bbox: tuple[float, float, float, float],
    frame_w: int,
    frame_h: int,
    expand_x_frac: float = 0.20,
    expand_up_frac: float = 0.35,
    expand_down_frac: float = 0.05,
) -> tuple[int, int, int, int, float, float]:
    """Widen a frame-normalised box and clamp it to the frame.

    ``expand_x_frac`` is the total widening (split across both sides).
    ``expand_up_frac`` / ``expand_down_frac`` are fractions of the box
    height. Returns pixel ``(x0, y0, x1, y1)`` and the crop's top and
    bottom in full-frame normalised coordinates.
    """
    if frame_w < 1:
        frame_w = 1
    if frame_h < 1:
        frame_h = 1
    xmin, ymin, xmax, ymax = bbox
    bw = max(0.0, xmax - xmin)
    bh = max(0.0, ymax - ymin)
    x0n = xmin - 0.5 * float(expand_x_frac) * bw
    x1n = xmax + 0.5 * float(expand_x_frac) * bw
    y0n = ymin - float(expand_up_frac) * bh
    y1n = ymax + float(expand_down_frac) * bh
    x0n = min(1.0, max(0.0, x0n))
    x1n = min(1.0, max(0.0, x1n))
    y0n = min(1.0, max(0.0, y0n))
    y1n = min(1.0, max(0.0, y1n))
    x0 = int(round(x0n * frame_w))
    x1 = int(round(x1n * frame_w))
    y0 = int(round(y0n * frame_h))
    y1 = int(round(y1n * frame_h))
    x0 = max(0, min(frame_w, x0))
    x1 = max(0, min(frame_w, x1))
    y0 = max(0, min(frame_h, y0))
    y1 = max(0, min(frame_h, y1))
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    crop_y0 = y0 / float(frame_h)
    crop_y1 = y1 / float(frame_h)
    return x0, y0, x1, y1, crop_y0, crop_y1


def landmark_to_full_frame(
    lm_x: float,
    lm_y: float,
    crop_box: tuple[int, int, int, int],
    frame_w: int,
    frame_h: int,
) -> tuple[float, float]:
    """Map a crop-normalised landmark into full-frame normalised coordinates."""
    x0, y0, x1, y1 = crop_box
    crop_w = max(1, x1 - x0)
    crop_h = max(1, y1 - y0)
    px = x0 + lm_x * crop_w
    py = y0 + lm_y * crop_h
    fw = float(frame_w) if frame_w else 1.0
    fh = float(frame_h) if frame_h else 1.0
    return px / fw, py / fh


def _joint_from_lm(lms, idx: int, crop_box, frame_w: int, frame_h: int) -> PoseJoint:
    lm = lms[idx]
    x, y = landmark_to_full_frame(float(lm.x), float(lm.y), crop_box, frame_w, frame_h)
    vis = float(getattr(lm, "visibility", 1.0))
    return PoseJoint(x=x, y=y, visibility=vis)


class PoseWorker:
    """Daemon thread: MediaPipe Pose on the latest RGB person crop."""

    def __init__(self, oak_reader: Any, cfg: Any) -> None:
        self._oak = oak_reader
        self._cfg = cfg
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._enabled = False
        self._sample: Optional[PoseSample] = None
        self._pose_ms: Optional[float] = None
        self._pose_hz: float = 0.0
        self._mp_pose_loaded = False
        self._last_error = ""
        self._pose = None
        self._mp_failed = False
        self._last_processed_ts: Optional[float] = None
        self._timing: list[tuple[float, float]] = []  # (ts, pose_s)
        self._hz_events: list[float] = []
        self._intr_cache: dict[tuple[int, int], tuple[float, float, float, float]] = {}

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="PoseWorker", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.set_pose_enabled(False)
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                logger.warning(
                    "PoseWorker thread did not exit; leaving MediaPipe Pose open"
                )
                return
            self._thread = None
        self._close_pose()

    def _close_pose(self) -> None:
        pose = self._pose
        self._pose = None
        if pose is None:
            return
        closer = getattr(pose, "close", None)
        if callable(closer):
            try:
                closer()
            except Exception:
                logger.warning("MediaPipe Pose close() failed", exc_info=True)

    def set_pose_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        with self._lock:
            changed = enabled != self._enabled
            self._enabled = enabled
        setter = getattr(self._oak, "set_pose_rgb_wanted", None)
        if callable(setter):
            setter(enabled)
        if changed and not enabled:
            self._clear_output()

    def get_pose_sample(self) -> Optional[PoseSample]:
        with self._lock:
            return self._sample

    def get_status(self) -> dict:
        with self._lock:
            return {
                "pose_ms": self._pose_ms,
                "pose_hz": self._pose_hz,
                "pose_enabled": self._enabled,
                "mp_pose_loaded": self._mp_pose_loaded,
                "last_error": self._last_error,
            }

    def _clear_output(self) -> None:
        with self._lock:
            self._sample = None
            self._pose_ms = None
            self._pose_hz = 0.0
            self._timing.clear()
            self._hz_events.clear()
            self._last_processed_ts = None
            if not self._mp_failed:
                self._last_error = ""

    def _oak_stopped(self) -> bool:
        checker = getattr(self._oak, "is_stopped", None)
        if callable(checker):
            try:
                return bool(checker())
            except Exception:
                return False
        ev = getattr(self._oak, "_stop_event", None)
        is_set = getattr(ev, "is_set", None)
        return bool(is_set()) if callable(is_set) else False

    def _run(self) -> None:
        # Build once, before the enable loop. A failure publishes nothing
        # and is never retried (fail closed).
        self._load_pose_once()
        if self._pose is None:
            while not self._stop.is_set():
                if self._oak_stopped():
                    break
                self._stop.wait(0.05)
            return
        max_hz = float(getattr(self._cfg, "pose_max_hz", 10.0) or 10.0)
        period = 1.0 / max(0.1, max_hz)
        while not self._stop.is_set():
            if self._oak_stopped():
                break
            with self._lock:
                enabled = self._enabled
            if not enabled:
                self._stop.wait(0.05)
                continue
            t0 = time.monotonic()
            try:
                self._process_once()
            except Exception as e:
                with self._lock:
                    self._last_error = str(e)
            elapsed = time.monotonic() - t0
            self._stop.wait(max(0.0, period - elapsed))

    def _load_pose_once(self) -> None:
        if self._pose is not None or self._mp_failed:
            return
        try:
            import mediapipe as _mp
            self._pose = _mp.solutions.pose.Pose(
                static_image_mode=True,
                model_complexity=0,
            )
            with self._lock:
                self._mp_pose_loaded = True
            logger.warning("Host-side MediaPipe Pose model loaded")
        except Exception as e:
            self._mp_failed = True
            self._pose = None
            with self._lock:
                self._last_error = f"mediapipe_pose_unavailable: {e}"
                self._mp_pose_loaded = False
            logger.warning(
                "MediaPipe Pose is not available; arms-up detection disabled"
            )

    def _detection_wh(self) -> tuple[int, int]:
        """NN input size: the same values oak_depth passes to requestOutput."""
        det = getattr(config, "oak_detection", None)
        w = int(getattr(det, "input_width", 640) or 640)
        h = int(getattr(det, "input_height", 352) or 352)
        return max(1, w), max(1, h)

    def _intrinsics_for(self, w: int, h: int):
        key = (int(w), int(h))
        cached = self._intr_cache.get(key)
        if cached is not None:
            return cached
        getter = getattr(self._oak, "get_intrinsics", None)
        if not callable(getter):
            return None
        try:
            intr = getter(int(w), int(h))
        except Exception:
            return None
        if not _intr_usable(intr):
            return None
        packed = (float(intr[0]), float(intr[1]), float(intr[2]), float(intr[3]))
        self._intr_cache[key] = packed
        return packed

    def _process_once(self) -> None:
        if self._pose is None or self._stop.is_set():
            return
        with self._lock:
            if not self._enabled:
                return
        getter = getattr(self._oak, "get_latest_rgb_frame", None)
        if not callable(getter):
            return
        result = getter()
        if not isinstance(result, tuple) or len(result) < 2:
            return
        frame, ts = result[0], result[1]
        if frame is None:
            self._publish(None)
            return
        if ts == self._last_processed_ts:
            return
        self._last_processed_ts = ts

        if not self._detections_fresh(ts):
            self._note_hz()
            self._publish(None)
            return

        dets_fn = getattr(self._oak, "get_person_detections", None)
        dets = dets_fn() if callable(dets_fn) else []
        bbox = select_person_bbox(dets)
        if bbox is None:
            self._note_hz()
            self._publish(None)
            return

        try:
            frame_h, frame_w = int(frame.shape[0]), int(frame.shape[1])
        except Exception:
            self._publish(None)
            return
        det_w, det_h = self._detection_wh()
        mapped = det_bbox_to_frame(
            bbox,
            (det_w, det_h),
            self._intrinsics_for(det_w, det_h),
            (frame_w, frame_h),
            self._intrinsics_for(frame_w, frame_h),
        )
        expand_x = float(getattr(self._cfg, "crop_expand_x_frac", 0.20))
        expand_up = float(getattr(self._cfg, "crop_expand_up_frac", 0.35))
        expand_down = float(getattr(self._cfg, "crop_expand_down_frac", 0.05))
        x0, y0, x1, y1, crop_y0, crop_y1 = expanded_person_crop(
            mapped, frame_w, frame_h, expand_x, expand_up, expand_down,
        )
        if x1 - x0 < 2 or y1 - y0 < 2:
            self._note_hz()
            self._publish(None)
            return

        import cv2 as _cv2
        crop = frame[y0:y1, x0:x1]
        rgb = _cv2.cvtColor(crop, _cv2.COLOR_BGR2RGB)
        t_inf = time.monotonic()
        results = self._pose.process(rgb)
        inf_s = time.monotonic() - t_inf
        self._record_timing(inf_s)
        self._note_hz()

        lms = getattr(results, "pose_landmarks", None)
        if lms is None or getattr(lms, "landmark", None) is None:
            self._publish(None)
            return
        landmarks = lms.landmark
        if len(landmarks) <= _R_WRIST:
            self._publish(None)
            return
        crop_box = (x0, y0, x1, y1)
        sample = PoseSample(
            ts=float(ts),
            l_shoulder=_joint_from_lm(landmarks, _L_SHOULDER, crop_box, frame_w, frame_h),
            r_shoulder=_joint_from_lm(landmarks, _R_SHOULDER, crop_box, frame_w, frame_h),
            l_elbow=_joint_from_lm(landmarks, _L_ELBOW, crop_box, frame_w, frame_h),
            r_elbow=_joint_from_lm(landmarks, _R_ELBOW, crop_box, frame_w, frame_h),
            l_wrist=_joint_from_lm(landmarks, _L_WRIST, crop_box, frame_w, frame_h),
            r_wrist=_joint_from_lm(landmarks, _R_WRIST, crop_box, frame_w, frame_h),
            nose_y=landmark_to_full_frame(
                float(landmarks[_NOSE].x), float(landmarks[_NOSE].y),
                crop_box, frame_w, frame_h,
            )[1],
            crop_y0=crop_y0,
            crop_y1=crop_y1,
        )
        self._publish(sample)

    def _detections_fresh(self, frame_ts: Any) -> bool:
        """False when the person list is older than max_det_age_s vs the frame."""
        ts_fn = getattr(self._oak, "get_person_detections_ts", None)
        if not callable(ts_fn):
            return False
        try:
            det_ts = float(ts_fn())
            frame_t = float(frame_ts)
        except (TypeError, ValueError):
            return False
        max_age = float(getattr(self._cfg, "max_det_age_s", 0.3))
        return (frame_t - det_ts) <= max_age

    def _publish(self, sample: Optional[PoseSample]) -> None:
        if self._stop.is_set():
            return
        with self._lock:
            if not self._enabled or self._stop.is_set():
                return
            self._sample = sample
            if sample is not None:
                self._last_error = ""

    def _record_timing(self, pose_s: float) -> None:
        now = time.monotonic()
        cutoff = now - _TIMING_WINDOW_S
        with self._lock:
            self._timing.append((now, pose_s))
            self._timing = [p for p in self._timing if p[0] >= cutoff]
            if self._timing:
                mean_s = sum(p[1] for p in self._timing) / len(self._timing)
                self._pose_ms = mean_s * 1000.0
            else:
                self._pose_ms = None

    def _note_hz(self) -> None:
        now = time.monotonic()
        cutoff = now - _TIMING_WINDOW_S
        with self._lock:
            self._hz_events.append(now)
            self._hz_events = [t for t in self._hz_events if t >= cutoff]
            span = _TIMING_WINDOW_S
            self._pose_hz = (len(self._hz_events) / span) if self._hz_events else 0.0
