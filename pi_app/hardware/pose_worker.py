"""Host-side MediaPipe Pose worker (daemon thread, not the vision loop).

Pulls the latest 640x480 RGB frame the OakDepthReader already has and the
latest person detections, crops the largest person, runs Pose, and publishes
a PoseSample in full-frame normalised coordinates.

MediaPipe is lazy-imported inside the thread so unit tests can load this
module without mediapipe installed.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional, Sequence

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


def select_person_bbox(
    detections: Sequence[Any],
    min_confidence: float = _PERSON_MIN_CONF,
) -> Optional[tuple[float, float, float, float]]:
    """Largest-area person bbox with confidence >= min_confidence, or None."""
    best = None
    best_area = -1.0
    for det in detections or ():
        try:
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
        self._mp_warned = False
        self._last_processed_ts: Optional[float] = None
        self._timing: list[tuple[float, float]] = []  # (ts, pose_s)
        self._hz_events: list[float] = []

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
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        pose = self._pose
        self._pose = None
        if pose is not None:
            closer = getattr(pose, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception:
                    pass

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
            self._last_error = ""
            self._timing.clear()
            self._hz_events.clear()
            self._last_processed_ts = None

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

    def _ensure_mp_pose(self):
        if self._pose is not None:
            return self._pose
        if self._mp_failed:
            return None
        try:
            import mediapipe as _mp
            self._pose = _mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=0,
            )
            with self._lock:
                self._mp_pose_loaded = True
            logger.warning("Host-side MediaPipe Pose model loaded")
            return self._pose
        except Exception as e:
            self._mp_failed = True
            with self._lock:
                self._last_error = f"mediapipe_pose_unavailable: {e}"
                self._mp_pose_loaded = False
            if not self._mp_warned:
                logger.warning(
                    "MediaPipe Pose is not available; arms-up detection disabled"
                )
                self._mp_warned = True
            return None

    def _process_once(self) -> None:
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
        crop_box = person_crop_box(bbox, frame_w, frame_h)
        x0, y0, x1, y1 = crop_box
        if x1 - x0 < 2 or y1 - y0 < 2:
            self._note_hz()
            self._publish(None)
            return

        pose = self._ensure_mp_pose()
        if pose is None:
            self._publish(None)
            return

        import cv2 as _cv2
        crop = frame[y0:y1, x0:x1]
        rgb = _cv2.cvtColor(crop, _cv2.COLOR_BGR2RGB)
        t_inf = time.monotonic()
        results = pose.process(rgb)
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
        )
        self._publish(sample)

    def _publish(self, sample: Optional[PoseSample]) -> None:
        with self._lock:
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
