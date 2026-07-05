"""
Autonomous person-following controller.

Architecture — five clean layers, each with a single responsibility:

  1. DetectionFilter  — raw PersonDetection list  → validated, normalised candidates
  2. TargetTracker    — candidates → single smoothed target (EMA, persistence, track-ID sticky)
  3. SteeringLayer    — normalized x-offset → lateral PID → steer_byte differential
  4. SpeedLayer       — depth error → proportional speed with dead zone
  5. SafetyLayer      — smooth acceleration, speed cap

  _mix_commands()     — (speed_offset, steer_offset) → (left_byte, right_byte)

  FollowMeController  — orchestrates all layers; also integrates the optional
                         trail-following Pure-Pursuit subsystem (unchanged logic,
                         plugs into the steering layer as an override).

Motor output is rate-limited to ~15 Hz (configurable), decoupled from the
30 fps vision pipeline.  Between output ticks the last command is held.
"""

from __future__ import annotations

import json
import logging
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT
from pi_app.control.drive_math import skid_steer_mix
from pi_app.control.gps_odometry import GpsOdometry, GpsOdometryConfig
from pi_app.control.odometry import DeadReckonOdometry
from pi_app.control.pure_pursuit import PurePursuitController, PursuitConfig
from pi_app.control.trail import TrailConfig, TrailManager

NEUTRAL = CENTER_OUTPUT_VALUE

_log = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Runtime-tunable Follow-Me parameters
# ─────────────────────────────────────────────────────────────────────────────
# The ONLY five FollowMeConfig fields that may be changed live (volatile, lost
# on restart) for steering auto-tuning. Each maps to an inclusive (min, max)
# hard bound — values outside the bound are rejected, never clamped. The web
# override endpoint and the standalone fm_autotune CLI both validate against
# this single source of truth. Note: speed_kp/ki/kd are deliberately absent —
# the velocity loop stays disabled (see FollowMeConfig).
TUNABLE_PARAM_BOUNDS: dict[str, tuple[float, float]] = {
    "pid_lateral_kp": (0.1, 0.8),
    "pid_lateral_kd": (0.0, 0.5),
    "target_ema_alpha": (0.2, 0.9),
    "steer_deadband_norm": (0.0, 0.1),
    "steer_slew_per_tick": (0.03, 0.4),
}


# ─────────────────────────────────────────────────────────────────────────────
# Public dataclass — API contract with oak_depth.py; do not rename fields.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class PersonDetection:
    """Single person detection from the OAK-D Lite spatial network."""
    x_m: float       # lateral offset from camera centre (+ = right)
    z_m: float       # forward distance in metres
    confidence: float
    bbox: tuple[float, float, float, float]  # xmin, ymin, xmax, ymax (normalised 0–1)
    track_id: int | None = None


# ─────────────────────────────────────────────────────────────────────────────
# Layer 1: Detection Filter
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class _FilteredDetection:
    """Validated, normalised detection ready for the tracker."""
    normalized_x: float    # horizontal offset: -1.0 (far left) … +1.0 (far right)
    x_m: float             # lateral offset in metres (for trail odometry)
    depth_m: float
    confidence: float
    bbox: tuple[float, float, float, float]
    track_id: int | None = None


class DetectionFilter:
    """Layer 1 — validates raw YOLO PersonDetections and normalises the output.

    Rejects detections that are:
    * below the confidence threshold
    * outside the valid depth range [min_depth_m, max_depth_m]
    * too small (bounding-box area below min_bbox_area as a frame fraction)
    """

    def __init__(
        self,
        conf_threshold: float,
        min_depth_m: float,
        max_depth_m: float,
        min_bbox_area: float,
        edge_margin: float = 0.0,
        min_bbox_width: float = 0.0,
        min_person_height_m: float = 0.0,
        camera_vfov_deg: float = 65.3,
    ) -> None:
        self._conf = conf_threshold
        self._min_depth = min_depth_m
        self._max_depth = max_depth_m
        self._min_area = min_bbox_area
        self._edge_margin = edge_margin
        self._min_bbox_width = min_bbox_width
        self._min_person_height_m = min_person_height_m
        self._camera_vfov_deg = camera_vfov_deg

    def process(self, raw: list[PersonDetection]) -> list[_FilteredDetection]:
        """Return validated and normalised detections."""
        out: list[_FilteredDetection] = []
        for det in raw:
            if det.confidence < self._conf:
                continue
            if not (self._min_depth <= det.z_m <= self._max_depth):
                continue
            area = (det.bbox[2] - det.bbox[0]) * (det.bbox[3] - det.bbox[1])
            if area < self._min_area:
                continue
            # ── Geometric rejection rules (each disabled when its threshold is ≤ 0) ──
            # Rule 1: Edge exclusion — reject sliver detections jammed against the frame edge.
            if self._edge_margin > 0:
                if det.bbox[0] > (1.0 - self._edge_margin) or det.bbox[2] < self._edge_margin:
                    continue
            # Rule 2: Minimum width — reject too-narrow boxes regardless of edge position.
            if self._min_bbox_width > 0:
                width = det.bbox[2] - det.bbox[0]
                if width < self._min_bbox_width:
                    continue
            # Rule 3: Minimum implied physical height — reject short ground blobs (animals).
            if self._min_person_height_m > 0:
                bbox_h = det.bbox[3] - det.bbox[1]
                implied_h_m = bbox_h * det.z_m * 2.0 * math.tan(
                    math.radians(self._camera_vfov_deg) / 2.0
                )
                if implied_h_m < self._min_person_height_m:
                    continue
            cx = (det.bbox[0] + det.bbox[2]) * 0.5
            normalized_x = (cx - 0.5) * 2.0  # map [0, 1] → [-1, +1]
            out.append(_FilteredDetection(
                normalized_x=normalized_x,
                x_m=det.x_m,
                depth_m=det.z_m,
                confidence=det.confidence,
                bbox=det.bbox,
                track_id=det.track_id,
            ))
        return out


# ─────────────────────────────────────────────────────────────────────────────
# Depth EMA filter — stabilises stereo depth at long range
# ─────────────────────────────────────────────────────────────────────────────

class DepthFilter:
    """Exponential-moving-average filter with physical-plausibility gating.

    Before updating the EMA, the implied velocity (|raw − filtered| / dt) is
    checked against *max_velocity_mps*.  Readings that exceed the threshold are
    rejected and the previous filtered value is returned unchanged.

    Anti-latch: rejections advance ``_last_time``, so after a genuine step
    change (e.g. a target switch with a depth gap) dt stays tiny and *every*
    subsequent frame would be rejected forever, regulating speed against the
    stale depth until the full loss timeout. To break that latch, a run of
    ``_MAX_CONSECUTIVE_REJECTS`` consecutive rejections is treated as evidence
    the world really moved: the raw value is accepted as a fresh seed. Any
    accepted sample (EMA or seed) resets the counter.
    """

    _MAX_CONSECUTIVE_REJECTS = 5

    def __init__(self, alpha: float = 0.35, max_velocity_mps: float = 5.0) -> None:
        self._alpha = alpha
        self._max_vel = max_velocity_mps
        self._filtered: float | None = None
        self._last_time: float | None = None
        self._reject_count: int = 0

    def update(self, raw_z: float, now: float) -> float:
        """Return filtered depth in metres."""
        if self._filtered is None:
            # First reading — seed the filter
            self._filtered = raw_z
            self._last_time = now
            self._reject_count = 0
            return self._filtered

        dt = now - self._last_time
        if dt <= 0.0:
            return self._filtered

        # Reject physically implausible jumps
        implied_vel = abs(raw_z - self._filtered) / dt
        if implied_vel > self._max_vel:
            self._reject_count += 1
            self._last_time = now
            if self._reject_count >= self._MAX_CONSECUTIVE_REJECTS:
                # Latch-breaker: a sustained "implausible" reading is real —
                # reseed the EMA to it rather than reject forever.
                self._filtered = raw_z
                self._reject_count = 0
                return self._filtered
            # Bad reading — keep previous estimate, but advance timestamp
            return self._filtered

        # Standard EMA update — an accepted sample clears the reject streak
        self._reject_count = 0
        self._filtered = self._alpha * raw_z + (1.0 - self._alpha) * self._filtered
        self._last_time = now
        return self._filtered

    def reset(self) -> None:
        """Clear state — call when follow-me is engaged/disengaged or the
        selected target switches (track_id change)."""
        self._filtered = None
        self._last_time = None
        self._reject_count = 0

    @property
    def value(self) -> float | None:
        return self._filtered


# ─────────────────────────────────────────────────────────────────────────────
# Layer 2: Target Tracker
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class _TargetState:
    """Current tracked target after EMA smoothing."""
    normalized_x: float    # EMA-smoothed horizontal offset
    x_m: float             # latest raw lateral offset in metres (for trail bias)
    depth_m: float         # latest raw depth
    confidence: float
    track_id: int | None
    last_seen_time: float  # time.monotonic() of most recent fresh detection


class TargetTracker:
    """Layer 2 — selects and smooths the follow target across frames.

    Selection strategy:
      * If the previously tracked ID is still visible, keep following it
        (track-ID sticky continuity).
      * Otherwise select the closest candidate by depth_m.

    EMA smoothing (alpha ~0.35) damps frame-to-frame jitter on the
    horizontal offset before it reaches the PID.

    Persistence: after the camera stops seeing the target, the last known
    state is held for up to ``persistence_s`` seconds before returning None.

    Sticky lock (defends against impostors that YOLO misclassifies as people):
      * SWITCH GRACE — a committed target momentarily absent from this frame's
        candidates is HELD (coast, not-fresh) for ``switch_grace_s`` rather than
        switching to a closer candidate.
      * ACQUIRE FLOOR — a NEW lock requires confidence >= ``acquire_confidence``
        (higher than the per-frame base filter). A committed target stays
        followed even if its confidence later drops below the floor.
      * SUSTAINED ACQUISITION — after a prior lock is lost, a challenger must
        qualify for ``acquire_min_frames`` consecutive frames before commit.
    """

    # Positional-continuity tolerance (normalized_x units, -1..+1) for matching a
    # None-id committed target to a candidate. ~0.30 ≈ 15% of frame width each
    # side: tight enough that a chicken offset from the person does not match.
    _NONE_ID_MATCH_NORM: float = 0.30

    def __init__(
        self,
        ema_alpha: float,
        persistence_s: float,
        switch_grace_s: float = 1.5,
        acquire_confidence: float = 0.65,
        acquire_min_frames: int = 3,
    ) -> None:
        self._alpha = ema_alpha
        self._persistence_s = persistence_s
        # Sticky-lock knobs (see FollowMeConfig). These only activate on dropouts
        # or new/competing candidates — a continuously-tracked target never trips
        # any of them, so default single-person behaviour is byte-identical.
        self._switch_grace_s = switch_grace_s
        self._acquire_confidence = acquire_confidence
        self._acquire_min_frames = max(1, int(acquire_min_frames))
        self._state: _TargetState | None = None
        self._fresh_raw_x_norm: float | None = None  # raw (pre-EMA) normalized_x of this tick's selected detection
        # Pending-challenger state for SUSTAINED ACQUISITION. A new candidate must
        # qualify for _acquire_min_frames consecutive frames before it is committed.
        # Keyed by track_id when present; for id=None we track positional continuity.
        self._pending_id: int | None = None
        self._pending_x: float | None = None       # last normalized_x of a None-id pending candidate
        self._pending_count: int = 0
        # Has the tracker ever committed a lock since the last reset()? The sustain
        # gate (and the grace hold, which presupposes a prior lock) only defend
        # AFTER an initial lock — i.e. on dropouts / re-acquisition / competing
        # impostors. The very first cold-start acquisition is immediate (subject
        # only to the confidence floor), so default single-person behaviour is
        # byte-identical: a continuously-tracked person never trips sustain/grace.
        self._has_committed: bool = False

    def update(
        self,
        candidates: list[_FilteredDetection],
        now: float,
    ) -> _TargetState | None:
        """Update tracker; return current state or None when target is lost.

        Sticky-lock state machine:
          * COMMITTED   — we hold a _state and it is present in this frame's
                          candidates (id match, or positional continuity for a
                          None-id target) → follow it normally (fresh).
          * GRACE-HOLD  — committed target momentarily ABSENT and NO other
                          candidate clears the acquire floor → HOLD it within
                          _switch_grace_s (coast, signal not-fresh); do NOT switch
                          to an untrusted impostor (the conf-0.54 chicken).
                          last_seen is NOT refreshed so grace + persistence age out.
          * HAND-OFF    — committed target absent but a *different* candidate
                          clears the acquire floor → adopt the closest such
                          qualifier immediately (a trusted replacement; this also
                          preserves the legitimate id-switch / depth reseed).
          * ACQUIRING   — no committed target (or committed lost beyond grace): a
                          challenger must clear _acquire_confidence and (after a
                          prior lock) sustain for _acquire_min_frames consecutive
                          frames before commit. Until then return None / coast. A
                          pure cold start acquires the closest qualifier at once,
                          so default single-person behaviour is unchanged.
        """
        if not candidates:
            # Zero-candidate path is unchanged: rely on the existing persistence
            # window. Grace is specifically the "candidates exist but not mine" case.
            self._fresh_raw_x_norm = None
            self._reset_pending()
            if self._state is None:
                return None
            if (now - self._state.last_seen_time) > self._persistence_s:
                return None
            # Return stale state; caller sees not-fresh and decays it.
            return self._state

        # ── Is our committed target present in this frame? ───────────────────
        # A target is "committed" once we hold any _state (id'd OR None-id). For
        # an id'd target, presence = exact track_id match. For a None-id target
        # (parse paths without tracklet ids) we use POSITIONAL continuity: the
        # candidate nearest in normalized_x to the held position, within
        # _NONE_ID_MATCH_NORM. This is what makes case (a) safe — a closer chicken
        # at a different x does NOT count as "my target", so we grace-hold the
        # person instead of switching. (Legacy None-id tracking just took the
        # closest by depth, which is exactly the case-(a) failure.)
        committed = self._state is not None
        committed_present = False
        committed_det: _FilteredDetection | None = None
        if committed and self._state.track_id is not None:
            for c in candidates:
                if c.track_id == self._state.track_id:
                    committed_present = True
                    committed_det = c
                    break
        elif committed:  # None-id committed target → positional continuity
            nearest = min(
                candidates,
                key=lambda d: abs(d.normalized_x - self._state.normalized_x),
            )
            if abs(nearest.normalized_x - self._state.normalized_x) <= self._NONE_ID_MATCH_NORM:
                committed_present = True
                committed_det = nearest

        if committed_present:
            # COMMITTED → follow it. An already-committed target is trusted even
            # if its confidence dropped below the acquire floor (Layer-1's 0.45
            # base filter already gated it). This is the normal single-person path.
            self._reset_pending()
            self._fresh_raw_x_norm = committed_det.normalized_x
            self._apply_ema(committed_det, now)
            return self._state

        if committed:
            # Committed target is ABSENT from this frame's candidates. Decide
            # between a GRACE-HOLD and a trusted hand-off.
            #
            # GRACE-HOLD (the case-(a) fix): if no other candidate clears the
            # ACQUIRE FLOOR, every alternative is an untrusted impostor (the
            # chicken at conf 0.54). HOLD the committed target — do NOT switch —
            # while within grace, signalling not-fresh so the caller decays
            # steer/speed. Do not refresh last_seen_time; let grace and the
            # persistence window age naturally.
            #
            # TRUSTED HAND-OFF: if a *different* candidate DOES clear the acquire
            # floor, it is a trustworthy target taking over (e.g. a distinct,
            # high-confidence id'd person). The committed target has vanished and
            # a trusted replacement is the only/closest qualifier, so we let the
            # ACQUIRING path adopt it rather than coast blindly. (This is also the
            # legitimate target-switch the depth-reseed path depends on — we never
            # hold against a target we actually trust.)
            within_grace = (now - self._state.last_seen_time) <= self._switch_grace_s
            qualifying = [
                c for c in candidates if c.confidence >= self._acquire_confidence
            ]
            if within_grace and not qualifying:
                self._fresh_raw_x_norm = None  # not fresh → caller decays steer/speed
                self._reset_pending()
                return self._state
            if within_grace and qualifying:
                # TRUSTED HAND-OFF within grace: the committed target vanished and
                # a floor-clearing replacement is present. Adopt the closest such
                # qualifier immediately (no sustain) — we only sustain-gate
                # acquisitions from a fully-lost state, not a direct hand-off to a
                # trusted target. Closest-by-depth among qualifiers, as elsewhere.
                best = min(qualifying, key=lambda d: d.depth_m)
                self._fresh_raw_x_norm = best.normalized_x
                self._apply_ema(best, now)
                self._has_committed = True
                self._reset_pending()
                return self._state
            # Grace expired → the committed target is truly lost. Drop the lock and
            # fall through to sustain-gated ACQUIRING (re-acquisition after loss).
            self._state = None

        # ── ACQUIRING ─────────────────────────────────────────────────────────
        # No committed target. Require a NEW target to clear the higher acquire
        # floor; if the tracker has locked before (re-acquisition after a loss),
        # also require it to sustain for _acquire_min_frames consecutive frames.
        acquired = self._acquire(candidates, now)
        if acquired is None:
            # Nothing qualifies (or still sustaining) → stay lost, coast.
            self._fresh_raw_x_norm = None
            return self._state  # None here (committed cleared above / never set)
        self._fresh_raw_x_norm = acquired.normalized_x
        self._apply_ema(acquired, now)
        self._has_committed = True
        self._reset_pending()
        return self._state

    def _acquire(
        self, candidates: list[_FilteredDetection], now: float
    ) -> _FilteredDetection | None:
        """ACQUISITION with a confidence floor + (post-loss) sustain.

        Among candidates clearing _acquire_confidence, pick the closest by depth.

        Cold start (never committed since reset): acquire that candidate
        immediately — default single-person behaviour is unchanged. The base
        detection_confidence=0.45 filter (Layer 1) plus this 0.65 floor are the
        only gates on the initial lock.

        Re-acquisition (we have committed before and lost the target): the chosen
        challenger must remain the closest qualifier for _acquire_min_frames
        consecutive frames before it is returned (committed). This filters
        flickering high-conf impostor blips (the chicken in case (b)).

        Returns the winning detection on the commit frame, else None (coast/lost).
        """
        qualifying = [c for c in candidates if c.confidence >= self._acquire_confidence]
        if not qualifying:
            # No candidate clears the floor — do not grab a low-conf chicken.
            self._reset_pending()
            return None

        best = min(qualifying, key=lambda d: d.depth_m)

        if not self._has_committed:
            # Cold start: immediate lock (no sustain), preserving legacy behaviour.
            return best

        # Continuity check: is `best` the same challenger we were counting?
        # Track by track_id when present; for id=None track positional continuity
        # (the closest qualifier must stay near where it was last frame).
        if best.track_id is not None:
            same = (best.track_id == self._pending_id)
        else:
            same = (
                self._pending_id is None
                and self._pending_x is not None
                and abs(best.normalized_x - self._pending_x) <= 0.25
            )

        if same:
            self._pending_count += 1
        else:
            # New challenger — start the sustain counter at 1.
            self._pending_id = best.track_id
            self._pending_count = 1
        self._pending_x = best.normalized_x

        if self._pending_count >= self._acquire_min_frames:
            return best
        return None

    def _reset_pending(self) -> None:
        self._pending_id = None
        self._pending_x = None
        self._pending_count = 0

    def _apply_ema(self, det: _FilteredDetection, now: float) -> None:
        if self._state is None:
            smoothed_x = det.normalized_x
        else:
            smoothed_x = (
                self._alpha * det.normalized_x
                + (1.0 - self._alpha) * self._state.normalized_x
            )
        self._state = _TargetState(
            normalized_x=smoothed_x,
            x_m=det.x_m,
            depth_m=det.depth_m,
            confidence=det.confidence,
            track_id=det.track_id,
            last_seen_time=now,
        )

    def reset(self) -> None:
        self._state = None
        self._fresh_raw_x_norm = None
        self._has_committed = False
        self._reset_pending()

    @property
    def last_seen_time(self) -> float:
        return self._state.last_seen_time if self._state is not None else 0.0

    @property
    def fresh_raw_x_norm(self) -> float | None:
        """Raw (pre-EMA) normalized_x of the selected detection this tick; None if no detection."""
        return self._fresh_raw_x_norm


# ─────────────────────────────────────────────────────────────────────────────
# Reusable PID controller
# ─────────────────────────────────────────────────────────────────────────────

class PIDController:
    """Generic PID with anti-windup integral clamping.

    All terms work in the caller's units.  ``dt`` is capped at 100 ms
    internally to prevent integrator blowup after stale / first-tick calls.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float = float("inf"),
        output_limit: float = float("inf"),
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral_limit = integral_limit
        self._output_limit = output_limit
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first_tick: bool = True

    def compute(self, error: float, dt: float) -> float:
        """Return PID output for the given error and elapsed time."""
        dt = max(1e-6, min(dt, 0.1))  # cap to avoid first-call blowup

        p = self.kp * error

        self._integral += error * dt
        self._integral = max(
            -self._integral_limit, min(self._integral_limit, self._integral)
        )
        i = self.ki * self._integral

        d = 0.0
        if not self._first_tick:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        self._first_tick = False

        output = p + i + d
        return max(-self._output_limit, min(self._output_limit, output))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_tick = True


# ─────────────────────────────────────────────────────────────────────────────
# Layer 3: Steering (PID lateral control)
# ─────────────────────────────────────────────────────────────────────────────

class SteeringLayer:
    """Layer 3 — lateral PID control.

    Input:  normalized horizontal offset of target (-1.0 = full left,
                                                    +1.0 = full right)
    Output: steering differential in motor-byte units (positive = turn right;
            left motor gets +steer, right motor gets -steer).

    The PID error IS the normalised offset: zero when the target is centred.
    The raw PID output (normalised) is then scaled to ±max_steer_byte.
    """

    def __init__(self, pid: PIDController, max_steer_byte: float) -> None:
        self._pid = pid
        self._max = max_steer_byte

    def compute(self, normalized_x: float, dt: float) -> float:
        """Compute steer offset in motor bytes from lateral error."""
        raw = self._pid.compute(normalized_x, dt)
        return max(-self._max, min(self._max, raw * self._max))

    def reset(self) -> None:
        self._pid.reset()


# ─────────────────────────────────────────────────────────────────────────────
# Layer 4: Speed (depth-based)
# ─────────────────────────────────────────────────────────────────────────────

class SpeedLayer:
    """Layer 4 — depth-based speed control, optionally closed-loop via velocity PID.

    When the robot is within ±dead_zone_m of the desired following distance,
    output is zero so the robot holds position without oscillating.
    Outside the dead zone, open-loop speed is proportional to the distance error.

    If a velocity_pid and speed_scale_mps_per_byte are supplied, and
    actual_speed_mps is passed to compute(), the layer closes the speed loop:
      velocity_error = target_speed_mps − actual_speed_mps
      correction_byte = PID(velocity_error) / speed_scale_mps_per_byte
    Falls back gracefully to open-loop when actual_speed_mps is None.
    """

    def __init__(
        self,
        target_dist_m: float,
        dead_zone_m: float,
        speed_gain: float,          # bytes per metre of distance error
        min_dist_m: float,
        max_speed_byte: float,
        velocity_pid: PIDController | None = None,
        speed_scale_mps_per_byte: float = 0.0075,
    ) -> None:
        self._target = target_dist_m
        self._dead_zone = dead_zone_m
        self._gain = speed_gain
        self._min_dist = min_dist_m
        self._max_speed = max_speed_byte
        self._velocity_pid = velocity_pid
        self._speed_scale = max(speed_scale_mps_per_byte, 1e-9)

    def compute(
        self,
        depth_m: float,
        actual_speed_mps: float | None = None,
        dt: float = 0.05,
    ) -> float:
        """Return forward speed offset in motor bytes (0 = hold/stop).

        When actual_speed_mps is provided and a velocity_pid is configured,
        a closed-loop correction is applied on top of the open-loop output.
        Otherwise (or when actual speed is None) behaves as pure open-loop.
        """
        if depth_m <= self._min_dist:
            if self._velocity_pid is not None:
                self._velocity_pid.reset()
            return 0.0
        error = depth_m - self._target
        if abs(error) <= self._dead_zone:
            if self._velocity_pid is not None:
                self._velocity_pid.reset()
            return 0.0
        if error <= 0.0:
            return 0.0  # too close — stop (backing up not implemented here)
        open_loop = min(self._max_speed, error * self._gain)

        # Closed-loop velocity correction when telemetry is available
        if self._velocity_pid is not None and actual_speed_mps is not None:
            target_speed_mps = open_loop * self._speed_scale
            velocity_error = target_speed_mps - actual_speed_mps
            correction_mps = self._velocity_pid.compute(velocity_error, dt)
            correction_byte = correction_mps / self._speed_scale
            return max(0.0, min(self._max_speed, open_loop + correction_byte))
        return open_loop


# ─────────────────────────────────────────────────────────────────────────────
# Layer 5: Safety
# ─────────────────────────────────────────────────────────────────────────────

class SafetyLayer:
    """Layer 5 — safety constraints on the combined motor command.

    Enforces:
    * Maximum speed cap
    * Smooth acceleration (slew-rate limiting on speed)

    The final slew limiter in controller.py provides an additional
    motor-level ramp; this layer guards within follow_me itself.
    """

    def __init__(self, max_speed_byte: float, max_accel_bps: float) -> None:
        self._max_speed = max_speed_byte
        self._max_accel = max_accel_bps
        self._prev_speed: float = 0.0
        self._prev_time: float = 0.0

    def apply(self, speed: float, steer: float, now: float) -> tuple[float, float]:
        """Return (safe_speed, safe_steer) with accel limiting and speed cap."""
        speed = min(speed, self._max_speed)
        if self._prev_time > 0.0:
            dt = now - self._prev_time
            if dt > 0.0:
                max_increase = self._max_accel * dt
                speed = min(speed, self._prev_speed + max_increase)
        speed = max(0.0, speed)
        self._prev_speed = speed
        self._prev_time = now
        return speed, steer

    def reset(self) -> None:
        self._prev_speed = 0.0
        self._prev_time = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Motor mixer
# ─────────────────────────────────────────────────────────────────────────────

def _mix_commands(speed_offset: float, steer_offset: float) -> tuple[int, int]:
    """Combine speed and steering differentials into left/right motor bytes."""
    return skid_steer_mix(
        speed_offset,
        steer_offset,
        neutral=NEUTRAL,
        min_output=MIN_OUTPUT,
        max_output=MAX_OUTPUT,
        deadband_byte=0,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Main controller — orchestrates all layers
# ─────────────────────────────────────────────────────────────────────────────

class FollowMeController:
    """Orchestrates the five follow-me layers.

    Public API (unchanged from prior version so controller.py needs no edits):
      update_pose(heading_deg, motor_l, motor_r, timestamp)
      update_gps(lat, lon, fix_quality, timestamp)
      compute(detections) -> (left_byte, right_byte)
      get_status()        -> dict
    """

    _MIN_LOST_TARGET_SPEED = 5  # minimum forward bytes during trail / search recovery

    def __init__(self, config: FollowMeConfig) -> None:
        self._cfg = config

        # Runtime overrides for config-backed tunables (FollowMeConfig is a
        # frozen dataclass and can't be mutated). Only the deadband/slew params
        # live here; kp/kd/alpha are applied directly to the sub-objects below.
        # See apply_tunable_params(). Volatile — lost on restart, by design.
        self._tunable_overrides: dict[str, float] = {}

        # ── Layer 1: Detection filter ────────────────────────────────────────
        self._filter = DetectionFilter(
            conf_threshold=config.detection_confidence,
            min_depth_m=config.min_distance_m,
            max_depth_m=config.max_distance_m,
            min_bbox_area=config.min_bbox_area,
            edge_margin=config.detect_edge_margin,
            min_bbox_width=config.detect_min_bbox_width,
            min_person_height_m=config.detect_min_person_height_m,
            camera_vfov_deg=config.detect_camera_vfov_deg,
        )

        # ── Depth EMA filter (between raw depth and speed/mode logic) ────
        self._depth_filter = DepthFilter(
            alpha=float(config.depth_ema_alpha),
            max_velocity_mps=float(config.depth_max_velocity_mps),
        )

        # ── Layer 2: Target tracker ──────────────────────────────────────────
        self._tracker = TargetTracker(
            ema_alpha=config.target_ema_alpha,
            persistence_s=config.target_persistence_s,
            switch_grace_s=config.target_switch_grace_s,
            acquire_confidence=config.target_acquire_confidence,
            acquire_min_frames=int(config.target_acquire_min_frames),
        )

        # ── Layer 3: Steering (PID) ──────────────────────────────────────────
        max_steer = float(config.max_steer_offset_byte)
        self._steering = SteeringLayer(
            pid=PIDController(
                kp=config.pid_lateral_kp,
                ki=config.pid_lateral_ki,
                kd=config.pid_lateral_kd,
                integral_limit=config.pid_lateral_integral_limit,
                output_limit=1.0,  # normalised; SteeringLayer scales to bytes
            ),
            max_steer_byte=max_steer,
        )

        # ── Layer 4: Speed (depth-based, closed-loop when telemetry available) ──
        max_speed = float(config.max_follow_speed_byte)
        max_speed_err = float(config.max_speed_error_m)
        _speed_scale = float(config.trail_speed_scale_mps_per_byte)
        _velocity_pid = PIDController(
            kp=float(config.speed_kp),
            ki=float(config.speed_ki),
            kd=float(config.speed_kd),
            integral_limit=float(config.speed_integral_limit),
        )
        self._speed = SpeedLayer(
            target_dist_m=config.follow_distance_m,
            dead_zone_m=float(config.speed_dead_zone_m),
            speed_gain=max_speed / max(max_speed_err, 0.1),
            min_dist_m=config.min_distance_m,
            max_speed_byte=max_speed,
            velocity_pid=_velocity_pid,
            speed_scale_mps_per_byte=_speed_scale,
        )

        # ── Layer 5: Safety ──────────────────────────────────────────────────
        self._safety = SafetyLayer(
            max_speed_byte=max_speed,
            max_accel_bps=float(config.max_speed_accel_byte_per_s),
        )

        # ── Motor output rate limiting ───────────────────────────────────────
        # Computation (PID, trail, steering) always runs at full call rate.
        # Only the emitted motor bytes are held at follow_output_rate_hz.
        output_hz = float(config.follow_output_rate_hz)
        self._output_interval_s = 1.0 / max(1.0, output_hz)
        self._last_output_time: float = 0.0
        # Separate from _last_output_time: tracks the previous compute() call so
        # PID dt reflects the vision-frame cadence, not the 15Hz output hold.
        self._last_compute_time: float = 0.0
        self._cached_left: int = NEUTRAL
        self._cached_right: int = NEUTRAL
        self._prev_target_present: bool = False  # for transition detection
        self._prev_fresh_detection: bool = False  # True only when persons visible this frame
        self._reacq_time: float = 0.0             # monotonic() of last lost→tracking transition
        self._last_fresh_steer: float = 0.0       # steer from last frame with fresh detection
        self._last_fresh_steer_time: float = 0.0  # monotonic() of that frame
        # Decay telemetry (Items 2 + 3 from Grok review)
        self._steer_decay_factor: float = 1.0     # 1.0=fresh, 0.0=fully decayed
        self._steer_hold_active: bool = False     # True while coasting on held steer
        self._last_fresh_detection: bool = False  # whether current frame had a fresh detection

        # ── Telemetry state ──────────────────────────────────────────────────
        self._tracking: bool = False
        self._last_target_z: float | None = None
        self._last_target_x: float | None = None      # x in metres (for trail bias)
        self._last_distance_error: float | None = None
        self._last_speed_offset: float = 0.0
        self._last_steer_offset: float = 0.0
        self._last_num_detections: int = 0
        self._last_target_confidence: float = 0.0
        self._last_target_track_id: int | None = None
        self._last_valid_time: float = 0.0
        self._pursuit_mode: str = "direct"

        # ── Steer slew cap (output-gate state) ──────────────────────────────
        self._last_emitted_steer: float = 0.0  # steer (bytes) at last 15Hz emission

        # ── Recorder state (per-tick FM trial JSONL) ─────────────────────────
        self._recorder_file = None   # open file handle, or None when not recording
        self._recorder_last_flush: float = 0.0
        self._recorder_engagement_ts: float | None = None
        self._last_x_err_norm: float | None = None       # post-deadband error for recorder
        self._last_slew_capped_steer: float = 0.0        # steer after slew cap for recorder
        # Arm state injected from controller.py each tick (or via set_arm_state).
        self._is_armed: bool = False

        # ── VESC telemetry (for closed-loop speed and slip detection) ────────
        self._actual_left_rpm: int | None = None
        self._actual_right_rpm: int | None = None
        self._actual_speed_mps: float | None = None
        self._actual_left_current_a: float | None = None
        self._actual_right_current_a: float | None = None
        self._actual_left_temp_c: float | None = None
        self._actual_right_temp_c: float | None = None
        # Charger inhibit state injected from controller.py each tick (2026-06-13
        # motor-cutout bug — surfaced here so the FM trial JSONL can show whether
        # a stop mid-chase was the charger inhibit, not a follow-me control bug.
        self._charger_inhibit: bool = False
        self._last_slip_active: bool = False
        # Consecutive-tick counter for the slip "going straight" guard. Evaluated
        # on the EMITTED/commanded steer; slip may only act once it persists.
        self._slip_straight_ticks: int = 0

        # ── Recorder "what the controller wanted" state (per-tick honesty) ────
        # steer the branch produced THIS tick BEFORE slip-comp and BEFORE the slew
        # cap, the branch label that produced it, whether slip modified this tick,
        # and whether this tick passed the 15 Hz output gate.
        self._last_steer_want: float = 0.0
        self._last_steer_src: str = "lost"
        self._last_emitted_flag: bool = False

        # ── Trail-following subsystems (Pure Pursuit — preserved) ────────────
        self._trail_enabled = bool(config.trail_follow_enabled)
        self._last_pursuit_mode: str = "direct"
        self._odometry: DeadReckonOdometry | None = None
        self._gps_odom: GpsOdometry | None = None
        self._trail: TrailManager | None = None
        self._pursuit: PurePursuitController | None = None
        self._pursuit_lookahead_x: float = 0.0
        self._pursuit_lookahead_y: float = 0.0
        self._trail_length: int = 0
        self._trail_distance_m: float = 0.0
        self._trail_rejected_jump_count: int = 0
        self._trail_rejected_speed_count: int = 0
        self._last_target_world_x: float | None = None
        self._last_target_world_y: float | None = None
        self._curvature_at_lookahead: float = 0.0
        self._speed_limited: bool = False
        self._odom_source: str = "none"
        self._last_mode_switch_time: float = 0.0
        # Trail exhaustion hysteresis (Fix 5) and one-shot extrapolation flag (Fix 4)
        self._trail_exhausted_count: int = 0
        self._trail_extrapolated: bool = False
        self._trail_extrapolation_count: int = 0
        self._trail_total_points_added: int = 0

        if self._trail_enabled:
            self._odometry = DeadReckonOdometry(
                speed_scale=config.trail_speed_scale_mps_per_byte
            )
            self._gps_odom = GpsOdometry(GpsOdometryConfig(
                cog_min_speed_mps=config.gps_cog_min_speed_mps,
                heading_alpha=config.gps_heading_alpha,
                cog_min_delta_m=config.gps_cog_min_delta_m,
            ))
            self._trail = TrailManager(TrailConfig(
                max_trail_points=config.trail_max_points,
                min_spacing_m=config.trail_min_spacing_m,
                max_age_s=config.trail_max_age_s,
                consume_radius_m=config.trail_consume_radius_m,
                max_step_m=config.trail_max_step_m,
                max_speed_mps=config.trail_max_speed_mps,
                smoothing_enabled=config.trail_smoothing_enabled,
                smoothing_window=config.trail_smoothing_window,
                smoothing_poly_order=config.trail_smoothing_poly_order,
            ))
            self._pursuit = PurePursuitController(PursuitConfig(
                lookahead_time_s=config.pursuit_lookahead_time_s,
                lookahead_min_m=config.pursuit_lookahead_min_m,
                lookahead_max_m=config.pursuit_lookahead_max_m,
                speed_scale_mps_per_byte=config.trail_speed_scale_mps_per_byte,
                wheelbase_m=config.pursuit_wheelbase_m,
                # Was a phantom fallback of 15.0 here vs 25.0 everywhere else
                # max_steer_offset_byte is read — now reads the single real
                # field directly so trail-pursuit steer shares the same cap
                # as every other consumer.
                max_steer_byte=float(config.max_steer_offset_byte),
                max_speed_byte=max_speed,
                curvature_scaling_enabled=config.pursuit_curvature_scaling_enabled,
                curvature_alpha=config.pursuit_curvature_alpha,
                min_speed_byte=float(config.pursuit_min_speed_byte),
                lookahead_curvature_points=config.pursuit_lookahead_curvature_points,
                max_accel_byte_per_s=float(config.pursuit_max_accel_byte_per_s),
            ))

    # ── Public API: odometry feeds ───────────────────────────────────────────

    def update_pose(
        self,
        heading_deg: float,
        motor_l: int,
        motor_r: int,
        timestamp: float,
    ) -> None:
        """Feed IMU heading and actual motor commands for dead-reckoning odometry.

        Called by controller.py every cycle before compute().
        """
        if self._odometry is not None:
            self._odometry.update(heading_deg, motor_l, motor_r, timestamp)
        if self._gps_odom is not None:
            self._gps_odom.update_gyro(heading_deg, timestamp)

    def update_gps(
        self,
        lat: float,
        lon: float,
        fix_quality: int,
        timestamp: float,
    ) -> None:
        """Feed GPS position for GPS-based trail odometry.

        Called by controller.py every cycle when a GPS reading is available.
        """
        if self._gps_odom is not None:
            self._gps_odom.update_gps(lat, lon, fix_quality, timestamp)

    def update_telemetry(
        self,
        left_rpm: int | None,
        right_rpm: int | None,
        actual_speed_mps: float | None,
        left_current_a: float | None = None,
        right_current_a: float | None = None,
        left_temp_c: float | None = None,
        right_temp_c: float | None = None,
        charger_inhibit: bool = False,
    ) -> None:
        """Feed VESC telemetry for closed-loop speed control and slip detection.

        Called by controller.py in FOLLOW_ME mode before each compute() call.
        Passing all-None gracefully disables closed-loop / slip features.
        The current/temp kwargs are optional for backward compatibility with
        existing callers and tests that do not supply them.

        ``charger_inhibit`` mirrors controller._charger_inhibit so the per-tick
        FM trial recorder can show it (2026-06-13 motor-cutout bug: a stop
        mid-chase looked like a follow-me bug until charger_inhibit was found).
        """
        self._actual_left_rpm = left_rpm
        self._actual_right_rpm = right_rpm
        self._actual_speed_mps = actual_speed_mps
        self._actual_left_current_a = left_current_a
        self._actual_right_current_a = right_current_a
        self._actual_left_temp_c = left_temp_c
        self._actual_right_temp_c = right_temp_c
        self._charger_inhibit = bool(charger_inhibit)

    # ── Public API: main compute ─────────────────────────────────────────────

    def compute(self, detections: list[PersonDetection]) -> tuple[int, int]:
        """Compute (left_byte, right_byte) to follow the best-scored person.

        Vision detections may arrive at up to 30 fps; all control computation
        (PID, trail breadcrumbs, steering mode selection) runs on every call.
        Only the emitted motor bytes are held at follow_output_rate_hz
        (default 15 Hz) so the robot doesn't jerk at the vision frame rate.
        """
        now = time.monotonic()
        self._last_num_detections = len(detections)

        # ── Layers 1 + 2: filter and track (full rate) ───────────────────────
        filtered = self._filter.process(detections)
        target = self._tracker.update(filtered, now)

        # Telemetry: always update from tracker result
        target_present = target is not None
        if target_present:
            # ── Target switch → reseed depth filter immediately ──────────────
            # track_id is real since 6ef3378. When the tracker locks onto a
            # different person the depth typically jumps; reseeding here stops
            # the new target's depth from being rejected against the old one's
            # filtered value (the rejection-latch failure mode).
            if (target.track_id is not None
                    and target.track_id != self._last_target_track_id):
                self._depth_filter.reset()
            # ── Apply depth EMA filter before any distance-based decisions ───
            filtered_depth = self._depth_filter.update(target.depth_m, now)
            target.depth_m = filtered_depth
            self._last_target_z = filtered_depth
            self._last_target_x = target.x_m    # metres, used by trail bias
            self._last_target_track_id = target.track_id
            self._last_target_confidence = target.confidence
            # Only mark "actively tracking" / refresh the valid-time when the
            # tracker COMMITTED to a fresh detection this frame. During a
            # grace-hold the held state is present (target_present) but no fresh
            # detection was selected (fresh_raw_x_norm is None), so we must not
            # extend the coasting window or seed the trail off a frame we did not
            # actually see the target. (Old code gated on `filtered`, which was
            # equivalent before sticky-lock since a non-empty filtered list always
            # produced a fresh selection.)
            if self._tracker.fresh_raw_x_norm is not None:
                self._tracking = True
                self._last_valid_time = now
        else:
            self._tracking = False

        # ── Trail breadcrumbs (full rate — accurate path needs every point) ───
        # Gate on a FRESH commit, not raw `filtered`: during a grace-hold candidates
        # exist (`filtered` non-empty) but the held target's coords are stale, so we
        # must not seed the trail off them. fresh_raw_x_norm is None on hold/coast.
        if (self._trail_enabled and self._trail is not None
                and self._tracker.fresh_raw_x_norm is not None and target_present):
            odom = self._pick_odometry()
            if odom is not None:
                wx, wy = odom.camera_to_world(target.x_m, target.depth_m)
                self._last_target_world_x = wx
                self._last_target_world_y = wy
                self._trail.add_point(wx, wy, now, speed_hint=self._last_speed_offset)
                self._trail_total_points_added += 1
                pose = odom.pose
                _prune_old = self._trail.length
                _prune_delta = self._trail.prune(pose.x, pose.y, pose.theta, now)
                if _prune_delta > 0 and self._pursuit is not None:
                    self._pursuit.adjust_for_prune(_prune_delta)
                    _log.debug(
                        "trail prune (active): -%d pts, idx adj -%d -> %d",
                        _prune_delta, _prune_delta, self._pursuit._last_closest_idx,
                    )
                self._trail_distance_m = self._trail.trail_distance()
                self._trail_rejected_jump_count = self._trail.rejected_jump_count
                self._trail_rejected_speed_count = self._trail.rejected_speed_count

        # ── Full control computation (full rate) ──────────────────────────────
        # dt for PID and safety is time since the last compute() CALL (the full
        # vision-frame cadence), NOT the last emitted output. The 15Hz output
        # hold keeps its own timing via _last_output_time further below; using
        # that here made PID derivative terms see alternating dt of ~0 and
        # ~66ms at 30fps (compute fires every frame, output only every other).
        dt = now - self._last_compute_time if self._last_compute_time > 0.0 else (
            1.0 / max(1.0, float(self._cfg.follow_output_rate_hz))
        )
        self._last_compute_time = now

        # True only when the tracker COMMITTED to a fresh detection this frame —
        # i.e. it followed the committed target or acquired a new one. During a
        # grace-hold (committed target absent, coasting) or an ACQUIRING frame
        # (challenger not yet sustained), the tracker sets fresh_raw_x_norm=None
        # so this is False and the persistence-decay path runs. Previously this
        # was bool(filtered), which the new sticky logic would mis-mark fresh when
        # candidates (e.g. a chicken) exist but the tracker did not select one.
        fresh_detection = self._tracker.fresh_raw_x_norm is not None

        if not target_present:
            self._prev_fresh_detection = False
            # _handle_lost_target returns pre-mix (speed, steer) — NOT motor
            # bytes. It also sets self._last_speed_offset / _last_steer_offset
            # as a side effect (mirroring the target-present branch below), so
            # the emission gate can apply the SAME steer slew cap + mixing to
            # every branch (tracking, persist, lost, search) uniformly.
            self._last_speed_offset, self._last_steer_offset = self._handle_lost_target(now)
            # Recorder honesty: what the lost-target handler wanted this tick,
            # and which branch produced it. _handle_lost_target sets
            # _pursuit_mode to "trail"/"search" while coasting, or resets to
            # "direct" on full timeout (commands neutral). Map to a recorder src.
            self._last_steer_want = self._last_steer_offset
            if self._tracking and self._pursuit_mode == "trail":
                self._last_steer_src = "trail"
            elif self._tracking and self._pursuit_mode == "search":
                self._last_steer_src = "search"
            else:
                self._last_steer_src = "lost"
        else:
            # Layer 4: Speed (closed-loop when VESC telemetry is available)
            speed = self._speed.compute(
                target.depth_m,
                actual_speed_mps=self._actual_speed_mps,
                dt=dt,
            )
            self._last_distance_error = target.depth_m - self._cfg.follow_distance_m

            # Layer 3: Steering — only when there is a FRESH detection this frame.
            # When the tracker is coasting on stale data (persistence window, persons=0)
            # hold the last fresh steer and decay it linearly to 0 over steer_hold_decay_s
            # so the robot continues turning during brief occlusions instead of driving straight.
            if not fresh_detection:
                elapsed_since_fresh = now - self._last_fresh_steer_time
                hold_decay_s = float(self._cfg.steer_hold_decay_s)
                # Speed-aware: at higher commanded speed decay faster (more dangerous to drive blind fast).
                # effective_decay = decay_s * max(0.3, 1.0 - speed_factor) where speed_factor ∈ [0, 1].
                max_speed = float(self._cfg.max_follow_speed_byte)
                speed_factor = speed / max_speed if max_speed > 0.0 else 0.0
                effective_decay_s = hold_decay_s * max(0.3, 1.0 - speed_factor)
                decay = max(0.0, 1.0 - elapsed_since_fresh / effective_decay_s) if effective_decay_s > 0.0 else 0.0
                # Clamp held steer to 70% of max during decay — prevents full-lock blind turns.
                max_s = float(self._cfg.max_steer_offset_byte)
                clamped_last_steer = max(-0.7 * max_s, min(0.7 * max_s, self._last_fresh_steer))
                steer = clamped_last_steer * decay
                # Decay SPEED on the same profile as steer. Previously speed kept
                # being computed from the frozen persistence depth while steer
                # decayed away — the robot drove blind at full speed but straight.
                # Apply the steer decay factor so forward motion ramps to 0 in
                # lock-step with the turn. (decay was computed from the pre-decay
                # speed, so the steer profile is unchanged.)
                speed = speed * decay
                self._steer_decay_factor = decay
                self._steer_hold_active = decay > 0.0
                self._last_fresh_detection = False
                # Recorder: this is a persistence-hold/decay tick — the controller
                # is coasting on the last fresh steer (decayed), not a fresh PID
                # result and not a true lost/search. Distinguish it explicitly.
                self._last_steer_want = steer
                self._last_steer_src = "persist"
            else:
                self._steer_decay_factor = 1.0
                self._steer_hold_active = False
                self._last_fresh_detection = True
                if not self._prev_fresh_detection:
                    self._reacq_time = now  # mark reacquisition start
                steer = self._compute_steering(target, speed, dt, now)
                # Recorder: fresh tick — record the branch's raw steer BEFORE the
                # reacq ramp, slip-comp, and slew cap, and which path produced it.
                # _compute_steering sets _pursuit_mode to "trail" or "direct".
                self._last_steer_want = steer
                self._last_steer_src = (
                    "trail" if self._pursuit_mode == "trail" else "direct"
                )
                # Ramp steer 0 → full over reacq_slew_window_s after a dropout to
                # prevent the spike on the first frames after reacquisition.
                # Skip the ramp on first-ever detection (no prior steer to spike from).
                reacq_window = max(
                    float(self._cfg.reacq_slew_window_s), 0.05
                )
                if self._reacq_time > 0.0 and self._last_fresh_steer_time > 0.0:
                    reacq_elapsed = now - self._reacq_time
                    if reacq_elapsed < reacq_window:
                        steer *= reacq_elapsed / reacq_window
                self._last_fresh_steer = steer
                self._last_fresh_steer_time = now

            self._prev_fresh_detection = fresh_detection

            # Slip detection & compensation (when VESC RPM telemetry available)
            speed, steer = self._apply_slip_compensation(speed, steer)

            # Layer 5: Safety
            speed, steer = self._safety.apply(speed, steer, now)

            self._last_speed_offset = speed
            self._last_steer_offset = steer
            # Mixing is deferred into the rate-limit gate (below) so the slew
            # cap can be applied uniformly to self._last_steer_offset — the
            # same code path the lost/search branch above now also feeds.

        # ── Rate-limit motor OUTPUT only ──────────────────────────────────────
        # Always emit immediately on first call or state transition (target
        # gained / lost). Otherwise hold the last command for the remainder of
        # the output interval so motors aren't updated at the full vision rate.
        dt_output = now - self._last_output_time
        state_changed = target_present != self._prev_target_present
        self._prev_target_present = target_present

        emitted_this_tick = (
            self._last_output_time <= 0.0
            or dt_output >= self._output_interval_s
            or state_changed
        )
        # Recorder honesty: True only on ticks that actually pass the 15 Hz output
        # gate (update motors). Held/duplicate vision ticks record emitted=False so
        # analysis can filter emitted==true to dedupe the ~32 Hz vision rate down to
        # the real 15 Hz command stream.
        self._last_emitted_flag = emitted_this_tick
        if emitted_this_tick:
            # Steer slew cap: limit change per output tick to avoid step-inputs
            # caused by PID spikes OR by a lost/search-mode mode transition.
            # Applied uniformly across EVERY branch (tracking, persist, lost,
            # search) so _last_emitted_steer is always a continuous, truthful
            # record of the last commanded steer — reacquisition after a lost/
            # search spell slews from that real value, never from a false 0.
            # Cap is (steer_slew_per_tick * max_byte) bytes.
            max_steer = float(self._cfg.max_steer_offset_byte)
            slew_limit = float(self._tunable("steer_slew_per_tick")) * max_steer
            slew_capped = max(
                self._last_emitted_steer - slew_limit,
                min(self._last_emitted_steer + slew_limit, self._last_steer_offset),
            )
            self._last_emitted_steer = slew_capped
            self._last_slew_capped_steer = slew_capped
            left, right = _mix_commands(self._last_speed_offset, slew_capped)
            self._cache_output((left, right), now)

        # ── Per-tick trial recorder ───────────────────────────────────────────
        self._write_trial_record(now, target)

        return self._cached_left, self._cached_right

    # ── Private helpers ──────────────────────────────────────────────────────

    def _cache_output(self, cmd: tuple[int, int], now: float) -> None:
        self._cached_left, self._cached_right = cmd
        self._last_output_time = now

    # ── Trial recorder ───────────────────────────────────────────────────────

    def _write_trial_record(self, now: float, target: _TargetState | None) -> None:
        """Append one JSON line per control tick to /tmp/fm_trials/<ts>.jsonl.

        All I/O is wrapped in try/except — never raises into the control loop.
        """
        try:
            if self._recorder_file is None:
                ts = int(time.time())
                self._recorder_engagement_ts = ts
                trial_dir = "/tmp/fm_trials"
                os.makedirs(trial_dir, exist_ok=True)
                path = os.path.join(trial_dir, f"{ts}.jsonl")
                self._recorder_file = open(path, "a", buffering=1)
                self._recorder_last_flush = now

            mode_raw = self._pursuit_mode if self._tracking else "lost"
            if mode_raw == "trail":
                mode_str = "pp"
            elif mode_raw == "search":
                mode_str = "search"
            elif mode_raw == "direct":
                mode_str = "direct"
            else:
                mode_str = "lost"

            left_byte, right_byte = _mix_commands(
                self._last_speed_offset, self._last_slew_capped_steer
            )
            record = {
                "t": now,
                "x_raw": self._tracker.fresh_raw_x_norm,
                "x_filt": round(target.normalized_x, 4) if target is not None else None,
                "x_err": round(self._last_x_err_norm, 4) if self._last_x_err_norm is not None else None,
                "steer": round(self._last_slew_capped_steer, 3),
                "speed": round(self._last_speed_offset, 3),
                "mode": mode_str,
                "track_id": target.track_id if target is not None else None,
                "depth": round(target.depth_m, 3) if target is not None else None,
                "conf": round(target.confidence, 3) if target is not None else None,
                "is_armed": self._is_armed,
                "left_byte": left_byte,
                "right_byte": right_byte,
                # ── Recorder honesty additions (do not rename existing keys) ────
                # What the controller WANTED this tick: the branch's raw steer
                # before slip-comp and before the slew cap.
                "steer_want": round(self._last_steer_want, 3),
                # Which branch actually produced steer_want this tick.
                # "direct"|"trail"|"search"|"persist"|"lost" — distinguishes a
                # persistence hold/decay from a true lost/search (the old "mode"
                # collapsed both). "mode" keeps its legacy values for fm_score.py.
                "steer_src": self._last_steer_src,
                # Did slip comp actually modify speed/steer this tick.
                "slip_active": bool(self._last_slip_active),
                # Did THIS tick pass the 15 Hz output gate (update motors) vs a
                # held/duplicate vision tick. Filter emitted==true to dedupe.
                "emitted": bool(self._last_emitted_flag),
                # Actual VESC eRPM per side (None when telemetry unavailable).
                # Without these, wheel-slip can't be confirmed from a recording —
                # the 2026-06-13 slip runaway took a live RPM bench to diagnose.
                "rpm_l": self._actual_left_rpm,
                "rpm_r": self._actual_right_rpm,
                # Motor phase current (A) per side — distinguishes over-current
                # cutout from thermal shutdown when reviewing a post-mortem log.
                "cur_l": round(self._actual_left_current_a, 2) if self._actual_left_current_a is not None else None,
                "cur_r": round(self._actual_right_current_a, 2) if self._actual_right_current_a is not None else None,
                # MOSFET (FET) temperature (°C) per side. Motor temp is parsed by
                # the VESC driver (STATUS_4) but not surfaced on VescTelemetry;
                # duty_cycle is also parsed (STATUS) but not on VescTelemetry —
                # both require a vesc.py change to expose (out of scope here).
                "tfet_l": round(self._actual_left_temp_c, 1) if self._actual_left_temp_c is not None else None,
                "tfet_r": round(self._actual_right_temp_c, 1) if self._actual_right_temp_c is not None else None,
                # BMS charger inhibit state this tick — a stop mid-chase can now be
                # attributed to this instead of mistaken for a follow-me control bug
                # (2026-06-13 field incident: debounced in bms.is_charging(), see there).
                "charger_inhibit": self._charger_inhibit,
            }
            self._recorder_file.write(json.dumps(record) + "\n")

            if now - self._recorder_last_flush >= 1.0:
                self._recorder_file.flush()
                self._recorder_last_flush = now
        except Exception:
            pass  # recorder must never interrupt the control loop

    def stop_recorder(self) -> None:
        """Close the trial recorder file if open. Call on FM disengage."""
        try:
            if self._recorder_file is not None:
                self._recorder_file.flush()
                self._recorder_file.close()
                self._recorder_file = None
        except Exception:
            pass

    def start_recorder(self) -> None:
        """Close any open recorder file and arm a fresh one for the next session.

        Call on FOLLOW_ME entry so that each engagement gets its own timestamped
        JSONL file.  The file itself is created lazily on the first compute()
        tick (same as before), so this is lightweight and safe to call from the
        controller's mode-transition site.
        """
        self.stop_recorder()  # ensure previous session's file is flushed + closed

    def set_arm_state(self, is_armed: bool) -> None:
        """Feed the controller arm state so it can be included in recorder rows.

        Called by controller.py each tick before compute() in FOLLOW_ME mode.
        """
        self._is_armed = bool(is_armed)

    def _pick_odometry(self):
        """Return the best available odometry source (GPS preferred)."""
        if self._gps_odom is not None and self._gps_odom.has_fix:
            self._odom_source = "gps"
            return self._gps_odom
        if self._odometry is not None:
            self._odom_source = "dead_reckon"
            return self._odometry
        self._odom_source = "none"
        return None

    def _compute_steering(
        self,
        target: _TargetState,
        speed_offset: float,
        dt: float,
        now: float,
    ) -> float:
        """Choose between trail Pure-Pursuit and direct PID for steering.

        Trail pursuit is used when conditions are met (trail enabled, enough
        breadcrumbs, target is far / turning).  Falls back to Layer 3 PID
        direct pursuit otherwise.
        """
        self._last_x_err_norm = None  # trail path never populates this; direct path overwrites below
        odom = self._pick_odometry()
        mode_dwell_s = float(self._cfg.mode_switch_dwell_s)

        if (
            self._trail_enabled
            and self._pursuit is not None
            and odom is not None
            and self._trail is not None
        ):
            trail_pts = self._trail.get_smoothed_trail()
            self._trail_length = len(trail_pts)
            min_pts = int(self._cfg.min_trail_points_for_pursuit)
            direct_dist = float(self._cfg.direct_pursuit_distance_m)
            direct_lat = float(self._cfg.direct_pursuit_lateral_m)

            time_in_mode = now - self._last_mode_switch_time
            if self._last_pursuit_mode == "trail":
                use_direct = (
                    target.depth_m < direct_dist
                    and abs(target.normalized_x) < direct_lat
                    and time_in_mode >= mode_dwell_s
                )
            else:
                use_direct = not (
                    (target.depth_m > direct_dist * 1.3
                     or abs(target.normalized_x) > direct_lat * 1.5)
                    and time_in_mode >= mode_dwell_s
                )

            if len(trail_pts) >= min_pts and not use_direct:
                pose = odom.pose
                smoothed = self._trail.get_smoothed_trail()
                curvatures = TrailManager.compute_curvatures(smoothed)
                cmd = self._pursuit.compute(
                    pose, smoothed, speed_offset,
                    curvatures=curvatures, now=now,
                )
                if cmd is not None:
                    if self._last_pursuit_mode != "trail":
                        self._last_mode_switch_time = now
                        self._steering.reset()  # clear PID integrator on mode switch
                    self._pursuit_mode = "trail"
                    self._last_pursuit_mode = "trail"
                    self._pursuit_lookahead_x = cmd.lookahead_x
                    self._pursuit_lookahead_y = cmd.lookahead_y
                    self._curvature_at_lookahead = cmd.curvature_at_lookahead
                    self._speed_limited = cmd.speed_limited
                    steer = cmd.steer_byte

                    # Person-position bias: blend live detection position into
                    # trail pursuit steering so the robot doesn't ignore where
                    # the person actually is when they drift laterally.
                    bias_w = float(self._cfg.trail_person_bias_weight)
                    if bias_w > 0.0:
                        direct_steer = self._steering.compute(target.normalized_x, dt)
                        steer = steer * (1.0 - bias_w) + direct_steer * bias_w
                        clamp_s = float(self._cfg.max_steer_offset_byte)
                        steer = max(-clamp_s, min(clamp_s, steer))
                    return steer

        # Direct PID pursuit fallback
        if self._last_pursuit_mode != "direct":
            self._last_mode_switch_time = now
            self._steering.reset()
        self._pursuit_mode = "direct"
        self._last_pursuit_mode = "direct"
        # Bbox-x deadband: suppress PID input while gait wobbles the centroid ±~2-3% frame.
        x_err = target.normalized_x
        deadband = float(self._tunable("steer_deadband_norm"))
        if abs(x_err) < deadband:
            x_err = 0.0
        # Record the post-deadband, PRE-boost error so the recorder's x_err
        # field keeps its original meaning (raw lateral error, not amplified).
        self._last_x_err_norm = x_err
        # Edge-boost: amplify the PID input proportionally as the person drifts
        # toward the frame edge (|x_err| > knee).  Center (|x_err| <= knee) is
        # byte-identical to the un-boosted path (edge_gain is exactly 1.0 there).
        # boost=0 disables the feature entirely.
        knee = float(self._cfg.steer_edge_knee)
        boost = float(self._cfg.steer_edge_boost)
        ax = abs(x_err)
        if boost > 0.0 and knee < 1.0 and ax > knee:
            edge_gain = 1.0 + boost * min(1.0, (ax - knee) / (1.0 - knee))
            x_err = x_err * edge_gain
        steer = self._steering.compute(x_err, dt)
        # Cap direct-mode steering lower than the global max; close-range detections
        # are noisier and a full-gain correction causes overshoot.  Also scale back
        # proportionally when confidence is below 0.6 to prevent spikes on uncertain
        # detections (common when the person nearly fills the frame).
        direct_max = float(self._cfg.direct_mode_max_steer_byte)
        steer = max(-direct_max, min(direct_max, steer))
        if target.confidence < 0.6:
            steer *= target.confidence / 0.6
        return steer

    def _apply_slip_compensation(
        self,
        speed: float,
        steer: float,
    ) -> tuple[float, float]:
        """Detect wheel slip from a COMMANDED-vs-ACTUAL rpm differential and,
        only when genuinely slipping while commanded straight, reduce throttle
        and inject a small, hard-bounded steer feed-forward.

        Hard off-switch: when ``slip_compensation_enabled`` is False (the shipped
        default) this is a TRUE no-op — it returns (speed, steer) unchanged with
        no throttle reduction and no steer term. Behavior is then byte-identical
        to running with no slip stage at all.

        Anti-feedback design (why this can't run away on a skid-steer robot):
          * Detection is on ``slip_diff = actual_rpm_diff − expected_diff`` where
            ``expected_diff = slip_cmd_diff_per_byte * commanded_steer``. Because a
            commanded turn raises ``expected_diff`` in lockstep with the actual
            differential it produces, a deliberate turn nets to ~0 slip_diff and is
            NOT read as slip. The old code acted on the raw ``actual_rpm_diff`` and
            so treated every turn as slip → positive feedback.
          * The "going straight" guard is evaluated on the EMITTED/commanded steer
            (``_last_emitted_steer``), not the pre-correction PID value, and must
            persist for ``slip_straight_persist_ticks`` consecutive ticks. So a
            transient never triggers, and the guard disengages the instant real
            turning begins.
          * Any steer feed-forward is clamped to ``slip_max_steer_byte`` (a small
            dedicated cap), and the TOTAL post-slip steer is re-clamped to the
            direct cap ``direct_mode_max_steer_byte`` — slip can never push the
            emitted steer past the direct cap, let alone to the global ±max.
          * No term is a function that increases the differential it reacts to: the
            feed-forward is driven by slip_diff (which a turn cancels), not by the
            raw differential.

        No-op when RPM telemetry is unavailable.
        """
        # 1. Hard off-switch — true no-op, not even throttle reduction.
        if not bool(self._cfg.slip_compensation_enabled):
            self._last_slip_active = False
            self._slip_straight_ticks = 0
            return speed, steer

        left_rpm = self._actual_left_rpm
        right_rpm = self._actual_right_rpm
        if left_rpm is None or right_rpm is None:
            self._last_slip_active = False
            self._slip_straight_ticks = 0
            return speed, steer

        # 2. Persistent "going straight" guard on the EMITTED/commanded steer.
        #    _last_emitted_steer is the steer actually sent at the last 15 Hz
        #    output gate — the true command, not this tick's raw PID value.
        commanded_steer = self._last_emitted_steer
        is_straight = abs(commanded_steer) < 5.0 and speed > 0.0
        if is_straight:
            self._slip_straight_ticks += 1
        else:
            self._slip_straight_ticks = 0
        persist_n = int(self._cfg.slip_straight_persist_ticks)
        straight_persisted = self._slip_straight_ticks >= persist_n

        # 3. Commanded-vs-actual slip estimate.
        actual_rpm_diff = float(left_rpm - right_rpm)
        k_cmd = float(self._cfg.slip_cmd_diff_per_byte)
        expected_diff = k_cmd * commanded_steer          # rough proportional model
        slip_diff = actual_rpm_diff - expected_diff      # residual after cancelling command

        threshold = float(self._cfg.slip_threshold_rpm)

        if abs(slip_diff) > threshold and straight_persisted:
            # 4. Throttle reduction on detected real slip.
            reduction = float(self._cfg.slip_throttle_reduction)
            speed = speed * (1.0 - reduction)

            # 5. Bounded steer feed-forward. Positive slip_diff (left spinning
            #    faster than commanded) → robot drifting right → small right-turn
            #    (positive steer) correction. Driven by slip_diff (a turn cancels
            #    it), never by the raw differential, so it cannot self-amplify.
            ff_gain = float(self._cfg.slip_feedforward_gain)
            slip_max = float(self._cfg.slip_max_steer_byte)
            steer_correction = slip_diff * ff_gain
            steer_correction = max(-slip_max, min(slip_max, steer_correction))
            steer = steer + steer_correction

            # 6. TOTAL post-slip steer clamped to the direct cap — slip can never
            #    push the emitted steer past it (and thus never near the global ±max).
            direct_max = float(self._cfg.direct_mode_max_steer_byte)
            steer = max(-direct_max, min(direct_max, steer))
            self._last_slip_active = True
        else:
            self._last_slip_active = False
        return speed, steer

    def _handle_lost_target(self, now: float) -> tuple[float, float]:
        """Handle absence of detections: trail pursuit → search → stop.

        Returns (speed_offset, steer_offset) — pre-mix, pre-slew-cap — NOT
        motor bytes. Mixing and the steer slew cap are applied uniformly for
        every branch (tracking, persist, lost, search) by the emission gate in
        compute(), so a lost/search steer request can never step-jump past the
        same per-tick cap that governs tracking-mode steering.
        """
        elapsed = now - self._last_valid_time

        if self._last_valid_time > 0.0:
            trail_max_s = float(self._cfg.lost_target_trail_pursuit_max_s)
            odom = self._pick_odometry()

            # ── Blind trail pursuit ──────────────────────────────────────────
            if (
                self._trail_enabled
                and self._pursuit is not None
                and odom is not None
                and self._trail is not None
                and elapsed < trail_max_s
            ):
                # Fix 4: extrapolate trail once on first lost-target call
                if not self._trail_extrapolated:
                    n_added = self._trail.extrapolate_trail(now)
                    if n_added > 0:
                        _log.debug(
                            "trail extrapolated +%d pts on target loss (elapsed=%.2fs)",
                            n_added, elapsed,
                        )
                    self._trail_extrapolation_count = n_added
                    self._trail_extrapolated = True

                trail_pts = self._trail.get_smoothed_trail()
                self._trail_length = len(trail_pts)
                self._trail_distance_m = self._trail.trail_distance()
                self._trail_rejected_jump_count = self._trail.rejected_jump_count
                self._trail_rejected_speed_count = self._trail.rejected_speed_count
                min_pts = int(self._cfg.min_trail_points_for_pursuit)

                if len(trail_pts) >= min_pts:
                    pose = odom.pose

                    # Fix 2: prune trail + adjust index before compute
                    _lt_prune_delta = self._trail.prune(pose.x, pose.y, pose.theta, now)
                    if _lt_prune_delta > 0:
                        self._pursuit.adjust_for_prune(_lt_prune_delta)
                        _log.debug(
                            "trail prune (lost): -%d pts, idx -> %d, rejected j=%d s=%d",
                            _lt_prune_delta, self._pursuit._last_closest_idx,
                            self._trail_rejected_jump_count,
                            self._trail_rejected_speed_count,
                        )
                    # Re-fetch smoothed trail after prune
                    trail_pts = self._trail.get_smoothed_trail()
                    self._trail_length = len(trail_pts)

                if len(trail_pts) >= min_pts:
                    pose = odom.pose
                    decay_frac = max(0.0, 1.0 - elapsed / trail_max_s)
                    fwd = max(
                        self._last_speed_offset * decay_frac,
                        self._MIN_LOST_TARGET_SPEED,
                    )
                    curvatures = TrailManager.compute_curvatures(trail_pts)
                    cmd = self._pursuit.compute(
                        pose, trail_pts, fwd,
                        curvatures=curvatures, now=now,
                    )
                    search_delay_s = float(self._cfg.search_mode_delay_s)
                    trail_exhausted_threshold = int(self._cfg.trail_exhausted_remaining)

                    last_pt = trail_pts[-1]
                    near_trail_end = math.hypot(
                        last_pt.x - pose.x, last_pt.y - pose.y
                    ) < 3.0

                    if cmd is not None:
                        dx_lk = cmd.lookahead_x - pose.x
                        dy_lk = cmd.lookahead_y - pose.y
                        fwd_x = math.cos(pose.theta)
                        fwd_y = math.sin(pose.theta)
                        lookahead_behind = (dx_lk * fwd_x + dy_lk * fwd_y) < 0
                        if cmd.trail_remaining <= 5:
                            _log.debug(
                                "trail_remaining=%d (pose=(%.2f,%.2f))",
                                cmd.trail_remaining, pose.x, pose.y,
                            )
                    else:
                        lookahead_behind = False

                    # Fix 5: hysteresis — require 3 consecutive frames before exhausted
                    _raw_exhausted = (
                        near_trail_end
                        or lookahead_behind
                        or (cmd is not None
                            and cmd.trail_remaining <= trail_exhausted_threshold)
                    )
                    if _raw_exhausted:
                        self._trail_exhausted_count += 1
                    else:
                        self._trail_exhausted_count = 0
                    trail_exhausted = self._trail_exhausted_count >= 3

                    trail_ok = cmd is not None and (
                        elapsed < search_delay_s or not trail_exhausted
                    )

                    if trail_ok:
                        self._pursuit_mode = "trail"
                        self._pursuit_lookahead_x = cmd.lookahead_x
                        self._pursuit_lookahead_y = cmd.lookahead_y
                        steer = cmd.steer_byte

                        # Ramp in a bias toward the last known person direction
                        # to anticipate the turn they were making.
                        last_x = self._last_target_x or 0.0
                        if abs(last_x) > 0.3:
                            bias_gain = float(self._cfg.lost_steer_bias_gain)
                            time_factor = min(elapsed / max(search_delay_s, 0.1), 1.5)
                            steer_bias = last_x * bias_gain * time_factor
                            max_s = float(self._cfg.max_steer_offset_byte)
                            steer = max(-max_s, min(max_s, steer + steer_bias))

                        self._last_steer_offset = steer
                        self._last_speed_offset = fwd
                        return fwd, steer

            # ── Gentle search rotation ────────────────────────────────────────
            if elapsed < trail_max_s:
                self._pursuit_mode = "search"
                search_steer = float(self._cfg.search_steer_cap_byte)

                # Fix 3: derive search direction from trail tangent (Kevin's actual
                # direction of travel) rather than last observed lateral offset.
                steer_sign = 0.0
                if self._trail is not None and odom is not None:
                    _trail_raw = self._trail.get_trail()
                    if len(_trail_raw) >= 2:
                        _ref_idx = max(0, len(_trail_raw) - 3)
                        _tdx = _trail_raw[-1].x - _trail_raw[_ref_idx].x
                        _tdy = _trail_raw[-1].y - _trail_raw[_ref_idx].y
                        _tdist = math.hypot(_tdx, _tdy)
                        if _tdist > 1e-6:
                            _p = odom.pose
                            _s = math.sin(_p.theta)
                            _c = math.cos(_p.theta)
                            # local_y > 0 → tangent going robot-left → search left → steer < 0
                            _local_y = -_tdx * _s + _tdy * _c
                            steer_sign = -_local_y / _tdist

                # Fall back to last known lateral position when no tangent
                if abs(steer_sign) < 0.05:
                    steer_sign = self._last_target_x or 0.0

                steer = (
                    math.copysign(search_steer, steer_sign)
                    if abs(steer_sign) > 0.05
                    else 0.0
                )
                # Search is a PURE PIVOT — no forward creep. Driving blind-forward
                # while only rotating to reacquire is the dangerous case; rotate
                # in place instead. (Blind trail pursuit above keeps its floor.)
                fwd = 0.0
                self._last_steer_offset = steer
                self._last_speed_offset = fwd
                return fwd, steer

        # ── Full timeout — stop and reset everything ──────────────────────────
        self._reset_tracking_state()
        return 0.0, 0.0

    def _reset_tracking_state(self) -> None:
        """Reset all tracking state after a full target-loss timeout."""
        self._tracking = False
        self._last_distance_error = None
        self._last_speed_offset = 0.0
        self._last_steer_offset = 0.0
        self._last_target_confidence = 0.0
        self._last_target_track_id = None
        self._cached_left = NEUTRAL
        self._cached_right = NEUTRAL
        self._prev_fresh_detection = False
        self._reacq_time = 0.0
        self._steer_decay_factor = 1.0
        self._steer_hold_active = False
        self._last_fresh_detection = False
        self._pursuit_mode = "direct"
        self._last_pursuit_mode = "direct"
        self._depth_filter.reset()
        self._tracker.reset()
        self._steering.reset()
        self._safety.reset()
        if self._trail is not None:
            self._trail.clear()
        if self._pursuit is not None:
            self._pursuit.reset()
        self._trail_distance_m = 0.0
        self._trail_rejected_jump_count = 0
        self._trail_rejected_speed_count = 0
        self._last_target_world_x = None
        self._last_target_world_y = None
        self._trail_exhausted_count = 0
        self._trail_extrapolated = False
        self._trail_extrapolation_count = 0
        self._trail_total_points_added = 0
        if self._odometry is not None:
            self._odometry.reset()
        if self._gps_odom is not None:
            self._gps_odom.reset()
        self._last_emitted_steer = 0.0
        self._last_x_err_norm = None
        self._last_slew_capped_steer = 0.0
        self._slip_straight_ticks = 0
        self._last_steer_want = 0.0
        self._last_steer_src = "lost"
        self._last_emitted_flag = False

    # ── Status / telemetry ───────────────────────────────────────────────────

    def _tunable(self, name: str) -> float:
        """Live value of a config-backed tunable: runtime override if set, else
        the frozen-config default. ``dict.get`` is a single atomic call under
        the GIL, so the control loop never sees a torn value mid-write."""
        val = self._tunable_overrides.get(name)
        return getattr(self._cfg, name) if val is None else val

    def get_tunable_params(self) -> dict:
        """Return the current live values of the five runtime-tunable params.

        Reads the actual live state — kp/kd off the PID, ema off the tracker,
        deadband/slew off the override layer — so it reflects exactly what the
        next control tick will use, not just the frozen config defaults.
        """
        return {
            "pid_lateral_kp": float(self._steering._pid.kp),
            "pid_lateral_kd": float(self._steering._pid.kd),
            "target_ema_alpha": float(self._tracker._alpha),
            "steer_deadband_norm": float(self._tunable("steer_deadband_norm")),
            "steer_slew_per_tick": float(self._tunable("steer_slew_per_tick")),
        }

    def apply_tunable_params(self, params: dict) -> dict:
        """Validate and apply runtime tuning params to the live control objects.

        Validation is all-or-nothing: if *any* key is unknown or *any* value is
        out of its hard bound (``TUNABLE_PARAM_BOUNDS``), raises ``ValueError``
        and applies nothing. Returns the dict of applied values on success.

        Thread-safety: the control loop reads each of these params as a single
        lookup per tick — kp/kd off ``self._steering._pid``, ema off
        ``self._tracker``, and deadband/slew via ``self._tunable()`` (one
        ``dict.get``). Applying a value here is a single attribute store
        (kp/kd/ema) or a single dict item store (deadband/slew). Each of those
        is one bytecode, atomic under the GIL, and no param is part of a read-
        modify-write that spans ticks. So plain assignment from the web thread
        is safe without a lock: a concurrent tick sees either the old or the new
        float, never a torn value. The only observable effect is that a single
        15 Hz tick might mix old/new across the five params (e.g. new kp with
        old kd for one tick) — harmless for tuning, and the very next tick is
        fully consistent. This method sets gains/filters ONLY; it cannot command
        motion.

        (``FollowMeConfig`` is a frozen dataclass, so deadband/slew are applied
        via the ``self._tunable_overrides`` layer rather than mutating config.)
        """
        # ── Phase 1: validate everything before touching any live state ──
        validated: dict[str, float] = {}
        for key, raw in params.items():
            if key not in TUNABLE_PARAM_BOUNDS:
                raise ValueError(f"unknown param: {key!r}")
            try:
                val = float(raw)
            except (TypeError, ValueError):
                raise ValueError(f"{key} must be a number, got {raw!r}")
            lo, hi = TUNABLE_PARAM_BOUNDS[key]
            # NaN/inf fail this comparison and are rejected.
            if not (lo <= val <= hi):
                raise ValueError(f"{key}={val} out of bounds [{lo}, {hi}]")
            validated[key] = val

        # ── Phase 2: apply (all values already validated) ──
        current = self.get_tunable_params()
        for key, val in validated.items():
            old = current[key]
            if key == "pid_lateral_kp":
                self._steering._pid.kp = val
            elif key == "pid_lateral_kd":
                self._steering._pid.kd = val
            elif key == "target_ema_alpha":
                self._tracker._alpha = val
            else:  # steer_deadband_norm / steer_slew_per_tick — read live via _tunable()
                self._tunable_overrides[key] = val
            _log.info("FM tunable param %s: %.4f -> %.4f", key, old, val)
        return validated

    def reset_debug_counters(self):
        """Reset visualization debug counters (rejected jumps/speeds, hysteresis)."""
        self._trail_rejected_jump_count = 0
        self._trail_rejected_speed_count = 0
        self._trail_exhausted_count = 0
        if self._trail is not None:
            self._trail._rejected_jump_count = 0
            self._trail._rejected_speed_count = 0

    def get_status(self, now: float | None = None) -> dict:
        """Return telemetry dict for web viewer / SSE / logging."""
        # Determine human-readable follow mode
        if now is None:
            now = time.monotonic()
        if self._tracking:
            follow_mode = "TRAIL_PURSUIT" if self._pursuit_mode == "trail" else "DIRECT_PID"
        elif self._pursuit_mode == "search":
            follow_mode = "SEARCH_ROTATE"
        elif self._trail_exhausted_count >= 3:
            follow_mode = "TRAIL_EXHAUSTED"
        elif (self._last_valid_time > 0.0
              and (now - self._last_valid_time)
              < float(self._cfg.lost_target_trail_pursuit_max_s)):
            follow_mode = "LOST_BLIND_TRAIL"
        else:
            follow_mode = "IDLE"

        # Lookahead distance from robot
        lookahead_dist_m = 0.0
        odom = self._pick_odometry()
        if odom is not None and (self._pursuit_lookahead_x != 0.0
                                  or self._pursuit_lookahead_y != 0.0):
            pose = odom.pose
            lookahead_dist_m = math.hypot(
                self._pursuit_lookahead_x - pose.x,
                self._pursuit_lookahead_y - pose.y,
            )

        status: dict = {
            "follow_me_tracking": self._tracking,
            "follow_me_target_z_m": self._last_target_z,
            "follow_me_target_x_m": self._last_target_x,
            "follow_me_distance_error_m": self._last_distance_error,
            "follow_me_speed_offset": self._last_speed_offset,
            "follow_me_steer_offset": self._last_steer_offset,
            "follow_me_num_detections": self._last_num_detections,
            "follow_me_target_confidence": self._last_target_confidence,
            "follow_me_target_track_id": self._last_target_track_id,
            "follow_me_pursuit_mode": self._pursuit_mode,
            "follow_me_slip_active": self._last_slip_active,
            "follow_me_actual_speed_mps": self._actual_speed_mps,
            # Decay telemetry (Grok review items 2+3)
            "follow_me_steer_decay_factor": round(self._steer_decay_factor, 3),
            "follow_me_fresh_detection": self._last_fresh_detection,
            "follow_me_steer_hold_active": self._steer_hold_active,
            "follow_me_depth_filtered_m": self._depth_filter.value,
            # ── Visualization / troubleshooting (Items 2 + 4) ────────────
            "follow_mode": follow_mode,
            "target_distance_m": self._last_target_z,
            "target_lateral_offset": round(
                self._tracker._state.normalized_x, 3
            ) if self._tracker._state is not None else None,
            "target_confidence": round(self._last_target_confidence, 3),
            "target_track_id": self._last_target_track_id,
            "hysteresis_count": self._trail_exhausted_count,
            "hysteresis_max": 3,
            "extrapolation_active": self._trail_extrapolated,
            "extrapolation_count": self._trail_extrapolation_count,
            "consume_radius_m": float(self._cfg.trail_consume_radius_m),
        }
        if self._trail_enabled:
            status["trail_length"] = self._trail_length
            status["trail_distance_m"] = round(self._trail_distance_m, 2)
            status["trail_rejected_jump_count"] = self._trail_rejected_jump_count
            status["trail_rejected_speed_count"] = self._trail_rejected_speed_count
            status["trail_lookahead_x"] = self._pursuit_lookahead_x
            status["trail_lookahead_y"] = self._pursuit_lookahead_y
            status["trail_curvature_at_lookahead"] = round(
                self._curvature_at_lookahead, 4)
            status["trail_speed_limited"] = self._speed_limited
            status["follow_me_target_world_x"] = self._last_target_world_x
            status["follow_me_target_world_y"] = self._last_target_world_y
            status["odom_source"] = self._odom_source
            status["trail_exhausted_count"] = self._trail_exhausted_count
            if self._pursuit is not None:
                status["trail_last_closest_idx"] = self._pursuit._last_closest_idx
            # ── Trail visualization fields ───────────────────────────────
            status["trail_point_count"] = self._trail_length
            status["trail_total_points"] = self._trail_total_points_added
            status["trail_remaining_m"] = round(self._trail_distance_m, 2)
            status["trail_lookahead_m"] = round(lookahead_dist_m, 2)
            status["trail_curvature"] = round(self._curvature_at_lookahead, 4)
            status["rejected_jumps"] = self._trail_rejected_jump_count
            status["rejected_speeds"] = self._trail_rejected_speed_count
            # Trail points for 2D canvas (last 20)
            if self._trail is not None:
                trail_pts = self._trail.get_smoothed_trail()
                viz_pts = trail_pts[-20:]
                status["trail_points_xy"] = [
                    [round(p.x, 3), round(p.y, 3)] for p in viz_pts
                ]
            else:
                status["trail_points_xy"] = []
            status["lookahead_point_xy"] = (
                [round(self._pursuit_lookahead_x, 3),
                 round(self._pursuit_lookahead_y, 3)]
                if (self._pursuit_lookahead_x != 0.0
                    or self._pursuit_lookahead_y != 0.0)
                else None
            )
            odom = self._pick_odometry()
            if odom is not None:
                pose = odom.pose
                status["odom_x"] = round(pose.x, 3)
                status["odom_y"] = round(pose.y, 3)
                status["odom_theta_deg"] = round(math.degrees(pose.theta), 1)
                status["robot_pose"] = {
                    "x": round(pose.x, 3),
                    "y": round(pose.y, 3),
                    "theta": round(pose.theta, 4),
                }
            else:
                status["robot_pose"] = None
            if self._gps_odom is not None:
                status["gps_speed_mps"] = round(self._gps_odom.speed_mps, 2)
        else:
            status["trail_points_xy"] = []
            status["lookahead_point_xy"] = None
            status["robot_pose"] = None
            status["trail_point_count"] = 0
            status["trail_total_points"] = 0
            status["trail_remaining_m"] = 0.0
            status["trail_lookahead_m"] = 0.0
            status["trail_curvature"] = 0.0
            status["rejected_jumps"] = 0
            status["rejected_speeds"] = 0
        return status
