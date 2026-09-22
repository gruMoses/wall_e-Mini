from dataclasses import dataclass
from typing import Any, Protocol, Tuple, List, Optional
import logging
import math
import time
import threading
import sys
from pathlib import Path

_logger = logging.getLogger(__name__)

# Add parent directory to path for config import
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.control.mapping import (
    map_pulse_to_byte, map_pulse_to_byte_saturated,
    CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT,
)
from pi_app.control.safety import update_safety, SafetyState, SafetyParams, SafetyEvent
from pi_app.control.state import DriveCommand, AutonomyCommand
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.gesture_control import (
    GestureStateMachine, GestureEvent, HandData,
    hand_poll_wanted as _hand_poll_wanted,
)
from pi_app.control.arms_up import ArmsUpDetector, PoseSample
from pi_app.control.waypoint_nav import WaypointNavController, NavState, mix_to_bytes
from pi_app.control.gps_heading_align import GpsHeadingAligner
from pi_app.control.rpm_plausibility import (
    RpmPlausibilityConfig,
    RpmPlausibilityGate,
    erpm_to_wheel_mps,
)
from pi_app.hardware.rtk_gps import GpsReading
from config import config

RC_STALE_TIMEOUT_S = 1.0


@dataclass(frozen=True)
class RCInputs:
    ch1_us: int
    ch2_us: int
    ch3_us: int
    ch4_us: int
    ch5_us: int
    last_update_epoch_s: float


class MotorDriver(Protocol):
    def set_tracks(self, left_byte: int, right_byte: int) -> None: ...
    def stop(self) -> None: ...
    def get_telemetry(self): ...  # Returns VescTelemetry | None


class ArmRelay(Protocol):
    def set_armed(self, armed: bool) -> None: ...


class ShutdownScheduler(Protocol):
    def schedule_shutdown(self, delay_seconds: float) -> None: ...


class NoopMotorDriver:
    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_telemetry(self):
        return None


class NoopArmRelay:
    def set_armed(self, armed: bool) -> None:
        pass


class ThreadedShutdownScheduler:
    def __init__(self, command: str = "sudo shutdown -h now") -> None:
        self._command = command
        self._scheduled = False

    def schedule_shutdown(self, delay_seconds: float) -> None:
        if self._scheduled:
            return
        self._scheduled = True

        def _task():
            time.sleep(delay_seconds)
            try:
                import subprocess
                subprocess.Popen(self._command.split())
            except Exception:
                pass

        t = threading.Thread(target=_task, name="SystemShutdown", daemon=False)
        t.start()


class Controller:
    NEUTRAL = CENTER_OUTPUT_VALUE

    def __init__(
        self,
        motor_driver: MotorDriver | None = None,
        arm_relay: ArmRelay | None = None,
        shutdown_scheduler: ShutdownScheduler | None = None,
        safety_params: SafetyParams | None = None,
        imu_compensator: Optional[ImuSteeringCompensator] = None,
        obstacle_avoidance: Optional[ObstacleAvoidanceController] = None,
        follow_me: Optional[FollowMeController] = None,
        waypoint_nav: Optional[WaypointNavController] = None,
        gesture_controller: Optional[GestureStateMachine] = None,
        gps_heading_aligner: Optional[GpsHeadingAligner] = None,
    ) -> None:
        self._motor = motor_driver or NoopMotorDriver()
        self._relay = arm_relay or NoopArmRelay()
        self._shutdown = shutdown_scheduler or ThreadedShutdownScheduler()
        self._safety_state = SafetyState(is_armed=False, last_transition_epoch_s=0.0)
        self._safety_params = safety_params or SafetyParams()
        
        # IMU steering compensation
        self._imu_compensator = imu_compensator
        # Use a monotonic clock so IMU update intervals are not affected
        # by system clock adjustments.
        self._last_imu_update = time.monotonic()
        if config.imu_steering.enabled:
            rate_hz = float(getattr(config.imu_steering, "update_rate_hz", 0))
            if rate_hz <= 0:
                raise ValueError("config.imu_steering.update_rate_hz must be positive")
            oak_poll_hz = float(getattr(config.imu_steering, "oak_imu_poll_hz", rate_hz))
            if oak_poll_hz > 0:
                # Keep controller update cadence aligned to available OAK IMU ingestion cadence.
                rate_hz = min(rate_hz, oak_poll_hz)
            self._imu_update_interval = 1.0 / rate_hz
        else:
            self._imu_update_interval = 1.0
        # Track when we begin moving straight to (re)lock heading
        self._was_moving_straight = False
        self._straight_latched = False
        self._imu_frame_check_ts: float = 0.0
        self._imu_frame_marker: Optional[tuple[int, int]] = None
        self._straight_disengage_deadline = 0.0
        self._straight_target_true_heading: Optional[float] = None

        # Obstacle avoidance, Follow Me, Waypoint Nav, and Gesture control
        self._obstacle_avoidance = obstacle_avoidance
        self._follow_me = follow_me
        self._waypoint_nav = waypoint_nav
        self._gesture = gesture_controller
        # GPS→IMU heading alignment locks only from an explicit forward,
        # straight manual-RC run, then remains frozen for the armed session.
        self._gps_heading_aligner = gps_heading_aligner
        self._mode = "MANUAL"  # "MANUAL", "FOLLOW_ME", or "WAYPOINT_NAV"
        self._obstacle_distance_m: float | None = None
        self._obstacle_age_s: float | None = None
        self._gps_reading: GpsReading | None = None
        self._person_detections: list[PersonDetection] = []
        self._hand_data: HandData | None = None
        self._pose_sample: PoseSample | None = None
        self._pose_status: dict = {}
        self._arms_up = ArmsUpDetector(getattr(config, "arms_up", None))
        # Bench-test reverse twitch. Times are compared against process()'s
        # mono_now — do not call time.monotonic() here (controller tests
        # pin the init call count). The web thread mutates the latch through
        # request_twitch_test(); process() mutates it under the same lock.
        self._twitch_lock = threading.Lock()
        self._twitch_test_latched: bool = False
        self._twitch_budget_left: int = 0
        self._twitch_expires_at: float = 0.0
        self._twitch_until: float = 0.0
        self._twitch_cooldown_until: float = 0.0
        self._twitch_count: int = 0
        self._twitch_blocked_reason: Optional[str] = None
        self._twitch_cancel_reason: Optional[str] = None
        self._twitch_cleared_reason: Optional[str] = None
        self._twitch_active: bool = False
        self._twitch_edge_pending: bool = False
        self._twitch_reverse_n: int = 22
        self._twitch_still_since: Optional[float] = None
        self._twitch_clamp_warned: bool = False

        # Charger inhibit: set True when BMS reports charging; blocks motor output.
        # Fail-open by design — cleared externally if BMS becomes unreachable.
        self._charger_inhibit: bool = False

        # Calibration mode: when True, process() outputs neutral and skips logic
        self._calibration_mode = False

        # Final-stage slew limiter state.
        self._slew_last_left = CENTER_OUTPUT_VALUE
        self._slew_last_right = CENTER_OUTPUT_VALUE
        # Previous FOLLOW_ME tick's post-slew common-mode forward byte, so the
        # velocity PID can target what actually reached the motors. None when
        # the previous process() tick was not FOLLOW_ME.
        self._last_follow_me_emitted_forward_byte: Optional[float] = None
        self._slew_last_update = time.monotonic()
        self._slew_initialized = False
        self._slew_seen_non_neutral = False

        # VESC telemetry state — polled every 50 ms in process().
        self._telem_last_poll: float = 0.0
        self._telem_last_valid: float = 0.0
        self._telem_stale_warned: bool = False
        # RPM plausibility gate: "commanded to move, eRPM says 0" for a sustained
        # window → RPM/speed nulled (open-loop) until real RPM returns. Guards the
        # velocity PID and slip detector against dead readback (2026-06-11 lunge).
        _vcfg = getattr(config, "vesc", None)
        self._rpm_gate: Optional[RpmPlausibilityGate] = None
        if bool(getattr(_vcfg, "rpm_plausibility_enabled", True)):
            self._rpm_gate = RpmPlausibilityGate(RpmPlausibilityConfig(
                enabled=True,
                min_cmd_bytes=int(getattr(_vcfg, "rpm_plausibility_min_cmd_bytes", 12)),
                min_erpm=int(getattr(_vcfg, "rpm_plausibility_min_erpm", 150)),
                window_s=float(getattr(_vcfg, "rpm_plausibility_window_s", 0.5)),
                hold_s=float(getattr(_vcfg, "rpm_plausibility_hold_s", 2.0)),
            ))
        self._rpm_gate_warned: bool = False
        self._vesc_rpm_plausible: bool = True
        self._actual_left_rpm: Optional[int] = None
        self._actual_right_rpm: Optional[int] = None
        self._actual_speed_mps: Optional[float] = None
        self._actual_left_current_a: Optional[float] = None
        self._actual_right_current_a: Optional[float] = None
        self._actual_left_temp_c: Optional[float] = None
        self._actual_right_temp_c: Optional[float] = None
        self._actual_left_motor_temp_c: Optional[float] = None
        self._actual_right_motor_temp_c: Optional[float] = None
        self._actual_left_duty: Optional[float] = None
        self._actual_right_duty: Optional[float] = None
        self._vesc_rx_frame_count: int = 0
        self._vesc_rx_parse_error_count: int = 0
        self._vesc_rx_recv_error_count: int = 0
        self._vesc_rx_reopen_count: int = 0
        self._vesc_rx_last_frame_age_s: Optional[float] = None
        # VESC low-voltage watchdog latch (early-warning + motor-cutoff). Mirrors
        # the charger_inhibit plumbing so the run log / SSE / dashboard can see it.
        self._vesc_pack_low_latched: bool = False
        # Per-motor STATUS(9) frame ages + RX-thread liveness. Already read below
        # to drive the open-loop staleness fallback; also surfaced (same
        # charger_inhibit plumbing) so the /debug board can show which motor
        # went silent and whether the CAN RX thread is alive.
        self._vesc_left_status_age_s: Optional[float] = None
        self._vesc_right_status_age_s: Optional[float] = None
        self._vesc_rx_thread_alive: Optional[bool] = None

    def _reset_imu_timestamp(self, now: float) -> None:
        """Reset the monotonic timestamp used to throttle IMU updates.

        This helper allows tests to control when the next IMU update is
        permitted without reaching into private attributes.
        """
        self._last_imu_update = now

    def set_obstacle_data(self, distance_m: float, age_s: float) -> None:
        """Feed latest depth reading from OakDepthReader."""
        self._obstacle_distance_m = distance_m
        self._obstacle_age_s = age_s

    def set_person_detections(self, detections: list[PersonDetection]) -> None:
        """Feed latest person detections from OakDepthReader."""
        self._person_detections = detections

    def set_hand_data(self, data: HandData | None) -> None:
        """Feed latest hand landmark data from OakDepthReader."""
        self._hand_data = data

    def set_pose_sample(self, sample: PoseSample | None) -> None:
        """Feed latest PoseSample from PoseWorker (None = no pose this frame)."""
        self._pose_sample = sample

    def set_pose_status(self, status: dict | None) -> None:
        """Feed PoseWorker diagnostics (pose_ms / pose_hz / mp_pose_loaded / ...)."""
        self._pose_status = dict(status) if status else {}

    def pose_wanted(self) -> bool:
        """True only while the bench-twitch latch is on.

        The main loop uses this to skip MediaPipe Pose unless the gated
        bench test is enabled. FOLLOW_ME, WAYPOINT_NAV, disarm, expiry,
        and a spent budget clear the latch, so pose stays off there.
        """
        return bool(self._twitch_test_latched)

    def request_twitch_test(self, enabled: bool) -> tuple[bool, str]:
        """Latch or clear the bench-twitch test. Called from the web thread.

        ``enabled=True`` is accepted only while the feature is permitted and
        the robot is armed, in MANUAL, not in emergency, not charger-inhibited,
        not pack-low latched, and not in calibration. Otherwise the latch is
        left unchanged and the reason is returned.

        ``enabled=False`` clears the latch and cancels an active pulse.
        The latch is volatile. It must not be reused for continuous back-up.
        """
        now = time.monotonic()
        cfg = getattr(config, "arms_up", None)
        with self._twitch_lock:
            if not enabled:
                self._cancel_twitch_pulse_locked(now, "api")
                self._twitch_test_latched = False
                self._twitch_budget_left = 0
                self._twitch_expires_at = 0.0
                self._twitch_edge_pending = False
                _logger.warning("ARMS-UP twitch test DISABLED (reason %s)", "api")
                return True, "ok"
            reason = self._twitch_enable_refusal_locked(cfg)
            if reason is not None:
                return False, reason
            budget = int(getattr(cfg, "twitch_test_budget", 3))
            max_s = float(getattr(cfg, "twitch_test_max_s", 300.0))
            self._twitch_test_latched = True
            self._twitch_budget_left = budget
            self._twitch_expires_at = now + max_s
            _logger.warning(
                "ARMS-UP twitch test ENABLED (budget %d, expires in %.0f s)",
                budget,
                max_s,
            )
            return True, "ok"

    def get_twitch_test_state(self) -> dict:
        """Latch snapshot for the web thread and telemetry consumers."""
        now = time.monotonic()
        with self._twitch_lock:
            return self._twitch_state_locked(now)

    def _twitch_enable_refusal_locked(self, cfg: Any) -> Optional[str]:
        if cfg is None or not bool(getattr(cfg, "enabled", True)):
            return "disabled"
        if not self._safety_state.is_armed:
            return "disarmed"
        if self._safety_state.emergency_active:
            return "emergency"
        if self._mode != "MANUAL":
            return "mode"
        if self._charger_inhibit:
            return "charger"
        if self._vesc_pack_low_latched:
            return "pack_low"
        if self._calibration_mode:
            return "calibration"
        return None

    def _twitch_state_locked(self, now: float) -> dict:
        if self._twitch_test_latched:
            expires: Optional[float] = self._twitch_expires_at - now
            if expires < 0.0:
                expires = 0.0
        else:
            expires = None
        return {
            "latched": self._twitch_test_latched,
            "budget_left": int(self._twitch_budget_left),
            "expires_in_s": expires,
            "twitch_count": int(self._twitch_count),
            "twitch_active": bool(self._twitch_active),
            "twitch_blocked_reason": self._twitch_blocked_reason,
            "twitch_cancel_reason": self._twitch_cancel_reason,
        }

    def set_gps_reading(self, reading: GpsReading | None) -> None:
        """Feed latest reading from RtkGpsReader."""
        self._gps_reading = reading

    def set_charger_inhibit(
        self,
        inhibit: bool,
        bms_current_a: Optional[float] = None,
        charge_fet_on: Optional[bool] = None,
    ) -> None:
        """Set or clear the charger inhibit flag.

        When True, process() will output neutral and stop the motors regardless
        of RC/BT commands.  Call with False when the BMS is unreachable (fail-open).

        ``bms_current_a`` / ``charge_fet_on`` are optional context for the one-line
        ENGAGE/RELEASE transition log below — charger_inhibit was previously logged
        NOWHERE, which is why the 2026-06-13 motor-cutout incident took a field-log
        deep-dive to root-cause instead of a grep.
        """
        inhibit = bool(inhibit)
        if inhibit != self._charger_inhibit:
            _logger.warning(
                "charger_inhibit %s — bms_current_a=%s charge_fet_on=%s",
                "ENGAGED (motors forced neutral)" if inhibit else "RELEASED",
                bms_current_a,
                charge_fet_on,
            )
        self._charger_inhibit = inhibit

    def _follow_me_target_present(self) -> bool:
        """True when a Follow Me target candidate is currently visible."""
        return bool(self._person_detections)

    def activate_follow_me(self) -> bool:
        """Enter FOLLOW_ME mode from web UI. Returns True if activated."""
        if (
            self._follow_me is not None
            and self._safety_state.is_armed
            and self._follow_me_target_present()
        ):
            self._mode = "FOLLOW_ME"
            self._safety_state.set_follow_me_active(True)
            self._follow_me.start_recorder()
            return True
        return False

    def deactivate_follow_me(self) -> None:
        """Return to MANUAL mode from Follow Me."""
        if self._mode == "FOLLOW_ME":
            self._mode = "MANUAL"
            self._safety_state.set_follow_me_active(False)
            if self._follow_me is not None:
                self._follow_me.stop_recorder()
            if self._gesture is not None:
                self._gesture.notify_external_deactivation()

    @property
    def motor_driver(self) -> MotorDriver:
        return self._motor

    @property
    def gesture_phase_active(self) -> bool:
        """True when the gesture machine is in phase ACTIVE (FIVE is honoured)."""
        return self._gesture is not None and self._gesture.is_active

    def hand_poll_wanted(self, is_armed: bool) -> bool:
        """Whether host-side MediaPipe Hands should run this tick.

        Armed and (mode is not FOLLOW_ME, or the gesture machine is ACTIVE).
        ``GestureConfig.hand_poll_in_follow_me=True`` restores the old
        always-when-armed behaviour.
        """
        cfg = getattr(config, "gesture", None)
        return _hand_poll_wanted(
            is_armed=bool(is_armed),
            mode=self._mode,
            phase_active=self.gesture_phase_active,
            hand_poll_in_follow_me=bool(getattr(cfg, "hand_poll_in_follow_me", False)),
        )

    @property
    def is_armed(self) -> bool:
        """Current armed state. Read by the calibration wizard so it can abort
        its direct motor commands when the operator disarms (ch3) mid-run."""
        return self._safety_state.is_armed

    @property
    def emergency_active(self) -> bool:
        """True once a latched ch5 e-stop has fired. Read by the calibration
        wizard so it aborts on emergency."""
        return self._safety_state.emergency_active

    def enter_calibration_mode(self) -> None:
        """Pause normal control; process() will output neutral commands."""
        self._mode = "MANUAL"
        self._calibration_mode = True
        self._motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)

    def exit_calibration_mode(self) -> None:
        """Resume normal control loop."""
        self._calibration_mode = False
        self._motor.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)

    @property
    def in_calibration_mode(self) -> bool:
        return self._calibration_mode

    def waypoint_nav_activation_blocked(self) -> Optional[str]:
        """Return a machine-readable reason if waypoint nav must not start."""
        nav = self._waypoint_nav
        gps = self._gps_reading
        if nav is not None and gps is not None and not nav.accepts_fix_quality(gps.fix_quality):
            return "gps_quality_not_trusted"

        aligner = self._gps_heading_aligner
        if aligner is None or not aligner.enabled:
            return None
        if self._imu_compensator is None or gps is None:
            return None
        if aligner.locked:
            return None
        return "heading_alignment_not_locked"

    def activate_waypoint_nav(self) -> Optional[str]:
        """Enter WAYPOINT_NAV mode (call from UI / CLI).

        Returns None on success, or a machine-readable block reason.
        """
        blocked = self.waypoint_nav_activation_blocked()
        if blocked is not None:
            return blocked
        if self._waypoint_nav is not None and not self._waypoint_nav.completed:
            self._mode = "WAYPOINT_NAV"
            return None
        return "waypoint_nav_unavailable"

    def deactivate_waypoint_nav(self) -> None:
        """Return to MANUAL mode from waypoint nav."""
        if self._mode == "WAYPOINT_NAV":
            self._mode = "MANUAL"

    @staticmethod
    def _scale_toward_neutral(byte_val: int, scale: float) -> int:
        """Interpolate a motor byte toward neutral (126) by the given scale.

        scale=1.0 -> unchanged, scale=0.0 -> neutral.
        """
        result = CENTER_OUTPUT_VALUE + (byte_val - CENTER_OUTPUT_VALUE) * scale
        return max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(result))))

    def _reset_slew_state(self, now_s: float) -> None:
        self._slew_last_left = CENTER_OUTPUT_VALUE
        self._slew_last_right = CENTER_OUTPUT_VALUE
        self._slew_last_update = now_s
        self._slew_initialized = False
        self._slew_seen_non_neutral = False
        self._straight_target_true_heading = None

    def _on_armed_session_ended(self) -> None:
        """An armed session ended (disarm, RC stale, e-stop).

        The GPS heading lock is KEPT (2026-09-19, Kevin's decision): the
        offset relates the IMU's boot frame to true north, and that frame is
        continuous across a disarm — the gyro keeps integrating, ZUPT holds
        the heading while parked, and the tracked bias keeps it honest. The
        lock is verified against live course over ground whenever the robot
        drives forward at RTK fixed (GpsHeadingAligner.update_cog) and is
        dropped on an IMU frame discontinuity (see _check_imu_frame_continuity).
        Before this change every disarm forced another straight run.
        """
        self._straight_target_true_heading = None

    def _check_imu_frame_continuity(self, mono_now: float) -> None:
        """Drop the GPS heading lock if the IMU heading frame was reseeded.

        An OAK USB reconnect or a producer cum reset can lose rotation that
        happened during the outage, so a frozen offset may no longer relate
        the heading to true north. Checked once per second from the reader's
        health counters; fail-closed.
        """
        if self._gps_heading_aligner is None or self._imu_compensator is None:
            return
        if (mono_now - self._imu_frame_check_ts) < 1.0:
            return
        self._imu_frame_check_ts = mono_now
        try:
            reader = getattr(self._imu_compensator, "imu_reader", None)
            health_fn = getattr(reader, "get_health", None)
            if not callable(health_fn):
                return
            h = health_fn() or {}
            marker = (
                int(h.get("oak_reconnect_count") or 0),
                int(h.get("count_cum_reset") or 0),
            )
            if (
                self._imu_frame_marker is not None
                and marker != self._imu_frame_marker
                and self._gps_heading_aligner.locked
            ):
                self._gps_heading_aligner.reset()
                _logger.warning(
                    "GPS heading lock dropped: IMU frame discontinuity "
                    "(oak_reconnect_count=%d, count_cum_reset=%d); drive forward "
                    "at RTK fixed to relock",
                    marker[0], marker[1],
                )
            self._imu_frame_marker = marker
        except Exception:
            pass

    def _heading_align_telemetry(self, raw_heading: Optional[float]) -> dict:
        """Nested heading-alignment observability for controller telemetry."""
        aligner = self._gps_heading_aligner
        if aligner is None:
            return {
                "enabled": False,
                "locked": False,
                "frozen": False,
                "refining": False,
                "offset_deg": 0.0,
                "corrected_heading_deg": raw_heading,
                "last_cog_deg": None,
                "last_speed_mps": None,
                "history_samples": 0,
                "cog_samples": 0,
                "cog_spread_deg": None,
                "lock_source": None,
            }
        st = aligner.status()
        corrected = aligner.correct(raw_heading) if raw_heading is not None else None
        return {
            "enabled": st.enabled,
            "locked": st.locked,
            "frozen": st.frozen,
            "refining": st.refining,
            "offset_deg": st.offset_deg,
            "corrected_heading_deg": corrected,
            "last_cog_deg": st.last_cog_deg,
            "last_speed_mps": st.last_speed_mps,
            "history_samples": st.history_samples,
            "cog_samples": st.cog_samples,
            "cog_spread_deg": st.cog_spread_deg,
            "lock_source": st.lock_source,
            "cog_verify_error_deg": st.cog_verify_error_deg,
            "relock_count": st.relock_count,
        }

    @staticmethod
    def _slew_toward_target(
        previous: int,
        target: int,
        max_accel_delta: float,
        max_decel_delta: float,
    ) -> int:
        prev_mag = abs(previous - CENTER_OUTPUT_VALUE)
        tgt_mag = abs(target - CENTER_OUTPUT_VALUE)
        delta = target - previous
        if delta == 0:
            return previous

        limit = max_accel_delta if tgt_mag > prev_mag else max_decel_delta
        if limit <= 0:
            return previous
        if abs(delta) <= limit:
            return target
        stepped = previous + int(round(limit if delta > 0 else -limit))
        return max(MIN_OUTPUT, min(MAX_OUTPUT, stepped))

    def _slew_rates_for_mode(self, mode: str) -> tuple[float, float]:
        slewc = getattr(config, "slew_limiter", None)
        if slewc is None:
            return 1e9, 1e9
        if mode == "FOLLOW_ME":
            return float(slewc.follow_me_accel_bps), float(slewc.follow_me_decel_bps)
        if mode == "WAYPOINT_NAV":
            return float(slewc.waypoint_nav_accel_bps), float(slewc.waypoint_nav_decel_bps)
        return float(slewc.manual_accel_bps), float(slewc.manual_decel_bps)

    def _compute_waypoint_autonomy_command(self, telemetry: dict) -> tuple[AutonomyCommand, bool, bool]:
        left = right = self.NEUTRAL
        wp_pivot_active = False
        wp_in_align = False
        nav_state = NavState.IDLE
        if self._waypoint_nav is None:
            return (
                AutonomyCommand(
                    source="WAYPOINT_NAV",
                    left_byte=left,
                    right_byte=right,
                    steering_input=0.0,
                    nav_state=nav_state.value,
                ),
                wp_pivot_active,
                wp_in_align,
            )

        gps = self._gps_reading
        if gps is not None:
            gps_age = time.monotonic() - gps.timestamp
            raw_heading = None
            if self._imu_compensator is not None:
                try:
                    raw_heading = self._imu_compensator.get_heading_deg()
                except Exception:
                    raw_heading = None
            if raw_heading is not None and self._gps_heading_aligner is not None:
                corrected_heading = self._gps_heading_aligner.correct(raw_heading)
            else:
                corrected_heading = raw_heading
            v_cmd, yaw_cmd, nav_state = self._waypoint_nav.compute(
                gps.latitude, gps.longitude, gps.fix_quality, gps_age,
                current_heading_deg=corrected_heading,
            )
            # Keep target heading in sync so DRIVE transition is seamless.
            if self._imu_compensator is not None and not self._waypoint_nav.completed:
                nav_st = self._waypoint_nav.get_status()
                if self._gps_heading_aligner is not None:
                    imu_target = self._gps_heading_aligner.imu_target_heading(nav_st.bearing_deg)
                else:
                    imu_target = nav_st.bearing_deg
                self._imu_compensator.set_target_heading(imu_target)
            deadband = int(getattr(config.waypoint_nav, "motor_deadband_byte", 12))
            left, right = mix_to_bytes(v_cmd, yaw_cmd, deadband_byte=deadband,
                                       neutral=CENTER_OUTPUT_VALUE, half_range=127)
            if nav_state == NavState.ALIGN:
                wp_in_align = True
                wp_pivot_active = True

        nav_st = self._waypoint_nav.get_status()
        telemetry["wp_index"] = nav_st.waypoint_index
        telemetry["wp_total"] = nav_st.waypoint_total
        telemetry["wp_name"] = nav_st.waypoint_name
        telemetry["wp_bearing_deg"] = nav_st.bearing_deg
        telemetry["wp_distance_m"] = nav_st.distance_m
        telemetry["wp_heading_error_deg"] = nav_st.heading_error_deg
        telemetry["wp_completed"] = nav_st.completed
        telemetry["nav_state"] = nav_st.state
        telemetry["wp_v_cmd"] = nav_st.v_cmd
        telemetry["wp_yaw_cmd"] = nav_st.yaw_cmd
        if nav_st.completed:
            self._mode = "MANUAL"

        return (
            AutonomyCommand(
                source="WAYPOINT_NAV",
                left_byte=left,
                right_byte=right,
                steering_input=0.0,
                nav_state=nav_st.state,
            ),
            wp_pivot_active,
            wp_in_align,
        )

    def _compute_follow_me_autonomy_command(self, telemetry: dict, mono_now: float) -> AutonomyCommand:
        left = right = self.NEUTRAL
        heading = 0.0
        if self._imu_compensator is not None:
            try:
                heading = self._imu_compensator.get_heading_deg()
                self._last_imu_update = mono_now
            except Exception:
                pass
        if self._follow_me is not None:
            self._follow_me.set_arm_state(self._safety_state.is_armed)
            self._follow_me.update_pose(
                heading, self._slew_last_left, self._slew_last_right, mono_now
            )
            if self._gps_reading is not None:
                self._follow_me.update_gps(
                    self._gps_reading.latitude,
                    self._gps_reading.longitude,
                    self._gps_reading.fix_quality,
                    self._gps_reading.timestamp,
                )
            self._follow_me.update_telemetry(
                left_rpm=self._actual_left_rpm,
                right_rpm=self._actual_right_rpm,
                actual_speed_mps=self._actual_speed_mps,
                left_current_a=self._actual_left_current_a,
                right_current_a=self._actual_right_current_a,
                left_temp_c=self._actual_left_temp_c,
                right_temp_c=self._actual_right_temp_c,
                charger_inhibit=self._charger_inhibit,
                emitted_forward_byte=self._last_follow_me_emitted_forward_byte,
            )
            detections = self._person_detections or []
            left, right = self._follow_me.compute(detections)
            fm_status = self._follow_me.get_status(now=mono_now)
            fm_status["follow_me_num_persons"] = len(detections)
            telemetry.update(fm_status)
        steering_input = self._bytes_to_steering_input(left, right)
        return AutonomyCommand(
            source="FOLLOW_ME",
            left_byte=left,
            right_byte=right,
            steering_input=steering_input,
        )

    def _update_arms_up_twitch(self, now: float) -> None:
        """Debounce the pose sample. A rising edge stays pending until the
        injection point accepts or rejects it on this same tick.
        """
        self._arms_up.update(self._pose_sample, now)
        if self._arms_up.rising_edge:
            self._twitch_edge_pending = True

    def _maintain_twitch_latch(self, now: float) -> None:
        """Clear the latch when a tick is no longer a legal bench-test tick.

        Budget exhaustion waits until the current pulse has finished.
        Clearing the latch also cancels a pulse that is still running.
        """
        with self._twitch_lock:
            self._twitch_cleared_reason = None
            if (
                not self._twitch_test_latched
                and not (now < self._twitch_until)
                and not self._twitch_active
            ):
                return
            reason = self._twitch_latch_clear_reason_locked(now)
            if reason is not None:
                self._clear_twitch_latch_locked(now, reason)

    def _twitch_latch_clear_reason_locked(self, now: float) -> Optional[str]:
        if self._safety_state.emergency_active:
            return "emergency"
        if not self._safety_state.is_armed:
            return "disarmed"
        if self._mode != "MANUAL":
            return "mode"
        if self._calibration_mode:
            return "calibration"
        if self._twitch_test_latched and now >= self._twitch_expires_at:
            return "expiry"
        if (
            self._twitch_test_latched
            and self._twitch_budget_left <= 0
            and not (now < self._twitch_until)
        ):
            return "budget"
        return None

    def _clear_twitch_latch_locked(self, now: float, reason: str) -> None:
        self._twitch_cleared_reason = reason
        if self._twitch_test_latched:
            self._twitch_test_latched = False
            self._twitch_budget_left = 0
            self._twitch_expires_at = 0.0
            _logger.warning("ARMS-UP twitch test DISABLED (reason %s)", reason)
        self._cancel_twitch_pulse_locked(now, reason)

    def _cancel_twitch_pulse_locked(self, now: float, reason: str) -> None:
        # Time, not the latched _twitch_active flag: that flag stays True
        # until the injection point, so a pulse that already reached its
        # end would be reported as cancelled by the latch clear that follows.
        pulse_live = now < self._twitch_until
        self._twitch_until = 0.0
        self._twitch_active = False
        if pulse_live:
            self._twitch_cancel_reason = reason

    def _twitch_early_exit(self, now: float, reason: str) -> None:
        """RC-stale and calibration returns: drop the latch, the pulse, and
        the detector streak. The re-arm latch inside the detector is kept.
        """
        with self._twitch_lock:
            self._twitch_edge_pending = False
            if (
                self._twitch_test_latched
                or self._twitch_active
                or now < self._twitch_until
            ):
                self._clear_twitch_latch_locked(now, reason)
            else:
                self._twitch_until = 0.0
                self._twitch_active = False
        self._arms_up.reset()
        self._twitch_still_since = None

    def _sticks_within_twitch_band(self, left: int, right: int, cfg: Any) -> bool:
        band = int(getattr(cfg, "twitch_stick_neutral_band", 6) if cfg is not None else 6)
        return (
            abs(int(left) - CENTER_OUTPUT_VALUE) <= band
            and abs(int(right) - CENTER_OUTPUT_VALUE) <= band
        )

    def _twitch_still_ok(self, now: float, cfg: Any) -> bool:
        since = self._twitch_still_since
        min_s = float(getattr(cfg, "twitch_min_still_s", 0.5) if cfg is not None else 0.5)
        if since is None or (now - since) < min_s - 1e-9:
            return False
        limit = float(getattr(cfg, "twitch_max_still_erpm", 300.0) if cfg is not None else 300.0)
        for rpm in (self._actual_left_rpm, self._actual_right_rpm):
            if rpm is None:
                continue
            if abs(float(rpm)) > limit:
                return False
        return True

    def _clamped_twitch_params(self, cfg: Any) -> tuple[int, float]:
        raw_n = int(getattr(cfg, "twitch_reverse_byte", 22) if cfg is not None else 22)
        raw_d = float(getattr(cfg, "twitch_duration_s", 0.25) if cfg is not None else 0.25)
        n = max(0, min(30, raw_n))
        dur = max(0.0, min(0.4, raw_d))
        if (n != raw_n or dur != raw_d) and not self._twitch_clamp_warned:
            self._twitch_clamp_warned = True
            _logger.warning(
                "ARMS-UP twitch config clamped: reverse_byte %s -> %s, duration_s %s -> %s",
                raw_n, n, raw_d, dur,
            )
        return n, dur

    def _twitch_start_block_locked(
        self,
        now: float,
        left: int,
        right: int,
        bt: tuple[int, int] | None,
        cfg: Any,
    ) -> Optional[str]:
        if cfg is None or not bool(getattr(cfg, "enabled", True)):
            return "disabled"
        if not self._twitch_test_latched:
            return self._twitch_cleared_reason or "latch"
        if self._twitch_budget_left <= 0:
            return "budget"
        if self._safety_state.emergency_active:
            return "emergency"
        if not self._safety_state.is_armed:
            return "disarmed"
        if self._mode != "MANUAL":
            return "mode"
        if self._charger_inhibit:
            return "charger"
        if self._vesc_pack_low_latched:
            return "pack_low"
        if self._calibration_mode:
            return "calibration"
        if bt is not None:
            return "bt_override"
        if not self._sticks_within_twitch_band(left, right, cfg):
            return "stick"
        if not self._twitch_still_ok(now, cfg):
            return "not_still"
        if now < self._twitch_cooldown_until:
            return "cooldown"
        return None

    def _twitch_continue_block_locked(
        self,
        left: int,
        right: int,
        bt: tuple[int, int] | None,
        cfg: Any,
    ) -> Optional[str]:
        # (a) latch only — a spent budget does not cancel the pulse it paid for.
        if not self._twitch_test_latched:
            return self._twitch_cleared_reason or "latch"
        if self._safety_state.emergency_active:
            return "emergency"
        if not self._safety_state.is_armed:
            return "disarmed"
        if self._mode != "MANUAL":
            return "mode"
        if self._charger_inhibit:
            return "charger"
        if self._vesc_pack_low_latched:
            return "pack_low"
        if self._calibration_mode:
            return "calibration"
        if bt is not None:
            return "bt_override"
        if not self._sticks_within_twitch_band(left, right, cfg):
            return "stick"
        return None

    def _service_twitch_injection(
        self,
        now: float,
        left: int,
        right: int,
        bt: tuple[int, int] | None,
    ) -> tuple[int, int]:
        """Start or continue the reverse pulse, after the per-mode command
        and the IMU correction. A blocked edge is consumed here.
        """
        cfg = getattr(config, "arms_up", None)
        with self._twitch_lock:
            if self._twitch_edge_pending:
                self._twitch_edge_pending = False
                reason = self._twitch_start_block_locked(now, left, right, bt, cfg)
                if reason is None:
                    n, dur = self._clamped_twitch_params(cfg)
                    cooldown = float(
                        getattr(cfg, "twitch_cooldown_s", 3.0) if cfg is not None else 3.0
                    )
                    self._twitch_reverse_n = n
                    self._twitch_budget_left = max(0, self._twitch_budget_left - 1)
                    self._twitch_until = now + dur
                    self._twitch_cooldown_until = now + cooldown
                    self._twitch_count += 1
                    self._twitch_blocked_reason = None
                    _logger.warning(
                        "ARMS-UP twitch: reverse %s bytes for %s s",
                        n,
                        dur,
                    )
                else:
                    self._twitch_blocked_reason = reason
            pulse = now < self._twitch_until
            if pulse:
                reason = self._twitch_continue_block_locked(left, right, bt, cfg)
                if reason is not None:
                    self._twitch_until = 0.0
                    self._twitch_active = False
                    self._twitch_cancel_reason = reason
                    pulse = False
                    _logger.warning("ARMS-UP twitch cancelled (reason %s)", reason)
                else:
                    byte = CENTER_OUTPUT_VALUE - int(self._twitch_reverse_n)
                    byte = max(MIN_OUTPUT, min(MAX_OUTPUT, byte))
                    left = right = byte
            self._twitch_active = pulse
        return left, right

    def _note_twitch_still(self, now: float, left: int, right: int) -> None:
        """Track how long the final emitted bytes have stayed at neutral.

        Disarm and any non-neutral byte clear it. The RC-stale and
        calibration returns clear it themselves before they return.
        """
        if (
            not self._safety_state.is_armed
            or int(left) != CENTER_OUTPUT_VALUE
            or int(right) != CENTER_OUTPUT_VALUE
        ):
            self._twitch_still_since = None
            return
        if self._twitch_still_since is None:
            self._twitch_still_since = now

    def _arms_up_telemetry(self, now: float) -> dict:
        sample = self._pose_sample
        status = self._pose_status or {}
        min_vis = None
        l_wrist_y = r_wrist_y = l_shoulder_y = r_shoulder_y = None
        if sample is not None:
            l_wrist_y = sample.l_wrist.y
            r_wrist_y = sample.r_wrist.y
            l_shoulder_y = sample.l_shoulder.y
            r_shoulder_y = sample.r_shoulder.y
            min_vis = min(
                sample.l_wrist.visibility,
                sample.r_wrist.visibility,
                sample.l_shoulder.visibility,
                sample.r_shoulder.visibility,
            )
        return {
            "raw": self._arms_up.raw,
            "active": self._arms_up.active,
            "streak_s": self._arms_up.streak_s,
            "sample_age_s": self._arms_up.last_sample_age_s,
            "l_wrist_y": l_wrist_y,
            "r_wrist_y": r_wrist_y,
            "l_shoulder_y": l_shoulder_y,
            "r_shoulder_y": r_shoulder_y,
            "min_visibility_seen": min_vis,
            "twitch_active": self._twitch_active,
            "twitch_count": self._twitch_count,
            "twitch_blocked_reason": self._twitch_blocked_reason,
            "twitch_cancel_reason": self._twitch_cancel_reason,
            "test_latched": self._twitch_test_latched,
            "test_budget_left": int(self._twitch_budget_left),
            "test_expires_in_s": (
                max(0.0, self._twitch_expires_at - now)
                if self._twitch_test_latched else None
            ),
            "still_s": (
                0.0 if self._twitch_still_since is None
                else max(0.0, now - self._twitch_still_since)
            ),
            "crop_y0": sample.crop_y0 if sample is not None else None,
            "crop_y1": sample.crop_y1 if sample is not None else None,
            "pose_ms": status.get("pose_ms"),
            "pose_hz": status.get("pose_hz"),
            "pose_enabled": status.get("pose_enabled"),
            "mp_pose_loaded": status.get("mp_pose_loaded"),
        }

    def process(
        self,
        rc: RCInputs,
        now_epoch_s: float | None = None,
        bt_override_bytes: tuple[int, int] | None = None,
    ) -> Tuple[DriveCommand, List[SafetyEvent], dict]:
        epoch_now = now_epoch_s if now_epoch_s is not None else time.time()
        mono_now = time.monotonic()
        was_armed = self._safety_state.is_armed

        # ── Poll VESC telemetry every 50 ms ──────────────────────────────────
        _vesc_cfg = getattr(config, "vesc", None)
        _telem_enabled = bool(getattr(_vesc_cfg, "vesc_telemetry_enabled", True))
        if _telem_enabled and (mono_now - self._telem_last_poll >= 0.05):
            try:
                _telem = self._motor.get_telemetry()
            except Exception:
                _telem = None
            try:
                _rx_health_fn = getattr(self._motor, "get_rx_health", None)
                _rx_health = _rx_health_fn() if callable(_rx_health_fn) else None
            except Exception:
                _rx_health = None
            self._telem_last_poll = mono_now
            if _telem is not None:
                self._telem_last_valid = mono_now
                self._actual_left_rpm = _telem.left_rpm
                self._actual_right_rpm = _telem.right_rpm
                self._actual_left_current_a = getattr(_telem, "left_current_a", None)
                self._actual_right_current_a = getattr(_telem, "right_current_a", None)
                self._actual_left_temp_c = getattr(_telem, "left_temp_c", None)
                self._actual_right_temp_c = getattr(_telem, "right_temp_c", None)
                self._actual_left_motor_temp_c = getattr(_telem, "left_motor_temp_c", None)
                self._actual_right_motor_temp_c = getattr(_telem, "right_motor_temp_c", None)
                self._actual_left_duty = getattr(_telem, "left_duty_cycle", None)
                self._actual_right_duty = getattr(_telem, "right_duty_cycle", None)
                # Convert the SIGNED mean eRPM to forward wheel speed (m/s). The
                # signed mean cancels the steering differential (left = v + yaw,
                # right = v − yaw), so a turn does not read as overspeed to the
                # velocity loop. An average of |eRPM| reads max(|v|, |yaw|).
                # Both sides report positive eRPM driving forward (bench
                # 2026-06-11, tools/vesc_rpm_bench.py).
                _lr, _rr = _telem.left_rpm, _telem.right_rpm
                _valid_rpms = [r for r in (_lr, _rr) if r is not None]
                if _valid_rpms:
                    _avg_erpm = sum(_valid_rpms) / len(_valid_rpms)
                    # Same kinematics as FollowMeConfig.speed_loop_mps_per_byte
                    # (pi_app/control/rpm_plausibility.py) — one scale for
                    # feedback and command, or the velocity loop carries a bias.
                    self._actual_speed_mps = erpm_to_wheel_mps(
                        _avg_erpm,
                        motor_poles=int(getattr(_vesc_cfg, "motor_poles", 14)),
                        drive_gear_ratio=float(getattr(_vesc_cfg, "drive_gear_ratio", 1.0)),
                        wheel_radius_m=float(getattr(_vesc_cfg, "wheel_radius_m", 0.085)),
                    )
                else:
                    self._actual_speed_mps = None
                self._vesc_rx_frame_count = int(getattr(_telem, "can_rx_frame_count", self._vesc_rx_frame_count))
                self._vesc_rx_parse_error_count = int(
                    getattr(_telem, "can_rx_parse_error_count", self._vesc_rx_parse_error_count)
                )
                self._vesc_rx_recv_error_count = int(
                    getattr(_telem, "can_rx_recv_error_count", self._vesc_rx_recv_error_count)
                )
                self._vesc_rx_reopen_count = int(
                    getattr(_telem, "can_rx_reopen_count", self._vesc_rx_reopen_count)
                )
                self._vesc_rx_last_frame_age_s = getattr(
                    _telem, "can_rx_last_frame_age_s", self._vesc_rx_last_frame_age_s
                )
                self._vesc_pack_low_latched = bool(
                    getattr(_telem, "pack_low_latched", False)
                )

                # ── Per-motor staleness fallback ─────────────────────────────
                # get_telemetry() returns non-None as long as EITHER motor has
                # ever reported an RPM frame — its per-motor rpm fields are
                # never reset, so a dead RX thread or one silent VESC would
                # otherwise freeze stale numbers that are indistinguishable
                # from fresh ones. left/right_status_age_s (per-motor STATUS(9)
                # frame age) is the real freshness signal; >0.5s on either
                # motor means that motor's RPM cannot be trusted, so both are
                # cleared to force follow_me/slip back to open-loop.
                _l_age = getattr(_telem, "left_status_age_s", None)
                _r_age = getattr(_telem, "right_status_age_s", None)
                _stale_ages = [a for a in (_l_age, _r_age) if a is not None]
                _rx_alive = getattr(_telem, "rx_thread_alive", None)
                # Surface these for the /debug board (read-only diagnostics).
                self._vesc_left_status_age_s = _l_age
                self._vesc_right_status_age_s = _r_age
                self._vesc_rx_thread_alive = _rx_alive
                _motor_stale = any(a > 0.5 for a in _stale_ages)
                _rx_dead = _rx_alive is False
                if _motor_stale or _rx_dead:
                    if not self._telem_stale_warned:
                        if _rx_dead:
                            _logger.warning(
                                "VESC CAN RX thread is dead — falling back to open-loop"
                            )
                        else:
                            _logger.warning(
                                "VESC telemetry stale for >500 ms "
                                "(left_age=%s right_age=%s) — falling back to open-loop",
                                _l_age, _r_age,
                            )
                        self._telem_stale_warned = True
                    self._actual_left_rpm = None
                    self._actual_right_rpm = None
                    self._actual_speed_mps = None
                    self._actual_left_current_a = None
                    self._actual_right_current_a = None
                    self._actual_left_temp_c = None
                    self._actual_right_temp_c = None
                    self._actual_left_motor_temp_c = None
                    self._actual_right_motor_temp_c = None
                    self._actual_left_duty = None
                    self._actual_right_duty = None
                else:
                    self._telem_stale_warned = False

                # ── RPM plausibility gate ────────────────────────────────────
                # Compare the bytes we most recently EMITTED (slew output of the
                # previous tick) with the eRPM just read back. Commanded to move
                # but reading ~0 for rpm_plausibility_window_s → the readback is
                # dead or the wheel is stalled; either way the velocity PID must
                # not chase it. Null the closed-loop inputs (same open-loop
                # fallback as stale telemetry) until real RPM returns.
                if self._rpm_gate is not None:
                    _plausible = self._rpm_gate.update(
                        mono_now,
                        left_cmd_byte=int(self._slew_last_left),
                        right_cmd_byte=int(self._slew_last_right),
                        left_rpm=self._actual_left_rpm,
                        right_rpm=self._actual_right_rpm,
                    )
                    self._vesc_rpm_plausible = _plausible
                    if not _plausible:
                        if not self._rpm_gate_warned:
                            _logger.warning(
                                "VESC RPM implausible: commanded L=%d R=%d bytes but eRPM "
                                "L=%s R=%s for >%.1fs (motors=%s) — treating telemetry as "
                                "invalid, falling back to open-loop",
                                int(self._slew_last_left), int(self._slew_last_right),
                                self._actual_left_rpm, self._actual_right_rpm,
                                self._rpm_gate.window_s,
                                ",".join(self._rpm_gate.tripped_motors),
                            )
                            self._rpm_gate_warned = True
                        self._actual_left_rpm = None
                        self._actual_right_rpm = None
                        self._actual_speed_mps = None
                    elif self._rpm_gate_warned:
                        _logger.warning("VESC RPM plausible again — closed-loop re-enabled")
                        self._rpm_gate_warned = False
            elif (self._telem_last_valid > 0.0
                  and (mono_now - self._telem_last_valid) > 0.5
                  and not self._telem_stale_warned):
                _logger.warning(
                    "VESC telemetry stale for >500 ms — falling back to open-loop"
                )
                self._telem_stale_warned = True
                self._actual_left_rpm = None
                self._actual_right_rpm = None
                self._actual_speed_mps = None
                self._actual_left_current_a = None
                self._actual_right_current_a = None
                self._actual_left_temp_c = None
                self._actual_right_temp_c = None
                self._actual_left_motor_temp_c = None
                self._actual_right_motor_temp_c = None
            if isinstance(_rx_health, dict):
                self._vesc_rx_frame_count = int(_rx_health.get("rx_frame_count", self._vesc_rx_frame_count))
                self._vesc_rx_parse_error_count = int(
                    _rx_health.get("rx_parse_error_count", self._vesc_rx_parse_error_count)
                )
                self._vesc_rx_recv_error_count = int(
                    _rx_health.get("rx_recv_error_count", self._vesc_rx_recv_error_count)
                )
                self._vesc_rx_reopen_count = int(_rx_health.get("rx_reopen_count", self._vesc_rx_reopen_count))
                _rx_age = _rx_health.get("rx_last_frame_age_s", self._vesc_rx_last_frame_age_s)
                self._vesc_rx_last_frame_age_s = (
                    float(_rx_age) if isinstance(_rx_age, (int, float)) else None
                )
                if "rx_thread_alive" in _rx_health:
                    self._vesc_rx_thread_alive = _rx_health.get("rx_thread_alive")

        # Keep IMU state and telemetry current even when RC staleness returns
        # early below. Producer-side yaw already integrates independently;
        # this consumes its cumulative delta without double-integrating.
        if self._imu_compensator is not None:
            self._imu_compensator.get_heading_deg()

        # RC staleness watchdog: if no RC update for >1s, force disarm
        rc_age = epoch_now - rc.last_update_epoch_s if rc.last_update_epoch_s > 0.0 else 0.0
        if rc_age > RC_STALE_TIMEOUT_S:
            self._motor.stop()
            self._reset_slew_state(mono_now)
            self._relay.set_armed(False)
            self._safety_state = SafetyState(
                is_armed=False,
                last_transition_epoch_s=epoch_now,
                emergency_active=self._safety_state.emergency_active,
            )
            self._mode = "MANUAL"
            if was_armed:
                self._on_armed_session_ended()
            cmd = DriveCommand(
                left_byte=CENTER_OUTPUT_VALUE,
                right_byte=CENTER_OUTPUT_VALUE,
                is_armed=False,
                emergency_active=self._safety_state.emergency_active,
            )
            self._last_follow_me_emitted_forward_byte = None
            self._twitch_early_exit(mono_now, "rc_stale")
            return cmd, [SafetyEvent.RC_STALE], {"mode": "MANUAL", "rc_stale": True, "rc_age_s": rc_age}

        # Update safety. This now runs on EVERY tick before the calibration
        # early-return below, so RC-stale disarm, ch3 disarm, and ch5 e-stop
        # take effect even while the calibration wizard is driving the motors.
        self._safety_state, events = update_safety(
            self._safety_state,
            ch3_us=rc.ch3_us,
            ch4_us=rc.ch4_us,
            ch5_us=rc.ch5_us,
            now_epoch_s=epoch_now,
            params=self._safety_params,
        )
        if was_armed and not self._safety_state.is_armed:
            self._on_armed_session_ended()

        # Initialize telemetry before any event handler can write to it.
        # (Previously this dict was created further down, so the
        # FOLLOW_ME_ENTERED-with-no-target branch raised UnboundLocalError.)
        # charger_inhibit is always present (not just on engage) so consumers
        # (run log, SSE, FM trial JSONL) can see the RELEASED state too —
        # previously it only appeared in the dict while inhibiting, so nothing
        # downstream could ever log/see the transition (2026-06-13 field bug).
        telemetry: dict = {"charger_inhibit": self._charger_inhibit}

        # Calibration early-return — now AFTER update_safety so safety is
        # always enforced. The wizard issues drive commands directly, so
        # process() outputs neutral here, but we still cut the motors and
        # mirror the emergency/disarm behavior when safety demands it.
        if self._calibration_mode:
            if any(e is SafetyEvent.EMERGENCY_TRIGGERED for e in events):
                self._shutdown.schedule_shutdown(delay_seconds=5.0)
            if (not self._safety_state.is_armed) or self._safety_state.emergency_active:
                self._motor.stop()
            self._relay.set_armed(self._safety_state.is_armed)
            cmd = DriveCommand(
                left_byte=CENTER_OUTPUT_VALUE,
                right_byte=CENTER_OUTPUT_VALUE,
                is_armed=self._safety_state.is_armed,
                emergency_active=self._safety_state.emergency_active,
            )
            self._last_follow_me_emitted_forward_byte = None
            self._twitch_early_exit(mono_now, "calibration")
            return cmd, events, {
                "mode": "CALIBRATING",
                "calibration": True,
                "is_armed": self._safety_state.is_armed,
                "emergency_active": self._safety_state.emergency_active,
            }

        # React to mode transitions from safety events
        for ev in events:
            if ev is SafetyEvent.FOLLOW_ME_ENTERED:
                if self._follow_me_target_present():
                    self._mode = "FOLLOW_ME"
                    if self._follow_me is not None:
                        self._follow_me.start_recorder()
                else:
                    # New engagement rule: don't enter Follow Me unless a
                    # target is already present.
                    self._mode = "MANUAL"
                    self._safety_state.set_follow_me_active(False)
                    telemetry["follow_me_activation_blocked"] = "no_target"
            elif ev in (SafetyEvent.FOLLOW_ME_EXITED, SafetyEvent.EMERGENCY_TRIGGERED):
                if self._follow_me is not None:
                    self._follow_me.stop_recorder()
                self._mode = "MANUAL"
                if self._gesture is not None:
                    self._gesture.notify_external_deactivation()
        # Disarm exits waypoint nav
        if not self._safety_state.is_armed and self._mode == "WAYPOINT_NAV":
            self._mode = "MANUAL"

        # Hand-gesture Follow Me activation/deactivation
        gesture_event: GestureEvent | None = None
        gesture_status: dict | None = None
        if self._gesture is not None:
            gesture_event = self._gesture.update(self._hand_data)
            gesture_status = self._gesture.get_status()
            if gesture_event is GestureEvent.ACTIVATE:
                if not self._safety_state.is_armed:
                    gesture_event = None  # cannot activate
                    gesture_status["event"] = None
                    gesture_status["event_reason"] = "blocked_disarmed"
                elif (
                    self._follow_me is None
                    or not self._follow_me_target_present()
                ):
                    gesture_event = None  # cannot activate
                    gesture_status["event"] = None
                    gesture_status["event_reason"] = "blocked_no_target"
                else:
                    self._mode = "FOLLOW_ME"
                    self._safety_state.set_follow_me_active(True)
                    self._follow_me.start_recorder()
            elif gesture_event is GestureEvent.DEACTIVATE:
                if self._mode == "FOLLOW_ME":
                    self._mode = "MANUAL"
                    self._safety_state.set_follow_me_active(False)
                    if self._follow_me is not None:
                        self._follow_me.stop_recorder()

        # Emergency triggered: stop motors, disarm, schedule shutdown
        if any(e is SafetyEvent.EMERGENCY_TRIGGERED for e in events):
            self._motor.stop()
            self._relay.set_armed(False)
            self._shutdown.schedule_shutdown(delay_seconds=5.0)

        # Command computation
        telemetry["mode"] = self._mode
        self._maintain_twitch_latch(mono_now)
        self._update_arms_up_twitch(mono_now)
        if self._gesture is not None:
            telemetry["gesture_phase"] = self._gesture.phase_name
            if gesture_event is not None:
                telemetry["gesture_event"] = gesture_event.name
            if gesture_status is None:
                gesture_status = self._gesture.get_status()
            gesture_status["hand_poll_enabled"] = self.hand_poll_wanted(
                self._safety_state.is_armed
            )
            telemetry["gesture"] = gesture_status

        wp_pivot_active = False
        wp_in_align = False
        autonomy_cmd: AutonomyCommand | None = None
        if self._mode == "WAYPOINT_NAV" and self._waypoint_nav is not None:
            autonomy_cmd, wp_pivot_active, wp_in_align = self._compute_waypoint_autonomy_command(telemetry)
            left = autonomy_cmd.left_byte
            right = autonomy_cmd.right_byte
            steering_input = autonomy_cmd.steering_input
        elif self._mode == "FOLLOW_ME" and self._follow_me is not None:
            autonomy_cmd = self._compute_follow_me_autonomy_command(telemetry, mono_now)
            left = autonomy_cmd.left_byte
            right = autonomy_cmd.right_byte
            steering_input = autonomy_cmd.steering_input
        elif bt_override_bytes is not None:
            left, right = bt_override_bytes
            steering_input = self._bytes_to_steering_input(left, right)
        else:
            # Tank-drive: ch1 is left throttle, ch2 is right throttle
            try:
                f_full = int(getattr(config.rc_map, 'forward_full_us', 1950))
                r_full = int(getattr(config.rc_map, 'reverse_full_us', 1050))
            except Exception:
                f_full, r_full = 1950, 1050

            left = map_pulse_to_byte_saturated(rc.ch1_us, f_full, r_full)
            right = map_pulse_to_byte_saturated(rc.ch2_us, f_full, r_full)
            steering_input = self._bytes_to_steering_input(left, right)

        telemetry["autonomy_source"] = autonomy_cmd.source if autonomy_cmd is not None else "MANUAL_OR_BT"

        # Apply IMU steering compensation — skip in Follow Me mode where the
        # controller intentionally changes heading to track a person.
        if self._mode == "FOLLOW_ME":
            imu_correction = None
            if self._imu_compensator is not None:
                self._imu_compensator.reset_target_heading()
        elif self._mode == "MANUAL" and bt_override_bytes is not None:
            # Web/BT teleop should follow operator wheel commands exactly.
            # Disable IMU auto-steer in this path to prevent left/right bias
            # from heading-hold corrections during straight W/S holds.
            imu_correction = None
            if self._imu_compensator is not None:
                self._imu_compensator.reset_target_heading()
        else:
            imu_correction = self._apply_imu_compensation(steering_input, mono_now)
        if wp_in_align:
            # ALIGN state drives a fixed pivot yaw directly; don't stack an
            # IMU correction on top. PID state still advances for the DRIVE
            # transition.
            imu_correction = None
        telemetry["steering_input"] = steering_input
        telemetry["imu_correction_raw"] = imu_correction
        telemetry["wp_in_align"] = wp_in_align

        # Expose PID debug fields from IMU compensator
        if self._imu_compensator is not None:
            try:
                status = self._imu_compensator.get_status()
                telemetry["pid_error_deg"] = status.pid_error_deg
                telemetry["pid_p"] = status.pid_p
                telemetry["pid_i"] = status.pid_i
                telemetry["pid_d"] = status.pid_d
                telemetry["pid_correction"] = status.pid_correction
            except Exception:
                pass
        
        # Straight-intent gating for dual-throttle skid steer
        tol = getattr(config.imu_steering, 'straight_equal_tolerance_us', 20)
        min_th = getattr(config.imu_steering, 'straight_min_throttle_us', 80)
        rel_pct = getattr(config.imu_steering, 'straight_relative_tolerance_pct', 0.15)
        hysteresis_s = getattr(config.imu_steering, 'straight_disengage_hysteresis_s', 0.0)

        # Derive moving_ok from the appropriate source. Heading lock uses the
        # separate manual_rc_forward_intent flag below; corrected motor output
        # and equal reverse RC commands must never qualify it.
        manual_rc_forward_intent = False
        if self._mode in ("FOLLOW_ME", "WAYPOINT_NAV"):
            left_diff = abs(left - CENTER_OUTPUT_VALUE)
            right_diff = abs(right - CENTER_OUTPUT_VALUE)
            moving_ok = max(left_diff, right_diff) >= 4
            # Allow IMU correction to pivot the robot when waypoint nav has
            # gated forward speed to 0 for a large heading error.
            if wp_pivot_active:
                moving_ok = True
        elif bt_override_bytes is not None:
            bt_left, bt_right = bt_override_bytes
            bt_left_diff = abs(bt_left - CENTER_OUTPUT_VALUE)
            bt_right_diff = abs(bt_right - CENTER_OUTPUT_VALUE)
            moving_ok = max(bt_left_diff, bt_right_diff) >= 20
        else:
            d1 = rc.ch1_us - 1500
            d2 = rc.ch2_us - 1500
            moving_ok = max(abs(d1), abs(d2)) >= min_th
            manual_rc_forward_intent = d1 >= min_th and d2 >= min_th

        # Determine equal_ok based on input source
        if self._mode in ("FOLLOW_ME", "WAYPOINT_NAV"):
            abs_diff = abs(left - right)
            equal_abs_ok = abs_diff <= 10
            max_abs = max(abs(left - CENTER_OUTPUT_VALUE), abs(right - CENTER_OUTPUT_VALUE), 1)
            equal_rel_ok = (abs_diff / max_abs) <= rel_pct
        elif bt_override_bytes is not None:
            bt_left, bt_right = bt_override_bytes
            abs_diff = abs(bt_left - bt_right)
            equal_abs_ok = abs_diff <= 10
            max_abs = max(abs(bt_left - CENTER_OUTPUT_VALUE), abs(bt_right - CENTER_OUTPUT_VALUE), 1)
            equal_rel_ok = (abs_diff / max_abs) <= rel_pct
        else:
            abs_diff = abs(rc.ch1_us - rc.ch2_us)
            equal_abs_ok = abs_diff <= tol
            # Relative check (difference relative to magnitude)
            max_abs = max(abs(d1), abs(d2), 1)
            equal_rel_ok = (abs_diff / max_abs) <= rel_pct
        equal_ok = equal_abs_ok or (moving_ok and equal_rel_ok)
        # Manual/BT: low steering intent should still count as straight command
        # even when raw track values are slightly mismatched.
        if self._mode == "MANUAL":
            hold_steer = float(getattr(config.imu_steering, "manual_hold_max_steering", 0.18))
            if moving_ok and abs(steering_input) <= hold_steer:
                equal_ok = True
        now_s = mono_now
        is_moving_straight = False
        if moving_ok and equal_ok:
            is_moving_straight = True
            self._straight_latched = True
            self._straight_disengage_deadline = now_s + hysteresis_s
        else:
            if self._straight_latched and now_s <= self._straight_disengage_deadline:
                is_moving_straight = True
            else:
                self._straight_latched = False
        if (
            self._imu_compensator is not None
            and is_moving_straight
            and self._mode == "MANUAL"
        ):
            try:
                imu_state = self._imu_compensator.get_status()
                raw_heading = float(imu_state.heading_deg)
                if self._gps_heading_aligner is not None and self._gps_heading_aligner.locked:
                    # Keep a true-frame target while driving straight, mapped
                    # through the frozen GPS-derived offset.
                    # In MANUAL BT/web teleop, prioritize operator intent and
                    # lock heading directly in the raw IMU frame.
                    allow_true_frame_retarget = not (
                        self._mode == "MANUAL" and bt_override_bytes is not None
                    )
                    if allow_true_frame_retarget:
                        if (not self._was_moving_straight) or self._straight_target_true_heading is None:
                            self._straight_target_true_heading = self._gps_heading_aligner.correct(raw_heading)
                        raw_target = self._gps_heading_aligner.imu_target_heading(self._straight_target_true_heading)
                        self._imu_compensator.set_target_heading(raw_target, reset_integral_jump_deg=180.0)
                    elif not self._was_moving_straight:
                        # Lock heading in raw IMU frame at straight-drive entry.
                        self._straight_target_true_heading = None
                        self._imu_compensator.set_target_heading(raw_heading, reset_integral_jump_deg=180.0)
                elif not self._was_moving_straight:
                    self._straight_target_true_heading = None
                    self._imu_compensator.reset_target_heading()
            except Exception:
                pass
        elif not is_moving_straight or self._mode != "MANUAL":
            self._straight_target_true_heading = None
        self._was_moving_straight = is_moving_straight

        self._check_imu_frame_continuity(mono_now)
        if (
            self._gps_heading_aligner is not None
            and self._gps_reading is not None
            and self._imu_compensator is not None
        ):
            try:
                align_imu = self._imu_compensator.get_status()
                self._gps_heading_aligner.update(
                    self._gps_reading.latitude,
                    self._gps_reading.longitude,
                    float(align_imu.heading_deg),
                    self._gps_reading.fix_quality,
                    self._gps_reading.timestamp,
                    lock_allowed=(
                        self._mode == "MANUAL"
                        and bt_override_bytes is None
                        and is_moving_straight
                        and manual_rc_forward_intent
                    ),
                    yaw_rate_dps=float(align_imu.yaw_rate_dps),
                )
                # Per-epoch course-over-ground lock: source-agnostic (RC or
                # web/BT teleop) and tolerant of a wobbly path. Forward intent
                # comes from the bytes last emitted to the motors, so a reverse
                # run (course 180 deg from heading) can never contribute.
                _rd = self._gps_reading
                _fwd = (
                    int(self._slew_last_left) > CENTER_OUTPUT_VALUE + 6
                    and int(self._slew_last_right) > CENTER_OUTPUT_VALUE + 6
                )
                self._gps_heading_aligner.update_cog(
                    float(align_imu.heading_deg),
                    getattr(_rd, "cog_deg", None),
                    getattr(_rd, "sog_mps", None),
                    _rd.fix_quality,
                    _rd.timestamp,
                    forward_intent=(self._mode == "MANUAL" and _fwd),
                    yaw_rate_dps=float(align_imu.yaw_rate_dps),
                )
            except Exception:
                pass

        # Apply correction to motor outputs continuously with steering-scaled blending
        corr_applied = None
        if imu_correction is not None:
            # Blend factor reduces correction as steering_input magnitude increases
            zero_at = float(getattr(config.imu_steering, 'correction_zero_at_steering', 0.5))
            si = max(0.0, min(1.0, abs(steering_input)))
            if zero_at <= 0.0:
                blend = 0.0
            else:
                blend = max(0.0, 1.0 - (si / zero_at))
            telemetry["correction_blend"] = blend
            corr = imu_correction * blend

            if self._mode == "MANUAL" and bt_override_bytes is not None:
                bt_max_corr = float(getattr(config.imu_steering, "manual_bt_max_correction", 12.0))
                corr = max(-bt_max_corr, min(bt_max_corr, corr))

            # Speed-dependent gain scheduling: attenuate correction at high
            # wheel speed where each byte produces more turning force.
            speed_scale = 1.0
            gs_enabled = bool(getattr(config.imu_steering, 'gain_schedule_enabled', False))
            if gs_enabled and moving_ok:
                ref = float(getattr(config.imu_steering, 'gain_schedule_ref_speed_byte', 50.0))
                speed_byte = max(abs(left - CENTER_OUTPUT_VALUE),
                                 abs(right - CENTER_OUTPUT_VALUE))
                if ref > 0.0 and speed_byte > ref:
                    speed_scale = ref / speed_byte
                corr = corr * speed_scale
            telemetry["speed_gain_scale"] = speed_scale

            # Apply corrections only when moving to avoid idle spin corrections
            if moving_ok and abs(corr) > 0.0:
                # Apply so that positive correction increases left and decreases right
                left = self._apply_steering_correction(left, corr)
                right = self._apply_steering_correction(right, -corr)
                corr_applied = corr

        # Optional per-side straight bias to cancel residual skew; only when straight intent
        if is_moving_straight:
            try:
                biasL = int(getattr(config.imu_steering, 'straight_bias_left_byte', 0))
                biasR = int(getattr(config.imu_steering, 'straight_bias_right_byte', 0))
                if biasL or biasR:
                    left = max(MIN_OUTPUT, min(MAX_OUTPUT, left + biasL))
                    right = max(MIN_OUTPUT, min(MAX_OUTPUT, right + biasR))
            except Exception:
                pass
        telemetry["imu_correction_applied"] = corr_applied

        # Arms-up bench twitch: replace the mode's L/R with a short equal
        # reverse pulse. Injected HERE — after per-mode command + IMU, before
        # obstacle scaling, disarm, charger inhibit, pack-low, RPM-plausibility
        # (telemetry only), and the slew limiter. Reverse is never gated by
        # the forward obstacle layer (is_forward_motion requires net forward).
        # MANUAL only. A blocked rising edge is consumed on this tick.
        left, right = self._service_twitch_injection(
            mono_now, left, right, bt_override_bytes
        )

        # Obstacle avoidance throttle scaling — front camera only gates forward motion.
        # Reverse commands must not be blocked by front camera detections.
        # "Forward motion" must be true net forward (sum of byte offsets > 0), not just
        # "either track is >neutral" — otherwise a pure pivot (L<126, R>126) is treated as
        # forward motion and scaling toward neutral collapses the pivot asymmetrically into
        # a forward-biased curve (the "left-hand circles" bug).
        obstacle_scale = 1.0
        is_forward_motion = (left + right) > 2 * CENTER_OUTPUT_VALUE
        if self._obstacle_avoidance is not None and self._obstacle_distance_m is not None:
            obstacle_scale = self._obstacle_avoidance.compute_throttle_scale(
                self._obstacle_distance_m,
                self._obstacle_age_s if self._obstacle_age_s is not None else 999.0,
                is_manual=(self._mode == "MANUAL"),
            )
            if is_forward_motion:
                # Scale only the common-mode (forward) component; preserve the
                # differential (yaw) so pivots-during-forward don't distort.
                common = (left + right) / 2.0 - CENTER_OUTPUT_VALUE
                diff = (left - right) / 2.0
                common_scaled = common * obstacle_scale
                left = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(CENTER_OUTPUT_VALUE + common_scaled + diff))))
                right = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(CENTER_OUTPUT_VALUE + common_scaled - diff))))
            oa_status = self._obstacle_avoidance.get_status()
            telemetry.update(oa_status)
        telemetry["obstacle_throttle_scale"] = obstacle_scale
        telemetry["obstacle_distance_m"] = self._obstacle_distance_m
        telemetry["slew_mode"] = self._mode
        telemetry["slew_enabled"] = bool(getattr(getattr(config, "slew_limiter", None), "enabled", False))
        telemetry["slew_bypassed"] = False
        telemetry["slew_hard_stop_active"] = False
        telemetry["slew_in_left"] = int(left)
        telemetry["slew_in_right"] = int(right)
        telemetry["slew_out_left"] = int(left)
        telemetry["slew_out_right"] = int(right)
        telemetry["slew_delta_left"] = 0
        telemetry["slew_delta_right"] = 0

        # If disarmed, force neutral outputs
        if not self._safety_state.is_armed:
            left = right = CENTER_OUTPUT_VALUE
            self._motor.stop()
            self._reset_slew_state(mono_now)
            telemetry["slew_out_left"] = left
            telemetry["slew_out_right"] = right
            telemetry["slew_delta_left"] = left - telemetry["slew_in_left"]
            telemetry["slew_delta_right"] = right - telemetry["slew_in_right"]
        elif self._charger_inhibit:
            # Charger connected (debounced — see bms.is_charging()): refuse all
            # drive commands regardless of RC input. telemetry["charger_inhibit"]
            # is already True here (set unconditionally above from self._charger_inhibit).
            left = right = CENTER_OUTPUT_VALUE
            self._motor.stop()
            self._reset_slew_state(mono_now)
            telemetry["slew_out_left"] = left
            telemetry["slew_out_right"] = right
            telemetry["slew_delta_left"] = left - telemetry["slew_in_left"]
            telemetry["slew_delta_right"] = right - telemetry["slew_in_right"]
        elif self._vesc_pack_low_latched:
            # VESC pack-low latch engaged (see vesc._check_voltage_shutdown):
            # refuse all drive commands. The driver's set_tracks() also forces
            # neutral (belt-and-suspenders), but the controller must observe
            # the latch too — otherwise the slew state keeps ramping toward the
            # commanded value while latched, and the moment the latch clears at
            # ~41V the fully-ramped command would be emitted instantly (full-
            # speed lurch). Resetting slew here makes release ramp from neutral.
            left = right = CENTER_OUTPUT_VALUE
            self._motor.stop()
            self._reset_slew_state(mono_now)
            telemetry["slew_out_left"] = left
            telemetry["slew_out_right"] = right
            telemetry["slew_delta_left"] = left - telemetry["slew_in_left"]
            telemetry["slew_delta_right"] = right - telemetry["slew_in_right"]
        else:
            left_in = int(left)
            right_in = int(right)
            slewc = getattr(config, "slew_limiter", None)
            mode_for_slew = self._mode
            slewc_enabled = bool(getattr(slewc, "enabled", False))
            hard_stop_threshold = float(getattr(slewc, "hard_stop_scale_threshold", 0.0))
            hard_stop_active = bool(self._safety_state.emergency_active) or any(
                e is SafetyEvent.EMERGENCY_TRIGGERED for e in events
            ) or (is_forward_motion and obstacle_scale <= hard_stop_threshold)
            slew_bypassed = False

            if slewc_enabled:
                bypass_on_hard_stop = bool(getattr(slewc, "bypass_on_hard_stop", True))
                # Web BT override: neutral release should be immediate so a
                # brief turn command does not keep turning from slew inertia.
                bt_neutral = (
                    bt_override_bytes is not None
                    and left_in == CENTER_OUTPUT_VALUE
                    and right_in == CENTER_OUTPUT_VALUE
                )
                if bt_neutral:
                    slew_bypassed = True
                    left = left_in
                    right = right_in
                    self._slew_initialized = True
                    self._slew_seen_non_neutral = (
                        left != CENTER_OUTPUT_VALUE or right != CENTER_OUTPUT_VALUE
                    )
                    self._slew_last_update = mono_now
                    self._slew_last_left = left
                    self._slew_last_right = right
                elif hard_stop_active and bypass_on_hard_stop:
                    slew_bypassed = True
                    left = left_in
                    right = right_in
                    self._slew_initialized = True
                    self._slew_seen_non_neutral = (
                        left != CENTER_OUTPUT_VALUE or right != CENTER_OUTPUT_VALUE
                    )
                    self._slew_last_update = mono_now
                    self._slew_last_left = left
                    self._slew_last_right = right
                else:
                    accel_bps, decel_bps = self._slew_rates_for_mode(mode_for_slew)
                    if hard_stop_active and not bypass_on_hard_stop:
                        decel_bps = float(getattr(slewc, "emergency_decel_bps", decel_bps))

                    dt_s = max(0.0, mono_now - self._slew_last_update)
                    snap_first = bool(getattr(slewc, "snap_first_command", True))
                    if mode_for_slew == "FOLLOW_ME":
                        snap_first = bool(getattr(slewc, "snap_first_follow_me", snap_first))
                    wants_motion = (
                        left_in != CENTER_OUTPUT_VALUE or right_in != CENTER_OUTPUT_VALUE
                    )
                    if snap_first and (not self._slew_seen_non_neutral) and wants_motion:
                        left = left_in
                        right = right_in
                    else:
                        max_accel_delta = max(0.0, accel_bps * dt_s)
                        max_decel_delta = max(0.0, decel_bps * dt_s)
                        left = self._slew_toward_target(
                            self._slew_last_left, left_in, max_accel_delta, max_decel_delta
                        )
                        right = self._slew_toward_target(
                            self._slew_last_right, right_in, max_accel_delta, max_decel_delta
                        )
                    self._slew_initialized = True
                    if left != CENTER_OUTPUT_VALUE or right != CENTER_OUTPUT_VALUE:
                        self._slew_seen_non_neutral = True
                    self._slew_last_update = mono_now
                    self._slew_last_left = left
                    self._slew_last_right = right
            else:
                left = left_in
                right = right_in
                self._slew_initialized = True
                self._slew_seen_non_neutral = (
                    left != CENTER_OUTPUT_VALUE or right != CENTER_OUTPUT_VALUE
                )
                self._slew_last_update = mono_now
                self._slew_last_left = left
                self._slew_last_right = right

            if left > MAX_OUTPUT:
                left = MAX_OUTPUT
            if right > MAX_OUTPUT:
                right = MAX_OUTPUT
            self._motor.set_tracks(left, right)
            telemetry["slew_bypassed"] = slew_bypassed
            telemetry["slew_hard_stop_active"] = hard_stop_active
            telemetry["slew_out_left"] = left
            telemetry["slew_out_right"] = right
            telemetry["slew_delta_left"] = left - left_in
            telemetry["slew_delta_right"] = right - right_in

        # Reflect arm relay state
        self._relay.set_armed(self._safety_state.is_armed)

        cmd = DriveCommand(
            left_byte=left,
            right_byte=right,
            is_armed=self._safety_state.is_armed,
            emergency_active=self._safety_state.emergency_active,
        )
        telemetry["motor_left_byte"] = left
        telemetry["motor_right_byte"] = right
        # Common-mode byte that actually reached the motors this tick (after
        # obstacle scaling and the slew limiter). Next FOLLOW_ME tick's
        # velocity PID targets this so it cannot wind up against a limit.
        if self._mode == "FOLLOW_ME":
            self._last_follow_me_emitted_forward_byte = (
                (left + right) / 2.0 - CENTER_OUTPUT_VALUE
            )
        else:
            self._last_follow_me_emitted_forward_byte = None
        telemetry["straight_intent"] = is_moving_straight
        telemetry["rc_equal_tol_us"] = tol
        telemetry["rc_equal_rel_pct"] = rel_pct
        telemetry["straight_latched"] = self._straight_latched
        telemetry["vesc_left_rpm"] = self._actual_left_rpm
        telemetry["vesc_right_rpm"] = self._actual_right_rpm
        telemetry["vesc_actual_speed_mps"] = self._actual_speed_mps
        telemetry["vesc_rpm_plausible"] = self._vesc_rpm_plausible
        telemetry["vesc_rpm_gate_trips"] = (
            self._rpm_gate.trip_count if self._rpm_gate is not None else 0
        )
        telemetry["vesc_left_duty"] = self._actual_left_duty
        telemetry["vesc_right_duty"] = self._actual_right_duty
        telemetry["vesc_rx_frame_count"] = self._vesc_rx_frame_count
        telemetry["vesc_rx_parse_error_count"] = self._vesc_rx_parse_error_count
        telemetry["vesc_rx_recv_error_count"] = self._vesc_rx_recv_error_count
        telemetry["vesc_rx_reopen_count"] = self._vesc_rx_reopen_count
        telemetry["vesc_rx_last_frame_age_s"] = self._vesc_rx_last_frame_age_s
        telemetry["vesc_pack_low_latched"] = self._vesc_pack_low_latched
        telemetry["vesc_left_status_age_s"] = self._vesc_left_status_age_s
        telemetry["vesc_right_status_age_s"] = self._vesc_right_status_age_s
        telemetry["vesc_rx_thread_alive"] = self._vesc_rx_thread_alive
        # MOSFET (FET) + motor winding temps from CAN STATUS_4 (°C); None when
        # VESC telemetry is missing or stale so the debug board can gap-fill.
        telemetry["vesc_left_temp_c"] = self._actual_left_temp_c
        telemetry["vesc_right_temp_c"] = self._actual_right_temp_c
        telemetry["vesc_left_motor_temp_c"] = self._actual_left_motor_temp_c
        telemetry["vesc_right_motor_temp_c"] = self._actual_right_motor_temp_c
        # Age of the current GPS fix, for troubleshooting a stale reading that
        # is still non-None (e.g. the reader stopped updating but the last
        # good reading is still sitting in self._gps_reading).
        telemetry["gps_age_s"] = (
            round(time.monotonic() - self._gps_reading.timestamp, 2)
            if self._gps_reading is not None else None
        )
        raw_heading_telem: Optional[float] = None
        if self._imu_compensator is not None:
            try:
                raw_heading_telem = float(self._imu_compensator.get_heading_deg())
            except Exception:
                raw_heading_telem = None
        heading_align = self._heading_align_telemetry(raw_heading_telem)
        telemetry["heading_align"] = heading_align
        telemetry["heading_offset_deg"] = heading_align["offset_deg"]
        telemetry["heading_offset_locked"] = heading_align["locked"]
        telemetry["heading_offset_frozen"] = heading_align["frozen"]
        telemetry["heading_offset_refining"] = heading_align["refining"]
        telemetry["corrected_heading_deg"] = heading_align["corrected_heading_deg"]
        self._note_twitch_still(mono_now, left, right)
        telemetry["arms_up"] = self._arms_up_telemetry(mono_now)
        return cmd, events, telemetry

    def _bytes_to_steering_input(self, left_byte: int, right_byte: int) -> float:
        """Convert left/right byte values to normalized steering input (-1.0 to 1.0)."""
        left_diff = left_byte - CENTER_OUTPUT_VALUE
        right_diff = right_byte - CENTER_OUTPUT_VALUE
        steering = (right_diff - left_diff) / 2.0 / CENTER_OUTPUT_VALUE
        return max(-1.0, min(1.0, steering))

    def _apply_imu_compensation(self, steering_input: float, now_s: float) -> Optional[float]:
        """Apply IMU steering compensation if available and timing allows."""
        if (self._imu_compensator is None or
            not config.imu_steering.enabled or
            now_s - self._last_imu_update < self._imu_update_interval):
            return None

        try:
            dt = now_s - self._last_imu_update
            correction = self._imu_compensator.update(steering_input, dt)
            self._last_imu_update = now_s
            
            if config.imu_steering.log_steering_corrections and correction is not None:
                print(f"IMU correction: {correction:.2f} (steering: {steering_input:.2f})")
            
            return correction
            
        except Exception as e:
            if config.imu_steering.fallback_on_error:
                print(f"IMU compensation failed, falling back to RC control: {e}")
            return None

    def _apply_steering_correction(self, base_byte: int, correction: float) -> int:
        """Apply steering correction to a motor byte value."""
        corrected = base_byte + int(round(correction))
        return max(MIN_OUTPUT, min(MAX_OUTPUT, corrected))


    def get_vesc_telemetry(self) -> dict:
        """Return current VESC telemetry values for external consumers (e.g. debug CLI)."""
        return {
            "left_rpm": self._actual_left_rpm,
            "right_rpm": self._actual_right_rpm,
            "actual_speed_mps": self._actual_speed_mps,
        }

    def get_imu_status(self) -> Optional[dict]:
        """Get IMU status information for monitoring."""
        if self._imu_compensator is None:
            return None
        
        try:
            status = self._imu_compensator.get_status()
            status_dict = {
                'heading_deg': status.heading_deg,
                'target_heading_deg': status.target_heading_deg,
                'yaw_rate_dps': status.yaw_rate_dps,
                'roll_deg': status.roll_deg,
                'pitch_deg': status.pitch_deg,
                'integral_error': status.integral_error,
                'pid_error_deg': status.pid_error_deg,
                'pid_p': status.pid_p,
                'pid_i': status.pid_i,
                'pid_d': status.pid_d,
                'pid_correction': status.pid_correction,
                'is_available': status.is_available,
                'is_calibrated': status.is_calibrated,
                'error_count': status.error_count,
                'saturated': status.saturated,
            }
            # OakImuReader (and future readers) may expose get_health() with
            # sample-identity / axis / scale diagnostics for field chalk tests.
            reader = getattr(self._imu_compensator, "imu_reader", None)
            health_fn = getattr(reader, "get_health", None) if reader is not None else None
            if callable(health_fn):
                try:
                    health = health_fn()
                    if isinstance(health, dict):
                        status_dict["oak_imu"] = health
                except Exception:
                    pass
            return status_dict
        except Exception:
            return None


