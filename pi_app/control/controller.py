from dataclasses import dataclass
from typing import Protocol, Tuple, List, Optional
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
from pi_app.control.gesture_control import GestureStateMachine, GestureEvent, HandData
from pi_app.control.waypoint_nav import WaypointNavController, NavState, mix_to_bytes
from pi_app.control.gps_heading_align import GpsHeadingAligner
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
        self._straight_disengage_deadline = 0.0
        self._straight_target_true_heading: Optional[float] = None

        # Obstacle avoidance, Follow Me, Waypoint Nav, and Gesture control
        self._obstacle_avoidance = obstacle_avoidance
        self._follow_me = follow_me
        self._waypoint_nav = waypoint_nav
        self._gesture = gesture_controller
        # GPS→IMU heading aligner runs every tick regardless of mode so
        # any consumer can ask for a true-north-referenced heading.
        self._gps_heading_aligner = gps_heading_aligner
        self._mode = "MANUAL"  # "MANUAL", "FOLLOW_ME", or "WAYPOINT_NAV"
        self._obstacle_distance_m: float | None = None
        self._obstacle_age_s: float | None = None
        self._gps_reading: GpsReading | None = None
        self._person_detections: list[PersonDetection] = []
        self._hand_data: HandData | None = None

        # Charger inhibit: set True when BMS reports charging; blocks motor output.
        # Fail-open by design — cleared externally if BMS becomes unreachable.
        self._charger_inhibit: bool = False

        # Calibration mode: when True, process() outputs neutral and skips logic
        self._calibration_mode = False

        # Final-stage slew limiter state.
        self._slew_last_left = CENTER_OUTPUT_VALUE
        self._slew_last_right = CENTER_OUTPUT_VALUE
        self._slew_last_update = time.monotonic()
        self._slew_initialized = False
        self._slew_seen_non_neutral = False

        # VESC telemetry state — polled every 50 ms in process().
        self._telem_last_poll: float = 0.0
        self._telem_last_valid: float = 0.0
        self._telem_stale_warned: bool = False
        self._actual_left_rpm: Optional[int] = None
        self._actual_right_rpm: Optional[int] = None
        self._actual_speed_mps: Optional[float] = None
        self._vesc_rx_frame_count: int = 0
        self._vesc_rx_parse_error_count: int = 0
        self._vesc_rx_recv_error_count: int = 0
        self._vesc_rx_reopen_count: int = 0
        self._vesc_rx_last_frame_age_s: Optional[float] = None

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

    def set_gps_reading(self, reading: GpsReading | None) -> None:
        """Feed latest reading from RtkGpsReader."""
        self._gps_reading = reading

    def set_charger_inhibit(self, inhibit: bool) -> None:
        """Set or clear the charger inhibit flag.

        When True, process() will output neutral and stop the motors regardless
        of RC/BT commands.  Call with False when the BMS is unreachable (fail-open).
        """
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
            return True
        return False

    def deactivate_follow_me(self) -> None:
        """Return to MANUAL mode from Follow Me."""
        if self._mode == "FOLLOW_ME":
            self._mode = "MANUAL"
            self._safety_state.set_follow_me_active(False)
            if self._gesture is not None:
                self._gesture.notify_external_deactivation()

    @property
    def motor_driver(self) -> MotorDriver:
        return self._motor

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

    def activate_waypoint_nav(self) -> None:
        """Enter WAYPOINT_NAV mode (call from UI / CLI)."""
        if self._waypoint_nav is not None and not self._waypoint_nav.completed:
            self._mode = "WAYPOINT_NAV"

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

    def process(
        self,
        rc: RCInputs,
        now_epoch_s: float | None = None,
        bt_override_bytes: tuple[int, int] | None = None,
    ) -> Tuple[DriveCommand, List[SafetyEvent], dict]:
        epoch_now = now_epoch_s if now_epoch_s is not None else time.time()
        mono_now = time.monotonic()

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
                self._telem_stale_warned = False
                self._telem_last_valid = mono_now
                self._actual_left_rpm = _telem.left_rpm
                self._actual_right_rpm = _telem.right_rpm
                # Convert average eRPM to wheel speed (m/s)
                _lr, _rr = _telem.left_rpm, _telem.right_rpm
                _valid_rpms = [abs(r) for r in (_lr, _rr) if r is not None]
                if _valid_rpms:
                    _avg_erpm = sum(_valid_rpms) / len(_valid_rpms)
                    _poles = int(getattr(_vesc_cfg, "motor_poles", 14))
                    _radius = float(getattr(_vesc_cfg, "wheel_radius_m", 0.085))
                    _gear = float(getattr(_vesc_cfg, "drive_gear_ratio", 1.0))
                    _pole_pairs = max(_poles // 2, 1)
                    _mech_rpm = _avg_erpm / max(_pole_pairs * max(_gear, 1e-6), 1e-6)
                    self._actual_speed_mps = (_mech_rpm / 60.0) * 2.0 * math.pi * _radius
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

        # ── GPS heading aligner: runs every tick in every mode ──────────────
        # The aligner self-gates on fix quality, displacement, and speed, so
        # it's safe to call unconditionally. Consumers read the corrected
        # heading via self._get_corrected_heading() or the aligner directly.
        if (
            self._gps_heading_aligner is not None
            and self._gps_reading is not None
            and self._imu_compensator is not None
        ):
            try:
                raw_heading = self._imu_compensator.get_heading_deg()
            except Exception:
                raw_heading = None
            if raw_heading is not None:
                self._gps_heading_aligner.update(
                    self._gps_reading.latitude,
                    self._gps_reading.longitude,
                    raw_heading,
                    self._gps_reading.fix_quality,
                    mono_now,
                )

        if self._calibration_mode:
            cmd = DriveCommand(
                left_byte=CENTER_OUTPUT_VALUE,
                right_byte=CENTER_OUTPUT_VALUE,
                is_armed=self._safety_state.is_armed,
                emergency_active=self._safety_state.emergency_active,
            )
            return cmd, [], {"mode": "CALIBRATING", "calibration": True}

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
            cmd = DriveCommand(
                left_byte=CENTER_OUTPUT_VALUE,
                right_byte=CENTER_OUTPUT_VALUE,
                is_armed=False,
                emergency_active=self._safety_state.emergency_active,
            )
            return cmd, [SafetyEvent.RC_STALE], {"mode": "MANUAL", "rc_stale": True, "rc_age_s": rc_age}

        # Update safety
        self._safety_state, events = update_safety(
            self._safety_state,
            ch3_us=rc.ch3_us,
            ch4_us=rc.ch4_us,
            ch5_us=rc.ch5_us,
            now_epoch_s=epoch_now,
            params=self._safety_params,
        )

        # React to mode transitions from safety events
        for ev in events:
            if ev is SafetyEvent.FOLLOW_ME_ENTERED:
                if self._follow_me_target_present():
                    self._mode = "FOLLOW_ME"
                else:
                    # New engagement rule: don't enter Follow Me unless a
                    # target is already present.
                    self._mode = "MANUAL"
                    self._safety_state.set_follow_me_active(False)
                    telemetry["follow_me_activation_blocked"] = "no_target"
            elif ev in (SafetyEvent.FOLLOW_ME_EXITED, SafetyEvent.EMERGENCY_TRIGGERED):
                self._mode = "MANUAL"
                if self._gesture is not None:
                    self._gesture.notify_external_deactivation()
        # Disarm exits waypoint nav
        if not self._safety_state.is_armed and self._mode == "WAYPOINT_NAV":
            self._mode = "MANUAL"

        # Hand-gesture Follow Me activation/deactivation
        gesture_event: GestureEvent | None = None
        if self._gesture is not None:
            gesture_event = self._gesture.update(self._hand_data)
            if gesture_event is GestureEvent.ACTIVATE:
                if (
                    self._follow_me is not None
                    and self._safety_state.is_armed
                    and self._follow_me_target_present()
                ):
                    self._mode = "FOLLOW_ME"
                    self._safety_state.set_follow_me_active(True)
                else:
                    gesture_event = None  # cannot activate
            elif gesture_event is GestureEvent.DEACTIVATE:
                if self._mode == "FOLLOW_ME":
                    self._mode = "MANUAL"
                    self._safety_state.set_follow_me_active(False)

        # Emergency triggered: stop motors, disarm, schedule shutdown
        if any(e is SafetyEvent.EMERGENCY_TRIGGERED for e in events):
            self._motor.stop()
            self._relay.set_armed(False)
            self._shutdown.schedule_shutdown(delay_seconds=5.0)

        # Command computation
        telemetry: dict = {"mode": self._mode}
        if self._gesture is not None:
            telemetry["gesture_phase"] = self._gesture.phase_name
            if gesture_event is not None:
                telemetry["gesture_event"] = gesture_event.name

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

        # Derive moving_ok from the appropriate source
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
        if self._imu_compensator is not None and is_moving_straight:
            try:
                imu_state = self._imu_compensator.get_status()
                raw_heading = float(imu_state.heading_deg)
                if self._gps_heading_aligner is not None and self._gps_heading_aligner.locked:
                    # Keep a true-frame target while driving straight. As GPS
                    # heading offset refines, remap target back into raw-IMU frame
                    # so heading correction stays seamless.
                    # In MANUAL BT/web teleop, prioritize operator intent and
                    # avoid live GPS-offset retargeting, which can inject turns
                    # during straight "W" holds if offset estimation jumps.
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
        elif not is_moving_straight:
            self._straight_target_true_heading = None
        self._was_moving_straight = is_moving_straight

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
            # Charger connected: refuse all drive commands regardless of RC input.
            left = right = CENTER_OUTPUT_VALUE
            self._motor.stop()
            self._reset_slew_state(mono_now)
            telemetry["charger_inhibit"] = True
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
        telemetry["straight_intent"] = is_moving_straight
        telemetry["rc_equal_tol_us"] = tol
        telemetry["rc_equal_rel_pct"] = rel_pct
        telemetry["straight_latched"] = self._straight_latched
        telemetry["vesc_left_rpm"] = self._actual_left_rpm
        telemetry["vesc_right_rpm"] = self._actual_right_rpm
        telemetry["vesc_actual_speed_mps"] = self._actual_speed_mps
        telemetry["vesc_rx_frame_count"] = self._vesc_rx_frame_count
        telemetry["vesc_rx_parse_error_count"] = self._vesc_rx_parse_error_count
        telemetry["vesc_rx_recv_error_count"] = self._vesc_rx_recv_error_count
        telemetry["vesc_rx_reopen_count"] = self._vesc_rx_reopen_count
        telemetry["vesc_rx_last_frame_age_s"] = self._vesc_rx_last_frame_age_s
        if self._gps_heading_aligner is not None:
            telemetry["heading_offset_deg"] = self._gps_heading_aligner.offset_deg
            telemetry["heading_offset_locked"] = self._gps_heading_aligner.locked
        else:
            telemetry["heading_offset_deg"] = 0.0
            telemetry["heading_offset_locked"] = False
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
            }
            return status_dict
        except Exception:
            return None


