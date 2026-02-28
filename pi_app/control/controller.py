from dataclasses import dataclass
from typing import Protocol, Tuple, List, Optional
import time
import threading
import sys
from pathlib import Path

# Add parent directory to path for config import
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.control.mapping import (
    map_pulse_to_byte, map_pulse_to_byte_saturated,
    CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT,
)
from pi_app.control.safety import update_safety, SafetyState, SafetyParams, SafetyEvent
from pi_app.control.state import DriveCommand
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.waypoint_nav import WaypointNavController
from pi_app.hardware.rtk_gps import GpsReading
from config import config


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


class ArmRelay(Protocol):
    def set_armed(self, armed: bool) -> None: ...


class ShutdownScheduler(Protocol):
    def schedule_shutdown(self, delay_seconds: float) -> None: ...


class NoopMotorDriver:
    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        pass

    def stop(self) -> None:
        pass


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
            rate_hz = getattr(config.imu_steering, "update_rate_hz", 0)
            if rate_hz <= 0:
                raise ValueError("config.imu_steering.update_rate_hz must be positive")
            self._imu_update_interval = 1.0 / rate_hz
        else:
            self._imu_update_interval = 1.0
        # Track when we begin moving straight to (re)lock heading
        self._was_moving_straight = False
        self._straight_latched = False
        self._straight_disengage_deadline = 0.0

        # Obstacle avoidance, Follow Me, and Waypoint Nav
        self._obstacle_avoidance = obstacle_avoidance
        self._follow_me = follow_me
        self._waypoint_nav = waypoint_nav
        self._mode = "MANUAL"  # "MANUAL", "FOLLOW_ME", or "WAYPOINT_NAV"
        self._obstacle_distance_m: float | None = None
        self._obstacle_age_s: float | None = None
        self._gps_reading: GpsReading | None = None
        self._person_detections: list[PersonDetection] = []

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

    def set_gps_reading(self, reading: GpsReading | None) -> None:
        """Feed latest reading from RtkGpsReader."""
        self._gps_reading = reading

    def activate_follow_me(self) -> bool:
        """Enter FOLLOW_ME mode from web UI. Returns True if activated."""
        if self._follow_me is not None and self._safety_state.is_armed:
            self._mode = "FOLLOW_ME"
            return True
        return False

    def deactivate_follow_me(self) -> None:
        """Return to MANUAL mode from Follow Me."""
        if self._mode == "FOLLOW_ME":
            self._mode = "MANUAL"

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

    def process(
        self,
        rc: RCInputs,
        now_epoch_s: float | None = None,
        bt_override_bytes: tuple[int, int] | None = None,
    ) -> Tuple[DriveCommand, List[SafetyEvent], dict]:
        epoch_now = now_epoch_s if now_epoch_s is not None else time.time()
        mono_now = time.monotonic()

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
                self._mode = "FOLLOW_ME"
            elif ev in (SafetyEvent.FOLLOW_ME_EXITED, SafetyEvent.EMERGENCY_TRIGGERED):
                self._mode = "MANUAL"
        # Disarm exits waypoint nav
        if not self._safety_state.is_armed and self._mode == "WAYPOINT_NAV":
            self._mode = "MANUAL"

        # Emergency triggered: stop motors, disarm, schedule shutdown
        if any(e is SafetyEvent.EMERGENCY_TRIGGERED for e in events):
            self._motor.stop()
            self._relay.set_armed(False)
            self._shutdown.schedule_shutdown(delay_seconds=5.0)

        # Command computation
        telemetry: dict = {"mode": self._mode}

        if self._mode == "WAYPOINT_NAV" and self._waypoint_nav is not None:
            gps = self._gps_reading
            if gps is not None:
                gps_age = time.monotonic() - gps.timestamp
                target_brg, speed = self._waypoint_nav.compute(
                    gps.latitude, gps.longitude, gps.fix_quality, gps_age,
                )
                if speed > 0 and self._imu_compensator is not None:
                    self._imu_compensator.set_target_heading(target_brg)
                fwd = min(CENTER_OUTPUT_VALUE + speed, MAX_OUTPUT)
                left = right = fwd
            else:
                left = right = self.NEUTRAL
            steering_input = 0.0
            nav_st = self._waypoint_nav.get_status()
            telemetry["wp_index"] = nav_st.waypoint_index
            telemetry["wp_total"] = nav_st.waypoint_total
            telemetry["wp_name"] = nav_st.waypoint_name
            telemetry["wp_bearing_deg"] = nav_st.bearing_deg
            telemetry["wp_distance_m"] = nav_st.distance_m
            telemetry["wp_completed"] = nav_st.completed
            if nav_st.completed:
                self._mode = "MANUAL"
        elif self._mode == "FOLLOW_ME" and self._follow_me is not None:
            detections = self._person_detections or []
            left, right = self._follow_me.compute(detections)
            steering_input = self._bytes_to_steering_input(left, right)
            fm_status = self._follow_me.get_status()
            fm_status["follow_me_num_persons"] = len(detections)
            telemetry.update(fm_status)
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

        # Apply IMU steering compensation if available and enabled
        imu_correction = self._apply_imu_compensation(steering_input, mono_now)
        telemetry["steering_input"] = steering_input
        telemetry["imu_correction_raw"] = imu_correction

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

        # Use actual motor commands for moving_ok when Bluetooth is active
        if bt_override_bytes is not None:
            # For Bluetooth, use the motor byte values to determine if moving
            bt_left, bt_right = bt_override_bytes
            bt_left_diff = abs(bt_left - CENTER_OUTPUT_VALUE)
            bt_right_diff = abs(bt_right - CENTER_OUTPUT_VALUE)
            moving_ok = max(bt_left_diff, bt_right_diff) >= 20  # Minimum movement threshold
        else:
            # For RC, use the original logic
            d1 = rc.ch1_us - 1500
            d2 = rc.ch2_us - 1500
            moving_ok = max(abs(d1), abs(d2)) >= min_th

        # Determine equal_ok based on input source
        if bt_override_bytes is not None:
            # For Bluetooth, check if both motors are similar (straight intent)
            bt_left, bt_right = bt_override_bytes
            abs_diff = abs(bt_left - bt_right)
            equal_abs_ok = abs_diff <= 10
            max_abs = max(abs(bt_left - CENTER_OUTPUT_VALUE), abs(bt_right - CENTER_OUTPUT_VALUE), 1)
            equal_rel_ok = (abs_diff / max_abs) <= rel_pct
        else:
            # For RC, use the original logic
            abs_diff = abs(rc.ch1_us - rc.ch2_us)
            equal_abs_ok = abs_diff <= tol
            # Relative check (difference relative to magnitude)
            max_abs = max(abs(d1), abs(d2), 1)
            equal_rel_ok = (abs_diff / max_abs) <= rel_pct
        equal_ok = equal_abs_ok or (moving_ok and equal_rel_ok)
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
        if self._imu_compensator is not None and is_moving_straight and not self._was_moving_straight:
            try:
                self._imu_compensator.reset_target_heading()
            except Exception:
                pass
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

        # Obstacle avoidance throttle scaling
        obstacle_scale = 1.0
        if self._obstacle_avoidance is not None and self._obstacle_distance_m is not None:
            obstacle_scale = self._obstacle_avoidance.compute_throttle_scale(
                self._obstacle_distance_m,
                self._obstacle_age_s if self._obstacle_age_s is not None else 999.0,
            )
            left = self._scale_toward_neutral(left, obstacle_scale)
            right = self._scale_toward_neutral(right, obstacle_scale)
            oa_status = self._obstacle_avoidance.get_status()
            telemetry.update(oa_status)
        telemetry["obstacle_throttle_scale"] = obstacle_scale
        telemetry["obstacle_distance_m"] = self._obstacle_distance_m

        # If disarmed, force neutral outputs
        if not self._safety_state.is_armed:
            left = right = CENTER_OUTPUT_VALUE
            self._motor.stop()
        else:
            if left > MAX_OUTPUT:
                left = MAX_OUTPUT
            if right > MAX_OUTPUT:
                right = MAX_OUTPUT
            self._motor.set_tracks(left, right)

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
            for k, v in status_dict.items():
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    status_dict[k] = int(round(v))
            return status_dict
        except Exception:
            return None


