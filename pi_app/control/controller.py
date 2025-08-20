from dataclasses import dataclass
from typing import Protocol, Tuple, List, Optional
import time
import threading
import sys
from pathlib import Path

# Add parent directory to path for config import
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.control.mapping import map_pulse_to_byte
from pi_app.control.safety import update_safety, SafetyState, SafetyParams, SafetyEvent
from pi_app.control.state import DriveCommand
from pi_app.control.imu_steering import ImuSteeringCompensator
from config import config


@dataclass(frozen=True)
class RCInputs:
    ch1_us: int
    ch2_us: int
    ch3_us: int
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
    def __init__(
        self,
        motor_driver: MotorDriver | None = None,
        arm_relay: ArmRelay | None = None,
        shutdown_scheduler: ShutdownScheduler | None = None,
        safety_params: SafetyParams | None = None,
        imu_compensator: Optional[ImuSteeringCompensator] = None,
    ) -> None:
        self._motor = motor_driver or NoopMotorDriver()
        self._relay = arm_relay or NoopArmRelay()
        self._shutdown = shutdown_scheduler or ThreadedShutdownScheduler()
        self._safety_state = SafetyState(is_armed=False, last_transition_epoch_s=0.0)
        self._safety_params = safety_params or SafetyParams()
        
        # IMU steering compensation
        self._imu_compensator = imu_compensator
        self._last_imu_update = 0.0
        self._imu_update_interval = 1.0 / config.imu_steering.update_rate_hz if config.imu_steering.enabled else 1.0

    def process(
        self,
        rc: RCInputs,
        now_epoch_s: float | None = None,
        bt_override_bytes: tuple[int, int] | None = None,
    ) -> Tuple[DriveCommand, List[SafetyEvent]]:
        now = now_epoch_s if now_epoch_s is not None else time.time()

        # Update safety
        self._safety_state, events = update_safety(
            self._safety_state,
            ch3_us=rc.ch3_us,
            ch5_us=rc.ch5_us,
            now_epoch_s=now,
            params=self._safety_params,
        )

        # Emergency triggered: stop motors, disarm, schedule shutdown
        if any(e is SafetyEvent.EMERGENCY_TRIGGERED for e in events):
            self._motor.stop()
            self._relay.set_armed(False)
            self._shutdown.schedule_shutdown(delay_seconds=5.0)

        # Command computation (prefer Bluetooth override if provided)
        if bt_override_bytes is not None:
            left, right = bt_override_bytes
            # Convert byte values to normalized steering input for IMU
            steering_input = self._bytes_to_steering_input(left, right)
        else:
            left = map_pulse_to_byte(rc.ch1_us)
            right = map_pulse_to_byte(rc.ch2_us)
            # Convert RC values to normalized steering input for IMU
            steering_input = self._bytes_to_steering_input(left, right)

        # Apply IMU steering compensation if available and enabled
        imu_correction = self._apply_imu_compensation(steering_input, now)
        
        # Apply correction to motor outputs
        if imu_correction is not None:
            left = self._apply_steering_correction(left, imu_correction)
            right = self._apply_steering_correction(right, -imu_correction)  # Opposite correction for right track

        # If disarmed, force neutral outputs
        if not self._safety_state.is_armed:
            left = right = 126
            self._motor.stop()
        else:
            self._motor.set_tracks(left, right)

        # Reflect arm relay state
        self._relay.set_armed(self._safety_state.is_armed)

        cmd = DriveCommand(
            left_byte=left,
            right_byte=right,
            is_armed=self._safety_state.is_armed,
            emergency_active=self._safety_state.emergency_active,
        )

        return cmd, events

    def _bytes_to_steering_input(self, left_byte: int, right_byte: int) -> float:
        """Convert left/right byte values to normalized steering input (-1.0 to 1.0)."""
        # Center is 126, so compute steering as difference from center
        left_diff = left_byte - 126
        right_diff = right_byte - 126
        
        # Average the steering input and normalize to [-1.0, 1.0]
        # Positive means turning right, negative means turning left
        steering = (right_diff - left_diff) / 2.0 / 126.0
        
        # Clamp to [-1.0, 1.0]
        return max(-1.0, min(1.0, steering))

    def _apply_imu_compensation(self, steering_input: float, now: float) -> Optional[float]:
        """Apply IMU steering compensation if available and timing allows."""
        if (self._imu_compensator is None or 
            not config.imu_steering.enabled or
            now - self._last_imu_update < self._imu_update_interval):
            return None
        
        try:
            dt = now - self._last_imu_update
            correction = self._imu_compensator.update(steering_input, dt)
            self._last_imu_update = now
            
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
        return max(0, min(255, corrected))

    def get_imu_status(self) -> Optional[dict]:
        """Get IMU status information for monitoring."""
        if self._imu_compensator is None:
            return None
        
        try:
            status = self._imu_compensator.get_status()
            return {
                'heading_deg': status.heading_deg,
                'target_heading_deg': status.target_heading_deg,
                'yaw_rate_dps': status.yaw_rate_dps,
                'is_available': status.is_available,
                'is_calibrated': status.is_calibrated,
                'error_count': status.error_count
            }
        except Exception:
            return None


