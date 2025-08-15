from dataclasses import dataclass
from typing import Protocol, Tuple, List
import time
import threading

from pi_app.control.mapping import map_pulse_to_byte
from pi_app.control.safety import update_safety, SafetyState, SafetyParams, SafetyEvent
from pi_app.control.state import DriveCommand


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
    ) -> None:
        self._motor = motor_driver or NoopMotorDriver()
        self._relay = arm_relay or NoopArmRelay()
        self._shutdown = shutdown_scheduler or ThreadedShutdownScheduler()
        self._safety_state = SafetyState(is_armed=False, last_transition_epoch_s=0.0)
        self._safety_params = safety_params or SafetyParams()

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
        else:
            left = map_pulse_to_byte(rc.ch1_us)
            right = map_pulse_to_byte(rc.ch2_us)

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


