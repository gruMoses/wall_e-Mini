from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple


class SafetyEvent(Enum):
    ARMED = auto()
    DISARMED = auto()
    EMERGENCY_TRIGGERED = auto()
    FOLLOW_ME_ENTERED = auto()
    FOLLOW_ME_EXITED = auto()
    RC_STALE = auto()


@dataclass(frozen=True)
class SafetyParams:
    arm_high_threshold_us: int = 1800
    arm_low_threshold_us: int = 1200
    emergency_threshold_us: int = 1800
    debounce_seconds: float = 0.3
    follow_me_high_threshold_us: int = 1800
    follow_me_low_threshold_us: int = 1200


@dataclass
class SafetyState:
    is_armed: bool = False
    last_transition_epoch_s: float = 0.0
    last_ch5_high: bool = False
    emergency_active: bool = False
    follow_me_active: bool = False
    last_ch4_high: bool = False

    def set_follow_me_active(self, active: bool) -> None:
        """Allow external callers (e.g. web UI) to sync follow-me state."""
        self.follow_me_active = active


def update_safety(
    state: SafetyState,
    ch3_us: int,
    ch4_us: int,
    ch5_us: int,
    now_epoch_s: float,
    params: SafetyParams = SafetyParams(),
) -> Tuple[SafetyState, List[SafetyEvent]]:
    """
    Compute the next safety state and emitted events from RC inputs.

    Rules:
    - Arm/disarm based on ch3 thresholds with debounce between state changes.
    - Emergency triggers on ch5 rising edge to >= emergency_threshold_us; once triggered, it latches.
      When emergency triggers, the system is forced DISARMED and emits EMERGENCY_TRIGGERED.
    - Follow Me activation: ch4 rising edge to >= follow_me_high_threshold_us while armed.
    - Follow Me deactivation: ch4 drops below follow_me_low_threshold_us, or disarm, or emergency.
    """
    events: List[SafetyEvent] = []

    ch4_high = ch4_us >= params.follow_me_high_threshold_us
    ch5_high = ch5_us >= params.emergency_threshold_us

    new_state = SafetyState(
        is_armed=state.is_armed,
        last_transition_epoch_s=state.last_transition_epoch_s,
        last_ch5_high=ch5_high,
        emergency_active=state.emergency_active,
        follow_me_active=state.follow_me_active,
        last_ch4_high=ch4_high,
    )

    # --- Emergency detection (rising edge) ---
    if not state.emergency_active and (not state.last_ch5_high and ch5_high):
        new_state.emergency_active = True
        if new_state.follow_me_active:
            new_state.follow_me_active = False
            events.append(SafetyEvent.FOLLOW_ME_EXITED)
        if new_state.is_armed:
            new_state.is_armed = False
            new_state.last_transition_epoch_s = now_epoch_s
            events.append(SafetyEvent.DISARMED)
        events.append(SafetyEvent.EMERGENCY_TRIGGERED)
        return new_state, events

    if new_state.emergency_active:
        return new_state, events

    # --- Arm/disarm from ch3 ---
    desired_armed = state.is_armed
    if ch3_us <= params.arm_low_threshold_us:
        desired_armed = False
    elif ch3_us >= params.arm_high_threshold_us:
        desired_armed = True

    if desired_armed != state.is_armed:
        if (now_epoch_s - state.last_transition_epoch_s) >= params.debounce_seconds:
            new_state.is_armed = desired_armed
            new_state.last_transition_epoch_s = now_epoch_s

            if desired_armed:
                events.append(SafetyEvent.ARMED)
            else:
                if new_state.follow_me_active:
                    new_state.follow_me_active = False
                    events.append(SafetyEvent.FOLLOW_ME_EXITED)
                events.append(SafetyEvent.DISARMED)

    # --- Follow Me from ch4 (only while armed) ---
    if new_state.is_armed:
        if not state.last_ch4_high and ch4_high and not state.follow_me_active:
            new_state.follow_me_active = True
            events.append(SafetyEvent.FOLLOW_ME_ENTERED)
        elif ch4_us <= params.follow_me_low_threshold_us and state.follow_me_active:
            new_state.follow_me_active = False
            events.append(SafetyEvent.FOLLOW_ME_EXITED)

    return new_state, events
