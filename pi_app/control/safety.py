from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple


class SafetyEvent(Enum):
    ARMED = auto()
    DISARMED = auto()
    EMERGENCY_TRIGGERED = auto()
    FOLLOW_ME_ENTERED = auto()
    FOLLOW_ME_EXITED = auto()


@dataclass(frozen=True)
class SafetyParams:
    arm_high_threshold_us: int = 1800
    arm_low_threshold_us: int = 1200
    emergency_threshold_us: int = 1800
    debounce_seconds: float = 0.3
    follow_me_tap_window_s: float = 2.0
    follow_me_tap_count: int = 4


@dataclass
class SafetyState:
    is_armed: bool = False
    last_transition_epoch_s: float = 0.0
    last_ch5_high: bool = False
    emergency_active: bool = False
    follow_me_active: bool = False
    tap_timestamps: list = field(default_factory=list)


def update_safety(
    state: SafetyState,
    ch3_us: int,
    ch5_us: int,
    now_epoch_s: float,
    params: SafetyParams = SafetyParams(),
) -> Tuple[SafetyState, List[SafetyEvent]]:
    """
    Compute the next safety state and emitted events from RC inputs.

    Rules:
    - Arm/disarm based on ch3 thresholds with 0.3 s debounce between state changes.
    - Emergency triggers on ch5 rising edge to >= emergency_threshold_us; once triggered, it latches.
      When emergency triggers, the system is forced DISARMED and emits EMERGENCY_TRIGGERED.
    - Follow Me activation: N rapid arm->disarm taps within a time window.
    - Follow Me deactivation: any single disarm while Follow Me is active, or emergency.
    """
    events: List[SafetyEvent] = []

    # Emergency detection (rising edge)
    ch5_high = ch5_us >= params.emergency_threshold_us
    new_state = SafetyState(
        is_armed=state.is_armed,
        last_transition_epoch_s=state.last_transition_epoch_s,
        last_ch5_high=ch5_high,
        emergency_active=state.emergency_active,
        follow_me_active=state.follow_me_active,
        tap_timestamps=list(state.tap_timestamps),
    )

    if not state.emergency_active and (not state.last_ch5_high and ch5_high):
        # Rising edge: trigger emergency
        new_state.emergency_active = True
        if new_state.follow_me_active:
            new_state.follow_me_active = False
            new_state.tap_timestamps = []
            events.append(SafetyEvent.FOLLOW_ME_EXITED)
        if new_state.is_armed:
            new_state.is_armed = False
            new_state.last_transition_epoch_s = now_epoch_s
            events.append(SafetyEvent.DISARMED)
        events.append(SafetyEvent.EMERGENCY_TRIGGERED)
        return new_state, events

    # If emergency already active, do not process arming changes further
    if new_state.emergency_active:
        return new_state, events

    # Determine desired arm state from ch3 thresholds
    desired_armed = state.is_armed
    if ch3_us <= params.arm_low_threshold_us:
        desired_armed = False
    elif ch3_us >= params.arm_high_threshold_us:
        desired_armed = True
    # else: 1200 < ch3 < 1800 -> hold previous

    # Debounce arm state changes
    if desired_armed != state.is_armed:
        if (now_epoch_s - state.last_transition_epoch_s) >= params.debounce_seconds:
            new_state.is_armed = desired_armed
            new_state.last_transition_epoch_s = now_epoch_s

            if desired_armed:
                events.append(SafetyEvent.ARMED)
            else:
                # Disarm transition — handle Follow Me tap logic
                if new_state.follow_me_active:
                    new_state.follow_me_active = False
                    new_state.tap_timestamps = []
                    events.append(SafetyEvent.FOLLOW_ME_EXITED)
                    events.append(SafetyEvent.DISARMED)
                else:
                    # Record tap timestamp and prune expired taps
                    new_state.tap_timestamps.append(now_epoch_s)
                    cutoff = now_epoch_s - params.follow_me_tap_window_s
                    new_state.tap_timestamps = [
                        t for t in new_state.tap_timestamps if t > cutoff
                    ]
                    if len(new_state.tap_timestamps) >= params.follow_me_tap_count:
                        new_state.follow_me_active = True
                        new_state.tap_timestamps = []
                        events.append(SafetyEvent.DISARMED)
                        events.append(SafetyEvent.FOLLOW_ME_ENTERED)
                    else:
                        events.append(SafetyEvent.DISARMED)
        # else: not enough time has elapsed; keep prior state

    return new_state, events


