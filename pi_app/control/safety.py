from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple


class SafetyEvent(Enum):
    ARMED = auto()
    DISARMED = auto()
    EMERGENCY_TRIGGERED = auto()
    FOLLOW_ME_ENTERED = auto()
    FOLLOW_ME_EXITED = auto()
    RC_STALE = auto()
    # Emitted with DISARMED when ch3 sat mid-band too long while armed
    # (arm_midband_disarm_s). The re-arm latch is set on the same tick.
    ARM_MIDBAND_DISARM = auto()


@dataclass(frozen=True)
class SafetyParams:
    arm_high_threshold_us: int = 1800
    arm_low_threshold_us: int = 1200
    emergency_threshold_us: int = 1800
    debounce_seconds: float = 0.3
    follow_me_high_threshold_us: int = 1800
    follow_me_low_threshold_us: int = 1200
    # RC arm-channel fail-safe (2026-11-05). The HotRC DS600 V3 "CH3/CH4
    # hybrid (jog)" mix, left on 2026-10-09..11-05, rests ch3 at ~1500 us
    # and sends ~2004 (or ~1000) only in one 40-140 ms burst per button
    # press. Level arming took one burst as "armed" and the mid-band rest
    # as "stay armed", so the switch could no longer disarm the robot.
    #
    # Arm only after ch3 has read >= arm_high_threshold_us on every tick
    # for at least this long. Every hybrid-mode high burst in the logs was
    # one logged tick with mid-band 103-139 ms later (20-40 ms in the
    # 50 Hz capture); 0.2 s keeps a margin above that plus one 30 Hz tick
    # of read jitter. A healthy switch holds 2000-2004 for the whole
    # session, so the cost is 0.2 s of arm latency.
    arm_high_persist_s: float = 0.2
    # While armed, disarm (and set the re-arm latch) when ch3 has read
    # strictly between the two thresholds on every tick for longer than
    # this. 2026-10-08 and 2026-11-05 healthy logs: zero mid-band ticks in
    # 5,285 logged armed ticks (about 9.5 min); a switch flip crosses
    # mid-band in 1-2 frames (<= 40 ms). 0.5 s is 15 control ticks at
    # 30 Hz, and at the 1.61 m/s top speed it is 0.8 m of travel on a dead
    # arm switch. Hybrid-mode replay: 0.52-0.60 s from the first logged
    # mid-band tick to the disarm at the ~10 Hz log rate.
    arm_midband_disarm_s: float = 0.5


@dataclass
class SafetyState:
    is_armed: bool = False
    last_transition_epoch_s: float = 0.0
    last_ch5_high: bool = False
    emergency_active: bool = False
    follow_me_active: bool = False
    last_ch4_high: bool = False
    # Software disarm (auto-disarm, mid-band disarm) holds this until ch3 is
    # seen at or below arm_low_threshold_us. While set, ch3 held high must
    # not arm.
    rearm_requires_switch_cycle: bool = False
    # Epoch of the first tick of the current unbroken run of ch3 at or
    # above arm_high_threshold_us; None when this tick's ch3 is below it.
    ch3_high_since_epoch_s: Optional[float] = None
    # Epoch of the first tick of the current unbroken run of ch3 strictly
    # between the two arm thresholds; None when this tick's ch3 is outside.
    ch3_midband_since_epoch_s: Optional[float] = None

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
    - Arm needs ch3 >= arm_high_threshold_us on every tick for arm_high_persist_s
      (one high frame no longer arms). Disarm on ch3 <= arm_low_threshold_us is
      unchanged: one low frame disarms once the debounce allows a transition.
    - Mid-band disarm: while armed, ch3 strictly between the two thresholds on
      every tick for longer than arm_midband_disarm_s disarms without waiting
      for the debounce, sets rearm_requires_switch_cycle, and emits DISARMED
      plus ARM_MIDBAND_DISARM.
    - Emergency triggers on ch5 rising edge to >= emergency_threshold_us; once triggered, it latches.
      When emergency triggers, the system is forced DISARMED and emits EMERGENCY_TRIGGERED.
    - Follow Me activation: ch4 rising edge to >= follow_me_high_threshold_us while armed.
    - Follow Me deactivation: ch4 drops below follow_me_low_threshold_us, or disarm, or emergency.
    - rearm_requires_switch_cycle: ch3 must be seen <= arm_low_threshold_us before a later
      high can arm. Any other ch3 value forces disarmed and leaves the latch set.
      The emergency early return keeps the latch.
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
        rearm_requires_switch_cycle=state.rearm_requires_switch_cycle,
        ch3_high_since_epoch_s=state.ch3_high_since_epoch_s,
        ch3_midband_since_epoch_s=state.ch3_midband_since_epoch_s,
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

    # --- ch3 run timers (RC arm-channel fail-safe, 2026-11-05) ---
    # A run restarts if the wall clock steps back past its start (NTP after
    # boot), so a backward step cannot hold off the mid-band disarm.
    ch3_low = ch3_us <= params.arm_low_threshold_us
    ch3_high = ch3_us >= params.arm_high_threshold_us
    ch3_midband = not ch3_low and not ch3_high
    if ch3_high:
        since = new_state.ch3_high_since_epoch_s
        if since is None or now_epoch_s < since:
            new_state.ch3_high_since_epoch_s = now_epoch_s
    else:
        new_state.ch3_high_since_epoch_s = None
    if ch3_midband:
        since = new_state.ch3_midband_since_epoch_s
        if since is None or now_epoch_s < since:
            new_state.ch3_midband_since_epoch_s = now_epoch_s
    else:
        new_state.ch3_midband_since_epoch_s = None

    # --- Mid-band disarm ---
    # A healthy arm switch never rests mid-band. The CH3/CH4 hybrid mix
    # does, and then only a lucky one-frame low burst could disarm. No
    # debounce wait: this is a fail-safe disarm, like the e-stop.
    if (
        state.is_armed
        and ch3_midband
        and (now_epoch_s - new_state.ch3_midband_since_epoch_s)
        > params.arm_midband_disarm_s
    ):
        new_state.is_armed = False
        new_state.last_transition_epoch_s = now_epoch_s
        new_state.rearm_requires_switch_cycle = True
        if new_state.follow_me_active:
            new_state.follow_me_active = False
            events.append(SafetyEvent.FOLLOW_ME_EXITED)
        events.append(SafetyEvent.DISARMED)
        events.append(SafetyEvent.ARM_MIDBAND_DISARM)
        return new_state, events

    # --- Arm/disarm from ch3 ---
    # Latch first. The level check below would otherwise re-arm on a switch
    # that never went low (2026-09-24 overnight-armed incident).
    desired_armed = state.is_armed
    if state.rearm_requires_switch_cycle:
        if ch3_low:
            new_state.rearm_requires_switch_cycle = False
        else:
            desired_armed = False

    if ch3_low:
        desired_armed = False
    elif (
        ch3_high
        and not new_state.rearm_requires_switch_cycle
        and (now_epoch_s - new_state.ch3_high_since_epoch_s)
        >= params.arm_high_persist_s
    ):
        desired_armed = True

    if desired_armed != state.is_armed:
        if (now_epoch_s - state.last_transition_epoch_s) >= params.debounce_seconds:
            new_state.is_armed = desired_armed
            new_state.last_transition_epoch_s = now_epoch_s

            if desired_armed:
                # Start the post-arm debounce where the old one-frame rule
                # would have armed: the start of this high run, or the end
                # of the previous transition's debounce if that is later.
                # arm_high_persist_s then never lengthens the time a ch3-low
                # disarm is held off after the switch goes up.
                new_state.last_transition_epoch_s = max(
                    new_state.ch3_high_since_epoch_s,
                    state.last_transition_epoch_s + params.debounce_seconds,
                )
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


def force_disarm(
    state: SafetyState,
    now_epoch_s: float,
) -> Tuple[SafetyState, List[SafetyEvent]]:
    """Disarm and latch until the arm switch is cycled.

    Does not modify ``state``. Does not touch the emergency latch.
    Emits DISARMED only when ``state`` was armed, and FOLLOW_ME_EXITED
    when follow-me was active.
    """
    events: List[SafetyEvent] = []
    if state.follow_me_active:
        events.append(SafetyEvent.FOLLOW_ME_EXITED)
    if state.is_armed:
        events.append(SafetyEvent.DISARMED)
    new_state = SafetyState(
        is_armed=False,
        last_transition_epoch_s=now_epoch_s,
        last_ch5_high=state.last_ch5_high,
        emergency_active=state.emergency_active,
        follow_me_active=False,
        last_ch4_high=state.last_ch4_high,
        rearm_requires_switch_cycle=True,
    )
    return new_state, events
