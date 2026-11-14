from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

from pi_app.control.rc_signal import rc_signal_ok, rc_switch_high, rc_switch_low


class SafetyEvent(Enum):
    ARMED = auto()
    DISARMED = auto()
    EMERGENCY_TRIGGERED = auto()
    FOLLOW_ME_ENTERED = auto()
    FOLLOW_ME_EXITED = auto()
    RC_STALE = auto()
    # Emitted with DISARMED when, while armed, ch3 has not held high for
    # arm_midband_disarm_s (and is not low). The re-arm latch is set on the
    # same tick.
    ARM_MIDBAND_DISARM = auto()
    # Emitted with DISARMED when, while armed, ch3 or ch5 reports no signal
    # (the RC_NO_SIGNAL_US sentinel or any width outside the valid range;
    # pi_app/control/rc_signal.py). The re-arm latch is set on the same
    # tick. Never with EMERGENCY_TRIGGERED: a dead e-stop lead is not an
    # e-stop, and the e-stop powers the Pi off.
    RC_NO_SIGNAL_DISARM = auto()


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
    # "Held high": ch3 >= arm_high_threshold_us on every tick for at least
    # this long. Arming needs it. Every hybrid-mode high burst in the logs
    # was one logged tick with mid-band 103-139 ms later (20-40 ms in the
    # 50 Hz capture); 0.2 s keeps a margin above that plus one 30 Hz tick
    # of read jitter. A healthy switch holds 2000-2004 for the whole
    # session, so the cost is 0.2 s of arm latency.
    arm_high_persist_s: float = 0.2
    # While armed and ch3 is not low, disarm (and set the re-arm latch)
    # when ch3 has not been held high for longer than this. Short bursts
    # never count as held high, so presses of the hybrid-mix button cannot
    # keep the robot armed. A healthy switch is held high on every tick;
    # one mid-band glitch costs it arm_high_persist_s (0.2 s) of this
    # budget. 2026-10-08 and 2026-11-05 healthy logs: zero non-high ticks
    # in 5,285 logged armed ticks (about 9.5 min). 0.5 s is 15 control
    # ticks at 30 Hz, and at the 1.61 m/s top speed it is 0.8 m of travel
    # on a dead arm switch.
    arm_midband_disarm_s: float = 0.5
    # A latch (rearm_requires_switch_cycle, every cause; ch4_requires_low)
    # clears only after a MEASURED low held on every tick for this long
    # (Kevin, 2026-11-13: "Held low 0.2 s"). One low sample used to clear it,
    # and a loose lead that bounces gives 500-1200 us samples: CH3 re-armed
    # about 0.6 s later and CH4 restarted FOLLOW_ME with the switch up all
    # the time (review sim, 2026-11-13). A no-signal tick, or any tick that
    # is not low, restarts the run. A real switch-off rests low for seconds,
    # so the cost is 0.2 s.
    latch_clear_low_persist_s: float = 0.2


@dataclass
class SafetyState:
    is_armed: bool = False
    # Safety clock (time.monotonic() in the controller) of the last arm or
    # disarm; -inf before the first one.
    last_transition_mono_s: float = float("-inf")
    last_ch5_high: bool = False
    emergency_active: bool = False
    follow_me_active: bool = False
    last_ch4_high: bool = False
    # Software disarm (auto-disarm, mid-band disarm, no signal, startup
    # interlock) holds this until ch3 is held at or below
    # arm_low_threshold_us for latch_clear_low_persist_s. While set, ch3
    # held high must not arm.
    rearm_requires_switch_cycle: bool = False
    # Safety clock of the first tick of the current unbroken run of ch3 at
    # or above arm_high_threshold_us; None when this tick's ch3 is below it.
    ch3_high_since_mono_s: Optional[float] = None
    # Safety clock of the last tick on which ch3 was held high (that run
    # had lasted arm_high_persist_s); None before the first such tick.
    ch3_held_high_mono_s: Optional[float] = None
    # Set when ch4 had no signal: Follow Me cannot start until ch4 is held at
    # or below follow_me_low_threshold_us (a measured width) for
    # latch_clear_low_persist_s. Without it, ch4 high -> no signal ->
    # mid-band -> high restarted Follow Me with the switch up all the time
    # (Grok review 2026-11-13).
    ch4_requires_low: bool = False
    # Safety clock of the first tick of the current unbroken run of a
    # measured ch3 (ch4) at or below its low threshold; None otherwise.
    ch3_low_since_mono_s: Optional[float] = None
    ch4_low_since_mono_s: Optional[float] = None
    # This tick: ch3 has been held low for latch_clear_low_persist_s.
    # Controller.arm_switch_held_off reads it (pack-low latch, person stop
    # hold release).
    ch3_low_held: bool = False

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
    now_mono_s: Optional[float] = None,
) -> Tuple[SafetyState, List[SafetyEvent]]:
    """
    Compute the next safety state and emitted events from RC inputs.

    Clock: every safety timer (debounce, arm hold, mid-band disarm) runs on
    ``now_mono_s`` (time.monotonic() in the controller), so an NTP step of
    the wall clock cannot change safety timing. The Pi has no RTC battery
    and NTP steps the clock after boot. When ``now_mono_s`` is None,
    ``now_epoch_s`` is the clock (unit tests that pass one time).

    Rules:
    - Arm/disarm based on ch3 thresholds with debounce between state changes.
    - Arm needs ch3 held high: >= arm_high_threshold_us on every tick for
      arm_high_persist_s (one high frame no longer arms). Disarm on
      ch3 <= arm_low_threshold_us is unchanged: one low frame disarms once
      the debounce allows a transition.
    - Mid-band disarm: while armed and ch3 is not low, ch3 not held high for
      longer than arm_midband_disarm_s disarms without waiting for the
      debounce, sets rearm_requires_switch_cycle, and emits DISARMED plus
      ARM_MIDBAND_DISARM.
    - Emergency triggers on ch5 rising edge to >= emergency_threshold_us; once triggered, it latches.
      When emergency triggers, the system is forced DISARMED and emits EMERGENCY_TRIGGERED.
    - Follow Me activation: ch4 rising edge to >= follow_me_high_threshold_us while armed.
    - Follow Me deactivation: ch4 drops below follow_me_low_threshold_us, or disarm, or emergency.
    - rearm_requires_switch_cycle: ch3 must be held <= arm_low_threshold_us on every
      tick for latch_clear_low_persist_s (Kevin, 2026-11-13; it was one sample)
      before a later high can arm. Any other ch3 value forces disarmed and leaves the
      latch set. The emergency early return keeps the latch.
    - No signal (safety findings 1 and 5, docs/rc_arduino_gps_wiring.md): a
      channel value that is not a measured width (the RC_NO_SIGNAL_US
      sentinel, or anything outside RC_VALID_MIN_US..RC_VALID_MAX_US; see
      pi_app/control/rc_signal.py) is never high, never mid-band and never
      low. No signal on ch3 or ch5 disarms at once (no debounce), sets
      rearm_requires_switch_cycle and clears the ch3 runs, even when
      already disarmed; armed, it emits DISARMED plus RC_NO_SIGNAL_DISARM.
      It never triggers the emergency: only a measured ch5 width at or
      above emergency_threshold_us does, on its rising edge (a ch5 that
      comes back high after no signal is such an edge). No signal on ch4
      ends Follow Me and sets ch4_requires_low: Follow Me cannot start
      again until ch4 is held at or below follow_me_low_threshold_us for
      latch_clear_low_persist_s.
    """
    events: List[SafetyEvent] = []
    now = now_epoch_s if now_mono_s is None else now_mono_s

    ch3_no_signal = not rc_signal_ok(ch3_us)
    ch4_no_signal = not rc_signal_ok(ch4_us)
    ch5_no_signal = not rc_signal_ok(ch5_us)
    ch4_high = rc_switch_high(ch4_us, params.follow_me_high_threshold_us)
    ch5_high = rc_switch_high(ch5_us, params.emergency_threshold_us)

    new_state = SafetyState(
        is_armed=state.is_armed,
        last_transition_mono_s=state.last_transition_mono_s,
        last_ch5_high=ch5_high,
        emergency_active=state.emergency_active,
        follow_me_active=state.follow_me_active,
        last_ch4_high=ch4_high,
        rearm_requires_switch_cycle=state.rearm_requires_switch_cycle,
        ch3_high_since_mono_s=state.ch3_high_since_mono_s,
        ch3_held_high_mono_s=state.ch3_held_high_mono_s,
        ch4_requires_low=state.ch4_requires_low,
    )
    # --- Held-low runs that clear the latches (Kevin, 2026-11-13) ---
    # Updated on every tick, before any early return. A measured low on
    # every tick for latch_clear_low_persist_s; a no-signal tick or any
    # other value restarts the run. One bounced low sample is not enough.
    ch3_low_now = rc_switch_low(ch3_us, params.arm_low_threshold_us)
    ch4_low_now = rc_switch_low(ch4_us, params.follow_me_low_threshold_us)
    new_state.ch3_low_since_mono_s = _run_start(state.ch3_low_since_mono_s, ch3_low_now, now)
    new_state.ch4_low_since_mono_s = _run_start(state.ch4_low_since_mono_s, ch4_low_now, now)
    ch3_low_held = _run_held(new_state.ch3_low_since_mono_s, now, params.latch_clear_low_persist_s)
    ch4_low_held = _run_held(new_state.ch4_low_since_mono_s, now, params.latch_clear_low_persist_s)
    new_state.ch3_low_held = ch3_low_held
    # Follow Me latch: no signal on ch4 sets it on every tick (armed or not);
    # only a measured ch4 low held for latch_clear_low_persist_s clears it.
    if ch4_no_signal:
        new_state.ch4_requires_low = True
    elif ch4_low_held:
        new_state.ch4_requires_low = False

    # --- Emergency detection (rising edge) ---
    if not state.emergency_active and (not state.last_ch5_high and ch5_high):
        new_state.emergency_active = True
        if new_state.follow_me_active:
            new_state.follow_me_active = False
            events.append(SafetyEvent.FOLLOW_ME_EXITED)
        if new_state.is_armed:
            new_state.is_armed = False
            new_state.last_transition_mono_s = now
            events.append(SafetyEvent.DISARMED)
        events.append(SafetyEvent.EMERGENCY_TRIGGERED)
        _clear_ch3_runs(new_state)
        return new_state, events

    if new_state.emergency_active:
        _clear_ch3_runs(new_state)
        return new_state, events

    # --- No signal on the arm or e-stop channel (findings 1 and 5) ---
    # A dead CH3 lead held the last width (armed forever); a dead CH5 lead
    # removed the e-stop with no alarm. Either one: disarm now, latch, and
    # forget the ch3 runs, so a lead that comes back with the switch up
    # cannot arm. Not the emergency: that powers the Pi off, and a loose
    # lead is not the operator's e-stop. Latch even when already disarmed:
    # the latch is what stops the robot arming on its own when the lead
    # comes back.
    if ch3_no_signal or ch5_no_signal:
        _clear_ch3_runs(new_state)
        new_state.rearm_requires_switch_cycle = True
        if new_state.follow_me_active:
            new_state.follow_me_active = False
            events.append(SafetyEvent.FOLLOW_ME_EXITED)
        if new_state.is_armed:
            new_state.is_armed = False
            new_state.last_transition_mono_s = now
            events.append(SafetyEvent.DISARMED)
            events.append(SafetyEvent.RC_NO_SIGNAL_DISARM)
        return new_state, events

    # --- ch3 runs (RC arm-channel fail-safe, 2026-11-05) ---
    # ch3 is a measured width from here on (no signal returned above).
    ch3_low = rc_switch_low(ch3_us, params.arm_low_threshold_us)
    ch3_high = rc_switch_high(ch3_us, params.arm_high_threshold_us)
    if ch3_high:
        if new_state.ch3_high_since_mono_s is None:
            new_state.ch3_high_since_mono_s = now
    else:
        new_state.ch3_high_since_mono_s = None
    held_high = (
        ch3_high and (now - new_state.ch3_high_since_mono_s) >= params.arm_high_persist_s
    )
    if held_high:
        new_state.ch3_held_high_mono_s = now

    # --- Mid-band disarm ---
    # A healthy arm switch is held high on every armed tick. The CH3/CH4
    # hybrid mix rests mid-band with short bursts that never count as held,
    # and then only a lucky one-frame low burst could disarm. No debounce
    # wait: this is a fail-safe disarm, like the e-stop. A low tick takes
    # the normal low disarm below.
    if state.is_armed and not ch3_low:
        if new_state.ch3_held_high_mono_s is None:
            # Armed without a held high on record (a state built by hand,
            # e.g. in a test): start the clock on this tick.
            new_state.ch3_held_high_mono_s = now
        elif (now - new_state.ch3_held_high_mono_s) > params.arm_midband_disarm_s:
            new_state.is_armed = False
            new_state.last_transition_mono_s = now
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
        # Only a measured low held for latch_clear_low_persist_s clears it
        # (Kevin, 2026-11-13); one low sample (a bounce) no longer does. No
        # signal returned above and never counts as low.
        if ch3_low_held:
            new_state.rearm_requires_switch_cycle = False
        else:
            desired_armed = False

    if ch3_low:
        desired_armed = False
    elif held_high and not new_state.rearm_requires_switch_cycle:
        desired_armed = True

    if desired_armed != state.is_armed:
        if (now - state.last_transition_mono_s) >= params.debounce_seconds:
            new_state.is_armed = desired_armed
            new_state.last_transition_mono_s = now

            if desired_armed:
                # Start the post-arm debounce where the old one-frame rule
                # would have armed: the start of this high run, or the end
                # of the previous transition's debounce if that is later.
                # arm_high_persist_s then never lengthens the time a ch3-low
                # disarm is held off after the switch goes up. If a loop
                # hitch lets one low frame disarm right after the arm, that
                # is the safe direction; re-arming still needs the debounce
                # and the hold, so it cannot chatter.
                new_state.last_transition_mono_s = max(
                    new_state.ch3_high_since_mono_s,
                    state.last_transition_mono_s + params.debounce_seconds,
                )
                events.append(SafetyEvent.ARMED)
            else:
                if new_state.follow_me_active:
                    new_state.follow_me_active = False
                    events.append(SafetyEvent.FOLLOW_ME_EXITED)
                events.append(SafetyEvent.DISARMED)

    # --- Follow Me from ch4 (only while armed) ---
    if new_state.is_armed:
        if (
            not state.last_ch4_high
            and ch4_high
            and not state.follow_me_active
            and not new_state.ch4_requires_low
        ):
            new_state.follow_me_active = True
            events.append(SafetyEvent.FOLLOW_ME_ENTERED)
        elif state.follow_me_active and (
            ch4_no_signal or rc_switch_low(ch4_us, params.follow_me_low_threshold_us)
        ):
            new_state.follow_me_active = False
            events.append(SafetyEvent.FOLLOW_ME_EXITED)

    return new_state, events


def _run_start(prev_since: Optional[float], condition: bool, now: float) -> Optional[float]:
    """Start of the current unbroken run of ``condition``; None when it is False."""
    if not condition:
        return None
    return now if prev_since is None else prev_since


def _run_held(since: Optional[float], now: float, persist_s: float) -> bool:
    return since is not None and (now - since) >= persist_s


def _clear_ch3_runs(state: SafetyState) -> None:
    """Forget the ch3 runs, so no hold can look satisfied later."""
    state.ch3_high_since_mono_s = None
    state.ch3_held_high_mono_s = None


def force_disarm(
    state: SafetyState,
    now_mono_s: float,
) -> Tuple[SafetyState, List[SafetyEvent]]:
    """Disarm and latch until the arm switch is cycled.

    ``now_mono_s`` is the safety clock (see update_safety). Does not modify
    ``state``. Does not touch the emergency latch. Emits DISARMED only when
    ``state`` was armed, and FOLLOW_ME_EXITED when follow-me was active.
    """
    events: List[SafetyEvent] = []
    if state.follow_me_active:
        events.append(SafetyEvent.FOLLOW_ME_EXITED)
    if state.is_armed:
        events.append(SafetyEvent.DISARMED)
    new_state = SafetyState(
        is_armed=False,
        last_transition_mono_s=now_mono_s,
        last_ch5_high=state.last_ch5_high,
        emergency_active=state.emergency_active,
        follow_me_active=False,
        last_ch4_high=state.last_ch4_high,
        rearm_requires_switch_cycle=True,
        ch4_requires_low=state.ch4_requires_low,
    )
    return new_state, events
