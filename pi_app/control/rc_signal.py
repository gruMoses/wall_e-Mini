"""RC "no signal" contract between the Arduino sketch and the Pi (pure logic).

Safety findings 1 and 5 of ``docs/rc_arduino_gps_wiring.md`` (2026-11-13):

1. The sketch changed a channel only on a pulse edge and sent a line every
   20 ms regardless, so a dead signal lead, an unpowered receiver or a
   receiver that stops its outputs held the LAST width forever (a dead CH3
   lead kept the robot armed, a dead CH5 lead removed the e-stop).
2. The sketch sent 1500 us for a width outside 800-2200 us, so garbage read
   as a valid neutral.

Wire format, Arduino to Pi, one line each 20 ms:

* ``ch1,ch2,ch3,ch4,ch5``       today's firmware (``dcc550e``, flashed)
* ``ch1,ch2,ch3,ch4,ch5,ch6``   the firmware with the no-pulse rule

Each field is a pulse width in us, ``RC_VALID_MIN_US`` to
``RC_VALID_MAX_US``, or ``RC_NO_SIGNAL_US`` (0): no valid pulse on that
channel for the sketch's timeout (100 ms), or the last pulse was out of
range. The Pi accepts both formats (requirement S12: the Pi changes first).

The reader keeps the sentinel in ``RCState`` / ``RCInputs``, so the logs show
it. Every consumer goes through the helpers below, so that:

* a no-signal value is never high, never mid-band and never neutral in a
  switch comparison;
* a no-signal value is never "switch OFF" either: an OFF reading grants
  things (it clears the re-arm latch, releases the person-stop hold, permits
  the vision-hang restart, clears the VESC pack-low latch), and a dead lead
  is not the operator's act;
* a no-signal track (CH1, CH2) reads neutral.

The safety reaction is in ``pi_app.control.safety.update_safety``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

RC_NO_SIGNAL_US = 0
# Valid pulse widths: the HotRC F-06A gives 1000-2500 us in its 50 Hz
# analogue mode and 500-2500 us in its 333 Hz digital mode (manual page 6;
# requirement S1). The sketch uses the same range.
RC_VALID_MIN_US = 500
RC_VALID_MAX_US = 2500
RC_NEUTRAL_US = 1500
RC_NUM_CHANNELS = 6

# What the robot does while a channel has no signal. Used in the journal.
_NO_SIGNAL_EFFECT = {
    1: "left track held at neutral",
    2: "right track held at neutral",
    3: "arm switch unknown: disarmed and latched, flip the arm switch off and on to re-arm",
    4: "FOLLOW_ME switch unknown: FOLLOW_ME off",
    5: "e-stop switch unknown: disarmed and latched (no e-stop power-off)",
    6: "not used",
}


def rc_signal_ok(us: object) -> bool:
    """True when ``us`` is a measured pulse width.

    The sentinel, None, a bool and any value outside RC_VALID_MIN_US to
    RC_VALID_MAX_US are "no signal".
    """
    if us is None or isinstance(us, bool):
        return False
    try:
        value = int(us)
    except (TypeError, ValueError):
        return False
    return RC_VALID_MIN_US <= value <= RC_VALID_MAX_US


def rc_switch_low(us: object, low_threshold_us: int) -> bool:
    """A measured width at or below ``low_threshold_us``. No signal is never low."""
    return rc_signal_ok(us) and int(us) <= int(low_threshold_us)  # type: ignore[arg-type]


def rc_switch_high(us: object, high_threshold_us: int) -> bool:
    """A measured width at or above ``high_threshold_us``. No signal is never high."""
    return rc_signal_ok(us) and int(us) >= int(high_threshold_us)  # type: ignore[arg-type]


def rc_track_us(us: object) -> int:
    """A track channel (CH1, CH2) for the mixer: neutral when there is no signal."""
    return int(us) if rc_signal_ok(us) else RC_NEUTRAL_US  # type: ignore[arg-type]


def rc_no_signal_effect(channel: int) -> str:
    return _NO_SIGNAL_EFFECT.get(int(channel), "unknown channel")


@dataclass(frozen=True)
class RcNoSignalEvent:
    """One logged entry into, or exit from, no signal on one channel."""

    channel: int  # 1..6
    no_signal: bool  # True: signal lost; False: signal back
    # How long the channel has been in this state when the event is logged
    # (an exit can be logged late, see RcNoSignalMonitor).
    since_s: float
    # Duration of the previous state (for an exit: how long it had no
    # signal); None for the first event of a channel.
    previous_state_s: Optional[float]
    # State changes since the previous logged event of this channel. 1 is
    # this change only; more means the rate limit merged a flapping lead.
    transitions: int

    def as_log_dict(self) -> dict:
        return {
            "event": "rc_no_signal" if self.no_signal else "rc_signal_restored",
            "channel": self.channel,
            "since_s": round(self.since_s, 3),
            "previous_state_s": (
                round(self.previous_state_s, 3) if self.previous_state_s is not None else None
            ),
            "transitions": self.transitions,
            "effect": rc_no_signal_effect(self.channel) if self.no_signal else None,
        }

    def journal_line(self) -> str:
        if self.no_signal:
            text = (
                f"RC CH{self.channel} NO SIGNAL: no valid pulse from the receiver "
                f"(signal lead, receiver power or receiver output); "
                f"{rc_no_signal_effect(self.channel)}"
            )
        else:
            lost = (
                f" after {self.previous_state_s:.1f} s"
                if self.previous_state_s is not None else ""
            )
            text = f"RC CH{self.channel} signal back{lost}"
        if self.transitions > 1:
            text += f" ({self.transitions} changes since the last line: the lead may be loose)"
        return text


@dataclass
class _ChannelLog:
    state: Optional[bool] = None  # None: no sample yet
    state_since_s: float = 0.0
    previous_state_s: Optional[float] = None
    logged_state: bool = False  # the state the last event said
    last_log_s: float = float("-inf")
    changes: int = 0


class RcNoSignalMonitor:
    """Per-channel journal and JSON events on entry to and exit from no signal.

    Rate limit: at most one event per channel per ``min_interval_s``. A
    change inside the interval is held back; when the interval has passed,
    the CURRENT state is logged if it differs from the last logged one, with
    the number of changes since. So a flapping lead gives one line per
    interval, and the log always ends in the true state. The first loss on a
    channel is logged at once.

    A value of None means the firmware does not report that channel (CH6 on
    the 5-field firmware); it is skipped.
    """

    def __init__(self, min_interval_s: float = 5.0) -> None:
        self._min_interval_s = max(0.0, float(min_interval_s))
        self._channels = [_ChannelLog() for _ in range(RC_NUM_CHANNELS)]

    def no_signal_channels(self) -> List[int]:
        return [i + 1 for i, c in enumerate(self._channels) if c.state]

    def update(self, now_s: float, values: Sequence[Optional[int]]) -> List[RcNoSignalEvent]:
        events: List[RcNoSignalEvent] = []
        for idx, us in enumerate(values[:RC_NUM_CHANNELS]):
            if us is None:
                continue
            ch = self._channels[idx]
            no_signal = not rc_signal_ok(us)
            if ch.state is None:
                ch.state = no_signal
                ch.state_since_s = now_s
                if no_signal:
                    ch.changes = 1
            elif no_signal != ch.state:
                ch.previous_state_s = now_s - ch.state_since_s
                ch.state = no_signal
                ch.state_since_s = now_s
                ch.changes += 1
            if ch.state != ch.logged_state and (now_s - ch.last_log_s) >= self._min_interval_s:
                events.append(
                    RcNoSignalEvent(
                        channel=idx + 1,
                        no_signal=bool(ch.state),
                        since_s=now_s - ch.state_since_s,
                        previous_state_s=ch.previous_state_s,
                        transitions=max(1, ch.changes),
                    )
                )
                ch.logged_state = bool(ch.state)
                ch.last_log_s = now_s
                ch.changes = 0
        return events
