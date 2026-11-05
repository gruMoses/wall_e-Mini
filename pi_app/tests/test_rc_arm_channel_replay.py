"""Replay real RC logs through update_safety (RC arm-channel fail-safe, 2026-11-05).

From 2026-10-09 to 2026-11-05 the HotRC DS600 V3 had its "CH3/CH4 hybrid
(jog)" mix on. ch3 then rested at ~1500 us and went to ~2004 (or ~1000) only
for one short burst per button press. The level arming armed on one burst and
then held the arm while ch3 sat mid-band.

Fixture ``fixtures/rc_arm_channel_traces.csv.gz`` (columns
``log,ts,ch3,ch4,ch5,armed,mode,events``) is every logged control tick
(``rc``, ``ts``, ``safety.armed``, ``mode``, ``events``) of these gitignored
logs on Kevin's Mac, with each disarmed heartbeat tail cut to 2 rows:

- ``logs/rc_fault_20261104/arm_20261104_1857*.log``, ``..._1933*``..``_1943*``
  (hybrid mix on), ``arm_20261105_0934*.log`` (transmitter fixed);
- ``logs/netting_20261008/arm_20261008_171239.log`` and ``..._175636.log``
  (long healthy drives, 5.2 and 3.9 min armed);
- ``logs/rc_fault_20261104/rc_truth_table_20261104.csv`` (50 Hz serial
  capture of the hybrid mix), cut to +-1.5 s around each non-mid-band frame;
  its ``armed`` and ``mode`` columns are empty.

Logged ticks are ~10 Hz while armed (the control loop runs at ~30 Hz), so a
decision lands on the first logged tick at or after the real one. The logged
``ts`` is fed as the safety (monotonic) clock.
"""

import csv
import gzip
import math
import os
from collections import OrderedDict
from typing import List, NamedTuple, Optional, Tuple

import pytest

from pi_app.control.safety import SafetyEvent, SafetyParams, SafetyState, update_safety

FIXTURE = os.path.join(os.path.dirname(__file__), "fixtures", "rc_arm_channel_traces.csv.gz")

NEW = SafetyParams()
# The level rule as it was before 2026-11-05: one high frame arms, mid-band holds.
OLD = SafetyParams(arm_high_persist_s=0.0, arm_midband_disarm_s=math.inf)

# Hybrid mix on; ch3 held high for 2-28 s first, then mid-band while armed.
HELD_THEN_MIDBAND = [
    "arm_20261104_193325",
    "arm_20261104_193405",
    "arm_20261104_193432",
    "arm_20261104_194219",
    "arm_20261104_194310",
]
# Hybrid mix on; the old rule armed on one high burst.
BURST_ARMED = [
    "arm_20261104_185703",
    "arm_20261104_185709",
    "arm_20261104_185712",
    "arm_20261104_185714",
    "arm_20261104_194325",
]
# Healthy transmitter: ch3 held at 2000-2004 for the whole armed session.
HEALTHY = [
    "arm_20261105_093424",
    "arm_20261105_093432",
    "arm_20261105_093443",
    "arm_20261105_093449",
    "arm_20261008_171239",
    "arm_20261008_175636",
    # 2026-11-04 19:42, mix off for a moment: ch3 held high 0.46 s and 0.35 s.
    "arm_20261104_194217",
    "arm_20261104_194218",
]
TRUTH_TABLE = "rc_truth_table_20261104"


class Tick(NamedTuple):
    ts: float
    ch3: int
    ch4: int
    ch5: int
    armed: Optional[bool]
    mode: str
    events: Tuple[str, ...]


def _load():
    traces = OrderedDict()
    with gzip.open(FIXTURE, "rt", newline="") as f:
        for r in csv.DictReader(f):
            traces.setdefault(r["log"], []).append(
                Tick(
                    ts=float(r["ts"]),
                    ch3=int(r["ch3"]),
                    ch4=int(r["ch4"]),
                    ch5=int(r["ch5"]),
                    armed=None if r["armed"] == "" else r["armed"] == "1",
                    mode=r["mode"],
                    events=tuple(e for e in r["events"].split("|") if e),
                )
            )
    return traces


TRACES = _load()


def _midband(ch3: int, p: SafetyParams = NEW) -> bool:
    return p.arm_low_threshold_us < ch3 < p.arm_high_threshold_us


def _replay(ticks, params, state=None) -> List[Tuple[Tick, SafetyState, List[SafetyEvent]]]:
    state = state if state is not None else SafetyState()
    out = []
    for t in ticks:
        state, events = update_safety(
            state, t.ch3, t.ch4, t.ch5, now_epoch_s=t.ts, params=params, now_mono_s=t.ts
        )
        out.append((t, state, events))
    return out


def _first(replay, event):
    for i, (_t, _s, ev) in enumerate(replay):
        if event in ev:
            return i
    return None


def _segments(ticks, max_gap_s=0.5):
    """Split the cut-down serial capture where rows were removed."""
    seg = [ticks[0]]
    for a, b in zip(ticks, ticks[1:]):
        if b.ts - a.ts > max_gap_s:
            yield seg
            seg = []
        seg.append(b)
    yield seg


def _seeded_armed(first: Tick) -> SafetyState:
    """The state the old level rule left on its ARMED tick."""
    return SafetyState(
        is_armed=True,
        last_transition_mono_s=first.ts,
        ch3_held_high_mono_s=first.ts,
        last_ch4_high=first.ch4 >= NEW.follow_me_high_threshold_us,
        follow_me_active=first.mode == "FOLLOW_ME",
        last_ch5_high=first.ch5 >= NEW.emergency_threshold_us,
    )


def test_fixture_has_every_session():
    for name in HELD_THEN_MIDBAND + BURST_ARMED + HEALTHY + [TRUTH_TABLE]:
        assert len(TRACES.get(name, [])) >= 5, name


@pytest.mark.parametrize("name", HELD_THEN_MIDBAND + BURST_ARMED + HEALTHY)
def test_old_rule_reproduces_the_logged_arm_state(name):
    """Harness check: the pre-2026-11-05 parameters give the logged state on every tick."""
    for t, state, _ev in _replay(TRACES[name], OLD):
        assert state.is_armed == t.armed, (name, t)


@pytest.mark.parametrize("name", HELD_THEN_MIDBAND)
def test_midband_after_a_held_arm_disarms_and_latches(name):
    ticks = TRACES[name]
    rep = _replay(ticks, NEW)

    # The held high still arms, on the first logged tick >= arm_high_persist_s.
    i_arm = _first(rep, SafetyEvent.ARMED)
    assert i_arm is not None
    assert ticks[i_arm].ts - ticks[0].ts >= NEW.arm_high_persist_s
    assert ticks[i_arm - 1].ts - ticks[0].ts < NEW.arm_high_persist_s

    # Mid-band onset while armed, then the trip on the first tick more than
    # arm_midband_disarm_s after the last held-high tick.
    i_on = next(i for i in range(i_arm, len(ticks)) if _midband(ticks[i].ch3))
    t_held = ticks[i_on - 1].ts
    assert rep[i_on - 1][1].ch3_held_high_mono_s == t_held
    i_trip = _first(rep, SafetyEvent.ARM_MIDBAND_DISARM)
    assert i_trip is not None and i_trip > i_on
    assert all(_midband(t.ch3) for t in ticks[i_on : i_trip + 1])
    assert ticks[i_trip].ts - t_held > NEW.arm_midband_disarm_s
    assert ticks[i_trip - 1].ts - t_held <= NEW.arm_midband_disarm_s
    t, state, events = rep[i_trip]
    assert SafetyEvent.DISARMED in events
    assert not state.is_armed and state.rearm_requires_switch_cycle

    # The old rule stayed armed on the dead switch until a lucky low burst.
    i_old = next(i for i, t in enumerate(ticks) if "DISARMED" in t.events)
    assert ticks[i_old].ts - ticks[i_on].ts > 2.0
    assert i_trip < i_old

    # Never armed again in the rest of the trace.
    assert all(not s.is_armed for _t, s, _ev in rep[i_trip:])


@pytest.mark.parametrize("name", BURST_ARMED)
def test_one_hybrid_burst_no_longer_arms(name):
    assert _first(_replay(TRACES[name], OLD), SafetyEvent.ARMED) == 0
    rep = _replay(TRACES[name], NEW)
    assert all(not s.is_armed for _t, s, _ev in rep)


def test_serial_capture_bursts_never_arm():
    """50 Hz capture: high bursts are 1-2 frames (20-40 ms)."""
    segs = list(_segments(TRACES[TRUTH_TABLE]))
    old_arms = sum(_first(_replay(s, OLD), SafetyEvent.ARMED) is not None for s in segs)
    assert old_arms == 2  # the two high bursts in the capture
    for seg in segs:
        assert all(not s.is_armed for _t, s, _ev in _replay(seg, NEW))


@pytest.mark.parametrize("name", BURST_ARMED)
def test_midband_disarm_after_a_burst_arm(name):
    """Seeded with the state the old rule left on its ARMED tick."""
    ticks = TRACES[name]
    first = ticks[0]
    rep = _replay(ticks[1:], NEW, state=_seeded_armed(first))
    i_trip = _first(rep, SafetyEvent.ARM_MIDBAND_DISARM)
    assert i_trip is not None
    # The burst tick is the last "held" tick the old rule left behind.
    t_trip = rep[i_trip][0].ts
    assert t_trip - first.ts > NEW.arm_midband_disarm_s
    assert rep[i_trip - 1][0].ts - first.ts <= NEW.arm_midband_disarm_s
    events = rep[i_trip][2]
    if first.mode == "FOLLOW_ME":
        assert events == [
            SafetyEvent.FOLLOW_ME_EXITED,
            SafetyEvent.DISARMED,
            SafetyEvent.ARM_MIDBAND_DISARM,
        ]
    else:
        assert events == [SafetyEvent.DISARMED, SafetyEvent.ARM_MIDBAND_DISARM]
    # It trips before the old rule's lucky low-burst disarm.
    t_old = next(t.ts for t in ticks if "DISARMED" in t.events)
    assert t_trip < t_old


def test_serial_capture_midband_disarm_when_seeded_armed():
    """50 Hz: an arm left by a burst trips 0.5 s after the burst."""
    trips = 0
    for seg in _segments(TRACES[TRUTH_TABLE]):
        i_hi = next(
            (i for i, t in enumerate(seg) if t.ch3 >= NEW.arm_high_threshold_us), None
        )
        if i_hi is None:
            continue
        rep = _replay(seg[i_hi + 1 :], NEW, state=_seeded_armed(seg[i_hi]))
        i_trip = _first(rep, SafetyEvent.ARM_MIDBAND_DISARM)
        assert i_trip is not None
        dt = rep[i_trip][0].ts - seg[i_hi].ts
        assert NEW.arm_midband_disarm_s < dt <= NEW.arm_midband_disarm_s + 0.025
        trips += 1
    assert trips == 2


@pytest.mark.parametrize("name", HEALTHY)
def test_healthy_logs_never_trip_and_arm_late_by_the_persistence_only(name):
    ticks = TRACES[name]
    rep = _replay(ticks, NEW)
    assert _first(rep, SafetyEvent.ARM_MIDBAND_DISARM) is None
    assert sum(SafetyEvent.ARMED in ev for _t, _s, ev in rep) == 1

    # The arm lands on the first logged tick >= arm_high_persist_s after
    # the old arm tick (the persistence, rounded up to the next logged tick).
    i_arm = _first(rep, SafetyEvent.ARMED)
    delay = ticks[i_arm].ts - ticks[0].ts
    assert delay >= NEW.arm_high_persist_s
    assert ticks[i_arm - 1].ts - ticks[0].ts < NEW.arm_high_persist_s
    assert delay <= NEW.arm_high_persist_s + 0.2

    # After that the state is the logged state on every tick: the ch3-low
    # disarm lands on the same tick as before.
    for t, state, _ev in rep[i_arm:]:
        assert state.is_armed == t.armed, (name, t)
    assert not rep[-1][1].rearm_requires_switch_cycle
