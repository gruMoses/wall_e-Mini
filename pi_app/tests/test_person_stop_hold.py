"""Person/animal stop HOLD (pi_app/control/person_stop_hold.py).

The 0.8 m stop was decided again at every depth poll with no memory. Inside
~0.4 m the box is clipped by the frame edge and the depth sample fails, the
corridor ignores pixels at or below min_depth_mm, and one such frame
released the stop (2026-11-08 14:27:40, FOLLOW_ME, ~0.66 s with Kevin at
0.3-0.9 m). The hold keeps distance 0.0 until the object that caused it
(the chain) is seen beyond the release range, or the operator ends it. The
reader-level tests drive OakDepthReader._poll_depth and fail on the
stateless code.
"""

import math
import time
import unittest
from types import SimpleNamespace

import numpy as np

from config import FollowMeConfig, ObstacleAvoidanceConfig, config as default_config
from pi_app.control.controller import (
    Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler,
)
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.person_stop_hold import (
    NeutralTimer, PersonStopHold, rc_link_live, side_clipped,
)
from pi_app.control.safety import SafetyParams
from pi_app.hardware.oak_depth import OakDepthReader

H, W = 400, 640
INF = float("inf")
OC = default_config.obstacle_avoidance
# Release on a walk-away: the chain resolves on its resolve_polls-th
# seen-beyond poll, which is also the first evidence poll.
BEYOND_TO_RELEASE = OC.person_stop_hold_resolve_polls + OC.person_stop_hold_release_polls - 1


def det(status="ok", z=0.0, bbox=(0.3, 0.0, 0.7, 1.0), tier="stop", label="person"):
    return SimpleNamespace(safety_tier=tier, label_name=label, z_m=z,
                           bbox=bbox, depth_status=status)


CENTRE_CLOSE = det("ok", 0.5, (0.25, 0.0, 0.75, 1.0))
LEFT_CLIPPED_AMBIG = det("ambiguous", 0.0, (0.001, 0.003, 0.28, 0.993))
LEFT_CLIPPED_FALSE_FAR = det("ok", 2.72, (0.001, 0.002, 0.226, 0.995))
FAR_IN_VIEW = det("ok", 2.0, (0.40, 0.0, 0.55, 0.9))


def person_at(z, cx=0.5, status="ok"):
    """A standing adult at range z: shoulders 0.5 m in the 70 deg view
    (width 0.357/z), 1.75 m tall in the 42 deg view (height 2.28/z, capped)."""
    w = min(0.6, 0.357 / z)
    h = min(1.0, 2.28 / z)
    return det(status, z if status == "ok" else 0.0,
               (cx - w / 2, 1.0 - h, cx + w / 2, 1.0))


def walk_away(z0=0.6, z1=2.5, step=0.1):
    """1.5 m/s at 15 polls/s: 0.1 m per poll."""
    z = z0
    while z <= z1 + 1e-9:
        yield round(z, 3)
        z += step


# --------------------------------------------------------------------------
# Reader-level: _poll_depth with real frames
# --------------------------------------------------------------------------

class _FakeFrame:
    def __init__(self, array):
        self._arr = array

    def getFrame(self):
        return self._arr


class _FakeQueue:
    def __init__(self, frame=None):
        self._frame = frame

    def tryGet(self):
        f, self._frame = self._frame, None
        return f


def _frame(mm):
    return np.full((H, W), mm, dtype=np.uint16)


CLEAR = _frame(3000)        # yard at 3 m: every corridor pixel valid
BLIND = _frame(0)           # a person inside min_depth_mm fills the view
NETTING = _frame(450)       # mesh over the lens: a wall at 0.45 m


class _ReaderHarness:
    def __init__(self, **obs):
        kwargs = dict(robot_width_m=0.820, min_depth_mm=350, safety_stop_radius_m=0.8,
                      safety_stop_bbox_width=0.40)
        kwargs.update(obs)
        self.r = OakDepthReader(obstacle_config=ObstacleAvoidanceConfig(**kwargs),
                                follow_me_config=FollowMeConfig())
        self._seq = 0

    def poll(self, frame, dets=None, fresh=True, release=False, autonomous=True,
             neutral=False, write_levels=True):
        """One depth poll. ``dets`` None keeps the last list (no new packet)."""
        with self.r._lock:
            if dets is not None:
                self.r._all_dets_state.detections = list(dets)
            if fresh:
                self._seq += 1
                self.r._det_fresh_ts = time.monotonic() - 0.001 * (1000 - self._seq % 1000)
        if write_levels:
            self.r.set_person_stop_hold_inputs(release=release, autonomous=autonomous,
                                               forward_neutral=neutral)
        q = _FakeQueue(None if frame is None else _FakeFrame(frame))
        self.r._poll_depth(q, _FakeQueue(None), np)
        return self.r.get_min_distance()[0]

    def snap(self):
        return self.r.get_person_stop_hold()


class TestReaderHold(unittest.TestCase):
    def test_config_enables_hold_beyond_radius(self):
        self.assertTrue(OC.person_stop_hold_enabled)
        self.assertFalse(OC.person_stop_hold_in_manual, "MANUAL hold needs Kevin's word")
        self.assertGreater(OC.person_stop_hold_release_m, OC.safety_stop_radius_m)

    def test_one_bad_frame_does_not_release(self):
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE]), 0.0)
        # Box clipped and too narrow for the width backstop, corridor sees past.
        self.assertEqual(h.poll(CLEAR, [LEFT_CLIPPED_AMBIG]), 0.0)
        self.assertEqual(h.poll(CLEAR, []), 0.0)

    def test_clipped_box_sampling_background_does_not_release(self):
        h = _ReaderHarness()
        h.poll(BLIND, [CENTRE_CLOSE])
        for _ in range(20):
            self.assertEqual(h.poll(CLEAR, [LEFT_CLIPPED_FALSE_FAR]), 0.0)

    def test_person_seen_beyond_releases_after_resolve_and_release_polls(self):
        h = _ReaderHarness()
        beyond = 0
        for z in walk_away(0.6, 3.0):
            dist = h.poll(CLEAR, [person_at(z)])
            if z >= 1.0:
                beyond += 1
                if beyond < BEYOND_TO_RELEASE:
                    self.assertEqual(dist, 0.0)
                else:
                    self.assertAlmostEqual(dist, z, places=3)
                    break
        self.assertEqual(beyond, BEYOND_TO_RELEASE)
        self.assertEqual(h.snap()["reason"], "seen_beyond")

    def test_blind_close_person_never_releases_on_corridor(self):
        # No detection (YOLO lost the person at 0.3 m) and no valid pixels.
        h = _ReaderHarness()
        h.poll(BLIND, [CENTRE_CLOSE])
        for _ in range(30):
            self.assertEqual(h.poll(BLIND, []), 0.0)

    def test_side_exit_autonomous_needs_operator(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG])   # leaves the view at the left edge
        for _ in range(30):
            self.assertEqual(h.poll(CLEAR, []), 0.0)
        self.assertFalse(h.snap()["resolved"])
        # Arm switch off with a live RC link.
        self.assertAlmostEqual(h.poll(CLEAR, [], release=True), 3.0, places=3)
        self.assertEqual(h.snap()["reason"], "operator")

    # ── MANUAL: no hold by default (person_stop_hold_in_manual False) ──────

    def test_manual_default_keeps_the_stateless_stop(self):
        """Kevin's 2026-09-27 rule: MANUAL keeps the stop as it was."""
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE], autonomous=False), 0.0)
        self.assertFalse(h.snap()["active"])
        # The person is gone on the next poll: the stop ends at once, as before.
        self.assertAlmostEqual(h.poll(CLEAR, [], autonomous=False), 3.0, places=3)

    def test_manual_default_netting_does_not_latch(self):
        """Mesh over the lens never reads clear. A MANUAL hold would need an
        arm-switch cycle each time the camera sees Kevin close to the robot."""
        h = _ReaderHarness()
        self.assertEqual(h.poll(NETTING, [CENTRE_CLOSE], autonomous=False), 0.0)
        self.assertAlmostEqual(h.poll(NETTING, [], autonomous=False), 0.45, places=3)
        self.assertFalse(h.snap()["active"])

    def test_drop_to_manual_keeps_hold_until_stick_neutral(self):
        """An automatic drop to MANUAL (vision-stale exit slews to the sticks)
        is not an operator release: the held 0.0 stays applied until the
        operator gives no drive command for the neutral hold time."""
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])                 # FOLLOW_ME: hold engaged
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG])           # side exit
        self.assertEqual(h.poll(CLEAR, []), 0.0)      # held
        for _ in range(20):                           # MANUAL, stick held forward
            self.assertEqual(h.poll(CLEAR, [], autonomous=False, neutral=False), 0.0)
        self.assertTrue(h.snap()["manual_carry"])
        # Stick let go: released; a later push is human input.
        self.assertAlmostEqual(h.poll(CLEAR, [], autonomous=False, neutral=True), 3.0, places=3)
        self.assertEqual(h.snap()["reason"], "manual_neutral")
        self.assertAlmostEqual(h.poll(CLEAR, [], autonomous=False, neutral=False), 3.0, places=3)
        # The stateless stop still acts in MANUAL, with no new hold.
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE], autonomous=False), 0.0)
        self.assertAlmostEqual(h.poll(CLEAR, [], autonomous=False), 3.0, places=3)

    def test_drop_to_manual_with_frozen_list_keeps_hold(self):
        """The vision-stale exit: no new packet, the frozen list does not
        trigger. Before the fix the drop published the stateless corridor."""
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG])
        for _ in range(20):
            self.assertEqual(h.poll(CLEAR, None, fresh=False, autonomous=False), 0.0)

    def test_manual_trigger_carries_into_autonomous_start(self):
        """The mode level reaches the worker one tick late, so the trigger
        poll still sees MANUAL. The next bad frame must be held."""
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE], autonomous=False), 0.0)
        self.assertFalse(h.snap()["active"])
        self.assertEqual(h.poll(CLEAR, [LEFT_CLIPPED_AMBIG]), 0.0)   # FOLLOW_ME now
        snap = h.snap()
        self.assertEqual((snap["active"], snap["reason"]), (True, "carry_in"))
        for _ in range(10):
            self.assertEqual(h.poll(CLEAR, []), 0.0)

    def test_stale_levels_fail_strict(self):
        h = _ReaderHarness()
        # Levels written once as MANUAL + stick neutral, then never refreshed.
        h.poll(CLEAR, [], autonomous=False, neutral=True)
        with h.r._lock:
            h.r._person_hold_levels_ts = time.monotonic() - 5.0
        # Strict: treated as autonomous, so a trigger engages the hold ...
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE], write_levels=False), 0.0)
        self.assertTrue(h.snap()["active"])
        # ... and the stale "neutral" does not release it.
        for _ in range(10):
            self.assertEqual(h.poll(CLEAR, [], write_levels=False), 0.0)

    def test_levels_never_written_fail_strict(self):
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE], write_levels=False), 0.0)
        self.assertTrue(h.snap()["active"])
        for _ in range(10):
            self.assertEqual(h.poll(CLEAR, [], write_levels=False), 0.0)

    def test_interior_loss_blocks_corridor_release(self):
        """A close, unmeasured box that vanished mid-frame (not at an edge,
        narrower than the width backstop): the chain is not resolved."""
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        h.poll(CLEAR, [det("ambiguous", 0.0, (0.40, 0.0, 0.60, 1.0))])
        for _ in range(30):
            self.assertEqual(h.poll(CLEAR, []), 0.0)
        self.assertFalse(h.snap()["resolved"])

    def test_inset_box_after_side_exit_blocks_corridor_release(self):
        """One box just inside the edge margin must not resolve the side exit."""
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG])
        h.poll(CLEAR, [det("ambiguous", 0.0, (0.06, 0.0, 0.25, 1.0))])
        for _ in range(30):
            self.assertEqual(h.poll(CLEAR, []), 0.0)

    def test_resolved_exit_does_not_release_on_corridor(self):
        """Safety review 4: the corridor is not evidence in an autonomous mode.

        Last seen far and unclipped (3 sightings), then out of view with a
        clear corridor. The first version of this design released here after
        5 empty polls (0.33 s, inside the gap limit), while a person who was
        seen beyond at 1.0 m can be back inside 0.8 m in 0.13 s, undetected.
        """
        h = _ReaderHarness()
        for z in walk_away(0.6, 1.2):           # 0.6 .. 1.2: three sightings >= 1.0
            h.poll(CLEAR, [person_at(z)])
        self.assertTrue(h.snap()["resolved"])
        for _ in range(30):
            self.assertEqual(h.poll(CLEAR, []), 0.0)
        snap = h.snap()
        self.assertTrue(snap["active"])
        self.assertFalse(snap["resolved"], "a poll without the chain clears resolved")
        # The operator still ends it.
        self.assertAlmostEqual(h.poll(CLEAR, [], release=True), 3.0, places=3)

    def test_seen_beyond_long_enough_releases_before_the_exit(self):
        """The normal walk-away: seen beyond for the full count, then released
        by seen_beyond while still in view. No corridor is needed."""
        h = _ReaderHarness()
        out = [h.poll(CLEAR, [person_at(z)]) for z in walk_away(0.6, 2.5)]
        beyond = sum(1 for z in walk_away(0.6, 2.5) if z >= OC.person_stop_hold_release_m)
        self.assertGreaterEqual(beyond, BEYOND_TO_RELEASE)
        first_free = next(i for i, v in enumerate(out) if v > 0.0)
        # Held on every poll up to the release, then free.
        self.assertTrue(all(v == 0.0 for v in out[:first_free]))
        self.assertEqual(h.snap()["reason"], "seen_beyond")
        self.assertFalse(h.snap()["active"])

    def test_label_flicker_of_the_only_box_keeps_the_chain(self):
        """2026-11-08 14:27:41: YOLO called the close, clipped operator
        "horse" for a packet. One box, same place, plausible size: the same
        object. The hold stays on him and releases when HE is seen beyond."""
        h = _ReaderHarness()
        h.poll(CLEAR, [person_at(0.6)])
        horse = person_at(0.6)
        horse.label_name = "horse"
        for _ in range(3):
            self.assertEqual(h.poll(CLEAR, [horse]), 0.0)
        self.assertFalse(h.snap()["operator_only"], h.snap())
        out = [h.poll(CLEAR, [person_at(z)]) for z in walk_away(0.7, 2.5)]
        self.assertGreater(out[-1], 0.0)
        self.assertEqual(h.snap()["reason"], "seen_beyond")

    def test_label_flicker_rule_needs_one_box_and_the_same_place(self):
        # Two boxes in view: the group rule stays, so the "horse" is a second
        # close object.
        h = _ReaderHarness()
        h.poll(CLEAR, [person_at(0.6)])
        horse = person_at(0.6)
        horse.label_name = "horse"
        for _ in range(3):
            h.poll(CLEAR, [person_at(0.6), horse])
        self.assertTrue(h.snap()["operator_only"])
        # One box of the other group, but not in the chain's place: not the chain.
        h = _ReaderHarness()
        h.poll(CLEAR, [person_at(0.6, cx=0.35)])
        goat = person_at(0.6, cx=0.60)
        goat.label_name = "sheep"
        for _ in range(3):
            self.assertEqual(h.poll(CLEAR, [goat]), 0.0)
        self.assertTrue(h.snap()["operator_only"])

    def test_far_person_does_not_release_vanished_close_animal(self):
        """The evidence must belong to the object that caused it."""
        goat = det("ok", 0.5, (0.15, 0.3, 0.45, 1.0), label="horse")
        kevin = det("ok", 2.0, (0.55, 0.0, 0.70, 0.9))
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [goat, kevin]), 0.0)
        for _ in range(30):                       # goat stepped out of view
            self.assertEqual(h.poll(CLEAR, [kevin]), 0.0)
        self.assertAlmostEqual(h.poll(CLEAR, [kevin], release=True), 2.0, places=3)

    def test_two_close_objects_are_operator_only(self):
        a = det("ok", 0.5, (0.05, 0.0, 0.40, 1.0))
        b = det("ok", 0.6, (0.60, 0.0, 0.94, 1.0))
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [a, b]), 0.0)
        snap = h.snap()
        self.assertTrue(snap["operator_only"])
        self.assertEqual(snap["operator_only_reason"], "second_trigger")
        for z in walk_away(0.6, 3.0):              # one walks away; the other is gone
            self.assertEqual(h.poll(CLEAR, [person_at(z, cx=0.25)]), 0.0)
        self.assertAlmostEqual(h.poll(CLEAR, [], release=True), 3.0, places=3)
        self.assertFalse(h.snap()["operator_only"])

    def test_netting_in_an_autonomous_mode_holds_until_operator(self):
        h = _ReaderHarness()
        h.poll(NETTING, [CENTRE_CLOSE])
        for _ in range(30):
            self.assertEqual(h.poll(NETTING, []), 0.0)
        self.assertAlmostEqual(h.poll(NETTING, [], release=True), 0.45, places=3)

    # ── MANUAL with person_stop_hold_in_manual True (not the default) ──────

    def test_side_exit_manual_releases_on_measured_clear_corridor(self):
        h = _ReaderHarness(person_stop_hold_in_manual=True)
        h.poll(CLEAR, [CENTRE_CLOSE], autonomous=False)
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG], autonomous=False)
        n = OC.person_stop_hold_release_polls
        for _ in range(n - 1):
            self.assertEqual(h.poll(CLEAR, [], autonomous=False), 0.0)
        self.assertAlmostEqual(h.poll(CLEAR, [], autonomous=False), 3.0, places=3)
        self.assertEqual(h.snap()["reason"], "corridor_clear")

    def test_netting_over_lens_holds_until_operator(self):
        h = _ReaderHarness(person_stop_hold_in_manual=True)
        h.poll(NETTING, [CENTRE_CLOSE], autonomous=False)
        for _ in range(30):
            # 0.0 exactly: the netting mute never applies to this channel.
            self.assertEqual(h.poll(NETTING, [], autonomous=False), 0.0)
        self.assertAlmostEqual(h.poll(NETTING, [], release=True, autonomous=False), 0.45, places=3)

    def test_operator_release_with_person_still_close_reengages(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE], release=True), 0.0)
        self.assertTrue(h.snap()["active"])

    def test_frozen_detection_stream_never_releases(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        # The last packet before the stall carried no stop-tier box; no new
        # packet afterwards. Clear corridor is not enough without one.
        h.poll(CLEAR, [], fresh=True)
        for _ in range(30):
            self.assertEqual(h.poll(CLEAR, None, fresh=False), 0.0)

    def test_old_packet_does_not_count(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        for i in range(30):
            with h.r._lock:
                h.r._all_dets_state.detections = []
                h.r._det_fresh_ts = time.monotonic() - 5.0 + 0.001 * i
            h.r._poll_depth(_FakeQueue(_FakeFrame(CLEAR)), _FakeQueue(None), np)
            self.assertEqual(h.r.get_min_distance()[0], 0.0)

    def test_stale_depth_keeps_hold_and_recovers_on_evidence(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [person_at(0.6)])
        for _ in range(30):
            h.poll(None, [])                   # no depth frame: no poll runs
        dist, _age = h.r.get_min_distance()
        self.assertEqual(dist, 0.0)            # held; the age makes it stale
        zs = list(walk_away(0.7, 2.0))
        dists = [h.poll(CLEAR, [person_at(z)]) for z in zs]
        first = zs.index(1.0) + BEYOND_TO_RELEASE - 1
        self.assertEqual(dists[:first], [0.0] * first)
        self.assertAlmostEqual(dists[first], zs[first], places=3)

    def test_disabled_restores_stateless_stop(self):
        h = _ReaderHarness(person_stop_hold_enabled=False)
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE]), 0.0)
        self.assertAlmostEqual(h.poll(CLEAR, []), 3.0, places=3)

    # ── exceptions never drop the stateless stop ───────────────────────────

    def _boom(self, h):
        def raise_(*a, **k):
            raise RuntimeError("boom")
        h.r._person_hold.update = raise_

    def test_exception_keeps_stateless_stop_and_active_hold(self):
        h = _ReaderHarness()
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE]), 0.0)       # engaged
        self._boom(h)
        self.assertEqual(h.poll(CLEAR, []), 0.0)                   # active: held
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE]), 0.0)       # trigger: stop
        # The operator still ends it, without a process restart.
        self.assertAlmostEqual(h.poll(CLEAR, [], release=True), 3.0, places=3)
        self.assertFalse(h.snap()["active"])
        self.assertAlmostEqual(h.poll(CLEAR, []), 3.0, places=3)   # inactive: stateless
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE]), 0.0)

    def test_malformed_boxes_through_the_reader(self):
        h = _ReaderHarness()
        junk = [
            det("ok", float("nan"), (0.3, 0.0, 0.6, 1.0)),
            det("ok", 2.0, None),
            det("ok", 2.0, (0.3, 0.0)),
            det("ok", 2.0, (0.3, float("nan"), 0.6, 1.0)),
            SimpleNamespace(safety_tier="stop", label_name=None, z_m=0.0, bbox=None,
                            depth_status=None),
        ]
        self.assertEqual(h.poll(CLEAR, [CENTRE_CLOSE] + junk), 0.0)
        for _ in range(10):
            self.assertEqual(h.poll(CLEAR, junk), 0.0)
        self.assertFalse(h.r._person_hold_error_logged)


# --------------------------------------------------------------------------
# Pure class
# --------------------------------------------------------------------------

class _Pure(unittest.TestCase):
    def setUp(self):
        self.t = 100.0

    def _hold(self, **kw):
        args = dict(release_distance_m=1.0, release_polls=5, min_valid_pct=20.0)
        args.update(kw)
        return PersonStopHold(**args)

    def _step(self, hold, dets, trig, cmm=INF, rej=True, cvp=0.0, autonomous=True,
              fresh=True, dt=0.067, neutral=False, stop_det=None, release=False):
        self.t += dt
        if trig and stop_det is None:
            stop_det = dets[0]
        return hold.update(now=self.t, triggered=trig, detections=dets, fresh_detections=fresh,
                           corridor_mm=cmm, corridor_rejected=rej, corridor_valid_pct=cvp,
                           stop_det=stop_det, autonomous=autonomous,
                           forward_neutral=neutral, operator_release=release)


class TestPureHold(_Pure):
    def test_20261108_142740_sequence_holds(self):
        # Logged lines 14:27:39.926 - 14:27:40.946 (FOLLOW_ME), condensed.
        seq = [
            ([det("ambiguous", 0.0, (0.0, 0.0, 0.506, 0.999))], True, 599, False, 64.4),
            ([det("far_veto", 0.0, (0.001, 0.003, 0.448, 0.998))], True, 440, False, 54.6),
            ([det("ambiguous", 0.0, (0.001, 0.003, 0.28, 0.993))], False, 545, False, 48.0),
            ([det("ok", 2.72, (0.001, 0.002, 0.226, 0.995))], False, 628, False, 29.1),
            ([det("ambiguous", 0.0, (0.0, 0.001, 0.124, 0.998))], False, 3085, False, 47.1),
            ([det("ok", 2.59, (0.0, 0.494, 0.114, 0.998))], False, 3194, False, 42.7),
            ([], False, 4661, False, 53.3),
            ([], False, 2873, False, 45.0),
            ([det("ambiguous", 0.0, (0.001, 0.0, 0.227, 0.997))], False, 2749, False, 26.7),
            ([det("ok", 0.42, (0.0, 0.003, 0.331, 1.0))], True, 2836, False, 19.6),
        ]
        hold = self._hold()
        for dets, trig, cmm, rej, cvp in seq:
            self.assertTrue(self._step(hold, dets, trig, cmm, rej, cvp))
        # Nothing here was ever attributable to the chain being far.
        self.assertFalse(hold.snapshot().resolved)

    def test_20261108_142754_sequence_releases(self):
        """Kevin walks away inside a right-clipped box, then unclipped; the
        hold must still release once he is unclipped beyond 1.0 m."""
        seq = [  # (ts, z, bbox) from arm_20261108_142513.log
            (54.070, 0.61, (0.548, 0.0, 0.967, 0.999)),
            (54.196, 0.99, (0.69, 0.001, 0.998, 1.0)),
            (54.316, 1.08, (0.708, 0.001, 0.998, 1.0)),
            (54.419, 1.15, (0.75, 0.0, 0.998, 0.999)),
            (54.521, 1.19, (0.771, 0.001, 0.998, 0.997)),
            (54.630, 1.24, (0.791, 0.001, 0.999, 0.997)),
            (54.757, 1.33, (0.777, 0.003, 0.999, 1.0)),
            (54.875, 1.46, (0.715, 0.004, 1.0, 0.996)),
            (54.984, 1.51, (0.696, 0.001, 1.0, 0.997)),
            (55.105, 1.56, (0.651, 0.002, 0.932, 0.998)),
            (55.213, 1.66, (0.617, 0.0, 0.875, 0.997)),
            (55.318, 1.71, (0.608, 0.0, 0.86, 0.996)),
            (55.438, 1.79, (0.622, 0.001, 0.851, 0.996)),
            (55.562, 1.92, (0.615, 0.0, 0.838, 0.996)),
            (55.686, 2.08, (0.594, 0.0, 0.809, 0.996)),
            (55.798, 2.20, (0.59, 0.0, 0.80, 0.996)),
        ]
        hold = self._hold(edge_margin=0.05)
        out = []
        for i, (ts, z, bb) in enumerate(seq):
            d = det("ok", z, bb)
            out.append(hold.update(now=ts, triggered=(i == 0), detections=[d], stop_det=d,
                                   fresh_detections=True, corridor_mm=2800.0,
                                   corridor_rejected=False, corridor_valid_pct=25.0))
            if i < len(seq) - 1:
                self.assertFalse(hold.snapshot().operator_only, i)
        self.assertTrue(all(out[:15]))          # clipped, then resolving: held
        self.assertFalse(out[15])               # 55.798: 3 beyond + 4 more evidence
        self.assertEqual(hold.snapshot().reason, "seen_beyond")

    def test_resolve_then_release_polls(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6)], True)
        for z in (0.7, 0.8, 0.9):
            self.assertTrue(self._step(hold, [person_at(z)], False))
        zs = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
        out = [self._step(hold, [person_at(z)], False) for z in zs]
        self.assertEqual(out, [True] * 6 + [False])

    def test_bad_poll_resets_evidence(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6)], True)
        for z in (0.8, 1.0, 1.2, 1.4):
            self._step(hold, [person_at(z)], False)
        self._step(hold, [LEFT_CLIPPED_AMBIG, person_at(1.5)], False)
        self.assertEqual(hold.snapshot().evidence_polls, 0)
        out = [self._step(hold, [person_at(1.6 + 0.1 * i)], False) for i in range(4)]
        self.assertTrue(all(out))

    def test_low_valid_support_is_not_clear(self):
        hold = self._hold()
        for z in (0.6, 0.8, 1.0, 1.2, 1.4):
            self._step(hold, [person_at(z)], z == 0.6)
        self.assertTrue(hold.snapshot().resolved)
        for _ in range(30):
            self.assertTrue(self._step(hold, [], False, cmm=3000, rej=False, cvp=19.9, dt=0.01))

    def test_near_corridor_is_not_clear(self):
        hold = self._hold()
        for z in (0.6, 0.8, 1.0, 1.2, 1.4):
            self._step(hold, [person_at(z)], z == 0.6)
        for _ in range(30):
            self.assertTrue(self._step(hold, [], False, cmm=999, rej=False, cvp=90.0, dt=0.01))

    def test_any_unmeasured_stop_box_blocks_seen_beyond(self):
        hold = self._hold()
        self._step(hold, [CENTRE_CLOSE], True)
        for _ in range(30):
            self.assertTrue(self._step(hold, [FAR_IN_VIEW, det("no_support", 0.0)], False))

    def test_side_clipped_boundary(self):
        m = OC.person_stop_hold_edge_margin
        self.assertEqual(m, 0.05)
        self.assertTrue(side_clipped((0.0, 0.0, 0.3, 1.0), m))
        self.assertTrue(side_clipped((0.049, 0.0, 0.3, 1.0), m))
        self.assertFalse(side_clipped((0.051, 0.0, 0.3, 1.0), m))
        self.assertTrue(side_clipped((0.6, 0.0, 0.951, 1.0), m))
        self.assertFalse(side_clipped((0.6, 0.0, 0.949, 1.0), m))
        self.assertTrue(side_clipped(None, m))
        self.assertTrue(side_clipped((0.1, "x", 0.3, 1.0), m))

    def test_seen_beyond_at_margin_boundary(self):
        hold = self._hold(edge_margin=0.05)
        self.assertFalse(hold.seen_beyond(det("ok", 2.0, (0.021, 0.0, 0.3, 1.0))))
        self.assertFalse(hold.seen_beyond(det("ok", 2.0, (0.049, 0.0, 0.3, 1.0))))
        self.assertTrue(hold.seen_beyond(det("ok", 2.0, (0.051, 0.0, 0.3, 1.0))))
        self.assertFalse(hold.seen_beyond(det("ok", 0.99, (0.3, 0.0, 0.6, 1.0))))
        self.assertTrue(hold.seen_beyond(det("ok", 1.0, (0.3, 0.0, 0.6, 1.0))))
        self.assertFalse(hold.seen_beyond(det("no_support", 2.0, (0.3, 0.0, 0.6, 1.0))))

    def test_walk_away_through_detection_gaps_releases(self):
        """One person walking away at 1.5 m/s, YOLO missing 0.2-0.35 s at a
        time: the chain survives every gap and still releases."""
        hold = self._hold()
        self._step(hold, [person_at(0.6)], True)
        z = 0.6
        for gap in (0.2, 0.35, 0.25, 0.3, 0.2):
            self.assertTrue(self._step(hold, [], False, dt=gap))   # missed packets
            z += 1.5 * gap
            for _ in range(2):
                z += 0.1
                self.assertTrue(self._step(hold, [person_at(z)], False))
                self.assertTrue(hold.snapshot().chain_present)
        released = False
        for _ in range(BEYOND_TO_RELEASE):
            z += 0.1
            if not self._step(hold, [person_at(z)], False):
                released = True
                break
        self.assertTrue(released)
        self.assertEqual(hold.snapshot().reason, "seen_beyond")

    def test_chain_lost_is_operator_only(self):
        hold = self._hold()
        for z in (0.6, 0.8, 1.0, 1.2):
            self._step(hold, [person_at(z)], z == 0.6)
        self.assertTrue(self._step(hold, [], False, dt=0.7))       # gap > 0.6 s
        snap = hold.snapshot()
        self.assertEqual((snap.operator_only, snap.operator_only_reason), (True, "lost"))
        for _ in range(30):                                       # clear corridor
            self.assertTrue(self._step(hold, [], False, cmm=3000, rej=False, cvp=60.0))
        self.assertFalse(self._step(hold, [], False, release=True))
        self.assertFalse(hold.snapshot().operator_only)

    def test_far_neighbour_when_close_drops_one_packet(self):
        """The close person misses one packet; a far neighbour inside the gate
        appears. It is not the chain; the hold becomes operator-only."""
        hold = self._hold()
        self._step(hold, [person_at(0.5, cx=0.45)], True)
        far = person_at(2.0, cx=0.62)
        for _ in range(40):
            self.assertTrue(self._step(hold, [far], False))
        snap = hold.snapshot()
        self.assertEqual((snap.operator_only, snap.operator_only_reason),
                         (True, "impostor_in_gate"))

    def test_far_neighbour_at_the_limit_2m_after_0_6s(self):
        """The last review's limit: 2.0 m exactly 0.6 s after a measured 0.5 m."""
        hold = self._hold()
        self._step(hold, [person_at(0.5, cx=0.45)], True)
        far = person_at(2.0, cx=0.55)
        self.assertTrue(self._step(hold, [far], False, dt=0.6))
        self.assertFalse(hold.snapshot().chain_present)
        self.assertTrue(hold.snapshot().operator_only)
        for _ in range(40):
            self.assertTrue(self._step(hold, [far], False))

    def test_carry_in_impostor_2_8m_at_1s(self):
        """A carry-in seed keeps its real sighting time: a far person in view
        at entry, 1.0 s after a measured 0.5 m, is not the close one."""
        hold = self._hold(carry_in_s=1.0)
        self._step(hold, [person_at(0.5, cx=0.5)], True, autonomous=False)
        far = person_at(2.8, cx=0.6)
        self.assertTrue(self._step(hold, [far], False, autonomous=True, dt=1.0))
        self.assertEqual(hold.snapshot().reason, "carry_in")
        self.assertFalse(hold.snapshot().chain_present)
        for _ in range(40):
            self.assertTrue(self._step(hold, [far], False))
        self.assertTrue(hold.snapshot().operator_only)

    def test_carry_in_impostor_from_unmeasured_seed(self):
        """The seed had no range (the width backstop fired): the implied
        reference is the stop radius, so 2.8 m 1.0 s later is not it."""
        hold = self._hold(carry_in_s=1.0)
        seed = det("no_support", 0.0, (0.28, 0.0, 0.72, 1.0))
        self._step(hold, [seed], True, autonomous=False)
        far = person_at(2.8, cx=0.6)
        self.assertTrue(self._step(hold, [far], False, autonomous=True, dt=1.0))
        self.assertFalse(hold.snapshot().chain_present)
        for _ in range(40):
            self.assertTrue(self._step(hold, [far], False))

    def test_clipped_or_unmeasured_box_cannot_refresh_the_range(self):
        """A clipped box's "ok" range (the background through the box) and an
        unmeasured box keep the chain alive but never move its range
        reference; a far box then is tested against the last TRUSTED range
        and its real age."""
        hold = self._hold()
        self._step(hold, [person_at(0.5, cx=0.30)], True)
        for _ in range(3):
            self._step(hold, [det("ok", 1.9, (0.0, 0.0, 0.30, 1.0))], False)   # clipped
        for _ in range(2):
            self._step(hold, [det("no_support", 0.0, (0.10, 0.0, 0.40, 1.0))], False)
        self.assertTrue(hold.snapshot().chain_present)
        far = det("ok", 2.0, (0.20, 0.05, 0.40, 0.95))
        for _ in range(40):
            self.assertTrue(self._step(hold, [far], False))
        self.assertTrue(hold.snapshot().operator_only)

    def test_clipped_fragment_does_not_open_the_area_test(self):
        """The area reference is set only by unclipped boxes: after slivers at
        the right edge, a small unclipped box is tested against the last full
        box, not against the sliver."""
        hold = self._hold()
        wide = det("no_support", 0.0, (0.35, 0.0, 0.80, 1.0))     # width backstop
        self._step(hold, [wide], True)
        for bb in ((0.60, 0.0, 1.0, 1.0), (0.80, 0.0, 1.0, 1.0), (0.90, 0.3, 1.0, 0.7)):
            self._step(hold, [det("ambiguous", 0.0, bb)], False)
            self.assertTrue(hold.snapshot().chain_present)
        small = det("no_support", 0.0, (0.80, 0.4, 0.90, 0.7))     # area 0.03
        self.assertTrue(self._step(hold, [small], False))
        self.assertFalse(hold.snapshot().chain_present)

    def test_area_test_is_two_sided(self):
        """A big box where a measured far chain was is not its continuation."""
        hold = self._hold(release_polls=100)       # keep it held for the test
        self._step(hold, [person_at(0.7, cx=0.5)], True)
        for z in walk_away(0.8, 3.0):
            self._step(hold, [person_at(z, cx=0.5)], False)
            self.assertTrue(hold.snapshot().chain_present)
        big = det("ambiguous", 0.0, (0.31, 0.0, 0.69, 1.0))       # 0.38 wide: no trigger
        self.assertTrue(self._step(hold, [big], False))
        self.assertFalse(hold.snapshot().chain_present)

    def test_equal_distance_tie_is_ambiguous(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6, cx=0.5)], True)
        a, b = person_at(0.7, cx=0.4), person_at(0.7, cx=0.6)
        self.assertTrue(self._step(hold, [a, b], False))
        snap = hold.snapshot()
        self.assertEqual((snap.operator_only, snap.operator_only_reason), (True, "ambiguous"))

    def test_second_close_object_later_is_operator_only(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6, cx=0.3)], True)
        intruder = det("ok", 0.5, (0.62, 0.0, 0.94, 1.0))
        self.assertTrue(hold.update(
            now=self.t + 0.067, triggered=True, detections=[person_at(0.7, cx=0.3), intruder],
            fresh_detections=True, corridor_mm=INF, corridor_rejected=True,
            corridor_valid_pct=0.0, stop_det=intruder, trigger_dets=[intruder]))
        self.assertEqual(hold.snapshot().operator_only_reason, "second_trigger")

    def test_one_frame_false_positive_does_not_latch(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6, cx=0.6)], True)
        fp = det("no_support", 0.0, (0.05, 0.5, 0.12, 0.8))
        released = False
        for i, z in enumerate(walk_away(0.7, 2.5)):
            dets = [person_at(z, cx=0.6)] + ([fp] if i == 2 else [])
            if not self._step(hold, dets, False):
                released = True
                break
        self.assertTrue(released)

    def test_two_poll_false_positive_latches(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6, cx=0.6)], True)
        fp = det("no_support", 0.0, (0.05, 0.5, 0.12, 0.8))
        for i, z in enumerate(walk_away(0.7, 3.0)):
            dets = [person_at(z, cx=0.6)] + ([fp] if i in (2, 3) else [])
            self.assertTrue(self._step(hold, dets, False))
        snap = hold.snapshot()
        self.assertEqual((snap.operator_only, snap.operator_only_reason), (True, "second_object"))

    def test_other_box_while_person_walks_away(self):
        """The case that grew the old track list: one box that matches nothing
        while the real person walks away in view. Seen beyond: ignored, the
        hold releases. Not seen beyond: operator-only, reported. Either way
        the state does not grow."""
        goat_far = det("ok", 3.5, (0.10, 0.6, 0.18, 0.75), label="cow")
        hold2 = self._hold()
        close = person_at(0.6, cx=0.6)
        self._step(hold2, [close, goat_far], True, stop_det=close)
        out = [self._step(hold2, [person_at(z, cx=0.6), goat_far], False)
               for z in walk_away(0.7, 2.5)]
        self.assertIn(False, out)
        hold3 = self._hold()
        goat_unmeasured = det("no_support", 0.0, (0.10, 0.6, 0.18, 0.75), label="cow")
        self._step(hold3, [close, goat_unmeasured], True, stop_det=close)
        n_attrs = len(vars(hold3))
        for z in walk_away(0.7, 2.5):
            self.assertTrue(self._step(hold3, [person_at(z, cx=0.6), goat_unmeasured], False))
        self.assertTrue(hold3.snapshot().operator_only)
        self.assertEqual(len(vars(hold3)), n_attrs)

    def test_bounded_state_and_flat_time(self):
        """10,000 polls of a stuck hold with moving boxes: the state stays the
        same size and the time per poll stays flat."""
        hold = self._hold()
        self._step(hold, [person_at(0.5)], True)

        def size(h):
            n = 0
            for v in list(vars(h).values()) + list(vars(h._chain or object()).values()
                                                    if h._chain is not None else []):
                if isinstance(v, (list, dict, set, tuple)):
                    n += len(v)
            return len(vars(h)), n

        s0 = size(hold)
        times = []
        for i in range(10000):
            k = i % 7
            dets = [person_at(0.5 + 0.05 * k, cx=0.1 + 0.11 * k),
                    det("no_support", 0.0, (0.05 * k, 0.2, 0.05 * k + 0.1, 0.6), label="dog"),
                    person_at(3.0, cx=0.9 - 0.1 * k)]
            t0 = time.perf_counter()
            self._step(hold, dets, False, dt=0.067)
            times.append(time.perf_counter() - t0)
        self.assertTrue(hold.active)
        self.assertEqual(size(hold), s0)
        first, last = sum(times[:1000]), sum(times[-1000:])
        self.assertLess(last, 3.0 * first + 0.01)

    def test_malformed_boxes_do_not_raise(self):
        hold = self._hold()
        bad = [
            det("ok", float("nan"), (0.3, 0.0, 0.6, 1.0)),
            det("ok", "far", (0.3, 0.0, 0.6, 1.0)),
            det("ok", 2.0, None),
            det("ok", 2.0, (0.3,)),
            det("ok", 2.0, ("a", 0.0, 0.6, 1.0)),
            det("ok", 2.0, (0.3, 0.0, float("inf"), 1.0)),
            SimpleNamespace(safety_tier="stop"),
            SimpleNamespace(safety_tier="stop", bbox=5),
        ]
        self.assertTrue(self._step(hold, [CENTRE_CLOSE] + bad, True, stop_det=CENTRE_CLOSE))
        for _ in range(10):
            self.assertTrue(self._step(hold, bad, False))
        # A malformed box as the trigger itself.
        hold2 = self._hold()
        no_box = bad[2]
        self.assertTrue(self._step(hold2, [no_box], True, stop_det=no_box))
        self.assertTrue(hold2.snapshot().operator_only)
        self.assertTrue(self._step(hold2, [], False, cmm=math.nan, rej=False, cvp=math.nan))
        # A None entry in the list, and a None list.
        self.assertTrue(self._step(hold, [None], False))
        self.assertTrue(self._step(hold, None, False))

    def test_carry_in_window(self):
        hold = self._hold(carry_in_s=1.0)
        self.assertFalse(self._step(hold, [CENTRE_CLOSE], True, autonomous=False))
        self.assertFalse(self._step(hold, [], False, autonomous=False, dt=0.9))
        self.assertTrue(self._step(hold, [], False, autonomous=True, dt=0.05))   # 0.95 s
        hold2 = self._hold(carry_in_s=1.0)
        self.assertFalse(self._step(hold2, [CENTRE_CLOSE], True, autonomous=False))
        self.assertFalse(self._step(hold2, [], False, autonomous=False, dt=1.0))
        self.assertFalse(self._step(hold2, [], False, autonomous=True, dt=0.05))  # 1.05 s

    def test_late_carry_in_still_resolves_on_the_same_person(self):
        """Entry 0.9 s after the trigger (beyond chain_gap_s): the seed must
        still continue on the person in view, or only the operator could
        release it (2026-11-07 12:18:50: FOLLOW_ME 0.5 s after a 0.69 m stop)."""
        hold = self._hold(carry_in_s=1.0)
        self._step(hold, [det("ok", 0.69, (0.68, 0.0, 0.999, 1.0))], True, autonomous=False)
        self._step(hold, [], False, autonomous=False, dt=0.85)
        far = det("ok", 1.3, (0.36, 0.0, 0.74, 1.0))
        self.assertTrue(self._step(hold, [far], False, autonomous=True, dt=0.05))
        self.assertEqual(hold.snapshot().reason, "carry_in")
        self.assertTrue(hold.snapshot().chain_present)
        results = [self._step(hold, [far], False) for _ in range(BEYOND_TO_RELEASE - 1)]
        self.assertEqual(results, [True] * (BEYOND_TO_RELEASE - 2) + [False])

    def test_carry_in_only_on_mode_entry(self):
        """A release inside the window must not re-engage on the next poll."""
        hold = self._hold(carry_in_s=1.0)
        self._step(hold, [person_at(0.6)], True)
        out = [self._step(hold, [person_at(z)], False) for z in walk_away(0.7, 1.6)]
        self.assertFalse(out[-1])
        self.assertFalse(self._step(hold, [person_at(1.7)], False))

    def test_manual_carry_operator_only_ends_on_neutral(self):
        hold = self._hold()
        self._step(hold, [person_at(0.6)], True)
        self._step(hold, [], False, dt=0.7)                         # lost
        self.assertTrue(hold.snapshot().operator_only)
        self.assertTrue(self._step(hold, [], False, autonomous=False, neutral=False))
        self.assertFalse(self._step(hold, [], False, autonomous=False, neutral=True))
        snap = hold.snapshot()
        self.assertEqual((snap.reason, snap.operator_only), ("manual_neutral", False))

    def test_manual_default_never_engages(self):
        hold = self._hold()
        for _ in range(20):
            self.assertFalse(self._step(hold, [CENTRE_CLOSE], True, autonomous=False))
        self.assertFalse(hold.active)


# --------------------------------------------------------------------------
# MANUAL "no drive command" timer and the RC-live helper
# --------------------------------------------------------------------------

class TestNeutralTimer(unittest.TestCase):
    def _run(self, seq, hold_s=0.25, dt=1 / 30):
        nt = NeutralTimer(band_byte=6.0, hold_s=hold_s)
        return [nt.update(i * dt, True, l, r) for i, (l, r) in enumerate(seq)]

    def test_reverse_is_not_neutral(self):
        self.assertFalse(any(self._run([(100, 100)] * 60)))

    def test_pivot_is_not_neutral(self):
        self.assertFalse(any(self._run([(100, 152)] * 60)))

    def test_one_tick_zero_crossing_is_not_neutral(self):
        seq = [(170, 170)] * 10 + [(126, 126)] + [(90, 90)] * 20
        self.assertFalse(any(self._run(seq)))

    def test_real_let_go_releases_after_hold(self):
        out = self._run([(170, 170)] * 5 + [(127, 125)] * 12)
        self.assertFalse(any(out[:5 + 7]))     # 7 ticks = 0.23 s < 0.25 s
        self.assertTrue(out[-1])               # 12 ticks = 0.37 s

    def test_unknown_or_not_manual_is_not_neutral(self):
        nt = NeutralTimer(6.0, 0.25)
        self.assertFalse(nt.update(0.0, True, None, 126))
        self.assertFalse(nt.update(1.0, False, 126, 126))
        self.assertFalse(nt.update(2.0, True, 126, 126))
        self.assertFalse(nt.update(2.1, True, None, None))   # resets
        self.assertFalse(nt.update(2.2, True, 126, 126))
        self.assertTrue(nt.update(2.5, True, 126, 126))
        self.assertFalse(nt.update(2.6, True, True, 126))    # bool is not a byte

    def test_reader_carry_with_reverse_then_pivot_stays_held(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG])
        nt = NeutralTimer(6.0, 0.25)
        t = 0.0
        for l, r in [(100, 100)] * 15 + [(100, 152)] * 15 + [(126, 126)] + [(170, 170)] * 5:
            t += 1 / 15
            self.assertEqual(h.poll(CLEAR, [], autonomous=False,
                                    neutral=nt.update(t, True, l, r)), 0.0)
        for _ in range(5):
            t += 1 / 15
            d = h.poll(CLEAR, [], autonomous=False, neutral=nt.update(t, True, 126, 126))
        self.assertAlmostEqual(d, 3.0, places=3)


class TestRcLinkLive(unittest.TestCase):
    def test_rc_link_live(self):
        self.assertFalse(rc_link_live(0.0, 10.0, 1.0))        # no frame yet
        self.assertFalse(rc_link_live(None, 10.0, 1.0))
        self.assertTrue(rc_link_live(9.5, 10.0, 1.0))
        self.assertTrue(rc_link_live(9.0, 10.0, 1.0))
        self.assertFalse(rc_link_live(8.9, 10.0, 1.0))
        self.assertFalse(rc_link_live(10.5, 10.0, 1.0))       # future stamp
        self.assertFalse(rc_link_live(float("nan"), 10.0, 1.0))


# --------------------------------------------------------------------------
# Controller: the held 0.0 zeroes forward only; a pivot still passes
# --------------------------------------------------------------------------

class _Motor(MotorDriver):
    def set_tracks(self, left_byte, right_byte):
        pass

    def stop(self):
        pass


class _Relay(ArmRelay):
    def set_armed(self, armed):
        pass


class _Shutdown(ShutdownScheduler):
    def schedule_shutdown(self, delay_seconds):
        pass


def _rc(ch1=1500, ch2=1500):
    return RCInputs(ch1_us=ch1, ch2_us=ch2, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                    last_update_mono_s=time.monotonic())


class TestHeldStopPassesPivot(unittest.TestCase):
    def _armed(self):
        c = Controller(
            motor_driver=_Motor(), arm_relay=_Relay(), shutdown_scheduler=_Shutdown(),
            obstacle_avoidance=ObstacleAvoidanceController(default_config.obstacle_avoidance),
            safety_params=SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0),
        )
        c.process(_rc(), now_epoch_s=100.0)
        return c

    def test_drive_cmd_bytes_rc_and_phone(self):
        """The 'no drive command' signal for a hold carried into MANUAL is the
        commanded track bytes before the obstacle layer, RC or phone."""
        c = self._armed()
        c.set_obstacle_data(0.0, 0.0)                 # held stop: output zeroed
        cmd, _, telem = c.process(_rc(2000, 2000), now_epoch_s=100.1)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (126, 126))
        self.assertGreater(telem["drive_cmd_left"], 140)
        _, _, telem = c.process(_rc(), now_epoch_s=100.2)
        self.assertEqual((telem["drive_cmd_left"], telem["drive_cmd_right"]), (126, 126))
        _, _, telem = c.process(_rc(), now_epoch_s=100.3, bt_override_bytes=(100, 152))
        self.assertNotEqual(telem["drive_cmd_left"], telem["drive_cmd_right"])

    def test_pivot_passes_forward_zeroed(self):
        h = _ReaderHarness()
        h.poll(CLEAR, [CENTRE_CLOSE])
        h.poll(CLEAR, [LEFT_CLIPPED_AMBIG])
        dist = h.poll(CLEAR, [])
        self.assertEqual(dist, 0.0)
        c = self._armed()
        c.set_obstacle_data(dist, 0.0)
        cmd, _, _ = c.process(_rc(2000, 2000), now_epoch_s=100.1)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
        c = self._armed()
        c.set_obstacle_data(dist, 0.0)
        cmd, _, _ = c.process(_rc(2000, 1000), now_epoch_s=100.1)
        # The pivot passes: tracks opposite, the common mode at neutral (one
        # byte of rounding from the half-byte differential).
        self.assertGreater(abs(cmd.left_byte - cmd.right_byte), 40)
        self.assertLessEqual(abs(cmd.left_byte + cmd.right_byte - 2 * CENTER_OUTPUT_VALUE), 1)


if __name__ == "__main__":
    unittest.main()
