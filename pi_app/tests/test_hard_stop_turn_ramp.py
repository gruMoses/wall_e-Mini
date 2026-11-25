"""Hard stop: the forward part stops at once, the turn part ramps.

Status audit 2026-11-25, section 2, item 6; Kevin's decision 2026-11-25.
Before: a hard stop (scale 0 on a forward command, or the e-stop) copied
the target to the motors with no slew. The forward part was zero, but the
turn reached the motors in one tick: a stick held forward and to one side
at the 0.8 m stop became a full pivot at once, and a track could jump from
one side of neutral to the other.

Now, in every mode: a step toward neutral passes at once (never across
it), a track that would cross neutral stops at 126 for that tick, a step
away from neutral ramps at the mode's accel rate, and the pair never has
more forward (or reverse) than the target. The pivot still passes (Kevin,
2026-11-08), ramped. These tests use the PRODUCTION slew config.
"""

import unittest
from unittest.mock import patch

from config import ObstacleAvoidanceConfig, FollowMeConfig, config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE as C
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.safety import SafetyParams
from pi_app.control.state import AutonomyCommand

SLEW = default_config.slew_limiter
DT = 0.04  # 25 Hz: integer slew steps at the production rates


def _step(bps: float) -> int:
    return int(round(bps * DT))


MANUAL_STEP = _step(SLEW.manual_accel_bps)
FOLLOW_STEP = _step(SLEW.follow_me_accel_bps)


class _Motor:
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_tracks(self, left_byte, right_byte):
        self.commands.append((left_byte, right_byte))

    def stop(self):
        self.stops += 1


class _Relay:
    def set_armed(self, armed):
        pass


class _Shutdown:
    def __init__(self):
        self.scheduled = []

    def schedule_shutdown(self, delay_seconds):
        self.scheduled.append(delay_seconds)


def _net(out):
    return (out[0] - C) + (out[1] - C)


def _crossed(a, b):
    return (a - C) * (b - C) < 0


class _Harness:
    """Armed controller on a fake monotonic clock, one tick = DT."""

    def __init__(self, test, follow_me=None):
        self.test = test
        self.clock = {"t": 10.0}
        self.dist = 5.0
        self.detections = None
        self._patch = patch("pi_app.control.controller.time.monotonic",
                            side_effect=lambda: self.clock["t"])
        self._patch.start()
        test.addCleanup(self._patch.stop)
        self.shutdown = _Shutdown()
        self.c = Controller(
            motor_driver=_Motor(), arm_relay=_Relay(), shutdown_scheduler=self.shutdown,
            obstacle_avoidance=ObstacleAvoidanceController(ObstacleAvoidanceConfig()),
            follow_me=follow_me,
            safety_params=SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0),
        )

    def tick(self, ch1=1500, ch2=1500, ch4=1000, ch5=1000):
        self.clock["t"] += DT
        self.c.set_obstacle_data(distance_m=self.dist, age_s=0.0)
        if self.detections is not None:
            self.c.set_person_detections(self.detections)
        rc = RCInputs(ch1_us=ch1, ch2_us=ch2, ch3_us=1900, ch4_us=ch4, ch5_us=ch5,
                      last_update_mono_s=self.clock["t"])
        cmd, events, telem = self.c.process(rc, now_epoch_s=self.clock["t"])
        return (cmd.left_byte, cmd.right_byte), telem, cmd, events


class TestProductionConfig(unittest.TestCase):
    def test_config_this_file_relies_on(self):
        self.assertTrue(SLEW.enabled)
        self.assertTrue(SLEW.bypass_on_hard_stop)
        self.assertEqual(SLEW.hard_stop_scale_threshold, 0.0)
        self.assertTrue(SLEW.snap_first_command)
        self.assertFalse(SLEW.snap_first_follow_me)
        self.assertGreater(MANUAL_STEP, 0)
        self.assertGreater(FOLLOW_STEP, 0)


class TestHardStopTracksRule(unittest.TestCase):
    """The pure rule, with explicit steps (accel 10, decel 14 bytes)."""

    A, D = 10.0, 14.0

    def _run(self, prev, target):
        return Controller._hard_stop_tracks(prev[0], prev[1], target[0], target[1],
                                            self.A, self.D)

    def test_reduction_toward_neutral_is_immediate(self):
        self.assertEqual(self._run((200, 200), (C, C)), (C, C))
        self.assertEqual(self._run((160, 92), (150, 102)), (150, 102))
        self.assertEqual(self._run((60, 60), (C, C)), (C, C))

    def test_emergency_target_neutral_is_immediate_from_anywhere(self):
        for prev in [(254, 0), (0, 254), (230, 20), (200, 150), (80, 90), (C, 254)]:
            self.assertEqual(self._run(prev, (C, C)), (C, C), prev)

    def test_track_that_would_cross_stops_at_neutral(self):
        # Pivot right to pivot left: both tracks cross; both stop at 126.
        self.assertEqual(self._run((160, 92), (92, 160)), (C, C))

    def test_forward_and_turn_stop_gives_zero_forward_and_ramps_the_turn(self):
        # Steady forward right turn (200, 150) meets the stop: target pivot
        # (151, 101). R would cross; L may drop at once; forward stays zero.
        out = self._run((200, 150), (151, 101))
        self.assertEqual(out, (C, C))
        nxt = self._run(out, (151, 101))
        self.assertEqual(nxt, (C + 10, C - 10))

    def test_increase_away_from_neutral_ramps_at_accel(self):
        self.assertEqual(self._run((C, C), (180, 72)), (C + 10, C - 10))
        self.assertEqual(self._run((150, 102), (180, 72)), (160, 92))

    def test_ramped_reverse_track_takes_forward_off_the_other_track(self):
        # (206, 86) = forward 20 + turn 60. Target pivot (186, 66): R must
        # ramp 86 -> 76, so L drops to 176 (not 186) to keep forward zero.
        out = self._run((206, 86), (186, 66))
        self.assertEqual(out, (176, 76))
        self.assertEqual(_net(out), 0)

    def test_ramped_forward_track_takes_reverse_off_the_other_track(self):
        # From (126, 110) to pivot (151, 101): L ramps to 136, R may reach
        # 101 at once, but that leaves reverse; R stops at 116 instead.
        out = self._run((C, 110), (151, 101))
        self.assertEqual(out, (136, 116))
        self.assertEqual(_net(out), 0)

    def test_no_step_ever_crosses_grows_too_fast_or_adds_forward(self):
        values = [0, 40, 90, 120, 125, C, 127, 140, 170, 210, 254]
        for pl in values:
            for pr in values:
                for tl in values:
                    tr = 2 * C - tl  # hard-stop targets have no forward part
                    if not 0 <= tr <= 254:
                        continue
                    out = self._run((pl, pr), (tl, tr))
                    self.assertEqual(_net(out), 0, (pl, pr, tl, tr, out))
                    for p, t, o in ((pl, tl, out[0]), (pr, tr, out[1])):
                        self.assertFalse(_crossed(p, o), (p, t, o))
                        self.assertLessEqual(abs(o - C), max(abs(p - C), abs(t - C)))
                        if abs(o - C) > abs(p - C):
                            self.assertLessEqual(abs(o - C) - abs(p - C), self.A)
                            self.assertEqual((o - C) * (t - C) > 0, True)


class TestManualHardStop(unittest.TestCase):
    """MANUAL: ch1 = left track, ch2 = right track."""

    FWD_RIGHT = (2000, 1700)  # forward, turning right: both tracks forward

    def _drive_to_steady(self, h, sticks, n=40):
        h.tick()  # arm at rest
        out = None
        for _ in range(n):
            out, telem, _, _ = h.tick(*sticks)
        self.assertEqual(out, (telem["slew_in_left"], telem["slew_in_right"]))
        return out

    def test_forward_turn_at_stop_zeroes_forward_at_once_and_ramps_turn(self):
        h = _Harness(self)
        before = self._drive_to_steady(h, self.FWD_RIGHT)
        self.assertGreater(before[0], C)
        self.assertGreater(before[1], C)
        h.dist = 0.0  # person/animal stop tier
        outs, targets = [], []
        for _ in range(25):
            out, telem, _, _ = h.tick(*self.FWD_RIGHT)
            self.assertTrue(telem["slew_hard_stop_active"])
            outs.append(out)
            targets.append((telem["slew_in_left"], telem["slew_in_right"]))
        target = targets[0]
        self.assertEqual(_net(target), 0)
        self.assertGreater(target[0] - target[1], 4 * MANUAL_STEP)  # a real turn
        # Forward zero from the first stop tick on.
        for out in outs:
            self.assertEqual(_net(out), 0, outs)
        # The turn ramps: first tick far below the target turn, then the
        # turn grows by at most one accel step per track each tick.
        self.assertLess(outs[0][0] - outs[0][1], target[0] - target[1])
        prev = outs[0]
        for out in outs[1:]:
            self.assertLessEqual(abs(out[0] - prev[0]), MANUAL_STEP + 1)
            self.assertLessEqual(abs(out[1] - prev[1]), MANUAL_STEP + 1)
            self.assertGreaterEqual(out[0] - out[1], prev[0] - prev[1])
            prev = out
        # And the pivot passes in full (Kevin, 2026-11-08).
        self.assertEqual(outs[-1], target)

    def test_track_that_would_cross_neutral_ramps_through_126(self):
        h = _Harness(self)
        before = self._drive_to_steady(h, self.FWD_RIGHT)
        h.dist = 0.0
        rights = [before[1]]
        target_r = None
        for _ in range(25):
            out, telem, _, _ = h.tick(*self.FWD_RIGHT)
            rights.append(out[1])
            target_r = telem["slew_in_right"]
        self.assertGreater(rights[0], C)
        self.assertLess(target_r, C)
        self.assertEqual(rights[1], C)  # stops at neutral on the crossing tick
        for a, b in zip(rights, rights[1:]):
            self.assertFalse(_crossed(a, b), rights)
            if abs(b - C) > abs(a - C):
                self.assertLessEqual(abs(b - C) - abs(a - C), MANUAL_STEP + 1)
        self.assertEqual(rights[-1], target_r)

    def test_turn_reversal_inside_the_stop_goes_through_neutral(self):
        h = _Harness(self)
        self._drive_to_steady(h, self.FWD_RIGHT)
        h.dist = 0.0
        out = None
        for _ in range(25):
            out, _, _, _ = h.tick(*self.FWD_RIGHT)
        self.assertGreater(out[0], C)
        self.assertLess(out[1], C)
        seq = [out]
        for _ in range(25):
            out, telem, _, _ = h.tick(1700, 2000)  # forward, turning left
            seq.append(out)
        self.assertEqual(seq[1], (C, C))
        for a, b in zip(seq, seq[1:]):
            self.assertEqual(_net(b), 0)
            self.assertFalse(_crossed(a[0], b[0]), seq)
            self.assertFalse(_crossed(a[1], b[1]), seq)
        self.assertEqual(seq[-1], (telem["slew_in_left"], telem["slew_in_right"]))
        self.assertLess(seq[-1][0], C)
        self.assertGreater(seq[-1][1], C)

    def test_pure_reduction_toward_neutral_is_immediate(self):
        # Straight forward at the stop: neutral in one tick.
        h = _Harness(self)
        self._drive_to_steady(h, (2000, 2000))
        h.dist = 0.0
        out, telem, _, _ = h.tick(2000, 2000)
        self.assertEqual(out, (C, C))
        self.assertTrue(telem["slew_bypassed"])
        # Inside the stop with the pivot established: a smaller turn, then
        # the stick released, each reach the motors in one tick.
        h2 = _Harness(self)
        self._drive_to_steady(h2, self.FWD_RIGHT)
        h2.dist = 0.0
        established = None
        for _ in range(25):
            established, _, _, _ = h2.tick(*self.FWD_RIGHT)
        out, telem, _, _ = h2.tick(1900, 1800)
        smaller = (telem["slew_in_left"], telem["slew_in_right"])
        self.assertLess(smaller[0] - smaller[1], established[0] - established[1])
        self.assertGreater(smaller[0], C)
        self.assertLess(smaller[1], C)
        self.assertEqual(out, smaller)
        self.assertTrue(telem["slew_bypassed"])
        out, _, _, _ = h2.tick(1500, 1500)
        self.assertEqual(out, (C, C))

    def test_first_command_after_arming_into_a_stop_ramps_the_turn(self):
        """snap_first_command is unchanged in the normal path; inside a hard
        stop the turn ramps on the first command after arming too."""
        h = _Harness(self)
        h.tick()  # arm at rest
        h.dist = 0.0
        out, telem, _, _ = h.tick(*self.FWD_RIGHT)
        self.assertEqual(_net(out), 0)
        self.assertLessEqual(out[0] - C, MANUAL_STEP)
        self.assertLessEqual(C - out[1], MANUAL_STEP)
        self.assertFalse(telem["slew_bypassed"])

    def test_snap_first_command_still_passes_outside_a_stop(self):
        h = _Harness(self)
        h.tick()  # arm at rest
        out, telem, _, _ = h.tick(*self.FWD_RIGHT)
        self.assertEqual(out, (telem["slew_in_left"], telem["slew_in_right"]))
        self.assertGreater(out[0] - C, MANUAL_STEP)

    def test_stop_release_still_ramps(self):
        """cce9b06: the release ramps from what was emitted, never snaps."""
        h = _Harness(self)
        self._drive_to_steady(h, self.FWD_RIGHT)
        h.dist = 0.0
        stopped = None
        for _ in range(25):
            stopped, _, _, _ = h.tick(*self.FWD_RIGHT)
        h.dist = 5.0
        seq = [stopped]
        for _ in range(30):
            out, telem, _, _ = h.tick(*self.FWD_RIGHT)
            self.assertFalse(telem["slew_hard_stop_active"])
            seq.append(out)
        stick = (telem["slew_in_left"], telem["slew_in_right"])
        self.assertNotEqual(seq[1], stick)
        for a, b in zip(seq, seq[1:]):
            self.assertLessEqual(abs(b[0] - a[0]), MANUAL_STEP + 1, seq)
            self.assertLessEqual(abs(b[1] - a[1]), int(round(SLEW.manual_decel_bps * DT)) + 1, seq)
        self.assertEqual(seq[-1], stick)

    def test_straight_stop_release_ramps_from_neutral(self):
        h = _Harness(self)
        self._drive_to_steady(h, (2000, 2000))
        h.dist = 0.0
        out, _, _, _ = h.tick(2000, 2000)
        self.assertEqual(out, (C, C))
        h.dist = 5.0
        out, _, _, _ = h.tick(2000, 2000)
        self.assertEqual(out, (C + MANUAL_STEP, C + MANUAL_STEP))

    def test_estop_while_turning_is_neutral_at_once(self):
        h = _Harness(self)
        self._drive_to_steady(h, self.FWD_RIGHT)
        out, _, cmd, _ = h.tick(*self.FWD_RIGHT, ch5=2000)
        self.assertEqual(out, (C, C))
        self.assertTrue(cmd.emergency_active)
        self.assertEqual(h.shutdown.scheduled, [5.0])

    def test_emergency_in_the_hard_stop_branch_is_neutral_at_once(self):
        """update_safety disarms on the e-stop, so the armed branch never
        sees emergency_active in service. Force it to cover the clause."""
        h = _Harness(self)
        self._drive_to_steady(h, self.FWD_RIGHT)
        h.c._safety_state.emergency_active = True
        out, telem, cmd, _ = h.tick(1500, 1500)
        self.assertTrue(cmd.is_armed)
        self.assertTrue(telem["slew_hard_stop_active"])
        self.assertEqual(out, (C, C))


class TestFollowMeHardStop(unittest.TestCase):
    """FOLLOW_ME: the pivot passes through the 0.8 m stop (Kevin,
    2026-11-08), now ramped at the FOLLOW_ME rates."""

    TARGET = PersonDetection(x_m=0.0, z_m=2.0, confidence=0.9, bbox=(0.45, 0.0, 0.55, 0.8))

    def _enter(self, h):
        h.detections = [self.TARGET]
        h.tick()  # arm
        h.tick(ch4=1900)
        self.assertEqual(h.c._mode, "FOLLOW_ME")

    def test_follow_me_pivot_during_stop_passes_ramped(self):
        h = _Harness(self, follow_me=FollowMeController(FollowMeConfig()))
        self._enter(h)
        # Pivot right with a little forward (net forward, so the stop applies).
        fm_cmd = AutonomyCommand(source="test", left_byte=170, right_byte=96,
                                 steering_input=0.5)
        h.c._compute_follow_me_autonomy_command = lambda telemetry, mono_now: fm_cmd
        before = (h.c._slew_last_left, h.c._slew_last_right)
        h.dist = 0.0
        outs = []
        for _ in range(20):
            out, telem, _, _ = h.tick(ch4=1900)
            self.assertEqual(telem["mode"], "FOLLOW_ME")
            self.assertTrue(telem["slew_hard_stop_active"])
            outs.append(out)
        target = (telem["slew_in_left"], telem["slew_in_right"])
        self.assertEqual(target, (163, 89))  # forward removed, turn kept
        prev = before
        for out in outs:
            self.assertEqual(_net(out), 0, outs)
            for p, o in zip(prev, out):
                self.assertFalse(_crossed(p, o), outs)
                self.assertLessEqual(abs(o - C) - abs(p - C), FOLLOW_STEP + 1, outs)
            prev = out
        self.assertLess(outs[0][0] - outs[0][1], target[0] - target[1])
        self.assertEqual(outs[-1], target)  # the pivot is not frozen


if __name__ == "__main__":
    unittest.main()
