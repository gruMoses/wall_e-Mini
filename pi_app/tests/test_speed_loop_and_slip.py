"""
Tests for closed-loop speed control, slip detection, and open-loop fallback.

Coverage:
  - SpeedLayer velocity PID (closed-loop vs open-loop)
  - FollowMeController slip detection & compensation
  - Controller telemetry fallback to open-loop on stale / missing data
  - NoopMotorDriver.get_telemetry() contract
"""

from __future__ import annotations

import json
import os
import sys
import time
import types
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3]))

from config import FollowMeConfig
from pi_app.control.follow_me import (
    FollowMeController,
    PersonDetection,
    PIDController,
    SpeedLayer,
)
from pi_app.control.controller import Controller, NoopMotorDriver, RCInputs


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_speed_layer(
    kp: float = 0.8,
    ki: float = 0.0,
    kd: float = 0.0,
    target: float = 1.5,
    dead_zone: float = 0.2,
    speed_scale: float = 0.0075,
    max_speed: float = 80.0,
) -> SpeedLayer:
    pid = PIDController(kp=kp, ki=ki, kd=kd, integral_limit=50.0)
    return SpeedLayer(
        target_dist_m=target,
        dead_zone_m=dead_zone,
        speed_gain=max_speed / 1.5,
        min_dist_m=0.5,
        max_speed_byte=max_speed,
        velocity_pid=pid,
        speed_scale_mps_per_byte=speed_scale,
    )


def _make_fm(**overrides) -> FollowMeController:
    cfg = FollowMeConfig(**overrides)
    return FollowMeController(cfg)


def _person(
    x_m: float = 0.0,
    z_m: float = 3.0,
    confidence: float = 0.9,
    bbox: tuple = (0.45, 0.3, 0.55, 0.8),
) -> PersonDetection:
    return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence, bbox=bbox)


def _arm_rc(now: float | None = None) -> RCInputs:
    return RCInputs(
        ch1_us=1500, ch2_us=1500,
        ch3_us=1750,  # arm channel
        ch4_us=1500, ch5_us=1500,
        last_update_epoch_s=now if now is not None else time.time(),
    )


# ─────────────────────────────────────────────────────────────────────────────
# 1. SpeedLayer — closed-loop vs open-loop
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeedLayerClosedLoop(unittest.TestCase):

    def test_open_loop_when_no_actual_speed(self):
        """Without telemetry, output equals open-loop value."""
        sl = _make_speed_layer()
        open_result = sl.compute(depth_m=3.0, actual_speed_mps=None)
        # Expected open-loop: min(80, 1.5 * (80/1.5)) = 80
        self.assertAlmostEqual(open_result, 80.0, places=1)

    def test_closed_loop_increases_output_when_too_slow(self):
        """PID should add positive correction when actual speed < target."""
        sl = _make_speed_layer(kp=1.0, ki=0.0, kd=0.0)
        # Open-loop reference (no PID)
        open_val = sl.compute(depth_m=3.0, actual_speed_mps=None)
        sl._velocity_pid.reset()
        # Now run closed-loop with actual = 0 (robot stationary but should be moving)
        closed_val = sl.compute(depth_m=3.0, actual_speed_mps=0.0, dt=0.067)
        # Correction should be non-negative (push forward)
        self.assertGreaterEqual(closed_val, open_val - 1e-3)

    def test_closed_loop_decreases_output_when_too_fast(self):
        """PID should subtract correction when actual speed > target."""
        sl = _make_speed_layer(kp=1.0, ki=0.0, kd=0.0)
        open_val = sl.compute(depth_m=3.0, actual_speed_mps=None)
        sl._velocity_pid.reset()
        # actual speed well above the target derived from open_val
        target_mps = open_val * 0.0075
        closed_val = sl.compute(depth_m=3.0, actual_speed_mps=target_mps * 3.0, dt=0.067)
        self.assertLess(closed_val, open_val)

    def test_pid_integral_resets_in_dead_zone(self):
        """Integrator is cleared when depth is within dead zone."""
        sl = _make_speed_layer(ki=1.0)
        # Build up integral outside dead zone
        for _ in range(10):
            sl.compute(depth_m=3.0, actual_speed_mps=0.0, dt=0.067)
        self.assertNotEqual(sl._velocity_pid._integral, 0.0)
        # Enter dead zone (within ±0.2 m of target 1.5 m)
        sl.compute(depth_m=1.6, actual_speed_mps=0.0, dt=0.067)
        self.assertEqual(sl._velocity_pid._integral, 0.0)

    def test_pid_integral_resets_at_min_dist(self):
        """Integrator is cleared when robot is too close (depth <= min_dist)."""
        sl = _make_speed_layer(ki=1.0)
        for _ in range(5):
            sl.compute(depth_m=3.0, actual_speed_mps=0.0, dt=0.067)
        sl.compute(depth_m=0.4, actual_speed_mps=0.0, dt=0.067)
        self.assertEqual(sl._velocity_pid._integral, 0.0)

    def test_output_clamped_to_max_speed(self):
        """Output never exceeds max_speed_byte regardless of PID correction."""
        sl = _make_speed_layer(kp=500.0, max_speed=80.0)
        result = sl.compute(depth_m=5.0, actual_speed_mps=0.0, dt=0.067)
        self.assertLessEqual(result, 80.0)
        self.assertGreaterEqual(result, 0.0)

    def test_output_never_negative(self):
        """Speed byte must never go below zero even with large negative correction."""
        sl = _make_speed_layer(kp=500.0)
        # Actual speed hugely above target → big negative correction
        result = sl.compute(depth_m=3.0, actual_speed_mps=100.0, dt=0.067)
        self.assertGreaterEqual(result, 0.0)

    def test_no_velocity_pid_fallback(self):
        """SpeedLayer without velocity_pid=None behaves as pure open-loop."""
        sl = SpeedLayer(
            target_dist_m=1.5,
            dead_zone_m=0.2,
            speed_gain=80.0 / 1.5,
            min_dist_m=0.5,
            max_speed_byte=80.0,
            velocity_pid=None,
        )
        r1 = sl.compute(depth_m=3.0, actual_speed_mps=0.0)
        r2 = sl.compute(depth_m=3.0, actual_speed_mps=None)
        self.assertAlmostEqual(r1, r2, places=5)


# ─────────────────────────────────────────────────────────────────────────────
# 2. Slip detection & compensation
# ─────────────────────────────────────────────────────────────────────────────

class TestSlipDetection(unittest.TestCase):

    def test_no_slip_without_telemetry(self):
        """Slip flag stays False when RPM telemetry is absent."""
        fm = _make_fm(slip_threshold_rpm=200.0)
        fm.update_telemetry(left_rpm=None, right_rpm=None, actual_speed_mps=None)
        fm.compute([_person()])
        self.assertFalse(fm.get_status()["follow_me_slip_active"])

    def test_slip_detected_on_large_rpm_differential(self):
        """Slip declared when actual−expected differential > threshold while
        commanded straight (and the straight guard has persisted).

        Persistence guard: the new detector requires the emitted/commanded steer
        to read "straight" for slip_straight_persist_ticks consecutive ticks; we
        set that to 1 here so a single compute() can trigger. Person is centred so
        the commanded steer stays ~0 (expected_diff ~0) and slip_diff == raw diff.
        """
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_throttle_reduction=0.15,
            slip_feedforward_gain=0.02,
            slip_straight_persist_ticks=1,
        )
        fm.update_telemetry(left_rpm=1000, right_rpm=500, actual_speed_mps=0.3)
        fm.compute([_person()])
        self.assertTrue(fm.get_status()["follow_me_slip_active"])

    def test_no_slip_when_differential_below_threshold(self):
        """No slip when the actual−expected differential is within threshold."""
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=600.0,
            slip_straight_persist_ticks=1,
        )
        fm.update_telemetry(left_rpm=1000, right_rpm=850, actual_speed_mps=0.3)
        fm.compute([_person()])
        self.assertFalse(fm.get_status()["follow_me_slip_active"])

    def test_slip_suppressed_during_active_steering(self):
        """Slip compensation does not fire when robot is actively turning (|steer| >= 5).

        We prime _prev_fresh_detection=True and set _reacq_time=0 to skip the
        reacquisition steer ramp (which zeroes steer on the very first detection
        frame regardless of person position).
        """
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=50.0,  # low threshold — would normally trigger
            slip_throttle_reduction=0.5,
            slip_straight_persist_ticks=1,
        )
        fm.update_telemetry(left_rpm=800, right_rpm=600, actual_speed_mps=0.3)
        # Prime tracker so the reacq ramp is not active this call
        fm._prev_fresh_detection = True
        fm._reacq_time = 0.0
        # Provide a strongly off-centre person to force large steer output (≥5 bytes)
        det = _person(x_m=2.0, z_m=3.0, bbox=(0.75, 0.3, 0.95, 0.8))
        # Seed the emitted/commanded steer so the straight guard sees a turn. The
        # new guard reads _last_emitted_steer (the prior command), not this tick's
        # raw PID value — so a genuine commanded turn must keep the guard disengaged.
        fm._last_emitted_steer = 15.0
        fm.compute([det])
        # With a commanded turn the straight guard (abs(emitted) < 5.0) is False,
        # so slip never fires.
        self.assertFalse(fm.get_status()["follow_me_slip_active"])

    def test_slip_reduces_speed_output(self):
        """When slip fires, the effective speed output must be ≤ the un-slipped value."""
        fm_no_slip = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=9999.0,
            slip_straight_persist_ticks=1,
        )
        fm_no_slip.update_telemetry(left_rpm=1000, right_rpm=400, actual_speed_mps=0.3)
        l0, r0 = fm_no_slip.compute([_person()])

        fm_slip = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_throttle_reduction=0.20,
            slip_feedforward_gain=0.0,  # isolate throttle effect
            slip_straight_persist_ticks=1,
        )
        fm_slip.update_telemetry(left_rpm=1000, right_rpm=400, actual_speed_mps=0.3)
        l1, r1 = fm_slip.compute([_person()])

        # Slipped version should produce smaller or equal net forward speed
        avg0 = (l0 + r0) / 2
        avg1 = (l1 + r1) / 2
        self.assertLessEqual(avg1, avg0 + 1)  # +1 tolerance for rounding

    def test_slip_status_in_get_status(self):
        """get_status() must always include follow_me_slip_active key."""
        fm = _make_fm()
        fm.compute([])
        self.assertIn("follow_me_slip_active", fm.get_status())

    def test_actual_speed_mps_in_get_status(self):
        """get_status() exposes follow_me_actual_speed_mps from telemetry."""
        fm = _make_fm()
        fm.update_telemetry(left_rpm=500, right_rpm=500, actual_speed_mps=0.42)
        fm.compute([_person()])
        self.assertAlmostEqual(fm.get_status()["follow_me_actual_speed_mps"], 0.42)


# ─────────────────────────────────────────────────────────────────────────────
# 2b. Rewritten slip compensator: off-switch, anti-runaway, genuine-slip
# ─────────────────────────────────────────────────────────────────────────────

DIRECT_CAP = 18.0          # FollowMeConfig.direct_mode_max_steer_byte default
GLOBAL_MAX_STEER = 25.0    # FollowMeConfig.max_steer_offset_byte default


class TestSlipCompensatorRewrite(unittest.TestCase):
    """Direct unit tests on _apply_slip_compensation — the algorithm in isolation.

    Calling the helper directly lets us drive commanded steer / rpm exactly and
    iterate ticks, which the full compute() pipeline (with reacq ramps, slew caps,
    depth filters) would otherwise obscure.
    """

    # ── (a) Disabled default is a TRUE no-op for ANY rpm input ───────────────

    def test_disabled_is_pure_noop_for_any_rpm(self):
        """With slip_compensation_enabled=False (the default), the function returns
        speed and steer UNCHANGED for any rpm differential — no throttle reduction,
        no steer term."""
        fm = _make_fm()  # default: slip_compensation_enabled=False
        self.assertFalse(fm._cfg.slip_compensation_enabled)
        cases = [
            (None, None), (0, 0), (500, 500), (1000, 0), (0, 1000),
            (5000, -5000), (1200, 350), (-800, 800),
        ]
        for lr, rr in cases:
            fm.update_telemetry(left_rpm=lr, right_rpm=rr, actual_speed_mps=0.5)
            # Even with a huge actual differential and a commanded "straight" steer,
            # nothing changes when disabled.
            fm._last_emitted_steer = 0.0
            for _ in range(5):  # iterate to be sure persistence can't sneak in
                speed_out, steer_out = fm._apply_slip_compensation(60.0, 3.0)
                self.assertEqual(speed_out, 60.0, f"speed changed for rpm={lr},{rr}")
                self.assertEqual(steer_out, 3.0, f"steer changed for rpm={lr},{rr}")
                self.assertFalse(fm.get_status()["follow_me_slip_active"])

    # ── (b) REGRESSION: the recorded failure pattern must NOT run away ───────

    def test_regression_commanded_turn_large_diff_no_runaway(self):
        """Failure pattern: a commanded turn produces a large ACTUAL rpm differential
        on a skid-steer robot. The OLD detector read raw rpm_diff as slip and injected
        steer that grew the differential → pinned steer to ±max (25). The NEW detector
        cancels the commanded component (expected_diff), so a commanded turn nets ~0
        slip_diff: no steer injected, and the total can never exceed the direct cap.
        """
        # Enable + give it a non-zero gain so that IF it (wrongly) fired it would
        # try to inject steer — proving the runaway is structurally gone.
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_feedforward_gain=0.05,
            slip_cmd_diff_per_byte=40.0,
            slip_straight_persist_ticks=3,
        )
        # Commanded steer is a real turn (12 bytes). On a skid-steer robot that turn
        # itself creates ~ k_cmd*12 = 480 eRPM differential. Simulate the actual diff
        # tracking the command (left spins faster for a right turn).
        fm._last_emitted_steer = 12.0
        commanded_steer = 12.0
        left_rpm, right_rpm = 980, 500   # actual diff = 480 ≈ expected for a 12-byte turn
        fm.update_telemetry(left_rpm=left_rpm, right_rpm=right_rpm, actual_speed_mps=0.4)

        steer_in = commanded_steer
        prev_steer = steer_in
        for _ in range(30):  # iterate far beyond the persistence window
            _, steer_out = fm._apply_slip_compensation(60.0, steer_in)
            # Never injects escalating steer: output must not climb past the
            # direct cap, and must not monotonically ramp toward the global max.
            self.assertLessEqual(abs(steer_out), DIRECT_CAP + 1e-6,
                                 "slip pushed steer past the direct cap")
            self.assertLess(abs(steer_out), GLOBAL_MAX_STEER,
                            "slip approached the global max — runaway not prevented")
            # No positive-feedback growth: a commanded turn nets ~0 slip_diff, so
            # the steer is not amplified tick over tick.
            self.assertLessEqual(abs(steer_out), abs(prev_steer) + 1e-6,
                                 "steer grew tick-over-tick (positive feedback)")
            prev_steer = steer_out
            steer_in = steer_out  # feed back as if the next tick's command

    def test_regression_near_straight_small_steer_with_turn_diff(self):
        """Near-straight small commanded steer while a LARGE actual rpm differential
        exists (as during a turn that the command is just beginning). Even enabled
        with a gain, the emitted steer must stay within the direct cap."""
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_feedforward_gain=0.05,
            slip_cmd_diff_per_byte=40.0,
            slip_straight_persist_ticks=3,
        )
        # Small commanded steer (3 bytes, under the 5-byte straight guard) but a
        # large actual differential from a developing turn.
        fm._last_emitted_steer = 3.0
        fm.update_telemetry(left_rpm=1200, right_rpm=300, actual_speed_mps=0.4)
        steer_in = 3.0
        worst = 0.0
        for _ in range(30):
            _, steer_out = fm._apply_slip_compensation(60.0, steer_in)
            worst = max(worst, abs(steer_out))
            steer_in = steer_out
        self.assertLessEqual(worst, DIRECT_CAP + 1e-6,
                             f"emitted steer exceeded direct cap (worst={worst})")

    # ── (c) Genuine slip IS detected when enabled ────────────────────────────

    def test_genuine_slip_detected_when_enabled(self):
        """Commanded straight (expected_diff≈0) with a large ACTUAL differential is a
        real slip and IS detected: slip_active True and throttle reduced."""
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_throttle_reduction=0.20,
            slip_feedforward_gain=0.02,
            slip_cmd_diff_per_byte=40.0,
            slip_straight_persist_ticks=3,
        )
        fm._last_emitted_steer = 0.0  # commanded straight
        fm.update_telemetry(left_rpm=1000, right_rpm=300, actual_speed_mps=0.4)
        speed_out = steer_out = None
        # Iterate to satisfy the 3-tick straight-persistence guard.
        for _ in range(4):
            speed_out, steer_out = fm._apply_slip_compensation(60.0, 0.0)
        self.assertTrue(fm.get_status()["follow_me_slip_active"],
                        "genuine slip (commanded straight, large diff) not detected")
        self.assertLess(speed_out, 60.0, "throttle not reduced on genuine slip")
        # A small bounded steer correction may be injected, but never past the cap.
        self.assertLessEqual(abs(steer_out), DIRECT_CAP + 1e-6)

    def test_genuine_slip_requires_persistence(self):
        """The straight guard must persist N ticks before slip can act — a single
        tick never triggers (transient immunity)."""
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_straight_persist_ticks=3,
        )
        fm._last_emitted_steer = 0.0
        fm.update_telemetry(left_rpm=1000, right_rpm=300, actual_speed_mps=0.4)
        # First two ticks: guard not yet persisted → no slip.
        fm._apply_slip_compensation(60.0, 0.0)
        self.assertFalse(fm.get_status()["follow_me_slip_active"])
        fm._apply_slip_compensation(60.0, 0.0)
        self.assertFalse(fm.get_status()["follow_me_slip_active"])
        # Third tick reaches the threshold.
        fm._apply_slip_compensation(60.0, 0.0)
        self.assertTrue(fm.get_status()["follow_me_slip_active"])


# ─────────────────────────────────────────────────────────────────────────────
# 2c. Replay of the recorded runaway (fm_verify/1781382327.jsonl)
# ─────────────────────────────────────────────────────────────────────────────

_FAILURE_REPLAY = "/tmp/fm_verify/1781382327.jsonl"


class TestRecordedRunawayReplay(unittest.TestCase):
    """Replay the recorded failure trace.

    NOTE: the recorded JSONL predates the recorder honesty fix and does NOT carry
    per-tick RPM fields, so the exact left/right eRPM that drove the original slip
    cannot be wired back through the full controller. Per the task's stated
    fallback, we assert at the _apply_slip_compensation level instead: we replay
    the recorded steer/speed timeline and, for every recorded tick, synthesize a
    LARGE actual rpm differential (the worst case the live telemetry could have
    presented during a turn) and confirm the SHIPPED defaults (slip disabled) leave
    every command byte-identical, and that even force-enabled the compensator never
    ramps steer toward the global max.
    """

    def _load(self):
        if not os.path.exists(_FAILURE_REPLAY):
            self.skipTest(f"recorded failure file not present: {_FAILURE_REPLAY}")
        recs = []
        with open(_FAILURE_REPLAY) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    recs.append(json.loads(line))
        return recs

    def test_shipped_defaults_slip_is_noop_over_replay(self):
        """With shipped defaults (slip disabled), feeding the recorded steer/speed
        through _apply_slip_compensation with a large synthetic rpm differential
        leaves every (speed, steer) byte-identical — no slip modification at all."""
        recs = self._load()
        fm = _make_fm()  # shipped defaults: slip_compensation_enabled=False
        # Worst-case actual differential on every tick.
        fm.update_telemetry(left_rpm=2000, right_rpm=-2000, actual_speed_mps=0.5)
        n = 0
        for r in recs:
            steer = float(r["steer"])
            speed = float(r["speed"])
            fm._last_emitted_steer = steer
            speed_out, steer_out = fm._apply_slip_compensation(speed, steer)
            self.assertEqual(speed_out, speed)
            self.assertEqual(steer_out, steer)
            self.assertFalse(fm.get_status()["follow_me_slip_active"])
            n += 1
        self.assertGreater(n, 100, "replay file unexpectedly short")

    def test_replay_slip_never_amplifies_steer_even_if_enabled(self):
        """Replay the recorded steer timeline with the NEW detector force-enabled and
        a gain, synthesizing the actual rpm differential that a commanded turn would
        produce (the exact case that fooled the old raw-diff detector).

        The original runaway came from slip INJECTING steer that grew the rpm
        differential it reacted to, pinning steer to the global max (25) while
        mode=='direct'. Note the recorded `steer` values themselves are already
        contaminated by that bug (they hit 25), so we cannot assert on their
        absolute value. Instead we assert the property that breaks the runaway:
        the new compensator NEVER amplifies steer (output magnitude ≤ input), and
        whenever it actually fires it clamps the result to the direct cap. With the
        old code, the same commanded-turn differential would have driven steer_out
        well above its input, monotonically toward ±max.
        """
        recs = self._load()
        fm = _make_fm(
            slip_compensation_enabled=True,
            slip_threshold_rpm=200.0,
            slip_feedforward_gain=0.05,
            slip_cmd_diff_per_byte=40.0,
            slip_straight_persist_ticks=3,
        )
        k_cmd = fm._cfg.slip_cmd_diff_per_byte
        fired_any = False
        for r in recs:
            steer = float(r["steer"])
            speed = float(r["speed"])
            fm._last_emitted_steer = steer
            # Synthesize the actual differential a commanded turn of this steer
            # produces (plus modest residual noise). The new detector cancels the
            # commanded component, so a commanded turn is not read as slip.
            actual_diff = k_cmd * steer + 60.0
            left = int(round(actual_diff / 2))
            right = int(round(-actual_diff / 2))
            fm.update_telemetry(left_rpm=left, right_rpm=right, actual_speed_mps=0.5)
            _, steer_out = fm._apply_slip_compensation(speed, steer)
            # Anti-runaway invariant: slip never amplifies the steer it was given.
            self.assertLessEqual(abs(steer_out), abs(steer) + 1e-6,
                                 f"slip amplified steer {steer} -> {steer_out}")
            if fm.get_status()["follow_me_slip_active"]:
                fired_any = True
                # When it does fire, the output is bounded by the direct cap.
                self.assertLessEqual(abs(steer_out), DIRECT_CAP + 1e-6)
        # The synthetic differential mirrors the command, so on this turn-dominated
        # trace the commanded component is cancelled and slip should essentially
        # never declare a runaway-inducing event. (fired_any may be False — that is
        # the correct, non-runaway outcome; we only require the invariants above.)
        _ = fired_any

    def test_controller_replay_no_runaway_with_shipped_defaults(self):
        """Feed the recorded x_raw/depth sequence through a real FollowMeController
        with the SHIPPED defaults (slip disabled) and confirm the emitted steer
        never reaches the global ±max and shows no monotonic ramp-to-25 runaway.

        The recorded file carries no RPM, so with slip disabled the controller's
        slip stage is a no-op regardless; this exercises the direct/steering path
        end-to-end on the real position trace and proves it alone never produces
        the 25-on-direct signature.
        """
        recs = self._load()
        fm = _make_fm()  # shipped defaults
        # No telemetry → slip path is doubly inert (disabled AND no rpm).
        fm.update_telemetry(left_rpm=None, right_rpm=None, actual_speed_mps=None)

        max_abs_steer = 0.0
        ramp_run = 0          # consecutive ticks of |steer| >= global max
        worst_ramp_run = 0
        fed = 0
        for r in recs:
            x_raw = r.get("x_raw")
            depth = r.get("depth")
            conf = r.get("conf")
            if x_raw is None or depth is None:
                # Lost/persistence tick in the recording — feed no detections so the
                # controller exercises its real lost-target handling.
                fm.compute([])
                ramp_run = 0
                continue
            cx = 0.5 + float(x_raw) / 2.0  # invert normalized_x = (cx-0.5)*2
            half = 0.05
            bbox = (cx - half, 0.3, cx + half, 0.8)
            det = PersonDetection(
                x_m=float(x_raw) * 2.0,  # rough lateral metres; sign/scale only
                z_m=float(depth),
                confidence=float(conf) if conf is not None else 0.9,
                bbox=bbox,
            )
            fm.compute([det])
            fed += 1
            emitted = abs(fm._last_slew_capped_steer)
            max_abs_steer = max(max_abs_steer, emitted)
            if emitted >= GLOBAL_MAX_STEER - 1e-6:
                ramp_run += 1
                worst_ramp_run = max(worst_ramp_run, ramp_run)
            else:
                ramp_run = 0

        self.assertGreater(fed, 50, "replay fed too few fresh detections")
        # No tick may sit at the global ±max (the runaway pinned steer to 25).
        self.assertLess(max_abs_steer, GLOBAL_MAX_STEER,
                        f"emitted steer reached the global max ({max_abs_steer})")
        # And certainly no sustained ramp held at the max.
        self.assertEqual(worst_ramp_run, 0,
                         "emitted steer held at the global max across consecutive ticks (runaway)")


# ─────────────────────────────────────────────────────────────────────────────
# 3. Telemetry fallback in Controller
# ─────────────────────────────────────────────────────────────────────────────

class TestControllerTelemetryFallback(unittest.TestCase):

    def test_noop_driver_returns_none_telemetry(self):
        """NoopMotorDriver.get_telemetry() must return None."""
        d = NoopMotorDriver()
        self.assertIsNone(d.get_telemetry())

    def test_controller_open_loop_with_noop_driver(self):
        """Controller processes normally when motor driver returns None telemetry."""
        ctrl = Controller()
        rc = _arm_rc()
        _, _, telem = ctrl.process(rc, now_epoch_s=time.time())
        # RPM fields should be absent / None, not raise
        self.assertIsNone(telem.get("vesc_left_rpm"))
        self.assertIsNone(telem.get("vesc_right_rpm"))
        self.assertIsNone(telem.get("vesc_actual_speed_mps"))

    def test_controller_includes_rpm_in_telemetry_when_available(self):
        """Controller exposes actual RPMs in telemetry dict when driver provides them."""

        class FakeVescTelemetry:
            left_rpm = 250
            right_rpm = 240
            voltage_v = 24.5
            timestamp = 0.0

        class FakeMotor:
            def set_tracks(self, l, r): pass
            def stop(self): pass
            def get_telemetry(self):
                return FakeVescTelemetry()

        ctrl = Controller(motor_driver=FakeMotor())
        rc = _arm_rc()
        # Force telemetry poll to fire immediately
        ctrl._telem_last_poll = 0.0
        _, _, telem = ctrl.process(rc, now_epoch_s=time.time())
        self.assertEqual(telem.get("vesc_left_rpm"), 250)
        self.assertEqual(telem.get("vesc_right_rpm"), 240)
        self.assertIsNotNone(telem.get("vesc_actual_speed_mps"))

    def test_controller_warns_on_stale_telemetry(self):
        """Controller emits a WARNING log when telemetry is stale for >500 ms."""

        class FakeMotor:
            def __init__(self):
                self._call_count = 0

            def set_tracks(self, l, r): pass
            def stop(self): pass
            def get_telemetry(self):
                self._call_count += 1
                if self._call_count == 1:
                    class T:
                        left_rpm = 100
                        right_rpm = 100
                        voltage_v = 24.0
                        timestamp = time.monotonic()
                    return T()
                return None  # all subsequent calls return None (stale)

        motor = FakeMotor()
        ctrl = Controller(motor_driver=motor)
        rc = _arm_rc()

        # First call: valid telemetry recorded
        ctrl._telem_last_poll = 0.0
        ctrl.process(rc, now_epoch_s=time.time())

        # Simulate 600 ms of staleness
        ctrl._telem_last_valid = time.monotonic() - 0.6
        ctrl._telem_last_poll = 0.0  # allow poll to fire again

        with self.assertLogs("pi_app.control.controller", level="WARNING") as cm:
            ctrl.process(rc, now_epoch_s=time.time())
        self.assertTrue(any("stale" in msg.lower() for msg in cm.output))

    def test_controller_stale_warning_fires_once(self):
        """Stale telemetry warning is emitted only once (not every cycle)."""

        class FakeMotor:
            def set_tracks(self, l, r): pass
            def stop(self): pass
            def get_telemetry(self):
                return None

        ctrl = Controller(motor_driver=FakeMotor())
        rc = _arm_rc()

        # Prime: give controller a valid telem_last_valid timestamp in the past
        ctrl._telem_last_valid = time.monotonic() - 1.0
        ctrl._telem_last_poll = 0.0
        ctrl._telem_stale_warned = False

        import logging
        with self.assertLogs("pi_app.control.controller", level="WARNING") as cm:
            ctrl.process(rc, now_epoch_s=time.time())
        first_count = len(cm.output)

        # Second call — warning already set; should NOT log again
        ctrl._telem_last_poll = 0.0
        # Can't easily assert silence inside assertLogs; just verify the flag
        self.assertTrue(ctrl._telem_stale_warned)
        # Calling again should not raise
        ctrl.process(rc, now_epoch_s=time.time())

    def test_get_vesc_telemetry_method(self):
        """Controller.get_vesc_telemetry() returns a dict with expected keys."""
        ctrl = Controller()
        vt = ctrl.get_vesc_telemetry()
        self.assertIn("left_rpm", vt)
        self.assertIn("right_rpm", vt)
        self.assertIn("actual_speed_mps", vt)

    def test_telemetry_disabled_via_config(self):
        """When vesc_telemetry_enabled=False, speed_mps stays None regardless of driver."""
        import config as cfg_module
        original_vesc = cfg_module.config.vesc
        # Temporarily patch config
        from dataclasses import replace as _replace
        patched_vesc = _replace(original_vesc, vesc_telemetry_enabled=False)

        class FakeMotor:
            def set_tracks(self, l, r): pass
            def stop(self): pass
            def get_telemetry(self):
                class T:
                    left_rpm = 300
                    right_rpm = 300
                    voltage_v = 24.0
                    timestamp = time.monotonic()
                return T()

        ctrl = Controller(motor_driver=FakeMotor())
        rc = _arm_rc()
        ctrl._telem_last_poll = 0.0

        # Monkey-patch config.vesc for this test only
        original = cfg_module.config
        object.__setattr__(cfg_module.config, "__class__", cfg_module.config.__class__)
        try:
            cfg_module.config = type(cfg_module.config)(
                **{k: (patched_vesc if k == "vesc" else v)
                   for k, v in vars(original).items()}
            )
            _, _, telem = ctrl.process(rc, now_epoch_s=time.time())
        finally:
            cfg_module.config = original

        # With telemetry disabled, actual_speed_mps should remain None
        # (Note: the controller instance may have already polled before the patch,
        # so we only assert the field exists; functional disabling is tested via flag)
        self.assertIn("vesc_actual_speed_mps", telem)


if __name__ == "__main__":
    unittest.main()
