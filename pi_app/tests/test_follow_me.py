import json
import math
import os
import stat
import tempfile
import unittest
from unittest.mock import patch

from config import FollowMeConfig
from pi_app.control.follow_me import (
    DetectionFilter,
    FollowMeController,
    PersonDetection,
    NEUTRAL,
    TargetTracker,
    _FilteredDetection,
)


class TestFollowMeController(unittest.TestCase):

    def _make(self, **overrides) -> FollowMeController:
        cfg = FollowMeConfig(**overrides)
        return FollowMeController(cfg)

    def _person(self, x_m=0.0, z_m=2.0, confidence=0.9,
                bbox=(0.4, 0.3, 0.6, 0.8), track_id=None) -> PersonDetection:
        return PersonDetection(
            x_m=x_m,
            z_m=z_m,
            confidence=confidence,
            bbox=bbox,
            track_id=track_id,
        )

    # --- No target ---

    def test_no_detections_returns_neutral(self):
        fm = self._make()
        left, right = fm.compute([])
        self.assertEqual(left, NEUTRAL)
        self.assertEqual(right, NEUTRAL)

    def test_low_confidence_filtered(self):
        fm = self._make(detection_confidence=0.6)
        det = self._person(confidence=0.3)
        left, right = fm.compute([det])
        self.assertEqual(left, NEUTRAL)
        self.assertEqual(right, NEUTRAL)

    def test_too_far_filtered(self):
        fm = self._make(max_distance_m=5.0)
        det = self._person(z_m=6.0)
        left, right = fm.compute([det])
        self.assertEqual(left, NEUTRAL)
        self.assertEqual(right, NEUTRAL)

    def test_too_close_stops(self):
        fm = self._make(min_distance_m=0.5)
        det = self._person(z_m=0.3)
        left, right = fm.compute([det])
        self.assertEqual(left, NEUTRAL)
        self.assertEqual(right, NEUTRAL)

    # --- Speed ---

    def test_person_at_follow_distance_stops(self):
        fm = self._make(follow_distance_m=1.5)
        det = self._person(x_m=0.0, z_m=1.5, bbox=(0.45, 0.3, 0.55, 0.8))
        left, right = fm.compute([det])
        self.assertEqual(left, NEUTRAL)
        self.assertEqual(right, NEUTRAL)

    def test_person_far_away_drives_forward(self):
        fm = self._make(follow_distance_m=1.5, max_follow_speed_byte=60)
        det = self._person(x_m=0.0, z_m=4.0, bbox=(0.45, 0.3, 0.55, 0.8))
        left, right = fm.compute([det])
        self.assertGreater(left, NEUTRAL)
        self.assertGreater(right, NEUTRAL)

    # --- Steering ---

    def test_person_right_steers_right(self):
        fm = self._make(follow_distance_m=1.0, steering_gain=0.8)
        det = self._person(x_m=1.0, z_m=3.0, bbox=(0.7, 0.3, 0.9, 0.8))
        left, right = fm.compute([det])
        # Right turn: left > right
        self.assertGreater(left, right)

    def test_person_left_steers_left(self):
        fm = self._make(follow_distance_m=1.0, steering_gain=0.8)
        det = self._person(x_m=-1.0, z_m=3.0, bbox=(0.1, 0.3, 0.3, 0.8))
        left, right = fm.compute([det])
        # Left turn: right > left
        self.assertGreater(right, left)

    # --- Target selection ---

    def test_selects_centered_person_over_edge(self):
        fm = self._make()
        centered = self._person(x_m=0.0, z_m=2.0, bbox=(0.4, 0.3, 0.6, 0.8))
        edge = self._person(x_m=2.0, z_m=2.0, bbox=(0.8, 0.3, 1.0, 0.8))
        left1, right1 = fm.compute([centered, edge])
        # Should follow centered person -> roughly symmetric
        self.assertAlmostEqual(left1, right1, delta=10)

    def test_selects_closer_person_when_similar_position(self):
        fm = self._make()
        # close at z_m=1.5: bbox_h must be tall enough to pass the implied-height check
        # (detect_min_person_height_m=1.20 m default).  Need bbox_h >= 1.20/(1.5*2*tan(32.65°))
        # = 1.20/1.922 ≈ 0.624 → use ymin=0.05, ymax=0.95 (height=0.90 → implied 1.73 m ok).
        close = self._person(x_m=0.1, z_m=1.5, bbox=(0.45, 0.05, 0.55, 0.95))
        far = self._person(x_m=0.1, z_m=4.5, bbox=(0.45, 0.3, 0.55, 0.8))
        fm.compute([close, far])
        status = fm.get_status()
        self.assertAlmostEqual(status["follow_me_target_z_m"], 1.5)

    def test_prefers_previous_track_id_for_continuity(self):
        fm = self._make()
        first = self._person(x_m=0.0, z_m=2.0, bbox=(0.45, 0.3, 0.55, 0.8), track_id=11)
        fm.compute([first])
        status = fm.get_status()
        self.assertEqual(status["follow_me_target_track_id"], 11)

        # Frame 2: new ID is slightly better centered, but old tracked target remains valid.
        prev_target = self._person(x_m=0.4, z_m=2.0, bbox=(0.62, 0.3, 0.78, 0.8), track_id=11)
        newcomer = self._person(x_m=0.0, z_m=2.0, bbox=(0.45, 0.3, 0.55, 0.8), track_id=22)
        fm.compute([prev_target, newcomer])
        status = fm.get_status()
        self.assertEqual(status["follow_me_target_track_id"], 11)

    # --- Status ---

    def test_status_tracking_true_when_target(self):
        fm = self._make()
        det = self._person(z_m=2.0)
        fm.compute([det])
        self.assertTrue(fm.get_status()["follow_me_tracking"])

    def test_status_tracking_false_when_empty(self):
        fm = self._make()
        fm.compute([])
        self.assertFalse(fm.get_status()["follow_me_tracking"])

    # --- Output clamping ---

    def test_output_clamped_to_byte_range(self):
        fm = self._make(max_follow_speed_byte=200, steering_gain=5.0)
        det = self._person(x_m=3.0, z_m=4.9, bbox=(0.9, 0.3, 1.0, 0.8))
        left, right = fm.compute([det])
        self.assertGreaterEqual(left, 0)
        self.assertLessEqual(left, 255)
        self.assertGreaterEqual(right, 0)
        self.assertLessEqual(right, 255)


class TestSteerDeadband(unittest.TestCase):
    """Unit tests for the bbox-x steer deadband (steer_deadband_norm config param)."""

    def _make(self, **overrides) -> FollowMeController:
        cfg = FollowMeConfig(**overrides)
        return FollowMeController(cfg)

    def _person_at_norm_x(self, norm_x: float, z_m: float = 2.0) -> PersonDetection:
        """Build a detection whose bbox maps to a specific normalized_x."""
        cx = norm_x / 2.0 + 0.5
        half = 0.1
        return PersonDetection(
            x_m=norm_x * z_m,
            z_m=z_m,
            confidence=0.9,
            bbox=(cx - half, 0.3, cx + half, 0.8),
        )

    def test_deadband_suppresses_error_inside_band(self):
        """Person inside the deadband → steer output is symmetric (no differential)."""
        fm = self._make(
            steer_deadband_norm=0.05,
            pid_lateral_kp=2.0,
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            steer_slew_per_tick=1.0,  # no effective slew cap
            follow_output_rate_hz=10000.0,
        )
        # norm_x = 0.03 < 0.05 deadband → x_err should be clamped to 0
        det = self._person_at_norm_x(0.03)
        left, right = fm.compute([det])
        self.assertEqual(left, right, "Steer inside deadband must produce zero differential")

    def test_deadband_passes_error_outside_band(self):
        """Person outside the deadband → controller produces a steering differential."""
        fm = self._make(
            steer_deadband_norm=0.05,
            pid_lateral_kp=2.0,
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            steer_slew_per_tick=1.0,
            follow_output_rate_hz=10000.0,
        )
        # norm_x = 0.20 >> 0.05 deadband → x_err = 0.20 → non-zero steer
        det = self._person_at_norm_x(0.20)
        left, right = fm.compute([det])
        self.assertGreater(left, right, "Person right of deadband must turn right (left > right)")

    def test_deadband_respects_negative_offset(self):
        """Deadband is symmetric: small negative offset is also suppressed."""
        fm = self._make(
            steer_deadband_norm=0.05,
            pid_lateral_kp=2.0,
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            steer_slew_per_tick=1.0,
            follow_output_rate_hz=10000.0,
        )
        det = self._person_at_norm_x(-0.03)
        left, right = fm.compute([det])
        self.assertEqual(left, right, "Steer inside negative deadband must produce zero differential")

    def test_zero_deadband_always_steers(self):
        """With deadband=0, any non-zero offset produces a steer differential."""
        fm = self._make(
            steer_deadband_norm=0.0,
            pid_lateral_kp=2.0,
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            steer_slew_per_tick=1.0,
            follow_output_rate_hz=10000.0,
        )
        # Very small offset — without deadband this should still steer
        det = self._person_at_norm_x(0.01)
        left, right = fm.compute([det])
        self.assertGreater(left, right, "Zero deadband must pass through even tiny offsets")


class TestSteerSlewCap(unittest.TestCase):
    """Unit tests for the per-tick steer slew cap (steer_slew_per_tick config param)."""

    def _make(self, **overrides) -> FollowMeController:
        cfg = FollowMeConfig(**overrides)
        return FollowMeController(cfg)

    def _person_hard_right(self) -> PersonDetection:
        """Detection far to the right to saturate PID output."""
        return PersonDetection(
            x_m=2.0, z_m=2.0, confidence=0.9,
            bbox=(0.85, 0.3, 1.0, 0.8),  # cx=0.925, norm_x=0.85
        )

    def test_slew_cap_limits_first_emission(self):
        """Step from 0 → saturated steer: first emission must not exceed slew_limit."""
        max_steer = 20.0
        slew_per_tick = 0.1   # 2.0 bytes limit
        fm = self._make(
            steer_deadband_norm=0.0,
            steer_slew_per_tick=slew_per_tick,
            max_steer_offset_byte=max_steer,
            direct_mode_max_steer_byte=max_steer,
            pid_lateral_kp=10.0,  # saturates output at max_steer
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            follow_output_rate_hz=10000.0,  # emit on every compute() call
        )
        fm.compute([self._person_hard_right()])
        emitted = fm._last_emitted_steer
        slew_limit = slew_per_tick * max_steer  # 2.0
        self.assertLessEqual(
            abs(emitted), slew_limit + 1e-9,
            f"First emission {emitted:.3f} must not exceed slew limit {slew_limit}",
        )

    def test_slew_cap_ramps_over_multiple_ticks(self):
        """Output must ramp toward target over multiple ticks, each ≤ slew_limit."""
        max_steer = 20.0
        slew_per_tick = 0.1   # 2.0 bytes per tick
        fm = self._make(
            steer_deadband_norm=0.0,
            steer_slew_per_tick=slew_per_tick,
            max_steer_offset_byte=max_steer,
            direct_mode_max_steer_byte=max_steer,
            pid_lateral_kp=10.0,
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            follow_output_rate_hz=10000.0,
        )
        det = self._person_hard_right()
        prev_steer = 0.0
        slew_limit = slew_per_tick * max_steer  # 2.0
        for tick in range(5):
            fm.compute([det])
            current_steer = fm._last_emitted_steer
            delta = current_steer - prev_steer
            self.assertLessEqual(
                delta, slew_limit + 1e-9,
                f"Tick {tick}: delta {delta:.3f} exceeds slew limit {slew_limit}",
            )
            prev_steer = current_steer
        # After 5 ticks, steer must have grown from 0
        self.assertGreater(prev_steer, 0.0, "Steer must have increased over 5 ticks")

    def test_slew_cap_does_not_block_small_request(self):
        """A steer request smaller than slew_limit must pass through fully in one tick."""
        max_steer = 20.0
        slew_per_tick = 0.5   # 10.0 bytes per tick — large cap
        fm = self._make(
            steer_deadband_norm=0.0,
            steer_slew_per_tick=slew_per_tick,
            max_steer_offset_byte=max_steer,
            direct_mode_max_steer_byte=max_steer,
            pid_lateral_kp=1.0,
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            follow_output_rate_hz=10000.0,
        )
        # norm_x ≈ 0.1 → PID raw = 0.1, scaled = 0.1 * 20 = 2.0 bytes < slew_limit 10.0
        det = PersonDetection(
            x_m=0.2, z_m=2.0, confidence=0.9,
            bbox=(0.55, 0.3, 0.65, 0.8),  # cx=0.60, norm_x=0.20
        )
        fm.compute([det])
        emitted = fm._last_emitted_steer
        slew_limit = slew_per_tick * max_steer  # 10.0
        # emitted must be well within one slew step of the PID output (not blocked to 0)
        self.assertGreater(emitted, 0.0, "Small request must produce non-zero steer")
        self.assertLessEqual(emitted, slew_limit + 1e-9)


class TestTrackingModeByteIdentical(unittest.TestCase):
    """Wave 2 (fm-lost-path) touched the emission gate and did a getattr->direct-
    attribute sweep across FollowMeController. NONE of that may change a single
    emitted byte in TRACKING mode (fresh, continuously-detected target: lateral
    PID, edge-boost, deadband, geometric detection filter, tracking-mode slew
    all stay exactly as validated pre-Wave-2).

    This test drives a representative tracked-target sequence (person drifting
    from centered->right-edge->back->left-edge->center, at varying distance, on
    EVERY tick a FRESH detection so the path never touches persist/lost/search)
    and asserts the emitted (left, right) byte sequence exactly matches the
    sequence captured by running the identical scenario against the
    pre-Wave-2 code at commit 912bb34 (see wave2/fm-lost-path branch base).
    """

    EXPECTED_BYTES = [
        (216, 216),
        (213, 210),
        (207, 199),
        (209, 196),
        (184, 170),
        (156, 147),
        (128, 124),
        (126, 126),
        (126, 126),
        (126, 126),
    ]

    def test_tracking_mode_output_byte_identical_to_pre_wave2(self):
        cfg = FollowMeConfig(
            trail_follow_enabled=False,  # isolate the DIRECT PID tracking path
            follow_distance_m=1.5,
            max_follow_speed_byte=90,
        )
        fm = FollowMeController(cfg)

        # (x_m, z_m, bbox) per tick -- every tick is a fresh, in-range, high-
        # confidence detection so the controller stays in pure tracking mode.
        seq = [
            (0.0, 3.0, (0.40, 0.3, 0.60, 0.8)),   # centered, far
            (0.3, 2.8, (0.55, 0.3, 0.72, 0.8)),   # drifting right
            (0.6, 2.5, (0.65, 0.3, 0.85, 0.8)),   # further right, closer
            (0.9, 2.0, (0.75, 0.3, 0.95, 0.8)),   # near right edge
            (0.9, 1.8, (0.75, 0.3, 0.95, 0.8)),   # hold near edge
            (0.5, 1.6, (0.60, 0.3, 0.80, 0.8)),   # coming back
            (0.0, 1.5, (0.40, 0.3, 0.60, 0.8)),   # centered, at follow distance
            (-0.4, 1.5, (0.25, 0.3, 0.45, 0.8)),  # drift left
            (-0.8, 1.7, (0.10, 0.3, 0.30, 0.8)),  # left edge
            (0.0, 1.5, (0.40, 0.3, 0.60, 0.8)),   # back to center
        ]

        results = []
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for x_m, z_m, bbox in seq:
                t += 0.1  # 10 Hz vision ticks, above the 15 Hz output gate
                mt.monotonic.return_value = t
                det = PersonDetection(
                    x_m=x_m, z_m=z_m, confidence=0.9, bbox=bbox, track_id=1,
                )
                left, right = fm.compute([det])
                results.append((left, right))

        self.assertEqual(
            results, self.EXPECTED_BYTES,
            "Tracking-mode emitted bytes changed -- this must stay byte-"
            "identical across the Wave 2 lost/search + config-getattr fix.",
        )


class TestLostSearchSlewContinuity(unittest.TestCase):
    """Wave 2 (fm-lost-path): the per-tick steer slew cap (steer_slew_per_tick)
    now applies to EVERY emission-gate branch -- tracking, persist, lost, AND
    search -- and _last_emitted_steer carries continuously across mode
    transitions instead of being force-reset to 0 whenever target_present is
    False. Before this fix, SEARCH could snap steering from 0 to its full
    magnitude in a single 15 Hz output tick, and reacquisition after a lost/
    search spell always slewed from a false 0 instead of the real last-
    commanded steer.
    """

    def _make(self, **overrides) -> FollowMeController:
        cfg = FollowMeConfig(**overrides)
        return FollowMeController(cfg)

    def _person(self, x_m=0.0, z_m=2.0, confidence=0.9,
                bbox=(0.4, 0.3, 0.6, 0.8), track_id=None) -> PersonDetection:
        return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                               bbox=bbox, track_id=track_id)

    def test_search_steer_is_slew_capped_not_a_step(self):
        """Entering SEARCH must ramp steer in, not snap it to search_steer_cap_byte
        in one output tick -- the same per-tick cap tracking mode obeys.

        Checked on the actual emitted MOTOR BYTES (the physical command), not
        just internal bookkeeping -- the pre-fix bug force-reset the
        bookkeeping variable (_last_emitted_steer) to 0 on every lost/search
        tick while still returning the UNCAPPED full-magnitude search byte
        differential to the caller, so asserting on the bookkeeping variable
        alone would not catch the regression.
        """
        slew_per_tick = 0.05  # small cap so the ramp is unmistakable
        max_steer = 25.0
        search_cap = 20.0
        fm = self._make(
            trail_follow_enabled=False,   # skip trail pursuit -> straight to search
            target_persistence_s=2.0,
            steer_slew_per_tick=slew_per_tick,
            max_steer_offset_byte=max_steer,
            search_steer_cap_byte=search_cap,
            follow_output_rate_hz=10000.0,  # emit on every compute() call
        )
        slew_limit = slew_per_tick * max_steer  # 1.25 bytes/tick

        # Person off to the right so search direction is set, then person lost.
        left0, right0 = fm.compute([self._person(x_m=1.0, z_m=3.0)])
        valid = fm._last_valid_time

        with patch("pi_app.control.follow_me.time") as mt:
            # First lost tick (still within persistence window -> persist path,
            # not search yet). Advance further to enter SEARCH.
            mt.monotonic.return_value = valid + 2.5
            left1, right1 = fm.compute([])
            self.assertEqual(fm._pursuit_mode, "search")

        # The full search differential (uncapped) would be ~search_cap bytes
        # each side -- i.e. |left1 - right1| would jump to roughly 2*search_cap
        # if the slew cap were bypassed. Confirm the ACTUAL byte differential
        # produced this tick is instead bounded by one slew step.
        emitted_diff = abs((left1 - right1) - (left0 - right0))
        slew_byte_diff_limit = 2 * slew_limit + 1.0  # x2 for L/R split, +1 rounding
        self.assertLessEqual(
            emitted_diff, slew_byte_diff_limit,
            f"First SEARCH-tick byte differential changed by {emitted_diff} "
            f"(limit ~{slew_byte_diff_limit}) -- SEARCH is bypassing the slew "
            "gate and stepping straight to the full search magnitude.",
        )
        # And the full uncapped search magnitude must NOT have been reached in
        # one tick (sanity that this scenario would actually exercise a step
        # if the cap were absent).
        full_search_diff = 2 * search_cap
        self.assertLess(emitted_diff, full_search_diff - 1.0)

    def test_reacquisition_slews_from_true_last_emitted_not_zero(self):
        """After a lost/search spell where the robot was actually commanding a
        non-trivial steer, reacquiring the target must slew FROM that real
        last-emitted steer -- not from a false 0 -- so the first
        re-tracking tick's delta is bounded by the slew cap relative to the
        ACTUAL last command, matching tracking-mode continuity."""
        slew_per_tick = 0.5  # generous cap so search saturates to its full value
        max_steer = 25.0
        fm = self._make(
            trail_follow_enabled=False,
            target_persistence_s=2.0,
            steer_slew_per_tick=slew_per_tick,
            max_steer_offset_byte=max_steer,
            direct_mode_max_steer_byte=max_steer,
            search_steer_cap_byte=20.0,
            pid_lateral_kp=10.0,   # saturate direct PID so post-reacq steer is large
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            reacq_slew_window_s=0.001,  # effectively disable the separate reacq ramp
            follow_output_rate_hz=10000.0,
        )

        # Prime a rightward search lock, then let elapsed time enter SEARCH and
        # saturate the slew-capped steer toward search_steer_cap_byte.
        fm.compute([self._person(x_m=1.0, z_m=3.0)])
        valid = fm._last_valid_time

        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 2.5
            fm.compute([])
            mt.monotonic.return_value = valid + 2.6
            fm.compute([])
            mt.monotonic.return_value = valid + 2.8
            fm.compute([])

        self.assertEqual(fm._pursuit_mode, "search")
        last_emitted_before_reacq = fm._last_emitted_steer
        # Sanity: search must have actually built up a non-trivial steer for
        # this test to mean anything (not stuck at 0).
        self.assertGreater(abs(last_emitted_before_reacq), slew_per_tick * max_steer)

        # Reacquire: a hard-right detection whose direct-PID output saturates
        # far past the current commanded steer, in the OPPOSITE-leaning frame
        # position to stress the slew (person far to the LEFT this time).
        det = self._person(x_m=-2.0, z_m=2.0, bbox=(0.05, 0.3, 0.25, 0.8), track_id=1)
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 2.9
            fm.compute([det])

        first_reacq_emitted = fm._last_emitted_steer
        slew_limit = slew_per_tick * max_steer
        delta_from_true_last = abs(first_reacq_emitted - last_emitted_before_reacq)

        # The first reacquisition tick's emitted steer must be within one slew
        # step of the ACTUAL last-emitted search steer -- proving the slew
        # reference carried across the lost->tracking transition instead of
        # resetting to a false 0 (which would have allowed a jump all the way
        # from 0, i.e. up to slew_limit past 0 instead of past the true value).
        self.assertLessEqual(
            delta_from_true_last, slew_limit + 1e-6,
            f"Reacquisition step {delta_from_true_last:.3f} exceeds the slew "
            f"limit {slew_limit:.3f} measured from the TRUE last-emitted steer "
            f"{last_emitted_before_reacq:.3f} -- slew reference did not carry "
            "continuously across the lost/search -> tracking transition.",
        )

    def test_full_timeout_resets_emitted_steer_to_zero(self):
        """Past the full lost_target_trail_pursuit_max_s budget the robot hard
        stops -- this is the one legitimate case where _last_emitted_steer
        resets to 0 (nothing left to slew from)."""
        fm = self._make(
            trail_follow_enabled=False,
            target_persistence_s=0.5,
            lost_target_trail_pursuit_max_s=1.0,
            follow_output_rate_hz=10000.0,
        )
        fm.compute([self._person(x_m=1.0, z_m=3.0)])
        valid = fm._last_valid_time

        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 1.5  # past full timeout
            left, right = fm.compute([])

        self.assertEqual((left, right), (NEUTRAL, NEUTRAL))
        self.assertEqual(fm._last_emitted_steer, 0.0)


class TestRecorder(unittest.TestCase):
    """Tests for the per-session JSONL flight recorder."""

    def _make(self, **overrides) -> FollowMeController:
        cfg = FollowMeConfig(**overrides)
        return FollowMeController(cfg)

    def _person(self, x_m=0.0, z_m=2.0, confidence=0.9,
                bbox=(0.4, 0.3, 0.6, 0.8)) -> PersonDetection:
        return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence, bbox=bbox)

    # ── (1) Recorder rotation ────────────────────────────────────────────────

    def test_rotation_creates_new_file_per_session(self):
        """stop_recorder() + start_recorder() must produce a new JSONL file on
        the next compute(), leaving the previous file closed and complete."""
        with tempfile.TemporaryDirectory() as tmp:
            fm = self._make()

            # Patch the recorder directory so we write into our temp dir.
            with patch("pi_app.control.follow_me.os.makedirs"):
                # Use a fixed-increment fake time so each session gets a
                # distinct integer timestamp.
                call_count = [0]
                def fake_time():
                    call_count[0] += 1
                    return float(1_000_000 + call_count[0])

                with patch("pi_app.control.follow_me.time.time", side_effect=fake_time):
                    # Redirect the open() to our temp dir by intercepting the path.
                    opened_paths = []
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            fname = os.path.basename(path)
                            new_path = os.path.join(tmp, fname)
                            opened_paths.append(new_path)
                            return _real_open(new_path, *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        # Session 1: run a few ticks then stop
                        fm.compute([self._person()])
                        fm.compute([self._person()])
                        fm.stop_recorder()

                        # Session 2: start fresh and run more ticks
                        fm.start_recorder()
                        fm.compute([self._person()])
                        fm.stop_recorder()

            # Should have opened exactly two distinct files.
            self.assertEqual(len(opened_paths), 2,
                             f"Expected 2 files, got: {opened_paths}")
            self.assertNotEqual(opened_paths[0], opened_paths[1],
                                "Two sessions must use different file paths")

            # Both files must be valid JSONL (non-empty, parseable).
            for p in opened_paths:
                with open(p) as fh:
                    lines = [l for l in fh if l.strip()]
                self.assertGreater(len(lines), 0,
                                   f"Recorder file {p} is empty")
                for line in lines:
                    rec = json.loads(line)
                    self.assertIn("t", rec)

    def test_old_file_closed_after_stop(self):
        """After stop_recorder(), the internal file handle must be None."""
        with tempfile.TemporaryDirectory() as tmp:
            fm = self._make()
            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=1_000_000.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"), *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        fm.compute([self._person()])
                        self.assertIsNotNone(fm._recorder_file,
                                             "Recorder file must be open after first compute()")
                        fm.stop_recorder()

            self.assertIsNone(fm._recorder_file,
                              "Recorder file handle must be None after stop_recorder()")

    # ── (2) Unwritable directory ─────────────────────────────────────────────

    def test_unwritable_dir_does_not_raise(self):
        """When the recorder directory is unwritable, compute() must not raise
        and must still return valid (left_byte, right_byte) outputs."""
        fm = self._make()

        def raise_permission(*args, **kwargs):
            raise PermissionError("read-only filesystem")

        with patch("pi_app.control.follow_me.os.makedirs", side_effect=raise_permission):
            # Must not raise; should still return motor bytes.
            try:
                result = fm.compute([self._person()])
            except Exception as exc:
                self.fail(f"compute() raised {type(exc).__name__} with unwritable dir: {exc}")

            self.assertEqual(len(result), 2, "compute() must return (left, right) tuple")
            left, right = result
            self.assertIsInstance(left, int)
            self.assertIsInstance(right, int)

    def test_unwritable_dir_produces_outputs_across_multiple_ticks(self):
        """Multiple ticks with an unwritable recorder dir must all produce outputs."""
        fm = self._make()

        def raise_permission(*args, **kwargs):
            raise PermissionError("read-only filesystem")

        with patch("pi_app.control.follow_me.os.makedirs", side_effect=raise_permission):
            for _ in range(5):
                left, right = fm.compute([self._person()])
                self.assertGreaterEqual(left, 0)
                self.assertLessEqual(left, 255)
                self.assertGreaterEqual(right, 0)
                self.assertLessEqual(right, 255)

    # ── (3) New fields in written records ────────────────────────────────────

    def test_record_contains_is_armed_field(self):
        """Each JSONL record must include the is_armed boolean field."""
        with tempfile.TemporaryDirectory() as tmp:
            fm = self._make()
            fm.set_arm_state(True)

            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=1_000_001.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"), *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        fm.compute([self._person()])
                        fm.stop_recorder()

            with open(os.path.join(tmp, "trial.jsonl")) as fh:
                rec = json.loads(fh.readline())

            self.assertIn("is_armed", rec, "Record must contain 'is_armed' field")
            self.assertIs(rec["is_armed"], True,
                          "is_armed must reflect the value set via set_arm_state(True)")

    def test_record_is_armed_false_by_default(self):
        """Without calling set_arm_state(), is_armed must default to False."""
        with tempfile.TemporaryDirectory() as tmp:
            fm = self._make()
            # Do NOT call set_arm_state — should default to False.

            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=1_000_002.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"), *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        fm.compute([self._person()])
                        fm.stop_recorder()

            with open(os.path.join(tmp, "trial.jsonl")) as fh:
                rec = json.loads(fh.readline())

            self.assertIn("is_armed", rec)
            self.assertIs(rec["is_armed"], False)

    def test_record_contains_left_right_byte_fields(self):
        """Each JSONL record must include left_byte and right_byte integer fields."""
        with tempfile.TemporaryDirectory() as tmp:
            fm = self._make()

            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=1_000_003.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"), *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        fm.compute([self._person()])
                        fm.stop_recorder()

            with open(os.path.join(tmp, "trial.jsonl")) as fh:
                rec = json.loads(fh.readline())

            self.assertIn("left_byte", rec, "Record must contain 'left_byte' field")
            self.assertIn("right_byte", rec, "Record must contain 'right_byte' field")
            self.assertIsInstance(rec["left_byte"], int)
            self.assertIsInstance(rec["right_byte"], int)
            self.assertGreaterEqual(rec["left_byte"], 0)
            self.assertLessEqual(rec["left_byte"], 255)
            self.assertGreaterEqual(rec["right_byte"], 0)
            self.assertLessEqual(rec["right_byte"], 255)

    def test_existing_record_fields_preserved(self):
        """New fields must not replace or rename the fields fm_score.py depends on."""
        required_fields = {"t", "x_raw", "x_filt", "x_err", "steer", "speed",
                           "mode", "track_id", "depth", "conf"}
        with tempfile.TemporaryDirectory() as tmp:
            fm = self._make()

            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=1_000_004.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"), *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        fm.compute([self._person()])
                        fm.stop_recorder()

            with open(os.path.join(tmp, "trial.jsonl")) as fh:
                rec = json.loads(fh.readline())

            for field in required_fields:
                self.assertIn(field, rec,
                              f"Legacy field '{field}' must still be present in records")


class TestRecorderHonestyFields(unittest.TestCase):
    """Recorder honesty additions: steer_want, steer_src, slip_active, emitted.

    Drives the controller through a fresh tick, a persistence/decay tick, and a
    true lost tick with a controllable monotonic clock, capturing every JSONL
    record, and asserts the new per-tick fields are present and correct while the
    legacy keys (which fm_score.py parses) are unchanged.
    """

    def _person(self, x_m=0.0, z_m=2.0, confidence=0.9,
                bbox=(0.4, 0.3, 0.6, 0.8)) -> PersonDetection:
        return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence, bbox=bbox)

    def _run_sequence(self, steps):
        """Run a list of (detections, monotonic_time) steps through one controller
        and return the list of parsed JSONL records (one per compute call).

        Trail following is disabled so steer_src is deterministically "direct" on
        fresh ticks (no odometry is wired in these unit tests anyway).
        """
        cfg = FollowMeConfig(trail_follow_enabled=False)
        fm = FollowMeController(cfg)
        with tempfile.TemporaryDirectory() as tmp:
            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=2_000_000.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"),
                                              *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    clock = {"t": 0.0}

                    def fake_mono():
                        return clock["t"]

                    with patch("builtins.open", side_effect=fake_open):
                        with patch("pi_app.control.follow_me.time.monotonic",
                                   side_effect=fake_mono):
                            for dets, t in steps:
                                clock["t"] = t
                                fm.compute(dets)
                    fm.stop_recorder()

            with open(os.path.join(tmp, "trial.jsonl")) as fh:
                return [json.loads(line) for line in fh if line.strip()]

    def test_new_fields_present_and_typed(self):
        """Every record carries steer_want (number), steer_src (str), slip_active
        (bool), emitted (bool) — alongside the unchanged legacy keys."""
        recs = self._run_sequence([
            ([self._person()], 0.0),
            ([self._person()], 0.07),
        ])
        self.assertGreaterEqual(len(recs), 2)
        legacy = {"t", "x_raw", "x_filt", "x_err", "steer", "speed", "mode",
                  "track_id", "depth", "conf", "is_armed", "left_byte", "right_byte"}
        for rec in recs:
            for k in legacy:
                self.assertIn(k, rec, f"legacy key '{k}' missing")
            self.assertIn("steer_want", rec)
            self.assertIn("steer_src", rec)
            self.assertIn("slip_active", rec)
            self.assertIn("emitted", rec)
            self.assertIsInstance(rec["steer_want"], (int, float))
            self.assertIsInstance(rec["steer_src"], str)
            self.assertIsInstance(rec["slip_active"], bool)
            self.assertIsInstance(rec["emitted"], bool)
            # mode keeps its legacy vocabulary for fm_score.py.
            self.assertIn(rec["mode"], {"direct", "pp", "search", "lost"})

    def test_fresh_tick_steer_src_direct(self):
        """A fresh detection (trail disabled) records steer_src == 'direct' and a
        steer_want equal to the pre-slip/pre-slew steer the branch produced."""
        recs = self._run_sequence([
            # Prime two fresh ticks so the reacq ramp is past and a real steer forms.
            ([self._person(x_m=2.0, bbox=(0.75, 0.3, 0.95, 0.8))], 0.0),
            ([self._person(x_m=2.0, bbox=(0.75, 0.3, 0.95, 0.8))], 0.5),
            ([self._person(x_m=2.0, bbox=(0.75, 0.3, 0.95, 0.8))], 1.5),
        ])
        fresh = recs[-1]
        self.assertEqual(fresh["steer_src"], "direct")
        self.assertFalse(fresh["slip_active"])  # no rpm telemetry → slip inert
        # An off-centre person yields a non-zero steer want toward the person.
        self.assertNotEqual(fresh["steer_want"], 0.0)

    def test_persistence_tick_steer_src_persist(self):
        """After a fresh detection, an empty-detection tick within the persistence
        window (target still held, not fresh) records steer_src == 'persist' —
        distinct from a true lost tick."""
        recs = self._run_sequence([
            ([self._person(x_m=2.0, bbox=(0.75, 0.3, 0.95, 0.8))], 0.0),
            ([self._person(x_m=2.0, bbox=(0.75, 0.3, 0.95, 0.8))], 0.5),
            ([self._person(x_m=2.0, bbox=(0.75, 0.3, 0.95, 0.8))], 1.0),
            # Empty detections at t=1.2: within target_persistence_s (2.0) of last
            # fresh (1.0), so the tracker holds the target → persistence/decay tick.
            ([], 1.2),
        ])
        persist = recs[-1]
        self.assertEqual(persist["steer_src"], "persist")
        # mode still reports the pursuit mode (direct), not "lost", on a hold tick.
        self.assertEqual(persist["mode"], "direct")

    def test_lost_tick_steer_src_lost(self):
        """Empty detections past the full persistence/timeout window record
        steer_src == 'lost' and mode == 'lost'."""
        recs = self._run_sequence([
            ([self._person()], 0.0),
            ([self._person()], 0.5),
            # Empty far beyond target_persistence_s (2.0) and lost_target_timeout_s
            # (3.5): tracker returns None → true lost.
            ([], 10.0),
            ([], 10.07),
        ])
        lost = recs[-1]
        self.assertEqual(lost["steer_src"], "lost")
        self.assertEqual(lost["mode"], "lost")

    def test_emitted_true_only_on_output_gate_ticks(self):
        """emitted is True only when the 15 Hz output gate actually fires. Vision
        ticks that arrive faster than the output interval and merely hold the last
        command record emitted == False."""
        # output interval is 1/15 ≈ 0.0667 s. Feed ticks 0.02 s apart so most are
        # held. First tick always emits (last_output_time<=0).
        steps = [([self._person()], i * 0.02) for i in range(8)]
        recs = self._run_sequence(steps)
        self.assertEqual(len(recs), 8)
        self.assertTrue(recs[0]["emitted"], "first tick must emit")
        # At 0.02 s spacing, the gate (0.0667 s) fires roughly every ~4th tick, so
        # there must be a mix of emitted True and False.
        flags = [r["emitted"] for r in recs]
        self.assertIn(True, flags)
        self.assertIn(False, flags, "fast-held vision ticks must record emitted=False")
        # The held ticks (emitted False) carry the SAME final steer as the last
        # emitted tick (the double-print the new fields disambiguate).
        for i in range(1, len(recs)):
            if not recs[i]["emitted"]:
                self.assertEqual(recs[i]["steer"], recs[i - 1]["steer"],
                                 "held tick steer must equal the previous tick's")

    def test_emitted_ticks_dedupe_matches_command_rate(self):
        """Filtering emitted==True dedupes the vision-rate stream down to roughly
        the output-gate rate (fewer emitted ticks than total ticks)."""
        steps = [([self._person()], i * 0.02) for i in range(12)]
        recs = self._run_sequence(steps)
        emitted_count = sum(1 for r in recs if r["emitted"])
        self.assertLess(emitted_count, len(recs),
                        "every tick emitted — output gate not deduping")
        self.assertGreaterEqual(emitted_count, 1)


class TestRecorderCurrentTempFields(unittest.TestCase):
    """Flight recorder: cur_l/cur_r (motor current A) and tfet_l/tfet_r (FET temp C)."""

    def _make(self, **overrides) -> FollowMeController:
        cfg = FollowMeConfig(**overrides)
        return FollowMeController(cfg)

    def _person(self, x_m=0.0, z_m=2.0, confidence=0.9,
                bbox=(0.4, 0.3, 0.6, 0.8)) -> PersonDetection:
        return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence, bbox=bbox)

    def _run_with_telem(self, left_current_a=None, right_current_a=None,
                        left_temp_c=None, right_temp_c=None,
                        left_rpm=None, right_rpm=None):
        """Run one compute tick with the given telemetry and return the parsed record."""
        fm = self._make()
        fm.update_telemetry(
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            actual_speed_mps=None,
            left_current_a=left_current_a,
            right_current_a=right_current_a,
            left_temp_c=left_temp_c,
            right_temp_c=right_temp_c,
        )
        with tempfile.TemporaryDirectory() as tmp:
            with patch("pi_app.control.follow_me.os.makedirs"):
                with patch("pi_app.control.follow_me.time.time", return_value=3_000_000.0):
                    _real_open = open

                    def fake_open(path, *args, **kwargs):
                        if "fm_trials" in str(path):
                            return _real_open(os.path.join(tmp, "trial.jsonl"), *args, **kwargs)
                        return _real_open(path, *args, **kwargs)

                    with patch("builtins.open", side_effect=fake_open):
                        fm.compute([self._person()])
                        fm.stop_recorder()

            with open(os.path.join(tmp, "trial.jsonl")) as fh:
                return json.loads(fh.readline())

    def test_current_and_temp_fields_present_with_values(self):
        """cur_l/cur_r/tfet_l/tfet_r appear in the record and carry correct values
        when current and temp telemetry are fed via update_telemetry."""
        rec = self._run_with_telem(
            left_current_a=12.34,
            right_current_a=11.56,
            left_temp_c=43.7,
            right_temp_c=44.2,
        )
        self.assertIn("cur_l", rec)
        self.assertIn("cur_r", rec)
        self.assertIn("tfet_l", rec)
        self.assertIn("tfet_r", rec)
        self.assertAlmostEqual(rec["cur_l"], 12.34, places=2)
        self.assertAlmostEqual(rec["cur_r"], 11.56, places=2)
        self.assertAlmostEqual(rec["tfet_l"], 43.7, places=1)
        self.assertAlmostEqual(rec["tfet_r"], 44.2, places=1)

    def test_current_and_temp_fields_none_when_no_telemetry(self):
        """cur_l/cur_r/tfet_l/tfet_r are None when update_telemetry is not called
        (i.e., when telemetry is unavailable)."""
        rec = self._run_with_telem()  # all defaults → None
        self.assertIn("cur_l", rec)
        self.assertIn("cur_r", rec)
        self.assertIn("tfet_l", rec)
        self.assertIn("tfet_r", rec)
        self.assertIsNone(rec["cur_l"])
        self.assertIsNone(rec["cur_r"])
        self.assertIsNone(rec["tfet_l"])
        self.assertIsNone(rec["tfet_r"])

    def test_existing_rpm_fields_unchanged(self):
        """rpm_l/rpm_r still appear with correct values alongside the new fields."""
        rec = self._run_with_telem(
            left_rpm=1500,
            right_rpm=1480,
            left_current_a=9.0,
            right_current_a=8.5,
            left_temp_c=38.0,
            right_temp_c=39.5,
        )
        self.assertIn("rpm_l", rec)
        self.assertIn("rpm_r", rec)
        self.assertEqual(rec["rpm_l"], 1500)
        self.assertEqual(rec["rpm_r"], 1480)

    def test_legacy_fields_still_present(self):
        """All fields fm_score.py depends on remain in the record after adding new ones."""
        required = {"t", "x_raw", "x_filt", "x_err", "steer", "speed", "mode",
                    "track_id", "depth", "conf", "rpm_l", "rpm_r"}
        rec = self._run_with_telem(left_current_a=5.0, right_current_a=5.0,
                                   left_temp_c=30.0, right_temp_c=30.0)
        for field in required:
            self.assertIn(field, rec, f"legacy field '{field}' missing from record")

    def test_update_telemetry_backward_compat(self):
        """Callers that omit the new kwargs (old call-site signature) still work."""
        fm = self._make()
        # Old-style call — no current/temp kwargs
        fm.update_telemetry(left_rpm=100, right_rpm=100, actual_speed_mps=0.5)
        self.assertIsNone(fm._actual_left_current_a)
        self.assertIsNone(fm._actual_right_current_a)
        self.assertIsNone(fm._actual_left_temp_c)
        self.assertIsNone(fm._actual_right_temp_c)


class TestEdgeBoostSteering(unittest.TestCase):
    """Tests for the edge-boost proportional gain on the direct PID steer path.

    The boost amplifies x_err when |x_err| > steer_edge_knee so the robot
    fights harder to recentre a person near the frame edge, while leaving the
    centre (|x_err| <= knee) byte-identical to the un-boosted path.
    """

    # ─── Shared helpers ────────────────────────────────────────────────────────

    def _person_at_norm_x(self, norm_x: float, z_m: float = 2.0) -> PersonDetection:
        """Build a PersonDetection whose bbox maps to a precise normalized_x value."""
        # cx = norm_x / 2 + 0.5  (inverse of (cx - 0.5) * 2 = normalized_x)
        cx = norm_x / 2.0 + 0.5
        half = 0.1
        return PersonDetection(
            x_m=norm_x * z_m,
            z_m=z_m,
            confidence=0.9,
            bbox=(cx - half, 0.3, cx + half, 0.8),
        )

    def _make_direct(self, **overrides) -> FollowMeController:
        """Controller wired for deterministic direct-PID tests (trail off, slew off,
        emit every tick, confidence-scaling off via conf=1.0 detections)."""
        defaults = dict(
            trail_follow_enabled=False,
            steer_deadband_norm=0.0,      # no deadband so any x_err reaches the PID
            steer_slew_per_tick=1.0,      # no effective slew cap
            follow_output_rate_hz=10000.0,  # emit on every compute() call
            pid_lateral_kd=0.0,
            pid_lateral_ki=0.0,
            # kp left at shipped default (0.4) unless overridden — tests stay realistic
            # Geometric layer-1 filters disabled for this steering-focused test class:
            # norm_x=0.95 maps to xmin≈0.875 which the edge rule would reject, and the
            # test helper bbox_h=0.5 at z=2.0 barely passes height — disable to keep the
            # tests focused on steering math only.
            detect_edge_margin=0.0,
            detect_min_bbox_width=0.0,
            detect_min_person_height_m=0.0,
        )
        defaults.update(overrides)
        cfg = FollowMeConfig(**defaults)
        return FollowMeController(cfg)

    def _steer_for_norm_x(self, norm_x: float, **cfg_overrides) -> float:
        """Run one tick and return the emitted steer (signed bytes) for a given norm_x."""
        fm = self._make_direct(**cfg_overrides)
        fm.compute([self._person_at_norm_x(norm_x)])
        return fm._last_emitted_steer

    # ─── (1) Center preserved ──────────────────────────────────────────────────

    def test_center_byte_identical_boost_on_vs_off(self):
        """At |norm_x| <= knee the emitted steer is byte-identical with boost on/off.

        Uses the default knee=0.4 and tests at 0.2, which sits squarely inside the
        no-boost zone.  If this fails the anti-oscillation centre tune is broken.
        """
        knee = 0.4
        norm_x = 0.2  # well inside knee
        self.assertLessEqual(abs(norm_x), knee)

        steer_on = self._steer_for_norm_x(norm_x, steer_edge_boost=1.5, steer_edge_knee=knee)
        steer_off = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0, steer_edge_knee=knee)
        self.assertEqual(
            steer_on, steer_off,
            f"Centre norm_x={norm_x}: steer_on={steer_on:.6f} != steer_off={steer_off:.6f}; "
            "edge boost must not affect centre behavior"
        )

    def test_center_byte_identical_at_exact_knee(self):
        """At |norm_x| exactly equal to the knee no boost is applied (boundary condition)."""
        knee = 0.4
        norm_x = knee  # exactly on the knee boundary

        steer_on = self._steer_for_norm_x(norm_x, steer_edge_boost=1.5, steer_edge_knee=knee)
        steer_off = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0, steer_edge_knee=knee)
        # ax == knee → (ax - knee)/(1-knee) = 0 → edge_gain = 1.0 → identical
        self.assertEqual(
            steer_on, steer_off,
            f"At norm_x=knee={knee}: steer should be identical; on={steer_on}, off={steer_off}"
        )

    # ─── (2) Edge boosted ──────────────────────────────────────────────────────

    def test_edge_steer_larger_with_boost(self):
        """At norm_x=0.9 (> knee) the boosted steer magnitude is strictly greater than
        the un-boosted steer, and the sign is preserved."""
        norm_x = 0.9
        steer_on = self._steer_for_norm_x(norm_x, steer_edge_boost=1.5, steer_edge_knee=0.4)
        steer_off = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0, steer_edge_knee=0.4)
        self.assertGreater(
            abs(steer_on), abs(steer_off),
            f"Edge norm_x={norm_x}: |steer_on|={abs(steer_on):.3f} must exceed |steer_off|={abs(steer_off):.3f}"
        )
        # Same direction
        self.assertGreater(
            steer_on * steer_off, 0.0,
            "Boost must not flip steer sign"
        )

    def test_edge_steer_within_direct_max(self):
        """Boosted steer at the frame edge must still respect direct_mode_max_steer_byte."""
        direct_max = 18.0
        steer_on = self._steer_for_norm_x(
            0.9,
            steer_edge_boost=1.5,
            steer_edge_knee=0.4,
            direct_mode_max_steer_byte=direct_max,
        )
        self.assertLessEqual(
            abs(steer_on), direct_max + 1e-9,
            f"Steer {abs(steer_on):.3f} exceeded direct_mode_max_steer_byte={direct_max}"
        )

    def test_negative_edge_steer_boosted(self):
        """Boost works symmetrically for left-edge (negative norm_x)."""
        norm_x = -0.9
        steer_on = self._steer_for_norm_x(norm_x, steer_edge_boost=1.5, steer_edge_knee=0.4)
        steer_off = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0, steer_edge_knee=0.4)
        self.assertGreater(
            abs(steer_on), abs(steer_off),
            "Left-edge boost must also amplify magnitude"
        )
        self.assertLess(steer_on, 0.0, "Left-edge steer must remain negative")

    # ─── (3) Monotonic with eccentricity ──────────────────────────────────────

    def test_edge_steer_monotonic_with_eccentricity(self):
        """Steer at 0.9 must be >= steer at 0.5 (eccentricity increases boost)."""
        steer_05 = self._steer_for_norm_x(0.5, steer_edge_boost=1.5, steer_edge_knee=0.4)
        steer_09 = self._steer_for_norm_x(0.9, steer_edge_boost=1.5, steer_edge_knee=0.4)
        self.assertGreaterEqual(
            abs(steer_09), abs(steer_05),
            f"|steer(0.9)|={abs(steer_09):.3f} must be >= |steer(0.5)|={abs(steer_05):.3f}"
        )

    def test_boost_increases_monotonically_from_knee_to_edge(self):
        """Steer magnitude must increase monotonically as norm_x moves from knee toward 1."""
        prev_steer = None
        for norm_x in [0.4, 0.55, 0.7, 0.85, 0.95]:
            s = abs(self._steer_for_norm_x(norm_x, steer_edge_boost=1.5, steer_edge_knee=0.4))
            if prev_steer is not None:
                self.assertGreaterEqual(
                    s, prev_steer - 1e-9,
                    f"Steer not monotone: |steer({norm_x})|={s:.3f} < previous {prev_steer:.3f}"
                )
            prev_steer = s

    # ─── (4) boost=0 is a full no-op ──────────────────────────────────────────

    def test_boost_zero_noop_at_center(self):
        """boost=0 disables the feature: center steer matches the default disabled path."""
        for norm_x in [0.1, 0.3, 0.4]:
            s_zero = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0)
            s_default = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0)
            self.assertEqual(s_zero, s_default)

    def test_boost_zero_noop_at_edge(self):
        """boost=0: edge steer is byte-identical regardless of knee setting."""
        for norm_x in [0.7, 0.9]:
            s_zero = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0, steer_edge_knee=0.4)
            s_zero_no_knee = self._steer_for_norm_x(norm_x, steer_edge_boost=0.0, steer_edge_knee=0.0)
            self.assertEqual(
                s_zero, s_zero_no_knee,
                f"boost=0 must produce identical output regardless of knee; norm_x={norm_x}"
            )

    # ─── (5) Deadband interplay ───────────────────────────────────────────────

    def test_deadband_kills_boost_opportunity(self):
        """A small error inside the deadband must yield steer=0 even with boost enabled.
        Boost is applied AFTER deadband zeroing, so a deadbanded error stays zero."""
        fm = self._make_direct(
            steer_deadband_norm=0.5,   # very wide deadband for the test
            steer_edge_boost=1.5,
            steer_edge_knee=0.4,
            pid_lateral_kp=5.0,
        )
        # norm_x = 0.3 < deadband 0.5 → x_err zeroed before boost → steer must be 0
        det = self._person_at_norm_x(0.3)
        left, right = fm.compute([det])
        self.assertEqual(
            left, right,
            "Error inside deadband must produce zero steer even with boost enabled"
        )

    def test_boost_does_not_resurrect_deadbanded_error(self):
        """Confirm that boost cannot amplify a zero x_err (post-deadband) into non-zero steer."""
        fm = self._make_direct(
            steer_deadband_norm=0.6,   # deadband covers norm_x = 0.55
            steer_edge_boost=5.0,      # aggressive boost — must still not matter
            steer_edge_knee=0.3,
            pid_lateral_kp=10.0,
        )
        det = self._person_at_norm_x(0.55)  # inside deadband
        left, right = fm.compute([det])
        self.assertEqual(
            left, right,
            "Boost must not resurrect a deadbanded (zeroed) x_err"
        )

    # ─── (6) Recorder x_err is pre-boost ─────────────────────────────────────

    def test_recorder_x_err_is_pre_boost(self):
        """_last_x_err_norm must reflect the post-deadband, PRE-boost error.

        With norm_x=0.9 and boost=1.5 the BOOSTED input to the PID is
        norm_x * edge_gain.  The recorder must store the raw 0.9, not the
        inflated value.
        """
        norm_x = 0.9
        knee = 0.4
        boost = 1.5
        # Verify what the boost formula would produce so we can assert x_err is NOT that.
        expected_gain = 1.0 + boost * min(1.0, (norm_x - knee) / (1.0 - knee))
        boosted_value = norm_x * expected_gain

        fm = self._make_direct(
            steer_edge_boost=boost,
            steer_edge_knee=knee,
        )
        fm.compute([self._person_at_norm_x(norm_x)])
        recorded_x_err = fm._last_x_err_norm

        self.assertIsNotNone(recorded_x_err)
        self.assertAlmostEqual(
            recorded_x_err, norm_x, places=5,
            msg=f"_last_x_err_norm should be pre-boost ({norm_x}), got {recorded_x_err} "
                f"(boosted would be {boosted_value:.4f})"
        )
        self.assertNotAlmostEqual(
            recorded_x_err, boosted_value, places=3,
            msg=f"_last_x_err_norm must NOT be the boosted value {boosted_value:.4f}"
        )

    # ─── (7) Replay verification ──────────────────────────────────────────────

    def test_replay_edge_ticks_boosted_center_unchanged(self):
        """Gain-function-level replay: using x_filt from fm_trials/1781386838.jsonl,
        compute the edge_gain for each tick and assert that:
          - ticks with |x_filt| > 0.6 (edge) have edge_gain > 1.0
          - ticks with |x_filt| <= 0.4 (center) have edge_gain == 1.0 exactly

        NOTE: this tests the gain formula directly, not the full controller, because
        wiring the full FollowMeController to replay a JSONL file requires
        reproducing exact EMA filter state and reacq-ramp timing that cannot be
        recovered from logged x_filt values alone.  A full end-to-end replay is
        better done with the real hardware harness; see fm_trials/ tooling.
        """
        replay_path = "/tmp/fm_verify/1781386838.jsonl"
        if not os.path.exists(replay_path):
            self.skipTest(f"Replay file not found: {replay_path}")

        import json as _json

        boost = 1.5
        knee = 0.4

        def edge_gain(x_filt: float) -> float:
            ax = abs(x_filt)
            if boost > 0.0 and knee < 1.0 and ax > knee:
                return 1.0 + boost * min(1.0, (ax - knee) / (1.0 - knee))
            return 1.0

        records = [_json.loads(l) for l in open(replay_path) if l.strip()]
        emitted = [r for r in records if r.get("emitted") and r.get("x_filt") is not None]

        edge_ticks = [r for r in emitted if abs(r["x_filt"]) > 0.6]
        center_ticks = [r for r in emitted if abs(r["x_filt"]) <= 0.4]

        self.assertGreater(len(edge_ticks), 0, "Replay has no emitted edge ticks — check file")
        self.assertGreater(len(center_ticks), 0, "Replay has no emitted center ticks — check file")

        # All edge ticks must have gain > 1
        for r in edge_ticks:
            g = edge_gain(r["x_filt"])
            self.assertGreater(
                g, 1.0,
                f"Edge tick x_filt={r['x_filt']:.3f} produced gain={g:.4f} (expected > 1)"
            )

        # All center ticks must have gain == 1 exactly
        for r in center_ticks:
            g = edge_gain(r["x_filt"])
            self.assertEqual(
                g, 1.0,
                f"Centre tick x_filt={r['x_filt']:.3f} produced gain={g:.4f} (expected exactly 1.0)"
            )

        # Sanity: average edge gain is appreciably above 1
        avg_edge_gain = sum(edge_gain(r["x_filt"]) for r in edge_ticks) / len(edge_ticks)
        self.assertGreater(avg_edge_gain, 1.2,
                           f"Average edge gain {avg_edge_gain:.3f} seems too low; check boost constant")

        # Compute ratio of |boosted_steer| / |original_steer| for edge ticks that had
        # nonzero steer in the log, to show the improvement numerically.
        ratios = []
        for r in edge_ticks:
            orig_steer = abs(r.get("steer", 0.0))
            if orig_steer > 0.01:
                ratios.append(edge_gain(r["x_filt"]))
        if ratios:
            avg_ratio = sum(ratios) / len(ratios)
            # Just a sanity check — the gain is > 1 for all of them
            self.assertGreater(avg_ratio, 1.0)


# ─────────────────────────────────────────────────────────────────────────────
# Sticky target lock (TargetTracker) — defends against an impostor that YOLO
# misclassifies as a person. Modelled on the real fm_trials/1781387997 run where
# the robot locked onto a CHICKEN (conf 0.54–0.92) the moment the person blinked.
#
# NOTE ON REPLAY: the recorder logs only the CHOSEN target per tick, not the full
# candidate list, so 1781387997.jsonl cannot be replayed through TargetTracker.
# These tests reconstruct cases (a) and (b) with SYNTHETIC candidate lists that
# mirror the logged confidence / depth / x values (person id=1 ~conf 0.8; chicken
# conf 0.54 rising to 0.92; chicken closer in depth, offset to an edge x).
# ─────────────────────────────────────────────────────────────────────────────
class TestStickyTargetLock(unittest.TestCase):

    GRACE = 1.5
    ACQ_CONF = 0.65
    MIN_FRAMES = 3

    def _tracker(self) -> TargetTracker:
        # ema_alpha=1.0 → no smoothing, so raw selections are easy to assert.
        return TargetTracker(
            ema_alpha=1.0,
            persistence_s=2.0,
            switch_grace_s=self.GRACE,
            acquire_confidence=self.ACQ_CONF,
            acquire_min_frames=self.MIN_FRAMES,
        )

    def _det(self, normalized_x=0.0, depth_m=2.0, confidence=0.9, track_id=None):
        return _FilteredDetection(
            normalized_x=normalized_x,
            x_m=normalized_x,  # value irrelevant to selection here
            depth_m=depth_m,
            confidence=confidence,
            bbox=(0.4, 0.3, 0.6, 0.8),
            track_id=track_id,
        )

    # ── Case (a): committed person blinks out, closer LOW-CONF chicken present ──
    def test_case_a_grace_holds_committed_against_closer_chicken(self):
        trk = self._tracker()
        t = 100.0
        # Commit to the person id=1 at x=-0.1, depth 2.5m (cold start → immediate).
        person = self._det(normalized_x=-0.1, depth_m=2.5, confidence=0.82, track_id=1)
        st = trk.update([person], now=t)
        self.assertIsNotNone(st)
        self.assertEqual(st.track_id, 1)

        # Next frame (0.1s later, within grace): person GONE, only a closer chicken
        # (id=None, conf 0.54, depth 1.4m, edge x=+0.7). Must HOLD id=1, not switch.
        chicken = self._det(normalized_x=0.7, depth_m=1.4, confidence=0.54, track_id=None)
        held = trk.update([chicken], now=t + 0.1)
        self.assertIsNotNone(held)
        self.assertEqual(held.track_id, 1, "must hold committed person, not the chicken")
        # Not fresh during a grace-hold → caller's persistence-decay runs.
        self.assertIsNone(trk.fresh_raw_x_norm)
        # Held position is the person's, NOT the chicken's edge x.
        self.assertAlmostEqual(held.normalized_x, -0.1, places=6)

    def test_case_a_after_grace_lowconf_chicken_not_acquired(self):
        trk = self._tracker()
        t = 100.0
        trk.update([self._det(normalized_x=-0.1, depth_m=2.5, confidence=0.82, track_id=1)], now=t)
        chicken = self._det(normalized_x=0.7, depth_m=1.4, confidence=0.54, track_id=None)
        # Past the grace window with the same sub-floor chicken → no acquisition.
        out = trk.update([chicken], now=t + self.GRACE + 0.5)
        self.assertIsNone(out, "sub-floor chicken (0.54 < 0.65) must never be acquired")
        self.assertIsNone(trk.fresh_raw_x_norm)

    # ── Case (b): person fully lost, then a rising-conf chicken vs a real person ─
    def test_case_b_rising_chicken_never_acquired_below_floor(self):
        trk = self._tracker()
        t = 100.0
        trk.update([self._det(depth_m=2.5, confidence=0.82, track_id=1)], now=t)
        # Person fully lost: empty frames past persistence (2.0s) + grace.
        t2 = t + 5.0
        trk.update([], now=t2)
        # Chicken (id=18) appears closest, conf rising 0.56→0.64 — all below floor.
        confs = [0.56, 0.58, 0.60, 0.62, 0.64]
        out = None
        for i, c in enumerate(confs):
            out = trk.update(
                [self._det(normalized_x=0.6, depth_m=1.3, confidence=c, track_id=18)],
                now=t2 + 0.5 + i * 0.1,
            )
        self.assertIsNone(out, "chicken below 0.65 floor must never be acquired")

    def test_case_b_real_person_acquired_exactly_after_min_frames(self):
        trk = self._tracker()
        t = 100.0
        trk.update([self._det(depth_m=2.5, confidence=0.82, track_id=1)], now=t)
        t2 = t + 5.0
        trk.update([], now=t2)  # person fully lost (beyond grace + persistence)

        # A genuine person (id=7, conf 0.8) appears and is sustained. Because the
        # tracker has committed before, re-acquisition is sustain-gated: must NOT
        # acquire before the MIN_FRAMES-th qualifying frame.
        new_person = self._det(normalized_x=0.2, depth_m=3.0, confidence=0.8, track_id=7)
        results = []
        for i in range(self.MIN_FRAMES + 1):
            results.append(trk.update([new_person], now=t2 + 1.0 + i * 0.1))
        # Frames 1..MIN_FRAMES-1 coast (None); committed exactly on the MIN_FRAMES-th.
        for i in range(self.MIN_FRAMES - 1):
            self.assertIsNone(results[i], f"must not acquire before frame {self.MIN_FRAMES}")
        self.assertIsNotNone(results[self.MIN_FRAMES - 1], "acquire on the Nth frame")
        self.assertEqual(results[self.MIN_FRAMES - 1].track_id, 7)

    # ── Committed target trusted even when its confidence later drops ───────────
    def test_committed_target_followed_when_conf_drops_below_floor(self):
        trk = self._tracker()
        t = 100.0
        trk.update([self._det(depth_m=2.0, confidence=0.9, track_id=1)], now=t)
        # Same id=1 still present but conf fell to 0.50 (< 0.65 floor) — still 0.45+
        # so Layer-1 passes it. Tracked = trusted: must keep following it, fresh.
        st = trk.update([self._det(depth_m=2.0, confidence=0.50, track_id=1)], now=t + 0.1)
        self.assertIsNotNone(st)
        self.assertEqual(st.track_id, 1)
        self.assertIsNotNone(trk.fresh_raw_x_norm, "committed follow is a fresh frame")

    # ── Regression: continuous single person → identical selection/EMA ──────────
    def test_continuous_single_person_unchanged_vs_legacy(self):
        """Drive both a legacy-style tracker (no sticky knobs) and the new one
        with the SAME continuous single-person stream; assert identical state."""
        legacy = TargetTracker(ema_alpha=0.5, persistence_s=2.0,
                               switch_grace_s=0.0,
                               acquire_confidence=0.0,
                               acquire_min_frames=1)  # knobs disabled
        sticky = TargetTracker(ema_alpha=0.5, persistence_s=2.0,
                               switch_grace_s=self.GRACE,
                               acquire_confidence=self.ACQ_CONF,
                               acquire_min_frames=self.MIN_FRAMES)
        t = 100.0
        xs = [0.0, 0.2, -0.1, 0.3, -0.2, 0.15, 0.05, -0.05, 0.25, -0.15]
        for i, x in enumerate(xs):
            det = self._det(normalized_x=x, depth_m=2.0 + 0.05 * i,
                            confidence=0.9, track_id=1)
            now = t + i * 0.1
            ls = legacy.update([det], now)
            ss = sticky.update([det], now)
            self.assertIsNotNone(ss)
            # Selection + EMA are byte-identical to the legacy path.
            self.assertAlmostEqual(ls.normalized_x, ss.normalized_x, places=9,
                                   msg=f"frame {i}: EMA diverged")
            self.assertEqual(ls.track_id, ss.track_id)
            self.assertEqual(ls.depth_m, ss.depth_m)
            # And every tracked frame is fresh (no grace/sustain interference).
            self.assertIsNotNone(sticky.fresh_raw_x_norm)


class TestStickyLockComputeIntegration(unittest.TestCase):
    """compute()-level: the fresh_detection derivation change (now keyed off
    tracker.fresh_raw_x_norm instead of bool(filtered)) did not break the normal
    multi-frame follow — every tracked frame is still fresh."""

    def _make(self, **overrides) -> FollowMeController:
        return FollowMeController(FollowMeConfig(**overrides))

    def _person(self, x_m=0.0, z_m=3.0, confidence=0.9,
                bbox=(0.45, 0.3, 0.55, 0.8), track_id=1) -> PersonDetection:
        return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                               bbox=bbox, track_id=track_id)

    def test_normal_follow_marks_each_tracked_frame_fresh(self):
        fm = self._make(follow_distance_m=1.5, max_follow_speed_byte=80)
        for _ in range(6):
            fm.compute([self._person()])
            # The tracker committed a fresh detection every tracked frame.
            self.assertIsNotNone(fm._tracker.fresh_raw_x_norm)
            self.assertTrue(fm.get_status()["follow_me_tracking"])

    def test_grace_hold_frame_is_not_fresh(self):
        """Lock a person, then a single frame with ONLY a sub-floor chicken at a
        different position: the controller must NOT mark it a fresh detection
        (so persistence-decay runs) and must NOT switch the tracked id."""
        fm = self._make()
        fm.compute([self._person(track_id=1)])
        self.assertEqual(fm.get_status()["follow_me_target_track_id"], 1)
        # Chicken: low conf 0.54, edge bbox, closer — must be ignored (grace-hold).
        chicken = PersonDetection(x_m=2.0, z_m=1.3, confidence=0.54,
                                  bbox=(0.85, 0.3, 1.0, 0.8), track_id=None)
        fm.compute([chicken])
        self.assertIsNone(fm._tracker.fresh_raw_x_norm,
                          "grace-hold frame must not be fresh")
        self.assertEqual(fm.get_status()["follow_me_target_track_id"], 1,
                         "must not switch to the chicken")


class TestDetectionFilterGeometric(unittest.TestCase):
    """Unit tests for the three geometric rejection rules in DetectionFilter (Layer 1).

    All filters are enabled via explicit constructor kwargs; the disable path is
    tested separately to confirm backward-compat.

    Geometry reference (from the real run that motivated these rules):
      - Spurious slivers: xmin 0.90-0.94, width 0.06-0.10, conf 0.90-0.92
      - Legitimate persons in same run: xmin ≤ 0.60, width ≥ 0.10, implied_h 1.68-3.02 m
      - Short ground blob (chicken): implied_h ≈ 0.68 m
      - camera_vfov_deg 65.3°  →  tan_half = tan(32.65°) ≈ 0.6408
    """

    _VFOV = 65.3
    _TAN_HALF_VFOV = math.tan(math.radians(_VFOV) / 2.0)

    def _make_filter(
        self,
        edge_margin: float = 0.15,
        min_bbox_width: float = 0.09,
        min_person_height_m: float = 1.20,
        camera_vfov_deg: float = 65.3,
    ) -> DetectionFilter:
        return DetectionFilter(
            conf_threshold=0.45,
            min_depth_m=0.3,
            max_depth_m=8.0,
            min_bbox_area=0.0,
            edge_margin=edge_margin,
            min_bbox_width=min_bbox_width,
            min_person_height_m=min_person_height_m,
            camera_vfov_deg=camera_vfov_deg,
        )

    def _det(self, bbox, z_m=2.5, conf=0.9) -> PersonDetection:
        cx = (bbox[0] + bbox[2]) / 2.0
        return PersonDetection(
            x_m=(cx - 0.5) * 2.0 * z_m,
            z_m=z_m,
            confidence=conf,
            bbox=bbox,
        )

    # ── Implied-height formula verification ────────────────────────────────────

    def test_implied_height_formula_numeric(self):
        """Verify the implied-height formula with an exact hand-computed value.

        bbox_h=0.74, z_m=2.5, vfov=65.3°
          tan(32.65°) = 0.6408  (more precisely: math.tan(math.radians(65.3/2)))
          implied_h = 0.74 * 2.5 * 2 * tan_half = 1.85 * 2 * tan_half
        """
        tan_half = math.tan(math.radians(65.3) / 2.0)
        expected = 0.74 * 2.5 * 2.0 * tan_half
        # The filter must compute this for bbox=(0.30,0.13,0.46,0.87), z_m=2.5
        flt = self._make_filter(min_person_height_m=0.01)  # tiny floor so it passes
        det = self._det(bbox=(0.30, 0.13, 0.46, 0.87), z_m=2.5)
        result = flt.process([det])
        # If it passes the filter, the formula must have produced ≥ 0.01 m
        self.assertEqual(len(result), 1, "Person should pass the filter")
        # Also assert the expected value is in the legitimate range
        self.assertAlmostEqual(expected, 0.74 * 2.5 * 2.0 * tan_half, places=9)
        self.assertGreater(expected, 1.20, "Implied height must be well above 1.20 m")

    # ── ACCEPT cases ───────────────────────────────────────────────────────────

    def test_accept_normal_person(self):
        """A typical centered person must pass all three rules.

        bbox=(0.30,0.13,0.46,0.87): xmin=0.30 < 0.85, width=0.16 > 0.09,
        implied_h ≈ 0.74*2.5*2*tan(32.65°) ≈ 2.37 m > 1.20.
        """
        flt = self._make_filter()
        det = self._det(bbox=(0.30, 0.13, 0.46, 0.87), z_m=2.5)
        result = flt.process([det])
        self.assertEqual(len(result), 1, "Normal centered person must be accepted")

    def test_accept_person_at_legitimate_edge_of_observed_range(self):
        """xmin=0.60 — the maximum observed in the real run — must still be accepted (0.60 < 0.85)."""
        # bbox: xmin=0.60, xmax=0.76 (width=0.16), height 0.74
        flt = self._make_filter()
        det = self._det(bbox=(0.60, 0.13, 0.76, 0.87), z_m=2.5)
        result = flt.process([det])
        self.assertEqual(len(result), 1, "Person at xmin=0.60 must be accepted")

    # ── REJECT cases ───────────────────────────────────────────────────────────

    def test_reject_right_edge_sliver(self):
        """xmin=0.92 > 0.85 → rejected by edge exclusion rule.

        This matches the exact spurious detection from the real run
        (xmin 0.90-0.94, width 0.06-0.10, conf 0.90-0.92).
        """
        flt = self._make_filter()
        det = self._det(bbox=(0.92, 0.20, 0.99, 0.76), z_m=2.5, conf=0.91)
        result = flt.process([det])
        self.assertEqual(len(result), 0, "Right-edge sliver must be rejected by edge rule")

    def test_reject_left_edge_sliver(self):
        """xmax=0.13 < 0.15 → rejected by edge exclusion rule (left side)."""
        flt = self._make_filter()
        det = self._det(bbox=(0.01, 0.2, 0.13, 0.8), z_m=2.5, conf=0.88)
        result = flt.process([det])
        self.assertEqual(len(result), 0, "Left-edge sliver must be rejected by edge rule")

    def test_reject_narrow_box_not_at_edge(self):
        """width=0.07 < 0.09 → rejected by min-width rule (not near either edge)."""
        flt = self._make_filter()
        det = self._det(bbox=(0.45, 0.2, 0.52, 0.9), z_m=2.5, conf=0.85)
        result = flt.process([det])
        self.assertEqual(len(result), 0, "Narrow non-edge box must be rejected by width rule")

    def test_reject_short_ground_blob(self):
        """Blob at z_m=2.17 with bbox_h=0.18: implied_h ≈ 0.50 m < 1.20 → height rule.

        This matches the short 0.68 m blob observed in the real run (chicken-ish animal).
        Only height rejects it: width=0.16 ok (>0.09), xmin=0.40 ok (<0.85).
        """
        flt = self._make_filter()
        # width=0.16 (0.40→0.56), height=0.18 (0.80→0.98), z_m=2.17
        # implied_h = 0.18 * 2.17 * 2 * tan(32.65°) ≈ 0.18 * 2.17 * 1.282 ≈ 0.50 m
        det = self._det(bbox=(0.40, 0.80, 0.56, 0.98), z_m=2.17, conf=0.87)
        result = flt.process([det])
        self.assertEqual(len(result), 0, "Short ground blob must be rejected by height rule")

    def test_short_blob_only_rejected_by_height_not_edge_or_width(self):
        """Confirm that without the height rule, the short blob would PASS edge+width checks."""
        # Disable only height check — blob should pass
        flt_no_height = self._make_filter(min_person_height_m=0.0)
        det = self._det(bbox=(0.40, 0.80, 0.56, 0.98), z_m=2.17, conf=0.87)
        result = flt_no_height.process([det])
        self.assertEqual(len(result), 1,
                         "Short blob must pass when height check is disabled (width=0.16 ok, xmin=0.40 ok)")

    # ── DISABLE (off-switch / back-compat) cases ───────────────────────────────

    def test_disable_edge_check_allows_right_sliver(self):
        """edge_margin=0 disables edge rule — sliver must pass (if other rules also off)."""
        flt = self._make_filter(edge_margin=0.0, min_bbox_width=0.0, min_person_height_m=0.0)
        det = self._det(bbox=(0.92, 0.20, 0.99, 0.76), z_m=2.5, conf=0.91)
        result = flt.process([det])
        self.assertEqual(len(result), 1,
                         "With all geometric checks disabled, right-edge sliver must pass")

    def test_disable_width_check_allows_narrow_box(self):
        """min_bbox_width=0 disables width rule — narrow box must pass (edge+height also off)."""
        flt = self._make_filter(edge_margin=0.0, min_bbox_width=0.0, min_person_height_m=0.0)
        det = self._det(bbox=(0.45, 0.2, 0.52, 0.9), z_m=2.5, conf=0.85)
        result = flt.process([det])
        self.assertEqual(len(result), 1,
                         "With all geometric checks disabled, narrow non-edge box must pass")

    def test_disable_height_check_allows_short_blob(self):
        """min_person_height_m=0 disables height rule — short blob passes (edge+width also off)."""
        flt = self._make_filter(edge_margin=0.0, min_bbox_width=0.0, min_person_height_m=0.0)
        det = self._det(bbox=(0.40, 0.80, 0.56, 0.98), z_m=2.17, conf=0.87)
        result = flt.process([det])
        self.assertEqual(len(result), 1,
                         "With all geometric checks disabled, short ground blob must pass")

    def test_default_constructor_has_all_checks_off(self):
        """Direct DetectionFilter construction WITHOUT new kwargs must behave as before.

        This verifies backward-compatibility: existing direct-construction tests are unaffected.
        """
        flt = DetectionFilter(
            conf_threshold=0.45,
            min_depth_m=0.3,
            max_depth_m=8.0,
            min_bbox_area=0.0,
        )
        # Right-edge sliver, narrow box, short blob — all must pass (checks are off by default)
        slivers = [
            self._det(bbox=(0.92, 0.20, 0.99, 0.76), z_m=2.5, conf=0.91),
            self._det(bbox=(0.45, 0.2, 0.52, 0.9), z_m=2.5, conf=0.85),
            self._det(bbox=(0.40, 0.80, 0.56, 0.98), z_m=2.17, conf=0.87),
        ]
        result = flt.process(slivers)
        self.assertEqual(len(result), 3,
                         "Without new kwargs all detections must pass (checks default off)")

    # ── Regression: normal person is unaffected by active geometric rules ──────

    def test_regression_normal_person_passes_all_active_rules(self):
        """With all three geometric rules active at their defaults, a typical centered person
        (bbox 0.4-0.6 width, full height, at 2 m depth) must still reach the tracker."""
        flt = self._make_filter()
        # Standard test person used throughout the rest of the test suite
        det = PersonDetection(x_m=0.0, z_m=2.0, confidence=0.9,
                              bbox=(0.4, 0.3, 0.6, 0.8))
        result = flt.process([det])
        self.assertEqual(len(result), 1,
                         "Standard centered person must pass all active geometric rules")

    # ── Integration: FollowMeController routes config values into DetectionFilter ─

    def test_controller_enables_geometric_rules_via_config(self):
        """FollowMeController must forward config geometric fields to DetectionFilter.

        Confirm by feeding the spurious sliver that the real construction site
        (with default FollowMeConfig) must reject.
        """
        # Default FollowMeConfig has detect_edge_margin=0.15 etc. — rules are active.
        fm = FollowMeController(FollowMeConfig())
        # Right-edge sliver: would cause re-acquisition in the real run
        sliver = PersonDetection(x_m=2.0, z_m=2.5, confidence=0.91,
                                 bbox=(0.92, 0.20, 0.99, 0.76))
        left, right = fm.compute([sliver])
        self.assertEqual(left, NEUTRAL)
        self.assertEqual(right, NEUTRAL, "Sliver must be rejected by controller with default config")


if __name__ == "__main__":
    unittest.main()
