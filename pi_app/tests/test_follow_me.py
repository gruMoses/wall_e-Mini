import json
import os
import stat
import tempfile
import unittest
from unittest.mock import patch

from config import FollowMeConfig
from pi_app.control.follow_me import FollowMeController, PersonDetection, NEUTRAL


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
        close = self._person(x_m=0.1, z_m=1.5, bbox=(0.45, 0.3, 0.55, 0.8))
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


if __name__ == "__main__":
    unittest.main()
