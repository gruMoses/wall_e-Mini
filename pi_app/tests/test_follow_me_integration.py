"""Integration tests for Follow Me trail-following and lost-target behavior.

Tests cover:
  - Lost-target trail following (robot continues along breadcrumb trail)
  - MIN_LOST_TARGET_SPEED fix when _last_speed_offset is zero
  - Empty trail stops the robot
  - Hysteresis prevents mode flapping between direct and trail pursuit
"""

import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.follow_me import FollowMeController, PersonDetection, NEUTRAL


def _make_controller(**overrides) -> FollowMeController:
    """Create a FollowMeController with trail following enabled by default."""
    defaults = dict(trail_follow_enabled=True)
    defaults.update(overrides)
    cfg = FollowMeConfig(**defaults)
    return FollowMeController(cfg)


def _person(x_m=0.0, z_m=2.0, confidence=0.9,
            bbox=(0.4, 0.3, 0.6, 0.8), track_id=None) -> PersonDetection:
    """Create a PersonDetection with sensible defaults."""
    return PersonDetection(
        x_m=x_m, z_m=z_m, confidence=confidence,
        bbox=bbox, track_id=track_id,
    )


class TestLostTargetTrailFollow(unittest.TestCase):
    """Tests for lost-target trail-following behavior."""

    def test_lost_target_with_trail_follows_trail(self):
        """When the person is lost but a trail exists, the robot follows the trail
        rather than returning to neutral."""
        fm = _make_controller(
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
            lost_target_timeout_s=5.0,
            trail_min_spacing_m=0.05,
        )

        # Simulate several cycles with a person detected straight ahead at 3 m.
        # Each cycle: update_pose (advance odometry), then compute (feed detection).
        t = 100.0
        for i in range(15):
            t += 0.1
            # Feed odometry with forward motor commands and heading 0 deg
            fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)
            left, right = fm.compute([_person(x_m=0.0, z_m=3.0)])
            # Robot should be driving forward while tracking
            self.assertGreater(left, NEUTRAL,
                               f"cycle {i}: left should be > NEUTRAL while tracking")
            self.assertGreater(right, NEUTRAL,
                               f"cycle {i}: right should be > NEUTRAL while tracking")

        # Now lose the person (empty detections). Use time.monotonic via mock
        # to ensure the lost-target timeout has not elapsed.
        t += 0.1
        fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)

        with patch("pi_app.control.follow_me.time") as mock_time:
            # Pretend monotonic returns a time just after the last valid detection
            mock_time.monotonic.return_value = fm._last_valid_time + 0.2
            left, right = fm.compute([])

        # The robot should NOT be at neutral -- it should be following the trail.
        self.assertFalse(
            left == NEUTRAL and right == NEUTRAL,
            "Robot should follow trail when target is lost but trail exists"
        )

        # Both motors should be on the forward side of NEUTRAL (both >= NEUTRAL).
        # The trail is straight ahead, so both wheels should drive forward.
        self.assertGreaterEqual(left, NEUTRAL,
                                "Left motor should be >= NEUTRAL (forward)")
        self.assertGreaterEqual(right, NEUTRAL,
                                "Right motor should be >= NEUTRAL (forward)")

    def test_lost_target_zero_last_speed_still_moves(self):
        """Even when _last_speed_offset is 0, the MIN_LOST_TARGET_SPEED fix
        ensures the robot moves forward along the trail instead of stalling."""
        fm = _make_controller(
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
            lost_target_timeout_s=5.0,
            trail_min_spacing_m=0.05,
        )

        # Build a trail by feeding detections
        t = 100.0
        for i in range(15):
            t += 0.1
            fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)
            fm.compute([_person(x_m=0.0, z_m=3.0)])

        # Force _last_speed_offset to 0 (simulates person at follow distance)
        fm._last_speed_offset = 0.0

        # Lose the person
        t += 0.1
        fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)

        with patch("pi_app.control.follow_me.time") as mock_time:
            mock_time.monotonic.return_value = fm._last_valid_time + 0.2
            left, right = fm.compute([])

        # The MIN_LOST_TARGET_SPEED fix: fwd = max(0 * 0.5, 5) = 5
        # So both motors should be above NEUTRAL (moving forward).
        self.assertGreater(left, NEUTRAL,
                           "Left motor should be > NEUTRAL despite zero last_speed_offset")
        self.assertGreater(right, NEUTRAL,
                           "Right motor should be > NEUTRAL despite zero last_speed_offset")

    def test_lost_target_empty_trail_stops(self):
        """When the person is lost and no trail has been built, the robot should
        return to neutral (stop)."""
        fm = _make_controller(
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
            lost_target_timeout_s=5.0,
        )

        # Do not feed any detections -- no trail is built.
        # Simulate a lost target immediately (never had a valid target).
        with patch("pi_app.control.follow_me.time") as mock_time:
            mock_time.monotonic.return_value = 100.0
            left, right = fm.compute([])

        # _last_valid_time is 0.0, so elapsed > timeout => full reset => neutral.
        self.assertEqual(left, NEUTRAL,
                         "Left motor should be NEUTRAL with no trail and no prior target")
        self.assertEqual(right, NEUTRAL,
                         "Right motor should be NEUTRAL with no trail and no prior target")


class TestModeSwitch(unittest.TestCase):
    """Tests for pursuit-mode hysteresis to prevent flapping between direct
    and trail modes."""

    def test_hysteresis_prevents_flapping(self):
        """Once in trail mode, oscillating the person distance around the
        direct_pursuit_distance_m threshold should not cause the mode to flip
        every cycle. Hysteresis requires the person to be clearly close to
        switch back to direct mode."""
        direct_dist = 2.0
        fm = _make_controller(
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
            lost_target_timeout_s=5.0,
            trail_min_spacing_m=0.05,
            direct_pursuit_distance_m=direct_dist,
            direct_pursuit_lateral_m=0.3,
            min_trail_points_for_pursuit=2,
        )

        # Phase 1: Build a trail by placing the person far away (well past the
        # direct_pursuit_distance_m threshold) so that trail mode engages.
        t = 100.0
        for i in range(20):
            t += 0.1
            fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)
            # Person at 4 m -- well above the 2.0 * 1.3 = 2.6 m threshold
            # needed to switch from direct to trail mode.
            fm.compute([_person(x_m=0.0, z_m=4.0)])

        # After enough cycles with the person far away, the controller should
        # be in trail mode.
        self.assertEqual(fm._pursuit_mode, "trail",
                         "Controller should be in trail mode after tracking far target")

        # Phase 2: Oscillate the person distance around the threshold.
        # The direct_pursuit_distance_m = 2.0, so 1.9 and 2.1 are just barely
        # on each side. With hysteresis, once in trail mode, the controller
        # should only switch to direct when the person is clearly close
        # (z_m < direct_dist AND |x_m| < direct_lat).
        # At z_m=2.1 (just above threshold), it should stay in trail mode.
        # At z_m=1.9 (just below threshold), hysteresis should still keep it
        # in trail mode because the person needs to be clearly close AND centered.
        mode_history = []
        for i in range(10):
            t += 0.1
            fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160, timestamp=t)
            # Oscillate between z_m=1.9 and z_m=2.1
            z = 2.1 if i % 2 == 0 else 1.9
            fm.compute([_person(x_m=0.0, z_m=z)])
            mode_history.append(fm._pursuit_mode)

        # Count mode transitions: should not flip every cycle.
        transitions = sum(
            1 for a, b in zip(mode_history, mode_history[1:]) if a != b
        )

        # With proper hysteresis, oscillating near the threshold should produce
        # far fewer transitions than the number of cycles. Without hysteresis
        # it would flip every cycle (9 transitions for 10 entries).
        self.assertLess(transitions, len(mode_history) - 1,
                        f"Too many mode transitions ({transitions}) -- "
                        f"hysteresis should prevent flapping. "
                        f"Mode history: {mode_history}")


if __name__ == "__main__":
    unittest.main()
