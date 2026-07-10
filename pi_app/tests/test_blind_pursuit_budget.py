"""Tests for the tightened blind-driving budget (FIX 2).

  (a) trail pursuit hard-stops at lost_target_trail_pursuit_max_s (now 3.0s)
  (b) during the persistence window speed decays to 0 in lock-step with steer
  (c) search is a pure pivot — forward component exactly 0 while rotating
"""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.follow_me import FollowMeController, PersonDetection, NEUTRAL


def _person(x_m=0.0, z_m=2.0, confidence=0.9,
            bbox=(0.4, 0.3, 0.6, 0.8), track_id=None) -> PersonDetection:
    return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                           bbox=bbox, track_id=track_id)


class TestTrailPursuitBudget(unittest.TestCase):
    def test_config_default_is_three_seconds(self):
        self.assertEqual(FollowMeConfig().lost_target_trail_pursuit_max_s, 3.0)

    def test_trail_pursuit_hard_stops_at_3s(self):
        """With a trail built, blind pursuit drives just under 3.0s and is a
        full stop (NEUTRAL/NEUTRAL) once the 3.0s budget is exceeded."""
        fm = FollowMeController(FollowMeConfig(
            trail_follow_enabled=True,
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
            trail_min_spacing_m=0.05,
            target_persistence_s=0.5,   # drop into the lost path quickly
        ))
        # Build the trail on a virtual clock: compute() reads time.monotonic()
        # for persistence/trail timing, so under real wall-clock a slow
        # iteration (> target_persistence_s) drops the tracker into the lost
        # path mid-build and corrupts the trail state (flaky under CPU load).
        t = 100.0
        with patch("pi_app.control.follow_me.time") as mt:
            for _ in range(15):
                t += 0.1
                mt.monotonic.return_value = t
                fm.update_pose(heading_deg=0.0, motor_l=160, motor_r=160,
                               timestamp=t)
                fm.compute([_person(x_m=0.0, z_m=3.0)])
        valid = fm._last_valid_time

        # Inside the 3.0s budget: still pursuing the trail (not a full stop).
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 1.0
            l1, r1 = fm.compute([])
        self.assertFalse(l1 == NEUTRAL and r1 == NEUTRAL,
                         "should still pursue the trail before 3.0s")

        # Past the 3.0s budget: hard stop.
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 3.1
            l2, r2 = fm.compute([])
        self.assertEqual((l2, r2), (NEUTRAL, NEUTRAL),
                         "blind trail pursuit must hard-stop at 3.0s")


class TestPersistenceSpeedDecay(unittest.TestCase):
    def test_speed_ramps_to_zero_during_persistence(self):
        """While the tracker coasts on stale state (target_present, no fresh
        detection), speed decays on the same profile as steer and reaches 0."""
        fm = FollowMeController(FollowMeConfig(
            follow_distance_m=1.0,
            max_follow_speed_byte=60,
            target_persistence_s=2.0,
            steer_hold_decay_s=1.0,
        ))
        # Off-centre + far so both speed and steer start clearly non-zero.
        fm.compute([_person(x_m=0.8, z_m=4.0)])
        valid = fm._last_valid_time
        self.assertGreater(fm._last_speed_offset, 0.0)

        # Early in the persistence window: speed still moving.
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 0.05
            fm.compute([])
        early_speed = fm._last_speed_offset
        self.assertGreater(early_speed, 0.0)

        # Mid window: strictly smaller (decaying).
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 0.5
            fm.compute([])
        self.assertLess(fm._last_speed_offset, early_speed)

        # Past the decay window (>= steer_hold_decay_s, still < persistence_s so
        # target stays present): speed has reached exactly 0.
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 1.2
            fm.compute([])
        self.assertEqual(fm._last_speed_offset, 0.0)
        self.assertTrue(fm._steer_hold_active is False or fm._steer_decay_factor == 0.0)


class TestSearchPurePivot(unittest.TestCase):
    def test_search_forward_byte_is_zero_while_steering(self):
        """In SEARCH state the forward component is exactly 0 (pure pivot)
        while steer is non-zero."""
        fm = FollowMeController(FollowMeConfig(
            trail_follow_enabled=False,   # skip trail pursuit -> straight to search
            target_persistence_s=2.0,
        ))
        # Person off to the right so the search direction (last lateral) is set.
        fm.compute([_person(x_m=1.0, z_m=3.0)])
        valid = fm._last_valid_time

        # elapsed in (persistence_s, trail_max_s) => search state.
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = valid + 2.5
            left, right = fm.compute([])

        self.assertEqual(fm._pursuit_mode, "search")
        self.assertEqual(fm._last_speed_offset, 0.0, "search forward byte must be 0")
        self.assertNotEqual(fm._last_steer_offset, 0.0, "search must still rotate")
        # Forward component = (left+right)/2 - NEUTRAL == 0  (symmetric pivot).
        self.assertEqual(left + right, 2 * NEUTRAL)


if __name__ == "__main__":
    unittest.main()
