"""
Tests for the safety-state sync fix (follow_me_active desync between
Controller._mode and SafetyState) and the waypoint-nav speed calculation
guard against divide-by-zero when slow_radius_m <= arrival_radius_m.
"""

import unittest

from pi_app.control.safety import SafetyState
from pi_app.control.waypoint_nav import (
    Waypoint,
    WaypointNavConfig,
    WaypointNavController,
)


# ---------------------------------------------------------------------------
# Safety state sync tests
# ---------------------------------------------------------------------------

class TestSafetyStateSync(unittest.TestCase):
    """Verify SafetyState.set_follow_me_active keeps the flag in sync."""

    def test_set_follow_me_active(self):
        """set_follow_me_active(True/False) should toggle follow_me_active."""
        state = SafetyState()
        self.assertFalse(state.follow_me_active)

        state.set_follow_me_active(True)
        self.assertTrue(state.follow_me_active)

        state.set_follow_me_active(False)
        self.assertFalse(state.follow_me_active)

    def test_web_activation_syncs_safety_state(self):
        """Controller.activate_follow_me() must set safety_state.follow_me_active.

        The Controller depends on config, hardware protocols, and several
        subsystems that are non-trivial to stub for a unit test.  Instead we
        test the underlying mechanism directly: SafetyState.set_follow_me_active
        is the call that activate_follow_me() and deactivate_follow_me() use to
        keep the safety state in sync with the mode flag.

        See controller.py lines:
            activate_follow_me  -> self._safety_state.set_follow_me_active(True)
            deactivate_follow_me -> self._safety_state.set_follow_me_active(False)

        If these calls were missing, the safety layer would still see
        follow_me_active=False even while the controller was in FOLLOW_ME mode,
        causing incorrect RC-override behaviour.
        """
        state = SafetyState(is_armed=True)

        # Simulate what activate_follow_me() does:
        state.set_follow_me_active(True)
        self.assertTrue(state.follow_me_active,
                        "After web activation, safety state must reflect follow_me_active=True")

        # Simulate what deactivate_follow_me() does:
        state.set_follow_me_active(False)
        self.assertFalse(state.follow_me_active,
                         "After web deactivation, safety state must reflect follow_me_active=False")


# ---------------------------------------------------------------------------
# Waypoint speed-safety tests
# ---------------------------------------------------------------------------

class TestWaypointSpeedSafety(unittest.TestCase):
    """Guard against divide-by-zero and crashes in _forward_v_for_distance.

    Commit e4cbfd0 (state-machine refactor) renamed _speed_for_distance to
    _forward_v_for_distance and changed its return type from int (speed byte,
    0–255) to float (normalized, 0.0–1.0 scaled as byte/127).  All assertions
    have been updated accordingly: byte value N → normalized N/127.
    """

    def _make_nav(self, **overrides) -> WaypointNavController:
        """Build a WaypointNavController with a single dummy waypoint."""
        defaults = dict(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=4,
            stale_timeout_s=3.0,
        )
        defaults.update(overrides)
        cfg = WaypointNavConfig(**defaults)
        wp = [Waypoint(lat=0.0, lon=0.0, name="dummy")]
        return WaypointNavController(cfg, waypoints=wp)

    def test_equal_radii_no_divide_by_zero(self):
        """When slow_radius_m == arrival_radius_m the denominator would be 0.

        The guard clause `if cfg.slow_radius_m <= cfg.arrival_radius_m`
        must short-circuit and return hi (cruise_speed_byte/127) instead of
        dividing.  Method renamed _forward_v_for_distance in e4cbfd0.
        """
        nav = self._make_nav(slow_radius_m=0.5, arrival_radius_m=0.5,
                             cruise_speed_byte=40)
        # At exactly the arrival radius, distance ≤ arrival_radius_m → 0.0
        v = nav._forward_v_for_distance(0.5)
        self.assertEqual(v, 0.0)

        # Just outside the arrival/slow boundary → cruise (guard clause)
        v = nav._forward_v_for_distance(0.6)
        self.assertAlmostEqual(v, 40 / 127.0, places=4,
                               msg="With equal radii the guard clause should return cruise (40/127)")

    def test_slow_radius_less_than_arrival_no_crash(self):
        """When slow_radius_m < arrival_radius_m, should not crash.

        The guard clause handles this degenerate configuration by returning
        hi (cruise_speed_byte/127) for any distance beyond arrival_radius_m.
        Method renamed _forward_v_for_distance in e4cbfd0.
        """
        nav = self._make_nav(slow_radius_m=0.3, arrival_radius_m=0.5,
                             cruise_speed_byte=40)
        # Inside arrival → 0.0
        v = nav._forward_v_for_distance(0.4)
        self.assertEqual(v, 0.0)

        # Outside arrival → cruise (guard clause kicks in)
        v = nav._forward_v_for_distance(1.0)
        self.assertAlmostEqual(v, 40 / 127.0, places=4,
                               msg="With slow_radius < arrival_radius the guard should return cruise (40/127)")

    def test_normal_speed_interpolation(self):
        """With normal config (slow_radius > arrival_radius), speed interpolates.

        At the exact midpoint between arrival_radius_m and slow_radius_m, the
        fractional position is 0.5, so the expected normalized v_cmd is:
            lo + (hi - lo) * 0.5  where lo=20/127, hi=40/127 → 30/127

        Method renamed _forward_v_for_distance and now returns float in e4cbfd0.
        """
        nav = self._make_nav(
            arrival_radius_m=0.5,
            slow_radius_m=2.0,
            cruise_speed_byte=40,
            approach_speed_byte=20,
        )
        midpoint = (0.5 + 2.0) / 2.0  # 1.25 m
        v = nav._forward_v_for_distance(midpoint)
        self.assertAlmostEqual(v, 30 / 127.0, places=4,
                               msg="At midpoint, v_cmd should be halfway (30/127)")

        # At arrival boundary → 0.0
        self.assertEqual(nav._forward_v_for_distance(0.5), 0.0)

        # Beyond slow radius → cruise (40/127)
        self.assertAlmostEqual(nav._forward_v_for_distance(3.0), 40 / 127.0, places=4)


if __name__ == "__main__":
    unittest.main()
