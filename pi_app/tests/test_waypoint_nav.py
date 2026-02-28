import math
import unittest

from pi_app.control.waypoint_nav import (
    EARTH_RADIUS_M,
    Waypoint,
    WaypointNavConfig,
    WaypointNavController,
    bearing_deg,
    haversine_m,
)


class TestHaversine(unittest.TestCase):

    def test_same_point_is_zero(self):
        self.assertAlmostEqual(haversine_m(40.0, -74.0, 40.0, -74.0), 0.0, places=3)

    def test_known_distance(self):
        # New York (40.7128, -74.0060) to Los Angeles (34.0522, -118.2437)
        dist = haversine_m(40.7128, -74.0060, 34.0522, -118.2437)
        self.assertAlmostEqual(dist, 3_944_422, delta=10_000)

    def test_short_distance(self):
        # ~111 m for 0.001 degrees latitude at equator
        dist = haversine_m(0.0, 0.0, 0.001, 0.0)
        self.assertAlmostEqual(dist, 111.2, delta=1.0)


class TestBearing(unittest.TestCase):

    def test_due_north(self):
        brg = bearing_deg(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(brg, 0.0, places=1)

    def test_due_east(self):
        brg = bearing_deg(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(brg, 90.0, places=1)

    def test_due_south(self):
        brg = bearing_deg(0.0, 0.0, -1.0, 0.0)
        self.assertAlmostEqual(brg, 180.0, places=1)

    def test_due_west(self):
        brg = bearing_deg(0.0, 0.0, 0.0, -1.0)
        self.assertAlmostEqual(brg, 270.0, places=1)


class TestSpeedProfile(unittest.TestCase):

    def _make(self, **kw) -> WaypointNavController:
        defaults = dict(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=4,
            stale_timeout_s=3.0,
        )
        defaults.update(kw)
        cfg = WaypointNavConfig(**defaults)
        wp = Waypoint(lat=0.001, lon=0.0, name="T")
        return WaypointNavController(cfg, [wp])

    def test_cruise_speed_when_far(self):
        nav = self._make()
        brg, speed = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertEqual(speed, 40)

    def test_approach_speed_near_waypoint(self):
        nav = self._make()
        # ~167 m away (inside slow_radius of 2.0 m needs to be much closer)
        # Use a waypoint very close
        nav.set_waypoints([Waypoint(lat=0.0, lon=0.000015, name="Near")])
        brg, speed = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        # ~1.67 m — inside slow_radius of 2.0 but above arrival
        self.assertGreater(speed, 0)
        self.assertLessEqual(speed, 40)

    def test_stop_at_arrival(self):
        nav = self._make()
        nav.set_waypoints([Waypoint(lat=0.0, lon=0.0, name="Here")])
        brg, speed = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertEqual(speed, 0)
        self.assertTrue(nav.completed)


class TestWaypointAdvancement(unittest.TestCase):

    def _make(self) -> WaypointNavController:
        cfg = WaypointNavConfig(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=4,
            stale_timeout_s=3.0,
        )
        wps = [
            Waypoint(lat=0.0, lon=0.0, name="A"),
            Waypoint(lat=0.001, lon=0.0, name="B"),
        ]
        return WaypointNavController(cfg, wps)

    def test_advances_past_first_waypoint(self):
        nav = self._make()
        self.assertEqual(nav.current_index, 0)
        # At first waypoint -> should advance to second
        nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertEqual(nav.current_index, 1)
        self.assertFalse(nav.completed)

    def test_completes_after_last_waypoint(self):
        nav = self._make()
        # Arrive at first
        nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        # Arrive at second
        nav.compute(0.001, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertTrue(nav.completed)

    def test_no_waypoints_returns_zero(self):
        cfg = WaypointNavConfig()
        nav = WaypointNavController(cfg, [])
        brg, speed = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertEqual(speed, 0)


class TestQualityGating(unittest.TestCase):

    def _make(self, min_q: int = 4) -> WaypointNavController:
        cfg = WaypointNavConfig(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=min_q,
            stale_timeout_s=3.0,
        )
        return WaypointNavController(cfg, [Waypoint(lat=1.0, lon=1.0)])

    def test_rtk_fixed_allowed(self):
        nav = self._make(min_q=4)
        _, speed = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertGreater(speed, 0)

    def test_gps_only_rejected(self):
        nav = self._make(min_q=4)
        _, speed = nav.compute(0.0, 0.0, fix_quality=1, gps_age_s=0.0)
        self.assertEqual(speed, 0)

    def test_stale_data_rejected(self):
        nav = self._make(min_q=1)
        _, speed = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=5.0)
        self.assertEqual(speed, 0)


if __name__ == "__main__":
    unittest.main()
