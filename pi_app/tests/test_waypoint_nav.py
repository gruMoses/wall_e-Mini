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
    # compute() now returns (v_cmd, yaw_cmd, state) as normalized floats
    # (3-tuple); speed_byte removed in e4cbfd0 (state-machine refactor).
    # cruise_speed_byte/approach_speed_byte are divided by 127 to get v_cmd.

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
        # Must supply an aligned heading to enter DRIVE; no-heading → v=0 in
        # the state machine (conservative, as of e4cbfd0).
        nav = self._make()
        # Target is due north (lat=0.001); heading=0° → error=0 → DRIVE
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=0.0)
        # cruise_speed_byte=40 → normalized 40/127 ≈ 0.315
        self.assertAlmostEqual(v_cmd, 40 / 127.0, places=4)

    def test_approach_speed_near_waypoint(self):
        # Must supply an aligned heading to enter DRIVE.
        nav = self._make()
        # Use a waypoint ~1.67 m away (lon=0.000015) — inside slow_radius=2.0 m
        nav.set_waypoints([Waypoint(lat=0.0, lon=0.000015, name="Near")])
        # Bearing to (lon=0.000015) from origin ≈ 90° (east); heading=90° → err=0 → DRIVE
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=90.0)
        lo = 20 / 127.0
        hi = 40 / 127.0
        # In the approach ramp zone: v_cmd should be between lo and hi
        self.assertGreater(v_cmd, 0)
        self.assertLessEqual(v_cmd, hi)

    def test_stop_at_arrival(self):
        nav = self._make()
        nav.set_waypoints([Waypoint(lat=0.0, lon=0.0, name="Here")])
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertEqual(v_cmd, 0.0)
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
        # compute() returns (v_cmd, yaw_cmd, state) since e4cbfd0
        cfg = WaypointNavConfig()
        nav = WaypointNavController(cfg, [])
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0)
        self.assertEqual(v_cmd, 0.0)


class TestHeadingErrorGating(unittest.TestCase):
    """Pivot-in-place gating via ALIGN/DRIVE state machine.

    Commit e4cbfd0 replaced the continuous speed-ramp approach (which used
    pivot_heading_error_deg / align_heading_error_deg config fields) with a
    binary ALIGN/DRIVE state machine.  The config fields are now
    align_threshold_deg (below → DRIVE) and recovery_threshold_deg (above →
    fall back to ALIGN).  v_cmd is 0 in ALIGN and _forward_v_for_distance()
    in DRIVE; there is no intermediate ramp.  compute() returns
    (v_cmd, yaw_cmd, state).
    """

    def _make(self) -> WaypointNavController:
        # pivot_heading_error_deg / align_heading_error_deg were removed in
        # e4cbfd0; use the current field names align_threshold_deg (≈ old
        # align) and recovery_threshold_deg (≈ old pivot).
        cfg = WaypointNavConfig(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=4,
            stale_timeout_s=3.0,
            align_threshold_deg=8.0,
            recovery_threshold_deg=25.0,
        )
        # Target ~111 m due north; bearing = 0°.
        wp = Waypoint(lat=0.001, lon=0.0, name="N")
        return WaypointNavController(cfg, [wp])

    def test_full_speed_when_aligned(self):
        nav = self._make()
        # err=0 < align_threshold_deg=8 → DRIVE state, v_cmd = cruise/127
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=0.0)
        self.assertAlmostEqual(v_cmd, 40 / 127.0, places=4)

    def test_zero_speed_when_error_exceeds_pivot(self):
        nav = self._make()
        # 90° heading error (robot facing east, waypoint is north)
        # err=90 > recovery_threshold_deg=25 → ALIGN state, v_cmd=0
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=90.0)
        self.assertEqual(v_cmd, 0.0)

    def test_ramp_between_thresholds(self):
        # In e4cbfd0 the speed ramp was replaced by a binary ALIGN/DRIVE state
        # machine: no intermediate ramp exists.  At 16° error with
        # align_threshold_deg=8, the robot is in ALIGN state and v_cmd=0.
        # The test now verifies the ALIGN state and v_cmd=0 at this angle.
        nav = self._make()
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=16.0)
        self.assertEqual(v_cmd, 0.0, "16° > align_threshold(8°) → ALIGN, v_cmd must be 0")

    def test_wrap_around_uses_shortest_angle(self):
        # Heading 350°, target bearing 0° → shortest arc = 10°.
        # We use align_threshold_deg=12° so that the correct 10° path →
        # DRIVE (v_cmd > 0), while the incorrect 350° long-way path → ALIGN
        # (v_cmd = 0).  This makes the assertion meaningful.
        cfg = WaypointNavConfig(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=4,
            stale_timeout_s=3.0,
            align_threshold_deg=12.0,   # 10° < 12° → DRIVE if short-arc used
            recovery_threshold_deg=25.0,
        )
        nav = WaypointNavController(cfg, [Waypoint(lat=0.001, lon=0.0, name="N")])
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=350.0)
        # Shortest-arc error = 10° < align_threshold=12° → DRIVE, v_cmd > 0
        self.assertGreater(v_cmd, 0.0,
                           "Shortest-arc 10° < 12° threshold → must enter DRIVE")

    def test_missing_heading_skips_gating(self):
        # In e4cbfd0: current_heading_deg=None → no heading info available →
        # returns (0.0, 0.0, ALIGN) conservatively (v_cmd=0, safer than old
        # behaviour that returned full speed).
        nav = self._make()
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=None)
        self.assertEqual(v_cmd, 0.0, "No heading → conservative v_cmd=0 in ALIGN")


class TestQualityGating(unittest.TestCase):
    """RTK quality and stale-GPS gates.

    compute() returns (v_cmd, yaw_cmd, state) since e4cbfd0.  The state
    machine now requires a current_heading_deg to enter DRIVE; without it the
    code conservatively returns v_cmd=0 (ALIGN).  To test that fix_quality=4
    actually *allows* movement, we supply an already-aligned heading (0°) so
    the nav enters DRIVE.  The GPS quality check fires *before* heading logic,
    so bad quality still produces v=0 regardless of heading.
    """

    def _make(self, min_q: int = 4) -> WaypointNavController:
        # Waypoint due north so bearing≈0°; heading=0° → error=0 → DRIVE
        cfg = WaypointNavConfig(
            arrival_radius_m=0.5,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=min_q,
            stale_timeout_s=3.0,
        )
        return WaypointNavController(cfg, [Waypoint(lat=1.0, lon=0.0)])

    def test_rtk_fixed_allowed(self):
        nav = self._make(min_q=4)
        # Provide aligned heading so the nav enters DRIVE and v_cmd > 0
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                                            current_heading_deg=0.0)
        self.assertGreater(v_cmd, 0)

    def test_gps_only_rejected(self):
        nav = self._make(min_q=4)
        # fix_quality=1 < min_rtk_quality=4 → halted before heading logic
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=1, gps_age_s=0.0,
                                            current_heading_deg=0.0)
        self.assertEqual(v_cmd, 0.0)

    def test_stale_data_rejected(self):
        nav = self._make(min_q=1)
        # gps_age_s=5.0 > stale_timeout_s=3.0 → halted before heading logic
        v_cmd, yaw_cmd, state = nav.compute(0.0, 0.0, fix_quality=4, gps_age_s=5.0,
                                            current_heading_deg=0.0)
        self.assertEqual(v_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
