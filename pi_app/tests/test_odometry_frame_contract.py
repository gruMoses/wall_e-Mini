"""The one odometry frame contract (pi_app/control/odometry.py), for BOTH
odometry classes and every consumer: camera placement, pure pursuit, trail
pruning and the blind-search tangent sign.

Contract: +y is 90 deg clockwise of +x, theta is clockwise-positive
(theta = radians(heading_deg), heading increases on a right turn), forward =
(cos, sin), right = (-sin, cos). GpsOdometry: +x north, +y east, true heading.

Bug pinned (found 2026-11-10, docs/status_audit_2026-11-10.md section 8):
GpsOdometry kept a counter-clockwise math angle (0 = east) but placed camera
points and was steered with the clockwise (-sin, cos) "right" vector. With a
trail point 3 m ahead and 1 m right and a 30 deg right pivot, dead reckoning
steered -1.8 bytes (left, correct) and GPS odometry +6.6 (right, wrong). Its
heading was also the raw boot-relative gyro (59-78 deg off true north on the
drives), while its positions were true GPS.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.follow_me import FollowMeController, _trail_tangent_steer_sign
from pi_app.control.gps_odometry import GpsOdometry, GpsOdometryConfig
from pi_app.control.odometry import (
    DeadReckonOdometry,
    RobotPose,
    forward_unit,
    right_unit,
    world_to_body,
)
from pi_app.control.pure_pursuit import PurePursuitController, PursuitConfig
from pi_app.control.trail import TrailConfig, TrailManager, TrailPoint

LAT0, LON0 = 30.2907, -95.2935
R_EARTH = 6_371_000.0
GYRO_BOOT_DEG = 10.0      # gyro heading while driving the calibration run
TRUE_COURSE_DEG = 80.0    # true course of that run -> offset +70 deg
BYTE = 190                # 64 bytes above neutral = 0.64 m/s at 0.01 m/s/byte
V = (BYTE - 126) * 0.01


def _ne_to_latlon(n: float, e: float) -> tuple[float, float]:
    lat = LAT0 + math.degrees(n / R_EARTH)
    lon = LON0 + math.degrees(e / (R_EARTH * math.cos(math.radians(LAT0))))
    return lat, lon


class _Driver:
    """Feeds one odometry with a simulated robot (true kinematics in N/E)."""

    def __init__(self, odom, gyro0_deg: float = GYRO_BOOT_DEG,
                 true0_deg: float = TRUE_COURSE_DEG, speed_scale: float = 0.01):
        self.odom = odom
        self.offset = true0_deg - gyro0_deg
        self.gyro = gyro0_deg
        self.n = self.e = 0.0
        self.t = 100.0
        self.scale = speed_scale
        self.next_fix = self.t
        self.fix_quality = 5

    def _feed(self, ml: int, mr: int) -> None:
        if isinstance(self.odom, GpsOdometry):
            self.odom.update_motion(self.gyro, ml, mr, self.t)
            if self.t >= self.next_fix - 1e-9:
                lat, lon = _ne_to_latlon(self.n, self.e)
                self.odom.update_gps(lat, lon, self.fix_quality, self.t)
                self.next_fix += 1.0
        else:
            self.odom.update(self.gyro, ml, mr, self.t)

    def drive(self, seconds: float, byte: int = BYTE, dt: float = 1 / 30) -> None:
        v = (byte - 126) * self.scale
        for _ in range(int(round(seconds / dt))):
            self.t += dt
            th = math.radians(self.gyro + self.offset)
            self.n += v * math.cos(th) * dt
            self.e += v * math.sin(th) * dt
            self._feed(byte, byte)

    def pivot(self, deg: float, rate_dps: float = 60.0, dt: float = 1 / 30) -> None:
        steps = max(1, int(round(abs(deg) / rate_dps / dt)))
        ml, mr = (166, 86) if deg > 0 else (86, 166)
        for _ in range(steps):
            self.t += dt
            self.gyro = (self.gyro + deg / steps) % 360.0
            self._feed(ml, mr)

    def hold(self, seconds: float, dt: float = 1 / 30) -> None:
        for _ in range(int(round(seconds / dt))):
            self.t += dt
            self._feed(126, 126)


def _ready_dr() -> tuple[DeadReckonOdometry, _Driver]:
    odom = DeadReckonOdometry(speed_scale=0.01)
    d = _Driver(odom)
    d.hold(0.2)
    return odom, d


def _ready_gps() -> tuple[GpsOdometry, _Driver]:
    """GPS odometry after a straight 6 s run: heading reference learned."""
    odom = GpsOdometry(GpsOdometryConfig(speed_scale_mps_per_byte=0.01))
    d = _Driver(odom)
    d.hold(0.5)
    d.drive(6.0)
    assert odom.pose_valid, "calibration run must teach the heading offset"
    return odom, d


def _pursuit_steer(odom, pts: list[TrailPoint], speed: float = 60.0) -> float:
    pp = PurePursuitController(PursuitConfig(max_steer_byte=40.0, curvature_scaling_enabled=False))
    cmd = pp.compute(odom.pose, pts, speed)
    assert cmd is not None
    return cmd.steer_byte


def _place(odom, x_cam: float, z_cam: float, t: float = 0.0) -> TrailPoint:
    wx, wy = odom.camera_to_world(x_cam, z_cam)
    return TrailPoint(wx, wy, t, 0.0)


FACTORIES = {"dead_reckon": _ready_dr, "gps": _ready_gps}


class TestFrameHelpers(unittest.TestCase):
    def test_right_is_clockwise_of_forward(self):
        for deg in (0.0, 37.0, 90.0, 200.0, 315.0):
            th = math.radians(deg)
            f, r = forward_unit(th), right_unit(th)
            # right = forward after a further +90 deg of (clockwise) heading
            f90 = forward_unit(th + math.pi / 2)
            self.assertAlmostEqual(r[0], f90[0], places=12)
            self.assertAlmostEqual(r[1], f90[1], places=12)
            self.assertAlmostEqual(f[0] * r[0] + f[1] * r[1], 0.0, places=12)

    def test_world_to_body_inverts_camera_to_world(self):
        for name, factory in FACTORIES.items():
            odom, d = factory()
            d.pivot(-123.0)
            for x_cam, z_cam in ((1.0, 3.0), (-2.0, 0.5), (0.0, 4.0)):
                with self.subTest(odom=name, x_cam=x_cam, z_cam=z_cam):
                    wx, wy = odom.camera_to_world(x_cam, z_cam)
                    fwd, right = world_to_body(odom.pose, wx, wy)
                    self.assertAlmostEqual(fwd, z_cam, places=9)
                    self.assertAlmostEqual(right, x_cam, places=9)


class TestPivotSteersTowardThePoint(unittest.TestCase):
    """Trail point 3 m ahead and 1 m right (bearing 18.4 deg right)."""

    def _check(self, name: str, pivot_deg: float, expect_sign: int):
        odom, d = FACTORIES[name]()
        pts = [_place(odom, 0.0, 0.3), _place(odom, 1.0, 3.0)]
        before = _pursuit_steer(odom, pts)
        self.assertGreater(before, 0.0, f"{name}: point is right before the pivot")
        d.pivot(pivot_deg)
        after = _pursuit_steer(odom, pts)
        self.assertEqual(int(math.copysign(1, after)), expect_sign,
                         f"{name}: steer {after:+.2f} after a {pivot_deg:+.0f} deg pivot")

    def test_dead_reckon_pivot_right_then_point_is_left(self):
        self._check("dead_reckon", +30.0, -1)

    def test_dead_reckon_pivot_left_then_point_is_further_right(self):
        self._check("dead_reckon", -30.0, +1)

    def test_gps_pivot_right_then_point_is_left(self):
        self._check("gps", +30.0, -1)

    def test_gps_pivot_left_then_point_is_further_right(self):
        self._check("gps", -30.0, +1)

    def test_both_odometries_agree_after_pivot(self):
        steers = {}
        for name in FACTORIES:
            odom, d = FACTORIES[name]()
            pts = [_place(odom, 0.0, 0.3), _place(odom, 1.0, 3.0)]
            d.pivot(30.0)
            steers[name] = _pursuit_steer(odom, pts)
        self.assertAlmostEqual(steers["dead_reckon"], steers["gps"], delta=0.3)


class TestTranslateThenCheck(unittest.TestCase):
    """Drive 3 m straight past points placed 2 m ahead."""

    def test_points_end_up_behind_with_their_lateral_kept(self):
        for name, factory in FACTORIES.items():
            with self.subTest(odom=name):
                odom, d = factory()
                centre = _place(odom, 0.0, 2.0)
                right = _place(odom, 1.0, 2.0)
                d.drive(3.0 / V)   # 3 m
                fwd_c, lat_c = world_to_body(odom.pose, centre.x, centre.y)
                fwd_r, lat_r = world_to_body(odom.pose, right.x, right.y)
                # GPS: the last fix is up to 1 s old and DR fills in; allow 5 cm.
                self.assertAlmostEqual(fwd_c, -1.0, delta=0.05)
                self.assertAlmostEqual(lat_c, 0.0, delta=0.05)
                self.assertAlmostEqual(fwd_r, -1.0, delta=0.05)
                self.assertAlmostEqual(lat_r, 1.0, delta=0.05)

    def test_prune_removes_only_what_was_driven_past(self):
        for name, factory in FACTORIES.items():
            with self.subTest(odom=name):
                odom, d = factory()
                tm = TrailManager(TrailConfig(min_spacing_m=0.01, max_step_m=10.0,
                                              max_speed_mps=100.0,
                                              smoothing_enabled=False))
                behind = _place(odom, 0.0, 1.0, t=1.0)
                ahead = _place(odom, 0.0, 1.6, t=1.1)
                tm.add_point(behind.x, behind.y, behind.timestamp)
                tm.add_point(ahead.x, ahead.y, ahead.timestamp)
                d.drive(1.3 / V)   # 1.3 m
                p = odom.pose
                removed = tm.prune(p.x, p.y, p.theta, now=2.0)
                self.assertEqual(removed, 1, "the point 0.3 m behind is consumed")
                self.assertEqual(tm.length, 1, "the point 0.3 m ahead stays")


class TestCameraPlacement(unittest.TestCase):
    def test_gps_camera_right_is_south_when_facing_east(self):
        """Physical check in the north-east frame: true heading 90 (east)."""
        odom, d = _ready_gps()
        # true course 80 after calibration; pivot right 10 deg -> east
        d.pivot(10.0)
        self.assertAlmostEqual(math.degrees(odom.pose.theta) % 360.0, 90.0, delta=0.5)
        x0, y0 = odom.pose.x, odom.pose.y
        wx, wy = odom.camera_to_world(1.0, 0.0)   # 1 m to the right
        self.assertAlmostEqual(wx - x0, -1.0, delta=0.02, msg="1 m south (north decreases)")
        self.assertAlmostEqual(wy - y0, 0.0, delta=0.02)
        wx, wy = odom.camera_to_world(0.0, 1.0)   # 1 m ahead
        self.assertAlmostEqual(wy - y0, 1.0, delta=0.02, msg="1 m east")

    def test_gps_heading_is_true_not_gyro(self):
        odom, _ = _ready_gps()
        self.assertAlmostEqual(math.degrees(odom.pose.theta) % 360.0, TRUE_COURSE_DEG, delta=0.5)
        self.assertAlmostEqual(odom.heading_offset_deg, TRUE_COURSE_DEG - GYRO_BOOT_DEG, delta=0.5)

    def test_camera_left_and_right_land_on_their_sides(self):
        for name, factory in FACTORIES.items():
            odom, d = factory()
            for pivot in (0.0, 75.0, -140.0):
                d.pivot(pivot) if pivot else None
                with self.subTest(odom=name, pivot=pivot):
                    for x_cam in (1.5, -1.5):
                        wx, wy = odom.camera_to_world(x_cam, 2.0)
                        _, lat = world_to_body(odom.pose, wx, wy)
                        self.assertEqual(math.copysign(1, lat), math.copysign(1, x_cam))
                        # and the same through pure pursuit
                        pts = [_place(odom, 0.0, 0.3), _place(odom, x_cam, 2.0)]
                        self.assertEqual(math.copysign(1, _pursuit_steer(odom, pts)),
                                         math.copysign(1, x_cam))


class TestSearchTangentSign(unittest.TestCase):
    def test_rightward_walk_steers_right_with_each_odometry(self):
        for name, factory in FACTORIES.items():
            odom, d = factory()
            for pivot in (0.0, 30.0, -100.0):
                d.pivot(pivot) if pivot else None
                with self.subTest(odom=name, pivot=pivot):
                    a = _place(odom, 1.0, 2.0)
                    b = _place(odom, 2.0, 2.5)
                    s = _trail_tangent_steer_sign(b.x - a.x, b.y - a.y, odom.pose.theta)
                    self.assertGreater(s, 0.0)
                    a = _place(odom, -1.0, 2.0)
                    b = _place(odom, -2.0, 2.5)
                    s = _trail_tangent_steer_sign(b.x - a.x, b.y - a.y, odom.pose.theta)
                    self.assertLess(s, 0.0)

    def test_tangent_recorded_before_a_pivot_is_judged_after_it(self):
        """Person walked straight ahead; robot then pivots 60 deg right:
        the old direction of travel is now to the robot's LEFT."""
        for name, factory in FACTORIES.items():
            with self.subTest(odom=name):
                odom, d = factory()
                a = _place(odom, 0.0, 2.0)
                b = _place(odom, 0.0, 3.0)
                d.pivot(60.0)
                s = _trail_tangent_steer_sign(b.x - a.x, b.y - a.y, odom.pose.theta)
                self.assertLess(s, 0.0)


class TestNoPoseBeforeTheReferenceIsKnown(unittest.TestCase):
    def test_dead_reckon_invalid_before_first_heading(self):
        odom = DeadReckonOdometry()
        self.assertFalse(odom.pose_valid)
        odom.update(float("nan"), 160, 160, 1.0)
        self.assertFalse(odom.pose_valid)
        odom.update(45.0, 160, 160, 1.1)
        self.assertTrue(odom.pose_valid)

    def test_dead_reckon_invalid_while_heading_not_live_then_new_frame(self):
        odom = DeadReckonOdometry()
        odom.update(10.0, 160, 160, 1.0)
        odom.update(10.0, 160, 160, 1.1)
        epoch = odom.frame_epoch
        odom.update(0.0, 160, 160, 1.2, heading_valid=False)
        self.assertFalse(odom.pose_valid)
        odom.update(0.0, 160, 160, 1.3, heading_valid=False)
        self.assertFalse(odom.pose_valid)
        odom.update(57.0, 160, 160, 1.4)
        self.assertTrue(odom.pose_valid)
        self.assertGreater(odom.frame_epoch, epoch, "a new frame after an unusable stretch")

    def test_dead_reckon_heading_jump_starts_a_new_frame(self):
        odom = DeadReckonOdometry()
        odom.update(10.0, 160, 160, 1.0)
        odom.update(12.0, 160, 160, 1.033)
        epoch = odom.frame_epoch
        odom.update(14.0, 160, 160, 1.066)
        self.assertEqual(odom.frame_epoch, epoch, "a 2 deg step is a turn")
        odom.update(80.0, 160, 160, 1.1)
        self.assertGreater(odom.frame_epoch, epoch, "a 66 deg step in one tick is not")

    def test_gps_no_pose_with_fix_but_no_straight_run(self):
        odom = GpsOdometry()
        d = _Driver(odom)
        d.hold(3.0)
        self.assertTrue(odom.has_fix)
        self.assertFalse(odom.heading_reference_known)
        self.assertFalse(odom.pose_valid)

    def test_gps_pivots_and_reversing_never_teach_the_offset(self):
        odom = GpsOdometry()
        d = _Driver(odom)
        d.hold(1.0)
        for _ in range(4):
            d.pivot(90.0, rate_dps=30.0)
        self.assertFalse(odom.heading_reference_known)
        # Reverse: the robot moves backwards, so the course is 180 deg off.
        for _ in range(int(5 / (1 / 30))):
            d.t += 1 / 30
            th = math.radians(d.gyro + d.offset)
            d.n -= 0.5 * math.cos(th) / 30
            d.e -= 0.5 * math.sin(th) / 30
            d._feed(76, 76)
        self.assertFalse(odom.heading_reference_known)

    def test_gps_needs_several_agreeing_samples(self):
        odom = GpsOdometry()
        d = _Driver(odom)
        d.hold(0.5)
        d.drive(1.05)
        self.assertFalse(odom.pose_valid, "one course sample is not enough")
        d.drive(3.0)
        self.assertTrue(odom.pose_valid)

    def test_gps_heading_survives_a_turn_and_cog_corrects_offset(self):
        """The gyro carries heading changes; the course only moves the offset
        (the old update_gyro overwrote the fused heading every tick)."""
        odom, d = _ready_gps()
        d.pivot(-45.0)
        self.assertAlmostEqual(math.degrees(odom.pose.theta) % 360.0,
                               (TRUE_COURSE_DEG - 45.0) % 360.0, delta=0.5)

    def test_gps_fix_quality_change_starts_a_new_frame(self):
        odom, d = _ready_gps()
        epoch = odom.frame_epoch
        d.drive(2.0)
        self.assertEqual(odom.frame_epoch, epoch)
        d.fix_quality = 2
        d.drive(1.5)
        self.assertGreater(odom.frame_epoch, epoch)
        self.assertTrue(odom.pose_valid, "the heading offset stays known")

    def test_gps_heading_not_live_hides_the_pose_and_forgets_offset_frame(self):
        odom, d = _ready_gps()
        odom.update_motion(0.0, 160, 160, d.t + 0.03, heading_valid=False)
        self.assertFalse(odom.pose_valid)
        odom.update_motion(d.gyro, 160, 160, d.t + 0.06)
        self.assertFalse(odom.heading_reference_known,
                         "the gyro frame may have moved; relearn the offset")


class TestFollowMeUsesOneValidFrame(unittest.TestCase):
    def _fm(self, **kw) -> FollowMeController:
        return FollowMeController(FollowMeConfig(trail_follow_enabled=True, **kw))

    def test_default_source_is_dead_reckon_even_with_gps_fix(self):
        fm = self._fm()
        fm.update_pose(10.0, 126, 126, 1.0)
        lat, lon = _ne_to_latlon(0.0, 0.0)
        fm.update_gps(lat, lon, 5, 1.0)
        self.assertIs(fm._pick_odometry(), fm._odometry)
        self.assertEqual(fm._odom_source, "dead_reckon")

    def test_gps_source_offers_nothing_until_reference_known(self):
        fm = self._fm(trail_odometry_source="gps")
        fm.update_pose(10.0, 126, 126, 1.0)
        lat, lon = _ne_to_latlon(0.0, 0.0)
        fm.update_gps(lat, lon, 5, 1.0)
        self.assertIsNone(fm._pick_odometry())
        self.assertEqual(fm._odom_source, "none")

    def test_stale_heading_offers_nothing(self):
        fm = self._fm()
        fm.update_pose(0.0, 160, 160, 1.0, heading_valid=False)
        self.assertIsNone(fm._pick_odometry())

    def test_frame_change_clears_the_trail(self):
        fm = self._fm()
        fm.update_pose(10.0, 126, 126, 1.0)
        odom = fm._pick_odometry()
        self.assertIsNotNone(odom)
        fm._trail.add_point(1.0, 0.0, 1.0)
        fm._trail.add_point(2.0, 0.0, 1.1)
        self.assertEqual(fm._trail.length, 2)
        self.assertIs(fm._pick_odometry(), odom)
        self.assertEqual(fm._trail.length, 2, "same frame: trail kept")
        fm.update_pose(0.0, 126, 126, 1.2, heading_valid=False)
        fm.update_pose(200.0, 126, 126, 1.3)
        self.assertIs(fm._pick_odometry(), odom)
        self.assertEqual(fm._trail.length, 0, "new frame: old points discarded")

    def test_flag_off_builds_no_odometry(self):
        fm = FollowMeController(FollowMeConfig(trail_follow_enabled=False))
        self.assertFalse(fm.uses_odometry)
        fm.update_pose(10.0, 160, 160, 1.0)
        fm.update_pose(10.0, 160, 160, 1.1, heading_valid=False)
        self.assertIsNone(fm._pick_odometry())


class TestRobotPoseUnchanged(unittest.TestCase):
    def test_robot_pose_fields(self):
        p = RobotPose(1.0, 2.0, 0.5, 3.0)
        self.assertEqual((p.x, p.y, p.theta, p.timestamp), (1.0, 2.0, 0.5, 3.0))


if __name__ == "__main__":
    unittest.main()
