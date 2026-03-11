import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from pi_app.control.odometry import DeadReckonOdometry, RobotPose
from pi_app.control.trail import TrailManager, TrailConfig, TrailPoint
from pi_app.control.pure_pursuit import PurePursuitController, PursuitConfig


class TestDeadReckonOdometry(unittest.TestCase):

    def test_initial_pose_is_zero(self):
        odo = DeadReckonOdometry()
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.theta, 0.0, places=5)

    def test_stationary_no_movement(self):
        odo = DeadReckonOdometry()
        odo.update(0.0, 126, 126, 1.0)
        odo.update(0.0, 126, 126, 2.0)
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=5)

    def test_straight_forward(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 176, 176, 0.0)
        odo.update(0.0, 176, 176, 0.1)
        # v = 50 * 0.01 = 0.5 m/s, dt = 0.1s => 0.05m
        self.assertAlmostEqual(odo.pose.x, 0.05, places=3)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=3)

    def test_heading_changes_direction(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(90.0, 176, 176, 0.0)
        odo.update(90.0, 176, 176, 0.1)
        # v = 0.5 m/s, dt = 0.1s, heading=90deg => move in +Y
        self.assertAlmostEqual(odo.pose.x, 0.0, places=3)
        self.assertAlmostEqual(odo.pose.y, 0.05, places=3)

    def test_reverse_ignored(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 100, 100, 0.0)
        odo.update(0.0, 100, 100, 0.1)
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=5)

    def test_camera_to_world_straight(self):
        odo = DeadReckonOdometry()
        wx, wy = odo.camera_to_world(x_cam=0.0, z_cam=2.0)
        self.assertAlmostEqual(wx, 2.0, places=5)
        self.assertAlmostEqual(wy, 0.0, places=5)

    def test_camera_to_world_rotated(self):
        odo = DeadReckonOdometry()
        odo._pose = RobotPose(1.0, 1.0, math.pi / 2, 0.0)
        wx, wy = odo.camera_to_world(x_cam=0.0, z_cam=3.0)
        self.assertAlmostEqual(wx, 1.0, places=2)
        self.assertAlmostEqual(wy, 4.0, places=2)

    def test_camera_to_world_lateral(self):
        odo = DeadReckonOdometry()
        wx, wy = odo.camera_to_world(x_cam=1.0, z_cam=2.0)
        self.assertAlmostEqual(wx, 2.0, places=5)
        self.assertAlmostEqual(wy, 1.0, places=5)

    def test_reset(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 176, 176, 0.0)
        odo.update(0.0, 176, 176, 1.0)
        odo.reset()
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.theta, 0.0, places=5)


class TestTrailManager(unittest.TestCase):

    def test_add_first_point(self):
        mgr = TrailManager(TrailConfig())
        mgr.add_point(1.0, 2.0, 0.0)
        self.assertEqual(mgr.length, 1)
        self.assertAlmostEqual(mgr.get_trail()[0].x, 1.0, places=5)
        self.assertAlmostEqual(mgr.get_trail()[0].y, 2.0, places=5)

    def test_min_spacing(self):
        cfg = TrailConfig(min_spacing_m=0.5)
        mgr = TrailManager(cfg)
        mgr.add_point(0.0, 0.0, 0.0)
        mgr.add_point(0.1, 0.1, 1.0)
        self.assertEqual(mgr.length, 1)

    def test_spacing_met(self):
        cfg = TrailConfig(min_spacing_m=0.3)
        mgr = TrailManager(cfg)
        mgr.add_point(0.0, 0.0, 0.0)
        mgr.add_point(0.3, 0.0, 1.0)
        self.assertEqual(mgr.length, 2)

    def test_prune_old_points(self):
        cfg = TrailConfig(max_age_s=5.0)
        mgr = TrailManager(cfg)
        mgr.add_point(0.0, 0.0, 0.0)
        mgr.add_point(1.0, 0.0, 1.0)
        mgr.prune(0.0, 0.0, 0.0, now=10.0)
        self.assertEqual(mgr.length, 0)

    def test_prune_behind_robot(self):
        cfg = TrailConfig(consume_radius_m=0.5)
        mgr = TrailManager(cfg)
        mgr.add_point(0.0, 0.0, 0.0)
        mgr.add_point(1.0, 0.0, 1.0)
        mgr.add_point(-0.2, 0.0, 2.0)
        mgr.prune(0.0, 0.0, 0.0, now=5.0)
        trail = mgr.get_trail()
        xs = [p.x for p in trail]
        self.assertNotIn(-0.2, xs)

    def test_trail_distance(self):
        cfg = TrailConfig(min_spacing_m=0.01)
        mgr = TrailManager(cfg)
        mgr.add_point(0.0, 0.0, 0.0)
        mgr.add_point(3.0, 0.0, 1.0)
        mgr.add_point(3.0, 4.0, 2.0)
        self.assertAlmostEqual(mgr.trail_distance(), 7.0, places=3)

    def test_clear(self):
        mgr = TrailManager(TrailConfig())
        mgr.add_point(1.0, 1.0, 0.0)
        mgr.clear()
        self.assertEqual(mgr.length, 0)


class TestPurePursuit(unittest.TestCase):

    def _pose(self, x=0.0, y=0.0, theta=0.0, ts=0.0) -> RobotPose:
        return RobotPose(x=x, y=y, theta=theta, timestamp=ts)

    def _point(self, x: float, y: float, ts: float = 0.0) -> TrailPoint:
        return TrailPoint(x=x, y=y, timestamp=ts, speed_hint=0.0)

    def test_too_few_points_returns_none(self):
        ctrl = PurePursuitController(PursuitConfig())
        pose = self._pose()
        self.assertIsNone(ctrl.compute(pose, [], 50.0))
        self.assertIsNone(ctrl.compute(pose, [self._point(0, 0)], 50.0))

    def test_straight_trail_no_steer(self):
        ctrl = PurePursuitController(PursuitConfig(lookahead_base_m=2.0))
        pose = self._pose()
        trail = [self._point(0, 0), self._point(5, 0), self._point(10, 0)]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertAlmostEqual(cmd.steer_byte, 0.0, places=1)

    def test_turns_toward_left_point(self):
        ctrl = PurePursuitController(PursuitConfig(lookahead_base_m=2.0))
        pose = self._pose()
        trail = [self._point(0, 0), self._point(2, 1)]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertLess(0, cmd.steer_byte)

    def test_turns_toward_right_point(self):
        ctrl = PurePursuitController(PursuitConfig(lookahead_base_m=2.0))
        pose = self._pose()
        trail = [self._point(0, 0), self._point(2, -1)]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertGreater(0, cmd.steer_byte)

    def test_lookahead_reaches_end(self):
        ctrl = PurePursuitController(PursuitConfig(lookahead_base_m=100.0))
        pose = self._pose()
        trail = [
            self._point(0, 0),
            self._point(1, 0),
            self._point(2, 0),
            self._point(3, 0),
        ]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.lookahead_x, 3.0)
        self.assertEqual(cmd.lookahead_y, 0.0)
        self.assertEqual(cmd.trail_remaining, 1)
