import dataclasses
import math
import sys
import unittest
from pathlib import Path

import numpy as np

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

    def test_dt_clamp_no_teleport(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 176, 176, 0.0)
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        odo.update(0.0, 176, 176, 5.0)
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=5)
        odo.update(0.0, 176, 176, 5.1)
        self.assertAlmostEqual(odo.pose.x, 0.05, places=3)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=3)

    def test_nan_heading_ignored(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 176, 176, 0.0)
        odo.update(0.0, 176, 176, 0.1)
        x_before = odo.pose.x
        y_before = odo.pose.y
        theta_before = odo.pose.theta
        ts_before = odo.pose.timestamp
        odo.update(float('nan'), 176, 176, 0.2)
        self.assertAlmostEqual(odo.pose.x, x_before, places=5)
        self.assertAlmostEqual(odo.pose.y, y_before, places=5)
        self.assertAlmostEqual(odo.pose.theta, theta_before, places=5)
        self.assertAlmostEqual(odo.pose.timestamp, ts_before, places=5)

    def test_inf_heading_ignored(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 176, 176, 0.0)
        odo.update(0.0, 176, 176, 0.1)
        x_before = odo.pose.x
        y_before = odo.pose.y
        theta_before = odo.pose.theta
        ts_before = odo.pose.timestamp
        odo.update(float('inf'), 176, 176, 0.2)
        self.assertAlmostEqual(odo.pose.x, x_before, places=5)
        self.assertAlmostEqual(odo.pose.y, y_before, places=5)
        self.assertAlmostEqual(odo.pose.theta, theta_before, places=5)
        self.assertAlmostEqual(odo.pose.timestamp, ts_before, places=5)

    def test_spin_in_place_zero_speed(self):
        odo = DeadReckonOdometry(speed_scale=0.01)
        odo.update(0.0, 176, 76, 0.0)
        odo.update(0.0, 176, 76, 0.1)
        self.assertAlmostEqual(odo.pose.x, 0.0, places=5)
        self.assertAlmostEqual(odo.pose.y, 0.0, places=5)

    def test_frozen_pose(self):
        odo = DeadReckonOdometry()
        with self.assertRaises(dataclasses.FrozenInstanceError):
            odo.pose.x = 999


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
        cfg = TrailConfig(min_spacing_m=0.01, max_step_m=10.0, max_speed_mps=20.0)
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


class TestSmoothing(unittest.TestCase):
    """Tests for Savitzky-Golay path smoothing."""

    def _build_trail(self, points, spacing=0.01):
        """Build a TrailManager with pre-loaded points."""
        cfg = TrailConfig(min_spacing_m=spacing, smoothing_enabled=True,
                          smoothing_window=5, smoothing_poly_order=2)
        mgr = TrailManager(cfg)
        for i, (x, y) in enumerate(points):
            mgr.add_point(x, y, float(i))
        return mgr

    def test_smoothed_trail_straight_unchanged(self):
        """Straight-line trail should be nearly unchanged after smoothing."""
        points = [(float(i), 0.0) for i in range(10)]
        mgr = self._build_trail(points, spacing=0.5)
        smoothed = mgr.get_smoothed_trail()
        for i, p in enumerate(smoothed):
            self.assertAlmostEqual(p.y, 0.0, places=5)
            self.assertAlmostEqual(p.x, float(i), places=3)

    def test_smoothed_trail_reduces_noise(self):
        """Zigzag noise on a straight trail should be reduced by smoothing."""
        points = [(float(i), 0.2 * ((-1) ** i)) for i in range(20)]
        mgr = self._build_trail(points, spacing=0.5)
        raw = mgr.get_trail()
        smoothed = mgr.get_smoothed_trail()
        # Max y deviation in raw vs smoothed (interior points only)
        half = 2  # window//2
        raw_max_dev = max(abs(raw[i].y) for i in range(half, len(raw) - half))
        smooth_max_dev = max(abs(smoothed[i].y) for i in range(half, len(smoothed) - half))
        self.assertLess(smooth_max_dev, raw_max_dev)

    def test_smoothed_trail_preserves_length(self):
        """Smoothed trail should have same number of points as raw trail."""
        points = [(float(i), float(i) * 0.1) for i in range(15)]
        mgr = self._build_trail(points, spacing=0.01)
        self.assertEqual(len(mgr.get_smoothed_trail()), len(mgr.get_trail()))

    def test_smoothed_trail_too_few_points_passthrough(self):
        """Trail with fewer points than window returns raw trail."""
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        mgr = self._build_trail(points, spacing=0.5)
        # Window is 5, we only have 3 points
        raw = mgr.get_trail()
        smoothed = mgr.get_smoothed_trail()
        self.assertEqual(len(smoothed), len(raw))
        for r, s in zip(raw, smoothed):
            self.assertEqual(r.x, s.x)
            self.assertEqual(r.y, s.y)

    def test_smoothed_trail_disabled_passthrough(self):
        """With smoothing_enabled=False, get_smoothed_trail returns raw trail."""
        cfg = TrailConfig(min_spacing_m=0.01, smoothing_enabled=False)
        mgr = TrailManager(cfg)
        for i in range(10):
            mgr.add_point(float(i), 0.1 * ((-1) ** i), float(i))
        raw = mgr.get_trail()
        smoothed = mgr.get_smoothed_trail()
        for r, s in zip(raw, smoothed):
            self.assertEqual(r.x, s.x)
            self.assertEqual(r.y, s.y)

    def test_sg_coefficients_sum_to_one(self):
        """SG convolution coefficients should sum to 1.0 (preserves DC)."""
        coeffs = TrailManager._compute_sg_coefficients(5, 2)
        self.assertAlmostEqual(float(np.sum(coeffs)), 1.0, places=10)
        coeffs7 = TrailManager._compute_sg_coefficients(7, 3)
        self.assertAlmostEqual(float(np.sum(coeffs7)), 1.0, places=10)


class TestCurvature(unittest.TestCase):
    """Tests for curvature computation."""

    def _point(self, x, y, ts=0.0):
        return TrailPoint(x=x, y=y, timestamp=ts, speed_hint=0.0)

    def test_straight_line_zero_curvature(self):
        """Three collinear points should produce curvature ~0."""
        trail = [self._point(0, 0), self._point(1, 0), self._point(2, 0)]
        curvatures = TrailManager.compute_curvatures(trail)
        self.assertEqual(len(curvatures), 3)
        self.assertAlmostEqual(curvatures[1], 0.0, places=5)

    def test_circle_known_curvature(self):
        """Points on a circle of radius R should produce curvature ~1/R."""
        R = 5.0
        n = 20
        trail = []
        for i in range(n):
            theta = i * 0.1  # small arc
            trail.append(self._point(R * math.cos(theta), R * math.sin(theta)))
        curvatures = TrailManager.compute_curvatures(trail)
        # Check interior points
        for i in range(2, n - 2):
            self.assertAlmostEqual(abs(curvatures[i]), 1.0 / R, places=1)

    def test_curvature_sign_left_vs_right(self):
        """Left turn should have opposite sign to right turn."""
        left_trail = [self._point(0, 0), self._point(1, 0.5), self._point(2, 1.5)]
        right_trail = [self._point(0, 0), self._point(1, -0.5), self._point(2, -1.5)]
        left_k = TrailManager.compute_curvatures(left_trail)
        right_k = TrailManager.compute_curvatures(right_trail)
        # Opposite signs
        self.assertNotEqual(0, left_k[1])
        self.assertTrue(left_k[1] * right_k[1] < 0)

    def test_curvature_too_few_points(self):
        """Fewer than 3 points returns all zeros."""
        self.assertEqual(TrailManager.compute_curvatures([]), [])
        trail = [self._point(0, 0), self._point(1, 0)]
        self.assertEqual(TrailManager.compute_curvatures(trail), [0.0, 0.0])

    def test_curvature_endpoints_zero(self):
        """First and last points should always be 0.0."""
        trail = [self._point(0, 0), self._point(1, 1), self._point(2, 0)]
        curvatures = TrailManager.compute_curvatures(trail)
        self.assertEqual(curvatures[0], 0.0)
        self.assertEqual(curvatures[-1], 0.0)


class TestPurePursuit(unittest.TestCase):

    def _pose(self, x=0.0, y=0.0, theta=0.0, ts=0.0) -> RobotPose:
        return RobotPose(x=x, y=y, theta=theta, timestamp=ts)

    def _point(self, x: float, y: float, ts: float = 0.0) -> TrailPoint:
        return TrailPoint(x=x, y=y, timestamp=ts, speed_hint=0.0)

    def _default_config(self, **overrides) -> PursuitConfig:
        """Create PursuitConfig with sensible test defaults."""
        defaults = dict(
            lookahead_time_s=0.8,
            lookahead_min_m=0.5,
            lookahead_max_m=5.0,
            speed_scale_mps_per_byte=0.01,
        )
        defaults.update(overrides)
        return PursuitConfig(**defaults)

    def test_too_few_points_returns_none(self):
        ctrl = PurePursuitController(self._default_config())
        pose = self._pose()
        self.assertIsNone(ctrl.compute(pose, [], 50.0))
        self.assertIsNone(ctrl.compute(pose, [self._point(0, 0)], 50.0))

    def test_straight_trail_no_steer(self):
        # Large min lookahead to ensure we look far ahead on straight trail
        ctrl = PurePursuitController(self._default_config(lookahead_min_m=2.0))
        pose = self._pose()
        trail = [self._point(0, 0), self._point(5, 0), self._point(10, 0)]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertAlmostEqual(cmd.steer_byte, 0.0, places=1)

    def test_turns_toward_left_point(self):
        ctrl = PurePursuitController(self._default_config(lookahead_min_m=2.0))
        pose = self._pose()
        trail = [self._point(0, 0), self._point(2, 1)]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertLess(0, cmd.steer_byte)

    def test_turns_toward_right_point(self):
        ctrl = PurePursuitController(self._default_config(lookahead_min_m=2.0))
        pose = self._pose()
        trail = [self._point(0, 0), self._point(2, -1)]
        cmd = ctrl.compute(pose, trail, 50.0)
        self.assertIsNotNone(cmd)
        self.assertGreater(0, cmd.steer_byte)

    def test_lookahead_reaches_end(self):
        # Very large max lookahead so we go past the end of trail
        ctrl = PurePursuitController(self._default_config(lookahead_min_m=100.0, lookahead_max_m=100.0))
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


class TestAdaptiveLookahead(unittest.TestCase):
    """Tests for speed-scaled adaptive lookahead distance."""

    def _pose(self, x=0.0, y=0.0, theta=0.0):
        return RobotPose(x=x, y=y, theta=theta, timestamp=0.0)

    def _point(self, x, y):
        return TrailPoint(x=x, y=y, timestamp=0.0, speed_hint=0.0)

    def _long_straight_trail(self, length=50):
        return [self._point(float(i) * 0.5, 0.0) for i in range(length)]

    def test_lookahead_scales_with_speed(self):
        """At higher speed, lookahead should be larger."""
        cfg = PursuitConfig(
            lookahead_time_s=0.8, lookahead_min_m=0.3, lookahead_max_m=5.0,
            speed_scale_mps_per_byte=0.01, curvature_scaling_enabled=False,
        )
        ctrl = PurePursuitController(cfg)
        trail = self._long_straight_trail()
        pose = self._pose()

        cmd_slow = ctrl.compute(pose, trail, 20.0)
        ctrl.reset()
        cmd_fast = ctrl.compute(pose, trail, 100.0)

        # Fast lookahead point should be further ahead
        self.assertGreater(cmd_fast.lookahead_x, cmd_slow.lookahead_x)

    def test_lookahead_clamps_min(self):
        """At zero speed, lookahead should equal min_m."""
        cfg = PursuitConfig(
            lookahead_time_s=0.8, lookahead_min_m=0.5, lookahead_max_m=5.0,
            speed_scale_mps_per_byte=0.01, curvature_scaling_enabled=False,
        )
        ctrl = PurePursuitController(cfg)
        trail = self._long_straight_trail()
        pose = self._pose()
        cmd = ctrl.compute(pose, trail, 0.0)
        # At speed=0, lookahead = max(0.5, min(5.0, 0*0.8)) = 0.5
        # lookahead_x should be ~0.5 from origin
        self.assertIsNotNone(cmd)
        self.assertAlmostEqual(cmd.lookahead_x, 0.5, places=1)

    def test_lookahead_clamps_max(self):
        """At very high speed, lookahead should not exceed max_m."""
        cfg = PursuitConfig(
            lookahead_time_s=0.8, lookahead_min_m=0.3, lookahead_max_m=2.0,
            speed_scale_mps_per_byte=0.01, curvature_scaling_enabled=False,
        )
        ctrl = PurePursuitController(cfg)
        trail = self._long_straight_trail()
        pose = self._pose()
        # speed_byte=500 => speed_mps=5.0 => lookahead = 5.0*0.8=4.0 > max=2.0
        cmd = ctrl.compute(pose, trail, 500.0)
        self.assertIsNotNone(cmd)
        self.assertAlmostEqual(cmd.lookahead_x, 2.0, places=1)


class TestForwardSearch(unittest.TestCase):
    """Tests for forward-only closest-point search."""

    def _pose(self, x=0.0, y=0.0, theta=0.0):
        return RobotPose(x=x, y=y, theta=theta, timestamp=0.0)

    def _point(self, x, y):
        return TrailPoint(x=x, y=y, timestamp=0.0, speed_hint=0.0)

    def test_forward_search_does_not_snap_backward(self):
        """On a figure-8 crossing trail, the controller should not jump back."""
        cfg = PursuitConfig(
            lookahead_min_m=0.5, lookahead_max_m=1.0, lookahead_time_s=0.5,
            speed_scale_mps_per_byte=0.01, closest_point_search_window=10,
            curvature_scaling_enabled=False,
        )
        ctrl = PurePursuitController(cfg)
        # Trail goes out and comes back near origin
        trail = [
            self._point(0, 0),      # 0
            self._point(1, 0.5),    # 1
            self._point(2, 1),      # 2
            self._point(3, 0.5),    # 3
            self._point(4, 0),      # 4 — crosses near y=0 again
            self._point(5, -0.5),   # 5
            self._point(6, -1),     # 6
        ]
        # First compute: robot at origin, should find index 0
        pose1 = self._pose(x=0.0, y=0.0)
        cmd1 = ctrl.compute(pose1, trail, 50.0)
        self.assertIsNotNone(cmd1)
        idx_after_first = ctrl._last_closest_idx

        # Move robot forward to near point 4 (which is near y=0 again)
        # The controller should NOT snap back to index 0
        pose2 = self._pose(x=3.5, y=0.2)
        cmd2 = ctrl.compute(pose2, trail, 50.0)
        self.assertIsNotNone(cmd2)
        self.assertGreaterEqual(ctrl._last_closest_idx, idx_after_first)

    def test_closest_idx_resets_on_clear(self):
        """After reset(), the search starts from index 0."""
        cfg = PursuitConfig(curvature_scaling_enabled=False)
        ctrl = PurePursuitController(cfg)
        ctrl._last_closest_idx = 15
        ctrl.reset()
        self.assertEqual(ctrl._last_closest_idx, 0)


class TestCurvatureVelocityScaling(unittest.TestCase):
    """Tests for curvature-based velocity scaling in pure pursuit."""

    def _pose(self, x=0.0, y=0.0, theta=0.0):
        return RobotPose(x=x, y=y, theta=theta, timestamp=0.0)

    def _point(self, x, y):
        return TrailPoint(x=x, y=y, timestamp=0.0, speed_hint=0.0)

    def test_velocity_scaling_slows_in_turn(self):
        """With high curvature, output speed should be less than input speed."""
        cfg = PursuitConfig(
            lookahead_min_m=1.0, lookahead_max_m=2.0, lookahead_time_s=0.5,
            speed_scale_mps_per_byte=0.01,
            curvature_scaling_enabled=True, curvature_alpha=5.0,
            min_speed_byte=10.0,
        )
        ctrl = PurePursuitController(cfg)
        # Curved trail
        trail = [self._point(0, 0), self._point(1, 0), self._point(2, 0.5),
                 self._point(3, 1.5), self._point(4, 3.0)]
        curvatures = TrailManager.compute_curvatures(trail)
        base_speed = 60.0
        cmd = ctrl.compute(self._pose(), trail, base_speed,
                           curvatures=curvatures, now=1.0)
        self.assertIsNotNone(cmd)
        self.assertLess(cmd.speed_byte, base_speed)
        self.assertTrue(cmd.speed_limited)

    def test_velocity_scaling_straight_unchanged(self):
        """With zero curvature, output speed should equal input speed."""
        cfg = PursuitConfig(
            lookahead_min_m=1.0, lookahead_max_m=3.0, lookahead_time_s=0.5,
            speed_scale_mps_per_byte=0.01,
            curvature_scaling_enabled=True, curvature_alpha=5.0,
        )
        ctrl = PurePursuitController(cfg)
        trail = [self._point(float(i), 0.0) for i in range(10)]
        curvatures = TrailManager.compute_curvatures(trail)
        base_speed = 60.0
        cmd = ctrl.compute(self._pose(), trail, base_speed,
                           curvatures=curvatures, now=1.0)
        self.assertIsNotNone(cmd)
        self.assertAlmostEqual(cmd.speed_byte, base_speed, places=1)
        self.assertFalse(cmd.speed_limited)

    def test_curvature_disabled_passthrough(self):
        """With curvature_scaling_enabled=False, speed should not be modified."""
        cfg = PursuitConfig(
            lookahead_min_m=1.0, lookahead_max_m=3.0, lookahead_time_s=0.5,
            speed_scale_mps_per_byte=0.01,
            curvature_scaling_enabled=False,
        )
        ctrl = PurePursuitController(cfg)
        # Very curved trail
        trail = [self._point(0, 0), self._point(1, 0), self._point(1.5, 1),
                 self._point(1, 2), self._point(0, 2.5)]
        curvatures = TrailManager.compute_curvatures(trail)
        base_speed = 80.0
        cmd = ctrl.compute(self._pose(), trail, base_speed,
                           curvatures=curvatures, now=1.0)
        self.assertIsNotNone(cmd)
        self.assertAlmostEqual(cmd.speed_byte, base_speed, places=1)

    def test_pre_deceleration(self):
        """With a sharp turn ahead, speed should reduce before reaching it."""
        cfg = PursuitConfig(
            lookahead_min_m=0.5, lookahead_max_m=1.5, lookahead_time_s=0.5,
            speed_scale_mps_per_byte=0.01,
            curvature_scaling_enabled=True, curvature_alpha=10.0,
            lookahead_curvature_points=5, min_speed_byte=10.0,
        )
        ctrl = PurePursuitController(cfg)
        # Straight then sharp turn
        trail = [self._point(float(i) * 0.5, 0.0) for i in range(6)]
        trail += [self._point(3.0, 0.5), self._point(3.0, 1.5), self._point(2.5, 2.5)]
        curvatures = TrailManager.compute_curvatures(trail)
        # Sharp curvature should appear around index 5-7
        base_speed = 60.0
        cmd = ctrl.compute(self._pose(), trail, base_speed,
                           curvatures=curvatures, now=1.0)
        self.assertIsNotNone(cmd)
        # Should decelerate because lookahead sees the upcoming curve
        self.assertLessEqual(cmd.speed_byte, base_speed)


class TestIntegration(unittest.TestCase):
    """End-to-end integration tests."""

    def _point(self, x, y, ts=0.0):
        return TrailPoint(x=x, y=y, timestamp=ts, speed_hint=0.0)

    def test_smoothed_trail_fed_to_pursuit(self):
        """Noisy trail -> smoothed -> fed to pursuit -> produces valid command."""
        cfg = TrailConfig(min_spacing_m=0.01, smoothing_enabled=True,
                          smoothing_window=5, smoothing_poly_order=2)
        mgr = TrailManager(cfg)
        # Noisy trail along x-axis
        for i in range(20):
            mgr.add_point(float(i) * 0.5, 0.15 * ((-1) ** i), float(i))

        smoothed = mgr.get_smoothed_trail()
        curvatures = TrailManager.compute_curvatures(smoothed)

        pursuit_cfg = PursuitConfig(
            lookahead_min_m=1.0, lookahead_max_m=3.0, lookahead_time_s=0.5,
            speed_scale_mps_per_byte=0.01, curvature_scaling_enabled=True,
        )
        ctrl = PurePursuitController(pursuit_cfg)
        pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)
        cmd = ctrl.compute(pose, smoothed, 50.0, curvatures=curvatures, now=1.0)
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd.speed_byte, 0)


if __name__ == "__main__":
    unittest.main()
