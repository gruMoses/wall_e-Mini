"""Blind trail-search steer sign vs. the CW-positive heading convention.

An independent review found that follow_me.py's blind search steer sign
(``_handle_lost_target``'s tangent-derived ``steer_sign``, now the pure
helper ``_trail_tangent_steer_sign``) was written for the OLD, CCW-positive
heading convention (before commit "Show the heading state honestly in the
UI" made heading_deg genuinely CW-positive, i.e. increasing on a right
turn). ``DeadReckonOdometry.camera_to_world`` uses the same "(-sin, cos)"
world-frame vector to place a camera-right point (``x_cam > 0``) at its
world position, and that vector is correct-by-construction under the CW
convention (it IS the forward direction 90 deg further along the
heading-increasing / right-turning sense). But follow_me.py's old formula
NEGATED the same dot product on the (now-stale) assumption that it pointed
LEFT, so a trail confirmed via camera_to_world to run physically RIGHT
produced a LEFT steer command -- backwards.

This file pins the fix (the negation removed) with the concrete scenario
from the review: robot at heading 0 (facing "world +x"/north), a trail
recorded via the real DeadReckonOdometry.camera_to_world with x_cam
increasing between two points (walking further to the robot's physical
right), asserting the resulting steer sign is POSITIVE (SteeringLayer's
documented convention: positive steer offset = turn right).
"""

from __future__ import annotations

import math
import unittest

from pi_app.control.follow_me import _trail_tangent_steer_sign
from pi_app.control.odometry import DeadReckonOdometry


class TestTrailTangentSteerSign(unittest.TestCase):
    def test_rightward_trail_at_heading_zero_steers_right(self):
        """Reproduces the review's scenario exactly, using the REAL
        DeadReckonOdometry.camera_to_world (not a hand-rolled substitute)."""
        odom = DeadReckonOdometry()
        self.assertEqual(odom.pose.theta, 0.0, "robot facing heading 0 (north)")

        # Person 1 m ahead, 1 m to the robot's camera-right (x_cam=+1), then
        # walking further right and a bit further away.
        ax, ay = odom.camera_to_world(x_cam=1.0, z_cam=1.0)
        bx, by = odom.camera_to_world(x_cam=2.0, z_cam=1.5)

        tdx, tdy = bx - ax, by - ay
        tdist = math.hypot(tdx, tdy)
        self.assertGreater(tdist, 1e-6)

        steer_sign = _trail_tangent_steer_sign(tdx, tdy, odom.pose.theta) / tdist
        self.assertGreater(
            steer_sign, 0.0,
            "a trail confirmed (via camera_to_world) to go physically right "
            "must produce a POSITIVE steer sign (right), per SteeringLayer's "
            "positive=turn-right convention",
        )

    def test_leftward_trail_at_heading_zero_steers_left(self):
        odom = DeadReckonOdometry()
        ax, ay = odom.camera_to_world(x_cam=-1.0, z_cam=1.0)
        bx, by = odom.camera_to_world(x_cam=-2.0, z_cam=1.5)
        tdx, tdy = bx - ax, by - ay
        tdist = math.hypot(tdx, tdy)

        steer_sign = _trail_tangent_steer_sign(tdx, tdy, odom.pose.theta) / tdist
        self.assertLess(steer_sign, 0.0)

    def test_matches_camera_to_world_right_vector_by_construction(self):
        """_trail_tangent_steer_sign must use EXACTLY the vector
        camera_to_world uses for x_cam, at an arbitrary non-zero heading too
        (not just the degenerate heading=0 case where sign conventions
        coincide)."""
        theta = math.radians(37.0)
        odom = DeadReckonOdometry()
        odom.update(heading_deg=37.0, motor_l=126, motor_r=126, timestamp=1.0)
        self.assertAlmostEqual(odom.pose.theta, theta, places=9)

        ax, ay = odom.camera_to_world(x_cam=1.0, z_cam=2.0)
        bx, by = odom.camera_to_world(x_cam=1.6, z_cam=2.3)
        tdx, tdy = bx - ax, by - ay

        # Ground truth: the world-frame vector camera_to_world assigns to a
        # unit increase in x_cam (its own "camera-right" axis).
        ax0, ay0 = odom.camera_to_world(x_cam=0.0, z_cam=0.0)
        ax1, ay1 = odom.camera_to_world(x_cam=1.0, z_cam=0.0)
        right_axis = (ax1 - ax0, ay1 - ay0)

        dot_via_helper = _trail_tangent_steer_sign(tdx, tdy, theta)
        dot_via_ground_truth = tdx * right_axis[0] + tdy * right_axis[1]
        self.assertAlmostEqual(dot_via_helper, dot_via_ground_truth, places=9)


if __name__ == "__main__":
    unittest.main()
