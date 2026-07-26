"""Tests for overlay projection using the device's real intrinsics.

BUG FIXED 2026-07-26: `_world_to_pixel` derived its focal length from a
hardcoded ``hfov_deg=81.0`` default that NO caller ever overrode, and placed the
principal point at the frame centre. 81 deg is the OAK-D Lite colour sensor's
DIAGONAL FOV; the measured horizontal is 70.01 deg (fx=456.89 at 640 wide, vs
374.67 implied by 81). The principal point is +14.9 px off centre on this unit.

So every world->pixel projection the recorder drew — trail breadcrumbs, the
lookahead marker, the consume-radius arc — was ~18% off, while the obstacle
corridor ran on different geometry entirely. Anyone validating the corridor by
eyeballing the recorded overlay against the physical robot would have been
comparing against the wrong picture.

Overlay geometry now comes from OakDepthReader.get_intrinsics(), the same
per-unit calibration the corridor uses.
"""

import math
import unittest

from pi_app.hardware.oak_recorder import _resolve_intrinsics, _world_to_pixel

# Measured on the real device, CAM_A @ 640 wide (pi_app/cli/oak_intrinsics.py).
MEASURED = (456.89, 456.89, 334.95, 240.0)
# What the old hardcoded 81 deg produced at the same width.
STALE_FX = (640 / 2.0) / math.tan(math.radians(81.0) / 2.0)


class TestResolveIntrinsics(unittest.TestCase):
    def test_supplied_intrinsics_win(self):
        self.assertEqual(_resolve_intrinsics(MEASURED, 640, 480), MEASURED)

    def test_fallback_uses_config_fov_not_81(self):
        fx, fy, cx, cy = _resolve_intrinsics(None, 640, 480)
        self.assertAlmostEqual(fx, 456.9, delta=15.0)
        self.assertNotAlmostEqual(fx, STALE_FX, delta=10.0)
        self.assertAlmostEqual(cx, 320.0)
        self.assertAlmostEqual(cy, 240.0)

    def test_malformed_intrinsics_fall_back(self):
        for bad in ((0, 0, 0, 0), (-1, 1, 1, 1), ("a", "b", "c", "d"), (1, 2), ()):
            with self.subTest(bad=bad):
                fx, _fy, _cx, _cy = _resolve_intrinsics(bad, 640, 480)
                self.assertGreater(fx, 0.0)

    def test_stale_81_degree_value_is_measurably_different(self):
        """Guards the premise: 81 deg is not a rounding difference from 70."""
        self.assertAlmostEqual(STALE_FX, 374.67, places=1)
        self.assertGreater(abs(MEASURED[0] - STALE_FX) / MEASURED[0], 0.15)


class TestWorldToPixelUsesSuppliedIntrinsics(unittest.TestCase):
    """Project a point 1 m right at 5 m ahead; robot at origin facing +x."""

    ARGS = dict(wx=5.0, wy=-1.0, robot_x=0.0, robot_y=0.0, robot_theta=0.0,
                img_w=640, img_h=480, camera_height_m=0.497)

    def test_uses_measured_focal_length_and_principal_point(self):
        px, _py = _world_to_pixel(**self.ARGS, intrinsics=MEASURED)
        # x_cam = +1.0, z_cam = 5.0 -> px = cx + fx * 0.2
        self.assertEqual(px, int(334.95 + 456.89 * 0.2))

    def test_result_differs_from_the_old_hardcoded_geometry(self):
        px_new, _ = _world_to_pixel(**self.ARGS, intrinsics=MEASURED)
        px_old = int(320.0 + STALE_FX * 0.2)
        self.assertNotEqual(px_new, px_old)
        # 426 vs 394 — 32 px on a 640-wide frame (5% of width) for a point only
        # 1 m off-axis at 5 m. Two components: the 18% focal-length error scales
        # with lateral offset, and a flat ~15 px from the mis-placed principal
        # point. Both grow as the point moves further off-axis.
        self.assertGreater(abs(px_new - px_old), 25)

    def test_behind_camera_returns_none(self):
        self.assertIsNone(_world_to_pixel(
            wx=-1.0, wy=0.0, robot_x=0.0, robot_y=0.0, robot_theta=0.0,
            img_w=640, img_h=480, intrinsics=MEASURED))

    def test_offscreen_returns_none(self):
        self.assertIsNone(_world_to_pixel(
            wx=0.5, wy=-50.0, robot_x=0.0, robot_y=0.0, robot_theta=0.0,
            img_w=640, img_h=480, intrinsics=MEASURED))

    def test_no_intrinsics_still_renders(self):
        """Offline replay has no device; annotation must still work."""
        self.assertIsNotNone(_world_to_pixel(**self.ARGS, intrinsics=None))


if __name__ == "__main__":
    unittest.main()
