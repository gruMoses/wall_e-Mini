"""Tests for DetectionFilter Rule 3 — the implied-physical-height gate.

Rule 3 exists to reject short ground blobs that YOLO labels "person" (the
config comments name a chicken; a large dog is the harder case). It computes:

    implied_h = bbox_h_norm * z_m * 2 * tan(vfov / 2)

and rejects anything under ``detect_min_person_height_m`` (1.20 m).

BUG FIXED 2026-07-26: ``detect_camera_vfov_deg`` was 65.3, derived from the
same wrong 81 deg (diagonal-as-horizontal) value as ``camera_hfov_deg``. The
factory EEPROM says the 640x352 detection frame is 42.13 deg. Because
implied_h scales with tan(vfov/2), 65.3 inflated every height by
tan(32.65)/tan(21.065) = 1.66x — so the 1.20 m gate was really a 0.72 m gate
and a dog-sized blob sailed through.

These tests pin the corrected geometry: real people pass across the whole
follow range, dog-sized blobs are rejected, and the old inflation is gone.
"""

import math
import unittest

from config import FollowMeConfig
from pi_app.control.follow_me import DetectionFilter, PersonDetection

CFG = FollowMeConfig()


def _bbox_h_for(height_m: float, z_m: float, vfov_deg: float) -> float:
    """Normalized bbox height an object of `height_m` subtends at `z_m`."""
    return height_m / (z_m * 2.0 * math.tan(math.radians(vfov_deg) / 2.0))


def _make_filter(**kw):
    params = dict(
        conf_threshold=0.0,
        min_depth_m=0.0,
        max_depth_m=100.0,
        min_bbox_area=0.0,
        edge_margin=0.0,
        min_bbox_width=0.0,
        min_person_height_m=CFG.detect_min_person_height_m,
        camera_vfov_deg=CFG.detect_camera_vfov_deg,
    )
    params.update(kw)
    return DetectionFilter(**params)


def _det(bbox_h: float, z_m: float) -> PersonDetection:
    # Centred box, full-ish width so only the height rule can reject it.
    y0 = max(0.0, 0.5 - bbox_h / 2.0)
    y1 = min(1.0, 0.5 + bbox_h / 2.0)
    return PersonDetection(
        x_m=0.0, z_m=z_m, confidence=0.9, bbox=(0.3, y0, 0.7, y1),
    )


class TestConfigIsTheMeasuredValue(unittest.TestCase):
    def test_vfov_matches_eeprom_measurement(self):
        """640x352 CAM_A, fy=456.89 -> 2*atan((352/2)/456.89) = 42.13 deg."""
        expected = math.degrees(2.0 * math.atan((352 / 2.0) / 456.89))
        self.assertAlmostEqual(CFG.detect_camera_vfov_deg, expected, places=1)

    def test_stale_diagonal_value_is_gone(self):
        self.assertNotAlmostEqual(CFG.detect_camera_vfov_deg, 65.3, places=1)


class TestRealPeoplePass(unittest.TestCase):
    """A 1.75 m adult must clear the gate across the whole follow range."""

    def test_across_follow_range(self):
        f = _make_filter()
        for z in (0.5, 1.0, 1.5, 2.5, 4.0, 6.0):
            with self.subTest(z=z):
                bbox_h = _bbox_h_for(1.75, z, CFG.detect_camera_vfov_deg)
                if bbox_h > 1.0:
                    continue  # closer than the frame can contain; not a gate case
                self.assertEqual(len(f.process([_det(bbox_h, z)])), 1)

    def test_short_adult_still_passes(self):
        f = _make_filter()
        bbox_h = _bbox_h_for(1.55, 3.0, CFG.detect_camera_vfov_deg)
        self.assertEqual(len(f.process([_det(bbox_h, 3.0)])), 1)


class TestShortBlobsRejected(unittest.TestCase):
    def test_dog_sized_blob_is_rejected(self):
        """~0.75 m — the case the OLD 1.66x inflation let straight through."""
        f = _make_filter()
        for z in (1.5, 3.0, 5.0):
            with self.subTest(z=z):
                bbox_h = _bbox_h_for(0.75, z, CFG.detect_camera_vfov_deg)
                self.assertEqual(len(f.process([_det(bbox_h, z)])), 0)

    def test_chicken_sized_blob_is_rejected(self):
        f = _make_filter()
        bbox_h = _bbox_h_for(0.45, 2.0, CFG.detect_camera_vfov_deg)
        self.assertEqual(len(f.process([_det(bbox_h, 2.0)])), 0)


class TestClippedBoxesSkipTheRule(unittest.TestCase):
    """A truncated box cannot be measured, so Rule 3 must not judge it.

    A 1.75 m adult overflows the 42.13 deg vertical frame closer than ~2.27 m.
    Once clipped, implied_h saturates at 0.77 * z, which is under the 1.20 m
    gate for anything nearer than ~1.56 m — so without this guard the operator
    is filtered out at follow_distance_m (1.5 m) itself.
    """

    def _clipped(self, z_m: float) -> PersonDetection:
        return PersonDetection(
            x_m=0.0, z_m=z_m, confidence=0.9, bbox=(0.3, 0.0, 0.7, 1.0),
        )

    def test_operator_survives_at_follow_distance(self):
        f = _make_filter()
        self.assertEqual(len(f.process([self._clipped(CFG.follow_distance_m)])), 1)

    def test_operator_survives_across_close_range(self):
        f = _make_filter()
        for z in (0.5, 0.8, 1.0, 1.5, 2.0, 2.2):
            with self.subTest(z=z):
                self.assertEqual(len(f.process([self._clipped(z)])), 1)

    def test_top_clipping_skips_the_rule(self):
        """Head out of frame => height unmeasurable => keep the detection."""
        f = _make_filter()
        head_cut = PersonDetection(
            x_m=0.0, z_m=1.5, confidence=0.9, bbox=(0.3, 0.0, 0.7, 0.6))
        self.assertEqual(len(f.process([head_cut])), 1)

    def test_bottom_clipping_alone_does_NOT_skip_the_rule(self):
        """Asymmetric on purpose. Anything resting on the ground — the animals
        this rule targets — sits against the frame bottom. If bottom contact
        excused the check, the rule would be disabled for its intended targets.
        """
        f = _make_filter()
        ground_blob = PersonDetection(
            x_m=0.0, z_m=2.17, confidence=0.87, bbox=(0.40, 0.80, 0.56, 1.0))
        self.assertEqual(len(f.process([ground_blob])), 0)

    def test_unclipped_short_blob_still_rejected_at_close_range(self):
        """The guard must not become a blanket bypass — a fully visible dog
        at 1.5 m is still rejected."""
        f = _make_filter()
        bbox_h = _bbox_h_for(0.75, 1.5, CFG.detect_camera_vfov_deg)
        self.assertLess(bbox_h, 1.0, "fixture must be unclipped for this test")
        self.assertEqual(len(f.process([_det(bbox_h, 1.5)])), 0)


class TestTheRegressionItself(unittest.TestCase):
    def test_old_vfov_would_have_admitted_a_dog(self):
        """Proves the bug was real: same blob, old vs new VFOV."""
        bbox_h = _bbox_h_for(0.75, 3.0, CFG.detect_camera_vfov_deg)
        blob = _det(bbox_h, 3.0)

        stale = _make_filter(camera_vfov_deg=65.3)
        fixed = _make_filter()

        self.assertEqual(len(stale.process([blob])), 1, "old value admitted it")
        self.assertEqual(len(fixed.process([blob])), 0, "corrected value rejects it")

    def test_inflation_factor_was_1_66x(self):
        ratio = math.tan(math.radians(65.3) / 2.0) / math.tan(
            math.radians(CFG.detect_camera_vfov_deg) / 2.0
        )
        self.assertAlmostEqual(ratio, 1.66, places=1)
        # ...which turned the nominal 1.20 m gate into a ~0.72 m one.
        self.assertAlmostEqual(CFG.detect_min_person_height_m / ratio, 0.72, places=1)


if __name__ == "__main__":
    unittest.main()
