"""Tests for the affine geo-transform (pixel ↔ GPS conversion)."""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from pi_app.control.geo_transform import (
    ControlPoint,
    MapCalibration,
    compute_affine,
    gps_to_pixel_pt,
    load_calibration,
    pixel_to_gps_pt,
    save_calibration,
)


# Reference point: roughly central Oklahoma (lat ~35.5, lon ~-97.5)
REF_LAT = 35.5
REF_LON = -97.5


def _make_points(n: int = 4) -> list[ControlPoint]:
    """Generate synthetic control points with a known affine relationship.

    Image: 4000x3000 pixels.
    GPS: property centred around (REF_LAT, REF_LON), roughly 200m x 150m.
    The mapping is: lat = REF_LAT + pixel_y * scale_y
                    lon = REF_LON + pixel_x * scale_x
    (simple scaling, no rotation — easy to verify analytically.)
    """
    # ~200m across 4000px => 0.05 m/px
    # At lat 35.5: 1 deg lat ~ 111,120 m, 1 deg lon ~ 90,550 m
    m_per_deg_lat = math.radians(6_371_000.0)
    m_per_deg_lon = math.radians(6_371_000.0) * math.cos(math.radians(REF_LAT))

    scale_x = 200.0 / 4000.0 / m_per_deg_lon   # pixel → degrees lon
    scale_y = 150.0 / 3000.0 / m_per_deg_lat    # pixel → degrees lat

    coords = [
        (500, 400),
        (3500, 400),
        (3500, 2600),
        (500, 2600),
    ]
    points = []
    for px, py in coords[:n]:
        lat = REF_LAT + py * scale_y
        lon = REF_LON + px * scale_x
        points.append(ControlPoint(pixel_x=px, pixel_y=py, lat=lat, lon=lon))
    return points


class TestComputeAffine(unittest.TestCase):
    """Test affine transform computation and round-trip accuracy."""

    def test_minimum_three_points(self):
        """Three points should produce a valid affine."""
        pts = _make_points(3)
        cal = compute_affine(pts)
        self.assertEqual(len(cal.residuals), 3)
        # With a perfect affine, residuals should be near zero
        self.assertLess(cal.rms_error_m, 0.01)

    def test_four_points_least_squares(self):
        """Four points with exact affine relationship → near-zero residuals."""
        pts = _make_points(4)
        cal = compute_affine(pts)
        self.assertEqual(len(cal.residuals), 4)
        self.assertLess(cal.rms_error_m, 0.01)
        for r in cal.residuals:
            self.assertLess(r, 0.01)

    def test_round_trip_pixel_gps_pixel(self):
        """pixel → GPS → pixel round-trip should return to original coords."""
        pts = _make_points(4)
        cal = compute_affine(pts)

        for p in pts:
            lat, lon = pixel_to_gps_pt(cal.pixel_to_gps, p.pixel_x, p.pixel_y)
            px_back, py_back = gps_to_pixel_pt(cal.gps_to_pixel, lat, lon)
            self.assertAlmostEqual(px_back, p.pixel_x, delta=0.5,
                                   msg=f"pixel_x round-trip failed for {p}")
            self.assertAlmostEqual(py_back, p.pixel_y, delta=0.5,
                                   msg=f"pixel_y round-trip failed for {p}")

    def test_round_trip_gps_pixel_gps(self):
        """GPS → pixel → GPS round-trip should return to original coords."""
        pts = _make_points(4)
        cal = compute_affine(pts)

        for p in pts:
            px, py = gps_to_pixel_pt(cal.gps_to_pixel, p.lat, p.lon)
            lat_back, lon_back = pixel_to_gps_pt(cal.pixel_to_gps, px, py)
            # Within ~1mm accuracy
            dist = _haversine(p.lat, p.lon, lat_back, lon_back)
            self.assertLess(dist, 0.01,
                            f"GPS round-trip error {dist:.4f}m for {p}")

    def test_interior_point_accuracy(self):
        """A point NOT used for calibration should still map accurately
        if it lies within the convex hull of the control points."""
        pts = _make_points(4)
        cal = compute_affine(pts)

        # Centre of the image
        test_px, test_py = 2000, 1500
        lat, lon = pixel_to_gps_pt(cal.pixel_to_gps, test_px, test_py)
        px_back, py_back = gps_to_pixel_pt(cal.gps_to_pixel, lat, lon)
        self.assertAlmostEqual(px_back, test_px, delta=1.0)
        self.assertAlmostEqual(py_back, test_py, delta=1.0)

    def test_too_few_points_raises(self):
        """Fewer than 3 points should raise ValueError."""
        pts = _make_points(4)[:2]
        with self.assertRaises(ValueError):
            compute_affine(pts)

    def test_collinear_points_raises(self):
        """Three collinear points should raise (singular matrix)."""
        pts = [
            ControlPoint(0, 0, REF_LAT, REF_LON),
            ControlPoint(100, 0, REF_LAT, REF_LON + 0.001),
            ControlPoint(200, 0, REF_LAT, REF_LON + 0.002),
        ]
        with self.assertRaises(ValueError):
            compute_affine(pts)


class TestPersistence(unittest.TestCase):
    """Test save/load round-trip."""

    def test_save_and_load(self):
        pts = _make_points(4)
        cal = compute_affine(pts)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name

        save_calibration(path, cal, "property.jpg", 4000, 3000)
        loaded = load_calibration(path)

        self.assertIsNotNone(loaded)
        self.assertEqual(len(loaded.control_points), 4)
        self.assertAlmostEqual(loaded.rms_error_m, cal.rms_error_m, places=6)

        # Verify the loaded transform still works
        for p in pts:
            lat, lon = pixel_to_gps_pt(loaded.pixel_to_gps, p.pixel_x, p.pixel_y)
            dist = _haversine(p.lat, p.lon, lat, lon)
            self.assertLess(dist, 0.01)

        Path(path).unlink()

    def test_load_nonexistent_returns_none(self):
        self.assertIsNone(load_calibration("/tmp/does_not_exist_12345.json"))


def _haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6_371_000.0 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


if __name__ == "__main__":
    unittest.main()
