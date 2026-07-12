"""Unit tests for GpsHeadingAligner."""

import unittest

from config import GpsHeadingAlignConfig
from pi_app.control.gps_heading_align import GpsHeadingAligner, _signed_error_deg


def _cfg(**overrides) -> GpsHeadingAlignConfig:
    defaults = dict(
        enabled=True,
        min_distance_m=0.8,
        min_speed_mps=0.12,
        min_fix_quality=4,
        alpha=0.1,
        history_seconds=8.0,
    )
    defaults.update(overrides)
    return GpsHeadingAlignConfig(**defaults)


def _drive_north(
    aligner: GpsHeadingAligner,
    *,
    start_lat: float = 40.0,
    start_lon: float = -74.0,
    raw_imu_deg: float = 0.0,
    steps: int = 2,
    dt_s: float = 1.0,
    lat_step: float = 0.00001,
    fix_quality: int = 4,
    base_ts: float = 1_000.0,
) -> None:
    """Simulate straight northward GPS samples (~1.1 m per lat_step)."""
    for i in range(steps):
        aligner.update(
            start_lat + i * lat_step,
            start_lon,
            raw_imu_deg,
            fix_quality,
            base_ts + i * dt_s,
        )


class TestGpsHeadingAligner(unittest.TestCase):

    def test_successful_lock(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, raw_imu_deg=5.0, steps=2)
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, -5.0, places=1)
        self.assertIsNotNone(aligner.last_cog_deg)
        self.assertAlmostEqual(aligner.last_cog_deg, 0.0, places=0)

    def test_stationary_does_not_lock(self):
        aligner = GpsHeadingAligner(_cfg())
        ts = 500.0
        for i in range(20):
            aligner.update(40.0, -74.0, 0.0, 4, ts + i * 0.5)
        self.assertFalse(aligner.locked)
        self.assertIsNone(aligner.last_cog_deg)
        self.assertIsNone(aligner.status().last_speed_mps)

    def test_low_speed_gate(self):
        aligner = GpsHeadingAligner(_cfg(min_speed_mps=2.0))
        _drive_north(aligner, dt_s=10.0, steps=2)
        self.assertFalse(aligner.locked)

    def test_bad_fix_gate_clears_history(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, steps=2)
        self.assertTrue(aligner.locked)
        offset = aligner.offset_deg
        aligner.update(40.00005, -74.0, 0.0, fix_quality=3, sample_ts=1_010.0)
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, offset, places=3)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_ema_refinement(self):
        aligner = GpsHeadingAligner(_cfg(alpha=0.5))
        _drive_north(aligner, raw_imu_deg=0.0, steps=2, base_ts=1_000.0)
        self.assertTrue(aligner.locked)
        first_offset = aligner.offset_deg
        _drive_north(
            aligner,
            raw_imu_deg=20.0,
            steps=2,
            start_lat=40.00002,
            base_ts=1_002.0,
        )
        self.assertTrue(aligner.locked)
        self.assertNotAlmostEqual(aligner.offset_deg, first_offset, places=2)

    def test_wraparound_lock_and_ema(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(40.0, -74.0, 350.0, 4, 1_000.0)
        aligner.update(40.00001, -74.0, 350.0, 4, 1_001.0)
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.correct(350.0), 0.0, places=0)

    def test_rtk_dropout_preserves_offset(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, steps=2)
        locked_offset = aligner.offset_deg
        aligner.update(40.00003, -74.0, 0.0, fix_quality=3, sample_ts=1_002.0)
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, locked_offset, places=3)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_rtk_float_rejected_for_lock_and_refinement(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(40.0, -74.0, 0.0, 4, 1_000.0)
        aligner.update(40.00001, -74.0, 5.0, 5, 1_001.0)
        self.assertFalse(aligner.locked)
        _drive_north(aligner, raw_imu_deg=5.0, steps=2, base_ts=1_002.0)
        locked_offset = aligner.offset_deg
        aligner.update(40.00004, -74.0, 5.0, fix_quality=5, sample_ts=1_004.0)
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, locked_offset, places=3)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_out_of_order_sample_ts_ignored(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(40.0, -74.0, 0.0, 4, 1_002.0)
        aligner.update(40.00001, -74.0, 0.0, 4, 1_001.0)
        self.assertEqual(aligner.status().history_samples, 1)
        self.assertFalse(aligner.locked)

    def test_reset_clears_state(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, steps=2)
        aligner.reset()
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.offset_deg, 0.0)
        self.assertEqual(aligner.status().history_samples, 0)
        self.assertIsNone(aligner.last_cog_deg)

    def test_correct_imu_target_round_trip(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, raw_imu_deg=15.0, steps=2)
        for bearing in (0.0, 45.0, 180.0, 270.0, 359.0):
            raw_target = aligner.imu_target_heading(bearing)
            self.assertAlmostEqual(aligner.correct(raw_target), bearing, places=5)

    def test_duplicate_sample_ts_ignored(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(40.0, -74.0, 0.0, 4, 1_000.0)
        aligner.update(40.0, -74.0, 0.0, 4, 1_000.0)
        self.assertEqual(aligner.status().history_samples, 1)
        aligner.update(40.00001, -74.0, 0.0, 4, 1_001.0)
        self.assertEqual(aligner.status().history_samples, 2)

    def test_controller_loop_duplicates_do_not_inflate_speed(self):
        """Repeated polls of the same GPS fix must not fake high speed."""
        aligner = GpsHeadingAligner(_cfg(min_distance_m=0.8, min_speed_mps=0.12))
        aligner.update(40.0, -74.0, 0.0, 4, 1_000.0)
        for tick in range(30):
            aligner.update(40.00001, -74.0, 0.0, 4, 1_000.0)
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 1)

    def test_disabled_is_noop(self):
        aligner = GpsHeadingAligner(_cfg(enabled=False))
        _drive_north(aligner, steps=3)
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_signed_error_wraparound_helper(self):
        self.assertAlmostEqual(_signed_error_deg(10.0, 350.0), 20.0, places=5)
        self.assertAlmostEqual(_signed_error_deg(350.0, 10.0), -20.0, places=5)


if __name__ == "__main__":
    unittest.main()
