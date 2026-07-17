"""Unit tests for GpsHeadingAligner."""

import unittest

from config import GpsHeadingAlignConfig
from pi_app.control.gps_heading_align import (
    GpsHeadingAligner,
    _bearing_deg,
    _signed_error_deg,
)


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
            lock_allowed=True,
            yaw_rate_dps=0.0,
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
            aligner.update(
                40.0, -74.0, 0.0, 4, ts + i * 0.5,
                lock_allowed=True, yaw_rate_dps=0.0,
            )
        self.assertFalse(aligner.locked)
        self.assertIsNone(aligner.last_cog_deg)
        self.assertIsNone(aligner.status().last_speed_mps)

    def test_low_speed_gate(self):
        aligner = GpsHeadingAligner(_cfg(min_speed_mps=2.0))
        _drive_north(aligner, dt_s=10.0, steps=2)
        self.assertFalse(aligner.locked)

    def test_bad_fix_after_lock_preserves_offset_and_history(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, steps=2)
        self.assertTrue(aligner.locked)
        offset = aligner.offset_deg
        history_before = aligner.status().history_samples
        aligner.update(
            40.00005, -74.0, 0.0, fix_quality=3, sample_ts=1_010.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, offset, places=3)
        self.assertEqual(aligner.status().history_samples, history_before)

    def test_bad_fix_on_new_sample_clears_unlocked_history(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 1)
        aligner.update(
            40.00001, -74.0, 0.0, fix_quality=3, sample_ts=1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_post_lock_samples_cannot_refine_offset(self):
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
        self.assertAlmostEqual(aligner.offset_deg, first_offset, places=5)
        self.assertTrue(aligner.status().frozen)
        self.assertFalse(aligner.status().refining)

    def test_correct_waypoint_trace_remains_at_initial_frozen_offset(self):
        """Replays the observed +175.7° lock and later circle samples."""
        aligner = GpsHeadingAligner(_cfg())
        start = (30.29070083, -95.29423150)
        lock_point = (30.29069217, -95.29423000)
        lock_cog = _bearing_deg(*start, *lock_point)
        lock_raw = (lock_cog - 175.7) % 360.0

        aligner.update(
            *start, lock_raw, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            *lock_point, lock_raw, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, 175.7, places=5)

        # Correct run observations span the pre-waypoint turn and waypoint
        # circle. The old EMA moved +175.7° to -34.9°, -75.1°, -143.6°,
        # +167.5°, and -103.9° at these stages.
        turn_and_waypoint_samples = [
            (30.29068867, -95.29436117, 133.1, 2_000.0, 0.0),
            (30.29071083, -95.29435150, 170.8, 2_001.0, -8.2),
            (30.29072283, -95.29436450, 246.0, 2_002.0, -4.2),
            (30.29072250, -95.29437183, 289.3, 2_003.0, -6.9),
            (30.29071233, -95.29438233, 194.8, 2_004.0, 9.5),
        ]
        for lat, lon, raw, ts, yaw in turn_and_waypoint_samples:
            aligner.update(
                lat, lon, raw, 4, ts,
                lock_allowed=False,
                yaw_rate_dps=yaw,
            )
            self.assertAlmostEqual(aligner.offset_deg, 175.7, places=5)

    def test_initial_lock_requires_explicit_straight_context(self):
        aligner = GpsHeadingAligner(_cfg())
        for i in range(3):
            aligner.update(
                40.0 + i * 0.00001, -74.0, 0.0, 4, 1_000.0 + i,
                lock_allowed=False,
                yaw_rate_dps=0.0,
            )
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_initial_lock_rejects_turning_and_clears_history(self):
        aligner = GpsHeadingAligner(_cfg(max_lock_yaw_rate_dps=3.0))
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.00001, -74.0, 5.0, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=5.1,
        )
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_wraparound_lock(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(
            40.0, -74.0, 350.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.00001, -74.0, 350.0, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.correct(350.0), 0.0, places=0)

    def test_rtk_dropout_preserves_offset(self):
        aligner = GpsHeadingAligner(_cfg())
        _drive_north(aligner, steps=2)
        locked_offset = aligner.offset_deg
        history_before = aligner.status().history_samples
        aligner.update(
            40.00003, -74.0, 0.0, fix_quality=3, sample_ts=1_002.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, locked_offset, places=3)
        self.assertEqual(aligner.status().history_samples, history_before)

    def test_rtk_float_rejected_for_lock_and_refinement(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.00001, -74.0, 5.0, 5, 1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertFalse(aligner.locked)
        _drive_north(aligner, raw_imu_deg=5.0, steps=2, base_ts=1_002.0)
        locked_offset = aligner.offset_deg
        history_before = aligner.status().history_samples
        aligner.update(
            40.00004, -74.0, 5.0, fix_quality=5, sample_ts=1_004.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, locked_offset, places=3)
        self.assertEqual(aligner.status().history_samples, history_before)

    def test_out_of_order_sample_ts_clears_history(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_002.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.00001, -74.0, 0.0, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 0)
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
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 1)
        aligner.update(
            40.00001, -74.0, 0.0, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 2)

    def test_controller_loop_duplicates_do_not_inflate_speed(self):
        """Repeated polls of the same GPS fix must not fake high speed."""
        aligner = GpsHeadingAligner(_cfg(min_distance_m=0.8, min_speed_mps=0.12))
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        for tick in range(30):
            aligner.update(
                40.00001, -74.0, 0.0, 4, 1_000.0,
                lock_allowed=True, yaw_rate_dps=0.0,
            )
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

    def test_duplicate_timestamp_yaw_spikes_do_not_clear_history(self):
        """Controller-loop yaw noise on the same GPS fix must not erase history."""
        aligner = GpsHeadingAligner(_cfg(max_lock_yaw_rate_dps=3.0))
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 1)
        for yaw in (0.0, 4.0, -5.0, 8.0, 0.0):
            aligner.update(
                40.0, -74.0, 0.0, 4, 1_000.0,
                lock_allowed=True, yaw_rate_dps=yaw,
            )
        self.assertEqual(aligner.status().history_samples, 1)
        aligner.update(
            40.00001, -74.0, 0.0, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 2)

    def test_high_yaw_new_gps_sample_clears_history(self):
        aligner = GpsHeadingAligner(_cfg(max_lock_yaw_rate_dps=3.0))
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.00001, -74.0, 0.0, 4, 1_001.0,
            lock_allowed=True, yaw_rate_dps=5.0,
        )
        self.assertEqual(aligner.status().history_samples, 0)
        self.assertFalse(aligner.locked)

    def test_duplicate_non_forward_ticks_do_not_clear_history(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        for _ in range(20):
            aligner.update(
                40.0, -74.0, 0.0, 4, 1_000.0,
                lock_allowed=False, yaw_rate_dps=0.0,
            )
        self.assertEqual(aligner.status().history_samples, 1)
        self.assertFalse(aligner.locked)

    def test_non_forward_new_sample_clears_history(self):
        aligner = GpsHeadingAligner(_cfg())
        aligner.update(
            40.0, -74.0, 0.0, 4, 1_000.0,
            lock_allowed=True, yaw_rate_dps=0.0,
        )
        aligner.update(
            40.00001, -74.0, 0.0, 4, 1_001.0,
            lock_allowed=False, yaw_rate_dps=0.0,
        )
        self.assertEqual(aligner.status().history_samples, 0)
        self.assertFalse(aligner.locked)


if __name__ == "__main__":
    unittest.main()
