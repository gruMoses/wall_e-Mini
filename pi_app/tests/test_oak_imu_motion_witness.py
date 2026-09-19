"""Motion witness gate for stationary gyro-bias tracking / ZUPT (2026-09-19).

Two independent reviewers found the same BLOCKER in the stationary detector
added by fb9e378: a genuine slow steady rotation (a cross-slope creep, a
gentle arc) satisfies the raw gyro/accel window's std + rate gates exactly as
well as a truly parked robot, because an IMU cannot vouch for its own
stillness. Measured: a true 1.5 deg/s turn frozen 39 of 40 s; a 0.08 deg/s^2
ramp ratcheted the bias to 70 deg/s.

The fix (this file): stationary now additionally requires a FRESH witness
from ``ImuYawProducer.set_motion_witness`` saying the wheels are physically
stopped (pushed from ``pi_app.app.main`` via
``pi_app.control.rpm_plausibility.wheels_stopped``). No witness, or a stale
one, means never stationary — no bias tracking, no ZUPT.

Also covers: the reader adopting the producer's TRACKED bias (not just its
boot calibration copy), the duplicate-branch yaw-rate freshness bound, the
``wheels_stopped`` helper itself, and ``calibrate_gyro`` pausing tracking.
"""

from __future__ import annotations

import math
import random
import unittest
from typing import List, Optional
from unittest.mock import patch

from pi_app.control.rpm_plausibility import wheels_stopped
from pi_app.hardware.oak_imu import OakImuReader, G_MSS
from pi_app.hardware.oak_imu_yaw_producer import ImuPacket, ImuYawProducer

DT = 0.01  # 100 Hz, the BMI270 cadence on this unit
SEED = 20260919


def _packet(
    ts: float,
    rng: random.Random,
    *,
    gy_dps: float = 0.0,
    gx_dps: float = 0.0,
    gz_dps: float = 0.0,
    gyro_noise_dps: float = 0.1,
    accel_noise_g: float = 0.01,
) -> ImuPacket:
    """One sample with independent Gaussian noise on each gyro axis + accel norm."""
    a_norm = (1.0 + rng.gauss(0.0, accel_noise_g)) * G_MSS
    return ImuPacket(
        device_ts_s=ts,
        host_ts_s=ts,
        gx_rads=math.radians(gx_dps + rng.gauss(0.0, gyro_noise_dps)),
        gy_rads=math.radians(gy_dps + rng.gauss(0.0, gyro_noise_dps)),
        gz_rads=math.radians(gz_dps + rng.gauss(0.0, gyro_noise_dps)),
        ax_mss=0.0,
        ay_mss=-a_norm,  # BMI270 +Y points down: gravity reads about -1 g on Y
        az_mss=0.0,
    )


def _tracking_producer(**kwargs) -> ImuYawProducer:
    prod = ImuYawProducer()
    prod.configure_stationary_tracking(
        enabled=kwargs.pop("enabled", True),
        zupt_enabled=kwargs.pop("zupt_enabled", False),
        window_s=kwargs.pop("window_s", 1.0),
        gyro_std_dps=kwargs.pop("gyro_std_dps", 0.3),
        accel_std_g=kwargs.pop("accel_std_g", 0.03),
        max_rate_dps=kwargs.pop("max_rate_dps", 2.0),
        bias_tau_s=kwargs.pop("bias_tau_s", 15.0),
    )
    assert not kwargs, f"unexpected kwargs: {kwargs}"
    return prod


class _FakeOak:
    """Producer-backed OakDepthReader stub exposing the full contract this
    commit adds: TRACKED bias + cadence in the ``_ImuState`` snapshot, and
    ``set_motion_witness`` forwarding to the producer (mirrors oak_depth.py)."""

    def __init__(self) -> None:
        self.producer = ImuYawProducer()
        self.age_s = 0.01

    def get_imu_data(self):
        snap = self.producer.snapshot()
        state = type(
            "_S",
            (),
            dict(
                ax_mss=snap.ax_mss,
                ay_mss=snap.ay_mss,
                az_mss=snap.az_mss,
                gx_rads=snap.gx_rads,
                gy_rads=snap.gy_rads,
                gz_rads=snap.gz_rads,
                timestamp=snap.timestamp,
                device_timestamp_s=snap.device_timestamp_s,
                cum_yaw_x_rad=snap.cum_yaw_x_rad,
                cum_yaw_y_rad=snap.cum_yaw_y_rad,
                cum_yaw_z_rad=snap.cum_yaw_z_rad,
                cum_yaw_grav_rad=snap.cum_yaw_grav_rad,
                yaw_generation=snap.generation,
                producer_packets_integrated=snap.packets_integrated,
                stationary=snap.stationary,
                zupt_active=snap.zupt_active,
                bias_gx_dps=snap.bias_gx_dps,
                bias_gy_dps=snap.bias_gy_dps,
                bias_gz_dps=snap.bias_gz_dps,
                producer_cadence_avg_s=snap.cadence_avg_s,
            ),
        )()
        return state, self.age_s

    def get_health(self):
        return {
            "connected": True, "reconnect_count": 0,
            "last_disconnect_ts": 0.0, "pipeline_running": True,
        }

    def get_imu_metrics(self):
        snap = self.producer.snapshot()
        return {
            "packets_received": snap.packets_received,
            "packets_integrated": snap.packets_integrated,
            "stationary": snap.stationary,
            "zupt_active": snap.zupt_active,
        }

    def get_imu_raw_gyro_dps(self):
        snap = self.producer.snapshot()
        return (
            (math.degrees(snap.gx_rads), math.degrees(snap.gy_rads),
             math.degrees(snap.gz_rads)),
            self.age_s,
        )

    def set_imu_gyro_bias_dps(self, bx, by, bz):
        self.producer.set_gyro_bias_dps(bx, by, bz)

    def set_imu_nmni(self, enabled, threshold_dps=0.3):
        self.producer.set_nmni(enabled, threshold_dps)

    def configure_stationary_tracking(self, **kwargs):
        self.producer.configure_stationary_tracking(**kwargs)

    def set_motion_witness(self, still, host_ts=None):
        ts = host_ts if host_ts is not None else self.producer.timestamp
        self.producer.set_motion_witness(bool(still), ts)

    # --- test helper --------------------------------------------------
    def feed(self, packets: List[ImuPacket], *, still: bool = True) -> None:
        if packets:
            self.set_motion_witness(still, packets[-1].host_ts_s)
        self.producer.ingest(packets)


# ─────────────────────────────────────────────────────────────────────────
# (a)(b)(c): witness presence / freshness gates stationary
# ─────────────────────────────────────────────────────────────────────────


class TestWitnessGatesStationary(unittest.TestCase):
    def test_a_quiet_stream_with_fresh_witness_becomes_stationary_and_tracks(self):
        rng = random.Random(SEED)
        prod = _tracking_producer(zupt_enabled=True)
        for i in range(300):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng)])
        self.assertTrue(prod.stationary)
        self.assertGreater(prod.bias_updates, 0)
        self.assertTrue(prod.zupt_active)

    def test_b_quiet_stream_without_witness_never_stationary_and_logs_idle(self):
        """No witness ever received: never stationary, no tracking, and a
        rate-limited WARNING once the idle threshold (5 s) is crossed."""
        rng = random.Random(SEED)
        prod = _tracking_producer()
        with self.assertLogs("pi_app.hardware.oak_imu_yaw_producer", "WARNING") as cm:
            for i in range(600):  # 6 s: crosses the 5 s idle threshold once
                t = i * DT
                prod.ingest([_packet(t, rng)])
                self.assertFalse(prod.stationary)
        self.assertEqual(prod.bias_updates, 0)
        self.assertEqual(prod.bias_gy_dps, 0.0)
        messages = [r.getMessage() for r in cm.records]
        idle_msgs = [m for m in messages if "no motion witness" in m]
        self.assertEqual(len(idle_msgs), 1, messages)

    def test_c_stale_witness_is_never_stationary(self):
        rng = random.Random(SEED)
        prod = _tracking_producer()
        prod.set_motion_witness(True, 0.0)  # set once, then let it go stale
        saw_past_timeout = False
        for i in range(300):
            t = i * DT
            prod.ingest([_packet(t, rng)])
            if t > 1.0:  # past the default 1.0 s witness_timeout_s
                saw_past_timeout = True
                self.assertFalse(prod.stationary)
        self.assertTrue(saw_past_timeout)


# ─────────────────────────────────────────────────────────────────────────
# (d)(e): the reviewers' case, and the std-gate boundary
# ─────────────────────────────────────────────────────────────────────────


class TestSlowTurnIsNeverAbsorbedAsBias(unittest.TestCase):
    def test_d_steady_slow_turn_without_witness_is_never_stationary(self):
        """THE REVIEWERS' CASE. A genuine 1.5 deg/s steady turn, quiet noise
        (std 0.1 dps), with the witness correctly saying the wheels are
        TURNING (not stopped): must never be declared stationary, must never
        engage ZUPT, must never touch the bias, and must integrate the full
        true angle (within 3%)."""
        rng = random.Random(SEED)
        prod = _tracking_producer(zupt_enabled=True, max_rate_dps=5.0)
        rate_dps = 1.5
        n = 4000  # 40 s at 100 Hz — the field-motivating duration
        for i in range(n):
            t = i * DT
            prod.set_motion_witness(False, t)  # wheels turning
            prod.ingest([_packet(t, rng, gy_dps=rate_dps, gyro_noise_dps=0.1)])
            self.assertFalse(prod.stationary)
        self.assertFalse(prod.zupt_active)
        self.assertEqual(prod.bias_updates, 0)
        self.assertEqual(prod.bias_gy_dps, 0.0)
        truth_deg = rate_dps * n * DT
        measured_deg = abs(math.degrees(prod.cum_y_rad))
        self.assertLess(abs(measured_deg - truth_deg) / truth_deg, 0.03)

    def test_d2_end_to_end_witness_from_wheels_stopped_on_a_real_pivot(self):
        """Same physical scenario as test_d, but the witness is DERIVED from
        wheels_stopped() itself (live opposite-sign RPM ~133/-133 eRPM, the
        commanded pivot bytes) rather than hand-set to False — the exact
        hole the independent review found in wheels_stopped()'s old polarity
        (RPM alone, ignoring command bytes)."""
        rng = random.Random(SEED)
        prod = _tracking_producer(zupt_enabled=True, max_rate_dps=5.0)
        rate_dps = 1.5
        n = 4000  # 40 s at 100 Hz
        left_byte, right_byte = 140, 110  # in-place pivot command
        left_rpm, right_rpm = 133.0, -133.0  # live readback for the same pivot
        for i in range(n):
            t = i * DT
            still = wheels_stopped(
                left_rpm, right_rpm, True, left_byte, right_byte,
                neutral=126, witness_min_erpm=30,
            )
            self.assertFalse(still, "wheels_stopped() must not call a real pivot stopped")
            prod.set_motion_witness(still, t)
            prod.ingest([_packet(t, rng, gy_dps=rate_dps, gyro_noise_dps=0.1)])
            self.assertFalse(prod.stationary)
        self.assertFalse(prod.zupt_active)
        self.assertEqual(prod.bias_updates, 0)
        self.assertEqual(prod.bias_gy_dps, 0.0)
        truth_deg = rate_dps * n * DT
        measured_deg = abs(math.degrees(prod.cum_y_rad))
        self.assertLess(abs(measured_deg - truth_deg) / truth_deg, 0.03)

    def test_e_hand_turn_with_witness_still_is_rejected_by_std_gate(self):
        """A jittery 1.5 dps nudge with the wheels genuinely stopped (witness
        True) must still be rejected — by the std gate on its own, so the
        witness alone is never sufficient either."""
        rng = random.Random(SEED)
        prod = _tracking_producer()
        for i in range(400):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng, gy_dps=1.5, gyro_noise_dps=0.5)])
            self.assertFalse(prod.stationary)


# ─────────────────────────────────────────────────────────────────────────
# (f): absolute max-rate bound (not residual-to-bias)
# ─────────────────────────────────────────────────────────────────────────


class TestAbsoluteMaxRateBound(unittest.TestCase):
    def test_f_hot_bias_within_bound_is_learnable(self):
        rng = random.Random(SEED)
        prod = _tracking_producer(max_rate_dps=5.0, bias_tau_s=0.5)
        for i in range(700):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng, gy_dps=2.6, gyro_noise_dps=0.05)])
        self.assertTrue(prod.stationary)
        self.assertAlmostEqual(prod.bias_gy_dps, 2.6, delta=0.2)

    def test_f_raw_mean_above_bound_is_not_stationary(self):
        rng = random.Random(SEED)
        prod = _tracking_producer(max_rate_dps=5.0)
        for i in range(400):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng, gy_dps=6.0, gyro_noise_dps=0.05)])
            self.assertFalse(prod.stationary)


# ─────────────────────────────────────────────────────────────────────────
# (g): reader adopts the producer's TRACKED bias
# ─────────────────────────────────────────────────────────────────────────


class TestReaderAdoptsTrackedBias(unittest.TestCase):
    def test_g_reader_reads_body_rates_from_tracked_bias_not_boot_copy(self):
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(
            oak,
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=False,
            stationary_bias_tracking_enabled=True,
            zupt_enabled=False,
            stationary_window_s=1.0,
            stationary_max_rate_dps=5.0,
            stationary_bias_tau_s=0.5,
        )
        imu.read()  # seed

        # Parked, quiet, witness True: track the bias to ~ -1.25 dps (the
        # measured field plateau).
        for i in range(800):
            t = i * DT
            oak.feed([_packet(t, rng, gy_dps=-1.25, gyro_noise_dps=0.05)], still=True)
            imu.read()
        self.assertAlmostEqual(oak.producer.bias_gy_dps, -1.25, delta=0.15)

        # A true +10 dps turn: raw sample reads bias + true rate = 8.75 dps.
        # The reader must subtract the TRACKED bias (-1.25), not its stale
        # self.gyro_bias_dps boot copy (still 0), to recover +10.
        t_next = 800 * DT
        oak.feed([_packet(t_next, rng, gy_dps=8.75, gyro_noise_dps=0.0)], still=False)
        data = imu.read()
        self.assertAlmostEqual(data["gz_dps"], 10.0, delta=0.15)
        health = imu.get_health()
        self.assertAlmostEqual(health["tracked_bias_dps"][1], -1.25, delta=0.15)


# ─────────────────────────────────────────────────────────────────────────
# (h): duplicate-branch live-rate freshness bound
# ─────────────────────────────────────────────────────────────────────────


class TestDuplicateBranchFreshnessBound(unittest.TestCase):
    def test_h_fresh_duplicate_reports_live_rate_stale_reports_zero(self):
        """A wedged camera thread must not hold a stale non-zero rate forever
        in the 'duplicate' branch (was unbounded, up to stale_age_s=0.5s)."""
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0, nmni_enabled=False)
        oak.feed([_packet(0.00, rng, gy_dps=50.0, gyro_noise_dps=0.0)])
        imu.read()
        oak.feed([_packet(0.02, rng, gy_dps=50.0, gyro_noise_dps=0.0)])
        imu.read()

        # Same cum, no new packets: duplicate branch. age=0.02s -> live rate.
        oak.age_s = 0.02
        d_fresh = imu.read()
        self.assertEqual(d_fresh["integrate_status"], "duplicate")
        self.assertAlmostEqual(d_fresh["gz_dps"], 50.0, places=3)

        # Same cum, age=0.3s: well past the tight duplicate-branch bound
        # (max(0.05, 3*cadence) — cadence here is ~0.02s) -> zero, not 50.
        oak.age_s = 0.3
        d_old = imu.read()
        self.assertEqual(d_old["integrate_status"], "duplicate")
        self.assertEqual(d_old["gz_dps"], 0.0)


# ─────────────────────────────────────────────────────────────────────────
# (i): wheels_stopped helper
# ─────────────────────────────────────────────────────────────────────────


class TestWheelsStopped(unittest.TestCase):
    """Command bytes gate FIRST (2026-09-19): a plausible-but-nonzero RPM used
    to be enough on its own, but on this drivetrain (14 poles, 34.2857:1,
    r=0.18415 m, 0.82 m track) the old 150 eRPM floor is ~1.68 deg/s per
    wheel, so a real 1.5 deg/s in-place pivot (~133 eRPM, opposite signs) was
    declared "stopped" — freezing the IMU heading mid-turn."""

    def test_i_rpm_plausible_both_below_threshold_is_stopped(self):
        self.assertTrue(
            wheels_stopped(10.0, -5.0, True, 126, 126, neutral=126, witness_min_erpm=30)
        )

    def test_i_rpm_plausible_one_above_threshold_is_not_stopped(self):
        self.assertFalse(
            wheels_stopped(200.0, 0.0, True, 126, 126, neutral=126, witness_min_erpm=30)
        )

    def test_i_rpm_none_falls_back_to_commanded_bytes(self):
        self.assertTrue(
            wheels_stopped(None, None, True, 127, 125, neutral=126, witness_min_erpm=30, byte_tol=2)
        )
        self.assertFalse(
            wheels_stopped(None, None, True, 140, 126, neutral=126, witness_min_erpm=30, byte_tol=2)
        )

    def test_i_implausible_rpm_falls_back_to_commanded_bytes(self):
        """rpm_plausible False (gate tripped): never trust the RPM reading,
        even if it looks like a clean 0 — fall back to commanded bytes."""
        self.assertTrue(
            wheels_stopped(0.0, 0.0, False, 126, 127, neutral=126, witness_min_erpm=30)
        )
        self.assertFalse(
            wheels_stopped(0.0, 0.0, False, 200, 126, neutral=126, witness_min_erpm=30)
        )

    def test_i_real_slow_pivot_with_plausible_rpm_is_not_stopped(self):
        """THE HOLE THE REVIEW FOUND: opposite-sign commanded bytes (an
        in-place pivot) with plausible RPM at ~133 eRPM (a real 1.5 deg/s
        pivot on this drivetrain) must never read as stopped — the command
        bytes alone already say the wheels are being driven."""
        self.assertFalse(
            wheels_stopped(133.0, -133.0, True, 140, 110, neutral=126, witness_min_erpm=30)
        )

    def test_i_spin_up_with_zero_rpm_and_off_neutral_bytes_is_not_stopped(self):
        """A command was just issued (bytes off neutral) but RPM hasn't
        caught up yet (still reads 0, plausibly — within the plausibility
        gate's spin-up window): the command bytes must gate this as moving,
        not the not-yet-caught-up RPM."""
        self.assertFalse(
            wheels_stopped(0.0, 0.0, True, 140, 126, neutral=126, witness_min_erpm=30)
        )


# ─────────────────────────────────────────────────────────────────────────
# (j): calibrate_gyro pauses and restores stationary tracking
# ─────────────────────────────────────────────────────────────────────────


class TestCalibrateGyroPausesTracking(unittest.TestCase):
    def test_j_calibrate_gyro_pauses_and_restores_stationary_tracking(self):
        oak = _FakeOak()
        imu = OakImuReader(
            oak,
            yaw_rate_source="gyro_y",
            stationary_bias_tracking_enabled=True,
            zupt_enabled=True,
        )
        with patch.object(
            oak, "configure_stationary_tracking",
            wraps=oak.configure_stationary_tracking,
        ) as mock_cfg:
            with patch.object(imu, "_sample_raw_gyro_dps", return_value=(None, float("inf"))):
                with patch("time.sleep", return_value=None):
                    imu.calibrate_gyro(duration_s=0.02)
        calls = [c.kwargs for c in mock_cfg.call_args_list]
        self.assertGreaterEqual(len(calls), 2)
        self.assertFalse(calls[0]["enabled"])
        self.assertFalse(calls[0]["zupt_enabled"])
        self.assertTrue(calls[-1]["enabled"])
        self.assertTrue(calls[-1]["zupt_enabled"])


# ─────────────────────────────────────────────────────────────────────────
# (k): ZUPT engage mid-batch keeps a real delta; the next duplicate zeroes
# ─────────────────────────────────────────────────────────────────────────


class TestZuptEngageMidBatch(unittest.TestCase):
    def test_k_engage_read_keeps_real_delta_next_duplicate_reports_zupt(self):
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(
            oak,
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=False,
            stationary_bias_tracking_enabled=True,
            zupt_enabled=True,
            stationary_window_s=1.0,
            stationary_max_rate_dps=5.0,
        )
        # Seed.
        oak.feed([_packet(0.0, rng, gy_dps=1.5)], still=True)
        imu.read()

        # Pre-window-full bulk (well short of the ~1.0s / 100-sample window):
        # every packet integrates normally (freeze=False).
        bulk = [_packet(i * DT, rng, gy_dps=1.5) for i in range(1, 50)]
        oak.feed(bulk, still=True)
        imu.read()

        # One big batch spanning the engage point with wide margin on both
        # sides (window fills around i=100): packets ~50-99 are pre-engage
        # (real, nonzero delta), ~100-249 are frozen. This is the "engage
        # mid-batch" case: one read() call folds both halves together.
        engage_batch = [_packet(i * DT, rng, gy_dps=1.5) for i in range(50, 250)]
        oak.feed(engage_batch, still=True)
        d_engage = imu.read()
        self.assertTrue(oak.producer.zupt_active, "producer never engaged ZUPT")
        self.assertEqual(d_engage["integrate_status"], "fresh")
        self.assertNotEqual(d_engage["gz_dps"], 0.0)

        # No new packets: the next read sees the same (frozen) cum. ZUPT is
        # still active and this read applied no real delta -> relabel to zupt.
        d_dup = imu.read()
        self.assertEqual(d_dup["integrate_status"], "zupt")
        self.assertEqual(d_dup["gz_dps"], 0.0)


if __name__ == "__main__":
    unittest.main()
