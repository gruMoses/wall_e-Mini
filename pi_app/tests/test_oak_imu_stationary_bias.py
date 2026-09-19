"""Stationary gyro-bias tracking and ZUPT on the OAK IMU producer.

Field evidence that motivated this (2026-09-19, robot on 3923f23, OAK-D Lite
BMI270, imu source oak_d): parked, the heading wound +0.9 deg/s. The robot JSON
log showed gy_body_dps (the bias-subtracted body rate about Y) at -0.03 right
after a GOOD 3 s boot calibration, then -0.46 (11:16), -0.83 (11:20), -1.07
(11:36), -1.25 (11:49), plateauing near -1.2 deg/s. That is thermal gyro-bias
drift with nothing tracking it: NMNI at 0.3 deg/s cannot gate a 1.2 deg/s rate,
and the old reader-side bias_adapt only ran when |rate| < 0.3 deg/s, so it could
never engage once the drift passed that threshold.
"""

from __future__ import annotations

import math
import random
import unittest
from typing import List, Optional

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


class TestStationaryDetection(unittest.TestCase):
    def test_quiet_stream_becomes_stationary_after_about_one_window(self):
        """A quiet gyro/accel window is necessary but no longer sufficient:
        stationary also requires a fresh wheels-stopped witness (2026-09-19),
        so this test now feeds one every packet (the robot really is parked)."""
        rng = random.Random(SEED)
        prod = _tracking_producer()
        seen_stationary_at: Optional[float] = None
        for i in range(400):  # 4 s
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng)])
            if prod.stationary and seen_stationary_at is None:
                seen_stationary_at = t
        self.assertIsNotNone(seen_stationary_at, "never detected a stationary robot")
        # Window is 1.0 s; allow the first packet (restart seed) plus slack.
        self.assertGreaterEqual(seen_stationary_at, 1.0)
        self.assertLess(seen_stationary_at, 1.3)
        self.assertTrue(prod.stationary)

    def test_fast_turn_is_never_stationary(self):
        rng = random.Random(SEED)
        prod = _tracking_producer()
        for i in range(400):
            prod.ingest([_packet(i * DT, rng, gy_dps=90.0)])
            self.assertFalse(prod.stationary)

    def test_slow_jittery_turn_is_never_stationary(self):
        """20 dps with 1 dps jitter: the std gate AND the max-rate gate reject it."""
        rng = random.Random(SEED)
        prod = _tracking_producer()
        for i in range(400):
            prod.ingest([_packet(i * DT, rng, gy_dps=20.0, gyro_noise_dps=1.0)])
            self.assertFalse(prod.stationary)

    def test_gap_resets_the_window(self):
        rng = random.Random(SEED)
        prod = _tracking_producer()
        for i in range(200):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng)])
        self.assertTrue(prod.stationary)
        # A gap longer than max_integrate_dt_s must drop the window, not leave
        # two samples straddling it and claim a "full" window.
        prod.set_motion_witness(True, 2.0 + 5.0)
        prod.ingest([_packet(2.0 + 5.0, rng)])
        self.assertFalse(prod.stationary)

    def test_disabled_tracking_still_reports_stationary_but_changes_nothing(self):
        rng = random.Random(SEED)
        prod = ImuYawProducer()  # defaults: tracking OFF, zupt OFF
        for i in range(300):
            prod.ingest([_packet(i * DT, rng, gy_dps=5.0)])
        self.assertFalse(prod.stationary_tracking_enabled)
        self.assertFalse(prod.zupt_enabled)
        self.assertFalse(prod.zupt_active)
        self.assertEqual(prod.bias_updates, 0)
        self.assertEqual(prod.bias_gy_dps, 0.0)
        # Integration is untouched: ~5 dps for ~3 s.
        self.assertAlmostEqual(math.degrees(prod.cum_y_rad), 5.0 * 2.99, delta=0.5)


class TestBiasTracking(unittest.TestCase):
    """A stationary stream whose TRUE bias ramps 0 -> -1.2 dps over 30 s.

    30 s is a caricature: on the robot the same 1.2 dps took about 37 minutes.
    A first-order tracker lags a ramp by ``slope * tau``, and the heading error
    that leaks through is therefore ``delta_bias * tau`` regardless of how long
    the ramp takes. The accelerated ramp here uses a proportionally scaled tau
    (0.5 s instead of the production 15 s) so the mechanism is what is under
    test rather than the time compression; the production tau is exercised
    against a field-rate ramp in the companion test below.
    """

    RAMP_END_DPS = -1.2
    RAMP_S = 30.0
    SCALED_TAU_S = 0.5

    def _run_ramp(
        self,
        prod: ImuYawProducer,
        *,
        ramp_end_dps: float = RAMP_END_DPS,
        ramp_s: float = RAMP_S,
        batch: int = 1,
    ) -> None:
        """A PARKED robot whose gyro bias itself drifts (thermal), not a real
        turn — the wheels-stopped witness is genuinely True throughout."""
        rng = random.Random(SEED)
        n = int(ramp_s / DT)
        i = 0
        while i < n:
            packets = []
            for _ in range(batch):
                if i >= n:
                    break
                t = i * DT
                packets.append(_packet(t, rng, gy_dps=ramp_end_dps * (t / ramp_s)))
                i += 1
            prod.set_motion_witness(True, packets[-1].host_ts_s)
            prod.ingest(packets)

    def test_tracking_on_keeps_the_heading_still(self):
        prod = _tracking_producer(zupt_enabled=False, bias_tau_s=self.SCALED_TAU_S)
        self._run_ramp(prod)
        self.assertLess(
            abs(math.degrees(prod.cum_y_rad)), 2.0,
            f"bias tracking failed: cum_y = {math.degrees(prod.cum_y_rad):.2f} deg",
        )
        self.assertLess(abs(prod.bias_gy_dps - self.RAMP_END_DPS), 0.15)
        self.assertGreater(prod.bias_updates, 0)

    def test_tracking_off_winds_up_the_heading(self):
        prod = ImuYawProducer()  # tracking disabled (today's behaviour)
        self._run_ramp(prod)
        self.assertGreater(
            abs(math.degrees(prod.cum_y_rad)), 15.0,
            "untracked bias ramp should wind the heading well past 15 deg",
        )

    def test_nmni_alone_cannot_gate_the_drifted_rate(self):
        """Documents why NMNI is not a fix: 1.2 dps is far above its 0.3 threshold."""
        prod = ImuYawProducer()
        prod.set_nmni(True, 0.3)
        self._run_ramp(prod)
        self.assertGreater(abs(math.degrees(prod.cum_y_rad)), 15.0)

    def test_production_tau_against_a_field_rate_ramp(self):
        """Production tau (15 s) over a 20-minute ramp to the measured -1.2 dps.

        Pins the real-world behaviour of the shipped default: the tracker
        converges to the true bias, and the heading wind-up drops by well over
        an order of magnitude versus no tracking. The residual (about
        delta_bias * tau = 18 deg over the whole ramp) is why ZUPT is enabled
        alongside it: while the robot is provably still, nothing integrates at
        all, so the residual never reaches the heading.
        """
        ramp_s = 1200.0
        tracked = _tracking_producer(zupt_enabled=False, bias_tau_s=15.0)
        self._run_ramp(tracked, ramp_s=ramp_s, batch=10)
        untracked = ImuYawProducer()
        self._run_ramp(untracked, ramp_s=ramp_s, batch=10)

        self.assertLess(abs(tracked.bias_gy_dps - self.RAMP_END_DPS), 0.05)
        tracked_deg = abs(math.degrees(tracked.cum_y_rad))
        untracked_deg = abs(math.degrees(untracked.cum_y_rad))
        self.assertGreater(untracked_deg, 600.0)
        self.assertLess(tracked_deg, untracked_deg * 0.1)

    def test_zupt_removes_the_residual_entirely(self):
        """With ZUPT on (the production default) a parked robot cannot drift."""
        prod = _tracking_producer(zupt_enabled=True, bias_tau_s=15.0)
        self._run_ramp(prod, ramp_s=1200.0, batch=10)
        self.assertTrue(prod.zupt_active)
        # A couple of window-lengths of integration before ZUPT engaged is all
        # that is allowed; after that the heading is frozen.
        self.assertLess(abs(math.degrees(prod.cum_y_rad)), 0.5)


class TestZupt(unittest.TestCase):
    def test_cum_frozen_while_stationary(self):
        rng = random.Random(SEED)
        prod = _tracking_producer(zupt_enabled=True)
        for i in range(300):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng, gy_dps=-1.2)])
        self.assertTrue(prod.zupt_active)
        self.assertGreaterEqual(prod.zupt_engage_count, 1)
        frozen_at = prod.cum_y_rad
        for i in range(300, 600):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng, gy_dps=-1.2)])
        self.assertAlmostEqual(prod.cum_y_rad, frozen_at, places=12)
        # The packet counter still advances: consumers rely on it being
        # monotonic to detect a producer replacement.
        self.assertGreater(prod.packets_integrated, 500)

    def test_turn_resumes_integration_within_one_window(self):
        rng = random.Random(SEED)
        prod = _tracking_producer(zupt_enabled=True)
        # 3 s parked
        for i in range(300):
            t = i * DT
            prod.set_motion_witness(True, t)
            prod.ingest([_packet(t, rng)])
        self.assertTrue(prod.zupt_active)

        # 30 dps turn for 3 s, starting mid-window. Wheels turning now — the
        # witness alone would already prevent stationary, but the point of
        # this test is that the std/rate gates resume integration on their
        # own even before a witness update; leave the witness at its last
        # (still-fresh) True value to isolate that.
        turn_start = 300
        resumed_at: Optional[int] = None
        for i in range(turn_start, turn_start + 300):
            prod.ingest([_packet(i * DT, rng, gy_dps=30.0)])
            if not prod.zupt_active and resumed_at is None:
                resumed_at = i
        self.assertIsNotNone(resumed_at, "ZUPT never disengaged during a 30 dps turn")
        # Must resume well inside one window (1.0 s = 100 packets).
        self.assertLess((resumed_at - turn_start) * DT, 1.0)

        integrated_deg = abs(math.degrees(prod.cum_y_rad))
        # Truth is the part of the turn that was actually integrated.
        truth_deg = 30.0 * (turn_start + 300 - resumed_at) * DT
        self.assertAlmostEqual(integrated_deg, truth_deg, delta=0.05 * truth_deg)

    def test_engage_and_disengage_log_once_per_ten_seconds(self):
        rng = random.Random(SEED)
        prod = _tracking_producer(zupt_enabled=True)
        with self.assertLogs("pi_app.hardware.oak_imu_yaw_producer", "WARNING") as cm:
            for i in range(300):
                t = i * DT
                prod.set_motion_witness(True, t)
                prod.ingest([_packet(t, rng)])
            self.assertTrue(prod.zupt_active)
            for i in range(300, 500):
                t = i * DT
                prod.set_motion_witness(False, t)  # wheels turning
                prod.ingest([_packet(t, rng, gy_dps=90.0)])
            self.assertFalse(prod.zupt_active)
        messages = [r.getMessage() for r in cm.records]
        self.assertTrue(any("zupt_engage" in m for m in messages), messages)
        self.assertTrue(any("zupt_disengage" in m for m in messages), messages)
        self.assertTrue(any("delta_since_last_log" in m for m in messages), messages)
        # Rate limit: a second engage inside 10 s must not add a record.
        before = len(cm.records)
        with self.assertRaises(AssertionError):
            # assertLogs raises if nothing is logged — that is the assertion
            # we want here (the re-engage is rate-limited into silence).
            with self.assertLogs("pi_app.hardware.oak_imu_yaw_producer", "WARNING"):
                for i in range(500, 800):
                    t = i * DT
                    prod.set_motion_witness(True, t)  # parked again
                    prod.ingest([_packet(t, rng)])
        self.assertTrue(prod.zupt_active)
        self.assertGreaterEqual(prod.zupt_engage_count, 2)
        self.assertEqual(len(cm.records), before)


class _ZuptFakeOak:
    """OakDepthReader stub exposing producer cum plus the ZUPT flags."""

    def __init__(self) -> None:
        self.producer = ImuYawProducer()
        self.age_s = 0.01
        self._stationary_cfg: dict = {}

    # --- OakImuReader contract -------------------------------------------
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
            ),
        )()
        return state, self.age_s

    def get_health(self):
        return {"connected": True, "reconnect_count": 0,
                "last_disconnect_ts": 0.0, "pipeline_running": True}

    def get_imu_metrics(self):
        snap = self.producer.snapshot()
        return {
            "packets_received": snap.packets_received,
            "packets_integrated": snap.packets_integrated,
            "stationary": snap.stationary,
            "zupt_active": snap.zupt_active,
            "zupt_engage_count": snap.zupt_engage_count,
            "bias_updates": snap.bias_updates,
            "bias_gx_dps": snap.bias_gx_dps,
            "bias_gy_dps": snap.bias_gy_dps,
            "bias_gz_dps": snap.bias_gz_dps,
            "window_gyro_std_dps": snap.window_gyro_std_dps,
            "window_accel_std_g": snap.window_accel_std_g,
            "last_bias_update_host_ts": snap.last_bias_update_host_ts,
            "stationary_tracking_enabled": self.producer.stationary_tracking_enabled,
            "zupt_enabled": self.producer.zupt_enabled,
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
        self._stationary_cfg = dict(kwargs)
        self.producer.configure_stationary_tracking(**kwargs)

    # --- test helper ------------------------------------------------------
    def feed(self, packets: List[ImuPacket], *, still: bool = True) -> None:
        """Feed packets, pushing a wheels-stopped witness alongside them.

        Default True: every existing caller in this file represents a parked
        robot (that was the implicit assumption before the motion-witness
        gate existed), so the default preserves those tests unchanged.
        """
        if packets:
            self.producer.set_motion_witness(bool(still), packets[-1].host_ts_s)
        self.producer.ingest(packets)


class TestReaderZuptIntegration(unittest.TestCase):
    def _reader(self, **kwargs):
        oak = _ZuptFakeOak()
        defaults = dict(
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=False,
            stationary_bias_tracking_enabled=True,
            zupt_enabled=True,
            stationary_window_s=1.0,
        )
        defaults.update(kwargs)
        return oak, OakImuReader(oak, **defaults)

    def test_reader_pushes_config_into_the_producer(self):
        oak, _imu = self._reader(
            stationary_gyro_std_dps=0.25,
            stationary_accel_std_g=0.02,
            stationary_max_rate_dps=1.5,
            stationary_bias_tau_s=20.0,
        )
        self.assertTrue(oak.producer.stationary_tracking_enabled)
        self.assertTrue(oak.producer.zupt_enabled)
        self.assertAlmostEqual(oak.producer.stationary_window_s, 1.0)
        self.assertAlmostEqual(oak.producer.stationary_gyro_std_dps, 0.25)
        self.assertAlmostEqual(oak.producer.stationary_accel_std_g, 0.02)
        self.assertAlmostEqual(oak.producer.stationary_max_rate_dps, 1.5)
        self.assertAlmostEqual(oak.producer.stationary_bias_tau_s, 20.0)

    def test_reader_reports_zero_rate_and_zupt_status(self):
        rng = random.Random(SEED)
        oak, imu = self._reader()
        imu.read()  # seed
        for i in range(300):
            oak.feed([_packet(i * DT, rng, gy_dps=-1.2)])
            data = imu.read()
        self.assertTrue(oak.producer.zupt_active)
        self.assertEqual(data["integrate_status"], "zupt")
        self.assertEqual(data["gz_dps"], 0.0)
        self.assertEqual(data["yaw_rate_world_dps"], 0.0)
        health = imu.get_health()
        self.assertTrue(health["zupt_active"])
        self.assertTrue(health["stationary"])
        self.assertEqual(health["yaw_rate_world_dps"], 0.0)

    def test_health_and_metrics_carry_the_new_fields(self):
        rng = random.Random(SEED)
        oak, imu = self._reader()
        imu.read()
        for i in range(300):
            oak.feed([_packet(i * DT, rng)])
        imu.read()
        health = imu.get_health()
        for key in (
            "zupt_active",
            "stationary",
            "gyro_bias_dps",
            "zupt_engage_count",
            "bias_updates",
            "bias_gx_dps",
            "bias_gy_dps",
            "bias_gz_dps",
            "window_gyro_std_dps",
            "window_accel_std_g",
            "last_bias_update_host_ts",
            "stationary_tracking_enabled",
            "zupt_enabled",
        ):
            self.assertIn(key, health)
        self.assertTrue(health["stationary_tracking_enabled"])
        self.assertTrue(health["zupt_enabled"])
        self.assertGreater(health["window_accel_std_g"], 0.0)

    def test_bias_adapt_kwargs_are_gone(self):
        """The reader-side adapter was removed; it could never engage."""
        oak = _ZuptFakeOak()
        with self.assertRaises(TypeError):
            OakImuReader(oak, bias_adapt_enabled=True)
        import config as cfg_mod

        self.assertFalse(hasattr(cfg_mod.ImuSteeringConfig, "oak_bias_adapt_enabled"))
        self.assertFalse(hasattr(cfg_mod.ImuSteeringConfig, "oak_bias_adapt_alpha"))


class TestCalibrateGyroLogging(unittest.TestCase):
    def test_logs_the_measured_bias(self):
        rng = random.Random(SEED)
        oak = _ZuptFakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", nmni_enabled=True)
        n = [0]

        def pumping_raw():
            n[0] += 1
            oak.feed([_packet(n[0] * DT, rng, gy_dps=0.15, gyro_noise_dps=0.0,
                              accel_noise_g=0.0)])
            return _ZuptFakeOak.get_imu_raw_gyro_dps(oak)

        from unittest.mock import patch

        with patch.object(oak, "get_imu_raw_gyro_dps", side_effect=pumping_raw):
            with patch("time.sleep", return_value=None):
                with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
                    bias = imu.calibrate_gyro(duration_s=0.05)
        self.assertAlmostEqual(bias[1], 0.15, places=3)
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("gyro bias measured", joined)
        self.assertIn("y=0.1500", joined)

    def test_logs_the_fallback_when_no_samples_arrive(self):
        oak = _ZuptFakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y")
        imu.gyro_bias_dps = (0.11, 0.22, -0.07)
        imu._sync_producer_config()

        from unittest.mock import patch

        with patch.object(imu, "_sample_raw_gyro_dps", return_value=(None, float("inf"))):
            with patch("time.sleep", return_value=None):
                with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
                    bias = imu.calibrate_gyro(duration_s=0.02)
        self.assertEqual(bias, (0.11, 0.22, -0.07))
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("no fresh IMU samples", joined)
        self.assertIn("keeping", joined)


if __name__ == "__main__":
    unittest.main()
