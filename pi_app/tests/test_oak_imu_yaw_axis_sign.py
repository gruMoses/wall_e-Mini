"""Derive the yaw-axis sign from gravity at calibration (2026-09-19).

The OAK pipeline pins the yaw channel to gyro_y and assumes the BMI270 +Y
axis points DOWN, which makes +gy a clockwise turn (see OakImuReader.read's
canonical convention). That is true for the current, validated mount — the
accelerometer reads about -1 g on Y at rest (a run log measured roll 1.7 deg,
pitch 1.9 deg, i.e. close to flat) — but it is a MOUNTING fact, not a software
one. If the camera is ever re-mounted upside down, +Y points UP, +gy silently
becomes counter-clockwise, and the heading inverts again exactly the way it
did before 2026-09-19 (see docs/heading_tuning.md's field-evidence note on
OakImuReader.read). Gravity answers the question at boot instead of assuming
it: ``calibrate_gyro`` now also averages the raw accelerometer and derives
``yaw_axis_sign`` from it.
"""

from __future__ import annotations

import math
import random
import unittest
from typing import List, Optional
from unittest.mock import patch

from pi_app.hardware.oak_imu import OakImuReader, G_MSS
from pi_app.hardware.oak_imu_yaw_producer import ImuPacket, ImuYawProducer

DT = 0.01
SEED = 20260919


def _packet(
    ts: float,
    rng: random.Random,
    *,
    gy_dps: float = 0.0,
    gx_dps: float = 0.0,
    gz_dps: float = 0.0,
    ax_g: float = 0.0,
    ay_g: float = -1.0,
    az_g: float = 0.0,
    gyro_noise_dps: float = 0.0,
    accel_noise_g: float = 0.0,
) -> ImuPacket:
    """One sample with an explicitly controllable gravity vector (ax/ay/az
    in g) — unlike the shared stationary-bias-test ``_packet``, which always
    puts gravity on -Y, this file needs to simulate an inverted or tilted
    mount too."""
    return ImuPacket(
        device_ts_s=ts,
        host_ts_s=ts,
        gx_rads=math.radians(gx_dps + rng.gauss(0.0, gyro_noise_dps)),
        gy_rads=math.radians(gy_dps + rng.gauss(0.0, gyro_noise_dps)),
        gz_rads=math.radians(gz_dps + rng.gauss(0.0, gyro_noise_dps)),
        ax_mss=(ax_g + rng.gauss(0.0, accel_noise_g)) * G_MSS,
        ay_mss=(ay_g + rng.gauss(0.0, accel_noise_g)) * G_MSS,
        az_mss=(az_g + rng.gauss(0.0, accel_noise_g)) * G_MSS,
    )


class _FakeOak:
    """Producer-backed OakDepthReader stub (mirrors oak_depth.py's contract)."""

    def __init__(self) -> None:
        self.producer = ImuYawProducer()
        self.age_s = 0.01

    def get_imu_data(self):
        snap = self.producer.snapshot()
        state = type(
            "_S",
            (),
            dict(
                ax_mss=snap.ax_mss, ay_mss=snap.ay_mss, az_mss=snap.az_mss,
                gx_rads=snap.gx_rads, gy_rads=snap.gy_rads, gz_rads=snap.gz_rads,
                timestamp=snap.timestamp, device_timestamp_s=snap.device_timestamp_s,
                cum_yaw_x_rad=snap.cum_yaw_x_rad, cum_yaw_y_rad=snap.cum_yaw_y_rad,
                cum_yaw_z_rad=snap.cum_yaw_z_rad, cum_yaw_grav_rad=snap.cum_yaw_grav_rad,
                yaw_generation=snap.generation,
                producer_packets_integrated=snap.packets_integrated,
                stationary=snap.stationary, zupt_active=snap.zupt_active,
                bias_gx_dps=snap.bias_gx_dps, bias_gy_dps=snap.bias_gy_dps,
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
        return {}

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

    def feed(self, packets: List[ImuPacket]) -> None:
        self.producer.ingest(packets)


def _calibrate_with_fixed_gravity(
    imu: OakImuReader,
    oak: _FakeOak,
    rng: random.Random,
    *,
    ax_g: float = 0.0,
    ay_g: float = -1.0,
    az_g: float = 0.0,
    duration_s: float = 0.05,
) -> tuple:
    """Run calibrate_gyro() while pumping in fresh samples with a FIXED
    gravity vector, the same pattern test_oak_imu_stationary_bias.py uses for
    gyro-only bias collection (patch get_imu_raw_gyro_dps with a side_effect
    that feeds the producer). Feeding also updates the producer's latest raw
    ax/ay/az_mss, which _sample_raw_accel_g reads via get_imu_data()."""
    n = [0]

    def pumping_raw():
        n[0] += 1
        oak.feed([_packet(n[0] * DT, rng, ax_g=ax_g, ay_g=ay_g, az_g=az_g)])
        return _FakeOak.get_imu_raw_gyro_dps(oak)

    with patch.object(oak, "get_imu_raw_gyro_dps", side_effect=pumping_raw):
        with patch("time.sleep", return_value=None):
            return imu.calibrate_gyro(duration_s=duration_s)


class TestYawAxisSignFromGravity(unittest.TestCase):
    def test_y_down_gives_positive_sign_and_cw_turn_increases_heading(self):
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0)
        with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
            _calibrate_with_fixed_gravity(imu, oak, rng, ay_g=-1.0)
        self.assertAlmostEqual(imu.yaw_axis_sign, 1.0)
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("+Y points DOWN", joined)
        self.assertIn("yaw_axis_sign=+1", joined)

        # A CW turn (+gy raw, this mount) must increase heading.
        oak.feed([_packet(1000 * DT, rng, ay_g=-1.0, gy_dps=0.0)])
        h0 = imu.read()["heading_deg"]
        batch = [_packet((1000 + i) * DT, rng, ay_g=-1.0, gy_dps=30.0) for i in range(1, 101)]
        oak.feed(batch)
        d = imu.read()
        self.assertAlmostEqual(d["heading_deg"], (h0 + 30.0) % 360.0, delta=0.5)
        self.assertGreater(d["gz_dps"], 0.0)

    def test_y_up_gives_negative_sign_and_physical_cw_still_increases_heading(self):
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0)
        with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
            _calibrate_with_fixed_gravity(imu, oak, rng, ay_g=1.0)
        self.assertAlmostEqual(imu.yaw_axis_sign, -1.0)
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("+Y points UP", joined)
        self.assertIn("mounted inverted", joined)
        self.assertIn("yaw_axis_sign=-1", joined)

        # A PHYSICAL CW turn on this (inverted) mount reads -gy raw. The
        # corrected heading must still INCREASE and gz_dps must still be
        # positive — the whole point of the sign correction.
        oak.feed([_packet(1000 * DT, rng, ay_g=1.0, gy_dps=0.0)])
        h0 = imu.read()["heading_deg"]
        batch = [_packet((1000 + i) * DT, rng, ay_g=1.0, gy_dps=-30.0) for i in range(1, 101)]
        oak.feed(batch)
        d = imu.read()
        self.assertAlmostEqual(d["heading_deg"], (h0 + 30.0) % 360.0, delta=0.5)
        self.assertGreater(d["gz_dps"], 0.0)

    def test_y_not_vertical_keeps_sign_positive_and_warns(self):
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0)
        with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
            _calibrate_with_fixed_gravity(imu, oak, rng, ax_g=-1.0, ay_g=0.0, az_g=0.0)
        self.assertAlmostEqual(imu.yaw_axis_sign, 1.0)
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("not vertical", joined)
        self.assertIn("gravity_projected", joined)

    def test_accel_absent_keeps_sign_unchanged_and_warns(self):
        oak = _FakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0)
        imu.yaw_axis_sign = -1.0  # simulate a prior calibration's result

        with patch.object(imu, "_sample_raw_gyro_dps", return_value=(None, float("inf"))):
            with patch.object(imu, "_sample_raw_accel_g", return_value=(None, float("inf"))):
                with patch("time.sleep", return_value=None):
                    with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
                        imu.calibrate_gyro(duration_s=0.02)
        self.assertAlmostEqual(imu.yaw_axis_sign, -1.0)  # unchanged
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("no accelerometer data", joined)

    def test_auto_disabled_keeps_sign_positive_even_when_mount_looks_inverted(self):
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(
            oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0,
            yaw_axis_sign_auto=False,
        )
        with self.assertLogs("pi_app.hardware.oak_imu", "WARNING") as cm:
            _calibrate_with_fixed_gravity(imu, oak, rng, ay_g=1.0)  # looks inverted
        self.assertAlmostEqual(imu.yaw_axis_sign, 1.0)  # never flipped
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("+Y points UP", joined)  # the check still ran / logged
        self.assertIn("check-only", joined)

    def test_rest_then_turn_through_producer_path_with_inverted_sign(self):
        """End-to-end: yaw_axis_sign=-1 flows through _select_cum's cum-delta
        path (not just the instantaneous rate) with correct magnitude AND
        sign for a real rest-then-turn sequence."""
        rng = random.Random(SEED)
        oak = _FakeOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0, nmni_enabled=False)
        imu.yaw_axis_sign = -1.0

        oak.feed([_packet(0.0, rng, gy_dps=0.0)])
        h0 = imu.read()["heading_deg"]

        # Physical CW 30 dps for 1.0 s -> raw gy = -30 dps on this (inverted) mount.
        batch = [_packet(0.01 * i, rng, gy_dps=-30.0) for i in range(1, 101)]
        oak.feed(batch)
        d = imu.read()
        self.assertAlmostEqual(d["heading_deg"], (h0 + 30.0) % 360.0, delta=0.5)
        self.assertGreater(d["gz_dps"], 0.0)

        health = imu.get_health()
        self.assertEqual(health["yaw_axis_sign"], -1.0)
        self.assertEqual(health["yaw_rate_sign"], -1.0)


if __name__ == "__main__":
    unittest.main()
