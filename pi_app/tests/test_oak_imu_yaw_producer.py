"""Focused tests for lossless OAK IMU producer yaw integration."""

from __future__ import annotations

import math
import unittest
from dataclasses import dataclass
from typing import List, Optional
from unittest.mock import patch

from pi_app.hardware.oak_imu import OakImuReader, G_MSS
from pi_app.hardware.oak_imu_yaw_producer import (
    ImuPacket,
    ImuYawProducer,
    sparse_snapshot_integrate_yaw_deg,
)


def _pkt(
    ts: float,
    gy_dps: float = 0.0,
    gx_dps: float = 0.0,
    gz_dps: float = 0.0,
    host_ts: Optional[float] = None,
) -> ImuPacket:
    return ImuPacket(
        device_ts_s=ts,
        host_ts_s=host_ts if host_ts is not None else ts,
        gx_rads=math.radians(gx_dps),
        gy_rads=math.radians(gy_dps),
        gz_rads=math.radians(gz_dps),
        ax_mss=0.0,
        ay_mss=-G_MSS,
        az_mss=0.0,
    )


class TestSparseUndercountReproduction(unittest.TestCase):
    def test_bursty_batches_undercount_on_sparse_path(self):
        """Old path: only last packet of each drain batch → under-report ~90%."""
        # 100 Hz for 1.0 s at +90 dps about Y → true free yaw = 90°.
        full = [(i * 0.01, 90.0) for i in range(101)]  # 0.00 .. 1.00
        true_yaw = sparse_snapshot_integrate_yaw_deg(full)
        self.assertAlmostEqual(true_yaw, 90.0, places=4)

        # Simulate 10 drains of 10 packets each; consumer only sees last of batch.
        sparse = []
        for batch_i in range(10):
            # batches cover [0.01..0.10], [0.11..0.20], ...
            last_ts = (batch_i + 1) * 0.10
            sparse.append((last_ts, 90.0))
        # Plus seed at t=0
        sparse = [(0.0, 90.0)] + sparse
        under = sparse_snapshot_integrate_yaw_deg(sparse)
        # 10 intervals of 0.10 s → 90° if no gap freeze; 0.10 < 0.15 so full 90.
        # To reproduce undercount with gap freeze, use larger inter-batch gaps.
        self.assertAlmostEqual(under, 90.0, places=2)

        # Realistic: batches arrive late so sparse snapshots are >0.15 s apart.
        # Only newest kept every 0.20 s → gap_freeze loses most rotation.
        sparse_gappy = [(0.0, 90.0)] + [(0.20 * (i + 1), 90.0) for i in range(5)]
        under_gappy = sparse_snapshot_integrate_yaw_deg(sparse_gappy, max_dt_s=0.15)
        self.assertEqual(under_gappy, 0.0)  # every step gap-frozen

    def test_producer_integrates_full_batch_under_delayed_consumer(self):
        prod = ImuYawProducer(max_integrate_dt_s=0.15)
        # One drain with 100 packets covering 1.0 s at 90 dps.
        batch = [_pkt(i * 0.01, gy_dps=90.0) for i in range(101)]
        snap = prod.ingest(batch)
        # First packet restarts; remaining 100 integrate at 0.01 s → 90°.
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_y_rad), 90.0, places=3)
        self.assertEqual(snap.packets_integrated, 100)
        self.assertEqual(snap.packets_gap_freeze, 0)


class TestImuYawProducer(unittest.TestCase):
    def test_out_of_order_packets_sorted_by_device_ts(self):
        prod = ImuYawProducer()
        # Deliver reverse order; sort should restore integration order.
        batch = [_pkt(0.03, gy_dps=50.0), _pkt(0.01, gy_dps=50.0), _pkt(0.02, gy_dps=50.0)]
        snap = prod.ingest(batch)
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_y_rad), 1.0, places=4)  # 50*0.02
        self.assertEqual(snap.packets_integrated, 2)

    def test_duplicates_and_regressed_timestamps(self):
        prod = ImuYawProducer()
        prod.ingest([_pkt(1.00, gy_dps=30.0), _pkt(1.01, gy_dps=30.0)])
        y1 = prod.cum_y_rad
        snap = prod.ingest([_pkt(1.01, gy_dps=30.0)])  # duplicate
        self.assertEqual(snap.packets_duplicate, 1)
        self.assertAlmostEqual(snap.cum_yaw_y_rad, y1, places=9)

        gen_before = snap.generation
        snap2 = prod.ingest([_pkt(0.05, gy_dps=200.0)])  # regressed
        self.assertGreater(snap2.packets_regressed, 0)
        self.assertGreater(snap2.generation, gen_before)
        self.assertAlmostEqual(snap2.cum_yaw_y_rad, y1, places=9)  # no jump

        snap3 = prod.ingest([_pkt(0.06, gy_dps=0.0)])
        self.assertEqual(snap3.last_status, "fresh")
        self.assertAlmostEqual(snap3.cum_yaw_y_rad, y1, places=9)

    def test_gap_freeze_no_huge_step(self):
        prod = ImuYawProducer(max_integrate_dt_s=0.15)
        prod.ingest([_pkt(5.0, gy_dps=10.0), _pkt(5.05, gy_dps=10.0)])
        y1 = prod.cum_y_rad
        snap = prod.ingest([_pkt(6.05, gy_dps=100.0)])
        self.assertEqual(snap.last_status, "gap_freeze")
        self.assertAlmostEqual(snap.cum_yaw_y_rad, y1, places=9)

    def test_backlog_cap_drops_oldest(self):
        prod = ImuYawProducer(max_packets_per_drain=5)
        # 10 packets; only newest 5 processed after sort.
        batch = [_pkt(i * 0.01, gy_dps=90.0) for i in range(10)]
        snap = prod.ingest(batch)
        self.assertEqual(snap.packets_backlog_dropped, 5)
        # Among 5 kept: 1 restart + 4 integrated → 90*0.04 = 3.6°
        self.assertEqual(snap.packets_integrated, 4)
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_y_rad), 3.6, places=3)

    def test_full_configured_backlog_integrates_without_drop(self):
        """Default cap aligns with host queue (512): full backlog must not drop."""
        from pi_app.hardware.oak_depth import (
            IMU_HOST_QUEUE_MAX_SIZE,
            IMU_MAX_PACKETS_PER_DRAIN,
        )

        self.assertEqual(IMU_MAX_PACKETS_PER_DRAIN, IMU_HOST_QUEUE_MAX_SIZE)
        self.assertEqual(IMU_MAX_PACKETS_PER_DRAIN, 512)

        prod = ImuYawProducer(max_packets_per_drain=IMU_MAX_PACKETS_PER_DRAIN)
        # 512 packets at 100 Hz / 90 dps: first restarts, 511 integrate.
        n = IMU_MAX_PACKETS_PER_DRAIN
        batch = [_pkt(i * 0.01, gy_dps=90.0) for i in range(n)]
        snap = prod.ingest(batch)
        self.assertEqual(snap.packets_backlog_dropped, 0)
        self.assertEqual(snap.packets_received, n)
        self.assertEqual(snap.packets_integrated, n - 1)
        expected_deg = 90.0 * 0.01 * (n - 1)
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_y_rad), expected_deg, places=2)

    def test_default_cap_matches_host_message_buffer(self):
        from pi_app.hardware.oak_depth import IMU_MAX_PACKETS_PER_DRAIN
        from pi_app.hardware.oak_imu_yaw_producer import _DEFAULT_MAX_PACKETS_PER_DRAIN

        self.assertEqual(_DEFAULT_MAX_PACKETS_PER_DRAIN, IMU_MAX_PACKETS_PER_DRAIN)
        self.assertEqual(ImuYawProducer().max_packets_per_drain, IMU_MAX_PACKETS_PER_DRAIN)

    def test_bias_applied_once_on_producer(self):
        prod = ImuYawProducer()
        prod.set_gyro_bias_dps(0.0, 10.0, 0.0)  # 10 dps bias on Y
        batch = [_pkt(i * 0.01, gy_dps=10.0) for i in range(11)]  # raw = bias
        snap = prod.ingest(batch)
        self.assertAlmostEqual(snap.cum_yaw_y_rad, 0.0, places=6)

    def test_independent_axes_no_silent_mix(self):
        prod = ImuYawProducer()
        batch = [_pkt(i * 0.01, gx_dps=90.0, gy_dps=0.0) for i in range(11)]
        snap = prod.ingest(batch)
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_x_rad), 9.0, places=3)
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_y_rad), 0.0, places=6)
        self.assertAlmostEqual(math.degrees(snap.cum_yaw_z_rad), 0.0, places=6)

    def test_pipeline_restart_reseeds_without_cum_jump(self):
        prod = ImuYawProducer()
        prod.ingest([_pkt(1.0, gy_dps=45.0), _pkt(1.02, gy_dps=45.0)])
        y1 = prod.cum_y_rad
        gen1 = prod.generation
        prod.note_pipeline_restart()
        self.assertGreater(prod.generation, gen1)
        snap = prod.ingest([_pkt(0.01, gy_dps=90.0), _pkt(0.02, gy_dps=90.0)])
        # After restart: first restarts, second integrates 90*0.01
        self.assertAlmostEqual(
            math.degrees(snap.cum_yaw_y_rad),
            math.degrees(y1) + 0.9,
            places=3,
        )


@dataclass
class FakeImuState:
    ax_mss: float = 0.0
    ay_mss: float = -G_MSS
    az_mss: float = 0.0
    gx_rads: float = 0.0
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    timestamp: float = 0.0
    device_timestamp_s: float = 0.0
    cum_yaw_x_rad: float = 0.0
    cum_yaw_y_rad: float = 0.0
    cum_yaw_z_rad: float = 0.0
    cum_yaw_grav_rad: float = 0.0
    yaw_generation: int = 0
    producer_packets_integrated: int = 0
    producer_integrated_time_s: float = 0.0
    last_integrated_device_ts_s: float = 0.0


class ProducerBackedFakeOak:
    """Fake OakDepthReader that runs the real ImuYawProducer."""

    def __init__(self) -> None:
        self.producer = ImuYawProducer()
        self.age_s = 0.01
        self.health = {
            "connected": True,
            "reconnect_count": 0,
            "last_disconnect_ts": 0.0,
            "pipeline_running": True,
        }
        self._bias = (0.0, 0.0, 0.0)

    def set_imu_gyro_bias_dps(self, gx, gy, gz):
        self._bias = (gx, gy, gz)
        self.producer.set_gyro_bias_dps(gx, gy, gz)

    def set_imu_nmni(self, enabled, threshold_dps=0.3):
        self.producer.set_nmni(enabled, threshold_dps)

    def get_imu_raw_gyro_dps(self):
        """Explicit raw contract (mirrors OakDepthReader)."""
        s = self.producer.snapshot()
        return (
            (
                math.degrees(s.gx_rads),
                math.degrees(s.gy_rads),
                math.degrees(s.gz_rads),
            ),
            self.age_s,
        )

    def get_health(self):
        return dict(self.health)

    def get_imu_metrics(self):
        s = self.producer.snapshot()
        from pi_app.hardware.oak_depth import (
            IMU_HOST_QUEUE_MAX_SIZE,
            IMU_MAX_PACKETS_PER_DRAIN,
        )

        return {
            "packets_received": s.packets_received,
            "packets_drained": s.packets_received,
            "packets_parsed": s.packets_received,
            "packets_integrated": s.packets_integrated,
            # Selection coalescing never used on lossless path (structural 0).
            "packets_coalesced": 0,
            "packets_duplicate": s.packets_duplicate,
            "packets_regressed": s.packets_regressed,
            "packets_restart": s.packets_restart,
            "packets_gap_freeze": s.packets_gap_freeze,
            "packets_backlog_dropped": s.packets_backlog_dropped,
            "cadence_avg_s": s.cadence_avg_s,
            "cadence_max_s": s.cadence_max_s,
            "producer_cum_yaw_x_deg": math.degrees(s.cum_yaw_x_rad),
            "producer_cum_yaw_y_deg": math.degrees(s.cum_yaw_y_rad),
            "producer_cum_yaw_z_deg": math.degrees(s.cum_yaw_z_rad),
            "producer_cum_yaw_grav_deg": math.degrees(s.cum_yaw_grav_rad),
            "producer_generation": s.generation,
            "producer_last_status": s.last_status,
            "queue_msgs_received": 0,
            "queue_msgs_consumed": 0,
            "queue_msgs_dropped": 0,
            "queue_msgs_overwrite_observable": False,
            "queue_drain_count": 0,
            "last_batch_packets": s.last_batch_packets,
            "host_queue_max_size": IMU_HOST_QUEUE_MAX_SIZE,
            "host_queue_blocking": False,
            "max_packets_per_drain": IMU_MAX_PACKETS_PER_DRAIN,
            # Drain-batch observability (not host-queue occupancy).
            "drain_batch_high_water_msgs": 0,
            "drain_batch_large_events": 0,
            "drain_batch_full_size_events": 0,
        }

    def get_imu_data(self):
        s = self.producer.snapshot()
        st = FakeImuState(
            ax_mss=s.ax_mss if s.timestamp else -0.0,
            ay_mss=s.ay_mss if s.timestamp else -G_MSS,
            az_mss=s.az_mss,
            gx_rads=s.gx_rads,
            gy_rads=s.gy_rads,
            gz_rads=s.gz_rads,
            timestamp=s.timestamp,
            device_timestamp_s=s.device_timestamp_s,
            cum_yaw_x_rad=s.cum_yaw_x_rad,
            cum_yaw_y_rad=s.cum_yaw_y_rad,
            cum_yaw_z_rad=s.cum_yaw_z_rad,
            cum_yaw_grav_rad=s.cum_yaw_grav_rad,
            yaw_generation=s.generation,
            producer_packets_integrated=s.packets_integrated,
            producer_integrated_time_s=s.integrated_time_s,
            last_integrated_device_ts_s=s.last_integrated_device_ts_s,
        )
        if st.ay_mss == 0.0 and st.timestamp == 0.0:
            st.ay_mss = -G_MSS
        return st, self.age_s

    def push_batch(self, packets: List[ImuPacket], age_s: float = 0.01) -> None:
        self.age_s = age_s
        self.producer.ingest(packets)

    def push(
        self,
        *,
        device_ts: float,
        gy_dps: float = 0.0,
        gx_dps: float = 0.0,
        gz_dps: float = 0.0,
        age_s: float = 0.01,
        host_ts: Optional[float] = None,
    ) -> None:
        self.push_batch(
            [_pkt(device_ts, gy_dps=gy_dps, gx_dps=gx_dps, gz_dps=gz_dps, host_ts=host_ts)],
            age_s=age_s,
        )


class TestOakImuReaderProducerPath(unittest.TestCase):
    def _reader(self, **kwargs):
        oak = ProducerBackedFakeOak()
        defaults = dict(
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=False,
            bias_adapt_enabled=False,
        )
        defaults.update(kwargs)
        return oak, OakImuReader(oak, **defaults)

    def test_accurate_under_delayed_consumer_bursts(self):
        """Consumer reads once per large batch — must still get full 90°."""
        oak, imu = self._reader()
        # Seed
        oak.push(device_ts=0.0, gy_dps=90.0)
        d0 = imu.read()
        self.assertEqual(d0["integrate_status"], "init")
        h0 = d0["heading_deg"]

        # 10 bursts of 10 packets (0.1 s each) at 90 dps → 90° free yaw total.
        t = 0.0
        for _ in range(10):
            batch = []
            for _j in range(10):
                t += 0.01
                batch.append(_pkt(t, gy_dps=90.0))
            oak.push_batch(batch)
            d = imu.read()  # one consumer read per batch
            self.assertEqual(d["integration_path"], "producer")

        # heading_deg = (-yaw) mod 360; +gy free yaw → heading decreases.
        final = imu.read()
        self.assertAlmostEqual(final["heading_deg"], (h0 - 90.0) % 360.0, places=2)
        health = imu.get_health()
        self.assertEqual(health["integration_path"], "producer")
        self.assertGreaterEqual(health["count_producer_packets"], 99)
        self.assertEqual(health.get("producer_packets_coalesced"), 0)

    def test_double_read_does_not_double_integrate(self):
        oak, imu = self._reader()
        oak.push(device_ts=1.0, gy_dps=50.0)
        imu.read()
        oak.push(device_ts=1.02, gy_dps=50.0)
        d1 = imu.read()
        h1 = d1["heading_deg"]
        self.assertEqual(d1["integrate_status"], "fresh")

        d2 = imu.read()  # same cum
        self.assertEqual(d2["integrate_status"], "duplicate")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)
        self.assertEqual(d2["gz_dps"], 0.0)

    def test_scale_applied_once(self):
        oak, imu = self._reader(yaw_rate_scale=0.5)
        oak.push(device_ts=0.0, gy_dps=90.0)
        h0 = imu.read()["heading_deg"]
        batch = [_pkt(0.01 * i, gy_dps=90.0) for i in range(1, 11)]  # 0.1 s
        oak.push_batch(batch)
        d = imu.read()
        # free yaw raw = 9°; scale 0.5 → 4.5° heading change
        self.assertAlmostEqual(d["heading_deg"], (h0 - 4.5) % 360.0, places=3)

    def test_reconnect_generation_no_jump(self):
        oak, imu = self._reader()
        oak.push(device_ts=10.0, gy_dps=30.0)
        imu.read()
        oak.push(device_ts=10.05, gy_dps=30.0)
        h1 = imu.read()["heading_deg"]

        oak.producer.note_pipeline_restart()
        # First sample after clock reseed is a restart seed (no integrate) — cum
        # continuous, so consumer must not discard heading.
        oak.push(device_ts=0.01, gy_dps=200.0)
        d2 = imu.read()
        self.assertIn(d2["integrate_status"], ("generation_reseed", "fresh", "duplicate"))
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)

        oak.push(device_ts=0.03, gy_dps=0.0)
        d3 = imu.read()
        self.assertEqual(d3["integrate_status"], "fresh")
        self.assertAlmostEqual(d3["heading_deg"], h1, places=5)

    def test_generation_jitter_preserves_unread_27deg(self):
        """Independent simulation: one jittered/regressed timestamp must not
        silently drop ~27° of already-valid producer rotation.

        Old consumer discarded all cum delta on any generation change.
        """
        oak, imu = self._reader()
        oak.push(device_ts=0.0, gy_dps=90.0)
        h0 = imu.read()["heading_deg"]

        # 0.30 s @ 90 dps → 27° free yaw accumulated on producer only.
        batch = [_pkt(0.01 * i, gy_dps=90.0) for i in range(1, 31)]
        oak.push_batch(batch)
        y_before = math.degrees(oak.producer.cum_y_rad)
        self.assertAlmostEqual(y_before, 27.0, places=2)

        # Timestamp regression bumps generation without zeroing cum.
        gen_before = oak.producer.generation
        oak.push(device_ts=0.001, gy_dps=200.0)  # regressed
        self.assertGreater(oak.producer.generation, gen_before)
        self.assertAlmostEqual(math.degrees(oak.producer.cum_y_rad), y_before, places=6)

        d = imu.read()
        # Must preserve the full 27° (heading decreases for +gy free yaw).
        self.assertAlmostEqual(d["heading_deg"], (h0 - 27.0) % 360.0, places=2)
        self.assertNotEqual(d["integrate_status"], "regressed")
        self.assertGreaterEqual(imu.get_health()["count_generation_change"], 1)
        self.assertEqual(imu.get_health()["count_cum_reset"], 0)

    def test_plus_10_minus_10_with_generation_preserves_reverse_delta(self):
        """Exact +10° then -10° (cum→0) with generation/timestamp must not reset.

        Old near-zero cum heuristic classified this as cum_reset and lost the
        reverse delta. Counter identity is continuous — apply both halves.
        """
        oak, imu = self._reader()
        oak.push(device_ts=0.0, gy_dps=100.0)
        h0 = imu.read()["heading_deg"]

        # +10° free yaw @ 100 Hz / 100 dps → 10 intervals after seed (0.01..0.10).
        plus = [_pkt(0.01 * i, gy_dps=100.0) for i in range(1, 11)]
        oak.push_batch(plus)
        d_plus = imu.read()
        self.assertAlmostEqual(d_plus["heading_deg"], (h0 - 10.0) % 360.0, places=3)
        y_plus = math.degrees(oak.producer.cum_y_rad)
        self.assertAlmostEqual(y_plus, 10.0, places=3)

        # Generation bump (pipeline reseed) without zeroing cum or rewinding
        # the integrated-packet counter. Cum stays at +10°.
        gen_before = oak.producer.generation
        integrated_before = oak.producer.packets_integrated
        oak.producer.note_pipeline_restart()
        self.assertGreater(oak.producer.generation, gen_before)
        self.assertEqual(oak.producer.packets_integrated, integrated_before)
        self.assertAlmostEqual(math.degrees(oak.producer.cum_y_rad), 10.0, places=3)

        # Also exercise a regressed timestamp seed on the new session clock.
        oak.push(device_ts=5.0, gy_dps=0.0)  # restart seed after reseed
        # -10° reverse turn brings free-yaw channels back to exactly 0.
        minus = [_pkt(5.0 + 0.01 * i, gy_dps=-100.0) for i in range(1, 11)]
        oak.push_batch(minus)
        y_after = math.degrees(oak.producer.cum_y_rad)
        self.assertAlmostEqual(y_after, 0.0, places=3)

        d_minus = imu.read()
        # Must apply the full reverse (heading returns toward h0), not freeze.
        # Old heuristic: prev_mag>5° and cur_mag≈0 → false cum_reset, lost -10°.
        # Compare on the circle (360° ≡ 0°).
        err = abs((d_minus["heading_deg"] - h0 + 180.0) % 360.0 - 180.0)
        self.assertLess(err, 0.05)
        self.assertNotEqual(d_minus["integrate_status"], "cum_reset")
        health = imu.get_health()
        self.assertEqual(health["count_cum_reset"], 0)
        self.assertGreaterEqual(health["count_generation_change"], 1)

    def test_true_counter_rewind_reset_freezes_heading(self):
        """True producer replacement: integrated-packet counter rewind only."""
        oak, imu = self._reader()
        oak.push(device_ts=0.0, gy_dps=90.0)
        imu.read()
        batch = [_pkt(0.01 * i, gy_dps=90.0) for i in range(1, 11)]  # +9°
        oak.push_batch(batch)
        h1 = imu.read()["heading_deg"]
        self.assertGreater(oak.producer.packets_integrated, 0)

        # Simulate integrator replacement: zero cums, rewind integrated, bump gen.
        # Near-zero cum alone is NOT the signal — counter rewind is.
        oak.producer.cum_x_rad = 0.0
        oak.producer.cum_y_rad = 0.0
        oak.producer.cum_z_rad = 0.0
        oak.producer.cum_grav_rad = 0.0
        oak.producer.packets_integrated = 0  # counter rewind = true reset
        oak.producer.generation += 1
        oak.producer.last_device_ts_s = None

        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "cum_reset")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)
        self.assertEqual(imu.get_health()["count_cum_reset"], 1)

        # Subsequent motion after reset still integrates from new baseline.
        oak.push(device_ts=1.0, gy_dps=0.0)
        imu.read()  # seed clocks
        oak.push(device_ts=1.10, gy_dps=90.0)  # 0.1 s @ 90 → 9°
        d3 = imu.read()
        self.assertAlmostEqual(d3["heading_deg"], (h1 - 9.0) % 360.0, places=2)

    def test_near_zero_cum_without_counter_rewind_is_not_reset(self):
        """Cum channels at ~0 with continuous counter must never freeze heading."""
        oak, imu = self._reader()
        oak.push(device_ts=0.0, gy_dps=100.0)
        h0 = imu.read()["heading_deg"]
        # +10° then -10° without any generation bump → net 0 free yaw.
        oak.push_batch([_pkt(0.01 * i, gy_dps=100.0) for i in range(1, 11)])
        imu.read()
        oak.push_batch([_pkt(0.10 + 0.01 * i, gy_dps=-100.0) for i in range(1, 11)])
        d = imu.read()
        self.assertAlmostEqual(math.degrees(oak.producer.cum_y_rad), 0.0, places=3)
        self.assertAlmostEqual(d["heading_deg"], h0 % 360.0, places=3)
        self.assertEqual(imu.get_health()["count_cum_reset"], 0)
        self.assertNotEqual(d["integrate_status"], "cum_reset")

    def test_stale_freezes(self):
        oak, imu = self._reader()
        oak.push(device_ts=3.0, gy_dps=40.0)
        imu.read()
        oak.push(device_ts=3.02, gy_dps=40.0)
        h1 = imu.read()["heading_deg"]

        oak.push(device_ts=3.04, gy_dps=40.0, age_s=1.5)
        d2 = imu.read()
        self.assertEqual(d2["integrate_status"], "stale")
        self.assertAlmostEqual(d2["heading_deg"], h1, places=8)

    def test_axis_modes_do_not_mix(self):
        oak, imu = self._reader(yaw_rate_source="gyro_x")
        oak.push(device_ts=0.0, gx_dps=90.0, gy_dps=90.0)
        h0 = imu.read()["heading_deg"]
        batch = [_pkt(0.01 * i, gx_dps=90.0, gy_dps=90.0) for i in range(1, 11)]
        oak.push_batch(batch)
        d = imu.read()
        # Only X contributes → 9° free, heading -9
        self.assertAlmostEqual(d["heading_deg"], (h0 - 9.0) % 360.0, places=3)
        self.assertEqual(d["yaw_rate_source_selected"], "gyro_x")

    def test_health_exposes_producer_counters(self):
        oak, imu = self._reader()
        oak.push(device_ts=1.0, gy_dps=5.0)
        imu.read()
        oak.push_batch([_pkt(1.0 + 0.01 * i, gy_dps=5.0) for i in range(1, 6)])
        imu.read()
        h = imu.get_health()
        for key in (
            "producer_packets_integrated",
            "producer_packets_received",
            "producer_packets_drained",
            "producer_packets_backlog_dropped",
            "producer_packets_gap_freeze",
            "producer_cum_yaw_y_deg",
            "integration_path",
            "count_producer_packets",
            "count_generation_change",
            "count_cum_reset",
            "host_queue_max_size",
            "drain_batch_high_water_msgs",
            "drain_batch_large_events",
            "drain_batch_full_size_events",
            "queue_msgs_overwrite_observable",
            "max_packets_per_drain",
            "gx_body_dps",
            "gy_body_dps",
            "gz_body_dps",
        ):
            self.assertIn(key, h)
        self.assertEqual(h["integration_path"], "producer")
        # Selection coalescing stays 0 by structure (not a manufactured loss proof).
        self.assertEqual(h.get("producer_packets_coalesced"), 0)
        self.assertGreaterEqual(h.get("host_queue_max_size"), 256)
        self.assertIs(h.get("queue_msgs_overwrite_observable"), False)
        self.assertEqual(h.get("max_packets_per_drain"), h.get("host_queue_max_size"))

    def test_controller_status_embeds_producer_health(self):
        from config import ImuSteeringConfig
        from pi_app.control.imu_steering import ImuSteeringCompensator
        from pi_app.control.controller import Controller

        oak, imu = self._reader()
        oak.push(device_ts=1.0, gy_dps=0.0)
        imu.read()

        class MiniController:
            def __init__(self_inner):
                with patch.object(imu, "calibrate_gyro", return_value=(0.0, 0.0, 0.0)):
                    self_inner._imu_compensator = ImuSteeringCompensator(
                        ImuSteeringConfig(enabled=True, calibration_timeout_s=0.1), imu
                    )

            get_imu_status = Controller.get_imu_status

        st = MiniController().get_imu_status()
        self.assertIsNotNone(st)
        self.assertIn("oak_imu", st)
        self.assertEqual(st["oak_imu"]["integration_path"], "producer")
        self.assertIn("producer_packets_integrated", st["oak_imu"])


class TestHostQueueConfig(unittest.TestCase):
    def test_imu_host_queue_and_producer_cap_aligned(self):
        from pi_app.hardware import oak_depth as od

        # Message-slot budget (not a hard 5.1 s wall-clock guarantee).
        self.assertEqual(od.IMU_HOST_QUEUE_MAX_SIZE, 512)
        self.assertEqual(od.IMU_MAX_PACKETS_PER_DRAIN, od.IMU_HOST_QUEUE_MAX_SIZE)
        self.assertIs(od.IMU_HOST_QUEUE_BLOCKING, False)
        # Structural: pipeline builder uses the module constants (source check).
        with open(od.__file__, encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn("createOutputQueue(", src)
        self.assertIn("maxSize=int(self._imu_host_queue_max_size)", src)
        self.assertIn("blocking=bool(self._imu_host_queue_blocking)", src)
        self.assertIn("IMU host queue must buffer multiple seconds", src)
        # Drain-batch observability — never host-queue occupancy claims.
        self.assertIn("drain_batch_high_water_msgs", src)
        self.assertIn("_note_drain_batch", src)
        self.assertNotIn("host_queue_saturated_events", src)
        self.assertNotIn("host_queue_near_full_events", src)
        # Successfully drained messages must not be counted as dropped.
        self.assertNotIn("queue_msgs_dropped +=", src)


class TestLegacySnapshotPathStillWorks(unittest.TestCase):
    """Stubs without cum fields keep the pre-producer harden path."""

    @dataclass
    class LegacyState:
        ax_mss: float = 0.0
        ay_mss: float = -G_MSS
        az_mss: float = 0.0
        gx_rads: float = 0.0
        gy_rads: float = 0.0
        gz_rads: float = 0.0
        timestamp: float = 0.0
        device_timestamp_s: float = 0.0

    class LegacyOak:
        def __init__(self):
            self.state = TestLegacySnapshotPathStillWorks.LegacyState()
            self.age_s = 0.01
            self.health = {
                "connected": True,
                "reconnect_count": 0,
                "last_disconnect_ts": 0.0,
                "pipeline_running": True,
            }

        def get_imu_data(self):
            return self.state, self.age_s

        def get_health(self):
            return dict(self.health)

        def push(self, *, device_ts, gy_dps=0.0, host_ts=None, age_s=0.01):
            self.age_s = age_s
            self.state = TestLegacySnapshotPathStillWorks.LegacyState(
                ay_mss=-G_MSS,
                gy_rads=math.radians(gy_dps),
                timestamp=float(host_ts if host_ts is not None else device_ts),
                device_timestamp_s=float(device_ts),
            )

    def test_fresh_integration_legacy(self):
        oak = self.LegacyOak()
        imu = OakImuReader(oak, yaw_rate_source="gyro_y", yaw_rate_scale=1.0, nmni_enabled=False)
        oak.push(device_ts=1.0, gy_dps=90.0)
        h0 = imu.read()["heading_deg"]
        oak.push(device_ts=1.05, gy_dps=90.0)
        d1 = imu.read()
        self.assertEqual(d1["integration_path"], "legacy_snapshot")
        self.assertEqual(d1["integrate_status"], "fresh")
        self.assertAlmostEqual(d1["heading_deg"], (h0 - 4.5) % 360.0, places=4)


class CorrectedSnapshotFakeOak(ProducerBackedFakeOak):
    """Footgun stand-in: get_imu_data exposes bias/NMNI-corrected rates.

    Mimics the circular-calibration failure mode: with NMNI on and bias=0,
    sub-threshold residual bias reads as exact zero from get_imu_data.
    Deliberately omits get_imu_raw_gyro_dps so calibrate must either pause
    NMNI or fail.
    """

    def get_imu_data(self):
        st, age = super().get_imu_data()
        bx, by, bz = self._bias
        gx = st.gx_rads - math.radians(float(bx))
        gy = st.gy_rads - math.radians(float(by))
        gz = st.gz_rads - math.radians(float(bz))
        thr = float(self.producer.nmni_threshold_dps)
        if self.producer.nmni_enabled:
            if abs(math.degrees(gx)) < thr:
                gx = 0.0
            if abs(math.degrees(gy)) < thr:
                gy = 0.0
            if abs(math.degrees(gz)) < thr:
                gz = 0.0
        st.gx_rads = gx
        st.gy_rads = gy
        st.gz_rads = gz
        return st, age

    def get_imu_raw_gyro_dps(self):
        # Force calibrate onto the get_imu_data fallback path.
        raise RuntimeError("raw API unavailable in this footgun fake")


class TestGyroCalibrateNmniNoCircular(unittest.TestCase):
    """Residual bias must be measurable with NMNI enabled; CW/CCW stay symmetric."""

    RESIDUAL_Y_DPS = 0.15  # below default NMNI threshold 0.3
    RESIDUAL_X_DPS = 0.08
    RESIDUAL_Z_DPS = -0.05

    def test_snapshot_gx_is_raw_not_nmni_gated(self):
        """Producer contract: latest gx/gy/gz stay raw even when NMNI is on."""
        prod = ImuYawProducer()
        prod.set_nmni(True, 0.3)
        prod.ingest([_pkt(0.0, gy_dps=self.RESIDUAL_Y_DPS)])
        prod.ingest([_pkt(0.01, gy_dps=self.RESIDUAL_Y_DPS)])
        snap = prod.snapshot()
        self.assertAlmostEqual(math.degrees(snap.gy_rads), self.RESIDUAL_Y_DPS, places=6)
        # Cum must NOT integrate sub-threshold residual when NMNI is on.
        self.assertAlmostEqual(snap.cum_yaw_y_rad, 0.0, places=9)

    def test_calibrate_recovers_subthreshold_bias_with_nmni_enabled(self):
        oak = ProducerBackedFakeOak()
        imu = OakImuReader(
            oak,
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=True,
            nmni_threshold_dps=0.3,
            bias_adapt_enabled=False,
        )
        # __init__ already enabled producer NMNI with bias=0.
        self.assertTrue(oak.producer.nmni_enabled)

        n = [0]

        def pumping_raw():
            n[0] += 1
            t = 0.01 * n[0]
            oak.push(
                device_ts=t,
                gx_dps=self.RESIDUAL_X_DPS,
                gy_dps=self.RESIDUAL_Y_DPS,
                gz_dps=self.RESIDUAL_Z_DPS,
            )
            return ProducerBackedFakeOak.get_imu_raw_gyro_dps(oak)

        with patch.object(oak, "get_imu_raw_gyro_dps", side_effect=pumping_raw):
            with patch("time.sleep", return_value=None):
                bias = imu.calibrate_gyro(duration_s=0.05)

        self.assertGreater(n[0], 5)
        self.assertAlmostEqual(bias[0], self.RESIDUAL_X_DPS, places=3)
        self.assertAlmostEqual(bias[1], self.RESIDUAL_Y_DPS, places=3)
        self.assertAlmostEqual(bias[2], self.RESIDUAL_Z_DPS, places=3)
        # NMNI restored after calibrate; bias pushed to producer.
        self.assertTrue(oak.producer.nmni_enabled)
        self.assertAlmostEqual(math.degrees(oak.producer.bias_gy_rads), self.RESIDUAL_Y_DPS, places=3)

    def test_calibrate_not_circular_when_get_imu_data_is_corrected(self):
        """If host snapshot exposes integrate-path rates, cal must still recover bias."""
        oak = CorrectedSnapshotFakeOak()
        imu = OakImuReader(
            oak,
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=True,
            nmni_threshold_dps=0.3,
            bias_adapt_enabled=False,
        )
        self.assertTrue(oak.producer.nmni_enabled)

        # Without the pause-NMNI guard, corrected get_imu_data would read 0.
        oak.push(device_ts=0.01, gy_dps=self.RESIDUAL_Y_DPS)
        st1, _ = CorrectedSnapshotFakeOak.get_imu_data(oak)
        self.assertAlmostEqual(math.degrees(st1.gy_rads), 0.0, places=6)

        n = [0]

        def pumping_get():
            n[0] += 1
            t = 0.01 * n[0]
            oak.push(
                device_ts=t,
                gx_dps=self.RESIDUAL_X_DPS,
                gy_dps=self.RESIDUAL_Y_DPS,
                gz_dps=self.RESIDUAL_Z_DPS,
            )
            # Unbound call — avoid recursion through the patch.
            return CorrectedSnapshotFakeOak.get_imu_data(oak)

        with patch.object(oak, "get_imu_data", side_effect=pumping_get):
            with patch("time.sleep", return_value=None):
                bias = imu.calibrate_gyro(duration_s=0.05)

        self.assertAlmostEqual(bias[0], self.RESIDUAL_X_DPS, places=3)
        self.assertAlmostEqual(bias[1], self.RESIDUAL_Y_DPS, places=3)
        self.assertAlmostEqual(bias[2], self.RESIDUAL_Z_DPS, places=3)
        self.assertTrue(oak.producer.nmni_enabled)

    def test_calibrate_restores_prior_bias_on_sampling_exception(self):
        """A mid-window sampling throw must not leave the transient bias=0.

        calibrate zeroes producer bias + turns NMNI off to open the sampling
        window. If sampling raises before a mean is computed, the reader and
        producer must both restore the *prior* bias (and NMNI on) — the caller
        (imu_steering) swallows the exception and does not retry, so a zeroed
        bias would silently reintroduce the residual-integrate skew.
        """
        oak = ProducerBackedFakeOak()
        imu = OakImuReader(
            oak,
            yaw_rate_source="gyro_y",
            yaw_rate_scale=1.0,
            nmni_enabled=True,
            nmni_threshold_dps=0.3,
            bias_adapt_enabled=False,
        )
        prior = (0.11, 0.22, -0.07)
        imu.gyro_bias_dps = prior
        imu._sync_producer_config()
        self.assertTrue(oak.producer.nmni_enabled)

        def boom():
            raise RuntimeError("sampling blew up mid-window")

        with patch.object(imu, "_sample_raw_gyro_dps", side_effect=boom):
            with patch("time.sleep", return_value=None):
                with self.assertRaises(RuntimeError):
                    imu.calibrate_gyro(duration_s=0.05)

        # Prior bias retained (never the transient zero); NMNI restored on.
        self.assertEqual(imu.gyro_bias_dps, prior)
        self.assertTrue(oak.producer.nmni_enabled)
        self.assertAlmostEqual(math.degrees(oak.producer.bias_gx_rads), prior[0], places=6)
        self.assertAlmostEqual(math.degrees(oak.producer.bias_gy_rads), prior[1], places=6)
        self.assertAlmostEqual(math.degrees(oak.producer.bias_gz_rads), prior[2], places=6)

    def test_cw_ccw_symmetric_after_bias_cal_with_nmni(self):
        """Uncorrected residual skews CW vs CCW; after cal both |Δ| match at scale=1."""
        residual = self.RESIDUAL_Y_DPS
        rate = 30.0  # dps turn rate (well above NMNI)
        duration_s = 3.0  # 90° of true rotation
        n_steps = int(duration_s / 0.01)

        def production_free_yaw_deg(apply_cal: bool, sign: float) -> float:
            """Production free-yaw = -yaw_rad_deg (matches chalk report sign)."""
            oak = ProducerBackedFakeOak()
            imu = OakImuReader(
                oak,
                yaw_rate_source="gyro_y",
                yaw_rate_scale=1.0,
                nmni_enabled=True,
                nmni_threshold_dps=0.3,
                bias_adapt_enabled=False,
            )
            if apply_cal:
                n = [0]

                def pumping_raw():
                    n[0] += 1
                    oak.push(
                        device_ts=0.01 * n[0],
                        gy_dps=residual,
                        gx_dps=0.0,
                        gz_dps=0.0,
                    )
                    return ProducerBackedFakeOak.get_imu_raw_gyro_dps(oak)

                with patch.object(oak, "get_imu_raw_gyro_dps", side_effect=pumping_raw):
                    with patch("time.sleep", return_value=None):
                        imu.calibrate_gyro(duration_s=0.05)
            # Seed consumer, then turn: raw = sign*rate + residual
            t0 = 10.0
            oak.push(device_ts=t0, gy_dps=sign * rate + residual)
            imu.read()
            batch = [
                _pkt(t0 + 0.01 * i, gy_dps=sign * rate + residual)
                for i in range(1, n_steps + 1)
            ]
            oak.push_batch(batch)
            imu.read()
            return -math.degrees(imu.yaw_rad)

        # sign=+1 → positive body gy (field CW); production free-yaw negative.
        cw_uncal = production_free_yaw_deg(False, +1.0)
        ccw_uncal = production_free_yaw_deg(False, -1.0)
        self.assertGreater(abs(abs(cw_uncal) - abs(ccw_uncal)), 0.3)

        cw = production_free_yaw_deg(True, +1.0)
        ccw = production_free_yaw_deg(True, -1.0)
        self.assertAlmostEqual(abs(cw), 90.0, places=1)
        self.assertAlmostEqual(abs(ccw), 90.0, places=1)
        self.assertAlmostEqual(abs(cw), abs(ccw), places=2)
        self.assertLess(cw, 0.0)
        self.assertGreater(ccw, 0.0)


if __name__ == "__main__":
    unittest.main()
