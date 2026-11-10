"""No heading hold from a dead heading (2026-11-06 19:34:37).

The OAK IMU stream stopped with the camera session still up. OakImuReader
froze the heading and reported ``integrate_status`` "stale" on every read for
38.8 h, but ImuSteeringCompensator kept ``is_available`` True and kept
computing a correction from the frozen heading: a turn in progress pinned the
correction at the 35-byte clamp with no fault. While the reader reports the
heading stale the compensator must return no correction, and when the heading
comes back it must re-capture the hold target once (the heading may have
moved while frozen, and a camera session rebuild bumps the yaw generation),
so there is no step. It must not re-apply a target on every tick.
"""

import math
import time
import unittest
import unittest.mock
from dataclasses import replace

from config import ImuSteeringConfig
from config import config as default_config
from pi_app.control.controller import Controller
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.hardware.oak_imu import OakImuReader
from pi_app.hardware.oak_imu_yaw_producer import ImuPacket, ImuYawProducer
from pi_app.tests.test_manual_heading_hold_retarget import (
    TICK_S,
    FakeMotor,
    FakeRelay,
    FakeShutdown,
    _rc,
)

DT = 1.0 / 30.0


class StaleableImu:
    """Scripted reader with OakImuReader's freshness fields."""

    use_mag = False
    calibration_path = None

    def __init__(self, heading_deg: float):
        self.heading_deg = heading_deg
        self.stale = False

    def calibrate_gyro(self, duration_s: float = 3.0):
        return (0.0, 0.0, 0.0)

    def calibrate_mag_hard_iron(self, duration_s: float = 5.0):
        return (0.0, 0.0, 0.0)

    def read(self) -> dict:
        return {
            "heading_deg": self.heading_deg % 360.0,
            "gz_dps": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "integrate_status": "stale" if self.stale else "fresh",
            "sample_age_s": 2.0 if self.stale else 0.01,
        }


def _comp(imu) -> ImuSteeringCompensator:
    return ImuSteeringCompensator(ImuSteeringConfig(calibration_timeout_s=0.1), imu)


class TestCompensatorStaleHeading(unittest.TestCase):
    def test_no_correction_while_the_heading_is_stale(self):
        imu = StaleableImu(0.0)
        comp = _comp(imu)
        imu.heading_deg = 20.0  # 20 deg right of the target: a real correction
        live = comp.update(0.0, DT)
        self.assertIsNotNone(live)
        self.assertLess(live, -5.0)
        imu.stale = True
        for _ in range(90):
            self.assertIsNone(comp.update(0.0, DT))
        status = comp.get_status()
        self.assertTrue(status.heading_stale)
        self.assertEqual(status.integral_error, 0.0)
        self.assertEqual(status.pid_correction, 0.0)
        # The IMU is still "available": reads succeed, the heading is just old.
        self.assertTrue(status.is_available)

    def test_recovery_recaptures_the_target_without_a_step(self):
        imu = StaleableImu(0.0)
        comp = _comp(imu)
        imu.heading_deg = 20.0
        comp.update(0.0, DT)
        imu.stale = True
        for _ in range(30):
            comp.update(0.0, DT)
        # The robot turned while the heading was frozen.
        imu.stale = False
        imu.heading_deg = 75.0
        first = comp.update(0.0, DT)
        self.assertIsNotNone(first)
        self.assertLess(abs(first), 0.5, "step on recovery")
        self.assertAlmostEqual(comp.get_status().target_heading_deg, 75.0)
        self.assertFalse(comp.get_status().heading_stale)
        # The target is captured once, not re-applied each tick: drift 6 deg
        # right and the hold pulls left.
        imu.heading_deg = 81.0
        self.assertLess(comp.update(0.0, DT), -3.0)
        self.assertAlmostEqual(comp.get_status().target_heading_deg, 75.0)

    def test_recovery_seen_first_by_get_heading_deg(self):
        """The controller reads the heading for telemetry every tick; the
        recapture must not depend on which call sees the fresh sample."""
        imu = StaleableImu(10.0)
        comp = _comp(imu)
        imu.stale = True
        comp.get_heading_deg()
        imu.stale = False
        imu.heading_deg = 50.0
        comp.get_heading_deg()
        self.assertAlmostEqual(comp.get_status().target_heading_deg, 50.0)
        self.assertLess(abs(comp.update(0.0, DT)), 0.5)

    def test_a_fresh_heading_never_recaptures(self):
        imu = StaleableImu(0.0)
        comp = _comp(imu)
        for h in (2.0, 4.0, 6.0, 8.0):
            imu.heading_deg = h
            comp.update(0.0, DT)
            comp.get_heading_deg()
        self.assertAlmostEqual(comp.get_status().target_heading_deg, 0.0)

    def test_reader_without_freshness_fields_is_unchanged(self):
        class PlainImu(StaleableImu):
            def read(self):
                d = super().read()
                del d["integrate_status"]
                del d["sample_age_s"]
                return d

        imu = PlainImu(0.0)
        comp = _comp(imu)
        imu.heading_deg = 20.0
        self.assertLess(comp.update(0.0, DT), -5.0)
        self.assertFalse(comp.get_status().heading_stale)

    def test_infinite_sample_age_is_stale(self):
        class NeverSampled(StaleableImu):
            def read(self):
                d = super().read()
                d["integrate_status"] = "init"
                d["sample_age_s"] = float("inf")
                return d

        comp = _comp(NeverSampled(0.0))
        self.assertIsNone(comp.update(0.0, DT))
        self.assertTrue(comp.get_status().heading_stale)

    def test_stale_agrees_with_the_waypoint_heading_guard(self):
        """The guard (controller._waypoint_heading_problem) reads
        heading_age_s; whenever the compensator sees a stale heading the
        guard's age is past waypoint_nav.heading_stale_s too."""
        imu = StaleableImu(0.0)
        comp = _comp(imu)
        imu.stale = True
        comp.update(0.0, DT)
        limit = default_config.waypoint_nav.heading_stale_s
        self.assertTrue(comp.get_status().heading_stale)
        self.assertGreater(comp.heading_age_s(), limit)


class _FakeOak:
    """OakDepthReader stand-in around the real producer; the test owns time."""

    def __init__(self):
        self.producer = ImuYawProducer()
        self.snap = None
        self.dev = 1.0
        self.now = 1000.0
        self.watchdog = None

    def set_imu_watchdog_enabled(self, enabled):
        self.watchdog = enabled

    def feed(self, turn_dps, n=3):
        packets = []
        for _ in range(n):
            self.dev += 0.01
            packets.append(ImuPacket(
                device_ts_s=self.dev, host_ts_s=self.now, gx_rads=0.0,
                gy_rads=math.radians(turn_dps), gz_rads=0.0,
                ax_mss=0.0, ay_mss=-9.7, az_mss=0.4,
            ))
        self.snap = self.producer.ingest(packets, host_now_s=self.now)

    def restart_session(self):
        self.producer.note_pipeline_restart()
        self.dev = 1.0  # the device clock restarts with the session

    def get_imu_data(self):
        sn = self.snap

        class S:
            pass

        st = S()
        for k in ("ax_mss", "ay_mss", "az_mss", "gx_rads", "gy_rads", "gz_rads",
                  "timestamp", "device_timestamp_s", "cum_yaw_x_rad", "cum_yaw_y_rad",
                  "cum_yaw_z_rad", "cum_yaw_grav_rad", "stationary", "zupt_active",
                  "bias_gx_dps", "bias_gy_dps", "bias_gz_dps"):
            setattr(st, k, getattr(sn, k, 0.0))
        st.yaw_generation = sn.generation
        st.producer_packets_integrated = sn.packets_integrated
        return st, self.now - sn.timestamp


class TestDeadQueueWithTheRealReader(unittest.TestCase):
    """The incident harness: real producer, real OakImuReader, real
    compensator. The producer stops mid-turn, as on 2026-11-06."""

    def test_dead_queue_mid_turn_gives_no_correction_and_no_step(self):
        oak = _FakeOak()
        oak.feed(0.0)
        reader = OakImuReader(oak)
        self.assertTrue(oak.watchdog, "OakImuReader must enable the IMU watchdog")
        comp = _comp(reader)
        comp.state.is_available = True

        def step(turn, live):
            oak.now += DT
            if live:
                oak.feed(turn)
            return comp.update(0.0, DT)

        # A right turn in progress under neutral sticks: the hold pulls left.
        for _ in range(40):
            live_corr = step(30.0, True)
        self.assertLess(live_corr, -10.0)
        frozen = reader.read()["heading_deg"]
        # The queue goes silent mid-turn. Before the fix the correction
        # stayed pinned at -35 (the clamp) for as long as the queue was dead.
        dead = [step(30.0, False) for _ in range(60)]
        self.assertTrue(all(c is None for c in dead[20:]), dead)
        self.assertEqual(reader.read()["integrate_status"], "stale")
        self.assertAlmostEqual(reader.read()["heading_deg"], frozen, delta=1.0)
        # Session rebuild: generation bump, device clock restart, and the
        # robot stopped turning.
        oak.restart_session()
        after = [step(0.0, True) for _ in range(10)]
        self.assertTrue(all(c is not None for c in after[1:]), after)
        self.assertLess(max(abs(c) for c in after[1:]), 1.0, after)
        self.assertFalse(comp.get_status().heading_stale)


class TestManualHoldAcrossAStaleHeading(unittest.TestCase):
    """The real Controller in MANUAL, straight sticks, scripted heading."""

    def test_manual_hold_stops_while_stale_and_recaptures_once(self):
        clock = {"t": 5_000.0}
        imu = StaleableImu(284.0)
        comp = _comp(imu)
        cfg = replace(default_config, rc_map=replace(default_config.rc_map, stick_expo=0.0))
        with unittest.mock.patch("pi_app.control.controller.time.monotonic", lambda: clock["t"]), \
                unittest.mock.patch("pi_app.control.controller.config", cfg):
            ctrl = Controller(
                motor_driver=FakeMotor(),
                arm_relay=FakeRelay(),
                shutdown_scheduler=FakeShutdown(),
                imu_compensator=comp,
            )
            ctrl._safety_state.is_armed = True

            def tick():
                clock["t"] += TICK_S
                _, _, telem = ctrl.process(
                    replace(_rc(1800, 1800), last_update_mono_s=time.monotonic())
                )
                return telem.get("imu_correction_applied") or 0.0

            for _ in range(30):
                tick()
            # Drift 8 deg right: the hold pulls left.
            imu.heading_deg = 292.0
            alive = [tick() for _ in range(5)]
            self.assertLess(min(alive), -3.0)
            # The heading freezes at 292 (stale).
            imu.stale = True
            stale = [tick() for _ in range(60)]
            self.assertTrue(all(c == 0.0 for c in stale[2:]), stale)
            self.assertTrue(ctrl.get_imu_status()["heading_stale"])
            # Back at 320 deg: no step back to 284 or 292.
            imu.stale = False
            imu.heading_deg = 320.0
            back = [tick() for _ in range(15)]
            self.assertLess(max(abs(c) for c in back), 1.0, back)
            self.assertAlmostEqual(comp.get_status().target_heading_deg, 320.0, delta=0.5)
            # Still alive, and the target is not re-applied every tick.
            imu.heading_deg = 326.0
            alive = [tick() for _ in range(3)]
            self.assertLess(min(alive), -3.0)
            self.assertFalse(ctrl.get_imu_status()["heading_stale"])


if __name__ == "__main__":
    unittest.main()
