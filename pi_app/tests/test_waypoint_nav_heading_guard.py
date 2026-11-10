"""WAYPOINT_NAV protects its heading reference (2026-11-10 status audit, item 3).

(a) An OAK reconnect during a run makes ``_check_imu_frame_continuity`` reset
    the GPS heading offset to 0. The run used to continue in the raw gyro
    frame, because the lock was checked only at activation.
(b) ``waypoint_nav_activation_blocked`` skipped the fix-quality check and the
    lock check when no GPS reading existed yet.
(c) ALIGN had no timeout, and nothing checked that the heading was fresh.

Now a run that loses its heading reference leaves WAYPOINT_NAV before the
tick's command is computed (the same exit as completion / stop: MANUAL, the
sticks are the target, the slew limiter ramps at the MANUAL rate), activation
is refused when the GPS or heading state is unknown, and ALIGN gives up after
``WaypointNavConfig.align_timeout_s``.
"""

import math
import time
import unittest
from contextlib import contextmanager
from dataclasses import replace
from unittest.mock import MagicMock, patch

from config import GpsHeadingAlignConfig, config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.gps_heading_align import GpsHeadingAligner
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.waypoint_nav import Waypoint, WaypointNavConfig, WaypointNavController
from pi_app.hardware.rtk_gps import GpsReading

DT = 1.0 / 30.0
ALIGN_TIMEOUT_S = float(default_config.waypoint_nav.align_timeout_s)
HEADING_STALE_S = float(default_config.waypoint_nav.heading_stale_s)


class FakeMotor:
    def __init__(self):
        self.commands = []

    def set_tracks(self, left_byte, right_byte):
        self.commands.append((left_byte, right_byte))

    def stop(self):
        pass

    def get_telemetry(self):
        return None


class FakeRelay:
    def set_armed(self, armed):
        pass


class FakeShutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


class FakeImu:
    """IMU compensator double with a controllable heading and heading age."""

    def __init__(self, heading_deg=0.0, age_s=0.02):
        self.heading = heading_deg
        self.age_s = age_s
        self.imu_reader = None

    def get_heading_deg(self):
        return self.heading

    def heading_age_s(self, now_s=None):
        return self.age_s

    def get_status(self):
        return MagicMock(heading_deg=self.heading, yaw_rate_dps=0.0)

    def set_target_heading(self, *_a, **_k):
        pass

    def reset_target_heading(self):
        pass

    def update(self, *_a, **_k):
        return None


@contextmanager
def _clock(clock):
    with patch("pi_app.control.controller.time.monotonic", side_effect=lambda: clock["t"]):
        yield


NORTH = Waypoint(lat=40.001, lon=-74.0, name="N")  # ~111 m north of (40, -74)


def _gps(clock, lat=40.0, lon=-74.0, fix_quality=4, age=0.0):
    return GpsReading(
        latitude=lat, longitude=lon, altitude_m=0.0, fix_quality=fix_quality,
        satellites_used=12, hdop=0.8, diff_age_s=0.5, station_id=1,
        timestamp=clock["t"] - age,
    )


class _Rig:
    def __init__(self, clock, *, heading=0.0, waypoints=None, aligner_enabled=True, imu="fake"):
        self.clock = clock
        self.aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=aligner_enabled))
        if aligner_enabled:
            self.aligner._locked = True
            self.aligner._offset_deg = 0.0
        self.imu = FakeImu(heading_deg=heading) if imu == "fake" else imu
        self.nav = WaypointNavController(WaypointNavConfig(), list(waypoints or [NORTH]))
        self.motor = FakeMotor()
        self.ctrl = Controller(
            motor_driver=self.motor,
            arm_relay=FakeRelay(),
            shutdown_scheduler=FakeShutdown(),
            imu_compensator=self.imu,
            waypoint_nav=self.nav,
            gps_heading_aligner=self.aligner,
        )
        self.ctrl._safety_state.is_armed = True
        self.ctrl._gps_reading = _gps(clock)

    def tick(self, dt=DT, **gps_kw):
        self.clock["t"] += dt
        self.ctrl._gps_reading = _gps(self.clock, **gps_kw)
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_mono_s=self.clock["t"])
        return self.ctrl.process(rc)


class TestRunLosesHeadingReference(unittest.TestCase):
    def test_lock_drop_mid_run_leaves_waypoint_nav(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            for _ in range(10):
                cmd, _, telem = rig.tick()
            self.assertEqual(telem["nav_state"], "DRIVE")
            self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")
            driving = cmd.left_byte
            self.assertGreater(driving, CENTER_OUTPUT_VALUE)

            rig.aligner.reset()  # what _check_imu_frame_continuity does
            cmd, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "MANUAL")
        self.assertEqual(telem["mode"], "MANUAL")
        self.assertEqual(telem["waypoint_nav_exit_reason"], "heading_lock_lost")
        # Same controlled exit as the other autonomous exits: the sticks
        # (centred) are the target and the slew limiter ramps toward them.
        self.assertLess(cmd.left_byte, driving)
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)

    def test_oak_reconnect_mid_run_leaves_waypoint_nav(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            health = {"oak_reconnect_count": 0, "count_cum_reset": 0}
            rig.imu.imu_reader = MagicMock(get_health=lambda: dict(health))
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            for _ in range(40):  # > 1 s: baseline marker recorded
                rig.tick()
            self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")
            health["oak_reconnect_count"] = 1
            reasons = []
            for _ in range(45):
                _, _, telem = rig.tick()
                if "waypoint_nav_exit_reason" in telem:
                    reasons.append(telem["waypoint_nav_exit_reason"])
            # A new run is refused until the robot relocks.
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "heading_alignment_not_locked")
        self.assertFalse(rig.aligner.locked)
        self.assertEqual(rig.ctrl._mode, "MANUAL")
        self.assertEqual(reasons, ["heading_lock_lost"])

    def test_stale_heading_leaves_waypoint_nav(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            rig.tick()
            rig.imu.age_s = HEADING_STALE_S + 0.05
            _, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "MANUAL")
        self.assertEqual(telem["waypoint_nav_exit_reason"], "heading_stale")

    def test_heading_at_the_limit_keeps_running(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            rig.imu.age_s = HEADING_STALE_S
            _, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")
        self.assertNotIn("waypoint_nav_exit_reason", telem)

    def test_non_finite_or_negative_heading_age_leaves(self):
        for bad in (float("inf"), float("nan"), -1.0):
            with self.subTest(age=bad):
                clock = {"t": 1000.0}
                with _clock(clock):
                    rig = _Rig(clock)
                    self.assertIsNone(rig.ctrl.activate_waypoint_nav())
                    rig.imu.age_s = bad
                    _, _, telem = rig.tick()
                self.assertEqual(rig.ctrl._mode, "MANUAL")
                self.assertEqual(telem["waypoint_nav_exit_reason"], "heading_stale")

    def test_compensator_without_heading_age_is_stale(self):
        class NoAgeImu(FakeImu):
            heading_age_s = None

        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock, imu=NoAgeImu())
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "heading_stale")


class TestActivationRefusedWhenStateUnknown(unittest.TestCase):
    def test_no_gps_reading_is_refused(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.ctrl._gps_reading = None
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "gps_state_unknown")
        self.assertEqual(rig.ctrl._mode, "MANUAL")

    def test_no_gps_reading_is_refused_even_with_aligner_disabled(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock, aligner_enabled=False)
            rig.ctrl._gps_reading = None
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "gps_state_unknown")

    def test_stale_gps_reading_is_refused(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.ctrl._gps_reading = _gps(clock, age=3.5)
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "gps_stale")
            rig.ctrl._gps_reading = _gps(clock, age=-1.0)  # stamp in the future
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "gps_stale")
            rig.ctrl._gps_reading = replace(_gps(clock), timestamp=float("nan"))
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "gps_stale")

    def test_rtk_float_is_refused(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.ctrl._gps_reading = _gps(clock, fix_quality=5)
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "gps_quality_not_trusted")

    def test_no_imu_is_refused_when_alignment_is_required(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock, imu=None)
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "heading_unavailable")

    def test_stale_heading_is_refused(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.imu.age_s = HEADING_STALE_S + 0.1
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "heading_stale")

    def test_good_state_activates(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            self.assertIsNone(rig.ctrl.waypoint_nav_activation_blocked())
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
        self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")


class TestAlignTimeout(unittest.TestCase):
    def test_align_that_never_converges_gives_up(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            # Bearing 0 deg, heading 180 deg, and the heading never changes
            # (a pivot that does not turn the robot).
            rig = _Rig(clock, heading=180.0)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            start = clock["t"]
            exit_at = None
            telem = {}
            while clock["t"] - start < ALIGN_TIMEOUT_S + 1.0:
                _, _, telem = rig.tick()
                if rig.ctrl._mode != "WAYPOINT_NAV":
                    exit_at = clock["t"] - start
                    break
                self.assertEqual(telem["nav_state"], "ALIGN")
        self.assertIsNotNone(exit_at)
        self.assertGreaterEqual(exit_at, ALIGN_TIMEOUT_S)
        self.assertLess(exit_at, ALIGN_TIMEOUT_S + 0.2)
        self.assertEqual(telem["waypoint_nav_exit_reason"], "align_timeout")

    def test_timer_restarts_after_drive(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock, heading=180.0)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            for _ in range(int((ALIGN_TIMEOUT_S - 1.0) / DT)):
                _, _, telem = rig.tick()
            self.assertEqual(telem["nav_state"], "ALIGN")
            rig.imu.heading = 0.0  # aligned: DRIVE
            for _ in range(5):
                _, _, telem = rig.tick()
            self.assertEqual(telem["nav_state"], "DRIVE")
            rig.imu.heading = 180.0  # recovery: ALIGN again, a new episode
            for _ in range(int((ALIGN_TIMEOUT_S - 1.0) / DT)):
                _, _, telem = rig.tick()
            self.assertEqual(telem["nav_state"], "ALIGN")
        self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")

    def test_intermediate_waypoint_still_rolls_through(self):
        clock = {"t": 1000.0}
        here = Waypoint(lat=40.0, lon=-74.0, name="HERE")  # inside arrival radius
        with _clock(clock):
            rig = _Rig(clock, heading=0.0, waypoints=[here, NORTH])
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            _, _, telem = rig.tick()
            self.assertEqual(telem["wp_index"], 1)
            for _ in range(3):
                cmd, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")
        self.assertEqual(telem["nav_state"], "DRIVE")
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)


class TestCompensatorHeadingAge(unittest.TestCase):
    """ImuSteeringCompensator.heading_age_s: time since the last good read
    plus the age of the sample that read returned."""

    def _comp(self, reader):
        cfg = replace(default_config.imu_steering, enabled=True)
        return ImuSteeringCompensator(cfg, reader)

    def _reader(self, sample_age_s=0.1):
        r = MagicMock()
        r.use_mag = False
        r.calibrate_gyro.return_value = (0.0, 0.0, 0.0)
        r.read.return_value = {
            "heading_deg": 10.0, "gz_dps": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "sample_age_s": sample_age_s,
        }
        return r

    def test_age_adds_sample_age(self):
        comp = self._comp(self._reader(sample_age_s=0.1))
        comp.get_heading_deg()
        t_read = comp.get_status().last_update_time
        self.assertAlmostEqual(comp.heading_age_s(t_read + 0.05), 0.15, places=6)

    def test_failed_reads_age_the_heading(self):
        reader = self._reader()
        comp = self._comp(reader)
        comp.get_heading_deg()
        t_read = comp.get_status().last_update_time
        reader.read.side_effect = RuntimeError("usb")
        comp.get_heading_deg()
        self.assertAlmostEqual(comp.heading_age_s(t_read + 2.0), 2.1, places=6)

    def test_unavailable_imu_is_infinitely_old(self):
        comp = self._comp(None)
        self.assertTrue(math.isinf(comp.heading_age_s(time.monotonic())))

    def test_bad_sample_age_is_infinitely_old(self):
        comp = self._comp(self._reader(sample_age_s=float("inf")))
        comp.get_heading_deg()
        self.assertTrue(math.isinf(comp.heading_age_s(comp.get_status().last_update_time)))


if __name__ == "__main__":
    unittest.main()
