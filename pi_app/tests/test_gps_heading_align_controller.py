"""Controller integration tests for GPS heading alignment hardening."""

import time
import unittest
from dataclasses import replace
from unittest.mock import MagicMock

from config import GpsHeadingAlignConfig
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.gps_heading_align import GpsHeadingAligner
from pi_app.control.safety import SafetyEvent, SafetyParams
from pi_app.control.waypoint_nav import Waypoint, WaypointNavConfig, WaypointNavController
from pi_app.hardware.rtk_gps import GpsReading


class FakeMotor:
    def set_tracks(self, left_byte, right_byte):
        pass

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
    def __init__(self, heading_deg: float = 0.0):
        self._heading = heading_deg
        self.target_history = []

    def get_heading_deg(self):
        return self._heading

    def get_status(self):
        return MagicMock(heading_deg=self._heading, yaw_rate_dps=0.0)

    def set_target_heading(self, heading, *_args, **_kwargs):
        self.target_history.append(heading)

    def reset_target_heading(self):
        pass

    def update(self, *_args, **_kwargs):
        return None


ARMED_RC = RCInputs(
    ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
    last_update_epoch_s=0.0,
)
DISARM_RC = RCInputs(
    ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000,
    last_update_epoch_s=0.0,
)


def _fresh_armed_rc() -> RCInputs:
    return replace(ARMED_RC, last_update_epoch_s=time.time())


def _fresh_disarm_rc() -> RCInputs:
    return replace(DISARM_RC, last_update_epoch_s=time.time())


def _make_controller(*, aligner=None, imu=None, waypoint_nav=None):
    return Controller(
        motor_driver=FakeMotor(),
        arm_relay=FakeRelay(),
        shutdown_scheduler=FakeShutdown(),
        imu_compensator=imu,
        waypoint_nav=waypoint_nav,
        gps_heading_aligner=aligner,
    )


def _gps(lat=40.0, lon=-74.0, fix_quality=4, ts=None) -> GpsReading:
    return GpsReading(
        latitude=lat,
        longitude=lon,
        altitude_m=0.0,
        fix_quality=fix_quality,
        satellites_used=12,
        hdop=0.8,
        diff_age_s=0.5,
        station_id=1,
        timestamp=ts if ts is not None else time.monotonic(),
    )


class TestHeadingAlignDisarmReset(unittest.TestCase):

    def test_manual_disarm_resets_aligner(self):
        cfg = GpsHeadingAlignConfig(enabled=True, min_distance_m=0.5, min_speed_mps=0.1)
        aligner = GpsHeadingAligner(cfg)
        aligner._locked = True
        aligner._offset_deg = 12.0
        ctrl = _make_controller(aligner=aligner, imu=FakeImu())
        ctrl._gps_reading = _gps()
        ctrl._safety_state.is_armed = True

        ctrl.process(_fresh_armed_rc())
        self.assertTrue(aligner.locked)

        ctrl.process(_fresh_disarm_rc())
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.offset_deg, 0.0)

    def test_rc_stale_resets_aligner(self):
        cfg = GpsHeadingAlignConfig(enabled=True)
        aligner = GpsHeadingAligner(cfg)
        aligner._locked = True
        aligner._offset_deg = 7.0
        ctrl = _make_controller(aligner=aligner, imu=FakeImu())
        ctrl._gps_reading = _gps()
        ctrl._safety_state.is_armed = True
        stale_rc = replace(_fresh_armed_rc(), last_update_epoch_s=time.time() - 5.0)

        _, events, _ = ctrl.process(stale_rc)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.offset_deg, 0.0)

    def test_initial_disarmed_tick_does_not_reset(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        aligner._offset_deg = 4.0
        ctrl = _make_controller(aligner=aligner, imu=FakeImu())
        ctrl._gps_reading = _gps()
        ctrl._safety_state.is_armed = False

        ctrl.process(_fresh_disarm_rc())
        self.assertTrue(aligner.locked)
        self.assertEqual(aligner.offset_deg, 4.0)

    def test_emergency_resets_aligner(self):
        cfg = GpsHeadingAlignConfig(enabled=True)
        aligner = GpsHeadingAligner(cfg)
        aligner._locked = True
        aligner._offset_deg = 9.0
        ctrl = _make_controller(aligner=aligner, imu=FakeImu())
        ctrl._gps_reading = _gps()
        ctrl._safety_state.is_armed = True
        estop_rc = replace(_fresh_armed_rc(), ch5_us=1900)

        _, events, _ = ctrl.process(estop_rc)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.offset_deg, 0.0)


class TestHeadingAlignManualForwardGate(unittest.TestCase):

    @staticmethod
    def _aligner() -> GpsHeadingAligner:
        return GpsHeadingAligner(
            GpsHeadingAlignConfig(
                enabled=True,
                min_distance_m=0.5,
                min_speed_mps=0.1,
                max_lock_yaw_rate_dps=3.0,
            )
        )

    def test_equal_reverse_rc_never_collects_or_locks(self):
        aligner = self._aligner()
        ctrl = _make_controller(aligner=aligner, imu=FakeImu(heading_deg=5.0))
        ctrl._safety_state.is_armed = True
        reverse_rc = replace(
            _fresh_armed_rc(),
            ch1_us=1300,
            ch2_us=1300,
        )

        ctrl._gps_reading = _gps(lat=40.0, ts=1_000.0)
        ctrl.process(reverse_rc)
        ctrl._gps_reading = _gps(lat=40.00001, ts=1_001.0)
        ctrl.process(reverse_rc)

        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 0)

    def test_equal_forward_rc_collects_and_locks(self):
        aligner = self._aligner()
        ctrl = _make_controller(aligner=aligner, imu=FakeImu(heading_deg=5.0))
        ctrl._safety_state.is_armed = True
        forward_rc = replace(
            _fresh_armed_rc(),
            ch1_us=1700,
            ch2_us=1700,
        )

        ctrl._gps_reading = _gps(lat=40.0, ts=1_000.0)
        ctrl.process(forward_rc)
        ctrl._gps_reading = _gps(lat=40.00001, ts=1_001.0)
        ctrl.process(forward_rc)

        self.assertTrue(aligner.locked)
        self.assertAlmostEqual(aligner.offset_deg, -5.0, places=1)


class TestWaypointNavActivationGate(unittest.TestCase):

    def _nav(self):
        cfg = WaypointNavConfig()
        return WaypointNavController(cfg, [Waypoint(lat=40.001, lon=-74.0, name="T")])

    def test_blocks_when_aligner_enabled_but_not_locked(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        ctrl = _make_controller(
            aligner=aligner,
            imu=FakeImu(),
            waypoint_nav=self._nav(),
        )
        ctrl._gps_reading = _gps()
        reason = ctrl.activate_waypoint_nav()
        self.assertEqual(reason, "heading_alignment_not_locked")

    def test_allows_when_locked(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        ctrl = _make_controller(
            aligner=aligner,
            imu=FakeImu(),
            waypoint_nav=self._nav(),
        )
        ctrl._gps_reading = _gps()
        self.assertIsNone(ctrl.activate_waypoint_nav())
        self.assertEqual(ctrl._mode, "WAYPOINT_NAV")

    def test_allows_when_aligner_disabled(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=False))
        ctrl = _make_controller(
            aligner=aligner,
            imu=FakeImu(),
            waypoint_nav=self._nav(),
        )
        ctrl._gps_reading = _gps()
        self.assertIsNone(ctrl.activate_waypoint_nav())

    def test_blocks_rtk_float_even_when_aligner_disabled(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=False))
        ctrl = _make_controller(
            aligner=aligner,
            imu=FakeImu(),
            waypoint_nav=self._nav(),
        )
        ctrl._gps_reading = _gps(fix_quality=5)

        self.assertEqual(ctrl.activate_waypoint_nav(), "gps_quality_not_trusted")
        self.assertEqual(ctrl._mode, "MANUAL")

    def test_allows_when_gps_stack_absent(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        ctrl = _make_controller(
            aligner=aligner,
            imu=None,
            waypoint_nav=self._nav(),
        )
        self.assertIsNone(ctrl.activate_waypoint_nav())


class TestHeadingAlignTelemetry(unittest.TestCase):

    def test_telemetry_includes_corrected_heading(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        aligner._offset_deg = 10.0
        ctrl = _make_controller(aligner=aligner, imu=FakeImu(heading_deg=80.0))
        ctrl._gps_reading = _gps()
        ctrl._safety_state.is_armed = True

        _, _, telem = ctrl.process(_fresh_armed_rc())
        self.assertAlmostEqual(telem["corrected_heading_deg"], 90.0, places=3)
        self.assertTrue(telem["heading_align"]["locked"])
        self.assertTrue(telem["heading_align"]["frozen"])
        self.assertFalse(telem["heading_align"]["refining"])
        self.assertAlmostEqual(telem["heading_align"]["offset_deg"], 10.0, places=3)
        self.assertTrue(telem["heading_offset_frozen"])
        self.assertFalse(telem["heading_offset_refining"])


class TestWaypointHeadingTargetOwnership(unittest.TestCase):

    def test_waypoint_drive_keeps_live_bearing_target(self):
        """Regression: generic straight hold overwrote waypoint target at DRIVE entry."""
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        aligner._offset_deg = -34.9
        imu = FakeImu(heading_deg=42.0)  # corrected 7.1°, within ALIGN threshold
        nav = WaypointNavController(
            WaypointNavConfig(align_threshold_deg=12.0),
            [Waypoint(lat=40.001, lon=-74.0, name="N")],
        )
        ctrl = _make_controller(aligner=aligner, imu=imu, waypoint_nav=nav)
        ctrl._gps_reading = _gps(lat=40.0, lon=-74.0)
        ctrl._safety_state.is_armed = True
        ctrl._mode = "WAYPOINT_NAV"

        _, _, telem = ctrl.process(_fresh_armed_rc())

        self.assertEqual(telem["nav_state"], "DRIVE")
        self.assertAlmostEqual(telem["wp_bearing_deg"], 0.0, places=1)
        self.assertAlmostEqual(
            imu.target_history[-1],
            aligner.imu_target_heading(telem["wp_bearing_deg"]),
            places=3,
        )
        self.assertNotAlmostEqual(imu.target_history[-1], imu._heading, places=3)

    def test_waypoint_mode_cannot_collect_history_or_change_frozen_offset(self):
        cfg = GpsHeadingAlignConfig(
            enabled=True,
            min_distance_m=0.5,
            min_speed_mps=0.1,
        )
        nav = WaypointNavController(
            WaypointNavConfig(),
            [Waypoint(lat=40.001, lon=-74.0, name="N")],
        )
        aligner = GpsHeadingAligner(cfg)
        ctrl = _make_controller(aligner=aligner, imu=FakeImu(), waypoint_nav=nav)
        ctrl._safety_state.is_armed = True
        ctrl._mode = "WAYPOINT_NAV"
        base_ts = time.monotonic()

        ctrl._gps_reading = _gps(lat=40.0, ts=base_ts)
        ctrl.process(_fresh_armed_rc())
        ctrl._gps_reading = _gps(lat=40.00001, ts=base_ts + 1.0)
        ctrl.process(_fresh_armed_rc())

        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().history_samples, 0)

        aligner._locked = True
        aligner._offset_deg = 175.7
        ctrl._gps_reading = _gps(
            lat=40.00002,
            lon=-73.99999,
            ts=base_ts + 2.0,
        )
        ctrl.process(_fresh_armed_rc())

        self.assertAlmostEqual(aligner.offset_deg, 175.7, places=5)
        self.assertEqual(aligner.status().history_samples, 0)


if __name__ == "__main__":
    unittest.main()


class TestCourseOverGroundLockThroughController(unittest.TestCase):
    """Web/BT teleop forward drive can lock via the receiver's course over ground."""

    def _aligner(self):
        return GpsHeadingAligner(
            GpsHeadingAlignConfig(
                enabled=True, min_fix_quality=4, min_distance_m=100.0,  # displacement path cannot lock
                min_speed_mps=0.1, max_lock_yaw_rate_dps=3.0,
                cog_lock_enabled=True, cog_min_speed_mps=0.3, cog_max_yaw_rate_dps=6.0,
                cog_min_samples=6, cog_max_spread_deg=8.0, cog_window_s=20.0,
            )
        )

    def test_bt_forward_drive_locks_from_cog(self):
        aligner = self._aligner()
        ctrl = _make_controller(aligner=aligner, imu=FakeImu(heading_deg=30.0))
        ctrl._safety_state.is_armed = True
        rc = _fresh_armed_rc()
        # Web/BT teleop forward at a moderate byte offset; the slew ramps the
        # emitted bytes above neutral+6 within a few ticks.
        for i in range(40):
            ctrl._gps_reading = replace(_gps(lat=40.0, ts=1_000.0 + i), cog_deg=120.0, sog_mps=0.6)
            ctrl.process(rc, bt_override_bytes=(160, 160))
        self.assertTrue(aligner.locked)
        self.assertEqual(aligner.lock_source, "cog")
        self.assertAlmostEqual(aligner.offset_deg, 90.0, places=1)

    def test_reverse_drive_never_locks_from_cog(self):
        aligner = self._aligner()
        ctrl = _make_controller(aligner=aligner, imu=FakeImu(heading_deg=30.0))
        ctrl._safety_state.is_armed = True
        rc = _fresh_armed_rc()
        for i in range(40):
            ctrl._gps_reading = replace(_gps(lat=40.0, ts=1_000.0 + i), cog_deg=300.0, sog_mps=0.6)
            ctrl.process(rc, bt_override_bytes=(90, 90))
        self.assertFalse(aligner.locked)
        self.assertEqual(aligner.status().cog_samples, 0)
