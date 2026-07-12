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

    def get_heading_deg(self):
        return self._heading

    def get_status(self):
        return MagicMock(heading_deg=self._heading)

    def set_target_heading(self, *_args, **_kwargs):
        pass

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
        self.assertAlmostEqual(telem["heading_align"]["offset_deg"], 10.0, places=3)


if __name__ == "__main__":
    unittest.main()
