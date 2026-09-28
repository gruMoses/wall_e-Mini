"""MANUAL heading hold must re-target after the operator steers (2026-09-27).

Field: RTK fixed at 16:52 locked the GPS heading offset (+22.5 deg). With
the offset locked, the controller re-applied a true-frame target, captured at
straight-drive entry, on every tick. The straight latch survives stick pulses
(0.8 s hysteresis, |steering| <= 0.18 counts as straight), so the target
stayed at the old heading while the operator turned right. Each time the
sticks evened out, the hold drove back to the left at up to the 35-byte cap
(17:32: 52 s of right stick, 27 deg to the left).

These tests run the real Controller and the real ImuSteeringCompensator on a
fake clock, with the offset locked and without an aligner. The heading is
scripted, so the scenario matches the log exactly: straight at 284 deg, a
right pulse that the controller still calls "straight" while the heading goes
to 314 deg, then even sticks again.
"""

import time
import unittest
import unittest.mock
from dataclasses import replace

from config import GpsHeadingAlignConfig, ImuSteeringConfig
from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.gps_heading_align import GpsHeadingAligner
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.waypoint_nav import Waypoint, WaypointNavConfig, WaypointNavController
from pi_app.hardware.rtk_gps import GpsReading

TICK_S = 1.0 / 30.0


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


class ScriptedImu:
    """IMU reader whose heading the test sets directly (CW-positive)."""

    use_mag = False
    calibration_path = None

    def __init__(self, heading_deg: float):
        self.heading_deg = heading_deg

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
        }


def _rc(ch1: int, ch2: int) -> RCInputs:
    # ch3 high = armed, ch4 low = MANUAL.
    return RCInputs(
        ch1_us=ch1, ch2_us=ch2, ch3_us=1900, ch4_us=1000, ch5_us=1000,
        last_update_epoch_s=time.time(),
    )


def _signed(a: float) -> float:
    return ((a + 180.0) % 360.0) - 180.0


class TestManualHeadingHoldRetarget(unittest.TestCase):

    def _run(self, aligner, expo=0.0):
        clock = {"t": 5_000.0}
        imu = ScriptedImu(284.0)
        comp = ImuSteeringCompensator(ImuSteeringConfig(calibration_timeout_s=0.1), imu)
        # expo 0.0 is the field run (stick expo came later that day). The
        # steering intent reads the linear stick bytes, so the pulse values
        # below reproduce the logged steering inputs at any expo.
        cfg = replace(default_config, rc_map=replace(default_config.rc_map, stick_expo=expo))
        with unittest.mock.patch("pi_app.control.controller.time.monotonic", lambda: clock["t"]), \
                unittest.mock.patch("pi_app.control.controller.config", cfg):
            ctrl = Controller(
                motor_driver=FakeMotor(),
                arm_relay=FakeRelay(),
                shutdown_scheduler=FakeShutdown(),
                imu_compensator=comp,
                gps_heading_aligner=aligner,
            )
            ctrl._safety_state.is_armed = True

            def tick(ch1, ch2):
                clock["t"] += TICK_S
                _, _, telem = ctrl.process(replace(_rc(ch1, ch2), last_update_epoch_s=time.time()))
                return telem

            # 1) Straight at 284 deg: the hold locks 284.
            for _ in range(30):
                tick(1800, 1800)
            self.assertAlmostEqual(comp.get_status().target_heading_deg, 284.0, delta=0.5)

            # 2) Right pulse. |steering| ~0.17: the compensator leaves neutral
            #    (exit 0.15) while the controller still latches "straight"
            #    (manual_hold_max_steering 0.18). Heading 284 -> 314.
            pulse = []
            for i in range(30):
                imu.heading_deg = 284.0 + 30.0 * (i + 1) / 30.0
                pulse.append(tick(1850, 1700))
            # Premise: this is the case the field log shows.
            self.assertTrue(all(t.get("straight_intent") for t in pulse),
                            "scenario premise: the pulse must stay latched as straight")
            self.assertTrue(all(t.get("imu_correction_applied") is None for t in pulse[3:]),
                            "scenario premise: the hold must be off while the operator steers")

            # 3) Even sticks again at 314 deg: the hold must keep 314, not
            #    drive back to 284.
            back = [tick(1800, 1800) for _ in range(15)]
            target = comp.get_status().target_heading_deg
            corrections = [t.get("imu_correction_applied") or 0.0 for t in back]

            # 4) The hold is still alive: drift 5 deg right, expect a left
            #    (negative) correction back toward 314.
            imu.heading_deg = 319.0
            alive = [tick(1800, 1800).get("imu_correction_applied") or 0.0 for _ in range(3)]
        self.assertLess(min(alive), -3.0, "heading hold stopped correcting")
        return target, corrections

    def test_locked_offset_holds_the_new_heading(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        aligner._offset_deg = 22.5
        target, corrections = self._run(aligner)
        self.assertTrue(aligner.locked, "the lock must stay on for this case to mean anything")
        self.assertLess(abs(_signed(target - 314.0)), 1.0,
                        "target reverted to the heading at straight entry")
        self.assertLess(max(abs(c) for c in corrections), 3.0,
                        "the hold dragged the robot back toward the old heading")

    def test_locked_offset_holds_the_new_heading_at_the_shipped_expo(self):
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        aligner._offset_deg = 22.5
        target, corrections = self._run(aligner, expo=default_config.rc_map.stick_expo)
        self.assertLess(abs(_signed(target - 314.0)), 1.0)
        self.assertLess(max(abs(c) for c in corrections), 3.0)

    def test_without_aligner_holds_the_new_heading(self):
        target, corrections = self._run(None)
        self.assertLess(abs(_signed(target - 314.0)), 1.0)
        self.assertLess(max(abs(c) for c in corrections), 3.0)

    def test_waypoint_target_does_not_survive_into_manual(self):
        """The straight latch carries across a mode switch (review of bd27f44).

        Nav stops (UI stop or completion) while the sticks are forward and
        equal: the first MANUAL straight tick must lock the current heading,
        not keep steering toward the waypoint bearing.
        """
        clock = {"t": 5_000.0}
        imu = ScriptedImu(327.0)
        comp = ImuSteeringCompensator(ImuSteeringConfig(calibration_timeout_s=0.1), imu)
        aligner = GpsHeadingAligner(GpsHeadingAlignConfig(enabled=True))
        aligner._locked = True
        aligner._offset_deg = 22.5  # due north = raw 337.5, 10.5 deg right of the heading
        nav = WaypointNavController(
            WaypointNavConfig(align_threshold_deg=12.0),
            [Waypoint(lat=40.001, lon=-74.0, name="N")],
        )
        with unittest.mock.patch("pi_app.control.controller.time.monotonic", lambda: clock["t"]):
            ctrl = Controller(
                motor_driver=FakeMotor(),
                arm_relay=FakeRelay(),
                shutdown_scheduler=FakeShutdown(),
                imu_compensator=comp,
                gps_heading_aligner=aligner,
                waypoint_nav=nav,
            )
            ctrl._safety_state.is_armed = True
            ctrl._mode = "WAYPOINT_NAV"

            def tick(ch1, ch2):
                clock["t"] += TICK_S
                ctrl._gps_reading = GpsReading(
                    latitude=40.0, longitude=-74.0, altitude_m=0.0, fix_quality=4,
                    satellites_used=12, hdop=0.8, diff_age_s=0.5, station_id=1,
                    timestamp=clock["t"],
                )
                _, _, telem = ctrl.process(_rc(ch1, ch2))
                return telem

            nav_ticks = [tick(1800, 1800) for _ in range(10)]
            # Premises: nav is driving, the latch is on at the switch, and the
            # hold is steering toward the bearing, not the heading.
            self.assertEqual(nav_ticks[-1]["nav_state"], "DRIVE")
            self.assertTrue(nav_ticks[-1].get("straight_intent"))
            self.assertGreater(abs(_signed(comp.get_status().target_heading_deg - 327.0)), 5.0)

            ctrl.deactivate_waypoint_nav()
            tick(1800, 1800)
            target = comp.get_status().target_heading_deg
        self.assertLess(abs(_signed(target - 327.0)), 0.5,
                        "the waypoint target survived into MANUAL")


if __name__ == "__main__":
    unittest.main()
