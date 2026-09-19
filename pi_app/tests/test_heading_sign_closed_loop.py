"""Closed-loop sign tests: a wrong heading sign must fail these, not pass them.

Why this file exists
--------------------
Before 2026-09-19 the OAK reader published a counter-clockwise-positive heading
(``heading = -yaw``) while ``gyro_y`` was already clockwise-positive, and two
downstream compensators hid it: ``ImuSteeringConfig.invert_output = True`` and
a negated ALIGN pivot in ``waypoint_nav``. Every existing unit test passed,
because each one checked a magnitude or an isolated term. The bug only shows up
when the controller is closed around a plant.

These tests close that loop against a tiny kinematic model, so an inverted sign
anywhere in the chain (reader, compensator, mixer, navigator) diverges instead
of converging.

Convention under test (canonical statement: ``OakImuReader.read``):
``heading_deg`` is compass-style, CLOCKWISE-POSITIVE viewed from above, in
``[0, 360)``; ``gz_dps`` == ``d(heading_deg)/dt``, so a right turn is positive.
"""

from __future__ import annotations

import math
import unittest

from config import ImuSteeringConfig
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.waypoint_nav import (
    NavState,
    Waypoint,
    WaypointNavConfig,
    WaypointNavController,
    mix_to_bytes,
)

# Plant gain: degrees of clockwise rotation per second per byte of (left-right).
# left faster than right => clockwise => heading INCREASES.
PLANT_K_DEG_PER_BYTE_S = 0.35
NEUTRAL = 126


class KinematicPlant:
    """Heading-only skid-steer model. heading += k * (left - right) * dt."""

    def __init__(self, heading_deg: float = 0.0, k: float = PLANT_K_DEG_PER_BYTE_S):
        self.heading_true_deg = float(heading_deg)
        self.k = float(k)
        self.last_rate_dps = 0.0

    def step(self, left_byte: float, right_byte: float, dt: float) -> None:
        rate = self.k * (float(left_byte) - float(right_byte))
        self.last_rate_dps = rate
        self.heading_true_deg += rate * dt


class PlantImuStub:
    """IMU reader backed by the plant, in the production sign convention."""

    use_mag = False
    calibration_path = None

    def __init__(self, plant: KinematicPlant):
        self._plant = plant

    def calibrate_gyro(self, duration_s: float = 3.0):
        return (0.0, 0.0, 0.0)

    def calibrate_mag_hard_iron(self, duration_s: float = 5.0):
        return (0.0, 0.0, 0.0)

    def read(self) -> dict:
        return {
            "heading_deg": self._plant.heading_true_deg % 360.0,
            # d(heading)/dt — clockwise-positive, same as heading.
            "gz_dps": self._plant.last_rate_dps,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
        }


def _signed_err(target: float, heading: float) -> float:
    return ((target - heading + 180.0) % 360.0) - 180.0


def run_heading_hold(
    *,
    invert_output: bool,
    initial_error_deg: float,
    duration_s: float = 10.0,
    dt: float = 0.02,
    forward_offset_byte: int = 40,
) -> tuple[float, float]:
    """Close the heading-hold loop; return (final |error|, worst |error|).

    The controller application rule is copied from ``controller.py``:
    ``left = base + corr``, ``right = base - corr`` — a positive correction
    increases the left track and decreases the right one, i.e. turns right.
    """
    # target 0, robot starts rotated by +initial_error_deg (heading ahead of
    # target), so the loop must steer LEFT (a negative correction) to recover.
    plant = KinematicPlant(heading_deg=initial_error_deg)
    imu = PlantImuStub(plant)
    cfg = ImuSteeringConfig(invert_output=invert_output, calibration_timeout_s=0.1)
    comp = ImuSteeringCompensator(cfg, imu)
    comp.set_target_heading(0.0)

    worst = abs(_signed_err(0.0, plant.heading_true_deg))
    steps = int(round(duration_s / dt))
    for _ in range(steps):
        corr = comp.update(0.0, dt)
        corr = 0.0 if corr is None else float(corr)
        left = NEUTRAL + forward_offset_byte + corr
        right = NEUTRAL + forward_offset_byte - corr
        plant.step(left, right, dt)
        worst = max(worst, abs(_signed_err(0.0, plant.heading_true_deg)))
    return abs(_signed_err(0.0, plant.heading_true_deg)), worst


class TestHeadingHoldClosedLoop(unittest.TestCase):
    def test_production_defaults_converge_without_overshooting_start(self):
        final, worst = run_heading_hold(invert_output=False, initial_error_deg=20.0)
        self.assertLess(final, 2.0, f"heading hold did not converge (final={final:.2f}°)")
        # Never worse than where it started: no sign-flip runaway, no wild ring.
        self.assertLessEqual(worst, 20.0 + 1e-6, f"overshot the initial error (worst={worst:.2f}°)")

    def test_mirrored_initial_error_also_converges(self):
        final, worst = run_heading_hold(invert_output=False, initial_error_deg=-20.0)
        self.assertLess(final, 2.0)
        self.assertLessEqual(worst, 20.0 + 1e-6)

    def test_invert_output_true_fails_to_converge(self):
        """Documents why the default flipped: the old setting is unstable here."""
        final, worst = run_heading_hold(invert_output=True, initial_error_deg=20.0)
        self.assertGreater(
            final, 20.0,
            "invert_output=True must NOT converge with a CW-positive heading "
            f"(final={final:.2f}°)",
        )
        self.assertGreater(worst, 20.0)

    def test_yaw_rate_term_damps_rather_than_amplifies(self):
        """A pure yaw-rate disturbance at zero error must be opposed.

        Under the old double flip, d_term = -kd * yaw_rate was ANTI-damping
        because the yaw rate was CW-positive while the heading was CCW-positive.
        """
        plant = KinematicPlant(heading_deg=0.0)
        imu = PlantImuStub(plant)
        comp = ImuSteeringCompensator(
            ImuSteeringConfig(kp=0.0, ki=0.0, kd=0.5, deadband_deg=0.0,
                              dterm_ema_alpha=0.0, calibration_timeout_s=0.1),
            imu,
        )
        comp.set_target_heading(0.0)
        # Robot is rotating clockwise (right) at +30 deg/s with zero error.
        plant.last_rate_dps = 30.0
        corr = comp.update(0.0, 0.02)
        self.assertIsNotNone(corr)
        # A negative correction is left-turning, which opposes the CW rotation.
        self.assertLess(float(corr), 0.0)


class TestWaypointAlignClosedLoop(unittest.TestCase):
    """ALIGN must pivot toward the bearing, through the real byte mixer."""

    def _nav_and_target(self, bearing_deg_target: float):
        cfg = WaypointNavConfig(
            arrival_radius_m=1.0,
            cruise_speed_byte=40,
            approach_speed_byte=20,
            slow_radius_m=2.0,
            min_rtk_quality=4,
            stale_timeout_s=3.0,
            align_threshold_deg=12.0,
            recovery_threshold_deg=25.0,
            pivot_yaw_cmd=0.5,
        )
        # ~111 m away on the requested bearing (near the equator).
        rad = math.radians(bearing_deg_target)
        wp = Waypoint(lat=0.001 * math.cos(rad), lon=0.001 * math.sin(rad), name="T")
        return WaypointNavController(cfg, [wp]), cfg

    def _drive_align(self, bearing_deg_target: float):
        nav, cfg = self._nav_and_target(bearing_deg_target)
        plant = KinematicPlant(heading_deg=0.0)
        dt = 0.02
        first_yaw = None
        reached_drive = False
        for _ in range(1000):
            v_cmd, yaw_cmd, state = nav.compute(
                0.0, 0.0, fix_quality=4, gps_age_s=0.0,
                current_heading_deg=plant.heading_true_deg % 360.0,
            )
            if first_yaw is None:
                first_yaw = yaw_cmd
            if state == NavState.DRIVE:
                reached_drive = True
                break
            left, right = mix_to_bytes(v_cmd, yaw_cmd, deadband_byte=12, neutral=NEUTRAL)
            plant.step(left, right, dt)
        err = abs(nav.get_status().heading_error_deg)
        return first_yaw, err, reached_drive, cfg

    def test_bearing_clockwise_of_heading_pivots_right_and_aligns(self):
        first_yaw, err, reached_drive, cfg = self._drive_align(40.0)
        self.assertGreater(first_yaw, 0.0, "bearing CW of heading must command yaw > 0")
        self.assertTrue(reached_drive, "ALIGN never converged into DRIVE")
        self.assertLess(err, cfg.align_threshold_deg)

    def test_bearing_counter_clockwise_of_heading_pivots_left_and_aligns(self):
        first_yaw, err, reached_drive, cfg = self._drive_align(-40.0)
        self.assertLess(first_yaw, 0.0, "bearing CCW of heading must command yaw < 0")
        self.assertTrue(reached_drive, "ALIGN never converged into DRIVE")
        self.assertLess(err, cfg.align_threshold_deg)


if __name__ == "__main__":
    unittest.main()
