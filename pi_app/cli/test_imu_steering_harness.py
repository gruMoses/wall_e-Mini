#!/usr/bin/env python3
"""
Standalone test harness for IMU steering compensator logic.

Covers:
- Initialization and availability
- Neutral detection and target-heading updates
- Deadband behavior
- Proportional term sign and magnitude
- Integral accumulation and clamp
- Derivative (yaw-rate) damping sign and magnitude
- Output clamp to max_correction
- Heading wrap-around normalization
- Non-neutral steering disables compensation
"""

import sys
import time
from pathlib import Path

# Ensure project root is on path
sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ImuSteeringConfig  # type: ignore
from pi_app.control.imu_steering import ImuSteeringCompensator  # type: ignore


class FakeImuReader:
    """Minimal IMU reader stub to drive the compensator deterministically."""

    def __init__(self, heading_deg: float = 0.0, yaw_rate_dps: float = 0.0):
        self.heading_deg = heading_deg % 360.0
        self._yaw_rate_dps = yaw_rate_dps
        self.roll_deg = 0.0
        self.pitch_deg = 0.0

    def calibrate_gyro(self, duration_s: float) -> None:  # noqa: D401
        pass

    def calibrate_mag_hard_iron(self, duration_s: float) -> None:  # noqa: D401
        pass

    def step(self, yaw_rate_dps: float, dt: float) -> None:
        """Advance simulated heading given a yaw rate and time delta."""
        self._yaw_rate_dps = float(yaw_rate_dps)
        self.heading_deg = (self.heading_deg + self._yaw_rate_dps * dt) % 360.0

    def read(self) -> dict:
        return {
            "heading_deg": float(self.heading_deg),
            "gz_dps": float(self._yaw_rate_dps),
            "roll_deg": float(self.roll_deg),
            "pitch_deg": float(self.pitch_deg),
        }


def assert_almost_equal(name: str, actual: float, expected: float, tol: float = 1e-6) -> None:
    if abs(actual - expected) > tol:
        raise AssertionError(f"{name}: expected {expected:.6f}, got {actual:.6f}")


def assert_true(name: str, cond: bool) -> None:
    if not cond:
        raise AssertionError(f"Assertion failed: {name}")


def run_tests() -> None:
    passed = 0
    failed = 0

    def _run(test_fn):
        nonlocal passed, failed
        try:
            test_fn()
            print(f"PASS: {test_fn.__name__}")
            passed += 1
        except Exception as e:  # noqa: BLE001
            print(f"FAIL: {test_fn.__name__} -> {e}")
            failed += 1

    # Tests
    _run(test_initialization)
    _run(test_deadband)
    _run(test_proportional_only)
    _run(test_integral_clamp)
    _run(test_derivative_only)
    _run(test_output_clamp)
    _run(test_wraparound_error)
    _run(test_neutral_transitions)
    _run(test_non_neutral_disable)

    print(f"\nSummary: {passed} passed, {failed} failed")
    if failed:
        raise SystemExit(1)


def make_compensator(
    imu: FakeImuReader,
    *,
    kp: float = 0.3,
    ki: float = 0.1,
    kd: float = 0.8,
    max_correction: int = 40,
    deadband_deg: float = 3.0,
    max_integral: float = 25.0,
) -> ImuSteeringCompensator:
    cfg = ImuSteeringConfig(
        enabled=True,
        kp=kp,
        ki=ki,
        kd=kd,
        max_correction=max_correction,
        deadband_deg=deadband_deg,
        max_integral=max_integral,
        invert_output=True,  # applied inside compensator
        log_steering_corrections=False,
        update_rate_hz=20.0,
        heading_hold_timeout_s=0.5,
        fallback_on_error=True,
        calibration_timeout_s=1.0,
    )
    return ImuSteeringCompensator(cfg, imu)


def tick(comp: ImuSteeringCompensator, imu: FakeImuReader, steering_input: float, dt: float) -> float | None:
    """Advance IMU by dt with its current yaw rate and call update."""
    imu.step(imu._yaw_rate_dps, dt)
    return comp.update(steering_input, dt)


def test_initialization() -> None:
    imu = FakeImuReader(heading_deg=10.0)
    comp = make_compensator(imu)
    st = comp.get_status()
    assert_true("IMU available after init", st.is_available)
    assert_almost_equal("Initial heading populated", st.heading_deg, 10.0, tol=1e-6)


def test_deadband() -> None:
    imu = FakeImuReader(heading_deg=0.5)
    comp = make_compensator(imu, kp=0.5, ki=0.0, kd=0.0, deadband_deg=2.0)
    comp.set_target_heading(0.0)
    corr = tick(comp, imu, steering_input=0.0, dt=0.05)
    assert_almost_equal("Deadband returns zero", float(corr or 0.0), 0.0, tol=1e-9)


def test_proportional_only() -> None:
    imu = FakeImuReader(heading_deg=10.0)
    comp = make_compensator(imu, kp=1.0, ki=0.0, kd=0.0, deadband_deg=0.0)
    comp.set_target_heading(0.0)
    corr = tick(comp, imu, steering_input=0.0, dt=0.05)
    # error = target - heading = -10 -> correction = +10 after inversion
    assert_almost_equal("P-only correction", float(corr or 0.0), 10.0, tol=1e-6)


def test_integral_clamp() -> None:
    imu = FakeImuReader(heading_deg=0.0)
    comp = make_compensator(imu, kp=0.0, ki=1.0, kd=0.0, deadband_deg=0.0, max_integral=2.5)
    comp.set_target_heading(10.0)  # constant +10 deg error
    # Run long enough to exceed integral clamp: integral = sum(error*dt)
    dt = 0.1
    for _ in range(200):
        corr = tick(comp, imu, steering_input=0.0, dt=dt)
        # keep IMU heading constant to isolate integral
        imu.heading_deg = 0.0
    # At clamp: integral_error = max_integral = 2.5; correction = -2.5 after inversion
    assert_almost_equal("Integral clamp", float(corr or 0.0), -2.5, tol=1e-3)


def test_derivative_only() -> None:
    imu = FakeImuReader(heading_deg=0.0, yaw_rate_dps=20.0)
    comp = make_compensator(imu, kp=0.0, ki=0.0, kd=0.8, deadband_deg=0.0)
    comp.set_target_heading(0.0)
    corr = tick(comp, imu, steering_input=0.0, dt=0.05)
    # d term = -kd * yaw_rate = -16 -> +16 after inversion
    assert_almost_equal("Derivative term", float(corr or 0.0), 16.0, tol=1e-6)


def test_output_clamp() -> None:
    imu = FakeImuReader(heading_deg=100.0)
    comp = make_compensator(imu, kp=10.0, ki=0.0, kd=0.0, deadband_deg=0.0, max_correction=40)
    comp.set_target_heading(0.0)
    corr = tick(comp, imu, steering_input=0.0, dt=0.05)
    assert_almost_equal("Output clamp (pos)", float(corr or 0.0), 40.0, tol=1e-6)


def test_wraparound_error() -> None:
    # Heading slightly ahead of target across 0/360 boundary
    imu = FakeImuReader(heading_deg=1.0)
    comp = make_compensator(imu, kp=1.0, ki=0.0, kd=0.0, deadband_deg=0.0)
    comp.set_target_heading(359.0)
    corr = tick(comp, imu, steering_input=0.0, dt=0.05)
    # error = 359 - 1 = 358 -> normalized to -2 -> +2 after inversion
    assert_almost_equal("Wrap-around error", float(corr or 0.0), 2.0, tol=1e-6)


def test_neutral_transitions() -> None:
    imu = FakeImuReader(heading_deg=45.0)
    comp = make_compensator(imu, kp=1.0, ki=1.0, kd=0.0, deadband_deg=0.0)
    # Start non-neutral, then enter neutral -> should set target to current heading and reset integral
    comp.update(steering_input=0.2, dt=0.05)  # non-neutral
    # change heading to 50, then go neutral; target should become 50
    imu.heading_deg = 50.0
    comp.update(steering_input=0.0, dt=0.05)  # transition to neutral
    st = comp.get_status()
    assert_almost_equal("Target set on entering neutral", st.target_heading_deg, 50.0, tol=1e-6)
    assert_almost_equal("Integral reset on entering neutral", st.integral_error, 0.0, tol=1e-9)


def test_non_neutral_disable() -> None:
    imu = FakeImuReader(heading_deg=0.0)
    comp = make_compensator(imu, kp=1.0, ki=0.5, kd=0.3, deadband_deg=0.0)
    comp.set_target_heading(10.0)
    corr = tick(comp, imu, steering_input=0.5, dt=0.05)  # active steering
    assert_true("Non-neutral returns None", corr is None)


if __name__ == "__main__":
    t0 = time.time()
    run_tests()
    print(f"Elapsed: {time.time() - t0:.3f}s")


