"""
Velocity-PID re-enable (2026-09-19): scale, gate, clamp, open-loop fallback.

  - The velocity loop's byte→m/s scale is pinned to the VescConfig drivetrain
    kinematics (the same numbers controller.py uses for eRPM→m/s), so the
    commanded target and the measured feedback can never drift apart.
  - RpmPlausibilityGate: "commanded to move but eRPM ≈ 0" for a window trips
    the gate; rpm None never trips it; trivial commands never trip it; it
    holds for hold_s and needs real RPM to recover.
  - Controller wiring: a tripped gate nulls rpm/actual_speed_mps in telemetry
    (open-loop fallback) and surfaces vesc_rpm_plausible / trip count.
  - SpeedLayer: open-loop still works when actual_speed_mps is None, and the
    closed-loop correction is bounded by speed_pid_max_correction_mps.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3]))

from config import FollowMeConfig, VescConfig
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection, PIDController, SpeedLayer
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.rpm_plausibility import (
    RpmPlausibilityConfig,
    RpmPlausibilityGate,
    byte_offset_to_erpm,
    erpm_to_wheel_mps,
    wheel_mps_per_byte,
)


def _arm_rc(now: float | None = None) -> RCInputs:
    return RCInputs(
        ch1_us=1500, ch2_us=1500,
        ch3_us=1750,  # arm channel
        ch4_us=1500, ch5_us=1500,
        last_update_epoch_s=now if now is not None else time.time(),
    )


# ─────────────────────────────────────────────────────────────────────────────
# 1. Scale: one set of kinematics for command and feedback
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeedLoopScale(unittest.TestCase):

    def _vesc_kw(self) -> dict:
        v = VescConfig()
        return dict(
            max_erpm=v.max_erpm, motor_poles=v.motor_poles,
            drive_gear_ratio=v.drive_gear_ratio, wheel_radius_m=v.wheel_radius_m,
        )

    def test_speed_loop_scale_matches_vesc_kinematics(self):
        """FollowMeConfig.speed_loop_mps_per_byte must equal the wheel speed one
        byte produces through max_erpm / poles / gearing / radius."""
        derived = wheel_mps_per_byte(**self._vesc_kw())
        self.assertAlmostEqual(FollowMeConfig().speed_loop_mps_per_byte, derived, places=5)

    def test_full_scale_byte_reproduces_documented_top_speed(self):
        """docs/gearing_memo.md §a: byte 254 → 15000 eRPM → 1.205 m/s."""
        v = self._vesc_kw()
        erpm = byte_offset_to_erpm(128, max_erpm=v["max_erpm"])
        self.assertAlmostEqual(erpm, 15000.0, places=6)
        mps = erpm_to_wheel_mps(erpm, motor_poles=v["motor_poles"],
                                drive_gear_ratio=v["drive_gear_ratio"],
                                wheel_radius_m=v["wheel_radius_m"])
        self.assertAlmostEqual(mps, 1.205, places=2)

    def test_loop_scale_is_wheel_speed_not_ground_speed(self):
        """The GPS-calibrated trail scale (ground speed on gravel) is ~20 % lower
        than the wheel-speed scale; the velocity loop must use the wheel scale."""
        cfg = FollowMeConfig()
        self.assertGreater(cfg.speed_loop_mps_per_byte, cfg.trail_speed_scale_mps_per_byte)
        ratio = cfg.trail_speed_scale_mps_per_byte / cfg.speed_loop_mps_per_byte
        self.assertGreater(ratio, 0.7)
        self.assertLess(ratio, 0.9)

    def test_controller_actual_speed_uses_same_kinematics(self):
        class T:
            left_rpm = 1500
            right_rpm = 1500
            voltage_v = 48.0
            timestamp = 0.0
            left_status_age_s = 0.01
            right_status_age_s = 0.01
            rx_thread_alive = True

        class FakeMotor:
            def set_tracks(self, l, r): pass
            def stop(self): pass
            def get_telemetry(self): return T()

        ctrl = Controller(motor_driver=FakeMotor())
        ctrl._telem_last_poll = 0.0
        _, _, telem = ctrl.process(_arm_rc(), now_epoch_s=time.time())
        v = VescConfig()
        expected = erpm_to_wheel_mps(1500, motor_poles=v.motor_poles,
                                     drive_gear_ratio=v.drive_gear_ratio,
                                     wheel_radius_m=v.wheel_radius_m)
        self.assertAlmostEqual(telem["vesc_actual_speed_mps"], expected, places=6)
        # 1500 eRPM is ~12.8 bytes' worth of command → ~0.12 m/s wheel speed
        # (the bench's 1500 eRPM test point; see tools/vesc_rpm_bench.py).
        self.assertAlmostEqual(expected, 0.1205, delta=0.002)


# ─────────────────────────────────────────────────────────────────────────────
# 2. RpmPlausibilityGate (pure)
# ─────────────────────────────────────────────────────────────────────────────

class TestRpmPlausibilityGate(unittest.TestCase):

    def _gate(self, **kw) -> RpmPlausibilityGate:
        cfg = RpmPlausibilityConfig(min_cmd_bytes=12, min_erpm=150, window_s=0.5, hold_s=2.0)
        for k, v in kw.items():
            setattr(cfg, k, v)
        return RpmPlausibilityGate(cfg)

    def _step(self, g, now, cmd=200, rpm=0):
        return g.update(now, left_cmd_byte=cmd, right_cmd_byte=cmd, left_rpm=rpm, right_rpm=rpm)

    def test_commanded_with_zero_rpm_trips_after_window(self):
        g = self._gate()
        self.assertTrue(self._step(g, 0.00))
        self.assertTrue(self._step(g, 0.20), "inside the window: still plausible (spin-up)")
        self.assertTrue(self._step(g, 0.49))
        self.assertFalse(self._step(g, 0.50), "window elapsed → tripped")
        self.assertTrue(g.tripped)
        self.assertEqual(g.trip_count, 1)
        self.assertEqual(set(g.tripped_motors), {"left", "right"})

    def test_rpm_none_never_trips(self):
        """Missing RPM is the staleness path's job; the gate must stay quiet."""
        g = self._gate()
        for i in range(40):
            self.assertTrue(self._step(g, i * 0.1, cmd=220, rpm=None))
        self.assertFalse(g.tripped)

    def test_trivial_command_with_zero_rpm_never_trips(self):
        g = self._gate()
        for i in range(40):
            self.assertTrue(self._step(g, i * 0.1, cmd=CENTER_OUTPUT_VALUE + 5, rpm=0))
        self.assertFalse(g.tripped)

    def test_plausible_rpm_resets_streak(self):
        g = self._gate()
        self._step(g, 0.0, rpm=0)
        self._step(g, 0.3, rpm=0)
        self._step(g, 0.4, rpm=1400)   # wheel spun up → streak broken
        self.assertTrue(self._step(g, 0.8, rpm=0), "new streak starts at 0.8")
        self.assertTrue(self._step(g, 1.2, rpm=0))
        self.assertFalse(self._step(g, 1.3, rpm=0))

    def test_reverse_command_and_negative_rpm_are_direction_agnostic(self):
        g = self._gate()
        for i in range(6):
            ok = g.update(i * 0.1, left_cmd_byte=40, right_cmd_byte=40, left_rpm=-1500, right_rpm=-1480)
            self.assertTrue(ok)
        self.assertFalse(g.tripped)

    def test_single_motor_trip_invalidates_gate(self):
        g = self._gate()
        for i in range(7):
            ok = g.update(i * 0.1, left_cmd_byte=200, right_cmd_byte=200, left_rpm=1400, right_rpm=0)
        self.assertFalse(ok)
        self.assertEqual(g.tripped_motors, ("right",))

    def test_hold_then_recover_needs_real_rpm(self):
        g = self._gate(hold_s=1.0)
        for i in range(6):
            self._step(g, i * 0.1, rpm=0)
        self.assertTrue(g.tripped)
        # RPM comes back immediately, but the hold has not elapsed.
        self.assertFalse(self._step(g, 0.9, rpm=1400))
        # Hold elapsed but RPM still 0 → stays tripped.
        self.assertFalse(self._step(g, 2.0, rpm=0))
        # Hold elapsed AND real RPM → recovered.
        self.assertTrue(self._step(g, 2.1, rpm=1400))
        self.assertFalse(g.tripped)
        self.assertEqual(g.trip_count, 1)

    def test_disabled_gate_is_always_plausible(self):
        g = self._gate(enabled=False)
        for i in range(20):
            self.assertTrue(self._step(g, i * 0.1, rpm=0))

    def test_reset_clears_trip(self):
        g = self._gate()
        for i in range(7):
            self._step(g, i * 0.1, rpm=0)
        self.assertTrue(g.tripped)
        g.reset()
        self.assertFalse(g.tripped)


# ─────────────────────────────────────────────────────────────────────────────
# 3. Controller wiring: tripped gate → open-loop fallback
# ─────────────────────────────────────────────────────────────────────────────

class _ZeroRpmMotor:
    """VESC that answers every command with eRPM 0 (dead readback / stall)."""

    def __init__(self, rpm: int = 0):
        self.rpm = rpm

    def set_tracks(self, l, r): pass
    def stop(self): pass

    def get_telemetry(self):
        class T:
            pass
        t = T()
        t.left_rpm = self.rpm
        t.right_rpm = self.rpm
        t.voltage_v = 48.0
        t.timestamp = time.monotonic()
        t.left_status_age_s = 0.01
        t.right_status_age_s = 0.01
        t.rx_thread_alive = True
        return t


class TestControllerPlausibilityFallback(unittest.TestCase):

    def _poll(self, ctrl, cmd_byte: int):
        # The gate compares the bytes EMITTED on the previous tick with the RPM
        # read back now; seed the slew state as if that tick had emitted cmd_byte
        # and force the 50 ms poll gate open.
        ctrl._slew_last_left = cmd_byte
        ctrl._slew_last_right = cmd_byte
        ctrl._telem_last_poll = 0.0
        _, _, telem = ctrl.process(_arm_rc(), now_epoch_s=time.time())
        return telem

    def _ctrl(self, motor, window_s=0.0, hold_s=0.0):
        ctrl = Controller(motor_driver=motor)
        ctrl._rpm_gate = RpmPlausibilityGate(RpmPlausibilityConfig(
            min_cmd_bytes=12, min_erpm=150, window_s=window_s, hold_s=hold_s,
        ))
        return ctrl

    def test_zero_rpm_under_command_falls_back_to_open_loop(self):
        ctrl = self._ctrl(_ZeroRpmMotor(rpm=0))
        t1 = self._poll(ctrl, 200)   # streak starts
        self.assertEqual(t1["vesc_left_rpm"], 0)
        self.assertTrue(t1["vesc_rpm_plausible"])
        t2 = self._poll(ctrl, 200)   # window (0 s) elapsed → tripped
        self.assertFalse(t2["vesc_rpm_plausible"])
        self.assertIsNone(t2["vesc_left_rpm"])
        self.assertIsNone(t2["vesc_right_rpm"])
        self.assertIsNone(t2["vesc_actual_speed_mps"])
        self.assertEqual(t2["vesc_rpm_gate_trips"], 1)

    def test_zero_rpm_at_neutral_is_fine(self):
        ctrl = self._ctrl(_ZeroRpmMotor(rpm=0))
        for _ in range(5):
            t = self._poll(ctrl, CENTER_OUTPUT_VALUE)
        self.assertTrue(t["vesc_rpm_plausible"])
        self.assertEqual(t["vesc_left_rpm"], 0)

    def test_real_rpm_under_command_stays_closed_loop(self):
        ctrl = self._ctrl(_ZeroRpmMotor(rpm=1500))
        for _ in range(5):
            t = self._poll(ctrl, 200)
        self.assertTrue(t["vesc_rpm_plausible"])
        self.assertEqual(t["vesc_left_rpm"], 1500)
        self.assertIsNotNone(t["vesc_actual_speed_mps"])

    def test_recovers_when_rpm_returns(self):
        motor = _ZeroRpmMotor(rpm=0)
        ctrl = self._ctrl(motor)
        self._poll(ctrl, 200)
        t = self._poll(ctrl, 200)
        self.assertFalse(t["vesc_rpm_plausible"])
        motor.rpm = 1500
        t = self._poll(ctrl, 200)
        self.assertTrue(t["vesc_rpm_plausible"])
        self.assertEqual(t["vesc_left_rpm"], 1500)

    def test_follow_me_sees_none_not_zero_when_tripped(self):
        """The whole point: the velocity PID must never integrate the bogus 0."""
        fm = FollowMeController(FollowMeConfig())
        ctrl = self._ctrl(_ZeroRpmMotor(rpm=0))
        ctrl._follow_me = fm
        self._poll(ctrl, 200)
        self._poll(ctrl, 200)
        self.assertIsNone(ctrl._actual_speed_mps)
        self.assertIsNone(ctrl._actual_left_rpm)


# ─────────────────────────────────────────────────────────────────────────────
# 4. SpeedLayer: open-loop when None, bounded correction when closed
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeedLayerWithShippedGains(unittest.TestCase):

    def _layer(self) -> tuple[SpeedLayer, FollowMeConfig]:
        cfg = FollowMeConfig()
        pid = PIDController(
            kp=cfg.speed_kp, ki=cfg.speed_ki, kd=cfg.speed_kd,
            integral_limit=cfg.speed_integral_limit,
            output_limit=cfg.speed_pid_max_correction_mps,
        )
        max_speed = float(cfg.max_follow_speed_byte)
        layer = SpeedLayer(
            target_dist_m=cfg.follow_distance_m,
            dead_zone_m=cfg.speed_dead_zone_m,
            speed_gain=max_speed / cfg.max_speed_error_m,
            min_dist_m=cfg.min_distance_m,
            max_speed_byte=max_speed,
            velocity_pid=pid,
            speed_scale_mps_per_byte=cfg.speed_loop_mps_per_byte,
        )
        return layer, cfg

    def test_open_loop_when_actual_speed_is_none(self):
        layer, cfg = self._layer()
        depth = cfg.follow_distance_m + 0.8
        expected = min(cfg.max_follow_speed_byte, 0.8 * cfg.max_follow_speed_byte / cfg.max_speed_error_m)
        out = layer.compute(depth, actual_speed_mps=None, dt=0.05)
        self.assertAlmostEqual(out, expected, places=6)

    def test_open_loop_equals_gains_zeroed_behaviour(self):
        """With telemetry absent the output is byte-identical to the shipped
        (2026-06-11 .. 2026-09-19) gains-zeroed loop."""
        layer, cfg = self._layer()
        zero_pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        legacy = SpeedLayer(
            target_dist_m=cfg.follow_distance_m, dead_zone_m=cfg.speed_dead_zone_m,
            speed_gain=cfg.max_follow_speed_byte / cfg.max_speed_error_m,
            min_dist_m=cfg.min_distance_m, max_speed_byte=cfg.max_follow_speed_byte,
            velocity_pid=zero_pid, speed_scale_mps_per_byte=0.0075,
        )
        for depth in (0.4, 1.0, 1.6, 2.0, 2.5, 3.5, 6.0):
            self.assertAlmostEqual(layer.compute(depth, None, 0.05), legacy.compute(depth, None, 0.05), places=9)

    def test_correction_is_bounded_by_clamp(self):
        layer, cfg = self._layer()
        depth = cfg.follow_distance_m + 1.0
        open_loop = layer.compute(depth, actual_speed_mps=None, dt=0.05)
        max_bytes = cfg.speed_pid_max_correction_mps / cfg.speed_loop_mps_per_byte
        # Wheel reads 0 m/s (huge shortfall): correction must be +clamp, no more.
        boosted = layer.compute(depth, actual_speed_mps=0.0, dt=0.05)
        self.assertGreater(boosted, open_loop)
        self.assertLessEqual(boosted, min(cfg.max_follow_speed_byte, open_loop + max_bytes) + 1e-6)
        # Wheel reads far too fast: correction must be −clamp, no more, never < 0.
        layer2, _ = self._layer()
        slowed = layer2.compute(depth, actual_speed_mps=2.0, dt=0.05)
        self.assertLess(slowed, open_loop)
        self.assertGreaterEqual(slowed, max(0.0, open_loop - max_bytes) - 1e-6)

    def test_zero_error_when_wheel_matches_command(self):
        """Correct scale ⇒ a wheel spinning at exactly the commanded speed gets
        no correction (the old mixed-scale loop had a permanent ~20 % error)."""
        layer, cfg = self._layer()
        depth = cfg.follow_distance_m + 1.0
        open_loop = layer.compute(depth, actual_speed_mps=None, dt=0.05)
        matched = layer.compute(depth, actual_speed_mps=open_loop * cfg.speed_loop_mps_per_byte, dt=0.05)
        self.assertAlmostEqual(matched, open_loop, places=6)

    def test_integral_cannot_wind_past_clamp(self):
        layer, cfg = self._layer()
        depth = cfg.follow_distance_m + 1.0
        open_loop = layer.compute(depth, actual_speed_mps=None, dt=0.05)
        max_bytes = cfg.speed_pid_max_correction_mps / cfg.speed_loop_mps_per_byte
        out = open_loop
        for _ in range(600):  # 30 s of "wheel never moves"
            out = layer.compute(depth, actual_speed_mps=0.0, dt=0.05)
        self.assertLessEqual(out, min(cfg.max_follow_speed_byte, open_loop + max_bytes) + 1e-6)


if __name__ == "__main__":
    unittest.main()
