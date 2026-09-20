"""Speed-loop anti-windup, persistence grace, and direct-pursuit turn scale."""

import unittest
from unittest.mock import patch

from config import FollowMeConfig
from pi_app.control.follow_me import (
    FollowMeController,
    PersonDetection,
    PIDController,
    SpeedLayer,
)


def _bbox_for_x_norm(x_norm: float, width: float = 0.10) -> tuple[float, float, float, float]:
    """Bbox whose centre maps to ``x_norm`` via DetectionFilter ((cx-0.5)*2)."""
    cx = (x_norm / 2.0) + 0.5
    return (cx - width / 2.0, 0.0, cx + width / 2.0, 0.8)


def _person_at_x_norm(x_norm: float, z_m: float = 4.0) -> PersonDetection:
    bbox = _bbox_for_x_norm(x_norm)
    return PersonDetection(
        x_m=x_norm, z_m=z_m, confidence=0.9, bbox=bbox, track_id=1,
    )


def _speed_layer() -> SpeedLayer:
    pid = PIDController(
        kp=0.6, ki=0.15, kd=0.0,
        integral_limit=1.0, output_limit=0.2,
    )
    return SpeedLayer(
        target_dist_m=1.5,
        dead_zone_m=0.2,
        speed_gain=110.0 / 1.5,
        min_dist_m=0.5,
        max_speed_byte=110.0,
        velocity_pid=pid,
        speed_scale_mps_per_byte=0.009416,
    )


class TestSpeedLayerEmittedForwardByte(unittest.TestCase):
    """Item 1: PID target follows the previous tick's post-limit forward byte."""

    def test_none_emitted_winds_toward_clamp(self):
        layer = _speed_layer()
        # depth 3.0 → error 1.5 → open_loop 110. actual 0.2 m/s is far below
        # the 1.04 m/s target, so the loop saturates at the 0.2 m/s clamp
        # (current behaviour when emitted_forward_byte is None).
        for _ in range(60):  # 3 s at 20 Hz
            layer.compute(
                depth_m=3.0, actual_speed_mps=0.2, dt=0.05,
                emitted_forward_byte=None,
            )
        self.assertAlmostEqual(layer.last_open_loop_byte, 110.0, places=3)
        self.assertAlmostEqual(layer.last_target_byte, 110.0, places=3)
        self.assertAlmostEqual(abs(layer.last_corr_mps), 0.2, places=3)
        self.assertTrue(layer.last_closed)

    def test_limited_emitted_does_not_wind_up(self):
        layer = _speed_layer()
        for _ in range(60):
            layer.compute(
                depth_m=3.0, actual_speed_mps=0.19, dt=0.05,
                emitted_forward_byte=20.0,
            )
        self.assertLess(abs(layer.last_corr_mps), 0.02)
        last_corr = layer.last_corr_byte if layer.last_corr_byte is not None else 0.0
        self.assertAlmostEqual(
            layer.last_target_byte, 20.0 - last_corr, delta=0.5,
        )
        self.assertLess(layer.last_target_byte, 30.0)

    def test_emitted_zero_resets_pid(self):
        layer = _speed_layer()
        for _ in range(20):
            layer.compute(
                depth_m=3.0, actual_speed_mps=0.2, dt=0.05,
                emitted_forward_byte=None,
            )
        self.assertGreater(abs(layer.last_corr_mps), 0.05)
        layer.compute(
            depth_m=3.0, actual_speed_mps=0.2, dt=0.05,
            emitted_forward_byte=0.0,
        )
        self.assertEqual(layer.last_corr_mps, 0.0)
        self.assertEqual(layer.last_corr_byte, 0.0)
        self.assertEqual(layer.last_target_byte, 0.0)
        self.assertEqual(layer._velocity_pid._integral, 0.0)


class TestSteerHoldGrace(unittest.TestCase):
    """Item 2: hold last fresh speed/steer through a short dropout, then decay."""

    def test_speed_and_steer_held_for_grace_then_decay(self):
        fm = FollowMeController(FollowMeConfig(
            trail_follow_enabled=False,
            steer_hold_grace_s=0.30,
            steer_hold_decay_s=1.0,
            steer_hold_decay_speed_floor=0.5,
            target_persistence_s=2.0,
        ))
        person = _person_at_x_norm(0.2, z_m=4.0)
        t0 = 1000.0
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = t0
            fm.compute([person])
            mt.monotonic.return_value = t0 + 0.5
            fm.compute([person])
            speed0 = fm._last_speed_offset
            steer0 = fm._last_steer_offset
            self.assertGreater(speed0, 0.0)
            self.assertNotEqual(steer0, 0.0)

            mt.monotonic.return_value = t0 + 0.5 + 0.25
            fm.compute([])
            self.assertAlmostEqual(fm._last_speed_offset, speed0, places=3)
            self.assertAlmostEqual(fm._last_steer_offset, steer0, places=3)
            self.assertTrue(fm._steer_hold_active)

            mt.monotonic.return_value = t0 + 0.5 + 0.9
            fm.compute([])
            self.assertLess(fm._last_speed_offset, speed0)
            self.assertLess(abs(fm._last_steer_offset), abs(steer0))


class TestDirectTurnSpeedScale(unittest.TestCase):
    """Item 3: reduce forward speed in direct pursuit when the person is off-centre."""

    def _make(self) -> FollowMeController:
        return FollowMeController(FollowMeConfig(
            trail_follow_enabled=False,
            detect_edge_margin=0.0,
            direct_turn_speed_knee_norm=0.30,
            direct_turn_speed_min_scale=0.35,
        ))

    def _run_with_x_norm(self, fm: FollowMeController, x_norm: float):
        captured = []
        orig = fm._safety.apply

        def spy(speed, steer, now):
            captured.append(speed)
            return orig(speed, steer, now)

        fm._safety.apply = spy
        speed_out = []
        orig_speed = fm._speed.compute

        def speed_spy(*args, **kwargs):
            out = orig_speed(*args, **kwargs)
            speed_out.append(out)
            return out

        fm._speed.compute = speed_spy
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = 2000.0
            fm.compute([_person_at_x_norm(x_norm, z_m=4.0)])
        return speed_out[0], captured[0], fm._turn_speed_scale

    def test_below_knee_scale_is_one(self):
        open_loop, to_safety, scale = self._run_with_x_norm(self._make(), 0.1)
        self.assertAlmostEqual(scale, 1.0, places=3)
        self.assertAlmostEqual(to_safety, open_loop, places=3)

    def test_mid_scale(self):
        open_loop, to_safety, scale = self._run_with_x_norm(self._make(), 0.65)
        self.assertAlmostEqual(scale, 0.675, places=3)
        self.assertAlmostEqual(to_safety, open_loop * 0.675, places=3)

    def test_full_edge_min_scale(self):
        open_loop, to_safety, scale = self._run_with_x_norm(self._make(), 1.0)
        self.assertAlmostEqual(scale, 0.35, places=3)
        self.assertAlmostEqual(to_safety, open_loop * 0.35, places=3)


class TestTelemetryAcceptsNewFields(unittest.TestCase):
    def test_update_telemetry_accepts_emitted_forward_byte_and_status_keys(self):
        fm = FollowMeController(FollowMeConfig())
        fm.update_telemetry(
            left_rpm=None, right_rpm=None, actual_speed_mps=None,
            emitted_forward_byte=20.0,
        )
        self.assertEqual(fm._emitted_forward_byte, 20.0)
        status = fm.get_status()
        self.assertIn("target_byte", status["speed_loop"])
        self.assertIn("turn_speed_scale", status)


if __name__ == "__main__":
    unittest.main()
