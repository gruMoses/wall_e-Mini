"""RC MANUAL stick expo reaches the motors (2026-09-27).

Expo applies to the RC tank-drive sticks only. The phone/BT teleop bytes
pass through unchanged.
"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.mapping import apply_stick_expo, map_pulse_to_byte_saturated


class FakeMotor:
    def set_tracks(self, left_byte, right_byte):
        pass

    def stop(self):
        pass


class FakeRelay:
    def set_armed(self, armed):
        pass


class FakeShutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


ARMED_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                    last_update_epoch_s=0.0)


def _cfg(expo):
    no_slew = replace(default_config.slew_limiter, enabled=False)
    return replace(default_config, slew_limiter=no_slew,
                   rc_map=replace(default_config.rc_map, stick_expo=expo))


def _first_forward_cmd(expo, ch1, ch2, bt=None):
    with patch("pi_app.control.controller.config", _cfg(expo)):
        ctrl = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(),
                          shutdown_scheduler=FakeShutdown())
        ctrl.process(ARMED_RC, now_epoch_s=0.5)
        rc = replace(ARMED_RC, ch1_us=ch1, ch2_us=ch2)
        cmd, _, _ = ctrl.process(rc, now_epoch_s=1.0, bt_override_bytes=bt)
    return cmd


class TestControllerStickExpo(unittest.TestCase):

    def test_default_is_the_agreed_value(self):
        self.assertEqual(default_config.rc_map.stick_expo, 0.6)

    def test_half_stick_is_gentler_with_expo(self):
        linear = _first_forward_cmd(0.0, 1725, 1725)
        expo = _first_forward_cmd(0.6, 1725, 1725)
        raw = map_pulse_to_byte_saturated(1725, 1950, 1050)
        self.assertEqual(linear.left_byte, raw)
        self.assertEqual(expo.left_byte, apply_stick_expo(raw, 0.6))
        self.assertLess(expo.left_byte, linear.left_byte)
        self.assertEqual(expo.left_byte, expo.right_byte)

    def test_full_stick_is_unchanged(self):
        self.assertEqual(_first_forward_cmd(0.6, 2000, 2000).left_byte,
                         _first_forward_cmd(0.0, 2000, 2000).left_byte)

    def test_phone_teleop_bytes_are_not_expo_shaped(self):
        a = _first_forward_cmd(0.0, 1500, 1500, bt=(190, 190))
        b = _first_forward_cmd(0.6, 1500, 1500, bt=(190, 190))
        self.assertEqual(a.left_byte, b.left_byte)


if __name__ == "__main__":
    unittest.main()
