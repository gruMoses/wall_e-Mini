"""RC MANUAL stick expo reaches the motors, and nothing else (2026-09-27).

Expo applies to the RC tank-drive sticks only (ratio-preserving,
mapping.apply_stick_expo_pair). The phone/BT teleop bytes pass through
unchanged. Steering intent (heading-hold neutral test, straight latch) and
the twitch stick band read the LINEAR stick bytes: review found that with
expo-shaped bytes the hold read gentle turns as straight and fought them.
"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.mapping import apply_stick_expo_pair, map_pulse_to_byte_saturated


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


def _first_cmd(expo, ch1, ch2, bt=None):
    with patch("pi_app.control.controller.config", _cfg(expo)):
        ctrl = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(),
                          shutdown_scheduler=FakeShutdown())
        ctrl.process(ARMED_RC, now_epoch_s=0.5)
        rc = replace(ARMED_RC, ch1_us=ch1, ch2_us=ch2)
        cmd, _, telem = ctrl.process(rc, now_epoch_s=1.0, bt_override_bytes=bt)
    return cmd, telem


def _lin(us):
    return map_pulse_to_byte_saturated(us, 1950, 1050)


class TestControllerStickExpo(unittest.TestCase):

    def test_default_is_the_agreed_value(self):
        self.assertEqual(default_config.rc_map.stick_expo, 0.6)

    def test_half_stick_is_gentler_with_expo(self):
        linear, _ = _first_cmd(0.0, 1725, 1725)
        expo, _ = _first_cmd(0.6, 1725, 1725)
        self.assertEqual(linear.left_byte, _lin(1725))
        self.assertEqual((expo.left_byte, expo.right_byte),
                         apply_stick_expo_pair(_lin(1725), _lin(1725), 0.6))
        self.assertLess(expo.left_byte, linear.left_byte)

    def test_arc_keeps_its_ratio(self):
        cmd, _ = _first_cmd(0.6, 1800, 1650)
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         apply_stick_expo_pair(_lin(1800), _lin(1650), 0.6))

    def test_full_stick_is_unchanged(self):
        self.assertEqual(_first_cmd(0.6, 2000, 2000)[0].left_byte,
                         _first_cmd(0.0, 2000, 2000)[0].left_byte)

    def test_steering_intent_is_the_same_with_and_without_expo(self):
        # Gentle turns, arcs, near-full-stick differences and a pivot: the
        # heading hold and the straight latch must classify each exactly as
        # with linear sticks.
        for ch1, ch2 in ((1580, 1500), (1650, 1500), (1660, 1580), (1700, 1600),
                         (1900, 1850), (1900, 1800), (1610, 1390), (1725, 1725)):
            _, t0 = _first_cmd(0.0, ch1, ch2)
            _, t6 = _first_cmd(0.6, ch1, ch2)
            self.assertAlmostEqual(t6["steering_input"], t0["steering_input"], places=9,
                                   msg="steering_input changed with expo at %d/%d" % (ch1, ch2))
            self.assertEqual(t6.get("straight_intent"), t0.get("straight_intent"))

    def test_phone_teleop_bytes_are_not_expo_shaped(self):
        a, _ = _first_cmd(0.0, 1500, 1500, bt=(190, 190))
        b, _ = _first_cmd(0.6, 1500, 1500, bt=(190, 190))
        self.assertEqual((a.left_byte, a.right_byte), (b.left_byte, b.right_byte))


if __name__ == "__main__":
    unittest.main()
