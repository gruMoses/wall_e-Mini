import unittest
from dataclasses import replace
from unittest.mock import patch

from pi_app.control.controller import Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler, RC_STALE_TIMEOUT_S
from pi_app.control.mapping import MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, MAX_OUTPUT, MIN_OUTPUT, CENTER_OUTPUT_VALUE
from pi_app.control.safety import SafetyEvent
from config import config as default_config


class FakeMotor(MotorDriver):
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        self.commands.append((left_byte, right_byte))

    def stop(self) -> None:
        self.stops += 1


class FakeRelay(ArmRelay):
    def __init__(self):
        self.states = []

    def set_armed(self, armed: bool) -> None:
        self.states.append(armed)


class FakeShutdown(ShutdownScheduler):
    def __init__(self):
        self.scheduled = []

    def schedule_shutdown(self, delay_seconds: float) -> None:
        self.scheduled.append(delay_seconds)


class TestController(unittest.TestCase):
    def test_process_disarmed_neutral(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, events, _ = c.process(rc, now_epoch_s=0.0)
        self.assertEqual(cmd.left_byte, 126)
        self.assertEqual(cmd.right_byte, 126)
        self.assertFalse(cmd.is_armed)
        self.assertEqual(motor.stops, 1)
        self.assertEqual(events, [])

    def test_process_armed_tracks(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)
        # Arm
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, events, _ = c.process(rc, now_epoch_s=0.3)
        self.assertTrue(cmd.is_armed)
        # Send some non-neutral values
        rc = RCInputs(
            ch1_us=MAX_PULSE_WIDTH_US,
            ch2_us=MIN_PULSE_WIDTH_US,
            ch3_us=1900,
            ch4_us=1000,
            ch5_us=1000,
            last_update_epoch_s=0.0,
        )
        cmd, events, _ = c.process(rc, now_epoch_s=0.6)
        self.assertEqual(motor.commands[-1], (MAX_OUTPUT, MIN_OUTPUT))
        self.assertTrue(cmd.is_armed)

    def test_emergency_triggers_shutdown_and_stop(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)
        # Arm first
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, events, _ = c.process(rc, now_epoch_s=0.5)
        self.assertTrue(cmd.is_armed)
        # Trigger emergency rising edge
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1500, ch4_us=1000, ch5_us=1900, last_update_epoch_s=0.0)
        cmd, events, _ = c.process(rc, now_epoch_s=0.6)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertFalse(cmd.is_armed)
        self.assertGreaterEqual(motor.stops, 1)
        self.assertEqual(shutdown.scheduled, [5.0])

    def test_init_requires_positive_update_rate(self):
        bad_imu = replace(default_config.imu_steering, update_rate_hz=0.0)
        bad_config = replace(default_config, imu_steering=bad_imu)
        with patch('pi_app.control.controller.config', bad_config):
            with self.assertRaises(ValueError):
                Controller()

    def test_reset_imu_timestamp_helper(self):
        with patch("pi_app.control.controller.time.monotonic", return_value=50.0):
            c = Controller()
        # On init, the monotonic clock value should be captured
        self.assertEqual(c._last_imu_update, 50.0)
        # Reset the timestamp via helper and ensure the internal value changes
        c._reset_imu_timestamp(123.0)
        self.assertEqual(c._last_imu_update, 123.0)


class TestRCStaleness(unittest.TestCase):
    def test_stale_rc_forces_disarm_and_neutral(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)

        now = 100.0
        # Arm
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=now)
        cmd, events, _ = c.process(rc, now_epoch_s=now + 0.3)
        self.assertTrue(cmd.is_armed)

        # Now time advances but last_update_epoch_s stays stale
        stale_now = now + 0.3 + RC_STALE_TIMEOUT_S + 0.5
        rc_stale = RCInputs(ch1_us=2100, ch2_us=2100, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                             last_update_epoch_s=now)
        cmd, events, telem = c.process(rc_stale, now_epoch_s=stale_now)
        self.assertFalse(cmd.is_armed)
        self.assertEqual(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertEqual(cmd.right_byte, CENTER_OUTPUT_VALUE)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertTrue(telem.get("rc_stale"))

    def test_fresh_rc_is_not_stale(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)

        now = 100.0
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=now)
        cmd, events, telem = c.process(rc, now_epoch_s=now + 0.3)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.RC_STALE, events)
        self.assertFalse(telem.get("rc_stale", False))


if __name__ == "__main__":
    unittest.main()


