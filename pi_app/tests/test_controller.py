import unittest

from pi_app.control.controller import Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler
from pi_app.control.mapping import MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US
from pi_app.control.safety import SafetyEvent


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
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, events = c.process(rc, now_epoch_s=0.0)
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
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, events = c.process(rc, now_epoch_s=0.3)
        self.assertTrue(cmd.is_armed)
        # Send some non-neutral values
        rc = RCInputs(
            ch1_us=MAX_PULSE_WIDTH_US,
            ch2_us=MIN_PULSE_WIDTH_US,
            ch3_us=1900,
            ch5_us=1000,
            last_update_epoch_s=0.0,
        )
        cmd, events = c.process(rc, now_epoch_s=0.6)
        self.assertEqual(motor.commands[-1], (255, 0))
        self.assertTrue(cmd.is_armed)

    def test_emergency_triggers_shutdown_and_stop(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)
        # Arm first
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch5_us=1000, last_update_epoch_s=0.0)
        cmd, events = c.process(rc, now_epoch_s=0.5)
        self.assertTrue(cmd.is_armed)
        # Trigger emergency rising edge
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1500, ch5_us=1900, last_update_epoch_s=0.0)
        cmd, events = c.process(rc, now_epoch_s=0.6)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertFalse(cmd.is_armed)
        self.assertGreaterEqual(motor.stops, 1)
        self.assertEqual(shutdown.scheduled, [5.0])


if __name__ == "__main__":
    unittest.main()


