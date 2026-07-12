import unittest
from dataclasses import replace
from unittest.mock import MagicMock, patch

from pi_app.control.controller import Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler, RC_STALE_TIMEOUT_S
from pi_app.control.mapping import MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, MAX_OUTPUT, MIN_OUTPUT, CENTER_OUTPUT_VALUE
from pi_app.control.safety import SafetyEvent, SafetyParams
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

    def test_stale_rc_still_refreshes_imu_heading(self):
        c = Controller(
            motor_driver=FakeMotor(),
            arm_relay=FakeRelay(),
            shutdown_scheduler=FakeShutdown(),
        )
        imu = MagicMock()
        c._imu_compensator = imu
        rc = RCInputs(
            ch1_us=1500,
            ch2_us=1500,
            ch3_us=1000,
            ch4_us=1000,
            ch5_us=1000,
            last_update_epoch_s=100.0,
        )

        cmd, events, telem = c.process(
            rc,
            now_epoch_s=100.0 + RC_STALE_TIMEOUT_S + 0.5,
        )

        imu.get_heading_deg.assert_called_once_with()
        self.assertFalse(cmd.is_armed)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertTrue(telem["rc_stale"])


class TestSlewLimiter(unittest.TestCase):
    def _make_controller(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        params = SafetyParams(debounce_seconds=0.0)
        c = Controller(
            motor_driver=motor,
            arm_relay=relay,
            shutdown_scheduler=shutdown,
            safety_params=params,
        )
        return c, motor

    def test_manual_asymmetric_accel_decel(self):
        slew_cfg = replace(
            default_config.slew_limiter,
            enabled=True,
            manual_accel_bps=100.0,
            manual_decel_bps=300.0,
            snap_first_command=False,
        )
        test_cfg = replace(default_config, slew_limiter=slew_cfg)
        arm_rc = RCInputs(
            ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0
        )
        with patch("pi_app.control.controller.config", test_cfg):
            with patch("pi_app.control.controller.time.monotonic", side_effect=[0.0, 0.0, 0.1, 0.2, 0.3]):
                c, motor = self._make_controller()
                c.process(arm_rc, now_epoch_s=0.1)
                cmd1, _, _ = c.process(arm_rc, now_epoch_s=0.2, bt_override_bytes=(220, 220))
                cmd2, _, _ = c.process(arm_rc, now_epoch_s=0.3, bt_override_bytes=(126, 126))

        # 100 bps * 0.1s = 10 byte accel step from neutral
        self.assertEqual(cmd1.left_byte, 136)
        self.assertEqual(cmd1.right_byte, 136)
        # 300 bps * 0.1s = 30 byte decel step toward neutral
        self.assertEqual(cmd2.left_byte, 126)
        self.assertEqual(cmd2.right_byte, 126)
        self.assertEqual(motor.commands[-1], (126, 126))

    def test_mode_aware_profile_follow_me_slower_than_manual(self):
        slew_cfg = replace(
            default_config.slew_limiter,
            enabled=True,
            manual_accel_bps=200.0,
            follow_me_accel_bps=50.0,
            manual_decel_bps=200.0,
            follow_me_decel_bps=200.0,
            snap_first_command=False,
        )
        test_cfg = replace(default_config, slew_limiter=slew_cfg)
        arm_rc = RCInputs(
            ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_epoch_s=0.0
        )
        with patch("pi_app.control.controller.config", test_cfg):
            with patch("pi_app.control.controller.time.monotonic", side_effect=[0.0, 0.0, 0.1, 0.2, 0.3]):
                c, _ = self._make_controller()
                c.process(arm_rc, now_epoch_s=0.1)
                manual_cmd, _, _ = c.process(arm_rc, now_epoch_s=0.2, bt_override_bytes=(200, 200))
                c._mode = "FOLLOW_ME"
                follow_cmd, _, _ = c.process(arm_rc, now_epoch_s=0.3, bt_override_bytes=(200, 200))

        self.assertEqual(manual_cmd.left_byte, 146)  # 200 bps * 0.1s = +20
        # FOLLOW_ME accel is lower; second step advances only 5 bytes.
        self.assertEqual(follow_cmd.left_byte, 151)
        self.assertEqual(follow_cmd.right_byte, 151)


class TestControllerPackLowLatch(unittest.TestCase):
    """The controller must OBSERVE the VESC pack-low latch, not rely on the
    driver's set_tracks() enforcement alone. Without controller-level handling
    the slew state (_slew_last_left/right) keeps advancing toward the commanded
    value while latched, so the instant the latch clears (~41V) the fully-
    ramped command is emitted = full-speed lurch. While latched the controller
    must force neutral + _motor.stop() + _reset_slew_state() every loop
    (mirroring the charger_inhibit block), so release ramps from neutral via
    the normal slew."""

    class _FakeVescTelemetry:
        left_rpm = 100
        right_rpm = 100

        def __init__(self, latched: bool):
            self.pack_low_latched = latched

    class _FakeVescMotor(FakeMotor):
        """FakeMotor that reports a controllable pack-low latch via telemetry."""

        def __init__(self):
            super().__init__()
            self.pack_low_latched = False

        def get_telemetry(self):
            return TestControllerPackLowLatch._FakeVescTelemetry(self.pack_low_latched)

    def test_latched_forces_neutral_pins_slew_and_release_ramps_from_neutral(self):
        slew_cfg = replace(
            default_config.slew_limiter,
            enabled=True,
            manual_accel_bps=100.0,
            manual_decel_bps=300.0,
            snap_first_command=False,
        )
        test_cfg = replace(default_config, slew_limiter=slew_cfg)
        params = SafetyParams(debounce_seconds=0.0)
        arm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000,
                          ch5_us=1000, last_update_epoch_s=0.0)
        motor = self._FakeVescMotor()
        # Controlled virtual clock (robust to however many monotonic() calls
        # each tick makes, unlike a fixed side_effect list).
        fake_now = {"t": 0.0}
        with patch("pi_app.control.controller.config", test_cfg):
            with patch("pi_app.control.controller.time.monotonic",
                       side_effect=lambda: fake_now["t"]):
                c = Controller(motor_driver=motor, arm_relay=FakeRelay(),
                               shutdown_scheduler=FakeShutdown(),
                               safety_params=params)
                motor.pack_low_latched = True
                fake_now["t"] = 1.0
                c.process(arm_rc, now_epoch_s=0.1)  # arm; telem poll engages latch

                # Two latched ticks with a full-forward web/BT command. Without
                # the controller-level block the slew state would fully ramp to
                # 220 across these ticks (1.0s + 0.5s of dt at 100 bps).
                fake_now["t"] = 2.0
                cmd_l1, _, _ = c.process(
                    arm_rc, now_epoch_s=0.2, bt_override_bytes=(220, 220))
                fake_now["t"] = 2.5
                cmd_l2, _, telem_l2 = c.process(
                    arm_rc, now_epoch_s=0.3, bt_override_bytes=(220, 220))

                # Latched: neutral output, motors stopped, slew pinned neutral.
                self.assertEqual((cmd_l1.left_byte, cmd_l1.right_byte),
                                 (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
                self.assertEqual((cmd_l2.left_byte, cmd_l2.right_byte),
                                 (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
                self.assertGreaterEqual(motor.stops, 2)
                self.assertEqual(c._slew_last_left, CENTER_OUTPUT_VALUE)
                self.assertEqual(c._slew_last_right, CENTER_OUTPUT_VALUE)
                self.assertTrue(telem_l2.get("vesc_pack_low_latched"))
                self.assertEqual(telem_l2.get("slew_out_left"), CENTER_OUTPUT_VALUE)

                # Release the latch; the next telemetry poll clears the flag.
                motor.pack_low_latched = False
                fake_now["t"] = 2.6
                cmd_r, _, telem_r = c.process(
                    arm_rc, now_epoch_s=0.4, bt_override_bytes=(220, 220))

        self.assertFalse(telem_r.get("vesc_pack_low_latched"))
        # Ramp from neutral: dt = 2.6 - 2.5 = 0.1s at 100 bps = +10 bytes —
        # NOT an instant jump to the commanded 220.
        self.assertEqual((cmd_r.left_byte, cmd_r.right_byte),
                         (CENTER_OUTPUT_VALUE + 10, CENTER_OUTPUT_VALUE + 10))

    def test_latched_telemetry_key_present_and_false_by_default(self):
        """vesc_pack_low_latched reaches the telemetry dict every tick (same
        visibility contract as charger_inhibit)."""
        c = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(),
                       shutdown_scheduler=FakeShutdown())
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000,
                      ch5_us=1000, last_update_epoch_s=0.0)
        _, _, telem = c.process(rc, now_epoch_s=0.3)
        self.assertIn("vesc_pack_low_latched", telem)
        self.assertFalse(telem["vesc_pack_low_latched"])


class TestFollowMeNoTargetRegression(unittest.TestCase):
    """Bug #2: flipping ch4 (follow-me) while armed with no person in frame
    must NOT raise UnboundLocalError on the telemetry dict."""

    def test_follow_me_entered_no_target_does_not_raise(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown)
        # Arm with ch4 low (no person detections fed → no target present)
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_epoch_s=100.0)
        cmd, _, _ = c.process(rc, now_epoch_s=100.5)
        self.assertTrue(cmd.is_armed)
        # ch4 rising edge while armed, still no target. Previously raised
        # UnboundLocalError because `telemetry` was referenced before assignment.
        rc2 = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1900, ch5_us=1000,
                       last_update_epoch_s=100.0)
        cmd2, events, telem = c.process(rc2, now_epoch_s=100.6)  # must not raise
        self.assertIn(SafetyEvent.FOLLOW_ME_ENTERED, events)
        self.assertEqual(telem.get("follow_me_activation_blocked"), "no_target")
        self.assertEqual(telem.get("mode"), "MANUAL")


class TestCalibrationSafety(unittest.TestCase):
    """Bug #1: safety (e-stop / disarm) must run every tick, including while in
    calibration mode where the wizard drives the motors directly."""

    def _armed_calibration_controller(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown,
                       safety_params=SafetyParams(debounce_seconds=0.0))
        # Arm
        arm = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_epoch_s=100.0)
        c.process(arm, now_epoch_s=100.0)
        self.assertTrue(c.is_armed)
        c.enter_calibration_mode()
        return c, motor, relay, shutdown

    def test_estop_during_calibration_disarms_stops_and_schedules_shutdown(self):
        c, motor, relay, shutdown = self._armed_calibration_controller()
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1900,
                      last_update_epoch_s=100.0)
        cmd, events, telem = c.process(rc, now_epoch_s=100.1)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertFalse(c.is_armed)
        self.assertFalse(cmd.is_armed)
        self.assertTrue(cmd.emergency_active)
        self.assertGreaterEqual(motor.stops, 1)
        self.assertEqual(shutdown.scheduled, [5.0])
        self.assertTrue(telem.get("calibration"))

    def test_disarm_during_calibration_stops_motor(self):
        c, motor, relay, shutdown = self._armed_calibration_controller()
        stops_before = motor.stops
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000,
                      last_update_epoch_s=100.0)  # ch3 low = disarm
        cmd, events, telem = c.process(rc, now_epoch_s=100.1)
        self.assertFalse(c.is_armed)
        self.assertGreater(motor.stops, stops_before)
        self.assertEqual(telem.get("mode"), "CALIBRATING")

    def test_armed_calm_calibration_does_not_stop_motor(self):
        # While safely armed, process() must not fight the wizard by stopping.
        c, motor, relay, shutdown = self._armed_calibration_controller()
        stops_before = motor.stops
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_epoch_s=100.0)
        cmd, events, telem = c.process(rc, now_epoch_s=100.1)
        self.assertTrue(c.is_armed)
        self.assertEqual(motor.stops, stops_before)
        self.assertTrue(telem.get("calibration"))


if __name__ == "__main__":
    unittest.main()


