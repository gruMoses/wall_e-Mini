import time
import unittest
from dataclasses import replace
from unittest.mock import MagicMock, patch

from pi_app.control.controller import Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler, RC_STALE_TIMEOUT_S
from pi_app.control.mapping import MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, MAX_OUTPUT, MIN_OUTPUT, CENTER_OUTPUT_VALUE
from pi_app.control.safety import SafetyEvent, SafetyParams
from pi_app.hardware.rtk_gps import GpsReading
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
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0)
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
        # One-tick arm as setup (arm timing is in test_rc_arm_channel_failsafe).
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown,
                       safety_params=SafetyParams(arm_high_persist_s=0.0))
        # Arm
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0)
        cmd, events, _ = c.process(rc, now_epoch_s=0.3)
        self.assertTrue(cmd.is_armed)
        # Send some non-neutral values
        rc = RCInputs(
            ch1_us=MAX_PULSE_WIDTH_US,
            ch2_us=MIN_PULSE_WIDTH_US,
            ch3_us=1900,
            ch4_us=1000,
            ch5_us=1000,
            last_update_mono_s=0.0,
        )
        cmd, events, _ = c.process(rc, now_epoch_s=0.6)
        self.assertEqual(motor.commands[-1], (MAX_OUTPUT, MIN_OUTPUT))
        self.assertTrue(cmd.is_armed)

    def test_emergency_triggers_shutdown_and_stop(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        # One-tick arm as setup (arm timing is in test_rc_arm_channel_failsafe).
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown,
                       safety_params=SafetyParams(arm_high_persist_s=0.0))
        # Arm first
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0)
        cmd, events, _ = c.process(rc, now_epoch_s=0.5)
        self.assertTrue(cmd.is_armed)
        # Trigger emergency rising edge
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1500, ch4_us=1000, ch5_us=1900, last_update_mono_s=0.0)
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


class TestControllerTelemetryAdditions(unittest.TestCase):
    """2026-09-19 logging-audit additions: gps_age_s and imu 'saturated'."""

    def test_gps_age_s_none_without_reading(self):
        c = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown())
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0)
        _, _, telem = c.process(rc, now_epoch_s=0.0)
        self.assertIsNone(telem.get("gps_age_s"))

    def test_gps_age_s_reflects_monotonic_delta(self):
        c = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown())
        t0 = time.monotonic()
        reading = GpsReading(
            latitude=1.0, longitude=2.0, altitude_m=0.0, fix_quality=4,
            satellites_used=8, hdop=1.0, diff_age_s=0.0, station_id=0,
            timestamp=t0,
        )
        c.set_gps_reading(reading)
        time.sleep(0.05)
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0)
        _, _, telem = c.process(rc, now_epoch_s=0.0)
        age = telem.get("gps_age_s")
        self.assertIsNotNone(age)
        self.assertGreaterEqual(age, 0.04)
        self.assertLess(age, 1.0)

    def test_get_imu_status_carries_saturated_key(self):
        from pi_app.control.imu_steering import ImuSteeringCompensator, ImuSteeringConfig
        # Controller() with no explicit imu_compensator wires up none
        # (_imu_compensator stays None, get_imu_status() returns None) --
        # inject a real compensator (no imu_reader; no hardware needed) to
        # exercise the get_imu_status() -> ImuSteeringState.saturated path.
        comp = ImuSteeringCompensator(ImuSteeringConfig(), imu_reader=None)
        c = Controller(
            motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
            imu_compensator=comp,
        )
        status = c.get_imu_status()
        self.assertIsNotNone(status)
        self.assertIn("saturated", status)
        self.assertFalse(status["saturated"])


class TestRCStaleness(unittest.TestCase):
    """RC-stale watchdog on the monotonic clock (2026-11-10).

    The RC reader stamps each frame with time.monotonic() and process()
    compares it with time.monotonic(). The wall clock (now_epoch_s) must not
    move the RC age: this Pi's clock steps at boot (RTC reset, then NTP).
    """

    def setUp(self):
        self.clock = {"t": 1000.0}
        self._mono = patch(
            "pi_app.control.controller.time.monotonic",
            side_effect=lambda: self.clock["t"],
        )
        self._mono.start()
        self.addCleanup(self._mono.stop)

    def _armed(self):
        motor = FakeMotor()
        # One-tick arm as setup (arm timing is in test_rc_arm_channel_failsafe).
        c = Controller(motor_driver=motor, arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
                       safety_params=SafetyParams(arm_high_persist_s=0.0))
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_mono_s=self.clock["t"])
        cmd, _, _ = c.process(rc, now_epoch_s=5000.0)
        self.assertTrue(cmd.is_armed)
        return c, motor

    def _rc(self, stamp, ch1=2100, ch2=2100):
        return RCInputs(ch1_us=ch1, ch2_us=ch2, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                        last_update_mono_s=stamp)

    def _assert_stale(self, cmd, events, telem):
        self.assertFalse(cmd.is_armed)
        self.assertEqual(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertEqual(cmd.right_byte, CENTER_OUTPUT_VALUE)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertTrue(telem.get("rc_stale"))

    def test_stale_rc_forces_disarm_and_neutral(self):
        c, motor = self._armed()
        stamp = self.clock["t"]
        self.clock["t"] += RC_STALE_TIMEOUT_S + 0.5
        cmd, events, telem = c.process(self._rc(stamp), now_epoch_s=5000.0 + RC_STALE_TIMEOUT_S + 0.5)
        self._assert_stale(cmd, events, telem)
        self.assertAlmostEqual(telem["rc_age_s"], RC_STALE_TIMEOUT_S + 0.5)

    def test_fresh_rc_is_not_stale(self):
        c, motor = self._armed()
        self.clock["t"] += 0.3
        cmd, events, telem = c.process(self._rc(self.clock["t"] - 0.3, 1500, 1500), now_epoch_s=5000.3)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.RC_STALE, events)
        self.assertFalse(telem.get("rc_stale", False))

    def test_wall_clock_step_back_does_not_freshen_a_dead_link(self):
        # Old code: rc_age = epoch_now - stamp, so a backward NTP step made
        # a dead link look fresh. The wall clock now plays no part.
        c, motor = self._armed()
        stamp = self.clock["t"]
        self.clock["t"] += 5.0  # 5 s with no frame
        # The wall clock stepped back so that it reads only 0.2 s past the
        # stamp: the epoch arithmetic would call this link fresh.
        cmd, events, telem = c.process(self._rc(stamp), now_epoch_s=stamp + 0.2)
        self._assert_stale(cmd, events, telem)

    def test_wall_clock_step_forward_does_not_stale_a_live_link(self):
        c, motor = self._armed()
        self.clock["t"] += 0.05
        cmd, events, telem = c.process(self._rc(self.clock["t"], 1500, 1500), now_epoch_s=self.clock["t"] + 3600.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.RC_STALE, events)

    def test_negative_age_is_stale(self):
        # A stamp in the future of this clock cannot be a real frame.
        c, motor = self._armed()
        cmd, events, telem = c.process(self._rc(self.clock["t"] + 2.0), now_epoch_s=self.clock["t"])
        self._assert_stale(cmd, events, telem)

    def test_non_finite_stamp_is_stale(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(stamp=bad):
                c, motor = self._armed()
                cmd, events, telem = c.process(self._rc(bad), now_epoch_s=5000.0)
                self._assert_stale(cmd, events, telem)
                self.assertIsNone(telem["rc_age_s"])

    def test_no_frame_yet_is_not_stale_and_cannot_arm(self):
        # 0.0 is the reader's "no frame yet" default (sticks centred, ch3
        # mid). Unchanged contract: not RC_STALE, and it cannot arm.
        c = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
                       safety_params=SafetyParams(arm_high_persist_s=0.0))
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1500, ch4_us=1500, ch5_us=1500,
                      last_update_mono_s=0.0)
        cmd, events, telem = c.process(rc, now_epoch_s=5000.0)
        self.assertNotIn(SafetyEvent.RC_STALE, events)
        self.assertFalse(cmd.is_armed)

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
            last_update_mono_s=self.clock["t"],
        )
        self.clock["t"] += RC_STALE_TIMEOUT_S + 0.5

        cmd, events, telem = c.process(rc, now_epoch_s=100.0)

        imu.get_heading_deg.assert_called_once_with()
        self.assertFalse(cmd.is_armed)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertTrue(telem["rc_stale"])


class TestSlewLimiter(unittest.TestCase):
    def _make_controller(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        params = SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0)
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
            ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0
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
            ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000, last_update_mono_s=0.0
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
    """Contract of 2026-11-10 (with the slew fix of commit 7057ca0): on a
    pack-guard limit the controller ramps its OUTPUT to the limit at the
    VescConfig.pack_low_ramp_s rate (no _motor.stop(): that was a step to RPM
    0 from up to 1.6 m/s), keeps the slew "motion seen" so the release ramps
    from neutral at the mode rates (no snap_first_command step), tells the
    driver whether the operator's request is zero throttle, and pushes the
    arm state and mode every tick so a disarm clears the latch. The driver's
    set_tracks() enforces the same limit again."""

    class _FakeVescTelemetry:
        left_rpm = 100
        right_rpm = 100

        def __init__(self, latched: bool):
            self.pack_low_latched = latched

    class _FakeVescMotor(FakeMotor):
        """FakeMotor that reports a controllable pack-guard limit."""

        def __init__(self):
            super().__init__()
            self.pack_low_latched = False
            self.limit = None
            self.contexts = []

        def get_telemetry(self):
            return TestControllerPackLowLatch._FakeVescTelemetry(self.pack_low_latched)

        def set_drive_context(self, armed, manual, request_neutral=None):
            self.contexts.append((armed, manual, request_neutral))

        def get_drive_limit_bytes(self):
            return self.limit

        def get_pack_guard_status(self):
            return {"latched": self.pack_low_latched, "reason": "pack_voltage"}

    DT = 0.1

    def _rc(self, now, ch):
        return RCInputs(ch1_us=ch, ch2_us=ch, ch3_us=1900, ch4_us=1000,
                        ch5_us=1000, last_update_mono_s=now)

    def _drive(self, motor, script):
        """script: list of (stick_us, limit or None); returns output bytes."""
        self.assertTrue(default_config.slew_limiter.snap_first_command)
        clock = {"t": 10.0}
        outs = []
        with patch("pi_app.control.controller.time.monotonic",
                   side_effect=lambda: clock["t"]):
            c = Controller(motor_driver=motor, arm_relay=FakeRelay(),
                           shutdown_scheduler=FakeShutdown(),
                           safety_params=SafetyParams(debounce_seconds=0.0,
                                                      arm_high_persist_s=0.0))
            for stick, limit in script:
                clock["t"] += self.DT
                motor.limit = limit
                cmd, _, telem = c.process(self._rc(clock["t"], stick), now_epoch_s=clock["t"])
                outs.append(cmd.left_byte)
        return outs, telem

    def test_trip_while_driving_ramps_to_neutral_no_step(self):
        motor = self._FakeVescMotor()
        ramp_s = float(default_config.vesc.pack_low_ramp_s)
        script = [(1500, None)] + [(2000, None)] * 20 + [(2000, 0)] * 15
        outs, telem = self._drive(motor, script)
        full = outs[20]
        self.assertEqual(full, 254)                         # at full speed
        latched = outs[21:]
        step = 128.0 / ramp_s * self.DT
        prev = full
        for b in latched:
            self.assertLessEqual(prev - b, step + 1)        # never a step down
            self.assertLessEqual(b, prev)                   # never up
            prev = b
        self.assertEqual(latched[-1], CENTER_OUTPUT_VALUE)  # zero after the ramp
        self.assertGreater(latched[0], CENTER_OUTPUT_VALUE + 100)
        self.assertEqual(motor.stops, 0)                    # no stop() step
        # The driver hears the operator still asks for throttle.
        self.assertEqual(motor.contexts[-1], (True, True, False))
        self.assertEqual(telem.get("vesc_pack_drive_limit_bytes"), 0)

    def test_release_with_held_stick_ramps_from_neutral(self):
        motor = self._FakeVescMotor()
        accel = float(default_config.slew_limiter.manual_accel_bps) * self.DT
        script = ([(1500, None)] + [(2000, None)] * 20 + [(2000, 0)] * 15
                  + [(2000, None)] * 3)
        outs, _ = self._drive(motor, script)
        rel = outs[-3:]
        for k, b in enumerate(rel, start=1):
            self.assertEqual(b, CENTER_OUTPUT_VALUE + int(round(k * accel)), outs)

    def test_cap_ramps_down_then_holds_and_a_stop_passes(self):
        motor = self._FakeVescMotor()
        script = ([(1500, None)] + [(2000, None)] * 20 + [(2000, 32)] * 15
                  + [(1500, 32)])
        outs, _ = self._drive(motor, script)
        capped = outs[21:-1]
        self.assertTrue(all(b <= a for a, b in zip(capped, capped[1:])))
        self.assertEqual(capped[-1], CENTER_OUTPUT_VALUE + 32)
        # Stick released: the normal decel applies, inside the envelope.
        self.assertLess(outs[-1], CENTER_OUTPUT_VALUE + 32)

    def test_guard_query_error_fails_closed(self):
        motor = self._FakeVescMotor()

        def boom():
            raise RuntimeError("guard unreadable")
        outs, _ = self._drive(motor, [(1500, None)] + [(2000, None)] * 5)
        moving = outs[-1]
        self.assertGreater(moving, CENTER_OUTPUT_VALUE)
        motor2 = self._FakeVescMotor()
        motor2.get_drive_limit_bytes = boom  # type: ignore[method-assign]
        with self.assertLogs("pi_app.control.controller", level="ERROR"):
            outs2, telem = self._drive(motor2, [(1500, None)] + [(2000, None)] * 5)
        self.assertEqual(outs2[-1], CENTER_OUTPUT_VALUE)
        self.assertEqual(telem.get("vesc_pack_drive_limit_bytes"), 0)

    def test_telemetry_flag_without_limit_method_falls_back_to_latch(self):
        motor = self._FakeVescMotor()
        del_limit = motor.get_drive_limit_bytes
        motor.get_drive_limit_bytes = lambda: None  # type: ignore[method-assign]
        motor.pack_low_latched = True
        outs, telem = self._drive(motor, [(1500, None), (2000, None), (2000, None)])
        self.assertEqual(outs[-1], CENTER_OUTPUT_VALUE)    # robot at rest stays at rest
        self.assertTrue(telem.get("vesc_pack_low_latched"))
        self.assertTrue(callable(del_limit))

    def test_disarm_edge_reaches_driver(self):
        params = SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0)
        arm_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000,
                          ch5_us=1000, last_update_mono_s=0.0)
        off_rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000,
                          ch5_us=1000, last_update_mono_s=0.0)
        motor = self._FakeVescMotor()
        fake_now = {"t": 1.0}
        with patch("pi_app.control.controller.time.monotonic",
                   side_effect=lambda: fake_now["t"]):
            c = Controller(motor_driver=motor, arm_relay=FakeRelay(),
                           shutdown_scheduler=FakeShutdown(), safety_params=params)
            c.process(arm_rc, now_epoch_s=0.1)
            self.assertEqual(motor.contexts[-1][:2], (True, True))
            fake_now["t"] = 1.2
            c._mode = "FOLLOW_ME"
            c.process(arm_rc, now_epoch_s=0.2)
            self.assertEqual(motor.contexts[-1][0], True)
            fake_now["t"] = 1.4
            c.process(off_rc, now_epoch_s=0.3)
        self.assertEqual(motor.contexts[-1][0], False)

    def test_latched_telemetry_key_present_and_false_by_default(self):
        """vesc_pack_low_latched reaches the telemetry dict every tick (same
        visibility contract as charger_inhibit)."""
        c = Controller(motor_driver=FakeMotor(), arm_relay=FakeRelay(),
                       shutdown_scheduler=FakeShutdown())
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000,
                      ch5_us=1000, last_update_mono_s=0.0)
        _, _, telem = c.process(rc, now_epoch_s=0.3)
        self.assertIn("vesc_pack_low_latched", telem)
        self.assertFalse(telem["vesc_pack_low_latched"])


class TestSlewRampAfterArmedStopRelease(unittest.TestCase):
    """An armed stop that hands control back must ramp, never snap.

    2026-11-05 09:31:19.72 (arm_20261105_093117.log): the charger inhibit
    released with the stick at full reverse and the next tick sent 0, 0.
    Root cause: the stop called _reset_slew_state(), which clears
    _slew_seen_non_neutral, and snap_first_command (True in production)
    passes the first non-neutral command unramped. These tests use the
    PRODUCTION slew config (snap_first_command=True); the older pack-low
    test sets it False, which is why it never saw the snap.
    """

    class _Motor(FakeMotor):
        def __init__(self):
            super().__init__()
            self.pack_low_latched = False

        def get_telemetry(self):
            latched = self.pack_low_latched

            class _T:
                left_rpm = 0
                right_rpm = 0
                pack_low_latched = latched
            return _T()

    ACCEL = float(default_config.slew_limiter.manual_accel_bps)
    DT = 0.1

    def _rc(self, now, ch=1500):
        return RCInputs(ch1_us=ch, ch2_us=ch, ch3_us=1900, ch4_us=1000,
                        ch5_us=1000, last_update_mono_s=now)

    def _run(self, engage, release, ch, oa=None, dist=None):
        """Arm, engage the stop with the stick held at ``ch``, release it;
        return (engaged output, the first 3 released outputs)."""
        self.assertTrue(default_config.slew_limiter.snap_first_command)
        motor = self._Motor()
        clock = {"t": 10.0}
        out = []
        with patch("pi_app.control.controller.time.monotonic",
                   side_effect=lambda: clock["t"]):
            c = Controller(motor_driver=motor, arm_relay=FakeRelay(),
                           shutdown_scheduler=FakeShutdown(),
                           obstacle_avoidance=oa,
                           safety_params=SafetyParams(debounce_seconds=0.0,
                                                      arm_high_persist_s=0.0))
            state = {"dist": 5.0}

            def tick(stick):
                clock["t"] += self.DT
                if oa is not None:
                    c.set_obstacle_data(distance_m=state["dist"], age_s=0.0)
                cmd, _, _ = c.process(self._rc(clock["t"], stick), now_epoch_s=clock["t"])
                self.assertTrue(cmd.is_armed)
                return (cmd.left_byte, cmd.right_byte)

            tick(1500)                       # arm at rest
            engage(c, motor, state)
            engaged = [tick(ch) for _ in range(3)]
            release(c, motor, state)
            out = [tick(ch) for _ in range(3)]
        return engaged, out

    def _expect_ramp(self, engaged, out, sign):
        step = int(round(self.ACCEL * self.DT))
        for e in engaged:
            self.assertEqual(e, (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
        for k, (l, r) in enumerate(out, start=1):
            want = CENTER_OUTPUT_VALUE + sign * k * step
            self.assertEqual((l, r), (want, want), out)

    @staticmethod
    def _charger_on(c, m, s):
        c.set_charger_inhibit(True)

    @staticmethod
    def _charger_off(c, m, s):
        c.set_charger_inhibit(False)

    def test_charger_release_with_stick_full_reverse_ramps(self):
        """The 2026-11-05 sequence: no 126 -> 0 step."""
        engaged, out = self._run(self._charger_on, self._charger_off, ch=1000)
        self._expect_ramp(engaged, out, sign=-1)

    def test_charger_release_with_stick_full_forward_ramps(self):
        engaged, out = self._run(self._charger_on, self._charger_off, ch=2000)
        self._expect_ramp(engaged, out, sign=+1)

    def test_charger_release_with_neutral_stick_stays_neutral(self):
        engaged, out = self._run(self._charger_on, self._charger_off, ch=1500)
        for e in engaged + out:
            self.assertEqual(e, (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))

    def test_pack_low_release_with_production_snap_config_ramps(self):
        def on(c, m, s):
            m.pack_low_latched = True

        def off(c, m, s):
            m.pack_low_latched = False
        engaged, out = self._run(on, off, ch=2000)
        self._expect_ramp(engaged, out, sign=+1)

    def test_person_stop_release_ramps_and_the_stop_stays_immediate(self):
        """Distance 0.0 (YOLO stop tier) bypasses the slew on engage; its
        release must ramp, not snap to the held stick."""
        from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
        from config import ObstacleAvoidanceConfig

        def on(c, m, s):
            s["dist"] = 0.0

        def off(c, m, s):
            s["dist"] = 5.0
        oa = ObstacleAvoidanceController(ObstacleAvoidanceConfig())
        engaged, out = self._run(on, off, ch=2000, oa=oa)
        self._expect_ramp(engaged, out, sign=+1)

    def test_person_stop_cuts_a_moving_command_in_one_tick(self):
        """The fix must not delay the stop: full forward to neutral at once."""
        from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
        from config import ObstacleAvoidanceConfig
        motor = self._Motor()
        clock = {"t": 10.0}
        with patch("pi_app.control.controller.time.monotonic",
                   side_effect=lambda: clock["t"]):
            c = Controller(motor_driver=motor, arm_relay=FakeRelay(),
                           shutdown_scheduler=FakeShutdown(),
                           obstacle_avoidance=ObstacleAvoidanceController(ObstacleAvoidanceConfig()),
                           safety_params=SafetyParams(debounce_seconds=0.0,
                                                      arm_high_persist_s=0.0))
            last = None
            for k in range(30):
                clock["t"] += self.DT
                c.set_obstacle_data(distance_m=5.0 if k < 29 else 0.0, age_s=0.0)
                cmd, _, _ = c.process(self._rc(clock["t"], 1500 if k == 0 else 2000),
                                      now_epoch_s=clock["t"])
                if k == 28:
                    last = cmd.left_byte
        self.assertGreater(last, CENTER_OUTPUT_VALUE + 50)
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))


class TestControllerVescBusTelemetry(unittest.TestCase):
    """Bus voltage, input current and motor current per VESC reach the telemetry
    dict (and from there the run log) and clear to None when the VESC goes stale."""

    class _FakeVescTelemetry:
        left_rpm = 100
        right_rpm = 100
        left_voltage_v = 54.5
        right_voltage_v = 54.6
        left_current_in_a = -6.5
        right_current_in_a = -5.0
        left_current_a = -18.2
        right_current_a = -17.9

        def __init__(self, left_status_age_s=0.01):
            self.left_status_age_s = left_status_age_s
            self.right_status_age_s = 0.01

    class _FakeVescMotor(FakeMotor):
        def __init__(self):
            super().__init__()
            self.status_age_s = 0.01

        def get_telemetry(self):
            return TestControllerVescBusTelemetry._FakeVescTelemetry(self.status_age_s)

    def _process(self, motor):
        c = Controller(motor_driver=motor, arm_relay=FakeRelay(),
                       shutdown_scheduler=FakeShutdown())
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1000, ch4_us=1000,
                      ch5_us=1000, last_update_mono_s=0.0)
        _, _, telem = c.process(rc, now_epoch_s=0.3)
        return telem

    def test_bus_values_in_telemetry(self):
        telem = self._process(self._FakeVescMotor())
        self.assertEqual(telem["vesc_left_bus_v"], 54.5)
        self.assertEqual(telem["vesc_right_bus_v"], 54.6)
        self.assertEqual(telem["vesc_left_bus_a"], -6.5)
        self.assertEqual(telem["vesc_right_bus_a"], -5.0)
        self.assertEqual(telem["vesc_left_motor_a"], -18.2)
        self.assertEqual(telem["vesc_right_motor_a"], -17.9)

    def test_bus_values_none_when_a_vesc_goes_stale(self):
        motor = self._FakeVescMotor()
        motor.status_age_s = 1.0  # past the 0.5 s staleness cut-off
        telem = self._process(motor)
        for key in ("vesc_left_bus_v", "vesc_right_bus_v",
                    "vesc_left_bus_a", "vesc_right_bus_a",
                    "vesc_left_motor_a", "vesc_right_motor_a"):
            self.assertIn(key, telem)
            self.assertIsNone(telem[key])

    def test_bus_keys_none_without_vesc_telemetry(self):
        telem = self._process(FakeMotor())
        self.assertIsNone(telem["vesc_left_bus_v"])
        self.assertIsNone(telem["vesc_right_bus_a"])


class TestFollowMeNoTargetRegression(unittest.TestCase):
    """Bug #2: flipping ch4 (follow-me) while armed with no person in frame
    must NOT raise UnboundLocalError on the telemetry dict."""

    def test_follow_me_entered_no_target_does_not_raise(self):
        motor = FakeMotor()
        relay = FakeRelay()
        shutdown = FakeShutdown()
        # One-tick arm as setup (arm timing is in test_rc_arm_channel_failsafe).
        c = Controller(motor_driver=motor, arm_relay=relay, shutdown_scheduler=shutdown,
                       safety_params=SafetyParams(arm_high_persist_s=0.0))
        # Arm with ch4 low (no person detections fed → no target present)
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_mono_s=time.monotonic())
        cmd, _, _ = c.process(rc, now_epoch_s=100.5)
        self.assertTrue(cmd.is_armed)
        # ch4 rising edge while armed, still no target. Previously raised
        # UnboundLocalError because `telemetry` was referenced before assignment.
        rc2 = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1900, ch5_us=1000,
                       last_update_mono_s=time.monotonic())
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
                       safety_params=SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0))
        # Arm
        arm = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                       last_update_mono_s=time.monotonic())
        c.process(arm, now_epoch_s=100.0)
        self.assertTrue(c.is_armed)
        c.enter_calibration_mode()
        return c, motor, relay, shutdown

    def test_estop_during_calibration_disarms_stops_and_schedules_shutdown(self):
        c, motor, relay, shutdown = self._armed_calibration_controller()
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1900,
                      last_update_mono_s=time.monotonic())
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
                      last_update_mono_s=time.monotonic())  # ch3 low = disarm
        cmd, events, telem = c.process(rc, now_epoch_s=100.1)
        self.assertFalse(c.is_armed)
        self.assertGreater(motor.stops, stops_before)
        self.assertEqual(telem.get("mode"), "CALIBRATING")

    def test_armed_calm_calibration_does_not_stop_motor(self):
        # While safely armed, process() must not fight the wizard by stopping.
        c, motor, relay, shutdown = self._armed_calibration_controller()
        stops_before = motor.stops
        rc = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                      last_update_mono_s=time.monotonic())
        cmd, events, telem = c.process(rc, now_epoch_s=100.1)
        self.assertTrue(c.is_armed)
        self.assertEqual(motor.stops, stops_before)
        self.assertTrue(telem.get("calibration"))


if __name__ == "__main__":
    unittest.main()


