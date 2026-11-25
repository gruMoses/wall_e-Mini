"""OAK camera absent at boot behaves like a stale camera (audit 2026-11-25, 2.4).

Before: ``OakDepthReader.detect()`` ran once in main.py. With no camera at
that moment no reader was built: no obstacle controller, no 0.8 m person
stop, no vision-stale gate, and MANUAL drove at full speed until the next
service restart. A camera that fails after startup instead leaves MANUAL at
the stale-depth floor (``manual_stale_throttle_scale``) and refuses
FOLLOW_ME and WAYPOINT_NAV.

Kevin's decision (2026-11-25): start the reader anyway and let it wait.
These tests use fake devices only (``detect`` and the device session are
replaced; no depthai call is made):

* the reader waits on USB enumeration only, opens no session, keeps its
  heartbeat (no false hang restart), logs sparingly, stops promptly;
* main.py's wiring (pi_app/app/oak_startup.py plus the per-tick feed) gives
  MANUAL the floor and refuses FOLLOW_ME and WAYPOINT_NAV, heading stale;
* a camera that arrives later opens the session in the same thread and the
  floor and gates clear with no restart;
* the camera-present-at-boot path is unchanged (no USB wait, no detect
  poll, recorder blocks for its queues as before).
"""

import logging
import shutil
import sys
import tempfile
import threading
import time
import types
import unittest
from dataclasses import replace

# Same stub as test_oak_reconnect.py: _run_pipeline imports depthai once the
# camera is "found"; the fake sessions below never touch it.
if "depthai" not in sys.modules:
    _fake_dai = types.ModuleType("depthai")

    class _FakeDevice:
        @staticmethod
        def getAllAvailableDevices():
            return []

    _fake_dai.Device = _FakeDevice
    sys.modules["depthai"] = _fake_dai

from config import config as default_config
from pi_app.app.oak_startup import start_oak_reader
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.safety import SafetyParams
from pi_app.hardware.oak_depth import OakDepthReader
from pi_app.hardware.oak_imu import OakImuReader
from pi_app.hardware.oak_recorder import OakRecorder
from pi_app.tests.test_waypoint_nav_heading_guard import _clock, _Rig

OAK_LOGGER = "pi_app.hardware.oak_depth"
STALE_FLOOR = float(default_config.obstacle_avoidance.manual_stale_throttle_scale)
HANG_S = 0.4

ARMED_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                    last_update_mono_s=0.0)
FWD_RC = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                  last_update_mono_s=0.0)
FM_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1900, ch5_us=1000,
                 last_update_mono_s=0.0)

PERSON = PersonDetection(x_m=0.0, z_m=2.0, confidence=0.9, bbox=(0.4, 0.1, 0.6, 0.9))


def _wait_until(pred, timeout_s=3.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.005)
    return bool(pred())


class _UsbCamera:
    """Fake USB bus: ``detect()`` answers ``present``; counts the polls."""

    def __init__(self, present=False):
        self.present = present
        self.polls = 0

    def detect(self):
        self.polls += 1
        return self.present


def _make_reader(usb, **backoff):
    reader = OakDepthReader(
        default_config.obstacle_avoidance,
        default_config.follow_me,
        detection_config=replace(default_config.oak_detection, vision_hang_s=HANG_S),
    )
    reader.detect = usb.detect  # instance attribute shadows the staticmethod
    reader._BOOT_WAIT_BACKOFF_S = backoff.get("boot", (0.01, 0.02, 0.05))
    reader._BOOT_WAIT_REMIND_S = backoff.get("remind", 600.0)
    reader._RECONNECT_BACKOFF_S = (0.02, 0.02, 0.02)
    return reader


def _live_session(reader, opened, usb=None, persons=()):
    """Fake device session: open, publish fresh depth / detections / IMU
    every 10 ms until stop(), like a healthy camera."""

    def run_once(dai, npmod):
        opened.append(usb.polls if usb is not None else None)
        with reader._lock:
            reader._pipeline_running = True
            reader._connected = True
        while not reader._stop_event.is_set():
            now = time.monotonic()
            with reader._lock:
                reader._last_pipeline_loop_ts = now
                reader._last_depth_recv_ts = now
                reader._depth_state.min_distance_m = 5.0
                reader._depth_state.timestamp = now
                reader._det_fresh_ts = now
                reader._det_state.persons = list(persons)
                reader._det_state.timestamp = now
                reader._imu_state.ay_mss = -9.81
                reader._imu_state.timestamp = now
            reader._stop_event.wait(0.01)
        with reader._lock:
            reader._pipeline_running = False
            reader._connected = False
        return True

    return run_once


class _Rec:
    """Records start() calls (the shape main.py uses) for the helper test."""

    def __init__(self):
        self.start_calls = []

    def start(self, **kw):
        self.start_calls.append(kw)


# ---------------------------------------------------------------------------
# The reader waits for a camera that is absent at boot.
# ---------------------------------------------------------------------------

class TestReaderWaitsForAbsentCamera(unittest.TestCase):

    def setUp(self):
        self.usb = _UsbCamera(present=False)
        self.opened = []
        self.reader = _make_reader(self.usb)
        self.reader._run_pipeline_once = _live_session(self.reader, self.opened, self.usb)

    def tearDown(self):
        self.reader.stop()

    def test_no_session_and_every_reading_stale(self):
        self.reader.start(wait_for_device=True)
        self.assertTrue(_wait_until(lambda: self.usb.polls >= 5))
        self.assertEqual(self.opened, [], "no device open against an absent camera")
        h = self.reader.get_health()
        self.assertTrue(h["awaiting_device"])
        self.assertFalse(h["connected"])
        self.assertFalse(h["pipeline_running"])
        self.assertTrue(h["critical_stale"])
        self.assertFalse(h["vision_hung"])
        _dist, age = self.reader.get_min_distance()
        self.assertGreater(age, default_config.obstacle_avoidance.stale_timeout_s)
        self.assertTrue(self.reader.get_detection_freshness()["stale"])
        _imu, imu_age = self.reader.get_imu_data()
        self.assertEqual(imu_age, float("inf"))

    def test_heartbeat_keeps_the_hang_monitor_quiet(self):
        # vision_hung would make main.py exit for a systemd restart while
        # disarmed. A wait for an absent camera is not a hung worker.
        self.reader.start(wait_for_device=True)
        time.sleep(HANG_S * 2.5)
        h = self.reader.get_health()
        self.assertTrue(h["awaiting_device"])
        self.assertTrue(h["vision_thread_alive"])
        self.assertFalse(h["vision_hung"])
        self.assertLess(h["vision_worker_age_s"], HANG_S)

    def test_camera_arriving_later_opens_the_session_in_the_same_thread(self):
        self.reader.start(wait_for_device=True)
        self.assertTrue(_wait_until(lambda: self.usb.polls >= 3))
        thread = self.reader._thread
        self.usb.present = True
        self.assertTrue(_wait_until(lambda: self.reader.get_health()["connected"]))
        self.assertEqual(len(self.opened), 1)
        self.assertIs(self.reader._thread, thread, "no restart: same worker thread")
        self.assertTrue(thread.is_alive())
        self.assertTrue(_wait_until(lambda: not self.reader.get_detection_freshness()["stale"]))
        h = self.reader.get_health()
        self.assertFalse(h["awaiting_device"])
        self.assertFalse(h["critical_stale"])
        _dist, age = self.reader.get_min_distance()
        self.assertLess(age, default_config.obstacle_avoidance.stale_timeout_s)

    def test_journal_first_miss_and_connect_only(self):
        with self.assertLogs(OAK_LOGGER, level="WARNING") as logs:
            self.reader.start(wait_for_device=True)
            self.assertTrue(_wait_until(lambda: self.usb.polls >= 20))
            self.usb.present = True
            self.assertTrue(_wait_until(lambda: len(self.opened) == 1))
        out = [r.getMessage() for r in logs.records]
        self.assertEqual(sum("not on USB at startup" in m for m in out), 1, out)
        self.assertEqual(sum("still not on USB" in m for m in out), 0, out)
        self.assertEqual(sum("OAK-D found on USB" in m for m in out), 1, out)

    def test_reminder_is_rate_limited(self):
        self.reader.stop()
        usb = _UsbCamera(present=False)
        reader = _make_reader(usb, boot=(0.005,), remind=0.1)
        reader._run_pipeline_once = _live_session(reader, [], usb)
        self.reader = reader
        with self.assertLogs(OAK_LOGGER, level="WARNING") as logs:
            reader.start(wait_for_device=True)
            time.sleep(0.35)
            reader.stop()
        out = [r.getMessage() for r in logs.records]
        reminders = sum("still not on USB" in m for m in out)
        self.assertGreater(usb.polls, 20)
        self.assertGreaterEqual(reminders, 1)
        self.assertLessEqual(reminders, 4)

    def test_stop_during_the_wait_is_prompt(self):
        self.reader._BOOT_WAIT_BACKOFF_S = (30.0,)
        self.reader.start(wait_for_device=True)
        self.assertTrue(_wait_until(lambda: self.usb.polls >= 1))
        t0 = time.monotonic()
        thread = self.reader._thread
        self.reader.stop()
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(self.opened, [])
        self.assertFalse(self.reader.get_health()["awaiting_device"])

    def test_camera_present_at_boot_skips_the_wait(self):
        # Unchanged path: start() opens the session at once; no USB poll.
        self.usb.present = True
        self.reader.start()
        self.assertTrue(_wait_until(lambda: len(self.opened) == 1))
        self.assertEqual(self.opened, [0], "the session opened before any detect() poll")
        self.assertFalse(self.reader.get_health()["awaiting_device"])


# ---------------------------------------------------------------------------
# main.py's startup helper.
# ---------------------------------------------------------------------------

class TestStartupHelper(unittest.TestCase):

    def test_absent_at_boot_still_starts_the_reader_waiting(self):
        rec = _Rec()
        reader, present = start_oak_reader(lambda: rec, lambda: False)
        self.assertIs(reader, rec)
        self.assertFalse(present)
        self.assertEqual(rec.start_calls, [{"wait_for_device": True}])

    def test_present_at_boot_starts_the_reader_as_before(self):
        rec = _Rec()
        reader, present = start_oak_reader(lambda: rec, lambda: True)
        self.assertIs(reader, rec)
        self.assertTrue(present)
        self.assertEqual(rec.start_calls, [{}])


# ---------------------------------------------------------------------------
# The controller gates, wired the way main.py wires them.
# ---------------------------------------------------------------------------

class FakeMotor:
    def __init__(self):
        self.commands = []

    def set_tracks(self, left_byte, right_byte):
        self.commands.append((left_byte, right_byte))

    def stop(self):
        pass


class FakeRelay:
    def set_armed(self, armed):
        pass


class FakeShutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


class TestAbsentCameraGatesTheController(unittest.TestCase):

    def setUp(self):
        self.usb = _UsbCamera(present=False)
        self.opened = []
        self.made = []

        def make():
            reader = _make_reader(self.usb)
            reader._run_pipeline_once = _live_session(
                reader, self.opened, self.usb, persons=[PERSON])
            self.made.append(reader)
            return reader

        self.reader, present = start_oak_reader(make, self.usb.detect)
        self.assertFalse(present)
        self.assertTrue(_wait_until(lambda: self.reader.get_health()["awaiting_device"]))
        self.ctrl = Controller(
            motor_driver=FakeMotor(),
            arm_relay=FakeRelay(),
            shutdown_scheduler=FakeShutdown(),
            obstacle_avoidance=ObstacleAvoidanceController(default_config.obstacle_avoidance),
            follow_me=FollowMeController(default_config.follow_me),
            safety_params=SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0),
        )
        # main.py registers the freshness getter whenever the reader exists.
        self.ctrl.set_vision_freshness_getter(self.reader.get_detection_freshness)

    def tearDown(self):
        self.reader.stop()

    def _feed(self):
        """The per-tick OAK feed of main.py's control loop."""
        dist, age = self.reader.get_min_distance()
        self.ctrl.set_obstacle_data(dist, age)
        self.ctrl.set_person_detections(self.reader.get_person_detections())

    def _tick(self, rc, now):
        self._feed()
        return self.ctrl.process(rc, now_epoch_s=now)

    def _arm(self):
        self._tick(ARMED_RC, 10.0)
        self.assertTrue(self.ctrl._safety_state.is_armed)

    def _heading_compensator(self):
        imu = OakImuReader(self.reader, yaw_rate_source="gyro_y",
                           stationary_bias_tracking_enabled=True, zupt_enabled=True)
        cfg = replace(default_config.imu_steering, calibration_timeout_s=0.02)
        return ImuSteeringCompensator(cfg, imu)

    def test_manual_gets_the_stale_floor(self):
        self.assertGreater(STALE_FLOOR, 0.0)
        self.assertLess(STALE_FLOOR, 1.0)
        self._arm()
        cmd, _ev, telem = self._tick(FWD_RC, 10.1)
        self.assertEqual(telem["mode"], "MANUAL")
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], STALE_FLOOR)
        for _ in range(30):
            cmd, _ev, telem = self._tick(FWD_RC, 10.2)
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], STALE_FLOOR)
        # Forward part at most the floor share of full stick.
        self.assertLessEqual(cmd.left_byte - CENTER_OUTPUT_VALUE,
                             int(round((255 - CENTER_OUTPUT_VALUE) * STALE_FLOOR)) + 1)
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)

    def test_follow_me_refused(self):
        self._arm()
        _cmd, _ev, telem = self._tick(FM_RC, 10.1)
        self.assertEqual(self.ctrl._mode, "MANUAL")
        self.assertEqual(telem.get("follow_me_activation_blocked"), "vision_stale")
        self.assertTrue(telem.get("vision_stale"))
        self.assertFalse(self.ctrl.activate_follow_me())
        self.assertEqual(self.ctrl._mode, "MANUAL")

    def test_waypoint_nav_refused_and_heading_stale(self):
        comp = self._heading_compensator()
        self.assertTrue(comp.get_status().heading_stale)
        self.assertEqual(comp.heading_age_s(), float("inf"))
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock, imu=comp)
            # Heading alone already refuses the run...
            self.assertEqual(rig.ctrl.waypoint_nav_activation_blocked(), "heading_stale")
            # ...and with main.py's freshness getter the stale stream does.
            rig.ctrl.set_vision_freshness_getter(self.reader.get_detection_freshness)
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "vision_stale")
        self.assertEqual(rig.ctrl._mode, "MANUAL")

    def test_camera_arrival_clears_the_floor_and_gates_without_restart(self):
        comp = self._heading_compensator()
        self._arm()
        _cmd, _ev, telem = self._tick(FWD_RC, 10.1)
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], STALE_FLOOR)
        self.assertFalse(self.ctrl.activate_follow_me())
        thread = self.reader._thread

        self.usb.present = True
        self.assertTrue(_wait_until(
            lambda: not self.reader.get_detection_freshness()["stale"]
            and self.reader.get_min_distance()[1]
            < default_config.obstacle_avoidance.stale_timeout_s
        ))
        self.assertEqual(len(self.made), 1, "one reader for the whole run")
        self.assertIs(self.reader._thread, thread, "no restart: same worker thread")

        _cmd, _ev, telem = self._tick(FWD_RC, 10.2)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)
        # FOLLOW_ME starts on a fresh stream with a person in view.
        self.assertTrue(self.ctrl.activate_follow_me())
        self.assertEqual(self.ctrl._mode, "FOLLOW_ME")
        self.ctrl.deactivate_follow_me()
        # The heading comes up on the compensator built at boot.
        comp.update(0.0, 1.0 / 30.0)
        self.assertFalse(comp.get_status().heading_stale)
        self.assertLess(comp.heading_age_s(), 0.5)
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.ctrl.set_vision_freshness_getter(self.reader.get_detection_freshness)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())


# ---------------------------------------------------------------------------
# The recorder does not block startup for an absent camera.
# ---------------------------------------------------------------------------

class _QueueReader:
    def __init__(self, ready=False):
        self.ready = ready
        self.timeouts = []
        self.queues = {"h265": None}

    def get_recording_queues(self, timeout_s=10.0):
        self.timeouts.append(timeout_s)
        return dict(self.queues) if self.ready else None


class TestRecorderAbsentAtBoot(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="walle_rec_")
        cfg = replace(default_config.oak_recording, recording_dir=self.tmp)
        self.rec = OakRecorder(cfg)

    def tearDown(self):
        self.rec.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_no_wait_then_queues_taken_once_the_session_opens(self):
        reader = _QueueReader(ready=False)
        t0 = time.monotonic()
        self.rec.start(reader, wait_for_queues=False)
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertEqual(reader.timeouts, [], "start() made no blocking fetch")
        self.rec._poll_device_queues()
        self.assertIsNone(self.rec._recording_queues)
        self.assertEqual(reader.timeouts, [0.0])
        reader.ready = True
        self.rec._poll_device_queues()
        self.assertEqual(self.rec._recording_queues, {"h265": None})
        self.rec._poll_device_queues()
        self.assertEqual(reader.timeouts, [0.0, 0.0], "no fetch after the queues are taken")

    def test_default_start_blocks_for_the_queues_as_before(self):
        reader = _QueueReader(ready=True)
        self.rec.start(reader)
        self.assertEqual(reader.timeouts, [15.0])
        self.assertEqual(self.rec._recording_queues, {"h265": None})
        self.rec._poll_device_queues()
        self.assertEqual(reader.timeouts, [15.0])

    def test_default_start_does_not_retry_later(self):
        # Unchanged: a blocking fetch that found nothing is not retried.
        reader = _QueueReader(ready=False)
        self.rec.start(reader)
        reader.ready = True
        self.rec._poll_device_queues()
        self.assertIsNone(self.rec._recording_queues)
        self.assertEqual(reader.timeouts, [15.0])


if __name__ == "__main__":
    unittest.main()
