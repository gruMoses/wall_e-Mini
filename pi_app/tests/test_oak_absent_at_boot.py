"""OAK camera absent at boot behaves like a stale camera (audit 2026-11-25, 2.4).

Before: ``OakDepthReader.detect()`` ran once in main.py. With no camera at
that moment no reader was built: no obstacle controller, no 0.8 m person
stop, no vision-stale gate, and MANUAL drove at full speed until the next
service restart. A camera that fails after startup instead leaves MANUAL at
the stale-depth floor (``manual_stale_throttle_scale``) and refuses
FOLLOW_ME and WAYPOINT_NAV.

Kevin's decision (2026-11-25): start the reader anyway and let it wait.
Grok's review of the first version (REJECT): stereo depth can be fresh
before the first detection packet of a session, and the stop tier then
sees no live detections, so MANUAL must stay on the floor until each
session's first detection packet (first-packet gate), also after a
reconnect.

Fake devices only (``detect`` and the device session are replaced; no
depthai call). The fake session uses the reader's own code where it
matters: ``_poll_depth`` on a depth frame, the session-start reset
``_reset_det_sequence_locked``, and ``_note_det_packet_freshness`` for a
detection packet. Depth and detections are switched separately.
"""

import shutil
import sys
import tempfile
import threading
import time
import types
import unittest
from dataclasses import replace

import numpy as np

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
from pi_app.app.oak_startup import bounded_detect, start_oak_reader
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.imu_steering import ImuSteeringCompensator
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.safety import SafetyParams
from pi_app.hardware.oak_depth import ObjectDetection, OakDepthReader
from pi_app.hardware.oak_imu import OakImuReader
from pi_app.hardware.oak_recorder import OakRecorder
from pi_app.tests.test_waypoint_nav_heading_guard import _clock, _Rig

OAK_LOGGER = "pi_app.hardware.oak_depth"
STALE_S = float(default_config.obstacle_avoidance.stale_timeout_s)
STALE_FLOOR = float(default_config.obstacle_avoidance.manual_stale_throttle_scale)
HANG_S = 0.4
# Forward bytes at the floor for a full stick (254): 126 + 128 * 0.1.
FLOOR_FWD_MAX = int(round((254 - CENTER_OUTPUT_VALUE) * STALE_FLOOR)) + 1

ARMED_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                    last_update_mono_s=0.0)
FWD_RC = RCInputs(ch1_us=2000, ch2_us=2000, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                  last_update_mono_s=0.0)
FM_RC = RCInputs(ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1900, ch5_us=1000,
                 last_update_mono_s=0.0)

PERSON = PersonDetection(x_m=0.0, z_m=2.0, confidence=0.9, bbox=(0.4, 0.1, 0.6, 0.9))
CLOSE_PERSON = ObjectDetection(
    label=0, label_name="person", confidence=0.9, x_m=0.0, z_m=0.5,
    bbox=(0.4, 0.1, 0.6, 0.9), safety_tier="stop",
)
GYRO_RAW_DPS = (0.30, -0.45, 0.10)


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


class _Frame:
    def __init__(self, arr):
        self._arr = arr

    def getFrame(self):
        return self._arr


class _OneShot:
    def __init__(self, item=None):
        self._item = item

    def tryGet(self):
        item, self._item = self._item, None
        return item


class _NnMsg:
    def __init__(self, seq):
        self._seq = seq

    def getSequenceNum(self):
        return self._seq


EMPTY_CORRIDOR = np.zeros((400, 640), dtype=np.uint16)


class _Session:
    """Fake device session driven by the reader's supervisor.

    ``depth`` / ``detections`` / ``imu`` switch what each 10 ms iteration
    publishes. ``drop()`` ends the session like a USB drop; the supervisor
    then backs off and opens the next one.
    """

    def __init__(self, reader, usb=None):
        self.reader = reader
        self.usb = usb
        self.depth = True
        self.detections = False
        self.imu = False
        self.stationary = False
        self.persons = []
        self.all_dets = []
        self.opened = []          # usb.polls at each open
        self._drop = threading.Event()
        self._seq = 0
        self._dev_t0 = time.monotonic()

    def drop(self):
        self._drop.set()

    def run_once(self, dai, npmod):
        r = self.reader
        self.opened.append(self.usb.polls if self.usb is not None else None)
        self._drop.clear()
        with r._lock:
            r._pipeline_running = True
            r._connected = True
            r._last_pipeline_loop_ts = time.monotonic()
            r._reset_det_sequence_locked()  # what the real session start does
        while not r._stop_event.is_set() and not self._drop.is_set():
            now = time.monotonic()
            with r._lock:
                r._last_pipeline_loop_ts = now
            if self.detections:
                self._seq += 1
                with r._lock:
                    r._det_state.persons = list(self.persons)
                    r._det_state.timestamp = now
                    r._all_dets_state.detections = list(self.all_dets)
                    r._all_dets_state.timestamp = now
                    r._note_det_packet_freshness(_NnMsg(self._seq), now)
            if self.depth:
                r._poll_depth(_OneShot(_Frame(EMPTY_CORRIDOR)), _OneShot(None), np)
                with r._lock:
                    r._last_depth_recv_ts = now
            if self.imu:
                with r._lock:
                    st = r._imu_state
                    st.gx_rads, st.gy_rads, st.gz_rads = (
                        np.radians(v) for v in GYRO_RAW_DPS)
                    st.ay_mss = -9.81
                    st.timestamp = now
                    st.device_timestamp_s = 1.0 + (now - self._dev_t0)
                    st.stationary = bool(self.stationary)
            r._stop_event.wait(0.01)
        with r._lock:
            r._pipeline_running = False
            r._connected = False
        return True


def _make_reader(usb, *, boot=(0.01, 0.02, 0.05), remind=600.0, detect_timeout=5.0):
    reader = OakDepthReader(
        default_config.obstacle_avoidance,
        default_config.follow_me,
        detection_config=replace(default_config.oak_detection, vision_hang_s=HANG_S),
    )
    reader.detect = usb.detect  # instance attribute shadows the staticmethod
    reader._BOOT_WAIT_BACKOFF_S = boot
    reader._BOOT_WAIT_REMIND_S = remind
    reader._BOOT_WAIT_DETECT_TIMEOUT_S = detect_timeout
    reader._RECONNECT_BACKOFF_S = (0.02, 0.02, 0.02)
    reader._wait_for_device = lambda timeout_s: True
    return reader


def _with_session(reader, usb=None):
    session = _Session(reader, usb)
    reader._run_pipeline_once = session.run_once
    return session


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
        self.reader = _make_reader(self.usb)
        self.session = _with_session(self.reader, self.usb)

    def tearDown(self):
        self.reader.stop()

    def test_no_session_and_every_reading_stale(self):
        self.reader.start(wait_for_device=True)
        self.assertTrue(_wait_until(lambda: self.usb.polls >= 5))
        self.assertEqual(self.session.opened, [], "no device open against an absent camera")
        h = self.reader.get_health()
        self.assertTrue(h["awaiting_device"])
        self.assertFalse(h["connected"])
        self.assertFalse(h["pipeline_running"])
        self.assertTrue(h["critical_stale"])
        self.assertFalse(h["vision_hung"])
        _dist, age = self.reader.get_min_distance()
        self.assertEqual(age, float("inf"), "never received: inf, like get_imu_data")
        fresh = self.reader.get_detection_freshness()
        self.assertTrue(fresh["stale"])
        self.assertFalse(fresh["session_seen"])
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
        self.session.detections = True
        self.reader.start(wait_for_device=True)
        self.assertTrue(_wait_until(lambda: self.usb.polls >= 3))
        thread = self.reader._thread
        self.usb.present = True
        self.assertTrue(_wait_until(lambda: self.reader.get_health()["connected"]))
        self.assertEqual(len(self.session.opened), 1)
        self.assertIs(self.reader._thread, thread, "no restart: same worker thread")
        self.assertTrue(thread.is_alive())
        self.assertTrue(_wait_until(lambda: self.reader.get_detection_freshness()["session_seen"]))
        self.assertFalse(self.reader.get_detection_freshness()["stale"])
        h = self.reader.get_health()
        self.assertFalse(h["awaiting_device"])
        self.assertFalse(h["critical_stale"])
        self.assertLess(self.reader.get_min_distance()[1], STALE_S)

    def test_journal_first_miss_and_connect_only(self):
        with self.assertLogs(OAK_LOGGER, level="WARNING") as logs:
            self.reader.start(wait_for_device=True)
            self.assertTrue(_wait_until(lambda: self.usb.polls >= 20))
            self.usb.present = True
            self.assertTrue(_wait_until(lambda: len(self.session.opened) == 1))
        out = [r.getMessage() for r in logs.records]
        self.assertEqual(sum("not on USB at startup" in m for m in out), 1, out)
        self.assertEqual(sum("still not on USB" in m for m in out), 0, out)
        self.assertEqual(sum("OAK-D found on USB" in m for m in out), 1, out)

    def test_reminder_is_rate_limited(self):
        self.reader.stop()
        usb = _UsbCamera(present=False)
        reader = _make_reader(usb, boot=(0.005,), remind=0.1)
        _with_session(reader, usb)
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
        self.assertEqual(self.session.opened, [])
        self.assertFalse(self.reader.get_health()["awaiting_device"])

    def test_stop_while_detect_returns_true_opens_no_session(self):
        def detect_then_stop():
            self.reader._stop_event.set()
            return True

        self.reader.detect = detect_then_stop
        self.reader.start(wait_for_device=True)
        self.assertTrue(_wait_until(lambda: not self.reader._thread.is_alive()))
        self.assertEqual(self.session.opened, [])

    def test_camera_present_at_boot_skips_the_wait(self):
        # Unchanged path: start() opens the session at once; no USB poll.
        self.usb.present = True
        self.reader.start()
        self.assertTrue(_wait_until(lambda: len(self.session.opened) == 1))
        self.assertEqual(self.session.opened, [0], "the session opened before any detect() poll")
        self.assertFalse(self.reader.get_health()["awaiting_device"])

    def test_session_seen_resets_on_each_session(self):
        # Reconnect: the old session's detection time is kept (its age keeps
        # growing) but the new session has not refreshed the stream yet.
        self.usb.present = True
        self.session.detections = True
        self.reader.start()
        self.assertTrue(_wait_until(lambda: self.reader.get_detection_freshness()["session_seen"]))
        self.session.detections = False
        self.session.drop()
        self.assertTrue(_wait_until(lambda: len(self.session.opened) == 2))
        self.assertTrue(_wait_until(lambda: self.reader.get_min_distance()[1] < STALE_S))
        fresh = self.reader.get_detection_freshness()
        self.assertFalse(fresh["session_seen"])
        self.assertIsNotNone(self.reader._det_fresh_ts, "old session's packet time kept")
        self.assertFalse(self.reader.get_health()["det_session_seen"])


class TestBlockedUsbEnumeration(unittest.TestCase):
    """detect() -> getAllAvailableDevices() has no timeout of its own."""

    def setUp(self):
        self.release = threading.Event()
        self.calls = []
        self.answer = False

        def blocking_detect():
            self.calls.append(time.monotonic())
            self.release.wait(10.0)
            return self.answer

        self.usb = _UsbCamera()
        self.reader = _make_reader(self.usb, detect_timeout=0.05)
        self.reader.detect = blocking_detect
        self.session = _with_session(self.reader, self.usb)

    def tearDown(self):
        self.release.set()
        self.reader.stop()

    def test_blocked_enumeration_stops_the_heartbeat(self):
        with self.assertLogs(OAK_LOGGER, level="WARNING") as logs:
            self.reader.start(wait_for_device=True)
            time.sleep(HANG_S * 2.5)
        h = self.reader.get_health()
        self.assertTrue(h["vision_hung"], "a blocked enumeration is not a live wait")
        self.assertTrue(h["awaiting_device"])
        self.assertEqual(len(self.calls), 1, "one probe in flight, not a pile-up")
        self.assertEqual(self.session.opened, [])
        out = [r.getMessage() for r in logs.records]
        self.assertEqual(sum("has not returned" in m for m in out), 1, out)
        t0 = time.monotonic()
        thread = self.reader._thread
        self.reader.stop()
        self.assertLess(time.monotonic() - t0, 1.0, "stop() while detect() is blocked")
        self.assertFalse(thread.is_alive())

    def test_enumeration_that_returns_late_still_finds_the_camera(self):
        self.answer = True
        self.reader.start(wait_for_device=True)
        time.sleep(0.2)
        self.assertEqual(self.session.opened, [])
        self.release.set()
        self.assertTrue(_wait_until(lambda: len(self.session.opened) == 1))
        self.assertFalse(self.reader.get_health()["vision_hung"])

    def test_startup_detect_is_bounded(self):
        rec = _Rec()
        t0 = time.monotonic()
        reader, present = start_oak_reader(lambda: rec, self.reader.detect, detect_timeout_s=0.05)
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertFalse(present)
        self.assertEqual(rec.start_calls, [{"wait_for_device": True}])
        self.assertIsNone(bounded_detect(self.reader.detect, 0.05))


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


class _ControllerRig(unittest.TestCase):
    """Reader + Controller wired as in main.py (registration and per-tick feed)."""

    present_at_boot = False

    def setUp(self):
        self.usb = _UsbCamera(present=self.present_at_boot)
        self.made = []

        def make():
            reader = _make_reader(self.usb)
            self.session = _with_session(reader, self.usb)
            self.made.append(reader)
            return reader

        self.reader, present = start_oak_reader(make, self.usb.detect)
        self.assertEqual(present, self.present_at_boot)
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
        self._now = 10.0

    def tearDown(self):
        self.reader.stop()

    def _feed(self):
        """The per-tick OAK feed of main.py's control loop."""
        dist, age = self.reader.get_min_distance()
        self.ctrl.set_obstacle_data(dist, age)
        self.ctrl.set_person_detections(self.reader.get_person_detections())

    def _tick(self, rc, bt=None):
        self._now += 0.033
        self._feed()
        return self.ctrl.process(rc, now_epoch_s=self._now, bt_override_bytes=bt)

    def _arm(self):
        self._tick(ARMED_RC)
        self.assertTrue(self.ctrl._safety_state.is_armed)

    def _full_stick(self, n=30, bt=None, dt_s=0.0):
        """Full forward for n ticks. ``dt_s`` real seconds between ticks:
        the slew limiter runs on the real monotonic clock."""
        rc = ARMED_RC if bt is not None else FWD_RC
        for _ in range(n):
            cmd, _ev, telem = self._tick(rc, bt)
            if dt_s:
                time.sleep(dt_s)
        return cmd, telem

    def _assert_floor(self, cmd, telem):
        """Scale at the floor, the slew target at the floor share of a full
        stick, and the motor bytes forward but no higher than that."""
        self.assertEqual(telem["mode"], "MANUAL")
        self.assertAlmostEqual(telem["obstacle_throttle_scale"], STALE_FLOOR)
        for key in ("slew_in_left", "slew_in_right"):
            self.assertGreater(telem[key], CENTER_OUTPUT_VALUE)
            self.assertLessEqual(telem[key] - CENTER_OUTPUT_VALUE, FLOOR_FWD_MAX)
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)
        self.assertLessEqual(cmd.left_byte - CENTER_OUTPUT_VALUE, FLOOR_FWD_MAX)
        self.assertLessEqual(cmd.right_byte - CENTER_OUTPUT_VALUE, FLOOR_FWD_MAX)

    def _depth_fresh(self):
        return _wait_until(lambda: self.reader.get_min_distance()[1] < STALE_S)

    def _heading_compensator(self):
        imu = OakImuReader(self.reader, yaw_rate_source="gyro_y",
                           stationary_bias_tracking_enabled=True, zupt_enabled=True)
        cfg = replace(default_config.imu_steering, calibration_timeout_s=0.02)
        comp = ImuSteeringCompensator(cfg, imu)
        imu._defer_duration_s = 0.2  # short still window for the test
        return comp


class TestAbsentCameraGatesTheController(_ControllerRig):

    def test_manual_gets_the_stale_floor(self):
        self.assertGreater(STALE_FLOOR, 0.0)
        self.assertLess(STALE_FLOOR, 1.0)
        self._arm()
        cmd, telem = self._full_stick()
        self._assert_floor(cmd, telem)

    def test_phone_override_gets_the_floor(self):
        self._arm()
        cmd, telem = self._full_stick(bt=(254, 254))
        self._assert_floor(cmd, telem)

    def test_follow_me_refused(self):
        self._arm()
        _cmd, _ev, telem = self._tick(FM_RC)
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

    def test_fresh_depth_before_the_first_detection_packet_keeps_the_floor(self):
        # Grok MUST-FIX: depth publishes on the first frame; detections later.
        self.session.depth = True
        self.session.detections = False
        self._arm()
        self.usb.present = True
        self.assertTrue(self._depth_fresh())
        self.assertFalse(self.reader.get_detection_freshness()["session_seen"])
        self.assertEqual(self.reader.get_person_detections(), [])
        cmd, telem = self._full_stick()
        self._assert_floor(cmd, telem)
        self._assert_floor(*self._full_stick(bt=(254, 254)))
        # First detection packet of the session: the floor lifts.
        self.session.detections = True
        self.assertTrue(_wait_until(lambda: self.reader.get_detection_freshness()["session_seen"]))
        self.assertTrue(self._depth_fresh())
        _cmd, telem = self._full_stick(n=2)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)

    def test_stop_tier_detection_still_stops_under_the_gate(self):
        # A close person left in the list (previous session) forces the stop.
        with self.reader._lock:
            self.reader._all_dets_state.detections = [CLOSE_PERSON]
        self.session.depth = True
        self.session.detections = False
        self._arm()
        self.usb.present = True
        self.assertTrue(_wait_until(lambda: self.reader.get_min_distance()[0] == 0.0
                                    and self.reader.get_min_distance()[1] < STALE_S))
        cmd, telem = self._full_stick()
        self.assertEqual(telem["obstacle_throttle_scale"], 0.0)
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))

    def test_camera_arrival_clears_the_floor_and_gates_without_restart(self):
        comp = self._heading_compensator()
        self.session.persons = [PERSON]
        self.session.detections = True
        self.session.imu = True
        self._arm()
        self._assert_floor(*self._full_stick(n=3))
        self.assertFalse(self.ctrl.activate_follow_me())
        thread = self.reader._thread

        self.usb.present = True
        self.assertTrue(_wait_until(
            lambda: self.reader.get_detection_freshness()["session_seen"]
            and not self.reader.get_detection_freshness()["stale"]
        ))
        self.assertTrue(self._depth_fresh())
        self.assertEqual(len(self.made), 1, "one reader for the whole run")
        self.assertIs(self.reader._thread, thread, "no restart: same worker thread")

        _cmd, telem = self._full_stick(n=2)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)
        # FOLLOW_ME starts on a fresh stream with a person in view.
        self.assertTrue(self.ctrl.activate_follow_me())
        self.assertEqual(self.ctrl._mode, "FOLLOW_ME")
        self.ctrl.deactivate_follow_me()
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.ctrl.set_vision_freshness_getter(self.reader.get_detection_freshness)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())

        # Heading: samples are fresh but the boot calibration saw none, so it
        # stays stale while the robot moves...
        for _ in range(30):
            comp.update(0.0, 1.0 / 30.0)
            time.sleep(0.01)
        self.assertTrue(comp.get_status().heading_stale)
        self.assertEqual(comp.heading_age_s(), float("inf"))
        self.assertTrue(comp.imu_reader.get_health()["gyro_bias_uncalibrated"])
        # ...and becomes valid after a still window, with the measured bias.
        self.session.stationary = True
        self.assertTrue(_wait_until(
            lambda: (comp.update(0.0, 1.0 / 30.0), not comp.get_status().heading_stale)[1],
            timeout_s=3.0,
        ))
        self.assertLess(comp.heading_age_s(), 0.5)
        imu = comp.imu_reader
        self.assertFalse(imu.get_health()["gyro_bias_uncalibrated"])
        for got, want in zip(imu.gyro_bias_dps, GYRO_RAW_DPS):
            self.assertAlmostEqual(got, want, places=6)
        self.assertEqual(imu.yaw_axis_sign, 1.0, "sign unchanged")


class TestReconnectBeforeFirstDetectionPacket(_ControllerRig):
    """Mid-run reconnect: the new session's depth comes before its first
    detection packet, and the old session's packet time is still stored."""

    present_at_boot = True

    def test_reconnect_keeps_the_floor_until_the_first_packet(self):
        self.session.detections = True
        self.assertTrue(_wait_until(lambda: self.reader.get_detection_freshness()["session_seen"]))
        self.assertTrue(self._depth_fresh())
        self._arm()
        _cmd, telem = self._full_stick(n=2)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)

        # USB drop; the next session publishes depth only.
        self.session.detections = False
        self.session.drop()
        self.assertTrue(_wait_until(lambda: len(self.session.opened) == 2))
        self.assertTrue(self._depth_fresh())
        self.assertFalse(self.reader.get_detection_freshness()["session_seen"])
        # The drop was short (backoff 0.02 s), so the old packet time is not
        # even stale: only the per-session flag holds the floor here.
        self.assertFalse(self.reader.get_detection_freshness()["stale"])
        # From full speed the slew ramps down (350 bytes/s) to the floor.
        cmd, telem = self._full_stick(n=50, dt_s=0.01)
        self._assert_floor(cmd, telem)

        self.session.detections = True
        self.assertTrue(_wait_until(lambda: self.reader.get_detection_freshness()["session_seen"]))
        self.assertTrue(self._depth_fresh())
        _cmd, telem = self._full_stick(n=2)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)


class TestFirstPacketGateUnit(unittest.TestCase):

    def setUp(self):
        self.oa = ObstacleAvoidanceController(default_config.obstacle_avoidance)

    def test_cap_in_manual_and_autonomous(self):
        self.assertEqual(self.oa.compute_throttle_scale(5.0, 0.0, is_manual=True), 1.0)
        self.assertAlmostEqual(
            self.oa.compute_throttle_scale(5.0, 0.0, is_manual=True, detections_unseen=True),
            STALE_FLOOR)
        self.assertEqual(
            self.oa.compute_throttle_scale(5.0, 0.0, is_manual=False, detections_unseen=True),
            0.0)

    def test_cap_overrides_the_netting_mute_and_keeps_the_stop(self):
        self.assertAlmostEqual(self.oa.compute_throttle_scale(
            0.5, 0.0, is_manual=True, corridor_muted=True, detections_unseen=True), STALE_FLOOR)
        self.assertEqual(self.oa.compute_throttle_scale(
            0.0, 0.0, is_manual=True, detections_unseen=True), 0.0)

    def test_controller_ignores_a_missing_key_and_a_failed_getter(self):
        # No general "stale detections" rule: only an explicit False gates.
        ctrl = Controller(
            motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
            obstacle_avoidance=self.oa,
            safety_params=SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0),
        )
        ctrl.set_vision_freshness_getter(lambda: {"age_s": 9.0, "stale": True})
        ctrl.process(ARMED_RC, now_epoch_s=10.0)
        ctrl.set_obstacle_data(5.0, 0.0)
        _cmd, _ev, telem = ctrl.process(FWD_RC, now_epoch_s=10.1)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)

        def boom():
            raise RuntimeError("reader gone")

        ctrl.set_vision_freshness_getter(boom)
        ctrl.set_obstacle_data(5.0, 0.0)
        _cmd, _ev, telem = ctrl.process(FWD_RC, now_epoch_s=10.2)
        self.assertEqual(telem["obstacle_throttle_scale"], 1.0)


class TestDeferredGyroCalibration(unittest.TestCase):

    def setUp(self):
        self.reader = OakDepthReader(default_config.obstacle_avoidance, default_config.follow_me)
        self.imu = OakImuReader(self.reader, yaw_rate_source="gyro_y")
        self.imu.calibrate_gyro(duration_s=0.02)

    def _step(self, t, still=True, raw=GYRO_RAW_DPS):
        return self.imu._deferred_bias_step(0.01, t, still, raw)

    def test_no_samples_at_boot_defers(self):
        self.assertTrue(self.imu._bias_uncalibrated)
        # First read "uncalibrated" (not "init"); with no sample at all the
        # stronger "stale" wins after that. Both are stale for the hold.
        self.assertEqual(self.imu.read()["integrate_status"], "uncalibrated")
        self.assertEqual(self.imu.read()["integrate_status"], "stale")

    def test_still_window_sets_the_bias(self):
        done = [self._step(1.0 + 0.1 * i) for i in range(11)]
        self.assertEqual(done, [False] * 10 + [True])
        self.assertFalse(self.imu._bias_uncalibrated)
        for got, want in zip(self.imu.gyro_bias_dps, GYRO_RAW_DPS):
            self.assertAlmostEqual(got, want)

    def test_motion_restarts_the_window_and_duplicates_do_not_count(self):
        for i in range(8):
            self.assertFalse(self._step(1.0 + 0.1 * i))
        self.assertFalse(self._step(1.8, still=False))
        self.assertFalse(self._step(1.8))  # duplicate device time
        for i in range(10):
            self.assertFalse(self._step(1.9 + 0.1 * i))
        self.assertTrue(self._step(2.9))
        self.assertEqual(self.imu.yaw_axis_sign, 1.0)

    def test_calibration_with_samples_is_unchanged(self):
        with self.reader._lock:
            self.reader._imu_state.gy_rads = np.radians(0.2)
            self.reader._imu_state.ay_mss = -9.81
            self.reader._imu_state.timestamp = time.monotonic()
        stop = threading.Event()

        def keep_fresh():
            while not stop.is_set():
                with self.reader._lock:
                    self.reader._imu_state.timestamp = time.monotonic()
                time.sleep(0.005)

        t = threading.Thread(target=keep_fresh, daemon=True)
        t.start()
        try:
            self.imu.calibrate_gyro(duration_s=0.05)
        finally:
            stop.set()
            t.join()
        self.assertFalse(self.imu._bias_uncalibrated)
        self.assertAlmostEqual(self.imu.gyro_bias_dps[1], 0.2, places=6)
        self.assertNotEqual(self.imu.read()["integrate_status"], "uncalibrated")


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
