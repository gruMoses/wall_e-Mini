"""Hardware-free tests for the 2026-09-20 vision-latency fix and diagnostics.

Covers: deadline sleep helper, detection/depth poll order, hand-poll gating,
GestureController.get_status() for a stable FIVE in IDLE, and the new
log_gating oak/gesture blocks.
"""
import sys
import types
import unittest
from types import SimpleNamespace
from unittest.mock import patch

# Stub depthai — no OAK hardware in CI. Mirrors test_hand_poll_gating.py.
if "depthai" not in sys.modules:
    _fake_dai = types.ModuleType("depthai")

    class _FakeDevice:
        @staticmethod
        def getAllAvailableDevices():
            return []

    _fake_dai.Device = _FakeDevice
    sys.modules["depthai"] = _fake_dai

from config import FollowMeConfig, ObstacleAvoidanceConfig, OakDetectionConfig
from pi_app.app.log_gating import build_log_obj, build_slow_obj
from pi_app.control.controller import Controller
from pi_app.control.gesture_control import (
    GestureController,
    GestureEvent,
    GestureStateMachine,
    _Phase,
    hand_poll_wanted,
)
from pi_app.hardware.oak_depth import OakDepthReader, vision_loop_sleep_s
from pi_app.tests.test_gesture_control import _hand, _make_open_hand_landmarks


def _make_reader(**kw) -> OakDepthReader:
    return OakDepthReader(
        obstacle_config=ObstacleAvoidanceConfig(),
        follow_me_config=FollowMeConfig(),
        **kw,
    )


def _rc(ch1=1500, ch2=1500, ch3=1500, ch4=1500, ch5=1000):
    return SimpleNamespace(ch1_us=ch1, ch2_us=ch2, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5)


def _cmd(left=126, right=126, armed=True, emergency=False):
    return SimpleNamespace(left_byte=left, right_byte=right, is_armed=armed, emergency_active=emergency)


def _base_kwargs(**overrides):
    kwargs = dict(
        now_ts=1000.0,
        src="RC",
        s=_rc(),
        bt_override=None,
        bt_age=None,
        imu_status=None,
        telem={},
        oak_depth_stats=None,
        oak_persons=[],
        gps_reading=None,
        bms_state=None,
        bms_charging=None,
        recording_state=None,
        cmd=_cmd(),
        loop_dt_ms=33,
        imu_dt_ms=33,
        imu_motion_witness_still=None,
        events=[],
    )
    kwargs.update(overrides)
    return kwargs


class _OrderReader(OakDepthReader):
    """Stub whose poll methods only record call order."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.order: list[str] = []

    def _poll_depth(self, *args, **kwargs) -> None:
        self.order.append("depth")

    def _poll_detections(self, *args, **kwargs) -> None:
        self.order.append("detections")


class TestVisionLoopSleep(unittest.TestCase):
    def test_remainder_of_period(self):
        self.assertAlmostEqual(
            vision_loop_sleep_s(0.0667, 0.02, True), 0.0467, places=4,
        )

    def test_overrun_yields_one_ms(self):
        self.assertEqual(vision_loop_sleep_s(0.0667, 0.10, True), 0.001)

    def test_flag_false_is_unconditional_period(self):
        self.assertAlmostEqual(
            vision_loop_sleep_s(0.0667, 0.02, False), 0.0667, places=4,
        )
        self.assertAlmostEqual(
            vision_loop_sleep_s(0.0667, 0.10, False), 0.0667, places=4,
        )


class TestPollOrder(unittest.TestCase):
    def test_detections_first_when_flag_on(self):
        reader = _OrderReader(
            obstacle_config=ObstacleAvoidanceConfig(),
            follow_me_config=FollowMeConfig(),
            detection_config=OakDetectionConfig(poll_detections_first=True),
        )
        reader._poll_vision_queues(None, None, None, None)
        self.assertEqual(reader.order, ["detections", "depth"])

    def test_depth_first_when_flag_off(self):
        reader = _OrderReader(
            obstacle_config=ObstacleAvoidanceConfig(),
            follow_me_config=FollowMeConfig(),
            detection_config=OakDetectionConfig(poll_detections_first=False),
        )
        reader._poll_vision_queues(None, None, None, None)
        self.assertEqual(reader.order, ["depth", "detections"])

    def test_default_is_detections_first(self):
        reader = _OrderReader(
            obstacle_config=ObstacleAvoidanceConfig(),
            follow_me_config=FollowMeConfig(),
        )
        reader._poll_vision_queues(None, None, None, None)
        self.assertEqual(reader.order, ["detections", "depth"])


class TestHandPollGatingPolicy(unittest.TestCase):
    def test_armed_manual(self):
        self.assertTrue(hand_poll_wanted(True, "MANUAL", False))

    def test_armed_follow_me_idle(self):
        self.assertFalse(hand_poll_wanted(True, "FOLLOW_ME", False))

    def test_armed_follow_me_active(self):
        self.assertTrue(hand_poll_wanted(True, "FOLLOW_ME", True))

    def test_disarmed(self):
        self.assertFalse(hand_poll_wanted(False, "MANUAL", False))
        self.assertFalse(hand_poll_wanted(False, "FOLLOW_ME", True))

    def test_hand_poll_in_follow_me_armed_is_enough(self):
        self.assertTrue(
            hand_poll_wanted(True, "FOLLOW_ME", False, hand_poll_in_follow_me=True)
        )
        self.assertFalse(
            hand_poll_wanted(False, "FOLLOW_ME", False, hand_poll_in_follow_me=True)
        )

    def test_controller_hand_poll_wanted_wiring(self):
        c = Controller()
        c._mode = "MANUAL"
        self.assertTrue(c.hand_poll_wanted(True))
        self.assertFalse(c.hand_poll_wanted(False))

        gsm = GestureStateMachine(cooldown_s=0.0)
        c._gesture = gsm
        c._mode = "FOLLOW_ME"
        self.assertFalse(c.hand_poll_wanted(True))

        gsm._phase = _Phase.ACTIVE
        self.assertTrue(c.hand_poll_wanted(True))

    def test_controller_restore_always_when_armed(self):
        c = Controller()
        c._mode = "FOLLOW_ME"
        fake_cfg = SimpleNamespace(
            gesture=SimpleNamespace(hand_poll_in_follow_me=True),
        )
        with patch("pi_app.control.controller.config", fake_cfg):
            self.assertTrue(c.hand_poll_wanted(True))
            self.assertFalse(c.hand_poll_wanted(False))


class TestGestureControllerStatus(unittest.TestCase):
    def test_stable_five_in_idle_is_ignored_not_active_phase(self):
        sm = GestureController(
            activation_sequence=(3, 4, 3),
            stop_gesture="FIVE",
            hold_frames=2,
            sequence_timeout_s=3.0,
            cooldown_s=0.0,
        )
        five = _make_open_hand_landmarks()
        ev = None
        for _ in range(2):
            ev = sm.update(_hand(five))
        self.assertIsNone(ev)
        self.assertEqual(sm.phase_name, "IDLE")
        st = sm.get_status()
        self.assertEqual(st["event_reason"], "ignored_not_active_phase")
        self.assertIsNone(st["event"])
        self.assertEqual(st["label"], "FIVE")
        self.assertEqual(st["finger_count"], 5)
        self.assertTrue(st["hand_detected"])
        self.assertGreater(st["hand_span_px"], 0.0)

    def test_activate_still_returns_event(self):
        # Behaviour of the sequence itself is unchanged; this only checks
        # get_status records the event that update() still returns.
        from pi_app.tests.test_gesture_control import (
            _make_four_fingers_landmarks,
            _make_three_fingers_landmarks,
        )
        sm = GestureController(hold_frames=1, cooldown_s=0.0)
        self.assertIsNone(sm.update(_hand(_make_three_fingers_landmarks())))
        self.assertIsNone(sm.update(_hand(_make_four_fingers_landmarks())))
        ev = sm.update(_hand(_make_three_fingers_landmarks()))
        self.assertEqual(ev, GestureEvent.ACTIVATE)
        st = sm.get_status()
        self.assertEqual(st["event"], "ACTIVATE")
        self.assertEqual(st["event_reason"], "activate")
        self.assertEqual(st["phase"], "ACTIVE")


class TestBuildLogObjLatencyFields(unittest.TestCase):
    def test_gesture_block_and_oak_fields_present(self):
        telem = {
            "gesture": {
                "hand_detected": True,
                "hand_span_px": 120.5,
                "finger_count": 5,
                "label": "FIVE",
                "streak": 12,
                "phase": "IDLE",
                "seq_idx": 0,
                "event": None,
                "event_reason": "ignored_not_active_phase",
                "hand_poll_enabled": True,
            },
        }
        oak = {
            "det_fps": 11.0,
            "vision_loop_hz": 14.5,
            "vision_work_ms": 52.3,
            "hand_poll_ms": 0.0,
            "nn_input_queue_size": 1,
            "hand_poll_enabled": True,
            "mp_loaded": False,
            "hand_detect_rate": 0.0,
        }
        obj = build_log_obj(**_base_kwargs(telem=telem, oak_camera_health=oak))
        g = obj["gesture"]
        self.assertTrue(g["hand_detected"])
        self.assertEqual(g["hand_span_px"], 120.5)
        self.assertEqual(g["finger_count"], 5)
        self.assertEqual(g["label"], "FIVE")
        self.assertEqual(g["streak"], 12)
        self.assertEqual(g["phase"], "IDLE")
        self.assertEqual(g["seq_idx"], 0)
        self.assertIsNone(g["event"])
        self.assertEqual(g["event_reason"], "ignored_not_active_phase")
        self.assertTrue(g["hand_poll_enabled"])
        oak_log = obj["oak"]
        self.assertEqual(oak_log["det_fps"], 11.0)
        self.assertEqual(oak_log["vision_loop_hz"], 14.5)
        self.assertEqual(oak_log["vision_work_ms"], 52.3)
        self.assertEqual(oak_log["hand_poll_ms"], 0.0)
        self.assertEqual(oak_log["nn_input_queue_size"], 1)
        self.assertTrue(oak_log["hand_poll_enabled"])

    def test_missing_telemetry_keys_are_none(self):
        obj = build_log_obj(**_base_kwargs())
        g = obj["gesture"]
        for key in (
            "hand_detected", "hand_span_px", "finger_count", "label",
            "streak", "phase", "seq_idx", "event", "event_reason",
            "hand_poll_enabled",
        ):
            self.assertIsNone(g[key], key)
        oak_log = obj["oak"]
        for key in (
            "vision_loop_hz", "vision_work_ms", "hand_poll_ms",
            "nn_input_queue_size", "hand_poll_enabled",
        ):
            self.assertIsNone(oak_log[key], key)

    def test_slow_log_gesture_fields(self):
        oak = {
            "det_fps": 8.0,
            "hand_poll_enabled": False,
            "hand_poll_ms": 0.0,
            "mp_loaded": True,
            "hand_detect_rate": 0.25,
        }
        obj = build_slow_obj(
            now_ts=1.0, imu_pipeline=None, oak_camera_health=oak, chip_temp_c=40.0,
        )
        g = obj["gesture"]
        self.assertFalse(g["hand_poll_enabled"])
        self.assertEqual(g["hand_poll_ms"], 0.0)
        self.assertTrue(g["mp_loaded"])
        self.assertEqual(g["hand_detect_rate"], 0.25)

    def test_slow_log_missing_keys_are_none(self):
        obj = build_slow_obj(
            now_ts=1.0, imu_pipeline=None, oak_camera_health=None, chip_temp_c=None,
        )
        g = obj["gesture"]
        self.assertIsNone(g["hand_poll_enabled"])
        self.assertIsNone(g["hand_poll_ms"])
        self.assertIsNone(g["mp_loaded"])
        self.assertIsNone(g["hand_detect_rate"])

    def test_get_health_exposes_new_fields(self):
        reader = _make_reader()
        h = reader.get_health()
        self.assertIn("vision_loop_hz", h)
        self.assertIn("vision_work_ms", h)
        self.assertIn("hand_poll_ms", h)
        self.assertIn("nn_input_queue_size", h)
        self.assertIn("hand_poll_enabled", h)
        self.assertIn("mp_loaded", h)
        self.assertIn("hand_detect_rate", h)
        self.assertTrue(h["hand_poll_enabled"])
        self.assertFalse(h["mp_loaded"])
        self.assertIsNone(h["nn_input_queue_size"])


if __name__ == "__main__":
    unittest.main()
