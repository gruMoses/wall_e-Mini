"""Detection-stream freshness for the follow-me gate (2026-09-25).

CAM_A stalled and get_person_detections() kept returning one frozen person.
The age must come from the frame's capture time, and a sequence number that
does not advance must not refresh that clock. The stored detection list is
unchanged: an empty list is a detection gap, and a stale close person must
still force the stop tier.
"""

import math
import unittest
from unittest.mock import patch

from config import FollowMeConfig, OakDetectionConfig, ObstacleAvoidanceConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.hardware.oak_depth import (
    OakDepthReader,
    _nn_msg_capture_ts,
    _nn_msg_sequence_num,
)


def _make_reader(vision_stale_s=None) -> OakDepthReader:
    obs = ObstacleAvoidanceConfig()
    det = None
    if vision_stale_s is not None:
        det = OakDetectionConfig(vision_stale_s=vision_stale_s)
    return OakDepthReader(
        obstacle_config=obs,
        follow_me_config=FollowMeConfig(),
        detection_config=det,
    )


class _Msg:
    """Minimal NN/detection message: host timestamp + sequence."""

    def __init__(self, ts, seq):
        self._ts = ts
        self._seq = seq

    def getTimestamp(self):
        return self._ts

    def getSequenceNum(self):
        return self._seq


class _NoSeq:
    def __init__(self, ts):
        self._ts = ts

    def getTimestamp(self):
        return self._ts


class _Timedelta:
    def __init__(self, seconds):
        self._seconds = seconds

    def total_seconds(self):
        return self._seconds


class _OnceQueue:
    def __init__(self, msg):
        self._msg = msg

    def tryGet(self):
        msg = self._msg
        self._msg = None
        return msg


class TestCaptureTimestamp(unittest.TestCase):
    def test_rejects_only_a_far_future_timestamp(self):
        clock = {"t": 1000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            # 0.6 s in the future: wrong time base.
            self.assertIsNone(_nn_msg_capture_ts(_Msg(1000.6, 1)))
            # Exactly 0.5 s in the future is still accepted.
            self.assertEqual(_nn_msg_capture_ts(_Msg(1000.5, 1)), 1000.5)
            # 10 s old is kept. The 5 s latency window does not apply here.
            self.assertEqual(_nn_msg_capture_ts(_Msg(990.0, 1)), 990.0)
            # timedelta, same conversion as _nn_msg_latency_s.
            self.assertEqual(
                _nn_msg_capture_ts(_Msg(_Timedelta(990.0), 1)),
                990.0,
            )

    def test_missing_sequence_is_none(self):
        self.assertIsNone(_nn_msg_sequence_num(_NoSeq(1.0)))
        self.assertIsNone(_nn_msg_sequence_num(_Msg(1.0, None)))
        self.assertEqual(_nn_msg_sequence_num(_Msg(1.0, 7)), 7)


class TestDetectionFreshness(unittest.TestCase):
    def test_config_default_and_reader_threshold(self):
        self.assertEqual(OakDetectionConfig().vision_stale_s, 1.5)
        self.assertEqual(_make_reader()._vision_stale_s, 1.5)
        self.assertEqual(_make_reader(vision_stale_s=0.4)._vision_stale_s, 0.4)

    def test_age_comes_from_capture_time_not_publish_time(self):
        reader = _make_reader()
        clock = {"t": 5000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            # Published now, but the frame was captured 0.8 s ago.
            reader._note_det_packet_freshness(_Msg(5000.0 - 0.8, 10), 5000.0)
            clock["t"] = 5000.4
            fresh = reader.get_detection_freshness()
        # Publish age would be 0.4 s. Capture age is 1.2 s.
        self.assertAlmostEqual(fresh["age_s"], 1.2, places=3)
        self.assertEqual(fresh["seq"], 10)
        self.assertEqual(fresh["seq_stuck_packets"], 0)
        self.assertFalse(fresh["stale"])
        health = None
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            health = reader.get_health()
        self.assertEqual(health["det_fresh_age_s"], round(fresh["age_s"], 3))
        self.assertEqual(health["det_seq"], 10)
        self.assertEqual(health["det_seq_stuck_packets"], 0)
        self.assertFalse(health["vision_stale"])

    def test_stuck_sequence_does_not_refresh_and_advance_resets(self):
        reader = _make_reader()
        clock = {"t": 5000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._note_det_packet_freshness(_Msg(4999.2, 10), 5000.0)
            # Same sequence, and a capture time that would look fresh if we
            # trusted it. The clock must stay on the first frame.
            clock["t"] = 5001.0
            reader._note_det_packet_freshness(_Msg(5001.0 - 0.05, 10), 5001.0)
            fresh = reader.get_detection_freshness()
            self.assertAlmostEqual(fresh["age_s"], 1.8, places=3)
            self.assertTrue(fresh["stale"])
            self.assertEqual(fresh["seq_stuck_packets"], 1)
            self.assertEqual(fresh["seq"], 10)
            # A backwards sequence did not advance either.
            reader._note_det_packet_freshness(_Msg(5001.0, 3), 5001.0)
            fresh = reader.get_detection_freshness()
            self.assertEqual(fresh["seq_stuck_packets"], 2)
            self.assertEqual(fresh["seq"], 10)
            self.assertAlmostEqual(fresh["age_s"], 1.8, places=3)
            # The next greater sequence refreshes and clears the count.
            reader._note_det_packet_freshness(_Msg(5001.0 - 0.1, 11), 5001.0)
            fresh = reader.get_detection_freshness()
            self.assertEqual(fresh["seq_stuck_packets"], 0)
            self.assertEqual(fresh["seq"], 11)
            self.assertAlmostEqual(fresh["age_s"], 0.1, places=3)
            self.assertFalse(fresh["stale"])

    def test_new_session_accepts_a_restarted_sequence_counter(self):
        # A reconnect rebuilds the pipeline and the device counter restarts
        # at a low number. Without the per-session reset the stream would
        # stay stale until the new counter passed the old one (hours).
        reader = _make_reader()
        clock = {"t": 6000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._note_det_packet_freshness(_Msg(6000.0 - 0.1, 90000), 6000.0)
            clock["t"] = 6010.0  # the reconnect took 10 s
            with reader._lock:
                reader._reset_det_sequence_locked()
            fresh = reader.get_detection_freshness()
            # Still stale until the new session's first packet arrives.
            self.assertTrue(fresh["stale"])
            self.assertIsNone(fresh["seq"])
            reader._note_det_packet_freshness(_Msg(6010.0 - 0.2, 3), 6010.0)
            fresh = reader.get_detection_freshness()
            self.assertEqual(fresh["seq"], 3)
            self.assertAlmostEqual(fresh["age_s"], 0.2, places=3)
            self.assertFalse(fresh["stale"])

    def test_missing_sequence_refreshes_from_capture_time(self):
        reader = _make_reader()
        clock = {"t": 2000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._note_det_packet_freshness(_NoSeq(2000.0 - 0.3), 2000.0)
            clock["t"] = 2000.5
            # Republish the same capture time with a new publish time.
            reader._note_det_packet_freshness(_NoSeq(2000.0 - 0.3), 2000.5)
            fresh = reader.get_detection_freshness()
        self.assertIsNone(fresh["seq"])
        self.assertEqual(fresh["seq_stuck_packets"], 0)
        # 2000.5 - 1999.7 = 0.8, not the 0 s a publish-time refresh would give.
        self.assertAlmostEqual(fresh["age_s"], 0.8, places=3)

    def test_future_capture_falls_back_to_publish_time(self):
        reader = _make_reader()
        clock = {"t": 1000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._note_det_packet_freshness(_Msg(1000.6, 1), 1000.0)
            fresh = reader.get_detection_freshness()
        self.assertAlmostEqual(fresh["age_s"], 0.0, places=3)
        self.assertFalse(fresh["stale"])

    def test_exact_threshold_is_not_stale(self):
        reader = _make_reader()
        clock = {"t": 1001.5}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._note_det_packet_freshness(_Msg(1000.0, 1), 1000.0)
            fresh = reader.get_detection_freshness()
            self.assertAlmostEqual(fresh["age_s"], 1.5, places=3)
            self.assertFalse(fresh["stale"])
            clock["t"] = 1001.5 + 0.001
            fresh = reader.get_detection_freshness()
            self.assertTrue(fresh["stale"])

    def test_before_first_packet_is_stale_with_infinite_age(self):
        reader = _make_reader()
        fresh = reader.get_detection_freshness()
        self.assertTrue(math.isinf(fresh["age_s"]))
        self.assertTrue(fresh["stale"])
        self.assertIsNone(fresh["seq"])
        self.assertEqual(fresh["seq_stuck_packets"], 0)
        health = reader.get_health()
        self.assertIsNone(health["det_fresh_age_s"])
        self.assertTrue(health["vision_stale"])
        self.assertIsNone(health["det_seq"])
        self.assertEqual(health["det_seq_stuck_packets"], 0)
        # Existing health keys stay.
        self.assertIn("detections_stale", health)
        self.assertIn("detections_age_s", health)
        self.assertIn("det_fps", health)

    def test_person_list_is_unchanged_while_stale(self):
        """A stale close person must keep forcing the stop tier."""
        reader = _make_reader()
        person = PersonDetection(
            x_m=-0.8, z_m=2.52, confidence=0.9,
            bbox=(0.30, 0.0, 0.48, 0.80), track_id=453,
        )
        reader._det_state.persons = [person]
        clock = {"t": 8000.0}
        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._note_det_packet_freshness(_Msg(8000.0 - 0.2, 453), 8000.0)
            clock["t"] = 8010.0
            fresh = reader.get_detection_freshness()
            got = reader.get_person_detections()
        self.assertTrue(fresh["stale"])
        self.assertGreater(fresh["age_s"], 1.5)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].x_m, -0.8)
        self.assertEqual(got[0].z_m, 2.52)
        self.assertEqual(got[0].track_id, 453)
        # Still there with no packet at all (infinite age).
        bare = _make_reader()
        bare._det_state.persons = [person]
        self.assertTrue(math.isinf(bare.get_detection_freshness()["age_s"]))
        self.assertEqual(bare.get_person_detections()[0].track_id, 453)

    def test_poll_publish_tracks_freshness(self):
        """The detection publish path, not only the helper, moves the clock."""
        reader = _make_reader()
        clock = {"t": 3000.0}

        class _DetMsg:
            detections = []

            def __init__(self, ts, seq):
                self._ts = ts
                self._seq = seq

            def getTimestamp(self):
                return self._ts

            def getSequenceNum(self):
                return self._seq

        with patch("pi_app.hardware.oak_depth.time.monotonic", side_effect=lambda: clock["t"]):
            reader._poll_detections(_OnceQueue(_DetMsg(3000.0 - 0.2, 4)))
            fresh = reader.get_detection_freshness()
            self.assertEqual(fresh["seq"], 4)
            self.assertAlmostEqual(fresh["age_s"], 0.2, places=3)
            self.assertEqual(fresh["seq_stuck_packets"], 0)
            clock["t"] = 3001.2
            reader._poll_detections(_OnceQueue(_DetMsg(3001.2 - 0.05, 4)))
            fresh = reader.get_detection_freshness()
        self.assertEqual(fresh["seq_stuck_packets"], 1)
        self.assertAlmostEqual(fresh["age_s"], 1.4, places=3)
        self.assertFalse(fresh["stale"])


if __name__ == "__main__":
    unittest.main()
