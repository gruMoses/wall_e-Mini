"""Tests for OakRecorder._write_mcap_telemetry: the fields RecordingTelemetry
already carries (main.py wires them from controller telemetry every tick)
but that were silently dropped before reaching the MCAP file (2026-09-19
logging audit, Commit C item 5).

_write_mcap_telemetry only calls writer.add_message(...) -- it never touches
self, so a fake writer that records its call is enough; no real mcap
library or OAK hardware is needed (the `mcap` package need not even be
installed, matching the rest of the test suite).
"""
import json
import unittest

from config import config
from pi_app.hardware.oak_recorder import OakRecorder, RecordingTelemetry


class _FakeMcapWriter:
    def __init__(self):
        self.messages = []

    def add_message(self, channel_id, log_time, data, publish_time):
        self.messages.append(
            {"channel_id": channel_id, "log_time": log_time, "data": data, "publish_time": publish_time}
        )


def _telem(**overrides) -> RecordingTelemetry:
    base = dict(
        timestamp=1000.0,
        mode="MANUAL",
        throttle_scale=1.0,
        obstacle_distance_m=None,
        motor_left=126,
        motor_right=126,
        is_armed=True,
        depth_stats=None,
        person_detections=[],
    )
    base.update(overrides)
    return RecordingTelemetry(**base)


class TestWriteMcapTelemetryNewFields(unittest.TestCase):

    def setUp(self):
        self.recorder = OakRecorder(config.oak_recording)
        self.writer = _FakeMcapWriter()

    def _write(self, telem):
        self.recorder._write_mcap_telemetry(self.writer, channel_id=3, t=telem)
        self.assertEqual(len(self.writer.messages), 1)
        return json.loads(self.writer.messages[0]["data"])

    def test_heading_and_offset_fields_present(self):
        obj = self._write(_telem(
            corrected_heading_deg=12.34,
            heading_offset_deg=-5.678,
            heading_offset_locked=True,
            heading_offset_frozen=False,
        ))
        self.assertEqual(obj["corrected_heading_deg"], 12.3)
        self.assertEqual(obj["heading_offset_deg"], -5.7)
        self.assertIs(obj["heading_offset_locked"], True)
        self.assertIs(obj["heading_offset_frozen"], False)

    def test_gps_block_present_when_lat_lon_set(self):
        obj = self._write(_telem(
            gps_lat=40.12345678, gps_lon=-74.87654321,
            gps_fix=4, gps_sats=14, gps_hdop=0.8, gps_diff_age_s=1.25,
        ))
        self.assertIn("gps", obj)
        self.assertEqual(obj["gps"]["lat"], 40.12345678)
        self.assertEqual(obj["gps"]["lon"], -74.87654321)
        self.assertEqual(obj["gps"]["fix"], 4)
        self.assertEqual(obj["gps"]["sats"], 14)
        self.assertEqual(obj["gps"]["hdop"], 0.8)
        self.assertEqual(obj["gps"]["diff_age_s"], 1.2)

    def test_gps_block_absent_without_lat_lon(self):
        obj = self._write(_telem())
        self.assertNotIn("gps", obj)

    def test_vesc_fields_present(self):
        obj = self._write(_telem(
            vesc_left_rpm=500, vesc_right_rpm=505, vesc_actual_speed_mps=0.812345,
        ))
        self.assertEqual(obj["vesc_left_rpm"], 500)
        self.assertEqual(obj["vesc_right_rpm"], 505)
        self.assertEqual(obj["vesc_actual_speed_mps"], 0.812)

    def test_charger_inhibit_nav_state_wp_heading_error(self):
        obj = self._write(_telem(
            charger_inhibit=True, nav_state="DRIVE", wp_heading_error_deg=7.89,
        ))
        self.assertIs(obj["charger_inhibit"], True)
        self.assertEqual(obj["nav_state"], "DRIVE")
        self.assertEqual(obj["wp_heading_error_deg"], 7.9)

    def test_charger_inhibit_false_still_written(self):
        # `if t.charger_inhibit is not None` must distinguish False from
        # "not set" (None) -- a naive truthy check would drop False silently.
        obj = self._write(_telem(charger_inhibit=False))
        self.assertIn("charger_inhibit", obj)
        self.assertIs(obj["charger_inhibit"], False)

    def test_fields_absent_when_none(self):
        obj = self._write(_telem())
        for key in (
            "corrected_heading_deg", "heading_offset_deg", "heading_offset_locked",
            "heading_offset_frozen", "gps", "vesc_left_rpm", "vesc_right_rpm",
            "vesc_actual_speed_mps", "charger_inhibit", "nav_state",
            "wp_heading_error_deg",
        ):
            self.assertNotIn(key, obj)

    def test_existing_fields_unaffected(self):
        obj = self._write(_telem(
            mode="FOLLOW_ME", throttle_scale=0.5, motor_left=150, motor_right=100,
            is_armed=True,
        ))
        self.assertEqual(obj["mode"], "FOLLOW_ME")
        self.assertEqual(obj["throttle_scale"], 0.5)
        self.assertEqual(obj["motor_left"], 150)
        self.assertEqual(obj["motor_right"], 100)
        self.assertIs(obj["is_armed"], True)


if __name__ == "__main__":
    unittest.main()
