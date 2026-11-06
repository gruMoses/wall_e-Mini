"""A camera session with log-only VIO, on the fake depthai of the
colour-stall tests: VIO builds with dropping inputs and logs poses, the IMU
reports switch to RAW with the y/z flip, and a VIO build failure still gives
a working camera session (2026-11-06)."""

import sys
import unittest
from unittest.mock import patch

from config import OakDetectionConfig
from pi_app.hardware import oak_depth as oak_depth_mod
from pi_app.tests import test_oak_color_stall as cs


class _Pose:
    def getTranslation(self):
        return type("T", (), {"x": 0.5, "y": 0.0, "z": 0.0})()

    def getQuaternion(self):
        return type("Q", (), {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})()


class _VioState(cs._DaiState):
    def __init__(self):
        super().__init__()
        self.vio_queue = cs._PollQueue(lambda: _Pose())
        self.enabled_reports = []

    def queue_for(self, kind, port):
        if kind == "vio" and port == "transform":
            return self.vio_queue
        return super().queue_for(kind, port)


class _Sensors:
    ACCELEROMETER_RAW = "ACCELEROMETER_RAW"
    GYROSCOPE_RAW = "GYROSCOPE_RAW"
    ACCELEROMETER_UNCALIBRATED = "ACCELEROMETER_UNCALIBRATED"
    GYROSCOPE_UNCALIBRATED = "GYROSCOPE_UNCALIBRATED"


class _SensorsWithoutRaw:
    ACCELEROMETER_UNCALIBRATED = "ACCELEROMETER_UNCALIBRATED"
    GYROSCOPE_UNCALIBRATED = "GYROSCOPE_UNCALIBRATED"


def _install(state, with_vio_ports=True, sensors=_Sensors):
    mod = cs._install_dai(state)
    mod.__version__ = "3.6.1"
    mod.IMUSensor = sensors

    class BasaltVIO:
        _kind = "vio"

    node = type("Node", (cs._NodeNS,), {"BasaltVIO": BasaltVIO})
    mod.node = node
    real_create = mod.Pipeline.create

    def create(self, node_type):
        made = real_create(self, node_type)
        if getattr(node_type, "_kind", "") == "imu":
            def enable(sensor, rate, _made=made):
                state.enabled_reports.append(sensor)
            made._cache["enableIMUSensor"] = enable
        return made

    mod.Pipeline.create = create
    return mod


class TestVioSession(unittest.TestCase):
    def setUp(self):
        self._prev_dai = sys.modules.get("depthai")

    def tearDown(self):
        if self._prev_dai is None:
            sys.modules.pop("depthai", None)
        else:
            sys.modules["depthai"] = self._prev_dai

    def _run(self, vio_enabled, with_vio_ports=True, sensors=_Sensors):
        state = _VioState()
        clock = cs._Clock()
        clock.stop_after = clock.t + 1.0
        reader = cs._make_reader(
            OakDetectionConfig(model_path="config.py", vio_enabled=vio_enabled),
            update_rate_hz=cs._RATE_HZ,
        )
        state.stop_on_start = 2
        state.reader = reader
        state.clock = clock
        clock.reader = reader
        _install(state, sensors=sensors)
        ports = cs._PORT_NAMES + (("imu", "transform") if with_vio_ports else ())
        with patch.object(cs, "_PORT_NAMES", ports), \
                patch("pi_app.hardware.oak_depth.time.monotonic", clock.monotonic), \
                patch("pi_app.hardware.oak_depth.time.sleep", clock.sleep), \
                self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            cs._run_supervisor(reader)
        return state, reader, "\n".join(caught.output)

    def test_vio_on_builds_logs_and_flips(self):
        state, reader, text = self._run(vio_enabled=True)
        self.assertEqual(state.starts, 1, text)
        self.assertIn("ACCELEROMETER_RAW", state.enabled_reports)
        self.assertIs(reader._imu_flip_yz, True)
        self.assertGreater(reader._vio_tracker.poses, 0)
        self.assertIn("(y/z flipped to RDF), VIO on", text)
        self.assertIsNone(reader._vio_disabled_reason)

    def test_vio_on_still_delivers_depth(self):
        state, reader, text = self._run(vio_enabled=True)
        self.assertGreater(reader._depth_recv_count, 0, text)

    def test_raw_enable_failure_comes_back_without_vio(self):
        """Grok 2026-11-06: a session that cannot enable RAW must not retry
        RAW forever; the next session runs on UNCALIBRATED without VIO."""
        state, reader, text = self._run(vio_enabled=True, sensors=_SensorsWithoutRaw)
        self.assertEqual(state.starts, 1, text)
        self.assertIn("session start failed with VIO", text)
        self.assertEqual(reader._vio_disabled_reason, "session start failed with VIO")
        self.assertIs(reader._imu_flip_yz, False)
        self.assertIn("ACCELEROMETER_UNCALIBRATED", state.enabled_reports)
        self.assertGreater(reader._depth_recv_count, 0)

    def test_vio_off_keeps_uncalibrated_and_no_flip(self):
        state, reader, text = self._run(vio_enabled=False)
        self.assertEqual(state.starts, 1, text)
        self.assertIn("ACCELEROMETER_UNCALIBRATED", state.enabled_reports)
        self.assertNotIn("ACCELEROMETER_RAW", state.enabled_reports)
        self.assertIs(reader._imu_flip_yz, False)
        self.assertEqual(reader._vio_tracker.poses, 0)
        self.assertIn("VIO off", text)

    def test_vio_build_failure_keeps_the_camera_session(self):
        state, reader, text = self._run(vio_enabled=True, with_vio_ports=False)
        self.assertEqual(state.starts, 1, text)
        self.assertGreater(reader._depth_recv_count, 0)
        self.assertTrue(reader._vio_disabled_reason.startswith("build failed"), text)
        self.assertIn("OAK VIO not built", text)
        self.assertEqual(reader._vio_tracker.poses, 0)
        # RAW was requested before the build failed; the flip still matches it.
        self.assertIs(reader._imu_flip_yz, True)


if __name__ == "__main__":
    unittest.main()
