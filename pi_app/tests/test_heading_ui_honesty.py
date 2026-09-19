"""The UI must show the heading state it actually has, not a plausible stand-in.

Three gaps this covers, all found on 2026-09-19:

1. ``property_map`` drew the robot marker from the RAW IMU heading. That heading
   is clockwise-positive but relative to BOOT orientation, so on a map whose
   marker contract is true north it was only right by luck. The waypoint UI
   already preferred ``corrected_heading_deg`` once the GPS aligner locked.
2. ``wp_heading_error_deg`` and ``nav_state`` were never serialized into the
   telemetry SSE, although ``waypoint_nav_ui`` reads both at the top level — so
   the nav panel showed IDLE and a 0 heading error no matter what the navigator
   was doing.
3. ``ImuSteeringCompensator`` swallowed init failures with a bare print, and
   eleven consecutive read errors latched ``is_available = False`` for the rest
   of the session, so a transient USB drop permanently disabled heading hold
   while startup still printed "IMU steering compensation enabled".
"""

from __future__ import annotations

import inspect
import json
import unittest
from typing import Optional
from unittest.mock import patch

from config import ImuSteeringConfig
from pi_app.control.imu_steering import (
    IMU_RECOVERY_RETRY_S,
    ImuSteeringCompensator,
)


class FlakyImuReader:
    """Raises on the first ``fail_count`` reads, then succeeds."""

    use_mag = False
    calibration_path = None

    def __init__(self, fail_count: int = 0, heading_deg: float = 10.0):
        self.fail_count = int(fail_count)
        self.heading_deg = float(heading_deg)
        self.read_calls = 0

    def calibrate_gyro(self, duration_s: float = 3.0):
        return (0.0, 0.0, 0.0)

    def calibrate_mag_hard_iron(self, duration_s: float = 5.0):
        return (0.0, 0.0, 0.0)

    def read(self) -> dict:
        self.read_calls += 1
        if self.fail_count > 0:
            self.fail_count -= 1
            raise RuntimeError("simulated IMU read failure")
        return {
            "heading_deg": self.heading_deg,
            "gz_dps": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
        }


class TestImuAvailabilityRecovery(unittest.TestCase):
    def test_init_failure_is_logged_and_exposed(self):
        imu = FlakyImuReader(fail_count=99)
        with self.assertLogs("pi_app.control.imu_steering", "WARNING") as cm:
            comp = ImuSteeringCompensator(ImuSteeringConfig(), imu)
        self.assertFalse(comp.get_status().is_available)
        self.assertIsNotNone(comp.init_error)
        self.assertIn("simulated IMU read failure", comp.init_error)
        joined = "\n".join(r.getMessage() for r in cm.records)
        self.assertIn("initialization failed", joined)

    def test_recovers_after_eleven_failures_and_the_retry_interval(self):
        # One read is consumed by _initialize_imu, then 11 more fail inside
        # update() to trip the error_count > 10 latch.
        imu = FlakyImuReader(fail_count=12)
        now = [1000.0]

        with patch("pi_app.control.imu_steering.time.monotonic", side_effect=lambda: now[0]):
            comp = ImuSteeringCompensator(ImuSteeringConfig(), imu)
            self.assertFalse(comp.get_status().is_available)

            # Init failed, so the first update() attempt is itself a recovery
            # try; walk it through the remaining failures.
            for _ in range(20):
                comp.update(0.0, 0.02)
                now[0] += IMU_RECOVERY_RETRY_S + 0.1
                if comp.get_status().is_available:
                    break

            self.assertTrue(
                comp.get_status().is_available,
                "compensator never recovered after the reader started working",
            )
            self.assertEqual(comp.get_status().error_count, 0)
            self.assertIsNone(comp.init_error)

    def test_retry_is_rate_limited(self):
        imu = FlakyImuReader(fail_count=99)
        now = [500.0]
        with patch("pi_app.control.imu_steering.time.monotonic", side_effect=lambda: now[0]):
            comp = ImuSteeringCompensator(ImuSteeringConfig(), imu)
            calls_after_init = imu.read_calls
            comp.update(0.0, 0.02)      # one retry attempt
            after_first = imu.read_calls
            self.assertEqual(after_first, calls_after_init + 1)
            for _ in range(5):          # same instant: no further reads
                comp.update(0.0, 0.02)
            self.assertEqual(imu.read_calls, after_first)
            now[0] += IMU_RECOVERY_RETRY_S + 0.1
            comp.update(0.0, 0.02)
            self.assertEqual(imu.read_calls, after_first + 1)

    def test_get_heading_deg_also_retries(self):
        imu = FlakyImuReader(fail_count=1, heading_deg=42.0)
        now = [0.0]
        with patch("pi_app.control.imu_steering.time.monotonic", side_effect=lambda: now[0]):
            comp = ImuSteeringCompensator(ImuSteeringConfig(), imu)
            self.assertFalse(comp.get_status().is_available)
            now[0] += IMU_RECOVERY_RETRY_S + 0.1
            heading = comp.get_heading_deg()
        self.assertTrue(comp.get_status().is_available)
        self.assertAlmostEqual(heading, 42.0, places=6)

    def test_latch_after_eleven_read_errors_then_recovery(self):
        """Init succeeds, 11 reads fail (tripping the latch), then recovery.

        Before 2026-09-19 the latch was permanent for the session.
        """
        imu = FlakyImuReader(fail_count=0, heading_deg=77.0)
        comp = ImuSteeringCompensator(ImuSteeringConfig(), imu)
        self.assertTrue(comp.get_status().is_available)

        now = [2000.0]
        with patch("pi_app.control.imu_steering.time.monotonic", side_effect=lambda: now[0]):
            imu.fail_count = 11
            with self.assertLogs("pi_app.control.imu_steering", "WARNING") as cm:
                for _ in range(11):
                    comp.update(0.0, 0.02)
                    now[0] += 0.02
            self.assertFalse(comp.get_status().is_available)
            self.assertIn(
                "disabled after", "\n".join(r.getMessage() for r in cm.records)
            )
            self.assertEqual(imu.fail_count, 0)  # the reader is healthy again

            # The latch resets the retry clock, so the very next call retries.
            comp.update(0.0, 0.02)
            self.assertTrue(comp.get_status().is_available)
            self.assertEqual(comp.get_status().error_count, 0)
            self.assertIsNone(comp.init_error)

    def test_healthy_reader_reports_available_and_no_error(self):
        comp = ImuSteeringCompensator(ImuSteeringConfig(), FlakyImuReader())
        self.assertTrue(comp.get_status().is_available)
        self.assertIsNone(comp.init_error)


class TestTelemetrySerializer(unittest.TestCase):
    """The waypoint UI reads nav_state / wp_heading_error_deg at the top level."""

    def _first_sse_payload(self, telem) -> dict:
        from werkzeug.test import EnvironBuilder

        from config import OakWebViewerConfig
        from pi_app.web.oak_viewer import create_app

        class _Recorder:
            recording_state = "idle"

            def get_latest_telemetry(self):
                return telem

        app = create_app(_Recorder(), OakWebViewerConfig())
        env = EnvironBuilder(path="/api/telemetry").get_environ()
        app_iter = app.wsgi_app(env, lambda *a, **k: None)
        try:
            chunk = next(iter(app_iter))
            text = chunk.decode() if isinstance(chunk, (bytes, bytearray)) else chunk
            return json.loads(text.split("data: ", 1)[1].strip())
        finally:
            closer = getattr(app_iter, "close", None)
            if closer is not None:
                closer()

    def _telem(self, **kwargs):
        from pi_app.hardware.oak_recorder import RecordingTelemetry

        base = dict(
            timestamp=1.0,
            mode="WAYPOINT_NAV",
            throttle_scale=1.0,
            obstacle_distance_m=None,
            motor_left=126,
            motor_right=126,
            is_armed=True,
            depth_stats=None,
            person_detections=[],
        )
        base.update(kwargs)
        return RecordingTelemetry(**base)

    def test_nav_state_and_heading_error_are_serialized(self):
        try:
            import werkzeug  # noqa: F401
        except ImportError:  # pragma: no cover - flask/werkzeug is a declared dep
            self.skipTest("werkzeug not installed")
        payload = self._first_sse_payload(
            self._telem(wp_heading_error_deg=-12.345, nav_state="ALIGN")
        )
        self.assertEqual(payload["nav_state"], "ALIGN")
        self.assertAlmostEqual(payload["wp_heading_error_deg"], -12.3, places=6)

    def test_missing_values_serialize_as_none(self):
        try:
            import werkzeug  # noqa: F401
        except ImportError:  # pragma: no cover
            self.skipTest("werkzeug not installed")
        payload = self._first_sse_payload(self._telem())
        self.assertIsNone(payload["nav_state"])
        self.assertIsNone(payload["wp_heading_error_deg"])

    def test_non_finite_heading_error_serializes_as_none(self):
        try:
            import werkzeug  # noqa: F401
        except ImportError:  # pragma: no cover
            self.skipTest("werkzeug not installed")
        payload = self._first_sse_payload(
            self._telem(wp_heading_error_deg=float("nan"), nav_state="DRIVE")
        )
        self.assertIsNone(payload["wp_heading_error_deg"])
        self.assertEqual(payload["nav_state"], "DRIVE")


class TestPropertyMapMarkerHeading(unittest.TestCase):
    def test_uses_corrected_heading_when_the_offset_is_locked(self):
        from pi_app.web import property_map

        src = inspect.getsource(property_map)
        self.assertIn(
            "d.heading_offset_locked && d.corrected_heading_deg != null",
            src,
            "property_map must prefer the true-north corrected heading",
        )
        self.assertIn("? d.corrected_heading_deg", src)
        self.assertIn(": (d.imu_heading_deg || 0)", src)
        # The old unconditional raw-heading marker must be gone.
        self.assertNotIn("heading: d.imu_heading_deg || 0", src)


if __name__ == "__main__":
    unittest.main()
