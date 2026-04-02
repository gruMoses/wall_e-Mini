"""
Tests for VESC CAN telemetry: frame parsing, voltage shutdown logic,
telemetry API thread-safety, and state management.

The python-can library is mocked so these tests run without real hardware.
"""

from __future__ import annotations

import struct
import sys
import threading
import time
import types
import unittest
from dataclasses import dataclass
from typing import Optional
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# Mock python-can before importing the VESC driver
# ---------------------------------------------------------------------------

class _FakeMessage:
    def __init__(self, arbitration_id: int, data: bytes, is_extended_id: bool = True):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class _FakeBus:
    def __init__(self, *args, **kwargs):
        self._queue: list[_FakeMessage] = []
        self.sent: list[_FakeMessage] = []

    def recv(self, timeout: float = 0.1) -> Optional[_FakeMessage]:
        if self._queue:
            return self._queue.pop(0)
        time.sleep(min(timeout, 0.01))
        return None

    def send(self, msg: _FakeMessage) -> None:
        self.sent.append(msg)

    def push(self, msg: _FakeMessage) -> None:
        """Test helper: enqueue a frame for the RX thread to consume."""
        self._queue.append(msg)


fake_can = types.ModuleType("can")
fake_can.Message = _FakeMessage
fake_can.interface = types.SimpleNamespace(Bus=lambda **kw: _FakeBus())
sys.modules.setdefault("can", fake_can)

# Now safe to import
from pi_app.hardware.vesc import VescCanDriver, _CAN_PACKET_STATUS, _CAN_PACKET_STATUS_4, _CAN_PACKET_STATUS_5  # noqa: E402


# ---------------------------------------------------------------------------
# Minimal VescConfig stand-in (avoids importing full config in tests)
# ---------------------------------------------------------------------------

@dataclass
class _Cfg:
    voltage_shutdown_threshold_v: float = 22.4
    voltage_shutdown_delay_s: float = 10.0


# ---------------------------------------------------------------------------
# Frame construction helpers
# ---------------------------------------------------------------------------

def _make_frame(packet_id: int, vesc_id: int, data: bytes) -> _FakeMessage:
    arb_id = (packet_id << 8) | vesc_id
    return _FakeMessage(arbitration_id=arb_id, data=data, is_extended_id=True)


def _status_frame(vesc_id: int, rpm: int, current_a: float, duty: float) -> _FakeMessage:
    data = (
        struct.pack(">i", rpm)
        + struct.pack(">h", int(current_a * 10))
        + struct.pack(">h", int(duty * 1000))
    )
    return _make_frame(_CAN_PACKET_STATUS, vesc_id, data)


def _status4_frame(vesc_id: int, temp_fet: float, temp_motor: float, current_in: float) -> _FakeMessage:
    data = (
        struct.pack(">h", int(temp_fet * 10))
        + struct.pack(">h", int(temp_motor * 10))
        + struct.pack(">h", int(current_in * 10))
        + struct.pack(">h", 0)  # pid_pos_err (unused)
    )
    return _make_frame(_CAN_PACKET_STATUS_4, vesc_id, data)


def _status5_frame(vesc_id: int, voltage_v: float) -> _FakeMessage:
    # Layout per observed firmware: [0:4] tachometer, [4:6] input_voltage × 10
    tach = struct.pack(">i", 0)
    volt = struct.pack(">h", int(voltage_v * 10))
    return _make_frame(_CAN_PACKET_STATUS_5, vesc_id, tach + volt)


# ---------------------------------------------------------------------------
# Helper: inject frames into the driver's bus queue
# ---------------------------------------------------------------------------

def _inject(driver: VescCanDriver, *frames: _FakeMessage) -> None:
    """Push frames onto the fake bus and wait for the RX thread to consume them."""
    bus: _FakeBus = driver._bus  # type: ignore[assignment]
    for f in frames:
        bus.push(f)
    # Wait until queue is drained (up to 1 s)
    deadline = time.monotonic() + 1.0
    while bus._queue and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.02)  # small extra margin for parser to finish


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestFrameParsing(unittest.TestCase):
    """Direct parser tests — no threading required."""

    def _driver(self) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        # Provide a fake bus so _ensure_bus() doesn't try to open SocketCAN
        d._bus = _FakeBus()
        return d

    def test_status_left_motor(self):
        d = self._driver()
        d._parse_status("left", _status_frame(2, 1234, 5.6, 0.42).data, time.monotonic())
        self.assertEqual(d.get_rpm("left"), 1234)
        self.assertAlmostEqual(d.get_current("left"), 5.6, places=1)

    def test_status_right_motor(self):
        d = self._driver()
        d._parse_status("right", _status_frame(1, -800, 12.3, 0.1).data, time.monotonic())
        self.assertEqual(d.get_rpm("right"), -800)
        self.assertAlmostEqual(d.get_current("right"), 12.3, places=1)

    def test_status4_temperatures(self):
        d = self._driver()
        d._parse_status4("left", _status4_frame(2, 41.5, 38.0, 7.2).data, time.monotonic())
        self.assertAlmostEqual(d.get_temperature("left"), 41.5, places=1)

    def test_status5_voltage_left(self):
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 24.6).data, time.monotonic())
        self.assertAlmostEqual(d.get_voltage(), 24.6, places=1)

    def test_status5_voltage_right(self):
        d = self._driver()
        d._parse_status5("right", _status5_frame(1, 23.1).data, time.monotonic())
        self.assertAlmostEqual(d.get_voltage(), 23.1, places=1)

    def test_voltage_max_of_two(self):
        """get_voltage() returns the higher reading to avoid false shutdown triggers."""
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 22.0).data, time.monotonic())
        d._parse_status5("right", _status5_frame(1, 23.5).data, time.monotonic())
        self.assertAlmostEqual(d.get_voltage(), 23.5, places=1)

    def test_voltage_none_when_no_frames(self):
        d = self._driver()
        self.assertIsNone(d.get_voltage())

    def test_short_frame_ignored(self):
        d = self._driver()
        # Too short — should not raise, just leave state unchanged
        d._parse_status("left", b"\x00\x01", time.monotonic())
        self.assertIsNone(d.get_rpm("left"))

    def test_status5_short_frame_ignored(self):
        d = self._driver()
        d._parse_status5("left", b"\x00" * 5, time.monotonic())  # needs 6 bytes
        self.assertIsNone(d.get_voltage())

    def test_negative_rpm(self):
        d = self._driver()
        d._parse_status("right", _status_frame(1, -3000, 0.0, -0.5).data, time.monotonic())
        self.assertEqual(d.get_rpm("right"), -3000)

    def test_duty_cycle_parsed(self):
        d = self._driver()
        now = time.monotonic()
        d._parse_status("left", _status_frame(2, 0, 0.0, 0.75).data, now)
        with d._telem_lock:
            self.assertAlmostEqual(d._left_telem.duty_cycle, 0.75, places=2)

    def test_motor_temp_parsed(self):
        d = self._driver()
        d._parse_status4("right", _status4_frame(1, 35.0, 55.5, 3.0).data, time.monotonic())
        with d._telem_lock:
            self.assertAlmostEqual(d._right_telem.temp_motor_c, 55.5, places=1)

    def test_current_in_parsed(self):
        d = self._driver()
        d._parse_status4("left", _status4_frame(2, 30.0, 40.0, 8.9).data, time.monotonic())
        with d._telem_lock:
            self.assertAlmostEqual(d._left_telem.current_in_a, 8.9, places=1)


class TestVoltageThreshold(unittest.TestCase):
    """Voltage monitor logic: threshold, delay, hysteresis, one-shot latch."""

    def _driver(self, threshold: float = 22.4, delay: float = 10.0) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg(
            voltage_shutdown_threshold_v=threshold,
            voltage_shutdown_delay_s=delay,
        ))
        d._bus = _FakeBus()
        return d

    def test_no_shutdown_when_voltage_ok(self):
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 25.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertFalse(d._shutdown_triggered)
        self.assertIsNone(d._low_voltage_since)

    def test_timer_starts_on_low_voltage(self):
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertFalse(d._shutdown_triggered)
        self.assertIsNotNone(d._low_voltage_since)

    def test_no_shutdown_before_delay_expires(self):
        d = self._driver(threshold=22.4, delay=10.0)
        d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
        # Call check multiple times without advancing time past the delay
        for _ in range(5):
            d._check_voltage_shutdown()
        self.assertFalse(d._shutdown_triggered)

    def test_shutdown_triggers_after_delay(self):
        d = self._driver(threshold=22.4, delay=0.05)
        with patch.object(d, "_trigger_low_voltage_shutdown") as mock_trigger:
            d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
            d._check_voltage_shutdown()
            time.sleep(0.1)
            d._check_voltage_shutdown()
            mock_trigger.assert_called_once()

    def test_timer_clears_on_voltage_recovery(self):
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertIsNotNone(d._low_voltage_since)
        # Voltage recovers
        d._parse_status5("left", _status5_frame(2, 25.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertIsNone(d._low_voltage_since)
        self.assertFalse(d._shutdown_triggered)

    def test_shutdown_latch_prevents_repeat_trigger(self):
        d = self._driver(threshold=22.4, delay=0.0)
        with patch.object(d, "_trigger_low_voltage_shutdown") as mock_trigger:
            d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
            # Force the latch directly
            d._shutdown_triggered = True
            d._check_voltage_shutdown()
            mock_trigger.assert_not_called()

    def test_shutdown_at_exact_threshold_not_triggered(self):
        """Voltage exactly at threshold is NOT below it — no trigger."""
        d = self._driver(threshold=22.4)
        d._parse_status5("left", _status5_frame(2, 22.4).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertIsNone(d._low_voltage_since)

    def test_no_shutdown_without_cfg(self):
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=None)
        d._bus = _FakeBus()
        d._parse_status5("left", _status5_frame(2, 10.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertFalse(d._shutdown_triggered)

    def test_no_shutdown_without_voltage_data(self):
        d = self._driver()
        # No status5 frames received — voltage is None
        d._check_voltage_shutdown()
        self.assertFalse(d._shutdown_triggered)
        self.assertIsNone(d._low_voltage_since)

    def test_trigger_sends_stop_and_sets_event(self):
        d = self._driver()
        d._stop_event = threading.Event()
        sent: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent.append(a)):
            with patch("subprocess.run"):
                d._trigger_low_voltage_shutdown(21.0)
        self.assertTrue(d._shutdown_triggered)
        self.assertTrue(d._stop_event.is_set())
        # Both motors should receive RPM=0
        self.assertIn((d._left_id, 0), sent)
        self.assertIn((d._right_id, 0), sent)


class TestRxThread(unittest.TestCase):
    """Integration tests: RX thread picks up frames from the fake bus."""

    def _started_driver(self) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        # Pre-inject a fake bus so the thread uses it immediately
        d._bus = _FakeBus()
        d.start()
        return d

    def test_rx_thread_parses_status_frame(self):
        d = self._started_driver()
        try:
            _inject(d, _status_frame(2, 1500, 4.0, 0.3))
            self.assertEqual(d.get_rpm("left"), 1500)
        finally:
            d.stop()

    def test_rx_thread_parses_status5_voltage(self):
        d = self._started_driver()
        try:
            _inject(d, _status5_frame(2, 24.2))
            self.assertAlmostEqual(d.get_voltage(), 24.2, places=1)
        finally:
            d.stop()

    def test_rx_thread_ignores_unknown_vesc_id(self):
        d = self._started_driver()
        try:
            _inject(d, _status_frame(99, 9999, 0.0, 0.0))
            self.assertIsNone(d.get_rpm("left"))
            self.assertIsNone(d.get_rpm("right"))
        finally:
            d.stop()

    def test_rx_thread_ignores_non_extended_frame(self):
        d = self._started_driver()
        try:
            frame = _FakeMessage(
                arbitration_id=(_CAN_PACKET_STATUS << 8) | 2,
                data=_status_frame(2, 500, 0.0, 0.0).data,
                is_extended_id=False,
            )
            _inject(d, frame)
            self.assertIsNone(d.get_rpm("left"))
        finally:
            d.stop()

    def test_stop_joins_thread(self):
        d = self._started_driver()
        d.stop()
        self.assertIsNone(d._rx_thread)

    def test_multiple_motors_independent(self):
        d = self._started_driver()
        try:
            _inject(d,
                _status_frame(2, 1000, 3.0, 0.2),   # left
                _status_frame(1, -500, 6.0, -0.1),  # right
            )
            self.assertEqual(d.get_rpm("left"), 1000)
            self.assertEqual(d.get_rpm("right"), -500)
        finally:
            d.stop()

    def test_voltage_shutdown_fires_via_rx_thread(self):
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg(
            voltage_shutdown_threshold_v=22.4,
            voltage_shutdown_delay_s=0.05,
        ))
        d._bus = _FakeBus()
        with patch("subprocess.run"):
            d.start()
            try:
                # Repeatedly inject low-voltage frames to keep timer advancing
                for _ in range(20):
                    d._bus.push(_status5_frame(2, 20.0))
                    time.sleep(0.01)
                time.sleep(0.2)
                self.assertTrue(d._shutdown_triggered)
            finally:
                # stop_event already set by shutdown path; thread may already be gone
                if d._rx_thread and d._rx_thread.is_alive():
                    d._stop_event.set()  # type: ignore[union-attr]
                    d._rx_thread.join(timeout=1.0)


class TestTelemetryApi(unittest.TestCase):
    """Ensure public API methods return correct types and handle None gracefully."""

    def _driver(self) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1)
        d._bus = _FakeBus()
        return d

    def test_initial_state_all_none(self):
        d = self._driver()
        self.assertIsNone(d.get_voltage())
        self.assertIsNone(d.get_rpm("left"))
        self.assertIsNone(d.get_rpm("right"))
        self.assertIsNone(d.get_current("left"))
        self.assertIsNone(d.get_temperature("left"))

    def test_api_after_all_frame_types(self):
        d = self._driver()
        now = time.monotonic()
        d._parse_status("left", _status_frame(2, 2000, 9.0, 0.5).data, now)
        d._parse_status4("left", _status4_frame(2, 42.1, 39.0, 5.5).data, now)
        d._parse_status5("left", _status5_frame(2, 25.3).data, now)
        self.assertEqual(d.get_rpm("left"), 2000)
        self.assertAlmostEqual(d.get_current("left"), 9.0, places=1)
        self.assertAlmostEqual(d.get_temperature("left"), 42.1, places=1)
        self.assertAlmostEqual(d.get_voltage(), 25.3, places=1)

    def test_right_motor_independent_of_left(self):
        d = self._driver()
        now = time.monotonic()
        d._parse_status("left", _status_frame(2, 100, 1.0, 0.1).data, now)
        d._parse_status("right", _status_frame(1, 200, 2.0, 0.2).data, now)
        self.assertEqual(d.get_rpm("left"), 100)
        self.assertEqual(d.get_rpm("right"), 200)

    def test_temperature_right_motor(self):
        d = self._driver()
        d._parse_status4("right", _status4_frame(1, 55.0, 60.0, 4.0).data, time.monotonic())
        self.assertAlmostEqual(d.get_temperature("right"), 55.0, places=1)


if __name__ == "__main__":
    unittest.main()
