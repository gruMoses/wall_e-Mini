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
        self.closed = False

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

    def shutdown(self) -> None:
        self.closed = True


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
    # Test-convenient values, NOT the real config defaults (real: threshold 39.0,
    # floor 30.0, delay 30.0 — see config.VescConfig). The legacy band/timer tests
    # below use a 22.4V test threshold with a low 15.0V floor so their ~20-21V test
    # frames land inside the shutdown band [floor, threshold). delay mirrors the new
    # 30s contract default (was 10.0; contract changed 2026-07-10).
    voltage_shutdown_threshold_v: float = 22.4
    voltage_shutdown_delay_s: float = 30.0
    voltage_shutdown_floor_v: float = 15.0


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

    def test_status4_decode_is_tenths_of_celsius(self):
        """STATUS_4 packing is int16 ×10 → °C (both FET and motor winding)."""
        d = self._driver()
        # 12.3 °C FET, -5.0 °C motor (cold soak / sensor quirk) — signed int16.
        d._parse_status4("left", _status4_frame(2, 12.3, -5.0, 0.0).data, time.monotonic())
        with d._telem_lock:
            self.assertAlmostEqual(d._left_telem.temp_fet_c, 12.3, places=1)
            self.assertAlmostEqual(d._left_telem.temp_motor_c, -5.0, places=1)

    def test_status4_short_frame_does_not_clobber_temps(self):
        d = self._driver()
        now = time.monotonic()
        d._parse_status4("left", _status4_frame(2, 40.0, 50.0, 1.0).data, now)
        d._parse_status4("left", b"\x00\x01\x02", now)  # too short — ignored
        with d._telem_lock:
            self.assertAlmostEqual(d._left_telem.temp_fet_c, 40.0, places=1)
            self.assertAlmostEqual(d._left_telem.temp_motor_c, 50.0, places=1)

    def test_get_telemetry_exposes_fet_and_motor_temps(self):
        """STATUS_4 FET + motor temps must surface on VescTelemetry for the
        debug temperature history graph (and follow-me logs)."""
        d = self._driver()
        now = time.monotonic()
        # RPM frame required so get_telemetry() returns non-None.
        d._parse_status("left", _status_frame(2, 100, 1.0, 0.1).data, now)
        d._parse_status("right", _status_frame(1, 110, 1.1, 0.1).data, now)
        d._parse_status4("left", _status4_frame(2, 41.5, 50.0, 2.0).data, now)
        d._parse_status4("right", _status4_frame(1, 42.0, 51.5, 2.1).data, now)
        telem = d.get_telemetry()
        self.assertIsNotNone(telem)
        self.assertAlmostEqual(telem.left_temp_c, 41.5, places=1)
        self.assertAlmostEqual(telem.right_temp_c, 42.0, places=1)
        self.assertAlmostEqual(telem.left_motor_temp_c, 50.0, places=1)
        self.assertAlmostEqual(telem.right_motor_temp_c, 51.5, places=1)

    def test_get_telemetry_temps_none_until_status4(self):
        """RPM-only traffic must not invent temperatures."""
        d = self._driver()
        now = time.monotonic()
        d._parse_status("left", _status_frame(2, 100, 1.0, 0.1).data, now)
        d._parse_status("right", _status_frame(1, 110, 1.1, 0.1).data, now)
        telem = d.get_telemetry()
        self.assertIsNotNone(telem)
        self.assertIsNone(telem.left_temp_c)
        self.assertIsNone(telem.right_temp_c)
        self.assertIsNone(telem.left_motor_temp_c)
        self.assertIsNone(telem.right_motor_temp_c)

    def test_current_in_parsed(self):
        d = self._driver()
        d._parse_status4("left", _status4_frame(2, 30.0, 40.0, 8.9).data, time.monotonic())
        with d._telem_lock:
            self.assertAlmostEqual(d._left_telem.current_in_a, 8.9, places=1)


class TestVoltageThreshold(unittest.TestCase):
    """In-band shutdown-timer logic: band entry, delay, hysteresis recovery.

    These exercise the [floor, threshold) band with a low 15.0V test floor so
    the ~20-21V test frames stay inside the band (see _Cfg). The contract change
    (2026-07-10) turned the old one-shot OS-shutdown latch into a recoverable
    pack-low latch with NO OS shutdown, so the two tests that used to assert the
    OS-shutdown behavior were rewritten (test_trigger_latches_..., below).
    """

    def _driver(self, threshold: float = 22.4, delay: float = 30.0,
                floor: float = 15.0) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg(
            voltage_shutdown_threshold_v=threshold,
            voltage_shutdown_delay_s=delay,
            voltage_shutdown_floor_v=floor,
        ))
        d._bus = _FakeBus()
        return d

    def test_no_shutdown_when_voltage_ok(self):
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 25.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertFalse(d._pack_low_latched)
        self.assertIsNone(d._low_voltage_since)

    def test_timer_starts_on_low_voltage(self):
        d = self._driver()
        d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
        d._check_voltage_shutdown()
        self.assertFalse(d._pack_low_latched)
        self.assertIsNotNone(d._low_voltage_since)

    def test_no_shutdown_before_delay_expires(self):
        # delay is the new 30s contract default (was 10.0 under the old contract).
        d = self._driver(threshold=22.4, delay=30.0)
        d._parse_status5("left", _status5_frame(2, 21.0).data, time.monotonic())
        # Call check multiple times without advancing time past the delay
        for _ in range(5):
            d._check_voltage_shutdown()
        self.assertFalse(d._pack_low_latched)

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
        self.assertFalse(d._pack_low_latched)

    def test_trigger_is_idempotent_and_never_shuts_down(self):
        """The old one-shot early-return contract is gone — the latch is now
        recoverable, so _check keeps running while latched. Re-entering the
        trigger keeps the latch set, does not raise, and NEVER calls the OS
        shutdown (subprocess.run)."""
        d = self._driver(threshold=22.4, delay=0.0)
        with patch("subprocess.run") as mock_run:
            with patch.object(d, "_send_rpm"):
                d._trigger_low_voltage_shutdown(21.0)
                self.assertTrue(d._pack_low_latched)
                d._trigger_low_voltage_shutdown(21.0)  # idempotent re-entry
                self.assertTrue(d._pack_low_latched)
        mock_run.assert_not_called()

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
        self.assertFalse(d._pack_low_latched)

    def test_no_shutdown_without_voltage_data(self):
        d = self._driver()
        # No status5 frames received — voltage is None
        d._check_voltage_shutdown()
        self.assertFalse(d._pack_low_latched)
        self.assertIsNone(d._low_voltage_since)

    def test_trigger_latches_and_cuts_motors_without_os_shutdown(self):
        """Repurposed from test_trigger_sends_stop_and_sets_event. The trigger
        now engages the recoverable pack-low latch and cuts motors, but must NOT
        set the stop-event (process keeps running) and must NOT invoke the OS
        shutdown (subprocess.run)."""
        d = self._driver()
        d._stop_event = threading.Event()
        sent: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent.append(a)):
            with patch("subprocess.run") as mock_run:
                d._trigger_low_voltage_shutdown(21.0)
        self.assertTrue(d._pack_low_latched)
        self.assertFalse(d._stop_event.is_set())   # process NOT killed
        mock_run.assert_not_called()                # NO OS shutdown
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
            d.shutdown()

    def test_rx_thread_parses_status5_voltage(self):
        d = self._started_driver()
        try:
            _inject(d, _status5_frame(2, 24.2))
            self.assertAlmostEqual(d.get_voltage(), 24.2, places=1)
        finally:
            d.shutdown()

    def test_rx_thread_ignores_unknown_vesc_id(self):
        d = self._started_driver()
        try:
            _inject(d, _status_frame(99, 9999, 0.0, 0.0))
            self.assertIsNone(d.get_rpm("left"))
            self.assertIsNone(d.get_rpm("right"))
        finally:
            d.shutdown()

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
            d.shutdown()

    def test_stop_joins_thread(self):
        d = self._started_driver()
        d.stop()
        self.assertIsNotNone(d._rx_thread)
        self.assertTrue(d._rx_thread.is_alive())
        d.shutdown()
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
            d.shutdown()

    def test_rx_health_counts_frames(self):
        d = self._started_driver()
        try:
            _inject(d, _status_frame(2, 1000, 2.0, 0.1), _status_frame(1, 900, 2.0, 0.1))
            h = d.get_rx_health()
            self.assertGreaterEqual(h["rx_frame_count"], 2)
            self.assertIsInstance(h["rx_last_frame_age_s"], float)
            self.assertEqual(h["rx_parse_error_count"], 0)
            self.assertEqual(h["rx_recv_error_count"], 0)
        finally:
            d.shutdown()

    def test_rx_health_per_status_type_counters(self):
        """Per-status-type counters distinguish STATUS(9)/STATUS_4(16)/STATUS_5(27)
        so ERPM packet flow is provable independently of the other frame types."""
        d = self._started_driver()
        try:
            _inject(
                d,
                _status_frame(2, 1000, 2.0, 0.1),    # STATUS (9), left
                _status_frame(1, 900, 2.0, 0.1),     # STATUS (9), right
                _status4_frame(2, 40.0, 38.0, 5.0),  # STATUS_4 (16)
                _status5_frame(2, 24.0),             # STATUS_5 (27)
            )
            h = d.get_rx_health()
            self.assertEqual(h["rx_status_count"], 2)
            self.assertEqual(h["rx_status4_count"], 1)
            self.assertEqual(h["rx_status5_count"], 1)
            # Aggregate count equals the sum of the per-type counts.
            self.assertEqual(
                h["rx_frame_count"],
                h["rx_status_count"] + h["rx_status4_count"] + h["rx_status5_count"],
            )
        finally:
            d.shutdown()

    def test_rx_reopens_bus_on_recv_error(self):
        d = self._started_driver()
        try:
            original_bus = d._bus
            self.assertIsNotNone(original_bus)

            def _boom(timeout=0.1):
                raise RuntimeError("forced recv failure")

            d._bus.recv = _boom  # type: ignore[method-assign]
            time.sleep(0.2)
            h = d.get_rx_health()
            self.assertGreaterEqual(h["rx_recv_error_count"], 1)
            self.assertGreaterEqual(h["rx_reopen_count"], 1)
            self.assertIsNot(d._bus, original_bus)
        finally:
            d.shutdown()

    def test_pack_low_latch_fires_via_rx_thread(self):
        # 20.0V sits in the band [15.0, 22.4) (test floor 15.0), so a sustained
        # in-band sag latches pack_low WITHOUT any OS shutdown or stop-event.
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg(
            voltage_shutdown_threshold_v=22.4,
            voltage_shutdown_delay_s=0.05,
            voltage_shutdown_floor_v=15.0,
        ))
        d._bus = _FakeBus()
        with patch("subprocess.run") as mock_run:
            d.start()
            try:
                # Repeatedly inject in-band frames to keep the sustain timer advancing
                for _ in range(20):
                    d._bus.push(_status5_frame(2, 20.0))
                    time.sleep(0.01)
                time.sleep(0.2)
                self.assertTrue(d._pack_low_latched)
                # Watchdog must NOT kill the process or shell out to shutdown.
                self.assertFalse(d._stop_event.is_set())  # type: ignore[union-attr]
                mock_run.assert_not_called()
            finally:
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


class TestPerMotorStaleness(unittest.TestCase):
    """Wave 2: get_telemetry() must expose per-motor STATUS(9) frame age so a
    consumer can detect ONE motor going silent even while the other motor's
    traffic keeps get_telemetry() returning non-None."""

    def _driver(self) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        d._bus = _FakeBus()
        return d

    def test_status_age_none_before_any_frame(self):
        d = self._driver()
        # Right motor has reported; left never has -> get_telemetry() is non-None
        # (aggregate rpm check) but left_status_age_s must stay None.
        d._parse_status("right", _status_frame(1, 500, 1.0, 0.1).data, time.monotonic())
        telem = d.get_telemetry()
        self.assertIsNotNone(telem)
        self.assertIsNone(telem.left_status_age_s)
        self.assertIsNotNone(telem.right_status_age_s)

    def test_status_age_populated_for_both_motors(self):
        d = self._driver()
        now = time.monotonic()
        d._parse_status("left", _status_frame(2, 1000, 1.0, 0.1).data, now)
        d._parse_status("right", _status_frame(1, 900, 1.0, 0.1).data, now)
        telem = d.get_telemetry()
        self.assertIsNotNone(telem.left_status_age_s)
        self.assertIsNotNone(telem.right_status_age_s)
        self.assertGreaterEqual(telem.left_status_age_s, 0.0)
        self.assertGreaterEqual(telem.right_status_age_s, 0.0)

    def test_one_motor_frozen_shows_larger_age_than_fresh_motor(self):
        """Core bug scenario: right motor keeps transmitting; left froze a
        while ago. get_telemetry() must NOT return None (old bug), but the
        per-motor ages must clearly distinguish stale left from fresh right."""
        d = self._driver()
        stale_time = time.monotonic() - 2.0  # left's last frame was 2s ago
        d._parse_status("left", _status_frame(2, 1234, 1.0, 0.1).data, stale_time)
        d._parse_status("right", _status_frame(1, 900, 1.0, 0.1).data, time.monotonic())
        telem = d.get_telemetry()
        self.assertIsNotNone(telem)  # aggregate check still passes (old behavior)
        self.assertGreaterEqual(telem.left_status_age_s, 1.9)
        self.assertLess(telem.right_status_age_s, 0.5)

    def test_rx_thread_alive_true_when_running(self):
        d = self._driver()
        d.start()
        try:
            d._parse_status("left", _status_frame(2, 100, 0.0, 0.0).data, time.monotonic())
            telem = d.get_telemetry()
            self.assertTrue(telem.rx_thread_alive)
            health = d.get_rx_health()
            self.assertTrue(health["rx_thread_alive"])
        finally:
            d.shutdown()

    def test_rx_thread_alive_none_when_never_started(self):
        d = self._driver()
        d._parse_status("left", _status_frame(2, 100, 0.0, 0.0).data, time.monotonic())
        telem = d.get_telemetry()
        self.assertIsNone(telem.rx_thread_alive)


class TestEnsureBusExceptionContainment(unittest.TestCase):
    """Wave 2: _ensure_bus() failures (SocketCAN OSError/CanError, not just
    RuntimeError) must never escape _send_rpm -> set_tracks -> the caller.
    Previously only RuntimeError was caught, so a real bus-open failure would
    propagate all the way to the unguarded main loop and kill the process."""

    def test_os_error_in_ensure_bus_does_not_propagate_from_send_rpm(self):
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        with patch.object(d, "_ensure_bus", side_effect=OSError("SocketCAN: no such device")):
            try:
                d._send_rpm(d._left_id, 500)
            except Exception as exc:  # pragma: no cover - failure path
                self.fail(f"_send_rpm must contain _ensure_bus OSError, but raised: {exc!r}")

    def test_os_error_in_ensure_bus_does_not_propagate_from_set_tracks(self):
        """The main-loop-facing entry point (set_tracks) must survive too."""
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        with patch.object(d, "_ensure_bus", side_effect=OSError("SocketCAN: no such device")):
            try:
                d.set_tracks(200, 50)
            except Exception as exc:  # pragma: no cover - failure path
                self.fail(f"set_tracks must survive a CAN bus-open failure, but raised: {exc!r}")

    def test_can_error_in_ensure_bus_counts_as_send_failure(self):
        """A contained bus-open failure should still be visible via the existing
        send-failure counters (consistent with the send-path failure handling)."""

        class _CanError(Exception):
            pass

        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        with patch.object(d, "_ensure_bus", side_effect=_CanError("bus init failed")):
            for _ in range(d._SEND_FAIL_THRESHOLD):
                d._send_rpm(d._left_id, 100)
        with d._telem_lock:
            self.assertGreaterEqual(d._send_fail_count, d._SEND_FAIL_THRESHOLD)
            self.assertTrue(d._send_fail_alert)

    def test_runtime_error_in_ensure_bus_still_contained(self):
        """Backward-compat: the original RuntimeError (python-can missing)
        path must still be silently contained, same as before."""
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        with patch.object(d, "_ensure_bus", side_effect=RuntimeError("python-can missing")):
            try:
                d._send_rpm(d._left_id, 100)
            except Exception as exc:  # pragma: no cover - failure path
                self.fail(f"_send_rpm must contain RuntimeError, but raised: {exc!r}")


class TestRxLoopStaleBlockContainment(unittest.TestCase):
    """Wave 2: the `msg is None` staleness-reopen block inside _rx_loop was the
    one segment NOT wrapped in try/except — an exception there would kill the
    daemon RX thread silently. It must now be contained like every sibling
    block in the loop, and the RX thread must keep running afterward."""

    def test_exception_in_stale_check_block_does_not_kill_rx_thread(self):
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg())
        d._bus = _FakeBus()

        # Force the `msg is None` branch to execute and make the staleness
        # check inside it raise, by poisoning _rx_last_reopen_s access via a
        # broken telem lock replacement is too invasive; instead corrupt
        # _rx_last_frame_s state so the "then" branch runs, and monkeypatch
        # time.monotonic to raise on the second call within that block.
        d._rx_last_frame_s = time.monotonic() - 5.0  # looks stale -> enters warn path
        d._rx_last_reopen_s = 0.0

        call_count = {"n": 0}
        real_monotonic = time.monotonic

        def _flaky_monotonic():
            call_count["n"] += 1
            if call_count["n"] == 2:
                raise RuntimeError("simulated clock failure inside stale-check block")
            return real_monotonic()

        d.start()
        try:
            with patch("pi_app.hardware.vesc.time.monotonic", side_effect=_flaky_monotonic):
                # Give the RX thread a couple of iterations to hit the poisoned path.
                deadline = real_monotonic() + 1.0
                while call_count["n"] < 2 and real_monotonic() < deadline:
                    time.sleep(0.01)
                time.sleep(0.1)
            # The RX thread must still be alive — the exception must have been
            # contained, not propagated out of _rx_loop.
            self.assertTrue(d._rx_thread.is_alive())
            # And normal operation must continue afterward (fresh frames still parse).
            _inject(d, _status_frame(2, 777, 0.0, 0.0))
            self.assertEqual(d.get_rpm("left"), 777)
        finally:
            d.shutdown()


class TestPackLowLatch(unittest.TestCase):
    """Revised low-voltage contract (2026-07-10): plausibility floor + a
    recoverable pack-low latch, and NO OS shutdown. Uses realistic 13S voltages
    (floor 30.0, threshold 39.0) matching the field incident.

    Regimes:
      * V < floor (30.0)              -> pack switched off / garbage: motors
                                         neutral, log once, NO latch, NO shutdown.
      * 30.0 <= V < 39.0 for delay s  -> pack-low latch engaged, motors cut,
                                         NO OS shutdown, process keeps running.
      * V >= 39.0 + hysteresis (2.0)  -> latch clears, motors re-enabled.
    """

    LOGGER = "pi_app.hardware.vesc"

    def _driver(self, threshold: float = 39.0, floor: float = 30.0,
                delay: float = 30.0) -> VescCanDriver:
        d = VescCanDriver(left_id=2, right_id=1, vesc_cfg=_Cfg(
            voltage_shutdown_threshold_v=threshold,
            voltage_shutdown_floor_v=floor,
            voltage_shutdown_delay_s=delay,
        ))
        d._bus = _FakeBus()
        d._stop_event = threading.Event()
        return d

    def _set_v(self, d: VescCanDriver, v: float) -> None:
        d._parse_status5("left", _status5_frame(2, v).data, time.monotonic())

    # (a) 6.8V reading -> no shutdown, motors stopped, log emitted, no latch.
    def test_below_floor_no_latch_motors_stopped_logged(self):
        d = self._driver()
        sent: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent.append(a)):
            with patch("subprocess.run") as mock_run:
                with self.assertLogs(self.LOGGER, level="WARNING") as cm:
                    self._set_v(d, 6.8)      # the exact field-incident reading
                    d._check_voltage_shutdown()
        self.assertFalse(d._pack_low_latched)
        self.assertIsNone(d._low_voltage_since)
        self.assertIn((d._left_id, 0), sent)
        self.assertIn((d._right_id, 0), sent)
        mock_run.assert_not_called()
        self.assertTrue(any("plausibility floor" in m for m in cm.output))

    def test_below_floor_logs_only_once_per_episode(self):
        d = self._driver()
        with patch.object(d, "_send_rpm") as mock_send:
            with self.assertLogs(self.LOGGER, level="WARNING") as cm:
                for _ in range(5):
                    self._set_v(d, 6.8)
                    d._check_voltage_shutdown()
        floor_logs = [m for m in cm.output if "plausibility floor" in m]
        self.assertEqual(len(floor_logs), 1)          # one line, not spammed
        self.assertEqual(mock_send.call_count, 2)     # neutral sent once (L+R)

    # (b) 38V sustained for the delay -> motors stopped + pack_low_latched, NO shutdown.
    def test_band_sustained_latches_and_cuts_motors_no_shutdown(self):
        d = self._driver(delay=0.05)
        sent: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent.append(a)):
            with patch("subprocess.run") as mock_run:
                self._set_v(d, 38.0)
                d._check_voltage_shutdown()          # starts the sustain timer
                self.assertFalse(d._pack_low_latched)
                time.sleep(0.1)
                self._set_v(d, 38.0)
                d._check_voltage_shutdown()          # delay elapsed -> latch
        self.assertTrue(d._pack_low_latched)
        self.assertFalse(d._stop_event.is_set())     # process NOT killed
        self.assertIn((d._left_id, 0), sent)
        self.assertIn((d._right_id, 0), sent)
        mock_run.assert_not_called()                 # NO OS shutdown

    # (c) 38V (timer running) then recovery to 45V -> timer resets, no latch.
    def test_band_then_recovery_resets_timer(self):
        d = self._driver(delay=30.0)
        self._set_v(d, 38.0)
        d._check_voltage_shutdown()
        self.assertIsNotNone(d._low_voltage_since)    # timer running
        self._set_v(d, 45.0)
        d._check_voltage_shutdown()
        self.assertIsNone(d._low_voltage_since)        # timer reset
        self.assertFalse(d._pack_low_latched)

    # (d) 38V (timer running) then drop below floor to 6V -> countdown CANCELLED.
    def test_band_then_below_floor_cancels_countdown(self):
        d = self._driver(delay=30.0)
        self._set_v(d, 38.0)
        d._check_voltage_shutdown()
        self.assertIsNotNone(d._low_voltage_since)
        with patch.object(d, "_send_rpm"):
            self._set_v(d, 6.0)
            d._check_voltage_shutdown()
        self.assertIsNone(d._low_voltage_since)        # countdown cancelled
        self.assertFalse(d._pack_low_latched)          # never latched

    # (e) below-floor then recovery above threshold -> normal monitoring resumes.
    def test_below_floor_then_recovery_resumes_monitoring(self):
        d = self._driver(delay=30.0)
        with patch.object(d, "_send_rpm"):
            self._set_v(d, 6.0)
            d._check_voltage_shutdown()
            self.assertTrue(d._below_floor_active)
            self._set_v(d, 45.0)                        # recover above threshold
            d._check_voltage_shutdown()
            self.assertFalse(d._below_floor_active)
            # A fresh in-band sag now starts a new countdown (monitoring resumed).
            self._set_v(d, 38.0)
            d._check_voltage_shutdown()
        self.assertIsNotNone(d._low_voltage_since)
        self.assertFalse(d._pack_low_latched)

    # (f) delay is config-driven, and the real config defaults are floor=30/delay=30.
    def test_real_config_defaults(self):
        from config import VescConfig
        cfg = VescConfig()
        self.assertEqual(cfg.voltage_shutdown_floor_v, 30.0)
        self.assertEqual(cfg.voltage_shutdown_delay_s, 30.0)
        self.assertEqual(cfg.voltage_shutdown_threshold_v, 39.0)

    def test_delay_is_config_driven(self):
        d = self._driver(delay=0.2)
        self._set_v(d, 38.0)
        d._check_voltage_shutdown()                    # start timer
        d._check_voltage_shutdown()                    # still within the delay
        self.assertFalse(d._pack_low_latched)
        time.sleep(0.25)
        self._set_v(d, 38.0)
        d._check_voltage_shutdown()                    # delay exceeded -> latch
        self.assertTrue(d._pack_low_latched)

    # Recovery: latched, then 45V -> latch clears, motors no longer forced.
    def test_latch_clears_on_recovery_and_reenables_motors(self):
        d = self._driver(delay=0.0)
        with patch.object(d, "_send_rpm"):
            d._trigger_low_voltage_shutdown(38.0)      # force the latch
        self.assertTrue(d._pack_low_latched)
        # While latched, set_tracks() forces neutral regardless of command.
        sent: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent.append(a)):
            d.set_tracks(200, 200)
        self.assertEqual(sent, [(d._left_id, 0), (d._right_id, 0)])
        # Recovery above threshold + hysteresis clears the latch.
        self._set_v(d, 45.0)
        d._check_voltage_shutdown()
        self.assertFalse(d._pack_low_latched)
        # Motors re-enabled: set_tracks() now sends the commanded (non-neutral) rpm.
        sent2: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent2.append(a)):
            d.set_tracks(200, 200)
        self.assertTrue(any(rpm != 0 for (_id, rpm) in sent2))

    def test_latch_holds_within_hysteresis_deadband(self):
        """Recovering to >= threshold but < threshold + hysteresis clears the
        sustain timer but must NOT release the latch (anti-chatter)."""
        d = self._driver(delay=0.0)
        with patch.object(d, "_send_rpm"):
            d._trigger_low_voltage_shutdown(38.0)
        self.assertTrue(d._pack_low_latched)
        self._set_v(d, 40.0)   # >= threshold (39) but < threshold+hyst (41)
        d._check_voltage_shutdown()
        self.assertIsNone(d._low_voltage_since)
        self.assertTrue(d._pack_low_latched)

    def test_latch_survives_can_dropout(self):
        """QA pin: while latched, a CAN dropout (voltage=None — no usable
        frames) must NOT release the latch or mutate any watchdog state — the
        monitor early-returns before touching anything. A dropout mid-latch
        would otherwise silently re-enable motors on a critically low pack."""
        d = self._driver()
        with patch.object(d, "_send_rpm"):
            d._trigger_low_voltage_shutdown(38.0)
        self.assertTrue(d._pack_low_latched)
        # No STATUS_5 voltage available -> _get_pack_voltage() returns None.
        self.assertIsNone(d._get_pack_voltage())
        before_since = d._low_voltage_since
        before_floor_flag = d._below_floor_active
        d._check_voltage_shutdown()                    # must be a no-op
        self.assertTrue(d._pack_low_latched)           # no false release
        self.assertEqual(d._low_voltage_since, before_since)
        self.assertEqual(d._below_floor_active, before_floor_flag)
        # Driver-level enforcement still holds during the dropout.
        sent: list = []
        with patch.object(d, "_send_rpm", side_effect=lambda *a: sent.append(a)):
            d.set_tracks(220, 220)
        self.assertEqual(sent, [(d._left_id, 0), (d._right_id, 0)])

    def test_pack_low_latched_surfaced_on_telemetry(self):
        d = self._driver(delay=0.0)
        # Need an RPM frame so get_telemetry() returns non-None.
        d._parse_status("left", _status_frame(2, 100, 0.0, 0.0).data, time.monotonic())
        self.assertFalse(d.get_pack_low_latched())
        with patch.object(d, "_send_rpm"):
            d._trigger_low_voltage_shutdown(38.0)
        vt = d.get_telemetry()
        self.assertIsNotNone(vt)
        self.assertTrue(vt.pack_low_latched)
        self.assertTrue(d.get_pack_low_latched())


if __name__ == "__main__":
    unittest.main()
