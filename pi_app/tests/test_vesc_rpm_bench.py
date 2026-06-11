"""
Tests for tools/vesc_rpm_bench.py.

Follows the mocking pattern from test_vesc_telemetry.py: installs a fake
`can` module in sys.modules before importing the VESC driver so no real
CAN bus is ever opened.

Coverage:
  1. Running without --i-confirm-wheels-are-off-the-ground exits nonzero and
     sends nothing.
  2. With mocked bus + confirm flag, stop (RPM=0) is sent after the normal
     run AND when the sampling loop raises mid-run (try/finally guarantee).
  3. --dry-run opens no bus.
  4. The hard ERPM cap (_ERPM_HARD_CAP) is enforced.
"""

from __future__ import annotations

import struct
import sys
import threading
import time
import types
from typing import Optional
from unittest.mock import patch


# ---------------------------------------------------------------------------
# Install fake `can` module BEFORE importing anything from pi_app or tools
# (mirrors test_vesc_telemetry.py pattern exactly)
# ---------------------------------------------------------------------------

class _FakeMessage:
    def __init__(self, arbitration_id: int, data: bytes, is_extended_id: bool = True):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class _FakeBus:
    """Thread-safe fake CAN bus with queued RX and recorded TX messages."""

    def __init__(self, *args, **kwargs):
        self._queue: list[_FakeMessage] = []
        self._lock = threading.Lock()
        self.sent: list[_FakeMessage] = []
        self.closed = False
        self._open_count = 0
        self._open_count += 1

    def recv(self, timeout: float = 0.1) -> Optional[_FakeMessage]:
        with self._lock:
            if self._queue:
                return self._queue.pop(0)
        time.sleep(min(timeout, 0.01))
        return None

    def send(self, msg: _FakeMessage) -> None:
        with self._lock:
            self.sent.append(msg)

    def push(self, msg: _FakeMessage) -> None:
        with self._lock:
            self._queue.append(msg)

    def shutdown(self) -> None:
        self.closed = True


# Global bus instance so tests can inspect it after a run
_the_bus: Optional[_FakeBus] = None
_bus_open_calls: int = 0


def _make_fake_bus(**kw) -> _FakeBus:
    global _the_bus, _bus_open_calls
    _bus_open_calls += 1
    _the_bus = _FakeBus(**kw)
    return _the_bus


fake_can = types.ModuleType("can")
fake_can.Message = _FakeMessage  # type: ignore[attr-defined]
fake_can.interface = types.SimpleNamespace(Bus=_make_fake_bus)  # type: ignore[attr-defined]
sys.modules.setdefault("can", fake_can)


# ---------------------------------------------------------------------------
# Now safe to import
# ---------------------------------------------------------------------------

import importlib
import os

# Ensure tools/ is importable by path manipulation
_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
_TOOLS_DIR = os.path.join(_REPO_ROOT, "tools")
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import pytest

# Import the bench module (tools/vesc_rpm_bench.py)
import vesc_rpm_bench as bench


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _args(
    confirm: bool = False,
    service_stopped: bool = False,
    dry_run: bool = False,
    rpm: int = 500,
    duration: float = 0.3,
    channel: str = "can0",
    left_id: int = 2,
    right_id: int = 1,
) -> object:
    """Build a minimal args namespace that mimics argparse output."""
    import argparse
    a = argparse.Namespace(
        i_confirm_wheels_are_off_the_ground=confirm,
        service_already_stopped=service_stopped,
        dry_run=dry_run,
        rpm=rpm,
        duration=duration,
        channel=channel,
        left_id=left_id,
        right_id=right_id,
    )
    return a


def _sent_erpms(bus: _FakeBus, can_id: int) -> list[int]:
    """Decode all RPM values sent to a given CAN ID."""
    arb = 0x300 + can_id
    result = []
    with bus._lock:
        for msg in bus.sent:
            if msg.arbitration_id == arb and len(msg.data) >= 4:
                result.append(struct.unpack(">i", msg.data[:4])[0])
    return result


# ---------------------------------------------------------------------------
# Tests: safety gate — no confirm flag
# ---------------------------------------------------------------------------

class TestSafetyGateNoConfirm:
    def test_exits_nonzero_without_confirm(self):
        """Without --i-confirm-wheels-are-off-the-ground, must exit with code != 0."""
        args = _args(confirm=False)
        with pytest.raises(SystemExit) as exc_info:
            bench._check_safety_gates(args)
        assert exc_info.value.code != 0

    def test_main_exits_nonzero_without_confirm(self, capsys):
        """main() must exit nonzero and never reach the bus when confirm is absent."""
        global _bus_open_calls
        calls_before = _bus_open_calls

        # Patch sys.argv for argparse
        with patch.object(sys, "argv", ["vesc_rpm_bench.py"]):
            with pytest.raises(SystemExit) as exc_info:
                bench.main()

        assert exc_info.value.code != 0
        # No bus should have been opened
        assert _bus_open_calls == calls_before, "CAN bus was opened despite missing confirm flag"

    def test_nothing_sent_without_confirm(self):
        """Safety gate must not send any CAN frames when confirm flag is absent."""
        global _the_bus
        _the_bus = None

        with patch.object(sys, "argv", ["vesc_rpm_bench.py"]):
            with pytest.raises(SystemExit):
                bench.main()

        # Either no bus was created at all, or it received no sends
        assert _the_bus is None or len(_the_bus.sent) == 0


# ---------------------------------------------------------------------------
# Tests: stop is ALWAYS sent (try/finally guarantee)
# ---------------------------------------------------------------------------

class TestStopAlwaysSent:
    """Verify that motors are stopped in the try/finally, both on normal exit
    and when the sampling loop raises an exception mid-run."""

    def _run_bench_with_fake_bus(self, rpm: int = 500, duration: float = 0.3) -> _FakeBus:
        """Run run_bench() with a real VescCanDriver over the fake CAN bus.
        Returns the bus that was used.
        """
        global _the_bus
        _the_bus = None

        args = _args(confirm=True, service_stopped=True, rpm=rpm, duration=duration)
        # systemctl check: pretend service is not active so gate passes
        with patch.object(bench, "_is_service_active", return_value=False):
            bench.run_bench(args)

        return _the_bus  # type: ignore[return-value]

    def test_stop_sent_after_normal_run(self):
        """After a normal run, RPM=0 must be sent to both left and right motors."""
        bus = self._run_bench_with_fake_bus(rpm=500, duration=0.3)
        assert bus is not None, "Bus was never created"

        left_vals = _sent_erpms(bus, 2)   # left_id default = 2
        right_vals = _sent_erpms(bus, 1)  # right_id default = 1

        assert 0 in left_vals, f"Left motor never received ERPM=0 (stop). Sent: {left_vals}"
        assert 0 in right_vals, f"Right motor never received ERPM=0 (stop). Sent: {right_vals}"

    def test_stop_sent_when_loop_raises(self):
        """If an exception fires inside the sampling loop, the finally block
        must still command ERPM=0 on both motors.

        We inject the exception by patching VescCanDriver.get_rpm so the crash
        is confined to the main-thread sampling loop (not the RX background
        thread, which also calls time.sleep and would be affected by a global
        time.sleep patch).
        """
        global _the_bus
        _the_bus = None

        args = _args(confirm=True, service_stopped=True, rpm=500, duration=5.0)

        # Raise on the 3rd call to get_rpm so the loop runs a bit first
        call_count = [0]
        from pi_app.hardware.vesc import VescCanDriver  # noqa: PLC0415

        original_get_rpm = VescCanDriver.get_rpm

        def _boom_get_rpm(self, motor):
            call_count[0] += 1
            if call_count[0] >= 3:
                raise RuntimeError("simulated mid-run crash")
            return original_get_rpm(self, motor)

        with patch.object(bench, "_is_service_active", return_value=False):
            with patch.object(VescCanDriver, "get_rpm", _boom_get_rpm):
                with pytest.raises(RuntimeError, match="simulated mid-run crash"):
                    bench.run_bench(args)

        bus = _the_bus
        assert bus is not None, "Bus was never created"

        left_vals = _sent_erpms(bus, 2)
        right_vals = _sent_erpms(bus, 1)

        assert 0 in left_vals, f"Left motor never received ERPM=0 after crash. Sent: {left_vals}"
        assert 0 in right_vals, f"Right motor never received ERPM=0 after crash. Sent: {right_vals}"


# ---------------------------------------------------------------------------
# Tests: --dry-run opens no bus
# ---------------------------------------------------------------------------

class TestDryRun:
    def test_dry_run_opens_no_bus(self, capsys):
        """--dry-run must not open any CAN bus regardless of confirm flags."""
        global _bus_open_calls
        calls_before = _bus_open_calls

        args = _args(dry_run=True, rpm=1000, duration=3.0)
        result = bench.run_dry(args)

        assert result == 0
        assert _bus_open_calls == calls_before, (
            f"CAN bus was opened during dry-run: {_bus_open_calls - calls_before} new opens"
        )

    def test_dry_run_via_main_no_confirm_needed(self, capsys):
        """main() with --dry-run must not require --i-confirm-wheels-are-off-the-ground."""
        with patch.object(sys, "argv", ["vesc_rpm_bench.py", "--dry-run"]):
            exit_code = bench.main()
        assert exit_code == 0

    def test_dry_run_prints_would_send(self, capsys):
        """Dry-run output must describe the RPM command and STOP command frames."""
        with patch.object(sys, "argv", ["vesc_rpm_bench.py", "--dry-run", "--rpm", "1500"]):
            bench.main()
        captured = capsys.readouterr().out
        assert "WOULD SEND" in captured
        assert "1500" in captured or "+1500" in captured
        assert "DRY-RUN" in captured

    def test_dry_run_shows_stop_frame(self, capsys):
        """Dry-run output must also show the STOP (ERPM=0) command."""
        args = _args(dry_run=True, rpm=1500, duration=2.0)
        bench.run_dry(args)
        captured = capsys.readouterr().out
        # The stop frame has ERPM=0
        assert "ERPM=0" in captured or "STOP" in captured


# ---------------------------------------------------------------------------
# Tests: hard ERPM cap
# ---------------------------------------------------------------------------

class TestErpmCap:
    def test_cap_enforced_via_safety_gate(self):
        """Requesting RPM above _ERPM_HARD_CAP must exit nonzero."""
        over_cap = bench._ERPM_HARD_CAP + 1
        args = _args(confirm=True, service_stopped=True, rpm=over_cap)
        with pytest.raises(SystemExit) as exc_info:
            bench._check_safety_gates(args)
        assert exc_info.value.code != 0

    def test_cap_enforced_exactly_at_boundary(self):
        """RPM exactly at the cap must pass the gate without error."""
        args = _args(confirm=True, service_stopped=True, rpm=bench._ERPM_HARD_CAP)
        with patch.object(bench, "_is_service_active", return_value=False):
            # Should not raise
            bench._check_safety_gates(args)

    def test_run_bench_clamps_rpm_to_cap(self):
        """Even if cap check somehow passes, run_bench clamps to _ERPM_HARD_CAP."""
        global _the_bus
        _the_bus = None

        over_cap = bench._ERPM_HARD_CAP + 1000
        args = _args(confirm=True, service_stopped=True, rpm=over_cap, duration=0.2)

        with patch.object(bench, "_is_service_active", return_value=False):
            bench.run_bench(args)

        bus = _the_bus
        assert bus is not None

        # Gather all ERPM values sent (excluding stop=0)
        with bus._lock:
            all_vals = [
                struct.unpack(">i", m.data[:4])[0]
                for m in bus.sent
                if len(m.data) >= 4
            ]
        nonzero = [v for v in all_vals if v != 0]
        for v in nonzero:
            assert abs(v) <= bench._ERPM_HARD_CAP, (
                f"Commanded ERPM {v} exceeds hard cap {bench._ERPM_HARD_CAP}"
            )

    def test_dry_run_cap_respected(self, capsys):
        """Dry-run output must show capped RPM, not the over-cap input."""
        over_cap = bench._ERPM_HARD_CAP + 999
        args = _args(dry_run=True, rpm=over_cap, duration=1.0)
        bench.run_dry(args)
        captured = capsys.readouterr().out
        # The over-cap value should NOT appear as the commanded ERPM
        assert str(over_cap) not in captured or str(bench._ERPM_HARD_CAP) in captured

    def test_main_exits_nonzero_over_cap(self, capsys):
        """Full main() path: over-cap RPM with confirm flags → nonzero exit."""
        over_cap = bench._ERPM_HARD_CAP + 1
        with patch.object(sys, "argv", [
            "vesc_rpm_bench.py",
            "--i-confirm-wheels-are-off-the-ground",
            "--service-already-stopped",
            "--rpm", str(over_cap),
        ]):
            with patch.object(bench, "_is_service_active", return_value=False):
                with pytest.raises(SystemExit) as exc_info:
                    bench.main()
        assert exc_info.value.code != 0


# ---------------------------------------------------------------------------
# Tests: RPM command refreshed during run / stop always last
# ---------------------------------------------------------------------------

class TestCommandRefresh:
    """Verify that the RPM command is re-sent on every sample tick so the VESC
    ~1 s firmware timeout never fires during a normal run, and that the stop
    command (ERPM=0) is always the very last RPM frame sent to each motor."""

    def _run_bench(self, rpm: int = 500, duration: float = 0.5) -> _FakeBus:
        global _the_bus
        _the_bus = None
        args = _args(confirm=True, service_stopped=True, rpm=rpm, duration=duration)
        with patch.object(bench, "_is_service_active", return_value=False):
            bench.run_bench(args)
        return _the_bus  # type: ignore[return-value]

    def test_multiple_rpm_sends_per_side(self):
        """Each motor must receive ≥ duration*5 RPM command sends (10 Hz over the run)."""
        duration = 0.5
        bus = self._run_bench(rpm=500, duration=duration)
        assert bus is not None, "Bus was never created"

        left_vals = _sent_erpms(bus, 2)   # left_id default = 2
        right_vals = _sent_erpms(bus, 1)  # right_id default = 1

        min_expected = int(duration * 5)  # conservative: at least half the 10 Hz ticks

        nonzero_left = [v for v in left_vals if v != 0]
        nonzero_right = [v for v in right_vals if v != 0]

        assert len(nonzero_left) >= min_expected, (
            f"Left motor received only {len(nonzero_left)} nonzero RPM sends "
            f"(expected >= {min_expected} for {duration}s at ~10 Hz). "
            f"All left sends: {left_vals}"
        )
        assert len(nonzero_right) >= min_expected, (
            f"Right motor received only {len(nonzero_right)} nonzero RPM sends "
            f"(expected >= {min_expected} for {duration}s at ~10 Hz). "
            f"All right sends: {right_vals}"
        )

    def test_stop_is_last_send_on_each_side(self):
        """ERPM=0 (stop) must be the final RPM frame sent to each motor."""
        bus = self._run_bench(rpm=500, duration=0.5)
        assert bus is not None, "Bus was never created"

        left_vals = _sent_erpms(bus, 2)
        right_vals = _sent_erpms(bus, 1)

        assert left_vals, "No frames sent to left motor at all"
        assert right_vals, "No frames sent to right motor at all"

        assert left_vals[-1] == 0, (
            f"Last left motor send was ERPM={left_vals[-1]}, expected 0 (stop). "
            f"Full sequence: {left_vals}"
        )
        assert right_vals[-1] == 0, (
            f"Last right motor send was ERPM={right_vals[-1]}, expected 0 (stop). "
            f"Full sequence: {right_vals}"
        )

    def test_stop_last_even_after_exception(self):
        """Even when the loop crashes mid-run, the stop frame must be the last
        frame sent to each motor (try/finally guarantee with refresh)."""
        global _the_bus
        _the_bus = None

        args = _args(confirm=True, service_stopped=True, rpm=500, duration=5.0)

        call_count = [0]
        from pi_app.hardware.vesc import VescCanDriver  # noqa: PLC0415

        original_get_rpm = VescCanDriver.get_rpm

        def _boom_get_rpm(self, motor):
            call_count[0] += 1
            if call_count[0] >= 3:
                raise RuntimeError("simulated crash for refresh test")
            return original_get_rpm(self, motor)

        with patch.object(bench, "_is_service_active", return_value=False):
            with patch.object(VescCanDriver, "get_rpm", _boom_get_rpm):
                with pytest.raises(RuntimeError, match="simulated crash for refresh test"):
                    bench.run_bench(args)

        bus = _the_bus
        assert bus is not None

        left_vals = _sent_erpms(bus, 2)
        right_vals = _sent_erpms(bus, 1)

        assert left_vals and left_vals[-1] == 0, (
            f"Left stop not last after crash. Sent: {left_vals}"
        )
        assert right_vals and right_vals[-1] == 0, (
            f"Right stop not last after crash. Sent: {right_vals}"
        )


# ---------------------------------------------------------------------------
# Tests: service-active gate
# ---------------------------------------------------------------------------

class TestServiceGate:
    def test_exits_when_service_active_no_override(self):
        """If wall-e service is active and --service-already-stopped not given, exit nonzero."""
        args = _args(confirm=True, service_stopped=False, rpm=500)
        with patch.object(bench, "_is_service_active", return_value=True):
            with pytest.raises(SystemExit) as exc_info:
                bench._check_safety_gates(args)
        assert exc_info.value.code != 0

    def test_passes_when_service_active_with_override(self):
        """--service-already-stopped overrides the service-active check."""
        args = _args(confirm=True, service_stopped=True, rpm=500)
        with patch.object(bench, "_is_service_active", return_value=True):
            # Should not raise
            bench._check_safety_gates(args)

    def test_passes_when_systemctl_unavailable(self):
        """On non-Linux (systemctl absent), service check is skipped."""
        args = _args(confirm=True, service_stopped=False, rpm=500)
        with patch.object(bench, "_is_service_active", return_value=None):
            # Should not raise — None means "can't tell", so gate is skipped
            bench._check_safety_gates(args)
