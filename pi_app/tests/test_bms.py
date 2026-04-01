"""
Tests for the Daly BMS service (pi_app/hardware/bms.py).

BLE (bleak) is mocked throughout so these tests run without hardware.
"""

import asyncio
import struct
import sys
import threading
import time
import types
import unittest
from dataclasses import dataclass
from typing import Optional
from unittest.mock import AsyncMock, MagicMock, patch

# ---------------------------------------------------------------------------
# Stub out bleak before importing bms so the module loads on any machine.
# ---------------------------------------------------------------------------
fake_bleak = types.ModuleType("bleak")


class _FakeBleakClient:
    """Minimal BleakClient shim used by tests that need protocol-level control."""

    def __init__(self, address, timeout=15):
        self.address = address
        self.services = []
        self.is_connected = True
        self._notify_handler = None

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        pass

    async def start_notify(self, char, handler):
        self._notify_handler = handler

    async def stop_notify(self, char):
        pass

    async def write_gatt_char(self, char, data, response=False):
        pass


fake_bleak.BleakClient = _FakeBleakClient
fake_bleak.BleakScanner = MagicMock()
sys.modules.setdefault("bleak", fake_bleak)

# Now import the module under test.
from pi_app.hardware.bms import BmsService, BmsState, _DALY_CMD  # noqa: E402


# ---------------------------------------------------------------------------
# Minimal BmsConfig stand-in
# ---------------------------------------------------------------------------

@dataclass
class _BmsCfg:
    enabled: bool = True
    bms_mac_address: str = "AA:BB:CC:DD:EE:FF"
    bms_poll_interval_s: float = 0.05
    bms_timeout_s: float = 30.0
    charger_inhibit_enabled: bool = True


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_daly_response(cmd_byte: int, payload: bytes) -> bytes:
    """Build a minimal Daly response frame: A5 01 <cmd> 08 <8-byte payload> <checksum>."""
    header = bytes([0xA5, 0x01, cmd_byte, 0x08])
    # Checksum is not validated in the parser, so zero is fine for tests.
    return header + payload + bytes([0x00])


def _soc_payload(
    voltage_raw: int = 370,   # 37.0 V  (×0.1)
    current_raw: int = 30100, # 10.0 A charging (30000 offset, ×0.1)
    soc_raw: int = 800,        # 80.0 %  (×0.1)
) -> bytes:
    """8-byte payload for SOC response (0x90)."""
    # Layout: voltage[2] gap[2] current[2] soc[2]
    return struct.pack(">HHHHxx", voltage_raw, 0, current_raw, soc_raw)[:8]


def _cell_range_payload(max_mv: int = 3850, min_mv: int = 3800) -> bytes:
    """8-byte payload for cell voltage range (0x91)."""
    # Layout: max_v[2] max_cell[1] min_v[2] min_cell[1] padding[2]
    buf = bytearray(8)
    struct.pack_into(">H", buf, 0, max_mv)
    buf[2] = 1
    struct.pack_into(">H", buf, 3, min_mv)
    buf[5] = 4
    return bytes(buf)


def _temp_payload(max_raw: int = 65, min_raw: int = 63) -> bytes:
    """8-byte payload for temperature range (0x92). raw = °C + 40."""
    buf = bytearray(8)
    buf[0] = max_raw
    buf[2] = min_raw
    return bytes(buf)


def _mosfet_payload(charge_on: bool = True, discharge_on: bool = True) -> bytes:
    """8-byte payload for MOSFET status (0x93)."""
    buf = bytearray(8)
    buf[0] = (0x01 if charge_on else 0) | (0x02 if discharge_on else 0)
    return bytes(buf)


def _status_payload(num_cells: int = 8, cycles: int = 42) -> bytes:
    """8-byte payload for pack status (0x94)."""
    buf = bytearray(8)
    buf[0] = num_cells
    struct.pack_into(">H", buf, 5, cycles)
    return bytes(buf)


def _errors_payload(flags: int = 0) -> bytes:
    """8-byte payload for errors (0x97)."""
    buf = bytearray(8)
    struct.pack_into(">H", buf, 0, flags)
    return bytes(buf)


# ---------------------------------------------------------------------------
# Unit tests: data parsing
# ---------------------------------------------------------------------------

class TestDalyParsing(unittest.TestCase):
    """Test that _do_poll correctly parses Daly response frames."""

    def _run_poll_with_responses(self, responses: dict) -> BmsState:
        """
        Run BmsService._do_poll() against synthetic responses and return
        the resulting BmsState.

        ``responses`` maps command name → bytes to feed as the notification.
        """
        cfg = _BmsCfg()
        svc = BmsService(cfg)

        async def run():
            buf = bytearray()
            event = asyncio.Event()

            # Fake write_gatt_char: immediately inject the response for the sent cmd
            _cmd_to_name = {v: k for k, v in _DALY_CMD.items()}

            async def fake_write(char, data, response=False):
                name = _cmd_to_name.get(bytes(data))
                if name and name in responses:
                    buf.clear()
                    buf.extend(responses[name])
                    event.set()
                else:
                    # No response configured — leave buf empty so timeout fires
                    pass

            client = MagicMock()
            client.write_gatt_char = fake_write

            await svc._do_poll(client, "fake-char", buf, event)

        asyncio.run(run())
        return svc.get_state()

    def test_soc_parsed(self):
        resp = _make_daly_response(0x90, _soc_payload(370, 30100, 800))
        state = self._run_poll_with_responses({"soc": resp})
        self.assertAlmostEqual(state.pack_voltage_v, 37.0, places=1)
        self.assertAlmostEqual(state.pack_current_a, 10.0, places=1)
        self.assertAlmostEqual(state.soc_pct, 80.0, places=1)

    def test_discharging_current_negative(self):
        # current_raw = 29900 → (29900 - 30000) * 0.1 = -10.0 A
        resp = _make_daly_response(0x90, _soc_payload(current_raw=29900))
        state = self._run_poll_with_responses({"soc": resp})
        self.assertAlmostEqual(state.pack_current_a, -10.0, places=1)

    def test_cell_voltage_range_parsed(self):
        resp = _make_daly_response(0x91, _cell_range_payload(3850, 3800))
        state = self._run_poll_with_responses({"cell_voltage_range": resp})
        self.assertEqual(state.cell_max_mv, 3850)
        self.assertEqual(state.cell_min_mv, 3800)
        self.assertEqual(state.cell_delta_mv, 50)

    def test_temperature_parsed(self):
        # raw 65 → 65-40 = 25°C, raw 63 → 23°C
        resp = _make_daly_response(0x92, _temp_payload(65, 63))
        state = self._run_poll_with_responses({"temp_range": resp})
        self.assertEqual(state.temp_max_c, 25)
        self.assertEqual(state.temp_min_c, 23)

    def test_mosfet_both_on(self):
        resp = _make_daly_response(0x93, _mosfet_payload(charge_on=True, discharge_on=True))
        state = self._run_poll_with_responses({"mosfet_status": resp})
        self.assertTrue(state.charge_fet_on)
        self.assertTrue(state.discharge_fet_on)

    def test_mosfet_charge_off(self):
        resp = _make_daly_response(0x93, _mosfet_payload(charge_on=False, discharge_on=True))
        state = self._run_poll_with_responses({"mosfet_status": resp})
        self.assertFalse(state.charge_fet_on)
        self.assertTrue(state.discharge_fet_on)

    def test_status_parsed(self):
        resp = _make_daly_response(0x94, _status_payload(num_cells=8, cycles=42))
        state = self._run_poll_with_responses({"status": resp})
        self.assertEqual(state.num_cells, 8)
        self.assertEqual(state.cycle_count, 42)

    def test_error_flags_parsed(self):
        # Bit 0 = "Cell OVP", bit 1 = "Cell UVP"
        resp = _make_daly_response(0x97, _errors_payload(flags=0b11))
        state = self._run_poll_with_responses({"errors": resp})
        self.assertIn("Cell OVP", state.error_flags)
        self.assertIn("Cell UVP", state.error_flags)

    def test_no_error_flags(self):
        resp = _make_daly_response(0x97, _errors_payload(flags=0))
        state = self._run_poll_with_responses({"errors": resp})
        self.assertEqual(state.error_flags, [])

    def test_partial_response_ignored(self):
        """A truncated response must not crash the parser; fields stay None."""
        state = self._run_poll_with_responses({"soc": b"\xA5\x01\x90\x08\x00"})
        # Voltage should remain None since the response was too short
        self.assertIsNone(state.pack_voltage_v)


# ---------------------------------------------------------------------------
# Unit tests: is_charging() logic
# ---------------------------------------------------------------------------

class TestIsCharging(unittest.TestCase):

    def _service(self, charger_inhibit_enabled=True, timeout_s=30.0) -> BmsService:
        cfg = _BmsCfg(
            charger_inhibit_enabled=charger_inhibit_enabled,
            bms_timeout_s=timeout_s,
        )
        return BmsService(cfg)

    def test_false_when_inhibit_disabled(self):
        svc = self._service(charger_inhibit_enabled=False)
        # Even if we inject a charging state, inhibit is disabled
        svc._state = BmsState(charge_fet_on=True, pack_current_a=5.0)
        svc._last_success_monotonic = time.monotonic()
        self.assertFalse(svc.is_charging())

    def test_false_before_first_poll(self):
        """Fail-open: return False if BMS has never been polled."""
        svc = self._service()
        # _last_success_monotonic is 0.0 by default
        self.assertFalse(svc.is_charging())

    def test_false_when_data_stale(self):
        """Fail-open: return False if last poll was > bms_timeout_s ago."""
        svc = self._service(timeout_s=5.0)
        svc._state = BmsState(charge_fet_on=True, pack_current_a=5.0)
        # Pretend last success was 10 seconds ago (> timeout_s=5)
        svc._last_success_monotonic = time.monotonic() - 10.0
        self.assertFalse(svc.is_charging())

    def test_true_when_charge_fet_on_and_positive_current(self):
        svc = self._service()
        svc._state = BmsState(charge_fet_on=True, pack_current_a=3.5)
        svc._last_success_monotonic = time.monotonic()
        self.assertTrue(svc.is_charging())

    def test_false_when_charge_fet_on_but_zero_current(self):
        """FET on but no current → standby, not charging."""
        svc = self._service()
        svc._state = BmsState(charge_fet_on=True, pack_current_a=0.0)
        svc._last_success_monotonic = time.monotonic()
        self.assertFalse(svc.is_charging())

    def test_false_when_charge_fet_on_but_discharging(self):
        svc = self._service()
        svc._state = BmsState(charge_fet_on=True, pack_current_a=-2.0)
        svc._last_success_monotonic = time.monotonic()
        self.assertFalse(svc.is_charging())

    def test_false_when_charge_fet_off(self):
        svc = self._service()
        svc._state = BmsState(charge_fet_on=False, pack_current_a=5.0)
        svc._last_success_monotonic = time.monotonic()
        self.assertFalse(svc.is_charging())

    def test_clears_after_timeout(self):
        """Charging → BMS drops → should revert to False after timeout."""
        svc = self._service(timeout_s=2.0)
        svc._state = BmsState(charge_fet_on=True, pack_current_a=3.0)
        svc._last_success_monotonic = time.monotonic()

        # Immediately after last success: still charging
        self.assertTrue(svc.is_charging())

        # Simulate no poll for > timeout
        svc._last_success_monotonic = time.monotonic() - 3.0
        self.assertFalse(svc.is_charging())


# ---------------------------------------------------------------------------
# Unit tests: connection management
# ---------------------------------------------------------------------------

class TestConnectionManagement(unittest.TestCase):

    def test_start_stop(self):
        """BmsService starts and stops cleanly without hardware."""
        cfg = _BmsCfg(bms_mac_address="")  # Empty MAC → idle loop, no BLE attempt
        svc = BmsService(cfg)
        svc.start()
        time.sleep(0.05)
        svc.stop()
        self.assertIsNone(svc._thread)

    def test_idempotent_start(self):
        """Calling start() twice doesn't spawn a second thread."""
        cfg = _BmsCfg(bms_mac_address="")
        svc = BmsService(cfg)
        svc.start()
        t1 = svc._thread
        svc.start()
        t2 = svc._thread
        self.assertIs(t1, t2)
        svc.stop()

    def test_get_state_returns_copy(self):
        """get_state() must return a copy, not the live object."""
        cfg = _BmsCfg(bms_mac_address="")
        svc = BmsService(cfg)
        s1 = svc.get_state()
        s2 = svc.get_state()
        self.assertIsNot(s1, s2)

    def test_state_initially_none(self):
        cfg = _BmsCfg(bms_mac_address="")
        svc = BmsService(cfg)
        s = svc.get_state()
        self.assertIsNone(s.pack_voltage_v)
        self.assertIsNone(s.soc_pct)
        self.assertFalse(s.connected)


# ---------------------------------------------------------------------------
# Integration test: controller charger inhibit
# ---------------------------------------------------------------------------

class TestControllerChargerInhibit(unittest.TestCase):
    """Verify controller.set_charger_inhibit() blocks drive commands."""

    def setUp(self):
        import sys
        from pathlib import Path
        # Ensure wall_e-Mini/ is on sys.path so the real config.py is importable.
        repo_root = str(Path(__file__).resolve().parents[2])
        if repo_root not in sys.path:
            sys.path.insert(0, repo_root)

        from pi_app.control.controller import Controller, RCInputs
        from pi_app.control.mapping import CENTER_OUTPUT_VALUE

        self.Controller = Controller
        self.RCInputs = RCInputs
        self.NEUTRAL = CENTER_OUTPUT_VALUE

    def _make_rc(self, ch1=1700, ch2=1700):
        return self.RCInputs(
            ch1_us=ch1, ch2_us=ch2, ch3_us=1800,
            ch4_us=1500, ch5_us=1500,
            last_update_epoch_s=time.time(),
        )

    def test_charger_inhibit_blocks_motion(self):
        """When charger inhibit is active, motors must output neutral."""
        ctrl = self.Controller()
        # Arm the controller via safety channel
        rc_armed = self._make_rc()
        # Feed several cycles to arm
        for _ in range(3):
            rc_armed = self.RCInputs(
                ch1_us=1700, ch2_us=1700, ch3_us=1800,
                ch4_us=1500, ch5_us=1500,
                last_update_epoch_s=time.time(),
            )
            ctrl.process(rc_armed)

        ctrl.set_charger_inhibit(True)
        cmd, _, telem = ctrl.process(rc_armed)

        self.assertEqual(cmd.left_byte, self.NEUTRAL)
        self.assertEqual(cmd.right_byte, self.NEUTRAL)
        self.assertTrue(telem.get("charger_inhibit"))

    def test_charger_inhibit_cleared_allows_motion(self):
        """Clearing the charger inhibit flag restores normal operation."""
        ctrl = self.Controller()
        ctrl.set_charger_inhibit(True)
        ctrl.set_charger_inhibit(False)

        rc = self._make_rc()
        cmd, _, telem = ctrl.process(rc)

        self.assertNotIn("charger_inhibit", telem)

    def test_fail_open_no_bms(self):
        """With no BMS configured, charger_inhibit stays False (fail-open)."""
        ctrl = self.Controller()
        # Default state: charger inhibit not set
        rc = self._make_rc()
        _, _, telem = ctrl.process(rc)
        self.assertFalse(telem.get("charger_inhibit", False))


if __name__ == "__main__":
    unittest.main()
