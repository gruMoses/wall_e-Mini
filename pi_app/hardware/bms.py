"""
Daly BMS Bluetooth service for WALL-E Mini.

Polls the Daly SPIM08HP BMS over BLE (bleak) in a background thread.
Thread-safe access to the latest battery state via get_state() / is_charging().

Fail-open charger inhibit: if the BMS has been unreachable for longer than
bms_timeout_s, is_charging() returns False so a lost BLE link never bricks
the robot.

References:
  - bms_scan.py at repo root for the Daly wire protocol
  - config.BmsConfig for tunable parameters
"""

import asyncio
import copy
import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

# Daly BLE service UUID fragment (0xFFF0 family)
_DALY_SERVICE_UUID_FRAGMENT = "fff0"

# Daly command bytes (A5 40 <cmd> 08 00*8 <checksum>)
_DALY_CMD = {
    "soc":               bytes([0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D]),
    "cell_voltage_range":bytes([0xA5, 0x40, 0x91, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E]),
    "temp_range":        bytes([0xA5, 0x40, 0x92, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F]),
    "mosfet_status":     bytes([0xA5, 0x40, 0x93, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80]),
    "status":            bytes([0xA5, 0x40, 0x94, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81]),
    "errors":            bytes([0xA5, 0x40, 0x97, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84]),
}

_DALY_ERROR_NAMES: List[str] = [
    "Cell OVP", "Cell UVP", "Pack OVP", "Pack UVP",
    "Charge OTP", "Charge UTP", "Discharge OTP", "Discharge UTP",
    "Charge OCP", "Discharge OCP", "Short Circuit", "IC Error",
    "FET Lock",
]


@dataclass
class BmsState:
    """Latest snapshot of BMS telemetry.

    All numeric fields are None until the first successful poll.
    ``connected`` reflects the current BLE link status.
    """
    # Pack-level
    pack_voltage_v: Optional[float] = None
    pack_current_a: Optional[float] = None   # positive = charging, negative = discharging
    soc_pct: Optional[float] = None

    # Cell voltages (mV)
    cell_max_mv: Optional[int] = None
    cell_min_mv: Optional[int] = None
    cell_delta_mv: Optional[int] = None

    # Temperature (°C)
    temp_max_c: Optional[int] = None
    temp_min_c: Optional[int] = None

    # MOSFET / charger state
    charge_fet_on: Optional[bool] = None
    discharge_fet_on: Optional[bool] = None

    # Pack info
    num_cells: Optional[int] = None
    cycle_count: Optional[int] = None

    # Active protection flags (list of human-readable names)
    error_flags: List[str] = field(default_factory=list)

    # Metadata
    last_update_epoch_s: float = 0.0
    connected: bool = False


class BmsService:
    """Threaded Daly BMS reader.

    Runs an asyncio event loop in a daemon thread and polls the BMS over BLE
    at the configured interval.  The main thread accesses state exclusively
    through ``get_state()`` and ``is_charging()``.

    Usage::

        svc = BmsService(config.bms)
        svc.start()

        state = svc.get_state()
        if svc.is_charging():
            ...

        svc.stop()
    """

    _BACKOFF_INITIAL_S: float = 5.0
    _BACKOFF_MAX_S: float = 60.0

    def __init__(self, bms_config) -> None:
        """
        Args:
            bms_config: a ``BmsConfig`` instance (or any object with the same
                attributes: ``bms_mac_address``, ``bms_poll_interval_s``,
                ``bms_timeout_s``, ``charger_inhibit_enabled``).
        """
        self._cfg = bms_config
        self._state = BmsState()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # Monotonic timestamp of the last successful poll (0.0 = never polled)
        self._last_success_monotonic: float = 0.0

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Start the background polling thread (idempotent)."""
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop, name="BmsService", daemon=True
        )
        self._thread.start()
        logger.info("BMS service started (MAC: %s)", self._cfg.bms_mac_address or "unset")

    def stop(self) -> None:
        """Stop the background thread and close the BLE connection."""
        self._stop_event.set()
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        logger.info("BMS service stopped")

    def get_state(self) -> BmsState:
        """Return a snapshot of the latest BMS telemetry (thread-safe)."""
        with self._lock:
            return copy.copy(self._state)

    def is_charging(self) -> bool:
        """Return True when the BMS positively reports active charging.

        Charging is defined as: charge FET enabled AND pack current > 0 A.

        Fail-open behaviour: if the BMS has been unreachable for longer than
        ``bms_timeout_s``, or has never been polled, this returns False so a
        stale / dropped BLE link never prevents the robot from driving.
        """
        if not self._cfg.charger_inhibit_enabled:
            return False

        # Fail-open: no data yet or data is too old
        elapsed = time.monotonic() - self._last_success_monotonic
        if self._last_success_monotonic == 0.0 or elapsed > self._cfg.bms_timeout_s:
            return False

        with self._lock:
            state = self._state

        return bool(
            state.charge_fet_on
            and state.pack_current_a is not None
            and state.pack_current_a > 0.0
        )

    # ------------------------------------------------------------------ #
    # Background asyncio loop
    # ------------------------------------------------------------------ #

    def _run_loop(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        try:
            loop.run_until_complete(self._poll_forever())
        except Exception:
            logger.exception("BMS event loop exited unexpectedly")
        finally:
            loop.close()
            self._loop = None

    async def _poll_forever(self) -> None:
        mac = self._cfg.bms_mac_address
        if not mac:
            logger.warning("BMS MAC address not configured — BMS service idle")
            return

        backoff = self._BACKOFF_INITIAL_S
        while not self._stop_event.is_set():
            try:
                await self._connect_and_poll(mac)
                backoff = self._BACKOFF_INITIAL_S  # reset on clean disconnect
            except Exception as exc:
                with self._lock:
                    self._state.connected = False
                logger.warning(
                    "BMS connection to %s failed: %s — retrying in %.0fs", mac, exc, backoff
                )
                await self._interruptible_sleep(backoff)
                backoff = min(backoff * 2.0, self._BACKOFF_MAX_S)

    async def _connect_and_poll(self, mac: str) -> None:
        """Connect to the BMS, subscribe to notifications, and poll until stop."""
        try:
            from bleak import BleakClient  # type: ignore
        except ImportError:
            logger.error(
                "bleak is not installed — install with: pip install bleak"
            )
            await self._interruptible_sleep(30.0)
            return

        logger.info("Connecting to Daly BMS at %s...", mac)
        async with BleakClient(mac, timeout=self._cfg.bms_timeout_s) as client:
            tx_char, rx_char = self._find_daly_chars(client)
            if tx_char is None or rx_char is None:
                logger.warning(
                    "Daly TX/RX characteristics not found on %s — "
                    "check BMS MAC and that it is advertising the 0xFFF0 service",
                    mac,
                )
                return

            logger.info(
                "Daly BMS connected — polling every %.0fs", self._cfg.bms_poll_interval_s
            )
            with self._lock:
                self._state.connected = True

            response_buf: bytearray = bytearray()
            response_event: asyncio.Event = asyncio.Event()

            def _notification_handler(sender, data):  # noqa: ANN001
                response_buf.extend(data)
                response_event.set()

            await client.start_notify(rx_char, _notification_handler)
            try:
                while not self._stop_event.is_set():
                    await self._do_poll(client, tx_char, response_buf, response_event)
                    await self._interruptible_sleep(self._cfg.bms_poll_interval_s)
            finally:
                try:
                    await client.stop_notify(rx_char)
                except Exception:
                    pass

    async def _do_poll(
        self,
        client,
        tx_char: str,
        buf: bytearray,
        event: asyncio.Event,
    ) -> None:
        """Issue all Daly queries and atomically update shared state."""

        async def send_recv(cmd_bytes: bytes, timeout: float = 3.0) -> bytes:
            buf.clear()
            event.clear()
            await client.write_gatt_char(tx_char, cmd_bytes, response=False)
            try:
                await asyncio.wait_for(event.wait(), timeout=timeout)
                # Brief settle to accumulate any trailing BLE packets
                await asyncio.sleep(0.2)
            except asyncio.TimeoutError:
                pass
            return bytes(buf)

        new = BmsState()
        new.connected = True

        # --- SOC: pack voltage, current, state of charge ---
        resp = await send_recv(_DALY_CMD["soc"])
        if len(resp) >= 13 and resp[2] == 0x90:
            p = resp[4:12]
            new.pack_voltage_v = struct.unpack(">H", p[0:2])[0] * 0.1
            current_raw = struct.unpack(">H", p[4:6])[0]
            new.pack_current_a = (current_raw - 30000) * 0.1
            new.soc_pct = struct.unpack(">H", p[6:8])[0] * 0.1

        # --- Cell voltage range ---
        resp = await send_recv(_DALY_CMD["cell_voltage_range"])
        if len(resp) >= 13 and resp[2] == 0x91:
            p = resp[4:12]
            max_mv = struct.unpack(">H", p[0:2])[0]
            min_mv = struct.unpack(">H", p[3:5])[0]
            new.cell_max_mv = max_mv
            new.cell_min_mv = min_mv
            new.cell_delta_mv = max_mv - min_mv

        # --- Temperature range ---
        resp = await send_recv(_DALY_CMD["temp_range"])
        if len(resp) >= 13 and resp[2] == 0x92:
            p = resp[4:12]
            new.temp_max_c = p[0] - 40
            new.temp_min_c = p[2] - 40

        # --- MOSFET / charger status ---
        resp = await send_recv(_DALY_CMD["mosfet_status"])
        if len(resp) >= 13 and resp[2] == 0x93:
            p = resp[4:12]
            status_byte = p[0]
            new.charge_fet_on = bool(status_byte & 0x01)
            new.discharge_fet_on = bool(status_byte & 0x02)

        # --- Pack status: cell count, cycle count ---
        resp = await send_recv(_DALY_CMD["status"])
        if len(resp) >= 13 and resp[2] == 0x94:
            p = resp[4:12]
            new.num_cells = p[0]
            new.cycle_count = struct.unpack(">H", p[5:7])[0]

        # --- Active protection / error flags ---
        resp = await send_recv(_DALY_CMD["errors"])
        if len(resp) >= 13 and resp[2] == 0x97:
            p = resp[4:12]
            flags_word = struct.unpack(">H", p[0:2])[0]
            new.error_flags = [
                name
                for i, name in enumerate(_DALY_ERROR_NAMES)
                if flags_word & (1 << i)
            ]

        new.last_update_epoch_s = time.time()

        logger.info(
            "BMS poll: %.1fV %.1fA SOC=%.1f%% "
            "cells=[%s,%s]mV Δ%smV temp=[%s,%s]°C "
            "chgFET=%s disFET=%s cycles=%s errors=%s",
            new.pack_voltage_v or 0.0,
            new.pack_current_a or 0.0,
            new.soc_pct or 0.0,
            new.cell_min_mv, new.cell_max_mv, new.cell_delta_mv,
            new.temp_min_c, new.temp_max_c,
            new.charge_fet_on, new.discharge_fet_on,
            new.cycle_count,
            ", ".join(new.error_flags) if new.error_flags else "none",
        )

        if new.error_flags:
            logger.error("BMS protection flags active: %s", ", ".join(new.error_flags))

        with self._lock:
            self._state = new
        self._last_success_monotonic = time.monotonic()

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _find_daly_chars(client) -> Tuple[Optional[str], Optional[str]]:
        """Return (tx_uuid, rx_uuid) for the Daly 0xFFF0 BLE service."""
        tx_char: Optional[str] = None
        rx_char: Optional[str] = None
        for service in client.services:
            if _DALY_SERVICE_UUID_FRAGMENT not in service.uuid.lower():
                continue
            for char in service.characteristics:
                props = char.properties
                if tx_char is None and (
                    "write" in props or "write-without-response" in props
                ):
                    tx_char = char.uuid
                if rx_char is None and "notify" in props:
                    rx_char = char.uuid
            if tx_char and rx_char:
                break
        return tx_char, rx_char

    async def _interruptible_sleep(self, seconds: float) -> None:
        """Sleep that wakes early when stop_event is set."""
        deadline = time.monotonic() + seconds
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            await asyncio.sleep(min(0.1, remaining))
