"""
Threaded RTK GPS reader for DFRobot GNSS-RTK rover module (KIT0198).

Communicates via I2C using the DFRobot register protocol (vendored from
DFRobot_RTK_LoRa.py).  Runs on a background daemon thread at 1 Hz and
exposes a thread-safe ``get_reading()`` API.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import GpsConfig

logger = logging.getLogger(__name__)

# DFRobot I2C register map (from DFRobot_RTK_LoRa.py)
_REG_LAT_1 = 7
_REG_LON_1 = 13
_REG_GPS_STATE = 19
_REG_USE_STAR = 20
_REG_HDOP_Z = 21
_REG_ALT_H = 23
_REG_DIF_Z = 29
_REG_DIF_X = 30
_REG_DIFID_H = 31
_REG_DIFID_L = 32
_REG_I2C_ID = 50
_REG_DATA_FLUSH = 80
_REG_OPERATION = 93
_MODULE_LORA = 10


@dataclass(frozen=True)
class GpsReading:
    latitude: float
    longitude: float
    altitude_m: float
    fix_quality: int
    satellites_used: int
    hdop: float
    diff_age_s: float   # seconds since last RTK correction via LoRa
    station_id: int     # base station ID (0 = none)
    timestamp: float    # time.monotonic()


class RtkGpsReader:
    """Background RTK GPS reader following the ArduinoRCReader thread pattern."""

    def __init__(self, config: GpsConfig) -> None:
        self._cfg = config
        self._reading: GpsReading | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @staticmethod
    def detect(bus: int = 1, addr: int = 0x20) -> bool:
        """Return True if the DFRobot RTK rover is present on I2C."""
        try:
            import smbus2
            b = smbus2.SMBus(bus)
            try:
                data = b.read_i2c_block_data(addr, _REG_I2C_ID, 1)
                return data[0] == addr
            finally:
                b.close()
        except Exception:
            return False

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name="RtkGpsReader", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def get_reading(self) -> GpsReading | None:
        with self._lock:
            return self._reading

    def _read_loop(self) -> None:
        try:
            import smbus2
        except ImportError:
            logger.error("smbus2 not installed — RTK GPS reader cannot start")
            return

        bus = self._cfg.i2c_bus
        addr = self._cfg.i2c_addr
        interval = 1.0 / max(self._cfg.update_rate_hz, 0.1)

        try:
            i2c = smbus2.SMBus(bus)
        except Exception:
            logger.exception("Failed to open I2C bus %d", bus)
            return

        try:
            # Confirm device presence
            dev_id = i2c.read_i2c_block_data(addr, _REG_I2C_ID, 1)
            if dev_id[0] != addr:
                logger.error("RTK device ID mismatch: expected 0x%02X, got 0x%02X",
                             addr, dev_id[0])
                return

            # Select LoRa mode
            i2c.write_i2c_block_data(addr, _REG_OPERATION, [_MODULE_LORA])
            time.sleep(1.0)
            logger.info("RTK GPS initialised (LoRa mode) on I2C bus %d addr 0x%02X",
                        bus, addr)
        except Exception:
            logger.exception("RTK GPS init failed")
            i2c.close()
            return

        try:
            while not self._stop_event.is_set():
                try:
                    self._poll(i2c, addr)
                except Exception:
                    logger.warning("RTK GPS poll error", exc_info=True)
                time.sleep(interval)
        finally:
            i2c.close()

    def _poll(self, i2c, addr: int) -> None:
        # Check for fresh data
        flush = i2c.read_i2c_block_data(addr, _REG_DATA_FLUSH, 1)
        if flush[0] == 0:
            return

        # smbus2 limits block reads to 32 bytes; split across the boundary.
        raw = i2c.read_i2c_block_data(addr, 0, 32)
        raw.append(i2c.read_i2c_block_data(addr, 32, 1)[0])

        lat = self._parse_lat(raw)
        lon = self._parse_lon(raw)
        quality = raw[_REG_GPS_STATE]
        sats = raw[_REG_USE_STAR]
        hdop = raw[_REG_HDOP_Z] + raw[_REG_HDOP_Z + 1] / 100.0
        alt = self._parse_alt(raw)
        diff_age = raw[_REG_DIF_Z] + raw[_REG_DIF_X] / 100.0
        station_id = raw[_REG_DIFID_H] * 256 + raw[_REG_DIFID_L]

        if quality < self._cfg.min_quality:
            return

        reading = GpsReading(
            latitude=lat,
            longitude=lon,
            altitude_m=alt,
            fix_quality=quality,
            satellites_used=sats,
            hdop=hdop,
            diff_age_s=diff_age,
            station_id=station_id,
            timestamp=time.monotonic(),
        )
        with self._lock:
            self._reading = reading

    @staticmethod
    def _parse_lat(raw: list[int]) -> float:
        dd = raw[_REG_LAT_1]
        mm = raw[_REG_LAT_1 + 1]
        mmmmm = raw[_REG_LAT_1 + 2] * 65536 + raw[_REG_LAT_1 + 3] * 256 + raw[_REG_LAT_1 + 4]
        direction = chr(raw[_REG_LAT_1 + 5])
        deg = dd + mm / 60.0 + mmmmm / 100000.0 / 60.0
        return -deg if direction == "S" else deg

    @staticmethod
    def _parse_lon(raw: list[int]) -> float:
        ddd = raw[_REG_LON_1]
        mm = raw[_REG_LON_1 + 1]
        mmmmm = raw[_REG_LON_1 + 2] * 65536 + raw[_REG_LON_1 + 3] * 256 + raw[_REG_LON_1 + 4]
        direction = chr(raw[_REG_LON_1 + 5])
        deg = ddd + mm / 60.0 + mmmmm / 100000.0 / 60.0
        return -deg if direction == "W" else deg

    @staticmethod
    def _parse_alt(raw: list[int]) -> float:
        h = raw[_REG_ALT_H]
        sign = 1.0
        if h & 0x80:
            h &= 0x7F
            sign = -1.0
        return (h * 256 + raw[_REG_ALT_H + 1] + raw[_REG_ALT_H + 2] / 100.0) * sign
