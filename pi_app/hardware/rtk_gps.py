"""
Threaded RTK GPS reader for DFRobot GNSS-RTK rover module (KIT0198).

Communicates via I2C using the DFRobot register protocol, vendored from
DFRobot_RTK_LoRa.py (DFRobot/DFRobot_RTK_LoRa,
https://raw.githubusercontent.com/DFRobot/DFRobot_RTK_LoRa/master/python/
raspberrypi/DFRobot_RTK_LoRa.py, fetched 2026-09-19). Runs on a background
daemon thread and exposes a thread-safe ``get_reading()`` API.

Receiver: Quectel LC29HDA behind an I2C co-processor at 0x20.
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

# DFRobot I2C register map (vendored from DFRobot_RTK_LoRa.py; see module
# docstring for the source URL). Registers not used here (LoRa transport
# config, 4G, etc.) are omitted.
_REG_YEAR_H = 0
_REG_YEAR_L = 1
_REG_MONTH = 2
_REG_DATE = 3
_REG_HOUR = 4
_REG_MINUTE = 5
_REG_SECOND = 6
_REG_LAT_1 = 7
_REG_LON_1 = 13
_REG_GPS_STATE = 19
_REG_USE_STAR = 20
_REG_HDOP_Z = 21
_REG_ALT_H = 23
_REG_SEP_H = 26
_REG_DIF_Z = 29
_REG_DIF_X = 30
_REG_DIFID_H = 31
_REG_DIFID_L = 32
_REG_I2C_ID = 50
_REG_DATA_FLUSH = 80
_REG_RMC_LEN = 83
_REG_RMC_ALL = 84
_REG_VTG_LEN = 87
_REG_VTG_ALL = 88
_REG_OPERATION = 93
_MODULE_LORA = 10

_KNOTS_TO_MPS = 0.514444


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
    # Added 2026-09-19 (Commit D). Defaulted so existing keyword-argument
    # construction (tests, gps_heading_align.py) keeps working unchanged.
    utc_iso: str | None = None          # None when the receiver reports year=0 (no fix yet)
    geoid_sep_m: float | None = None
    cog_deg: float | None = None        # course over ground; None when stationary/unavailable
    sog_mps: float | None = None        # speed over ground
    nmea_mode: str | None = None        # RMC/VTG mode indicator character (A/D/N/R/F/...)


def _nmea_checksum_valid(sentence: str) -> bool:
    """Validate a $...*HH NMEA sentence's XOR checksum."""
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, tail = sentence[1:].partition("*")
    cksum_str = tail.strip()[:2]
    if len(cksum_str) != 2:
        return False
    try:
        expected = int(cksum_str, 16)
    except ValueError:
        return False
    actual = 0
    for ch in body:
        actual ^= ord(ch)
    return actual == expected


def _nmea_fields(sentence: str, suffix: str) -> list[str]:
    """Validate checksum + sentence type, return the comma-split fields
    (field 0 is the talker+sentence id, e.g. "GNRMC"). Raises ValueError."""
    sentence = sentence.strip("\x00").strip()
    if not sentence:
        raise ValueError("empty sentence")
    if not _nmea_checksum_valid(sentence):
        raise ValueError(f"bad NMEA checksum: {sentence!r}")
    body = sentence[1:sentence.index("*")]
    fields = body.split(",")
    if not fields[0].endswith(suffix):
        raise ValueError(f"not a {suffix} sentence: {fields[0]!r}")
    return fields


def parse_rmc_sentence(sentence: str) -> tuple[float | None, float | None, str | None]:
    """Parse a $--RMC sentence into (cog_deg, sog_mps, mode).

    cog_deg/sog_mps are None when their field is empty (typically means the
    receiver is stationary / has no course solution). Raises ValueError on a
    bad checksum or a sentence that isn't RMC.
    """
    fields = _nmea_fields(sentence, "RMC")
    # 0=id 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=sog_knots 8=cog_deg
    # 9=date 10=magvar 11=magvar_dir [12=mode indicator, NMEA 2.3+]
    sog_str = fields[7] if len(fields) > 7 else ""
    cog_str = fields[8] if len(fields) > 8 else ""
    mode = fields[12].strip() or None if len(fields) > 12 else None
    sog_mps = float(sog_str) * _KNOTS_TO_MPS if sog_str.strip() else None
    cog_deg = float(cog_str) if cog_str.strip() else None
    return cog_deg, sog_mps, mode


def parse_vtg_sentence(sentence: str) -> tuple[float | None, float | None]:
    """Parse a $--VTG sentence into (cog_deg, sog_mps). Raises ValueError."""
    fields = _nmea_fields(sentence, "VTG")
    # 0=id 1=cog_true 2='T' 3=cog_mag 4='M' 5=sog_knots 6='N' 7=sog_kmh 8='K'
    cog_str = fields[1] if len(fields) > 1 else ""
    sog_str = fields[5] if len(fields) > 5 else ""
    cog_deg = float(cog_str) if cog_str.strip() else None
    sog_mps = float(sog_str) * _KNOTS_TO_MPS if sog_str.strip() else None
    return cog_deg, sog_mps


class RtkGpsReader:
    """Background RTK GPS reader following the ArduinoRCReader thread pattern."""

    _REOPEN_BACKOFF_S = 1.0
    _CONSECUTIVE_ERROR_LIMIT = 10

    def __init__(self, config: GpsConfig) -> None:
        self._cfg = config
        self._reading: GpsReading | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Epoch dedup (poll-rate aliasing fix): only publish once per unique
        # UTC second, even though we poll faster than the GPS updates.
        self._last_utc_key: tuple[int, ...] | None = None
        self._epoch_count: int = 0

        # Health / observability.
        self._reconnect_count: int = 0
        self._poll_error_count: int = 0
        self._consecutive_i2c_errors: int = 0
        self._rmc_errors: int = 0
        self._last_poll_ok_t: float | None = None
        self._last_mode_value: int | None = None

    @staticmethod
    def detect(bus: int = 1, addr: int = 0x20, max_attempts: int = 3) -> bool:
        """Return True if the DFRobot RTK rover is present on I2C.

        Retries twice beyond the first attempt (``max_attempts=3`` total) --
        a cold I2C bus or a receiver still booting can fail the very first
        probe. Every failed attempt is logged with the exception text.
        """
        for attempt in range(1, max_attempts + 1):
            try:
                import smbus2
                b = smbus2.SMBus(bus)
                try:
                    data = b.read_i2c_block_data(addr, _REG_I2C_ID, 1)
                    return data[0] == addr
                finally:
                    b.close()
            except Exception as exc:
                logger.warning(
                    "RTK GPS detect() attempt %d/%d failed: %s", attempt, max_attempts, exc
                )
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

    def get_health(self) -> dict:
        """Reconnect/poll-error counters, last-poll age, RMC error count,
        and the last-read operation-mode register value (93)."""
        with self._lock:
            last_ok = self._last_poll_ok_t
            return {
                "reconnect_count": self._reconnect_count,
                "poll_error_count": self._poll_error_count,
                "consecutive_i2c_errors": self._consecutive_i2c_errors,
                "last_poll_age_s": (
                    round(time.monotonic() - last_ok, 2) if last_ok is not None else None
                ),
                "rmc_errors": self._rmc_errors,
                "mode_register_value": self._last_mode_value,
            }

    # -- Worker thread -------------------------------------------------------

    def _read_loop(self) -> None:
        try:
            import smbus2
        except ImportError:
            logger.error("smbus2 not installed — RTK GPS reader cannot start")
            return

        bus = self._cfg.i2c_bus
        addr = self._cfg.i2c_addr
        interval = 1.0 / max(self._cfg.poll_hz, 0.1)

        try:
            i2c = smbus2.SMBus(bus)
        except Exception:
            logger.exception("Failed to open I2C bus %d", bus)
            return

        try:
            dev_id = i2c.read_i2c_block_data(addr, _REG_I2C_ID, 1)
            if dev_id[0] != addr:
                logger.error("RTK device ID mismatch: expected 0x%02X, got 0x%02X",
                             addr, dev_id[0])
                return
            self._ensure_lora_mode(i2c, addr)
            logger.info("RTK GPS initialised on I2C bus %d addr 0x%02X", bus, addr)
        except Exception:
            logger.exception("RTK GPS init failed")
            i2c.close()
            return

        try:
            while not self._stop_event.is_set():
                try:
                    self._poll(i2c, addr)
                    with self._lock:
                        self._consecutive_i2c_errors = 0
                except Exception:
                    with self._lock:
                        self._consecutive_i2c_errors += 1
                        self._poll_error_count += 1
                        consecutive = self._consecutive_i2c_errors
                    logger.warning(
                        "RTK GPS poll error (%d consecutive)", consecutive, exc_info=True
                    )
                    if consecutive >= self._CONSECUTIVE_ERROR_LIMIT:
                        logger.warning(
                            "RTK GPS: %d consecutive I2C errors — closing and reopening bus",
                            consecutive,
                        )
                        try:
                            i2c.close()
                        except Exception:
                            pass
                        if self._stop_event.wait(self._REOPEN_BACKOFF_S):
                            break
                        try:
                            i2c = smbus2.SMBus(bus)
                            with self._lock:
                                self._reconnect_count += 1
                                self._consecutive_i2c_errors = 0
                            logger.warning("RTK GPS: I2C bus reopened")
                        except Exception:
                            logger.exception("RTK GPS: failed to reopen I2C bus")
                if self._stop_event.wait(interval):
                    break
        finally:
            try:
                i2c.close()
            except Exception:
                pass

    def _ensure_lora_mode(self, i2c, addr: int) -> None:
        """Read register 93 first; write MODULE_LORA (10) only on mismatch;
        read back and log the confirmed mode at WARNING (the app installs
        no logging handler, so INFO is dropped — this must reach journalctl)."""
        current = i2c.read_i2c_block_data(addr, _REG_OPERATION, 1)[0]
        with self._lock:
            self._last_mode_value = current
        if current == _MODULE_LORA:
            logger.warning("RTK GPS: already in LoRa mode (register 93 = %d)", current)
            return
        i2c.write_i2c_block_data(addr, _REG_OPERATION, [_MODULE_LORA])
        time.sleep(1.0)
        confirmed = i2c.read_i2c_block_data(addr, _REG_OPERATION, 1)[0]
        with self._lock:
            self._last_mode_value = confirmed
        if confirmed == _MODULE_LORA:
            logger.warning("RTK GPS: LoRa mode confirmed (register 93 = %d)", confirmed)
        else:
            logger.warning(
                "RTK GPS: LoRa mode write did not take — register 93 reads %d after write",
                confirmed,
            )

    def _poll(self, i2c, addr: int) -> None:
        # Register 32 (DIFID_L) is read FIRST, then registers 0-31 as one
        # block, so the two halves of station_id (DIFID_H at 31, DIFID_L at
        # 32) cannot tear across two I2C transactions landing on either
        # side of a device-side register update.
        difid_l = i2c.read_i2c_block_data(addr, _REG_DIFID_L, 1)[0]
        raw = i2c.read_i2c_block_data(addr, 0, 32)

        utc_key = (
            raw[_REG_YEAR_H], raw[_REG_YEAR_L], raw[_REG_MONTH], raw[_REG_DATE],
            raw[_REG_HOUR], raw[_REG_MINUTE], raw[_REG_SECOND],
        )
        if utc_key == self._last_utc_key:
            # Same epoch as the last publish (5 Hz poll vs. ~1 Hz GPS
            # update) -- publishing again would just alias the same fix
            # under a newer timestamp. The data-flush register (80) is
            # deliberately NOT used for this gate: it did not reliably
            # correspond to a genuinely new epoch.
            return
        self._last_utc_key = utc_key
        self._epoch_count += 1

        lat = self._parse_lat(raw)
        lon = self._parse_lon(raw)
        quality = raw[_REG_GPS_STATE]
        sats = raw[_REG_USE_STAR]
        hdop = raw[_REG_HDOP_Z] + raw[_REG_HDOP_Z + 1] / 100.0
        alt = self._parse_alt(raw)
        sep = self._parse_sep(raw)
        diff_age = raw[_REG_DIF_Z] + raw[_REG_DIF_X] / 100.0
        station_id = raw[_REG_DIFID_H] * 256 + difid_l
        utc_iso = self._format_utc(raw)

        cog_deg = sog_mps = nmea_mode = None
        cog_every = max(1, int(self._cfg.cog_read_every))
        if self._cfg.read_nmea_sentences and (self._epoch_count % cog_every == 0):
            cog_deg, sog_mps, nmea_mode = self._read_course_speed(i2c, addr)

        with self._lock:
            prev_quality = self._reading.fix_quality if self._reading is not None else None
        if prev_quality is not None and prev_quality != quality:
            logger.warning(
                "RTK GPS fix_quality %d -> %d (utc=%s sats=%d hdop=%.2f "
                "diff_age_s=%.1f station_id=%d)",
                prev_quality, quality, utc_iso, sats, hdop, diff_age, station_id,
            )

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
            utc_iso=utc_iso,
            geoid_sep_m=sep,
            cog_deg=cog_deg,
            sog_mps=sog_mps,
            nmea_mode=nmea_mode,
        )
        with self._lock:
            self._reading = reading
            self._last_poll_ok_t = time.monotonic()

    def _read_course_speed(self, i2c, addr: int) -> tuple[float | None, float | None, str | None]:
        """Read course/speed over ground: RMC primary, VTG as a course/speed
        fallback when RMC's course field is empty. Never raises -- any
        failure increments self._rmc_errors (RMC only) and is logged, and
        the main position read is unaffected either way."""
        cog_deg = sog_mps = mode = None
        try:
            rmc = self._read_gnss_sentence(i2c, addr, _REG_RMC_LEN, _REG_RMC_ALL)
            if rmc:
                cog_deg, sog_mps, mode = parse_rmc_sentence(rmc)
        except Exception:
            with self._lock:
                self._rmc_errors += 1
            logger.debug("RTK GPS RMC read/parse failed", exc_info=True)
            return None, None, None

        if cog_deg is None:
            try:
                vtg = self._read_gnss_sentence(i2c, addr, _REG_VTG_LEN, _REG_VTG_ALL)
                if vtg:
                    vtg_cog, vtg_sog = parse_vtg_sentence(vtg)
                    if vtg_cog is not None:
                        cog_deg = vtg_cog
                    if sog_mps is None and vtg_sog is not None:
                        sog_mps = vtg_sog
            except Exception:
                logger.debug("RTK GPS VTG fallback read/parse failed", exc_info=True)

        return cog_deg, sog_mps, mode

    @staticmethod
    def _read_gnss_sentence(i2c, addr: int, reg_len: int, reg_all: int) -> str:
        """Mirrors DFRobot_RTK_LoRa.get_gnss_message()'s I2C-mode loop: read
        the 1-byte length from reg_len, then for each 32-byte chunk write
        the running byte offset back to reg_len and read the chunk from
        reg_all."""
        length = i2c.read_i2c_block_data(addr, reg_len, 1)[0]
        if length == 0:
            return ""
        len1, len2 = divmod(length, 32)
        chunks: list[int] = []
        writelen = 0
        for num in range(len1 + 1):
            read_len = len2 if num == len1 else 32
            if read_len == 0:
                break
            i2c.write_i2c_block_data(addr, reg_len, [writelen & 0xFF])
            chunk = i2c.read_i2c_block_data(addr, reg_all, read_len)
            chunks.extend(chunk)
            writelen += read_len
            time.sleep(0.001)
        time.sleep(0.01)
        return "".join(chr(b) for b in chunks)

    @staticmethod
    def _format_utc(raw: list[int]) -> str | None:
        year = raw[_REG_YEAR_H] * 256 + raw[_REG_YEAR_L]
        if year == 0:
            return None
        month, date = raw[_REG_MONTH], raw[_REG_DATE]
        hour, minute, second = raw[_REG_HOUR], raw[_REG_MINUTE], raw[_REG_SECOND]
        return f"{year:04d}-{month:02d}-{date:02d}T{hour:02d}:{minute:02d}:{second:02d}Z"

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

    @staticmethod
    def _parse_sep(raw: list[int]) -> float:
        """Geoid separation (registers 26-28) -- same sign/scale shape as
        _parse_alt (registers 23-25)."""
        h = raw[_REG_SEP_H]
        sign = 1.0
        if h & 0x80:
            h &= 0x7F
            sign = -1.0
        return (h * 256 + raw[_REG_SEP_H + 1] + raw[_REG_SEP_H + 2] / 100.0) * sign
