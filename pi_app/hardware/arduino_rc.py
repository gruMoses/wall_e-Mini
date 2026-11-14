import threading
import time
import logging
import re
from dataclasses import dataclass
from typing import Optional, Tuple, List

import glob

from pi_app.control.rc_signal import (
    RC_NO_SIGNAL_US,
    RC_VALID_MAX_US,
    RC_VALID_MIN_US,
)

logger = logging.getLogger(__name__)

try:
    from prometheus_client import Counter

    RC_READ_ERRORS = Counter(
        "rc_read_errors_total", "Number of RC read failures"
    )
except Exception:  # pragma: no cover
    RC_READ_ERRORS = None

RC_READ_ERROR_COUNT = 0

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc


# Line formats (docs/rc_arduino_gps_wiring.md, section 7):
#   5 fields  ch1,ch2,ch3,ch4,ch5      today's firmware (dcc550e)
#   6 fields  ch1,ch2,ch3,ch4,ch5,ch6  the firmware with the no-pulse rule
# Each field is a width in us (RC_VALID_MIN_US..RC_VALID_MAX_US) or the
# RC_NO_SIGNAL_US sentinel (0). The old 4-field format (no ch4) is no longer
# accepted: no firmware since 5f275d0 sends it, and a 4-field line is now
# most likely the tail of a 5-field line, with every channel shifted.
CSV_FIELDS_5 = 5
CSV_FIELDS_6 = 6
BAUD_RATE = 115200
DETECTION_READ_TIMEOUT_S = 0.5
DETECTION_WINDOW_S = 5.0
STEADY_READ_TIMEOUT_S = 1.0

# Digits only: no sign, no spaces, no "1_500" (int() accepts that).
_LINE_RE = re.compile(r"^\d{1,4}(?:,\d{1,4}){4,5}$")

# Sanitization
MIN_ACCEPTABLE_US = RC_VALID_MIN_US
MAX_ACCEPTABLE_US = RC_VALID_MAX_US
CENTER_PULSE_WIDTH_US = 1500
DEADBAND_US = 25

ParsedLine = Tuple[int, int, int, int, int, Optional[int]]


@dataclass
class RCState:
    # A width in us, or RC_NO_SIGNAL_US (0): the Arduino saw no valid pulse
    # on that channel (pi_app/control/rc_signal.py says how to read it).
    ch1_us: int = CENTER_PULSE_WIDTH_US
    ch2_us: int = CENTER_PULSE_WIDTH_US
    ch3_us: int = CENTER_PULSE_WIDTH_US
    ch4_us: int = CENTER_PULSE_WIDTH_US
    ch5_us: int = CENTER_PULSE_WIDTH_US
    # time.monotonic() when this frame was read; 0.0 = no frame yet. The
    # staleness checks compare it with time.monotonic(). Never the wall
    # clock: the Pi's clock steps at boot (RTC reset, then NTP), and a
    # backward step made a dead link look fresh.
    last_update_mono_s: float = 0.0
    # CH6 (A0, not used yet); None when the firmware sends 5 fields.
    ch6_us: Optional[int] = None

    def as_tuple(self) -> Tuple[int, int, int, int, int]:
        return (self.ch1_us, self.ch2_us, self.ch3_us, self.ch4_us, self.ch5_us)


def _list_candidate_ports(preferred: Optional[str] = None) -> List[str]:
    candidates: List[str] = []
    if preferred:
        candidates.append(preferred)
    # Prefer ttyUSB first, then ttyACM
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))

    # Deduplicate but preserve order
    seen = set()
    unique: List[str] = []
    for path in candidates:
        if path not in seen:
            unique.append(path)
            seen.add(path)
    return unique


def _try_detect_on_port(device: str) -> bool:
    """Open a device and confirm it emits parseable RC CSV lines."""
    try:
        with serial.Serial(device, BAUD_RATE, timeout=DETECTION_READ_TIMEOUT_S) as ser:
            ser.reset_input_buffer()
            start = time.monotonic()
            while time.monotonic() - start < DETECTION_WINDOW_S:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    text = raw.decode("utf-8", errors="strict").strip()
                except UnicodeDecodeError:
                    # Skip undecodable lines during detection
                    continue
                values = _parse_csv_line(text)
                if values is not None:
                    return True
    except Exception:
        return False
    return False


def discover_arduino_port(preferred: Optional[str] = "/dev/ttyUSB0") -> Optional[str]:
    """
    Scan candidate serial devices and return the first that looks like our Arduino RC stream.
    """
    for device in _list_candidate_ports(preferred):
        if _try_detect_on_port(device):
            return device
    return None


def _parse_csv_line(line: str) -> Optional[ParsedLine]:
    """Parse one stripped line into (ch1, ch2, ch3, ch4, ch5, ch6 or None).

    5 or 6 fields of plain digits. Each field must be a valid width or the
    RC_NO_SIGNAL_US sentinel; any other value means a corrupt line (neither
    firmware sends one: dcc550e clamps to 800..2200), so the whole line is
    dropped, not one field, because a corrupt line can have its fields
    shifted. A dropped line does not refresh the frame stamp, so a stream of
    them goes RC-stale after 1 s.
    """
    if not _LINE_RE.match(line):
        return None
    values: List[Optional[int]] = [int(p) for p in line.split(",")]
    for v in values:
        if v != RC_NO_SIGNAL_US and not (RC_VALID_MIN_US <= v <= RC_VALID_MAX_US):  # type: ignore[operator]
            return None
    if len(values) == CSV_FIELDS_5:
        values.append(None)
    return tuple(values)  # type: ignore[return-value]


class _LineFramer:
    """Drop lines that can be fragments of a line, then parse.

    A fragment parses as a shorter line with every channel shifted (the tail
    of a 6-field line is a valid-looking 5-field line). So:
    - a read without a newline (readline timed out mid-line) is dropped, and
      so is the next line, which starts mid-line;
    - the first line after a port is opened is dropped (the port can be
      opened mid-line; normally that line is the "RC Reader initialized"
      banner anyway);
    - once a 6-field line is seen on this port, 5-field lines are dropped.
    """

    def __init__(self) -> None:
        self._skip_next = False
        self._six_fields = False

    def new_port(self) -> None:
        self._skip_next = True
        self._six_fields = False

    def feed(self, raw: bytes) -> Optional[ParsedLine]:
        if not raw.endswith(b"\n"):
            self._skip_next = True
            return None
        if self._skip_next:
            self._skip_next = False
            return None
        try:
            line = raw.decode("ascii", errors="strict").strip()
        except UnicodeDecodeError:
            return None
        values = _parse_csv_line(line)
        if values is None:
            return None
        if values[5] is None:
            if self._six_fields:
                return None
        else:
            self._six_fields = True
        return values


def _apply_deadband(value_us: int) -> int:
    if value_us == RC_NO_SIGNAL_US:
        return value_us
    if abs(value_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_PULSE_WIDTH_US
    return value_us


class ArduinoRCReader:
    """
    Background reader that exposes the latest sanitized RC values from the Arduino.

    - Autodetects the serial port unless one is provided
    - Parses lines with 5 CSV fields (ch1..ch5, today's firmware) or 6
      (ch1..ch6, the firmware with the no-pulse rule); see _LineFramer for
      the lines it drops
    - A field is a width (500..2500 µs) or RC_NO_SIGNAL_US (0), kept as is;
      a line with any other value is dropped whole (the old reader put the
      last valid value in that field, which held a dead channel)
    - Applies ±25 µs deadband around 1500 µs to measured widths
    - Uses a 1.0 s read timeout during steady-state
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud_rate: int = BAUD_RATE,
        steady_timeout_s: float = STEADY_READ_TIMEOUT_S,
    ) -> None:
        self._requested_port = port
        self._baud_rate = baud_rate
        self._steady_timeout_s = steady_timeout_s

        self._port_in_use: Optional[str] = None
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()

        self._framer = _LineFramer()
        self._state = RCState()

    @property
    def port(self) -> Optional[str]:
        return self._port_in_use

    def start(self) -> str:
        if self._serial is not None:
            return self._port_in_use or ""

        port = self._requested_port or discover_arduino_port()
        if port is None:
            raise RuntimeError(
                "Arduino RC not detected on any serial port (/dev/ttyUSB* or /dev/ttyACM*)"
            )

        ser = serial.Serial(port, self._baud_rate, timeout=self._steady_timeout_s)
        ser.reset_input_buffer()
        self._framer.new_port()
        self._serial = ser
        self._port_in_use = port

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, name="ArduinoRCReader", daemon=True)
        self._thread.start()
        return port

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def get_state(self) -> RCState:
        return self._state

    def _reconnect(self) -> bool:
        """Close current serial, rediscover the Arduino, and reopen.

        Returns True on successful reconnection, False otherwise.
        """
        try:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        except Exception:
            pass

        port = discover_arduino_port(preferred=self._requested_port or self._port_in_use)
        if port is None:
            return False

        try:
            ser = serial.Serial(port, self._baud_rate, timeout=self._steady_timeout_s)
            ser.reset_input_buffer()
            self._framer.new_port()
            self._serial = ser
            self._port_in_use = port
            logger.info("Reconnected to Arduino RC on %s", port)
            return True
        except Exception:
            return False

    def _read_loop(self) -> None:
        global RC_READ_ERROR_COUNT
        assert self._serial is not None
        consecutive_errors = 0
        while not self._stop_event.is_set():
            ser = self._serial
            if ser is None:
                if not self._reconnect():
                    for _ in range(20):
                        if self._stop_event.is_set():
                            return
                        time.sleep(0.1)
                    continue
                consecutive_errors = 0
                continue

            try:
                raw = ser.readline()
                if not raw:
                    continue
                values = self._framer.feed(raw)
                if values is None:
                    continue

                now = time.monotonic()
                ch6 = values[5]
                self._state = RCState(
                    ch1_us=_apply_deadband(values[0]),
                    ch2_us=_apply_deadband(values[1]),
                    ch3_us=_apply_deadband(values[2]),
                    ch4_us=_apply_deadband(values[3]),
                    ch5_us=_apply_deadband(values[4]),
                    last_update_mono_s=now,
                    ch6_us=_apply_deadband(ch6) if ch6 is not None else None,
                )
                consecutive_errors = 0
            except (serial.SerialException, OSError):
                consecutive_errors += 1
                RC_READ_ERROR_COUNT += 1
                if RC_READ_ERRORS is not None:
                    RC_READ_ERRORS.inc()
                logger.warning("Serial connection lost (error #%d), attempting reconnect...",
                               consecutive_errors)
                self._serial = None
            except Exception:
                consecutive_errors += 1
                RC_READ_ERROR_COUNT += 1
                if RC_READ_ERRORS is not None:
                    RC_READ_ERRORS.inc()
                logger.exception("RC read failed")
                if consecutive_errors >= 10:
                    logger.warning("Too many consecutive errors, attempting reconnect...")
                    self._serial = None
                    consecutive_errors = 0
                time.sleep(0.01)

    def send_motor_command(self, left_byte: int, right_byte: int) -> None:
        ser = self._serial
        if ser is None:
            return
        left = max(0, min(254, int(left_byte)))
        right = max(0, min(254, int(right_byte)))
        line = f"M,{left},{right}\n".encode("utf-8")
        with self._write_lock:
            try:
                ser.write(line)
                ser.flush()
            except Exception:
                pass


