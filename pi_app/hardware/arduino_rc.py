import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple, List

import glob

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


CSV_FIELDS_OLD = 4
CSV_FIELDS_NEW = 5
BAUD_RATE = 115200
DETECTION_READ_TIMEOUT_S = 0.5
DETECTION_WINDOW_S = 5.0
STEADY_READ_TIMEOUT_S = 1.0

# Sanitization
MIN_ACCEPTABLE_US = 800
MAX_ACCEPTABLE_US = 2200
CENTER_PULSE_WIDTH_US = 1500
DEADBAND_US = 25


@dataclass
class RCState:
    ch1_us: int = CENTER_PULSE_WIDTH_US
    ch2_us: int = CENTER_PULSE_WIDTH_US
    ch3_us: int = CENTER_PULSE_WIDTH_US
    ch4_us: int = CENTER_PULSE_WIDTH_US
    ch5_us: int = CENTER_PULSE_WIDTH_US
    last_update_epoch_s: float = 0.0

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


def _parse_csv_line(line: str) -> Optional[Tuple[int, int, int, int, int]]:
    parts = line.split(",")
    if len(parts) not in (CSV_FIELDS_OLD, CSV_FIELDS_NEW):
        return None
    try:
        raw_values = [int(p.strip()) for p in parts]
    except ValueError:
        return None
    if len(raw_values) == CSV_FIELDS_NEW:
        return (
            raw_values[0],
            raw_values[1],
            raw_values[2],
            raw_values[3],
            raw_values[4],
        )
    # Backward compatibility for legacy firmware output: ch1,ch2,ch3,ch5
    return (
        raw_values[0],
        raw_values[1],
        raw_values[2],
        CENTER_PULSE_WIDTH_US,
        raw_values[3],
    )


def _apply_deadband(value_us: int) -> int:
    if abs(value_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_PULSE_WIDTH_US
    return value_us


class ArduinoRCReader:
    """
    Background reader that exposes the latest sanitized RC values from the Arduino.

    - Autodetects the serial port unless one is provided
    - Parses lines with 5 CSV fields (ch1,ch2,ch3,ch4,ch5), while accepting legacy 4-field lines
    - Sanitizes each field: if outside [800, 2200] µs, substitute last valid (initial 1500)
    - Applies ±25 µs deadband around 1500 µs
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

        self._last_valid: List[int] = [
            CENTER_PULSE_WIDTH_US,
            CENTER_PULSE_WIDTH_US,
            CENTER_PULSE_WIDTH_US,
            CENTER_PULSE_WIDTH_US,
            CENTER_PULSE_WIDTH_US,
        ]
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
                try:
                    line = raw.decode("utf-8", errors="strict").strip()
                except UnicodeDecodeError:
                    continue
                values = _parse_csv_line(line)
                if values is None:
                    continue

                sanitized = list(values)
                for idx, value in enumerate(values):
                    if value < MIN_ACCEPTABLE_US or value > MAX_ACCEPTABLE_US:
                        sanitized[idx] = self._last_valid[idx]
                    else:
                        sanitized[idx] = value
                        self._last_valid[idx] = value
                    sanitized[idx] = _apply_deadband(sanitized[idx])

                now = time.time()
                self._state = RCState(
                    ch1_us=sanitized[0],
                    ch2_us=sanitized[1],
                    ch3_us=sanitized[2],
                    ch4_us=sanitized[3],
                    ch5_us=sanitized[4],
                    last_update_epoch_s=now,
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


