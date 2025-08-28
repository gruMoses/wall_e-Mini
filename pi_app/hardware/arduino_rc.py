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


CSV_FIELDS = 4
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
    ch5_us: int = CENTER_PULSE_WIDTH_US
    last_update_epoch_s: float = 0.0

    def as_tuple(self) -> Tuple[int, int, int, int]:
        return (self.ch1_us, self.ch2_us, self.ch3_us, self.ch5_us)


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
    """Open a device and confirm it emits lines with exactly 4 integer CSV fields."""
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


def _parse_csv_line(line: str) -> Optional[Tuple[int, int, int, int]]:
    parts = line.split(",")
    if len(parts) != CSV_FIELDS:
        return None
    try:
        values = tuple(int(p.strip()) for p in parts)  # type: ignore[assignment]
    except ValueError:
        return None
    return values  # type: ignore[return-value]


def _apply_deadband(value_us: int) -> int:
    if abs(value_us - CENTER_PULSE_WIDTH_US) <= DEADBAND_US:
        return CENTER_PULSE_WIDTH_US
    return value_us


class ArduinoRCReader:
    """
    Background reader that exposes the latest sanitized RC values from the Arduino.

    - Autodetects the serial port unless one is provided
    - Parses lines with exactly 4 integer CSV fields
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

    # Internal
    def _read_loop(self) -> None:
        global RC_READ_ERROR_COUNT
        assert self._serial is not None
        ser = self._serial
        while not self._stop_event.is_set():
            try:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="strict").strip()
                except UnicodeDecodeError:
                    # Skip undecodable lines
                    continue
                values = _parse_csv_line(line)
                if values is None:
                    # Ignore non-CSV debug lines like "DBG raw: ..."
                    continue

                sanitized = list(values)
                for idx, value in enumerate(values):
                    if value < MIN_ACCEPTABLE_US or value > MAX_ACCEPTABLE_US:
                        sanitized[idx] = self._last_valid[idx]
                    else:
                        sanitized[idx] = value
                        self._last_valid[idx] = value
                    # Apply deadband around 1500 us
                    sanitized[idx] = _apply_deadband(sanitized[idx])

                now = time.time()
                self._state = RCState(
                    ch1_us=sanitized[0],
                    ch2_us=sanitized[1],
                    ch3_us=sanitized[2],
                    ch5_us=sanitized[3],
                    last_update_epoch_s=now,
                )
            except Exception:
                RC_READ_ERROR_COUNT += 1
                if RC_READ_ERRORS is not None:
                    RC_READ_ERRORS.inc()
                logger.exception("RC read failed")
                time.sleep(0.01)

    # Motor command sending for Arduino-based motor driver fallback
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


