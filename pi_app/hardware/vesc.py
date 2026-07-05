"""
VESC CAN motor driver with telemetry reception and low-voltage shutdown.

Design:
- TX path (unchanged): set_tracks(left_byte, right_byte) converts byte offsets to RPM and
  sends CAN_PACKET_SET_RPM (extended frame 0x300 + can_id).
- RX path (new): background daemon thread reads VESC status broadcasts and updates
  thread-safe telemetry state.
- Voltage monitor: once pack voltage stays below threshold for a configurable duration,
  motors are stopped and 'sudo shutdown -h now' is invoked (hysteresis: one-shot latch).

VESC CAN status frame layout (all big-endian, extended IDs):
  arbitration_id = (packet_id << 8) | vesc_can_id

  CAN_PACKET_STATUS (9):
    [0:4]  int32   RPM
    [4:6]  int16   motor current × 10  → A
    [6:8]  int16   duty cycle × 1000   → fraction

  CAN_PACKET_STATUS_4 (16):
    [0:2]  int16   temp_fet    × 10  → °C
    [2:4]  int16   temp_motor  × 10  → °C
    [4:6]  int16   current_in  × 10  → A
    [6:8]  int16   pid_pos_err × 50  (not used)

  CAN_PACKET_STATUS_5 (27):
    [0:4]  int32   tachometer (not used)
    [4:6]  int16   input_voltage × 10 → V
    [6:8]  int16   (reserved / SOC — not used)

  Note: observed firmware sends 8-byte frames; voltage is at [4:6].
"""

from __future__ import annotations

import logging
import os
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT

logger = logging.getLogger(__name__)

# VESC CAN packet IDs for status broadcast frames
_CAN_PACKET_STATUS = 9
_CAN_PACKET_STATUS_4 = 16
_CAN_PACKET_STATUS_5 = 27


@dataclass
class _MotorTelemetry:
    """Mutable telemetry state for one motor (access only under VescCanDriver._telem_lock)."""
    rpm: Optional[int] = None
    current_a: Optional[float] = None
    duty_cycle: Optional[float] = None
    temp_fet_c: Optional[float] = None
    temp_motor_c: Optional[float] = None
    current_in_a: Optional[float] = None
    voltage_v: Optional[float] = None
    last_status_s: float = 0.0
    last_status4_s: float = 0.0
    last_status5_s: float = 0.0


@dataclass
class VescTelemetry:
    """Combined telemetry snapshot returned to the control loop.

    Constructed by VescCanDriver.get_telemetry() and consumed by Controller
    for closed-loop speed control and slip detection.
    """
    left_rpm: Optional[int] = None
    right_rpm: Optional[int] = None
    left_current_a: Optional[float] = None
    right_current_a: Optional[float] = None
    left_temp_c: Optional[float] = None
    right_temp_c: Optional[float] = None
    voltage_v: Optional[float] = None
    timestamp: float = 0.0
    can_send_alert: bool = False
    can_send_fail_count: int = 0
    can_rx_frame_count: int = 0
    can_rx_parse_error_count: int = 0
    can_rx_recv_error_count: int = 0
    can_rx_reopen_count: int = 0
    can_rx_last_frame_age_s: Optional[float] = None
    # Per-motor STATUS(9)/RPM frame age — lets a consumer detect ONE motor
    # going silent even while the other keeps transmitting (see get_telemetry()).
    left_status_age_s: Optional[float] = None
    right_status_age_s: Optional[float] = None
    # False if the background RX thread has died (e.g. an unhandled exception
    # escaped _rx_loop). Telemetry can look "fresh" for a while after that
    # because get_telemetry() only reports frame age, not thread liveness.
    rx_thread_alive: Optional[bool] = None


class VescCanDriver:
    def __init__(
        self,
        channel: str = "can0",
        left_id: int = 2,
        right_id: int = 1,
        max_rpm: int = 3000,
        vesc_cfg=None,  # VescConfig instance; kept as Any to avoid import-time coupling
    ) -> None:
        self._channel = channel
        self._left_id = left_id
        self._right_id = right_id
        self._max_rpm = max_rpm
        self._cfg = vesc_cfg
        self._bus = None  # Lazy-init python-can Bus
        self._bus_lock = threading.Lock()

        # Telemetry state — read/write only under _telem_lock
        self._telem_lock = threading.Lock()
        self._left_telem = _MotorTelemetry()
        self._right_telem = _MotorTelemetry()

        # Background RX thread state
        self._stop_event: Optional[threading.Event] = None
        self._rx_thread: Optional[threading.Thread] = None

        # Low-voltage shutdown tracking (accessed only from the RX thread)
        self._low_voltage_since: Optional[float] = None
        self._shutdown_triggered = False

        # CAN send failure tracking (thread-safe via _telem_lock)
        self._send_fail_count: int = 0
        self._send_fail_consecutive: int = 0
        self._send_fail_first_ts: float = 0.0
        self._send_fail_last_ts: float = 0.0
        self._send_fail_alert: bool = False  # True when N consecutive failures within T seconds
        _SEND_FAIL_THRESHOLD = 5        # consecutive failures to trigger alert
        _SEND_FAIL_WINDOW_S = 2.0       # time window for consecutive failure detection
        self._SEND_FAIL_THRESHOLD = _SEND_FAIL_THRESHOLD
        self._SEND_FAIL_WINDOW_S = _SEND_FAIL_WINDOW_S

        # CAN RX health counters — used for live diagnostics and telemetry.
        self._rx_frame_count: int = 0
        # Per-status-type RX counters — the aggregate rx_frame_count can't tell
        # which status packet types are arriving, a blind spot that previously
        # caused a misdiagnosis ("rpm stuck at 0"). These prove ERPM/STATUS(9)
        # packet flow independently of STATUS_4/STATUS_5.
        self._rx_status_count: int = 0   # CAN_PACKET_STATUS   (9)
        self._rx_status4_count: int = 0  # CAN_PACKET_STATUS_4 (16)
        self._rx_status5_count: int = 0  # CAN_PACKET_STATUS_5 (27)
        self._rx_parse_error_count: int = 0
        self._rx_recv_error_count: int = 0
        self._rx_reopen_count: int = 0
        self._rx_last_frame_s: float = 0.0
        self._rx_last_reopen_s: float = 0.0

    # ──────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def detect(channel: str = "can0") -> bool:
        """Lightweight check for CAN network device presence."""
        return os.path.exists(f"/sys/class/net/{channel}")

    def start(self) -> None:
        """Start background CAN telemetry RX thread."""
        self._stop_event = threading.Event()
        t = threading.Thread(target=self._rx_loop, daemon=True, name="vesc-can-rx")
        t.start()
        self._rx_thread = t
        logger.info("VESC CAN RX thread started (left_id=%d right_id=%d)", self._left_id, self._right_id)

    def stop(self) -> None:
        """Command both motors to zero RPM without stopping telemetry RX."""
        self._send_rpm(self._left_id, 0)
        self._send_rpm(self._right_id, 0)
        with self._telem_lock:
            self._send_fail_consecutive = 0

    def shutdown(self) -> None:
        """Terminate background RX thread and close CAN bus."""
        self.stop()
        if self._stop_event is not None:
            self._stop_event.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None
        self._close_bus()

    # ──────────────────────────────────────────────────────────────────────────
    # MotorDriver API
    # ──────────────────────────────────────────────────────────────────────────

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        left_rpm = self._byte_to_rpm(left_byte, self._max_rpm)
        right_rpm = self._byte_to_rpm(right_byte, self._max_rpm)
        self._send_rpm(self._left_id, left_rpm)
        self._send_rpm(self._right_id, right_rpm)

    # ──────────────────────────────────────────────────────────────────────────
    # Telemetry API
    # ──────────────────────────────────────────────────────────────────────────

    def get_voltage(self) -> Optional[float]:
        """Return the most recently received pack input voltage (V), or None."""
        return self._get_pack_voltage()

    def get_rpm(self, motor: str) -> Optional[int]:
        """Return actual electrical RPM for 'left' or 'right' motor, or None."""
        with self._telem_lock:
            return self._telem_for(motor).rpm

    def get_current(self, motor: str) -> Optional[float]:
        """Return motor phase current (A) for 'left' or 'right', or None."""
        with self._telem_lock:
            return self._telem_for(motor).current_a

    def get_temperature(self, motor: str) -> Optional[float]:
        """Return MOSFET temperature (°C) for 'left' or 'right', or None."""
        with self._telem_lock:
            return self._telem_for(motor).temp_fet_c

    def get_telemetry(self) -> Optional[VescTelemetry]:
        """Return a combined telemetry snapshot for the control loop.

        Returns None if no STATUS frames have been received yet (RPM is the
        primary signal; until at least one motor reports RPM the snapshot is
        not useful for closed-loop control).
        """
        with self._telem_lock:
            l = self._left_telem
            r = self._right_telem
            if l.rpm is None and r.rpm is None:
                return None
            lv = l.voltage_v
            rv = r.voltage_v
            readings = [v for v in (lv, rv) if v is not None]
            voltage = max(readings) if readings else None
            now = time.monotonic()
            telem = VescTelemetry(
                left_rpm=l.rpm,
                right_rpm=r.rpm,
                left_current_a=l.current_a,
                right_current_a=r.current_a,
                left_temp_c=l.temp_fet_c,
                right_temp_c=r.temp_fet_c,
                voltage_v=voltage,
                timestamp=max(l.last_status_s, r.last_status_s),
                left_status_age_s=(now - l.last_status_s) if l.last_status_s > 0.0 else None,
                right_status_age_s=(now - r.last_status_s) if r.last_status_s > 0.0 else None,
            )
            telem.can_send_alert = self._send_fail_alert
            telem.can_send_fail_count = self._send_fail_count
            telem.can_rx_frame_count = self._rx_frame_count
            telem.can_rx_parse_error_count = self._rx_parse_error_count
            telem.can_rx_recv_error_count = self._rx_recv_error_count
            telem.can_rx_reopen_count = self._rx_reopen_count
            telem.can_rx_last_frame_age_s = (
                max(0.0, time.monotonic() - self._rx_last_frame_s)
                if self._rx_last_frame_s > 0.0
                else None
            )
        # Thread liveness is independent of _telem_lock (it's a plain
        # threading.Thread reference, not telemetry state).
        _rx_thread = self._rx_thread
        telem.rx_thread_alive = _rx_thread.is_alive() if _rx_thread is not None else None
        return telem

    def get_rx_health(self) -> dict:
        """Return CAN RX health counters for diagnostics/telemetry UI."""
        with self._telem_lock:
            health = {
                "rx_frame_count": int(self._rx_frame_count),
                "rx_status_count": int(self._rx_status_count),
                "rx_status4_count": int(self._rx_status4_count),
                "rx_status5_count": int(self._rx_status5_count),
                "rx_parse_error_count": int(self._rx_parse_error_count),
                "rx_recv_error_count": int(self._rx_recv_error_count),
                "rx_reopen_count": int(self._rx_reopen_count),
                "rx_last_frame_age_s": (
                    max(0.0, time.monotonic() - self._rx_last_frame_s)
                    if self._rx_last_frame_s > 0.0
                    else None
                ),
            }
        _rx_thread = self._rx_thread
        health["rx_thread_alive"] = _rx_thread.is_alive() if _rx_thread is not None else None
        return health

    # ──────────────────────────────────────────────────────────────────────────
    # Background CAN RX loop
    # ──────────────────────────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        stop_event = self._stop_event
        while stop_event is not None and not stop_event.is_set():
            try:
                with self._bus_lock:
                    self._ensure_bus()
            except RuntimeError:
                # python-can unavailable.
                time.sleep(1.0)
                continue
            except Exception as exc:
                # SocketCAN open/init can fail transiently; force reopen path.
                logger.warning("VESC CAN open failed on %s: %s", self._channel, exc)
                self._close_bus()
                with self._telem_lock:
                    self._rx_reopen_count += 1
                time.sleep(1.0)
                continue

            try:
                with self._bus_lock:
                    msg = self._bus.recv(timeout=0.1)  # type: ignore[union-attr]
            except Exception as exc:
                logger.warning("VESC CAN recv error on %s: %s; reopening bus", self._channel, exc)
                with self._telem_lock:
                    self._rx_recv_error_count += 1
                    self._rx_reopen_count += 1
                self._close_bus()
                time.sleep(0.05)
                continue

            if msg is None:
                try:
                    now = time.monotonic()
                    with self._telem_lock:
                        last_frame_s = self._rx_last_frame_s
                        last_reopen_s = self._rx_last_reopen_s
                    if (
                        last_frame_s > 0.0
                        and (now - last_frame_s) > 1.0
                        and (now - last_reopen_s) > 1.0
                    ):
                        logger.warning(
                            "VESC CAN RX stale for %.2fs on %s; reopening bus",
                            now - last_frame_s,
                            self._channel,
                        )
                        with self._telem_lock:
                            self._rx_reopen_count += 1
                            self._rx_last_reopen_s = now
                        self._close_bus()
                except Exception as exc:
                    # This block must never be allowed to kill the daemon RX
                    # thread silently (see rx_thread_alive below) — contain it
                    # the same way every sibling block in this loop is contained.
                    logger.warning("VESC CAN RX stale-check error on %s: %s", self._channel, exc)
                    with self._telem_lock:
                        self._rx_reopen_count += 1
                    self._close_bus()
                continue

            if not msg.is_extended_id:
                continue

            vesc_id = msg.arbitration_id & 0xFF
            packet_id = (msg.arbitration_id >> 8) & 0xFF

            if vesc_id not in (self._left_id, self._right_id):
                continue

            motor = "left" if vesc_id == self._left_id else "right"
            now = time.monotonic()

            try:
                if packet_id == _CAN_PACKET_STATUS:
                    with self._telem_lock:
                        self._rx_frame_count += 1
                        self._rx_status_count += 1
                        self._rx_last_frame_s = now
                    self._parse_status(motor, msg.data, now)
                elif packet_id == _CAN_PACKET_STATUS_4:
                    with self._telem_lock:
                        self._rx_frame_count += 1
                        self._rx_status4_count += 1
                        self._rx_last_frame_s = now
                    self._parse_status4(motor, msg.data, now)
                elif packet_id == _CAN_PACKET_STATUS_5:
                    with self._telem_lock:
                        self._rx_frame_count += 1
                        self._rx_status5_count += 1
                        self._rx_last_frame_s = now
                    self._parse_status5(motor, msg.data, now)
            except Exception as exc:
                with self._telem_lock:
                    self._rx_parse_error_count += 1
                logger.debug("VESC frame parse error (pkt=%d motor=%s): %s", packet_id, motor, exc)

            try:
                self._check_voltage_shutdown()
            except Exception as exc:
                logger.warning("VESC voltage monitor error: %s; reopening bus", exc)
                with self._telem_lock:
                    self._rx_parse_error_count += 1
                    self._rx_reopen_count += 1
                self._close_bus()

    # ──────────────────────────────────────────────────────────────────────────
    # Frame parsers
    # ──────────────────────────────────────────────────────────────────────────

    def _parse_status(self, motor: str, data: bytes, now: float) -> None:
        """CAN_PACKET_STATUS (9): RPM, motor current, duty cycle."""
        if len(data) < 8:
            return
        rpm = struct.unpack_from(">i", data, 0)[0]
        current_raw = struct.unpack_from(">h", data, 4)[0]
        duty_raw = struct.unpack_from(">h", data, 6)[0]
        with self._telem_lock:
            t = self._telem_for(motor)
            t.rpm = rpm
            t.current_a = current_raw / 10.0
            t.duty_cycle = duty_raw / 1000.0
            t.last_status_s = now

    def _parse_status4(self, motor: str, data: bytes, now: float) -> None:
        """CAN_PACKET_STATUS_4 (16): MOSFET temp, motor temp, input current."""
        if len(data) < 8:
            return
        temp_fet_raw = struct.unpack_from(">h", data, 0)[0]
        temp_motor_raw = struct.unpack_from(">h", data, 2)[0]
        current_in_raw = struct.unpack_from(">h", data, 4)[0]
        with self._telem_lock:
            t = self._telem_for(motor)
            t.temp_fet_c = temp_fet_raw / 10.0
            t.temp_motor_c = temp_motor_raw / 10.0
            t.current_in_a = current_in_raw / 10.0
            t.last_status4_s = now

    def _parse_status5(self, motor: str, data: bytes, now: float) -> None:
        """CAN_PACKET_STATUS_5 (27): tachometer (ignored), input voltage.

        Observed VESC firmware sends 8-byte frames with voltage at bytes [4:6].
        """
        if len(data) < 6:
            return
        voltage_raw = struct.unpack_from(">h", data, 4)[0]
        with self._telem_lock:
            t = self._telem_for(motor)
            t.voltage_v = voltage_raw / 10.0
            t.last_status5_s = now

    # ──────────────────────────────────────────────────────────────────────────
    # Low-voltage shutdown
    # ──────────────────────────────────────────────────────────────────────────

    def _get_pack_voltage(self) -> Optional[float]:
        """Return best-available pack voltage.

        Both VESCs are on the same pack.  Taking the maximum of the two readings
        prevents a transient glitch on one controller from causing a false shutdown
        trigger while still detecting genuine pack sag (which affects both).
        """
        with self._telem_lock:
            lv = self._left_telem.voltage_v
            rv = self._right_telem.voltage_v
        readings = [v for v in (lv, rv) if v is not None]
        return max(readings) if readings else None

    def _check_voltage_shutdown(self) -> None:
        """Called from the RX thread after every processed frame."""
        if self._shutdown_triggered or self._cfg is None:
            return

        voltage = self._get_pack_voltage()
        if voltage is None:
            return

        threshold: float = self._cfg.voltage_shutdown_threshold_v
        delay: float = self._cfg.voltage_shutdown_delay_s

        if voltage < threshold:
            if self._low_voltage_since is None:
                self._low_voltage_since = time.monotonic()
                logger.warning(
                    "VESC: pack voltage %.2fV below threshold %.2fV — "
                    "will shut down in %.0fs if sustained",
                    voltage, threshold, delay,
                )
            elif time.monotonic() - self._low_voltage_since >= delay:
                self._trigger_low_voltage_shutdown(voltage)
        else:
            if self._low_voltage_since is not None:
                logger.info(
                    "VESC: voltage recovered to %.2fV — low-voltage timer cleared", voltage
                )
            self._low_voltage_since = None

    def _trigger_low_voltage_shutdown(self, voltage: float) -> None:
        """Latch the shutdown flag, stop motors, and invoke OS shutdown."""
        self._shutdown_triggered = True  # One-shot latch — never cleared
        logger.critical(
            "VESC: CRITICAL LOW VOLTAGE %.2fV (threshold %.2fV) — "
            "stopping motors and initiating OS shutdown",
            voltage,
            self._cfg.voltage_shutdown_threshold_v,  # type: ignore[union-attr]
        )
        # Stop motors immediately from the RX thread (don't wait for join)
        self._send_rpm(self._left_id, 0)
        self._send_rpm(self._right_id, 0)
        # Signal the RX loop to exit (we ARE the RX thread; join happens externally)
        if self._stop_event is not None:
            self._stop_event.set()
        # Run OS shutdown in a separate thread so we don't block or deadlock
        threading.Thread(
            target=lambda: subprocess.run(["sudo", "shutdown", "-h", "now"], check=False),
            daemon=True,
            name="vesc-os-shutdown",
        ).start()

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _telem_for(self, motor: str) -> _MotorTelemetry:
        """Return the telemetry object for 'left' or 'right' (caller holds lock)."""
        return self._left_telem if motor == "left" else self._right_telem

    def _ensure_bus(self) -> None:
        if self._bus is not None:
            return
        try:
            import can  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                "python-can is required for VESC CAN control. "
                "Install with: pip install python-can"
            ) from exc
        self._bus = can.interface.Bus(channel=self._channel, bustype="socketcan")

    def _close_bus(self) -> None:
        with self._bus_lock:
            bus = self._bus
            self._bus = None
        if bus is None:
            return
        try:
            shutdown = getattr(bus, "shutdown", None)
            if callable(shutdown):
                shutdown()
        except Exception:
            pass

    def _send_rpm(self, can_id: int, rpm: int) -> None:
        try:
            with self._bus_lock:
                self._ensure_bus()
        except Exception as exc:
            # RuntimeError (python-can missing) or a SocketCAN OSError/CanError
            # from Bus() construction — either way this must never escape to
            # the caller (set_tracks -> Controller.process -> the unguarded
            # main loop). Close/reset the bus and count it like a send failure
            # so callers see it in can_send_alert/can_send_fail_count.
            now = time.monotonic()
            with self._telem_lock:
                self._send_fail_count += 1
                self._send_fail_consecutive += 1
                if self._send_fail_first_ts == 0.0:
                    self._send_fail_first_ts = now
                self._send_fail_last_ts = now
                if (self._send_fail_consecutive >= self._SEND_FAIL_THRESHOLD
                        and (now - self._send_fail_first_ts) <= self._SEND_FAIL_WINDOW_S):
                    if not self._send_fail_alert:
                        self._send_fail_alert = True
                        logger.warning(
                            'VESC CAN send alert: %d consecutive failures in %.1fs - %s',
                            self._send_fail_consecutive,
                            now - self._send_fail_first_ts,
                            exc,
                        )
            self._close_bus()
            return

        try:
            import can  # type: ignore
        except Exception:
            return

        arbitration_id = 0x300 + can_id
        data = struct.pack(">i", int(rpm))
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        close_bus = False
        try:
            with self._bus_lock:
                assert self._bus is not None
                self._bus.send(msg)
            with self._telem_lock:
                self._send_fail_consecutive = 0
                if self._send_fail_alert:
                    logger.info('VESC CAN send recovered after %d failures', self._send_fail_count)
                    self._send_fail_alert = False
        except Exception as exc:
            now = time.monotonic()
            with self._telem_lock:
                self._send_fail_count += 1
                self._send_fail_consecutive += 1
                if self._send_fail_first_ts == 0.0:
                    self._send_fail_first_ts = now
                self._send_fail_last_ts = now
                if (self._send_fail_consecutive >= self._SEND_FAIL_THRESHOLD
                        and (now - self._send_fail_first_ts) <= self._SEND_FAIL_WINDOW_S):
                    if not self._send_fail_alert:
                        self._send_fail_alert = True
                        logger.warning(
                            'VESC CAN send alert: %d consecutive failures in %.1fs - %s',
                            self._send_fail_consecutive,
                            now - self._send_fail_first_ts,
                            exc,
                        )
                close_bus = True
        if close_bus:
            self._close_bus()

    @staticmethod
    def _byte_to_rpm(byte_value: int, max_rpm: Optional[int] = None) -> int:
        max_rpm_val = 3000 if max_rpm is None else max_rpm
        clamped = min(MAX_OUTPUT, max(MIN_OUTPUT, byte_value))
        if clamped == CENTER_OUTPUT_VALUE:
            return 0
        if clamped > CENTER_OUTPUT_VALUE:
            span_up = MAX_OUTPUT - CENTER_OUTPUT_VALUE
            normalized = (clamped - CENTER_OUTPUT_VALUE) / span_up
            return int(round(normalized * max_rpm_val))
        else:
            normalized = (CENTER_OUTPUT_VALUE - clamped) / CENTER_OUTPUT_VALUE
            return -int(round(normalized * max_rpm_val))
