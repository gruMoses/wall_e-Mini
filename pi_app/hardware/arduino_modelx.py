"""
Stub motor driver for OSOYOO Model X through Arduino.

Assumption: The Arduino will be updated to accept simple commands over the same
USB serial link (or a second one), e.g., lines like: "M,<left>,<right>\n"
where <left>/<right> are 0..255 track bytes with 126 as neutral.

This module provides a MotorDriver-compatible class that buffers the latest
command. Actual serial I/O can be added when the Arduino firmware is ready.
"""

from __future__ import annotations

from typing import Optional, Tuple

try:
    from pi_app.hardware.arduino_rc import ArduinoRCReader
except Exception:  # pragma: no cover
    ArduinoRCReader = None  # type: ignore


class ArduinoModelXDriver:
    def __init__(self, rc_reader: Optional[ArduinoRCReader] = None) -> None:
        self._last_tracks: Optional[Tuple[int, int]] = None
        self._stops: int = 0
        self._rc_reader = rc_reader

    # MotorDriver API
    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        # TODO: send to Arduino when protocol is defined
        self._last_tracks = (max(0, min(254, left_byte)), max(0, min(254, right_byte)))
        if self._rc_reader is not None:
            self._rc_reader.send_motor_command(self._last_tracks[0], self._last_tracks[1])

    def stop(self) -> None:
        # TODO: send stop/neutral command to Arduino
        self._stops += 1
        self._last_tracks = (126, 126)
        if self._rc_reader is not None:
            self._rc_reader.send_motor_command(126, 126)

    # Introspection for tests/monitoring
    @property
    def last_tracks(self) -> Optional[Tuple[int, int]]:
        return self._last_tracks

    @property
    def stops(self) -> int:
        return self._stops


