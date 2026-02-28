"""
Motor driver for OSOYOO Model X through Arduino.

Sends commands over USB serial: "M,<left>,<right>\n"
where <left>/<right> are 0..254 motor bytes with 126 as neutral.
"""

from __future__ import annotations

from typing import Optional, Tuple

from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT

try:
    from pi_app.hardware.arduino_rc import ArduinoRCReader
except Exception:  # pragma: no cover
    ArduinoRCReader = None  # type: ignore


class ArduinoModelXDriver:
    def __init__(self, rc_reader: Optional[ArduinoRCReader] = None) -> None:
        self._last_tracks: Optional[Tuple[int, int]] = None
        self._stops: int = 0
        self._rc_reader = rc_reader

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        self._last_tracks = (
            max(MIN_OUTPUT, min(MAX_OUTPUT, left_byte)),
            max(MIN_OUTPUT, min(MAX_OUTPUT, right_byte)),
        )
        if self._rc_reader is not None:
            self._rc_reader.send_motor_command(self._last_tracks[0], self._last_tracks[1])

    def stop(self) -> None:
        self._stops += 1
        self._last_tracks = (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        if self._rc_reader is not None:
            self._rc_reader.send_motor_command(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)

    @property
    def last_tracks(self) -> Optional[Tuple[int, int]]:
        return self._last_tracks

    @property
    def stops(self) -> int:
        return self._stops
