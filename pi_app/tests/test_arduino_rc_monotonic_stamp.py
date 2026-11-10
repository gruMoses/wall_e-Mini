"""The RC reader stamps each frame with time.monotonic() (2026-11-10).

The RC-stale watchdog in ``Controller.process`` compares the stamp with
``time.monotonic()``. A wall-clock stamp would let a backward clock step
(this Pi's RTC resets at boot, then NTP corrects it) make a dead link look
fresh, so the stamp must be monotonic at the source.
"""

from unittest.mock import patch

import pytest

serial = pytest.importorskip("serial")

from pi_app.hardware import arduino_rc  # noqa: E402
from pi_app.hardware.arduino_rc import ArduinoRCReader, RCState  # noqa: E402


class _FakeSerial:
    def __init__(self, reader, lines):
        self._reader = reader
        self._lines = list(lines)

    def readline(self):
        if self._lines:
            return self._lines.pop(0)
        self._reader._stop_event.set()
        return b""


def test_default_state_has_no_frame_stamp():
    st = RCState()
    assert st.last_update_mono_s == 0.0
    assert not hasattr(st, "last_update_epoch_s")


def test_frame_is_stamped_with_monotonic_not_wall_clock():
    reader = ArduinoRCReader(port="/dev/null")
    reader._serial = _FakeSerial(reader, [b"1600,1400,1900,1000,1000\n"])
    with patch.object(arduino_rc.time, "monotonic", return_value=123.25), \
            patch.object(arduino_rc.time, "time", return_value=9_999_999.0):
        reader._read_loop()
    st = reader.get_state()
    assert st.last_update_mono_s == 123.25
    assert (st.ch1_us, st.ch2_us, st.ch3_us) == (1600, 1400, 1900)
