"""Arduino RC line parser: 5 or 6 fields and the no-signal sentinel (S12).

Safety findings 1 and 5 of docs/rc_arduino_gps_wiring.md: the new sketch
sends ``ch1,ch2,ch3,ch4,ch5,ch6*HH`` with RC_NO_SIGNAL_US (0) for a channel
that has no valid pulse and a CRC-8 after "*". The Pi must accept that
format BEFORE any firmware sends it, and today's 5-field lines from today's
firmware (dcc550e) must give exactly the same RCState as before.

Contract change in review (Grok, 2026-11-13): the 6-field format carries a
CRC and 5-field lines keep the old 800..2200 range, because with 500..2500
and no CRC a dropped "1" made "1500" a valid 500 us (full stick, switch
OFF). So a 5-field line with a 0 is now dropped.
"""

import random
from unittest.mock import patch

import pytest

serial = pytest.importorskip("serial")

from pi_app.control.rc_signal import RC_NO_SIGNAL_US, rc_line_crc8  # noqa: E402
from pi_app.hardware import arduino_rc  # noqa: E402
from pi_app.hardware.arduino_rc import (  # noqa: E402
    ArduinoRCReader,
    RCState,
    _LineFramer,
    _parse_csv_line,
)


def l6(values, crc=None):
    """A 6-field line as the new sketch prints it (no line end)."""
    payload = ",".join(str(v) for v in values)
    if crc is None:
        crc = rc_line_crc8(payload.encode("ascii"))
    return f"{payload}*{crc:02X}"


def b6(values):
    return (l6(values) + "\r\n").encode("ascii")


class _FakeSerial:
    """readline() returns the given lines, then stops the read loop.

    ``states`` records the reader state before each read, i.e. after the
    previous line was processed.
    """

    def __init__(self, reader, lines):
        self._reader = reader
        self._lines = list(lines)
        self.states = []

    def readline(self):
        self.states.append(self._reader.get_state())
        if self._lines:
            return self._lines.pop(0)
        self._reader._stop_event.set()
        return b""


def _run_reader(lines, new_port=False):
    reader = ArduinoRCReader(port="/dev/null")
    if new_port:
        reader._framer.new_port()
    fake = _FakeSerial(reader, lines)
    reader._serial = fake
    clock = {"t": 100.0}

    def mono():
        clock["t"] += 0.02
        return clock["t"]

    with patch.object(arduino_rc.time, "monotonic", side_effect=mono):
        reader._read_loop()
    # states[0] is before the first line; states[i] is after line i.
    return reader, fake.states[1:]


# --- _parse_csv_line ---------------------------------------------------------


def test_five_fields_parse_with_no_ch6():
    assert _parse_csv_line("1600,1400,1900,1000,1000") == (1600, 1400, 1900, 1000, 1000, None)


def test_six_fields_parse_with_ch6():
    assert _parse_csv_line(l6([1600, 1400, 1900, 1000, 1000, 1200])) == (
        1600, 1400, 1900, 1000, 1000, 1200,
    )


def test_crc_matches_the_sketch_vectors():
    # The same vectors as tools/arduino_rc_reader_host_test/test_rc_channel.cpp.
    assert rc_line_crc8(b"123456789") == 0xF4
    assert l6([1500, 1500, 2000, 1000, 1000, 1500]) == "1500,1500,2000,1000,1000,1500*25"
    assert l6([0, 0, 0, 0, 0, 0]) == "0,0,0,0,0,0*10"
    assert l6([992, 2004, 2004, 992, 996, 0]) == "992,2004,2004,992,996,0*AD"
    assert l6([500, 2500, 1500, 1000, 2000, 0]) == "500,2500,1500,1000,2000,0*51"


@pytest.mark.parametrize("idx", range(6))
def test_the_sentinel_is_accepted_in_every_field(idx):
    values = [1500, 1500, 2000, 1000, 1000, 1500]
    values[idx] = RC_NO_SIGNAL_US
    assert _parse_csv_line(l6(values)) == tuple(values)


def test_six_field_range_edges_are_valid():
    assert _parse_csv_line(l6([500, 2500, 500, 2500, 500, 2500])) == (500, 2500, 500, 2500, 500, 2500)


def test_five_field_range_edges_are_the_old_ones():
    assert _parse_csv_line("800,2200,800,2200,800") == (800, 2200, 800, 2200, 800, None)
    assert _parse_csv_line("799,1500,2000,1000,1000") is None
    assert _parse_csv_line("1500,1500,2201,1000,1000") is None


def test_a_dropped_digit_is_never_a_valid_width():
    """Grok review: "1500" without its "1" is 500 us, a full stick or a
    switch OFF. The 5-field range and the 6-field CRC keep it out."""
    assert _parse_csv_line("500,1500,1500,1000,1000") is None  # 5 fields
    good = l6([1500, 1500, 1500, 1000, 1000, 1500])
    payload, crc = good.split("*")
    for i in range(len(payload)):
        damaged = payload[:i] + payload[i + 1:] + "*" + crc
        assert _parse_csv_line(damaged) is None, damaged


@pytest.mark.parametrize(
    "line",
    [
        "",
        "RC Reader initialized",
        "DBG width/seq/age_us: 1500/12/400 1500/12/400",
        "1500,1500,2000,1000",  # 4 fields: the old ch1,ch2,ch3,ch5 format, or a tail
        "1500,1500,2000,1000,1000,1500",  # 6 fields with no CRC
        "1500,1500,2000,1000,1000,1500*24",  # wrong CRC (right one is 25)
        "1500,1500,2000,1000,1000,1500*2a",  # lower-case hex
        "1500,1500,2000,1000,1000,1500*025",
        l6([1500, 1500, 2000, 1000, 1000, 1500, 1500]),  # 7 fields
        l6([1500, 1500, 2000, 1000, 1000]),  # 5 fields with a CRC
        "0,0,0,0,0",  # the sentinel in a 5-field line: dcc550e never sends it
        "1500,1500,2000,1000,",
        ",1500,1500,2000,1000,1000",
        "1500,,2000,1000,1000",
        "1500, 1500,2000,1000,1000",  # a space
        "1500,1500,2000,1000,-1",  # a sign
        "+1500,1500,2000,1000,1000",
        "1_500,1500,2000,1000,1000",  # int() would accept this
        "1500,1500,2000,1000,1000x",
        "15001500,2000,1000,1000,1500",  # merged fields
        l6([499, 1500, 2000, 1000, 1000, 1500]),  # below the valid range, CRC good
        l6([1500, 1500, 2501, 1000, 1000, 1500]),  # above the valid range, CRC good
        l6([1500, 1500, 2000, 1000, 1000, 1]),  # ch6 out of range, CRC good
        l6(["01500", 1500, 2000, 1000, 1000, 1500]),  # 5 digits
    ],
)
def test_malformed_lines_are_dropped_whole(line):
    assert _parse_csv_line(line) is None


def test_zero_with_leading_zeros_is_the_sentinel():
    # "00" is the integer 0. The sketch never prints leading zeros; with a
    # good CRC the Pi reads it as no signal, the safe direction.
    assert _parse_csv_line(l6(["00", 1500, 2000, 1000, 1000, 1500])) == (
        0, 1500, 2000, 1000, 1000, 1500,
    )


# --- _LineFramer ---------------------------------------------------------------


GOOD6 = [1500, 1500, 2000, 1000, 1000, 1500]


def test_framer_drops_the_first_line_after_a_port_opens():
    f = _LineFramer()
    f.new_port()
    assert f.feed(b"1500,2000,1000,1000,1500\r\n") is None  # maybe a tail
    assert f.feed(b6(GOOD6)) == tuple(GOOD6)


def test_framer_drops_an_unterminated_read_and_the_next_line():
    f = _LineFramer()
    assert f.feed(b"1500,1500,2000,1000,1000,1") is None  # readline timed out
    assert f.feed(b"500,1500,2000,1000,1000,1500\r\n") is None  # the rest of it
    assert f.feed(b6(GOOD6)) is not None


def test_framer_drops_five_field_lines_after_a_six_field_line():
    f = _LineFramer()
    assert f.feed(b6(GOOD6)) is not None
    # A 6-field line that lost ",1500*25": it looks like a 5-field line of
    # today's firmware. Once a 6-field line was seen on the port, it is not.
    assert f.feed(b"1500,1500,2000,1000,1000\r\n") is None
    assert f.feed(b6(GOOD6)) is not None


def test_framer_new_port_forgets_the_six_field_latch():
    f = _LineFramer()
    assert f.feed(b6(GOOD6)) is not None
    f.new_port()
    assert f.feed(b"RC Reader initialized\r\n") is None
    assert f.feed(b"1500,1500,2000,1000,1000\r\n") == (1500, 1500, 2000, 1000, 1000, None)


def test_framer_drops_non_ascii():
    f = _LineFramer()
    assert f.feed(b"1500,1500,\xff2000,1000,1000\r\n") is None
    assert f.feed(b"1500,1500,2000,1000,1000\r\n") is not None


# --- ArduinoRCReader._read_loop -------------------------------------------------


def test_reader_keeps_the_sentinel_and_ch6():
    reader, states = _run_reader([b6([1600, 0, 1900, 0, 1000, 0]), b6([1600, 1400, 0, 1000, 1000, 1200])])
    a, b = states[0], states[1]
    assert (a.ch1_us, a.ch2_us, a.ch3_us, a.ch4_us, a.ch5_us, a.ch6_us) == (1600, 0, 1900, 0, 1000, 0)
    assert (b.ch1_us, b.ch2_us, b.ch3_us, b.ch4_us, b.ch5_us, b.ch6_us) == (1600, 1400, 0, 1000, 1000, 1200)
    assert b.last_update_mono_s > a.last_update_mono_s > 0.0


def test_reader_drops_a_corrupt_line_without_refreshing_the_stamp():
    reader, states = _run_reader([b"1600,1400,1900,1000,1000\r\n", b"1600,1400,19,1000,1000\r\n"])
    assert states[1] is states[0]  # no new RCState
    assert states[1].ch3_us == 1900


def test_reader_after_open_ignores_the_first_line():
    reader, states = _run_reader(
        [b"00,2000,1000,1000,1500\r\n", b"1500,1500,2000,1000,1000\r\n"], new_port=True,
    )
    assert states[0].last_update_mono_s == 0.0  # first line dropped
    assert states[1].ch3_us == 2000


def test_reader_applies_the_deadband_to_widths_only():
    reader, states = _run_reader([b6([1520, 1475, 0, 1510, 1000, 1490])])
    s = states[0]
    assert (s.ch1_us, s.ch2_us, s.ch3_us, s.ch4_us, s.ch5_us, s.ch6_us) == (1500, 1500, 0, 1500, 1000, 1500)


def test_default_state_is_unchanged():
    st = RCState()
    assert (st.ch1_us, st.ch2_us, st.ch3_us, st.ch4_us, st.ch5_us) == (1500,) * 5
    assert st.ch6_us is None and st.last_update_mono_s == 0.0
    assert st.as_tuple() == (1500,) * 5


# --- Regression: today's 5-field lines give the same state as before ----------


def _old_reader_values(lines):
    """The reader before this change (git 8e2bb70), frozen for comparison."""
    min_us, max_us, center, deadband = 800, 2200, 1500, 25
    last_valid = [center] * 5
    out = []
    for raw in lines:
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="strict").strip()
        except UnicodeDecodeError:
            continue
        parts = line.split(",")
        if len(parts) not in (4, 5):
            continue
        try:
            vals = [int(p.strip()) for p in parts]
        except ValueError:
            continue
        if len(vals) == 4:
            vals = [vals[0], vals[1], vals[2], center, vals[3]]
        sanitized = []
        for idx, v in enumerate(vals):
            if v < min_us or v > max_us:
                v = last_valid[idx]
            else:
                last_valid[idx] = v
            if abs(v - center) <= deadband:
                v = center
            sanitized.append(v)
        out.append(tuple(sanitized))
    return out


def _todays_lines():
    """Lines today's firmware can send: widths 800..2200 (it sends 1500 for
    anything outside), the logged extremes, and the transmitter-off values."""
    rng = random.Random(20261113)
    lines = [
        b"1500,1500,1500,1500,1500\r\n",  # sketch default / clamped
        b"992,2004,2004,992,996\r\n",
        b"1500,1500,1004,1000,1000\r\n",  # transmitter off, 2026-11-11
        b"800,2200,800,2200,800\r\n",
        b"1525,1475,1526,1474,1500\r\n",  # deadband edges
        b"2000,2000,2004,2004,2000\r\n",  # e-stop high
    ]
    for _ in range(2000):
        vals = [rng.randint(800, 2200) for _ in range(5)]
        lines.append((",".join(map(str, vals)) + "\r\n").encode())
    return lines


def test_todays_lines_give_the_same_state_as_the_old_reader():
    lines = _todays_lines()
    expected = _old_reader_values(lines)
    reader, states = _run_reader(lines)
    got = [(s.ch1_us, s.ch2_us, s.ch3_us, s.ch4_us, s.ch5_us) for s in states[: len(lines)]]
    assert len(expected) == len(lines)
    assert got == expected
    assert all(s.ch6_us is None for s in states)
    # Each line refreshed the stamp, as before.
    stamps = [s.last_update_mono_s for s in states[: len(lines)]]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))


def test_todays_lines_are_found_by_port_detection():
    assert _parse_csv_line("1500,1500,1004,1000,1000") is not None
