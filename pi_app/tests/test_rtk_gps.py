"""Tests for the DFRobot RTK GPS reader (2026-09-19, Commit D): UTC/geoid
decode, the register-32-then-0..31 read order, publishing every fix quality
(no more silent low-quality drop), UTC-second dedup (poll-rate aliasing
fix), LoRa mode write-only-on-mismatch, RMC/VTG parsing, fix-quality
transition logging, and the health dict.

No real I2C hardware or smbus2 install is required: _poll()/_ensure_lora_mode()/
_read_gnss_sentence() take the i2c handle as a plain argument (duck-typed),
so a small FakeI2C stands in. Only detect() imports smbus2 itself; that test
stubs sys.modules["smbus2"], mirroring test_oak_reconnect.py's depthai stub.
"""
import sys
import types
import unittest

from config import GpsConfig
from pi_app.hardware.rtk_gps import (
    RtkGpsReader,
    parse_rmc_sentence,
    parse_vtg_sentence,
    _nmea_checksum_valid,
)


def _nmea_checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"


def _nmea(body: str) -> str:
    return f"${body}*{_nmea_checksum(body)}"


RMC_WITH_COURSE = _nmea(
    "GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,A"
)
RMC_NO_COURSE = _nmea(
    "GNRMC,123519,A,4807.038,N,01131.000,E,,,230394,,,A"
)
VTG_SENTENCE = _nmea("GNVTG,084.4,T,077.0,M,022.4,N,041.5,K,A")


class FakeI2C:
    """A dict-of-registers fake for read_i2c_block_data/write_i2c_block_data,
    plus optional named byte-string "sentences" served through the RMC/VTG
    length+chunk protocol (registers 83/84 and 87/88)."""

    def __init__(self, block: list[int], sentences: dict[int, str] | None = None):
        self.block = list(block)  # registers 0..32
        self.sentences = sentences or {}  # {len_reg: sentence_str}
        self.reads: list[tuple[int, int]] = []
        self.writes: list[tuple[int, list[int]]] = []
        self._offsets: dict[int, int] = {}
        self.mode_reg_value = 10  # REG_OPERATION default: already LoRa

    def read_i2c_block_data(self, addr, reg, n):
        self.reads.append((reg, n))
        if reg == 93:
            return [self.mode_reg_value]
        if reg in (83, 87):  # *_LEN register
            sentence = self.sentences.get(reg, "")
            return [len(sentence)]
        if reg in (84, 88):  # *_ALL register
            len_reg = 83 if reg == 84 else 87
            sentence = self.sentences.get(len_reg, "")
            off = self._offsets.get(len_reg, 0)
            return [ord(c) for c in sentence[off:off + n]]
        if 0 <= reg <= 32:
            return self.block[reg:reg + n]
        raise AssertionError(f"unexpected register read: {reg}")

    def write_i2c_block_data(self, addr, reg, data):
        self.writes.append((reg, list(data)))
        if reg in (83, 87):
            self._offsets[reg] = data[0]
        if reg == 93:
            self.mode_reg_value = data[0]


def make_block(
    *, year=2026, month=9, date=19, hour=12, minute=0, second=0,
    lat_dd=40, lat_mm=7, lat_mmmmm=2280, lat_dir="N",
    lon_ddd=74, lon_mm=0, lon_mmmmm=0, lon_dir="W",
    quality=4, sats=14, hdop_z=0, hdop_x=80,
    alt_h=10, alt_l=0, alt_x=0,
    sep_h=0, sep_l=20, sep_x=0,
    dif_z=1, dif_x=25, difid_h=0, difid_l=7,
) -> list[int]:
    """Build a 33-byte register-0..32 block with the given field values,
    encoded exactly as the DFRobot protocol packs them."""
    b = [0] * 33
    b[0], b[1] = year // 256, year % 256
    b[2], b[3], b[4], b[5], b[6] = month, date, hour, minute, second
    b[7], b[8] = lat_dd, lat_mm
    b[9] = (lat_mmmmm >> 16) & 0xFF
    b[10] = (lat_mmmmm >> 8) & 0xFF
    b[11] = lat_mmmmm & 0xFF
    b[12] = ord(lat_dir)
    b[13], b[14] = lon_ddd, lon_mm
    b[15] = (lon_mmmmm >> 16) & 0xFF
    b[16] = (lon_mmmmm >> 8) & 0xFF
    b[17] = lon_mmmmm & 0xFF
    b[18] = ord(lon_dir)
    b[19], b[20] = quality, sats
    b[21], b[22] = hdop_z, hdop_x
    b[23], b[24], b[25] = alt_h, alt_l, alt_x
    b[26], b[27], b[28] = sep_h, sep_l, sep_x
    b[29], b[30] = dif_z, dif_x
    b[31], b[32] = difid_h, difid_l
    return b


def _reader(**cfg_overrides) -> RtkGpsReader:
    cfg_overrides.setdefault("read_nmea_sentences", False)
    cfg = GpsConfig(**cfg_overrides)
    return RtkGpsReader(cfg)


class TestNmeaChecksum(unittest.TestCase):
    def test_valid_checksum_accepted(self):
        self.assertTrue(_nmea_checksum_valid(RMC_WITH_COURSE))

    def test_corrupted_checksum_rejected(self):
        bad = RMC_WITH_COURSE[:-2] + "00"
        self.assertFalse(_nmea_checksum_valid(bad))

    def test_missing_delimiters_rejected(self):
        self.assertFalse(_nmea_checksum_valid("not a sentence"))


class TestParseRmc(unittest.TestCase):
    def test_parses_course_speed_mode(self):
        cog, sog, mode = parse_rmc_sentence(RMC_WITH_COURSE)
        self.assertAlmostEqual(cog, 84.4)
        self.assertAlmostEqual(sog, 22.4 * 0.514444)
        self.assertEqual(mode, "A")

    def test_empty_course_field_is_none(self):
        cog, sog, mode = parse_rmc_sentence(RMC_NO_COURSE)
        self.assertIsNone(cog)
        self.assertIsNone(sog)
        self.assertEqual(mode, "A")

    def test_bad_checksum_raises(self):
        corrupted = RMC_WITH_COURSE[:-2] + "00"
        with self.assertRaises(ValueError):
            parse_rmc_sentence(corrupted)

    def test_wrong_sentence_type_raises(self):
        with self.assertRaises(ValueError):
            parse_rmc_sentence(VTG_SENTENCE)


class TestParseVtg(unittest.TestCase):
    def test_parses_course_and_speed(self):
        cog, sog = parse_vtg_sentence(VTG_SENTENCE)
        self.assertAlmostEqual(cog, 84.4)
        self.assertAlmostEqual(sog, 22.4 * 0.514444)


class TestBlockDecode(unittest.TestCase):
    def test_utc_iso(self):
        raw = make_block(year=2026, month=9, date=19, hour=12, minute=34, second=56)
        self.assertEqual(RtkGpsReader._format_utc(raw), "2026-09-19T12:34:56Z")

    def test_utc_iso_none_when_year_zero(self):
        raw = make_block(year=0)
        self.assertIsNone(RtkGpsReader._format_utc(raw))

    def test_geoid_sep_positive_and_negative(self):
        pos = make_block(sep_h=1, sep_l=44, sep_x=50)
        self.assertAlmostEqual(RtkGpsReader._parse_sep(pos), 300.5)
        neg = make_block(sep_h=1 | 0x80, sep_l=44, sep_x=50)
        self.assertAlmostEqual(RtkGpsReader._parse_sep(neg), -300.5)

    def test_register_32_read_before_0_31_block(self):
        """station_id must not tear: read register 32 (DIFID_L) FIRST, then
        registers 0-31 (which includes DIFID_H at 31) as one block."""
        reader = _reader()
        i2c = FakeI2C(make_block(difid_h=0, difid_l=7))
        reader._poll(i2c, 0x20)
        self.assertEqual(i2c.reads[0], (32, 1))
        self.assertEqual(i2c.reads[1], (0, 32))

    def test_full_reading_fields(self):
        reader = _reader()
        i2c = FakeI2C(make_block(
            lat_dd=40, lat_mm=7, lat_mmmmm=2280, lat_dir="N",
            lon_ddd=74, lon_mm=0, lon_mmmmm=0, lon_dir="W",
            quality=4, sats=14, hdop_z=0, hdop_x=80,
            alt_h=10, alt_l=0, alt_x=0, dif_z=1, dif_x=25,
            difid_h=0, difid_l=7,
        ))
        reader._poll(i2c, 0x20)
        r = reader.get_reading()
        self.assertIsNotNone(r)
        self.assertAlmostEqual(r.latitude, 40 + 7 / 60.0 + 2280 / 100000.0 / 60.0)
        self.assertAlmostEqual(r.longitude, -(74 + 0 / 60.0))
        self.assertEqual(r.fix_quality, 4)
        self.assertEqual(r.satellites_used, 14)
        self.assertAlmostEqual(r.hdop, 0.8)
        self.assertAlmostEqual(r.altitude_m, 2560.0)
        self.assertAlmostEqual(r.diff_age_s, 1.25)
        self.assertEqual(r.station_id, 7)
        self.assertEqual(r.utc_iso, "2026-09-19T12:00:00Z")


class TestLowQualityEpochsPublished(unittest.TestCase):
    def test_quality_0_still_published(self):
        # min_quality is gone: every reading publishes with its real
        # fix_quality now, no reader-side drop.
        reader = _reader()
        i2c = FakeI2C(make_block(quality=0))
        reader._poll(i2c, 0x20)
        r = reader.get_reading()
        self.assertIsNotNone(r)
        self.assertEqual(r.fix_quality, 0)

    def test_gps_config_has_no_min_quality_field(self):
        self.assertFalse(hasattr(GpsConfig(), "min_quality"))


class TestUtcSecondDedup(unittest.TestCase):
    def test_same_second_does_not_republish(self):
        reader = _reader()
        block = make_block(second=10)
        reader._poll(FakeI2C(block), 0x20)
        first = reader.get_reading()
        reader._poll(FakeI2C(block), 0x20)
        second = reader.get_reading()
        self.assertIs(first, second, "polling within the same UTC second must not republish")

    def test_new_second_republishes(self):
        reader = _reader()
        reader._poll(FakeI2C(make_block(second=10)), 0x20)
        first = reader.get_reading()
        reader._poll(FakeI2C(make_block(second=11)), 0x20)
        second = reader.get_reading()
        self.assertIsNot(first, second)
        self.assertEqual(second.utc_iso, "2026-09-19T12:00:11Z")

    def test_data_flush_register_never_read_for_gating(self):
        reader = _reader()
        i2c = FakeI2C(make_block())
        reader._poll(i2c, 0x20)
        self.assertNotIn(80, [reg for reg, _ in i2c.reads])


class TestLoraModeWriteOnlyOnMismatch(unittest.TestCase):
    def test_no_write_when_already_lora(self):
        reader = _reader()
        i2c = FakeI2C(make_block())
        i2c.mode_reg_value = 10  # MODULE_LORA
        reader._ensure_lora_mode(i2c, 0x20)
        self.assertEqual(i2c.writes, [])
        self.assertEqual(reader.get_health()["mode_register_value"], 10)

    def test_writes_and_reads_back_on_mismatch(self):
        reader = _reader()
        i2c = FakeI2C(make_block())
        i2c.mode_reg_value = 20  # MODULE_4G
        reader._ensure_lora_mode(i2c, 0x20)
        self.assertEqual(i2c.writes, [(93, [10])])
        self.assertEqual(i2c.mode_reg_value, 10)
        self.assertEqual(reader.get_health()["mode_register_value"], 10)


class TestRmcErrorHealthCounter(unittest.TestCase):
    def test_bad_checksum_increments_rmc_errors_and_no_cog(self):
        reader = _reader(read_nmea_sentences=True)
        corrupted = RMC_WITH_COURSE[:-2] + "00"
        i2c = FakeI2C(make_block(), sentences={83: corrupted})
        reader._poll(i2c, 0x20)
        r = reader.get_reading()
        self.assertIsNone(r.cog_deg)
        self.assertIsNone(r.sog_mps)
        self.assertEqual(reader.get_health()["rmc_errors"], 1)
        # A bad RMC must never break the position read.
        self.assertEqual(r.fix_quality, 4)

    def test_good_rmc_populates_cog_sog_mode(self):
        reader = _reader(read_nmea_sentences=True)
        i2c = FakeI2C(make_block(), sentences={83: RMC_WITH_COURSE})
        reader._poll(i2c, 0x20)
        r = reader.get_reading()
        self.assertAlmostEqual(r.cog_deg, 84.4)
        self.assertAlmostEqual(r.sog_mps, 22.4 * 0.514444)
        self.assertEqual(r.nmea_mode, "A")
        self.assertEqual(reader.get_health()["rmc_errors"], 0)

    def test_vtg_fallback_when_rmc_course_empty(self):
        reader = _reader(read_nmea_sentences=True)
        i2c = FakeI2C(
            make_block(),
            sentences={83: RMC_NO_COURSE, 87: VTG_SENTENCE},
        )
        reader._poll(i2c, 0x20)
        r = reader.get_reading()
        self.assertAlmostEqual(r.cog_deg, 84.4)  # from VTG
        self.assertAlmostEqual(r.sog_mps, 22.4 * 0.514444)  # from VTG (RMC's was also empty)


class TestFixQualityTransitionLogging(unittest.TestCase):
    def test_transition_logs_at_warning_with_context(self):
        reader = _reader()
        reader._poll(FakeI2C(make_block(second=1, quality=1)), 0x20)
        with self.assertLogs("pi_app.hardware.rtk_gps", level="WARNING") as cm:
            reader._poll(FakeI2C(make_block(second=2, quality=4)), 0x20)
        joined = " ".join(cm.output)
        self.assertIn("fix_quality", joined)
        self.assertIn("1", joined)
        self.assertIn("4", joined)

    def test_no_log_when_quality_unchanged(self):
        reader = _reader()
        reader._poll(FakeI2C(make_block(second=1, quality=4)), 0x20)
        with self.assertRaises(AssertionError):
            with self.assertLogs("pi_app.hardware.rtk_gps", level="WARNING"):
                reader._poll(FakeI2C(make_block(second=2, quality=4)), 0x20)


class TestHealthDict(unittest.TestCase):
    def test_defaults(self):
        reader = _reader()
        h = reader.get_health()
        self.assertEqual(h["reconnect_count"], 0)
        self.assertEqual(h["poll_error_count"], 0)
        self.assertEqual(h["consecutive_i2c_errors"], 0)
        self.assertIsNone(h["last_poll_age_s"])
        self.assertEqual(h["rmc_errors"], 0)
        self.assertIsNone(h["mode_register_value"])

    def test_last_poll_age_s_after_a_poll(self):
        reader = _reader()
        reader._poll(FakeI2C(make_block()), 0x20)
        h = reader.get_health()
        self.assertIsNotNone(h["last_poll_age_s"])
        self.assertGreaterEqual(h["last_poll_age_s"], 0.0)


class TestDetectRetriesTwice(unittest.TestCase):
    def setUp(self):
        self._had_smbus2 = "smbus2" in sys.modules
        self._orig_smbus2 = sys.modules.get("smbus2")

    def tearDown(self):
        if self._had_smbus2:
            sys.modules["smbus2"] = self._orig_smbus2
        else:
            sys.modules.pop("smbus2", None)

    def test_retries_on_failure_then_succeeds(self):
        calls = {"n": 0}

        class _FlakySMBus:
            def __init__(self, bus):
                calls["n"] += 1
                if calls["n"] < 3:
                    raise OSError("no such device")

            def read_i2c_block_data(self, addr, reg, n):
                return [addr]

            def close(self):
                pass

        fake_mod = types.ModuleType("smbus2")
        fake_mod.SMBus = _FlakySMBus
        sys.modules["smbus2"] = fake_mod

        self.assertTrue(RtkGpsReader.detect(bus=1, addr=0x20, max_attempts=3))
        self.assertEqual(calls["n"], 3)

    def test_gives_up_after_max_attempts(self):
        class _AlwaysFails:
            def __init__(self, bus):
                raise OSError("no such device")

        fake_mod = types.ModuleType("smbus2")
        fake_mod.SMBus = _AlwaysFails
        sys.modules["smbus2"] = fake_mod

        self.assertFalse(RtkGpsReader.detect(bus=1, addr=0x20, max_attempts=3))


if __name__ == "__main__":
    unittest.main()
