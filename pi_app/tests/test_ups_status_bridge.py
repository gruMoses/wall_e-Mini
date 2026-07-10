"""Tests for the UPS status-file bridge on the daemon (write) side.

The daemon publishes /tmp/ups_status.json each poll loop so the web /debug
board can show live UPS state without touching I2C. The single hard
requirement here is that this publish is PROVABLY INERT to the daemon's
detection/shutdown decisions: any write failure must be swallowed, never
raised into the poll loop. These tests exercise that isolation directly plus
the happy-path atomic write and a roundtrip through the web reader.

smbus2 / ina219 are hardware-only packages; they are stubbed before importing
the daemon (same pattern as test_ups_ac_debounce.py) purely so the module
imports on a dev machine with no I2C.
"""

import json
import os
import sys
import types
import unittest
from unittest import mock

# ---------------------------------------------------------------------------
# Stub hardware-only modules before importing the daemon.
# ---------------------------------------------------------------------------
if "smbus2" not in sys.modules:
    fake_smbus2 = types.ModuleType("smbus2")

    class _FakeSMBus:
        def __init__(self, *_args, **_kwargs):
            pass

        def read_byte_data(self, *_args, **_kwargs):
            raise RuntimeError("no hardware in unit tests")

        def write_byte_data(self, *_args, **_kwargs):
            raise RuntimeError("no hardware in unit tests")

    fake_smbus2.SMBus = _FakeSMBus
    sys.modules["smbus2"] = fake_smbus2

if "ina219" not in sys.modules:
    fake_ina219 = types.ModuleType("ina219")

    class _FakeINA219:
        def __init__(self, *_args, **_kwargs):
            pass

        def configure(self):
            pass

        def current(self):
            return 0.0

        def voltage(self):
            return 0.0

    fake_ina219.INA219 = _FakeINA219
    sys.modules["ina219"] = fake_ina219

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "scripts"))

import upsPlus_power_daemon as daemon  # noqa: E402

# The reader on the web side; used for a full write→read roundtrip.
from pi_app.web.oak_viewer import read_ups_status  # noqa: E402


class WriteStatusFileHappyPath(unittest.TestCase):
    def test_writes_valid_json_with_all_fields(self):
        with mock.patch("time.time", return_value=1234.5):
            with _tmp() as path:
                ok = daemon.write_status_file(
                    ac_present=True, typec_mv=5000, microusb_mv=10,
                    seconds_without_charge=0, detect_only=False,
                    batt_v=3.9, batt_ma=1200.0, protect_mv=3400, path=path,
                )
                self.assertTrue(ok)
                data = json.loads(open(path).read())
        self.assertEqual(
            set(data.keys()),
            {"ts", "ac_present", "typec_mv", "microusb_mv", "batt_v",
             "batt_ma", "protect_mv", "detect_only", "seconds_without_charge"},
        )
        self.assertEqual(data["ts"], 1234.5)
        self.assertIs(data["ac_present"], True)
        self.assertEqual(data["typec_mv"], 5000)
        self.assertEqual(data["protect_mv"], 3400)
        self.assertEqual(data["seconds_without_charge"], 0)

    def test_no_tmp_file_left_behind(self):
        with _tmp() as path:
            daemon.write_status_file(
                ac_present=False, typec_mv=0, microusb_mv=0,
                seconds_without_charge=7, detect_only=True, path=path,
            )
            leftovers = [n for n in os.listdir(os.path.dirname(path))
                         if n.startswith(os.path.basename(path) + ".tmp")]
        self.assertEqual(leftovers, [])

    def test_roundtrips_through_web_reader(self):
        with mock.patch("time.time", return_value=5000.0):
            with _tmp() as path:
                daemon.write_status_file(
                    ac_present=False, typec_mv=0, microusb_mv=0,
                    seconds_without_charge=12, detect_only=True,
                    batt_v=3.5, batt_ma=-800.0, protect_mv=3400, path=path,
                )
                st = read_ups_status(path, max_age_s=10.0, now=5000.5)
        self.assertTrue(st["available"])
        self.assertFalse(st["stale"])
        self.assertIs(st["ac_present"], False)
        self.assertEqual(st["seconds_without_charge"], 12)
        self.assertTrue(st["detect_only"])
        self.assertEqual(st["batt_ma"], -800.0)


class WriteStatusFileIsolation(unittest.TestCase):
    """The write must NEVER raise — a bad path returns False, no exception."""

    def test_unwritable_path_returns_false_not_raise(self):
        # Directory does not exist → open() raises → must be swallowed.
        bad = "/nonexistent_dir_xyz_zzz/ups_status.json"
        self.assertFalse(os.path.isdir(os.path.dirname(bad)))
        result = daemon.write_status_file(
            ac_present=True, typec_mv=5000, microusb_mv=0,
            seconds_without_charge=0, detect_only=False, path=bad,
        )
        self.assertFalse(result)

    def test_serialization_error_is_swallowed(self):
        # Force json.dumps to blow up mid-write; the helper must still not raise.
        with _tmp() as path:
            with mock.patch("upsPlus_power_daemon.json.dumps",
                            side_effect=ValueError("boom")):
                result = daemon.write_status_file(
                    ac_present=True, typec_mv=1, microusb_mv=1,
                    seconds_without_charge=0, detect_only=False, path=path,
                )
            self.assertFalse(result)
            self.assertFalse(os.path.exists(path))


class WriteStatusFileSymlinkHardening(unittest.TestCase):
    """The temp file must be created O_EXCL|O_NOFOLLOW under an unpredictable
    name — /tmp is world-writable and the daemon runs as root, so an attacker
    pre-planting a symlink at a predictable temp path must NOT get an
    arbitrary-file-write; the publish is refused, swallowed, and retried on a
    later poll."""

    def _write(self, path):
        return daemon.write_status_file(
            ac_present=True, typec_mv=5000, microusb_mv=0,
            seconds_without_charge=0, detect_only=False, path=path,
        )

    def test_preplanted_symlink_at_temp_path_refused_and_target_untouched(self):
        with _tmp() as path:
            victim = os.path.join(os.path.dirname(path), "victim.txt")
            with open(victim, "w") as f:
                f.write("precious")
            # Make the (normally random) temp name predictable, then squat it
            # with a symlink pointing at the victim — the classic /tmp attack.
            with mock.patch("upsPlus_power_daemon.secrets.token_hex",
                            return_value="feedfacecafe"):
                planted = f"{path}.tmp.{os.getpid()}.feedfacecafe"
                os.symlink(victim, planted)
                result = self._write(path)
            # Refused (O_EXCL sees the path exists; symlinks are never
            # followed) and swallowed — no exception reached us.
            self.assertFalse(result)
            # The victim was NOT written through the symlink.
            with open(victim) as f:
                self.assertEqual(f.read(), "precious")
            # The status file was not published this cycle.
            self.assertFalse(os.path.exists(path))
            # And we did not unlink the foreign path we never created.
            self.assertTrue(os.path.islink(planted))

    def test_preplanted_regular_file_at_temp_path_refused(self):
        # O_EXCL also refuses a squatted regular file (EEXIST) — swallowed,
        # foreign file left alone, publish skipped.
        with _tmp() as path:
            with mock.patch("upsPlus_power_daemon.secrets.token_hex",
                            return_value="deadbeef0000"):
                squatted = f"{path}.tmp.{os.getpid()}.deadbeef0000"
                with open(squatted, "w") as f:
                    f.write("squatter")
                result = self._write(path)
            self.assertFalse(result)
            self.assertFalse(os.path.exists(path))
            with open(squatted) as f:
                self.assertEqual(f.read(), "squatter")

    def test_temp_created_with_excl_nofollow_and_random_suffix(self):
        opens = []
        real_open = os.open

        def spy(p, flags, *args, **kwargs):
            opens.append((p, flags))
            return real_open(p, flags, *args, **kwargs)

        with _tmp() as path:
            with mock.patch("upsPlus_power_daemon.os.open", side_effect=spy):
                self.assertTrue(self._write(path))
                self.assertTrue(self._write(path))
        self.assertEqual(len(opens), 2)
        for p, flags in opens:
            self.assertTrue(flags & os.O_CREAT)
            self.assertTrue(flags & os.O_EXCL)
            self.assertTrue(flags & os.O_NOFOLLOW)
            # pid + 12 hex chars of secrets.token_hex(6) entropy.
            self.assertRegex(os.path.basename(p),
                             r"^ups_status\.json\.tmp\.\d+\.[0-9a-f]{12}$")
        # Unpredictable: two consecutive writes must not reuse a temp name.
        self.assertNotEqual(opens[0][0], opens[1][0])


class PublishStatusIsolation(unittest.TestCase):
    """_publish_status wraps the supplemental I2C read + the write; neither
    can propagate into the poll loop, and a failed context read degrades
    gracefully to None rather than aborting the publish."""

    def test_supplemental_read_failure_still_publishes(self):
        # read_ups_snapshot raises (stub SMBus has no hardware); the publish
        # must still write the authoritative typec/microusb values it was given.
        with mock.patch("time.time", return_value=9000.0):
            with _tmp() as path:
                with mock.patch.object(daemon, "UPS_STATUS_FILE", path):
                    daemon._publish_status(
                        daemon.smbus2.SMBus(1), 0x17, None,
                        ac_present=True, typec_mv=5050, microusb_mv=0,
                        seconds_without_charge=0, detect_only=False,
                    )
                data = json.loads(open(path).read())
        self.assertEqual(data["typec_mv"], 5050)
        self.assertIsNone(data["batt_v"])       # supplemental read failed → None
        self.assertIsNone(data["protect_mv"])

    def test_write_helper_raising_is_swallowed(self):
        # Even a write_status_file that somehow raises must not escape.
        with mock.patch.object(daemon, "write_status_file",
                               side_effect=RuntimeError("explode")):
            with mock.patch.object(daemon, "read_ups_snapshot",
                                   side_effect=RuntimeError("explode")):
                # Should return None without raising.
                self.assertIsNone(daemon._publish_status(
                    object(), 0x17, None,
                    ac_present=False, typec_mv=0, microusb_mv=0,
                    seconds_without_charge=3, detect_only=True,
                ))


class DecisionLogicUnchanged(unittest.TestCase):
    """Guard: the status-bridge additions did not touch the AC-detection
    thresholds or the debounce constants the shutdown decision depends on."""

    def test_key_decision_constants_intact(self):
        self.assertEqual(daemon.AC_PRESENT_VOLTAGE_THRESHOLD_MV, 4000)
        self.assertEqual(daemon.AC_LOSS_DEBOUNCE_S, 10.0)
        self.assertEqual(daemon.NO_CHARGE_GRACE_SECONDS, 30)
        self.assertEqual(daemon.BATTERY_PROTECTION_MV, 3400)

    def test_ac_present_decision_still_voltage_only(self):
        # Sanity: the authoritative instant decision is unchanged by our edits.
        self.assertTrue(daemon.is_ac_present_instant(4800, 0))
        self.assertFalse(daemon.is_ac_present_instant(0, 3999))


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

class _tmp:
    """Context manager yielding a fresh temp file path, cleaned up after."""

    def __enter__(self):
        import tempfile
        self._dir = tempfile.mkdtemp()
        return os.path.join(self._dir, "ups_status.json")

    def __exit__(self, *exc):
        import shutil
        shutil.rmtree(self._dir, ignore_errors=True)
        return False


if __name__ == "__main__":
    unittest.main()
