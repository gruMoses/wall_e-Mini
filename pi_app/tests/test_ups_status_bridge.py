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
             "batt_ma", "protect_mv", "ts_batt", "batt_stale", "detect_only",
             "seconds_without_charge"},
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


class PolitenessConstants(unittest.TestCase):
    """The I2C-politeness cadence/holdoff knobs are the named values the
    daemon and this suite agree on."""

    def test_politeness_constants(self):
        self.assertEqual(daemon.AC_RECOVERY_QUIET_S, 5.0)
        self.assertEqual(daemon.SUPPLEMENTAL_READ_PERIOD_S, 10.0)


class DebounceWindowFlag(unittest.TestCase):
    """AcPresenceTracker.in_debounce_window drives the daemon's sag-suppression;
    it must be True only while a possible loss is being debounced (state still
    PRESENT), and False both when cleanly present and once committed MISSING."""

    def test_in_debounce_window_transitions(self):
        t = daemon.AcPresenceTracker(debounce_s=10.0)
        t.update(5000, 0, now=0.0)                 # cleanly present
        self.assertFalse(t.in_debounce_window)
        t.update(0, 0, now=1.0)                    # first low reading — window opens
        self.assertTrue(t.in_debounce_window)
        self.assertTrue(t.state)                   # still PRESENT, debouncing
        t.update(0, 0, now=5.0)                    # still low, inside window
        self.assertTrue(t.in_debounce_window)
        t.update(0, 0, now=11.0)                   # debounce elapsed → MISSING
        self.assertFalse(t.in_debounce_window)     # window closed; state-missing takes over
        self.assertFalse(t.state)
        t.update(5000, 0, now=12.0)                # charger back → PRESENT
        self.assertFalse(t.in_debounce_window)
        self.assertTrue(t.state)


class SupplementalGate(unittest.TestCase):
    """The pure I2C-politeness decision: WHEN a poll may issue the extra
    (non-decision) snapshot + INA219 reads. No hardware, no file I/O."""

    def test_steady_state_cadence_is_10s_not_1s(self):
        # 25 one-second steady AC-present polls; supplemental should fire on the
        # first poll then only every SUPPLEMENTAL_READ_PERIOD_S (10s), not 1Hz.
        last_unclean = None
        last_supp = None
        fired = []
        for i in range(25):
            now = 100.0 + i
            do, last_unclean = daemon._supplemental_gate(
                instant_present=True, in_debounce=False, state_missing=False,
                now=now, last_unclean_at=last_unclean, last_supplemental_at=last_supp,
            )
            if do:
                fired.append(now)
                last_supp = now
        self.assertEqual(fired, [100.0, 110.0, 120.0])

    def test_transfer_window_forces_quiet_even_when_due(self):
        # Cadence long overdue (last read at t=0, now t=1000) but any of the
        # three sag conditions this poll must still force quiet.
        do, lu = daemon._supplemental_gate(
            instant_present=False, in_debounce=False, state_missing=False,
            now=1000.0, last_unclean_at=None, last_supplemental_at=0.0,
        )
        self.assertFalse(do)               # instant-not-present wins over "due"
        self.assertEqual(lu, 1000.0)       # and this poll is recorded as unclean
        do2, _ = daemon._supplemental_gate(
            instant_present=True, in_debounce=True, state_missing=False,
            now=1000.0, last_unclean_at=None, last_supplemental_at=0.0,
        )
        self.assertFalse(do2)              # debounce window open
        do3, _ = daemon._supplemental_gate(
            instant_present=True, in_debounce=False, state_missing=True,
            now=1000.0, last_unclean_at=None, last_supplemental_at=0.0,
        )
        self.assertFalse(do3)              # debounced state already MISSING

    def test_recovery_holdoff_after_ac_returns(self):
        last_unclean = None
        last_supp = None
        res = {}
        # t=0..3: AC missing/sagging — quiet, each poll marks itself unclean.
        for now in (0.0, 1.0, 2.0, 3.0):
            do, last_unclean = daemon._supplemental_gate(
                instant_present=False, in_debounce=False, state_missing=True,
                now=now, last_unclean_at=last_unclean, last_supplemental_at=last_supp)
            res[now] = do
            if do:
                last_supp = now
        # t=4..8: AC present again; must stay quiet through the 5s holdoff
        # (measured from the last unclean poll t=3) and only resume once elapsed.
        for now in (4.0, 5.0, 6.0, 7.0, 8.0):
            do, last_unclean = daemon._supplemental_gate(
                instant_present=True, in_debounce=False, state_missing=False,
                now=now, last_unclean_at=last_unclean, last_supplemental_at=last_supp)
            res[now] = do
            if do:
                last_supp = now
        self.assertFalse(any(res[t] for t in (0.0, 1.0, 2.0, 3.0)))  # quiet during outage
        self.assertFalse(res[4.0])   # 1s past last-unclean, inside holdoff
        self.assertFalse(res[7.0])   # 4s past, still inside holdoff (<5s)
        self.assertTrue(res[8.0])    # 5s elapsed → supplemental resumes


class PublishStatusPoliteness(unittest.TestCase):
    """_publish_status honors do_supplemental: when False it issues NO
    supplemental I2C (no 16-reg snapshot, no INA219 read) yet still publishes
    fresh AC state + charger voltages with the battery fields marked stale;
    when True it refreshes the caller's batt_cache and clears the stale flag."""

    def test_no_supplemental_read_when_quiet_but_still_publishes(self):
        ina = mock.Mock()
        cache = {"batt_v": 3.71, "batt_ma": -1400.0, "protect_mv": 3400,
                 "ts_batt": 900.0}
        with mock.patch("time.time", return_value=1000.0):
            with _tmp() as path:
                with mock.patch.object(daemon, "UPS_STATUS_FILE", path):
                    with mock.patch.object(daemon, "read_ups_snapshot") as snap_spy:
                        daemon._publish_status(
                            daemon.smbus2.SMBus(1), 0x17, ina,
                            ac_present=False, typec_mv=0, microusb_mv=0,
                            seconds_without_charge=5, detect_only=False,
                            do_supplemental=False, batt_cache=cache,
                        )
                        snap_spy.assert_not_called()   # NO 16-register snapshot
                data = json.loads(open(path).read())
        ina.current.assert_not_called()                # NO INA219 read
        ina.voltage.assert_not_called()
        self.assertIs(data["ac_present"], False)       # AC state still published
        self.assertTrue(data["batt_stale"])            # battery marked stale
        self.assertEqual(data["batt_v"], 3.71)         # last-known reused
        self.assertEqual(data["ts_batt"], 900.0)       # from cache, not "now"
        self.assertEqual(data["ts"], 1000.0)           # file ts is fresh

    def test_supplemental_read_refreshes_cache_and_clears_stale(self):
        cache = {"batt_v": 1.0, "batt_ma": 1.0, "protect_mv": 1, "ts_batt": 1.0}
        fake_snap = {"battery_v": 3.95, "battery_i_ma": 1234.0, "protect_mv": 3400}
        with mock.patch("time.time", return_value=3000.0):
            with _tmp() as path:
                with mock.patch.object(daemon, "UPS_STATUS_FILE", path):
                    with mock.patch.object(daemon, "read_ups_snapshot",
                                           return_value=fake_snap):
                        daemon._publish_status(
                            daemon.smbus2.SMBus(1), 0x17, None,
                            ac_present=True, typec_mv=5000, microusb_mv=0,
                            seconds_without_charge=0, detect_only=False,
                            do_supplemental=True, batt_cache=cache,
                        )
                data = json.loads(open(path).read())
        self.assertFalse(data["batt_stale"])
        self.assertEqual(data["batt_v"], 3.95)
        self.assertEqual(data["ts_batt"], 3000.0)
        # Cache mutated in place so the next quiet poll republishes these.
        self.assertEqual(cache["batt_v"], 3.95)
        self.assertEqual(cache["ts_batt"], 3000.0)

    def test_status_always_carries_ts_and_ts_batt(self):
        # Even on a quiet poll the file carries both timestamps: ts fresh,
        # ts_batt from the cache (honest staleness).
        with mock.patch("time.time", return_value=2000.0):
            with _tmp() as path:
                with mock.patch.object(daemon, "UPS_STATUS_FILE", path):
                    daemon._publish_status(
                        daemon.smbus2.SMBus(1), 0x17, None,
                        ac_present=True, typec_mv=5000, microusb_mv=0,
                        seconds_without_charge=0, detect_only=False,
                        do_supplemental=False, batt_cache={"ts_batt": 111.0},
                    )
                q = json.loads(open(path).read())
        self.assertIn("ts", q)
        self.assertIn("ts_batt", q)
        self.assertEqual(q["ts"], 2000.0)
        self.assertEqual(q["ts_batt"], 111.0)
        self.assertTrue(q["batt_stale"])


class TransitionLoggingSnapshotFree(unittest.TestCase):
    """AC state-change logging must issue ZERO supplemental I2C. The old code
    read a fresh snapshot on every transition — at the worst possible moments
    (the 'missing' edge mid-outage, the 'present again' edge inside the
    recovery holdoff). Transitions now log this poll's detection-read charger
    voltages + last-cached battery values."""

    def test_formatter_is_pure_and_tags_battery_age(self):
        with mock.patch.object(daemon, "read_ups_snapshot") as snap_spy:
            msg = daemon._format_ac_transition_log(
                ac_present=False, typec_mv=123, microusb_mv=0,
                batt_cache={"batt_v": 3.71, "batt_ma": -1400.0,
                            "protect_mv": 3400, "ts_batt": 100.0},
                now_wall=112.0,
            )
        snap_spy.assert_not_called()
        self.assertIn("missing", msg)
        self.assertIn("typec=123mV", msg)          # detection read, this poll
        self.assertIn("batt=3.71V", msg)           # cached, not freshly read
        self.assertIn("-1400.0", msg)
        self.assertIn("batt 12s old", msg)         # honest staleness age

    def test_formatter_handles_empty_cache(self):
        msg = daemon._format_ac_transition_log(
            ac_present=True, typec_mv=5000, microusb_mv=0, batt_cache={})
        self.assertIn("present", msg)
        self.assertIn("typec=5000mV", msg)
        self.assertIn("no battery read yet", msg)

    def test_ac_missing_transition_triggers_no_snapshot_read(self):
        """Drive main() through boot grace, steady AC, a sag, and the debounced
        MISSING transition on a fake clock. Every read_ups_snapshot call
        (startup + steady-state cadence) must predate the sag; the transition
        log line must still carry detection-read voltages + cached battery."""
        clk = {"t": 0.0}
        snapshot_calls = []
        fake_snap = {
            "typec_mv": 5000, "microusb_mv": 0, "protect_mv": 3400,
            "shutdown_countdown_s": 0, "auto_power_on": 1,
            "sample_period_min": 2, "battery_v": 3.9, "battery_i_ma": 250.0,
        }

        def fake_read_snapshot(*_a, **_k):
            snapshot_calls.append(clk["t"])
            return dict(fake_snap)

        SAG_START = 25.0   # charger voltage drops here; debounce (10s) → t=35

        def fake_read_charger(_bus, _addr):
            return (5000, 0) if clk["t"] < SAG_START else (0, 0)

        class _StopLoop(Exception):
            pass

        def fake_sleep(s):
            clk["t"] += s
            if clk["t"] >= 40.0:   # well past the transition, before grace expiry
                raise _StopLoop

        with _tmp() as path:
            with mock.patch.object(daemon, "UPS_STATUS_FILE", path), \
                 mock.patch.object(daemon, "detect_addr", return_value=0x17), \
                 mock.patch.object(daemon, "read_ups_snapshot",
                                   side_effect=fake_read_snapshot), \
                 mock.patch.object(daemon, "read_charger_voltages",
                                   side_effect=fake_read_charger), \
                 mock.patch("time.monotonic", side_effect=lambda: clk["t"]), \
                 mock.patch("time.sleep", side_effect=fake_sleep):
                with self.assertLogs(level="INFO") as captured:
                    with self.assertRaises(_StopLoop):
                        daemon.main(detect_only=True)

        missing_lines = [m for m in captured.output
                         if "UPS AC state changed -> missing" in m]
        self.assertEqual(len(missing_lines), 1)
        line = missing_lines[0]
        # Charger voltages from THIS poll's detection read (sagged to 0)…
        self.assertIn("typec=0mV", line)
        self.assertIn("microusb=0mV", line)
        # …battery from the cache filled during steady state, marked cached.
        self.assertIn("batt=3.9V", line)
        self.assertIn("250.0", line)
        self.assertIn("cached", line)
        # ZERO supplemental I2C during the sag/outage: every snapshot call
        # (startup config log + steady-state cadence) happened before the sag.
        self.assertTrue(snapshot_calls)
        self.assertTrue(all(t < SAG_START for t in snapshot_calls),
                        f"snapshot call(s) during sag window: {snapshot_calls}")


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
