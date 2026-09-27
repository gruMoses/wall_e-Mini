"""Main-loop simulation of the UPS watcher with a frozen charger register.

Drives the real ``upsPlus_power_daemon.main()`` loop with a fake clock and
fake hardware (typec_mv frozen at 5057 mV, as on 2026-09-26). Guards the
interplay Grok found on 2026-09-27: the USB shed lowers the drain below the
loss threshold, which must not cancel the shutdown and cycle forever.
"""

import logging
import os
import sys
import types
import unittest
from unittest import mock

# ---------------------------------------------------------------------------
# Stub out smbus2 / ina219 before importing the daemon so it imports cleanly
# on a machine with no I2C hardware or these hardware-only packages.
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


class _Stop(Exception):
    pass


def _simulate(scenario, horizon_s):
    """Returns [(sim_time, event)] for 'outage', 'back_after_shed', 'taper'."""
    clock = {"now": 0.0}
    state = {"shed": False, "back_at": None}
    events = []

    def current_ma(now):
        if scenario == "taper":
            return -3372.0 if 40 <= (now % 90) < 49 else 1800.0
        power_on = now < 30 or (state["back_at"] is not None and now >= state["back_at"])
        if power_on:
            return 1800.0
        return -1050.0 if state["shed"] else -4500.0

    class FakeTime:
        @staticmethod
        def monotonic():
            return clock["now"]

        @staticmethod
        def time():
            return 1_000_000.0 + clock["now"]

        @staticmethod
        def sleep(seconds):
            clock["now"] += seconds
            if clock["now"] > horizon_s:
                raise _Stop()

    def snapshot(_bus, _addr, _ina):
        return {"typec_mv": 5057, "microusb_mv": 0, "protect_mv": 3400,
                "shutdown_countdown_s": 0, "auto_power_on": 1,
                "sample_period_min": 2, "battery_v": 3.9,
                "battery_i_ma": current_ma(clock["now"])}

    def shed():
        state["shed"] = True
        events.append((clock["now"], "shed"))
        if scenario == "back_after_shed" and state["back_at"] is None:
            state["back_at"] = clock["now"] + 1.0

    def restore():
        state["shed"] = False
        events.append((clock["now"], "restore"))
        return True

    def shutdown(*_a, **kwargs):
        events.append((clock["now"], "shutdown", kwargs.get("usb_already_shed")))
        raise _Stop()

    gauge = types.SimpleNamespace(
        configure=lambda: None,
        current=lambda: current_ma(clock["now"]),
        voltage=lambda: 3.9,
    )
    patches = [
        mock.patch.object(daemon, "time", FakeTime),
        mock.patch.object(daemon, "smbus2", types.SimpleNamespace(SMBus=lambda *a, **k: object())),
        mock.patch.object(daemon, "detect_addr", lambda _bus: 0x17),
        mock.patch.object(daemon, "INA219", lambda *a, **k: gauge),
        mock.patch.object(daemon, "read_charger_voltages", lambda _b, _a: (5057, 0)),
        mock.patch.object(daemon, "write_reg_verified", lambda *a, **k: True),
        mock.patch.object(daemon, "read_reg_with_retry", lambda *a, **k: 0),
        mock.patch.object(daemon, "warn_marginal_boot_supply", lambda _s: False),
        mock.patch.object(daemon, "read_ups_snapshot", snapshot),
        mock.patch.object(daemon, "write_status_file", lambda **k: None),
        mock.patch.object(daemon, "shed_usb_load", shed),
        mock.patch.object(daemon, "restore_usb_load", restore),
        mock.patch.object(daemon, "run_shutdown_sequence", shutdown),
    ]
    for p in patches:
        p.start()
    logging.disable(logging.CRITICAL)
    try:
        daemon.main(detect_only=False)
    except _Stop:
        pass
    finally:
        logging.disable(logging.NOTSET)
        for p in reversed(patches):
            p.stop()
    return events


class TestFrozenRegisterLoop(unittest.TestCase):
    def test_outage_sheds_then_shuts_down_cleanly(self):
        events = _simulate("outage", 400)
        kinds = [e[1] for e in events]
        self.assertEqual(kinds, ["shed", "shutdown"])
        shed_t, shutdown = events[0][0], events[1]
        self.assertLessEqual(shed_t - 30, 70)          # loss declared within ~60-70 s
        self.assertLessEqual(shutdown[0] - shed_t, 5)   # grace, then halt
        self.assertTrue(shutdown[2])                    # USB already shed

    def test_power_back_after_the_shed_cancels_without_cycling(self):
        events = _simulate("back_after_shed", 600)
        kinds = [e[1] for e in events]
        self.assertEqual(kinds, ["shed", "restore"])
        self.assertGreaterEqual(events[1][0] - events[0][0], 15)

    def test_charger_taper_never_sheds_or_shuts_down(self):
        self.assertEqual(_simulate("taper", 3600), [])


if __name__ == "__main__":
    unittest.main()
