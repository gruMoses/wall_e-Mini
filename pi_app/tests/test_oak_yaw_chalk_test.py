"""Focused tests for the disarmed OAK yaw chalk harness.

Covers the non-stream baseline desync bug (poll-less input wait), synchronized
mark capture, expected-zero / long bias scale safety, NMNI CLI override, and
no motor/config writes.
"""

from __future__ import annotations

import inspect
import io
import math
import os
import unittest
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional
from unittest.mock import MagicMock, patch

from pi_app.cli import oak_yaw_chalk_test as chalk
from pi_app.cli.oak_yaw_chalk_test import (
    MarkSnapshot,
    RateWindow,
    TriadIntegrator,
    _pass_band,
    capture_mark_snapshot,
    format_bias_dps,
    parse_chalk_args,
    poll_imu_once,
    production_matches_neg_producer_y,
    recommended_scale,
    resolve_nmni_enabled,
    wait_for_enter_with_poll,
)


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


@dataclass
class FakeImuState:
    gx_rads: float = 0.0
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    ax_mss: float = 0.0
    ay_mss: float = -9.80665
    az_mss: float = 0.0
    timestamp: float = 0.0
    device_timestamp_s: float = 0.0
    cum_yaw_x_rad: float = 0.0
    cum_yaw_y_rad: float = 0.0
    cum_yaw_z_rad: float = 0.0
    cum_yaw_grav_rad: float = 0.0
    yaw_generation: int = 1
    packets_integrated: int = 0


class FakeOak:
    """Minimal OakDepthReader stub with advancing producer cum."""

    def __init__(self) -> None:
        self.state = FakeImuState()
        self.age_s = 0.01
        self.nmni_enabled: Optional[bool] = None
        self.nmni_threshold: Optional[float] = None
        self.gyro_bias_dps = (0.0, 0.0, 0.0)
        self._metrics: Dict = {
            "packets_received": 0,
            "packets_parsed": 0,
            "packets_integrated": 0,
            "packets_coalesced": 0,
            "packets_duplicate": 0,
            "packets_regressed": 0,
            "packets_restart": 0,
            "packets_gap_freeze": 0,
            "packets_backlog_dropped": 0,
            "cadence_avg_s": 0.01,
            "cadence_max_s": 0.01,
            "producer_cum_yaw_x_deg": 0.0,
            "producer_cum_yaw_y_deg": 0.0,
            "producer_cum_yaw_z_deg": 0.0,
            "producer_cum_yaw_grav_deg": 0.0,
            "producer_generation": 1,
            "last_batch_packets": 1,
            "host_queue_max_size": 512,
            "host_queue_blocking": False,
            "max_packets_per_drain": 512,
            "drain_batch_high_water_msgs": 1,
            "drain_batch_large_events": 0,
            "drain_batch_full_size_events": 0,
            "queue_msgs_dropped": 0,
            "queue_msgs_overwrite_observable": False,
        }

    def get_imu_data(self):
        return self.state, self.age_s

    def get_health(self):
        return {
            "connected": True,
            "reconnect_count": 0,
            "last_disconnect_ts": 0.0,
            "pipeline_running": True,
        }

    def get_imu_metrics(self):
        m = dict(self._metrics)
        m["producer_cum_yaw_x_deg"] = math.degrees(self.state.cum_yaw_x_rad)
        m["producer_cum_yaw_y_deg"] = math.degrees(self.state.cum_yaw_y_rad)
        m["producer_cum_yaw_z_deg"] = math.degrees(self.state.cum_yaw_z_rad)
        m["packets_integrated"] = self.state.packets_integrated
        m["producer_generation"] = self.state.yaw_generation
        return m

    def set_imu_nmni(self, enabled, threshold_dps=0.3):
        self.nmni_enabled = bool(enabled)
        self.nmni_threshold = float(threshold_dps)

    def set_imu_gyro_bias_dps(self, bx, by, bz):
        self.gyro_bias_dps = (float(bx), float(by), float(bz))

    def advance_producer_y(self, delta_deg: float, packets: int = 10) -> None:
        """Simulate producer integrating more free-yaw about Y."""
        self.state.cum_yaw_y_rad += math.radians(delta_deg)
        self.state.packets_integrated += int(packets)
        self.state.device_timestamp_s += 0.01 * packets
        self.state.timestamp = self.state.device_timestamp_s
        self.state.gy_rads = math.radians(delta_deg / max(0.01, 0.01 * packets))
        self._metrics["packets_received"] = self.state.packets_integrated
        self._metrics["packets_parsed"] = self.state.packets_integrated
        self._metrics["packets_integrated"] = self.state.packets_integrated


class FakeImu:
    """Stand-in for OakImuReader that tracks free yaw like production."""

    def __init__(self, oak: FakeOak, scale: float = 1.0) -> None:
        self._oak = oak
        self.yaw_rad = 0.0
        self._scale = scale
        self._last_cum_y: Optional[float] = None
        self._reads = 0
        self.gyro_bias_dps = (0.01, -0.02, 0.005)

    def read(self) -> Dict:
        self._reads += 1
        st, age = self._oak.get_imu_data()
        cum_y = float(st.cum_yaw_y_rad)
        if self._last_cum_y is None:
            self._last_cum_y = cum_y
        else:
            self.yaw_rad += (cum_y - self._last_cum_y) * self._scale
            self._last_cum_y = cum_y
        heading = (-math.degrees(self.yaw_rad) + 360.0) % 360.0
        return {
            "heading_deg": heading,
            "yaw_deg": -math.degrees(self.yaw_rad),
            "gx_dps": math.degrees(st.gx_rads),
            "gy_dps": math.degrees(st.gy_rads),
            "gz_dps": math.degrees(st.gz_rads),
            "gx_body_dps": math.degrees(st.gx_rads),
            "gy_body_dps": math.degrees(st.gy_rads),
            "gz_body_dps": math.degrees(st.gz_rads),
            "sample_age_s": age,
            "device_timestamp_s": st.device_timestamp_s,
            "producer_cum_yaw_x_deg": math.degrees(st.cum_yaw_x_rad),
            "producer_cum_yaw_y_deg": math.degrees(st.cum_yaw_y_rad),
            "producer_cum_yaw_z_deg": math.degrees(st.cum_yaw_z_rad),
            "count_producer_packets": float(st.packets_integrated),
            "yaw_generation": float(st.yaw_generation),
            "integrate_status": "fresh",
        }

    def get_health(self) -> Dict:
        st, age = self._oak.get_imu_data()
        m = self._oak.get_imu_metrics()
        return {
            "yaw_rate_source_selected": "gyro_y",
            "integrate_status": "fresh",
            "sample_age_s": age,
            "integration_path": "producer",
            "gx_body_dps": math.degrees(st.gx_rads),
            "gy_body_dps": math.degrees(st.gy_rads),
            "gz_body_dps": math.degrees(st.gz_rads),
            "count_duplicate": 0,
            "count_stale": 0,
            "count_regressed": 0,
            "count_restart": 0,
            "count_integrated": self._reads,
            "count_producer_packets": st.packets_integrated,
            "count_generation_change": 0,
            "count_cum_reset": 0,
            "count_gap_freeze": 0,
            "yaw_generation": st.yaw_generation,
            "producer_cum_yaw_x_deg": math.degrees(st.cum_yaw_x_rad),
            "producer_cum_yaw_y_deg": math.degrees(st.cum_yaw_y_rad),
            "producer_cum_yaw_z_deg": math.degrees(st.cum_yaw_z_rad),
            "producer_packets_integrated": st.packets_integrated,
            "producer_packets_received": m["packets_received"],
            "producer_packets_parsed": m["packets_parsed"],
            "producer_packets_gap_freeze": 0,
            "producer_packets_backlog_dropped": 0,
            "producer_packets_duplicate": 0,
            "producer_packets_restart": 0,
            "producer_packets_regressed": 0,
            "producer_packets_coalesced": 0,
            "last_host_sample_ts": st.timestamp,
            **{k: v for k, v in m.items() if k.startswith("host_") or k.startswith("drain_") or k.startswith("queue_") or k.startswith("max_")},
        }

    def calibrate_gyro(self, duration_s: float = 3.0):
        return self.gyro_bias_dps


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class TestPassBandAndScale(unittest.TestCase):
    def test_recommended_scale_no_div_zero_expected_zero(self):
        self.assertTrue(math.isnan(recommended_scale(0.0, 0.0)))
        self.assertTrue(math.isnan(recommended_scale(1.5, 0.0)))
        self.assertTrue(math.isnan(recommended_scale(0.0, 90.0)))

    def test_recommended_scale_normal(self):
        self.assertAlmostEqual(recommended_scale(45.0, 90.0), 2.0)
        self.assertAlmostEqual(
            recommended_scale(45.0, 90.0, current_scale=1.0, production=True),
            2.0,
        )

    def test_pass_band_expected_zero(self):
        self.assertTrue(_pass_band(0.5, 0.0, abs_tol_deg=8.0, rel_tol=0.1))
        self.assertFalse(_pass_band(12.0, 0.0, abs_tol_deg=8.0, rel_tol=0.1))

    def test_format_bias_six_decimals(self):
        s = format_bias_dps((0.123456789, -0.000001, 1.0))
        self.assertIn("x=0.123457", s)
        self.assertIn("y=-0.000001", s)
        self.assertIn("z=1.000000", s)


class TestNmniCli(unittest.TestCase):
    def test_resolve_nmni_from_config(self):
        en, src = resolve_nmni_enabled(None, True)
        self.assertTrue(en)
        self.assertEqual(src, "config")
        en, src = resolve_nmni_enabled(None, False)
        self.assertFalse(en)
        self.assertEqual(src, "config")

    def test_resolve_nmni_cli_override(self):
        en, src = resolve_nmni_enabled(False, True)
        self.assertFalse(en)
        self.assertEqual(src, "cli")
        en, src = resolve_nmni_enabled(True, False)
        self.assertTrue(en)
        self.assertEqual(src, "cli")

    def test_parse_nmni_flags(self):
        a = parse_chalk_args(["--no-nmni", "--expected", "0", "--stream", "--bias-s", "10"])
        self.assertIs(a.nmni, False)
        self.assertEqual(a.expected, 0.0)
        self.assertTrue(a.stream)
        self.assertEqual(a.bias_s, 10.0)

        b = parse_chalk_args(["--nmni", "--expected", "90"])
        self.assertIs(b.nmni, True)

        c = parse_chalk_args(["--expected", "90"])
        self.assertIsNone(c.nmni)


class TestPromptPolling(unittest.TestCase):
    def test_wait_polls_until_enter_non_stream(self):
        polls = {"n": 0}
        stdin = io.StringIO("\n")

        def poll():
            polls["n"] += 1
            return {"heading_deg": float(polls["n"])}

        # First two selects: empty; third: stdin ready
        calls = {"i": 0}

        def fake_select(r, w, x, timeout=None):
            calls["i"] += 1
            if calls["i"] < 3:
                return ([], [], [])
            return ([stdin], [], [])

        sleeps: List[float] = []
        wait_for_enter_with_poll(
            poll,
            0.01,
            stream=False,
            stdin=stdin,
            select_fn=fake_select,
            sleep_fn=lambda t: sleeps.append(t),
            prompt="MARK?",
            print_fn=lambda *a, **k: None,
        )
        self.assertGreaterEqual(polls["n"], 3)
        # non-stream uses select timeout, not sleep
        self.assertEqual(sleeps, [])

    def test_wait_polls_stream_mode(self):
        polls = {"n": 0}
        stdin = io.StringIO("\n")
        lines: List[str] = []

        def poll():
            polls["n"] += 1
            return {"heading_deg": 1.0}

        calls = {"i": 0}

        def fake_select(r, w, x, timeout=None):
            calls["i"] += 1
            if calls["i"] < 2:
                return ([], [], [])
            return ([stdin], [], [])

        wait_for_enter_with_poll(
            poll,
            0.001,
            stream=True,
            stream_line=lambda d: f"hdg={d['heading_deg']}",
            stdin=stdin,
            select_fn=fake_select,
            sleep_fn=lambda t: None,
            prompt="stream",
            print_fn=lambda *a, **k: lines.append(" ".join(str(x) for x in a)),
        )
        self.assertGreaterEqual(polls["n"], 2)


class TestBaselineDesyncRegression(unittest.TestCase):
    """Reproduce: non-stream wait without poll_once → stale triad vs fresh start_prod.

    Hardware symptom: start production heading 10.43, producer Y −17.94,
    production delta mismatched to triad. Root cause: input() without polling,
    then imu.read() refreshes production while triad is frozen.
    """

    def test_bug_reproduction_stale_triad_vs_refreshed_production(self):
        oak = FakeOak()
        imu = FakeImu(oak, scale=1.0)
        triad = TriadIntegrator()

        def poll():
            return poll_imu_once(imu, triad, reader=None)

        # Warm-up: seed baselines while still.
        for _ in range(3):
            poll()

        # Simulate BUG: producer advances during wait WITHOUT poll_once
        # (old non-stream `input()` path).
        oak.advance_producer_y(-17.94, packets=50)

        # Old start_prod path: refresh production only.
        data = imu.read()  # advances production from producer cum
        start_prod_heading = float(data["heading_deg"])
        start_prod_yaw = -math.degrees(imu.yaw_rad)
        start_y_stale = triad.y_deg  # NEVER updated during wait

        # Turn +7.55° on producer Y (production free-yaw sign is −Y).
        oak.advance_producer_y(7.55, packets=20)
        # End: poll would update triad; production needs read.
        poll()
        end_prod_yaw = -math.degrees(imu.yaw_rad)
        end_y = triad.y_deg

        d_prod = end_prod_yaw - start_prod_yaw
        d_y = end_y - start_y_stale

        # Bug signature: production start absorbed pre-mark motion (−17.94 →
        # heading change), triad start stayed at ~0, so triad delta includes
        # pre-mark −17.94 while production delta is only the turn (~ −7.55? wait
        # production free = −cum_y at scale 1).
        # imu.yaw_rad tracks +cum_y; prod_free = −degrees(yaw_rad).
        # After first read after −17.94 advance: yaw_rad = −17.94°, free = +17.94
        # Actually: yaw_rad += delta_cum_y; start free = -deg(yaw) = -(-17.94)=+17.94
        # After +7.55: yaw = -17.94+7.55 = -10.39; free = +10.39; d_prod = -7.55
        # triad start stale = 0; end after poll = -17.94+7.55 = -10.39; d_y = -10.39
        self.assertAlmostEqual(d_prod, -7.55, places=2)
        self.assertNotAlmostEqual(d_y, d_prod, places=1)
        # Exact mismatch: triad includes pre-mark producer motion.
        self.assertAlmostEqual(d_y, -17.94 + 7.55, places=2)
        self.assertFalse(production_matches_neg_producer_y(d_prod, d_y))

    def test_fix_poll_while_waiting_keeps_baselines_synced(self):
        oak = FakeOak()
        imu = FakeImu(oak, scale=1.0)
        triad = TriadIntegrator()

        def poll():
            return poll_imu_once(imu, triad, reader=None)

        for _ in range(3):
            poll()

        # Same pre-mark producer advance, but we poll (fixed wait path).
        oak.advance_producer_y(-17.94, packets=50)
        poll()  # continuous poll during wait

        start = capture_mark_snapshot(imu, triad, poll, rebaseline_triad=True)
        # After rebaseline, triad start is 0; producer cum is absolute.
        self.assertAlmostEqual(start.triad_y_deg, 0.0, places=6)

        oak.advance_producer_y(7.55, packets=20)
        end = capture_mark_snapshot(imu, triad, poll, rebaseline_triad=False)

        d_prod = end.prod_free_yaw_deg - start.prod_free_yaw_deg
        d_y = end.triad_y_deg - start.triad_y_deg
        d_producer_y = (
            end.producer_cum_y_deg - start.producer_cum_y_deg
            if end.producer_cum_y_deg is not None and start.producer_cum_y_deg is not None
            else None
        )

        self.assertIsNotNone(d_producer_y)
        self.assertAlmostEqual(d_y, d_producer_y, places=5)
        self.assertAlmostEqual(d_prod, -d_producer_y, places=5)
        self.assertTrue(production_matches_neg_producer_y(d_prod, d_producer_y))
        # Turn-only: magnitude ~7.55, not contaminated by −17.94 pre-mark.
        self.assertAlmostEqual(d_y, 7.55, places=2)
        self.assertAlmostEqual(d_prod, -7.55, places=2)

    def test_hardware_mismatch_numbers_pass_after_fix(self):
        """Full scripted scenario matching field numbers after fix."""
        oak = FakeOak()
        imu = FakeImu(oak, scale=1.0)
        triad = TriadIntegrator()
        polls = []

        def poll():
            d = poll_imu_once(imu, triad, reader=None)
            polls.append(d)
            return d

        for _ in range(5):
            poll()

        # Idle drift / bump during MARK START wait (producer Y → −17.94)
        oak.advance_producer_y(-17.94, packets=100)
        # Continuous poll while "waiting"
        for _ in range(3):
            poll()

        start = capture_mark_snapshot(imu, triad, poll, rebaseline_triad=True)
        # Production heading is whatever free-yaw maps to mod 360; free is synced.
        self.assertAlmostEqual(start.triad_x_deg, 0.0, places=6)
        self.assertAlmostEqual(start.triad_y_deg, 0.0, places=6)

        # Small residual turn after mark (field report production Δ +7.55 was
        # the *mismatched* symptom under the bug; under the fix a true +7.55°
        # producer-Y motion yields production free Δ −7.55).
        oak.advance_producer_y(7.55, packets=30)
        for _ in range(2):
            poll()
        end = capture_mark_snapshot(imu, triad, poll, rebaseline_triad=False)

        d_prod = end.prod_free_yaw_deg - start.prod_free_yaw_deg
        d_y = end.triad_y_deg - start.triad_y_deg
        d_py = end.producer_cum_y_deg - start.producer_cum_y_deg

        self.assertAlmostEqual(d_y, d_py, places=5)
        self.assertAlmostEqual(d_prod, -d_py, places=5)
        self.assertTrue(production_matches_neg_producer_y(d_prod, d_py))


class TestSynchronizedBaselines(unittest.TestCase):
    def test_capture_mark_rebaselines_triad_atomically(self):
        oak = FakeOak()
        imu = FakeImu(oak)
        triad = TriadIntegrator()

        def poll():
            return poll_imu_once(imu, triad)

        poll()  # establish triad baseline at cum≈0
        oak.advance_producer_y(10.0, packets=5)
        poll()
        self.assertAlmostEqual(triad.y_deg, 10.0, places=4)

        snap = capture_mark_snapshot(imu, triad, poll, rebaseline_triad=True)
        self.assertAlmostEqual(snap.triad_y_deg, 0.0, places=6)
        self.assertAlmostEqual(triad.y_deg, 0.0, places=6)
        self.assertIsNotNone(snap.producer_cum_y_deg)

    def test_rate_window_stats_and_nmni_fraction(self):
        rw = RateWindow()
        rw.add_sample(0.0, 0.1, 0.0, nmni_threshold_dps=0.3)
        rw.add_sample(0.0, 1.0, 0.0, nmni_threshold_dps=0.3)
        rw.add_sample(0.0, -0.05, 0.0, nmni_threshold_dps=0.3)
        sy = rw.y.summary()
        self.assertEqual(int(sy["count"]), 3)
        self.assertAlmostEqual(sy["mean"], (0.1 + 1.0 - 0.05) / 3.0, places=6)
        self.assertEqual(rw.gy_below_nmni, 2)
        self.assertAlmostEqual(rw.gy_below_nmni_fraction(), 2 / 3, places=6)


class TestExpectedZeroAndLongBias(unittest.TestCase):
    def test_expected_zero_scale_and_pass_band(self):
        # Stationary report rows must not throw / div0
        for meas in (0.0, 0.3, -0.2):
            rec = recommended_scale(meas, 0.0, 1.0, production=False)
            self.assertTrue(math.isnan(rec))
            rec_p = recommended_scale(meas, 0.0, 1.0, production=True)
            self.assertTrue(math.isnan(rec_p))
        self.assertTrue(_pass_band(0.0, 0.0, 8.0, 0.1))

    def test_parse_expected_zero_stream_bias_ten(self):
        args = parse_chalk_args(
            ["--expected", "0", "--stream", "--bias-s", "10", "--no-nmni"]
        )
        self.assertEqual(args.expected, 0.0)
        self.assertTrue(args.stream)
        self.assertEqual(args.bias_s, 10.0)
        # recommended_scale with expected 0 never raises
        self.assertTrue(math.isnan(recommended_scale(0.0, args.expected)))


class TestNoMotorOrConfigWrites(unittest.TestCase):
    def test_module_source_has_no_motor_calls(self):
        src_path = Path(inspect.getsourcefile(chalk))
        src = src_path.read_text(encoding="utf-8")
        # Motor / drive stack must stay out of the chalk harness.
        for token in (
            "from pi_app.hardware.vesc",
            "import pi_app.hardware.vesc",
            "from pi_app.hardware.arduino_rc",
            "from pi_app.hardware.arduino_modelx",
            "VescCanDriver",
            "ArduinoMotor",
            "set_duty(",
            "set_rpm(",
            "json.dump",
            "open(config",
        ):
            self.assertNotIn(token, src, msg=f"forbidden token in chalk harness: {token}")
        # Only read config; never assign into imu_steering production knobs.
        self.assertNotIn("oak_yaw_rate_source =", src)
        self.assertNotIn("oak_yaw_rate_scale =", src)
        self.assertNotIn("oak_nmni_enabled =", src)

    def test_main_does_not_write_config_or_motors(self):
        """Smoke: main aborts on no device; never constructs motors."""
        with patch.object(chalk.OakDepthReader, "detect", return_value=False):
            with patch("builtins.print"):
                rc = chalk.main(["--expected", "0", "--bias-s", "10", "--stream"])
        self.assertEqual(rc, 1)

    def test_config_defaults_untouched_by_nmni_resolve(self):
        # resolve only returns values; does not mutate config
        import config as cfg_mod

        before = bool(cfg_mod.config.imu_steering.oak_nmni_enabled)
        resolve_nmni_enabled(False, before)
        resolve_nmni_enabled(True, before)
        after = bool(cfg_mod.config.imu_steering.oak_nmni_enabled)
        self.assertEqual(before, after)
        # Production axis/scale defaults preserved
        self.assertEqual(cfg_mod.config.imu_steering.oak_yaw_rate_source, "gyro_y")
        self.assertEqual(cfg_mod.config.imu_steering.oak_yaw_rate_scale, 1.0)


class TestProductionNegProducerY(unittest.TestCase):
    def test_match_helper(self):
        self.assertTrue(production_matches_neg_producer_y(-90.0, 90.0))
        self.assertTrue(production_matches_neg_producer_y(90.0, -90.0))
        self.assertFalse(production_matches_neg_producer_y(90.0, 90.0))


if __name__ == "__main__":
    unittest.main()
