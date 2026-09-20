"""Tests for tools/analyze_follow_me_log offline FOLLOW_ME log analyzer."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.analyze_follow_me_log import analyze, main  # noqa: E402


T0 = 1789861810.0
DT = 0.1


def _iso(ts: float) -> str:
    # Deterministic label; analyzer only echoes ts_iso, it does not parse it.
    frac = int(round((ts - T0) * 1000))
    return f"2026-09-19 18:50:10.{frac:03d}"


def _header() -> dict:
    return {
        "type": "session_header",
        "ts": T0 - 1.0,
        "ts_iso": "2026-09-19 18:50:09.000",
        "git_sha": "deadbeef",
        "config": {
            "follow_me": {
                "speed_kp": 0.6,
                "speed_ki": 0.15,
                "speed_kd": 0.0,
                "speed_integral_limit": 1.0,
                "speed_pid_max_correction_mps": 0.20,
                "speed_loop_mps_per_byte": 0.009434,
                "follow_distance_m": 1.5,
                "max_follow_speed_byte": 110,
            },
            "vesc": {"rpm_plausibility_enabled": True},
            "imu_steering": {"kp": 1.2, "ki": 0.0, "kd": 0.1},
        },
    }


def _slow() -> dict:
    return {"type": "slow", "ts": T0, "ts_iso": _iso(T0), "imu_pipeline": {}}


def _tick(i: int, *, mode: str = "FOLLOW_ME", **over) -> dict:
    ts = T0 + i * DT
    bbox = over.pop("bbox", [0.41, 0.05, 0.59, 0.85])
    fresh = over.pop("fresh_detection", True)
    throttle = over.pop("throttle_scale", 1.0)
    speed_offset = over.pop("speed_offset", 80.0)
    obj = {
        "ts": ts,
        "ts_iso": _iso(ts),
        "mode": mode,
        "loop_dt_ms": 32,
        "motor": {"L": 154, "R": 154},
        "vesc": {
            "left_rpm": 731, "right_rpm": 17, "speed_mps": 0.03,
            "rpm_plausible": True, "gate_trips": 0,
            "l_temp_c": 39.2, "r_temp_c": 38.5, "l_duty": 0.132, "r_duty": 0.078,
        },
        "obstacle": {
            "distance_m": 2.4, "throttle_scale": throttle,
            "depth_p5_mm": 2419.0, "depth_p50_mm": 3138.8, "depth_valid_pct": 1.0,
        },
        "safety": {"armed": True, "emergency": False},
        "detections": [{
            "x_m": 0.06, "z_m": 2.36, "conf": 0.94, "bbox": bbox,
        }],
        "imu": {
            "heading_deg": 211.4,
            "oak_imu": {
                "yaw_rate_world_dps": 1.2, "heading_deg": 211.4,
                "stationary": False, "zupt_active": False,
            },
        },
        "follow_me": {
            "tracking": True,
            "target_z_m": 2.4, "target_x_m": 0.1,
            "target_track_id": 2.0, "num_persons": 1.0,
            "distance_error_m": 0.9, "speed_offset": speed_offset,
            "steer_offset": -0.1, "actual_speed_mps": 0.0,
            "pursuit_mode": "direct", "trail_length": 1.0,
            "confidence": 0.9, "num_detections": 1.0,
            "steer_decay_factor": 1.0, "fresh_detection": fresh,
            "steer_hold_active": False,
            "speed_loop": {
                "open_loop_byte": 65.9, "target_mps": 0.6, "actual_mps": 0.0,
                "err_mps": 0.6, "p": 0.4, "i": 0.0, "d": -0.0,
                "corr_mps": 0.2, "corr_byte": 21.2, "closed": True,
            },
        },
    }
    for k, v in over.items():
        obj[k] = v
    return obj


def _write_log(path: Path) -> None:
    """12 FOLLOW_ME ticks + 2 MANUAL; one dropout CUT; one obstacle tick; one thin bbox."""
    lines = [_header(), _slow()]
    for i in range(12):
        kw: dict = {}
        if i < 5:
            kw["speed_offset"] = 80.0
        else:
            kw["speed_offset"] = 45.0  # drop of 35 bytes at i=5 (0.5 s)
        if i == 5:
            # CUT window tick: dropout + width-0.06 detection the filter should count.
            kw["fresh_detection"] = False
            kw["bbox"] = [0.40, 0.10, 0.46, 0.80]  # width 0.06
        if i == 10:
            kw["throttle_scale"] = 0.4
        lines.append(_tick(i, **kw))
    lines.append(_tick(12, mode="MANUAL"))
    lines.append(_tick(13, mode="MANUAL"))
    path.write_text("".join(json.dumps(obj) + "\n" for obj in lines), encoding="utf-8")


class TestAnalyzeFollowMeLog(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.log_path = Path(self._tmp.name) / "run.jsonl"
        _write_log(self.log_path)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_analyze_finds_segment_and_dropout_cut(self) -> None:
        result = analyze([self.log_path])
        self.assertEqual(result["header"]["n_segments"], 1)
        self.assertEqual(result["header"]["segments"][0]["n_ticks"], 12)
        self.assertEqual(result["header"]["n_follow_me_ticks"], 12)
        cuts = result["jumpiness"]["cuts"]
        self.assertGreaterEqual(len(cuts), 1)
        self.assertEqual(cuts[0]["cause"], "dropout")
        self.assertEqual(result["detection_filter"]["n_ticks"], 1)
        self.assertGreaterEqual(result["detection_filter"]["n_width_lt_0_07"], 1)
        self.assertEqual(result["obstacle"]["n_ticks"], 1)
        self.assertEqual(result["obstacle"]["n_runs"], 1)

    def test_main_writes_json_with_jumpiness(self) -> None:
        out = Path(self._tmp.name) / "metrics.json"
        rc = main([str(self.log_path), "--json", str(out)])
        self.assertEqual(rc, 0)
        self.assertTrue(out.is_file())
        data = json.loads(out.read_text(encoding="utf-8"))
        self.assertIn("jumpiness", data)
        self.assertIn("cuts", data["jumpiness"])

    def test_direct_cap_falls_back_to_32_when_header_omits_it(self) -> None:
        result = analyze([self.log_path])
        self.assertEqual(result["steering"]["direct_cap"], 32.0)

    def test_direct_cap_from_session_header(self) -> None:
        header = _header()
        header["config"]["follow_me"]["direct_mode_max_steer_byte"] = 40.0
        path = Path(self._tmp.name) / "cap.jsonl"
        path.write_text(
            "".join(json.dumps(obj) + "\n" for obj in [header, _tick(0)]),
            encoding="utf-8",
        )
        result = analyze([path])
        self.assertEqual(result["steering"]["direct_cap"], 40.0)

    def test_heading_yaw_fit_recovers_slope_and_lag(self) -> None:
        """Heading derivative vs L-R delayed 3 ticks recovers slope 0.7.

        Logged yaw_rate_dps is zero on alternate ticks so the old logged-rate
        fit is the unreliable series the analyzer must still report.
        """
        n = 120
        dt = DT
        period = 80
        omega = 2.0 * math.pi / (period * dt)
        amp = 40.0
        lag_ticks = 3
        plant = 0.7
        heading_off = 350.0
        lines: list[dict] = [_header()]
        for i in range(n):
            t = i * dt
            lr = amp * math.sin(omega * t)
            heading_unw = heading_off - (plant * amp / omega) * math.cos(
                omega * (t - lag_ticks * dt)
            )
            heading = heading_unw % 360.0
            logged_rate = 0.0 if (i % 2 == 0) else 5.0
            tk = _tick(i)
            tk["motor"] = {"L": 160.0 + lr / 2.0, "R": 160.0 - lr / 2.0}
            tk["imu"] = {
                "heading_deg": heading,
                "yaw_rate_dps": logged_rate,
                "oak_imu": {
                    "yaw_rate_world_dps": logged_rate,
                    "heading_deg": heading,
                },
            }
            tk["follow_me"]["speed_loop"]["actual_mps"] = 0.8
            lines.append(tk)
        path = Path(self._tmp.name) / "yaw_plant.jsonl"
        path.write_text("".join(json.dumps(obj) + "\n" for obj in lines), encoding="utf-8")
        result = analyze([path])
        fit = result["steering"]["yaw_moving"]
        self.assertEqual(fit["lag_ticks"], 3)
        self.assertAlmostEqual(fit["slope"], 0.7, delta=0.05)
        logged = fit["logged"]
        self.assertGreaterEqual(logged["n_zero"], n // 2 - 1)
        self.assertEqual(logged["n_samples"], n)


if __name__ == "__main__":
    unittest.main()
