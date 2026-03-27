"""Tests for tools/replay_follow_me_log offline replay harness."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import pi_app.control.follow_me as follow_me_mod  # noqa: E402
from config import config  # noqa: E402
from pi_app.control.follow_me import FollowMeController, PersonDetection  # noqa: E402
from pi_app.control.mapping import CENTER_OUTPUT_VALUE  # noqa: E402
from tools.follow_me_analyze import parse_follow_me_runs  # noqa: E402
from tools.replay_follow_me_log import replay_run  # noqa: E402


FIXTURE = _ROOT / "pi_app" / "tests" / "fixtures" / "follow_me_replay_sample.jsonl"


class TestReplayFollowMeLog(unittest.TestCase):
    def test_parse_fixture_has_one_segment(self) -> None:
        runs = parse_follow_me_runs(str(FIXTURE))
        self.assertEqual(len(runs), 1)
        self.assertEqual(len(runs[0]), 3)

    def test_replay_run_produces_rows(self) -> None:
        runs = parse_follow_me_runs(str(FIXTURE))
        rows = replay_run(runs[0], config.follow_me, no_gps=True)
        self.assertEqual(len(rows), 3)
        self.assertIn("replay_steer_offset", rows[0])
        self.assertIn("replay_speed_offset", rows[0])
        self.assertIn("pursuit_mode", rows[0])

    def test_simulation_roundtrip_matches_forward_run(self) -> None:
        """Forward-simulate Follow Me, record pseudo-log frames, replay offline — outputs must match."""
        real_mono = follow_me_mod.time.monotonic
        mono = [0.0]

        def fake_mono() -> float:
            return float(mono[0])

        follow_me_mod.time.monotonic = fake_mono  # type: ignore[assignment]
        try:
            ctrl = FollowMeController(config.follow_me)
            t0 = 1000.0
            dt = 0.02
            prev_l = int(CENTER_OUTPUT_VALUE)
            prev_r = int(CENTER_OUTPUT_VALUE)
            frames: list[dict] = []
            for i in range(25):
                mono[0] = i * dt
                ctrl.update_pose(0.0, prev_l, prev_r, mono[0])
                z_m = 2.8
                x_m = 0.12 * math.sin(i * 0.15)
                dets = [
                    PersonDetection(
                        x_m=x_m,
                        z_m=z_m,
                        confidence=0.9,
                        bbox=(0.0, 0.0, 1.0, 1.0),
                        track_id=1,
                    )
                ]
                left, right = ctrl.compute(dets)
                st = ctrl.get_status()
                frames.append(
                    {
                        "ts": t0 + i * dt,
                        "mode": "FOLLOW_ME",
                        "imu": {"heading_deg": 0.0},
                        "motor": {"L": left, "R": right},
                        "follow_me": {
                            "num_detections": 1,
                            "target_z_m": z_m,
                            "target_x_m": x_m,
                            "confidence": 0.9,
                            "speed_offset": st.get("follow_me_speed_offset"),
                            "steer_offset": st.get("follow_me_steer_offset"),
                            "pursuit_mode": st.get("follow_me_pursuit_mode"),
                        },
                        "gps": {"lat": None, "lon": None, "fix": None},
                    }
                )
                prev_l, prev_r = left, right
        finally:
            follow_me_mod.time.monotonic = real_mono  # type: ignore[assignment]

        rows = replay_run(frames, config.follow_me, no_gps=True)
        self.assertEqual(len(rows), len(frames))
        for i, (row, fr) in enumerate(zip(rows, frames, strict=True)):
            self.assertEqual(
                row["replay_L"],
                fr["motor"]["L"],
                f"frame {i} L mismatch",
            )
            self.assertEqual(
                row["replay_R"],
                fr["motor"]["R"],
                f"frame {i} R mismatch",
            )


if __name__ == "__main__":
    unittest.main()
