"""Tests for tools/replay_follow_me_log offline replay harness."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from config import config  # noqa: E402
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


if __name__ == "__main__":
    unittest.main()
