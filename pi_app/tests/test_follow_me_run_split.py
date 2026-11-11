"""parse_follow_me_runs must split on a real mode change, not on slow lines."""

from __future__ import annotations

import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.follow_me_analyze import RUN_GAP_S, parse_follow_me_runs  # noqa: E402


def _tick(ts: float, mode: str) -> dict:
    return {"ts": ts, "mode": mode, "follow_me": {}}


def _write(tmp_path: Path, lines: list) -> str:
    p = tmp_path / "arm_test.log"
    with open(p, "w") as f:
        for ln in lines:
            f.write((ln if isinstance(ln, str) else json.dumps(ln)) + "\n")
    return str(p)


def test_slow_and_event_lines_do_not_split_a_run(tmp_path):
    lines = [{"type": "session_header", "ts": 99.0}]
    t = 100.0
    for i in range(30):
        lines.append(_tick(t, "FOLLOW_ME"))
        t += 0.1
        if i % 9 == 8:
            lines.append({"type": "slow", "ts": t, "oak": {}})
        if i == 15:
            lines.append({"type": "event", "event": "oak_health", "ts": t})
    lines.append("not json")
    lines += [_tick(t, "MANUAL"), _tick(t + 0.1, "MANUAL")]
    t += 1.0
    for _ in range(5):
        lines.append(_tick(t, "FOLLOW_ME"))
        t += 0.1
    runs = parse_follow_me_runs(_write(tmp_path, lines))
    assert [len(r) for r in runs] == [30, 5]


def test_session_header_ends_a_run(tmp_path):
    lines = [_tick(1.0, "FOLLOW_ME"), _tick(1.1, "FOLLOW_ME"),
             {"type": "session_header", "ts": 1.15},
             _tick(1.2, "FOLLOW_ME")]
    runs = parse_follow_me_runs(_write(tmp_path, lines))
    assert [len(r) for r in runs] == [2, 1]


def test_time_gap_ends_a_run(tmp_path):
    lines = [_tick(1.0, "FOLLOW_ME"), _tick(1.1, "FOLLOW_ME"),
             _tick(1.1 + RUN_GAP_S + 1.0, "FOLLOW_ME")]
    runs = parse_follow_me_runs(_write(tmp_path, lines))
    assert [len(r) for r in runs] == [2, 1]
