"""Tests for tools/imu_pipeline_analyzer.py's dual-format imu_pipeline
reader (2026-09-19 logging audit, Commit C): imu_pipeline moved from the
per-tick line into a once/second {"type": "slow", ...} line. The analyzer
must read new-format logs (slow line only) and still read old logs captured
before the migration (imu_pipeline embedded in every per-tick line).
"""
import io
import json
import sys
import tempfile
import unittest
from contextlib import contextmanager, redirect_stdout
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.imu_pipeline_analyzer import _imu_pipeline_rows, main


def _old_format_tick(imu_pipeline, loop_dt_ms=20):
    # Pre-migration per-tick row: imu_pipeline embedded directly, no "type".
    return {"ts": 1000.0, "loop_dt_ms": loop_dt_ms, "imu_pipeline": imu_pipeline}


def _new_format_tick(loop_dt_ms=20):
    # Post-migration per-tick row: no imu_pipeline at all.
    return {"ts": 1000.0, "loop_dt_ms": loop_dt_ms}


def _slow_line(imu_pipeline):
    return {"type": "slow", "ts": 1000.0, "imu_pipeline": imu_pipeline}


class TestImuPipelineRowsDualFormat(unittest.TestCase):

    def test_prefers_slow_lines_when_present(self):
        rows = [
            _new_format_tick(),
            _new_format_tick(),
            _slow_line({"metrics_available": True, "queue_msgs_received": 1}),
            _slow_line({"metrics_available": True, "queue_msgs_received": 2}),
        ]
        result = _imu_pipeline_rows(rows)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["queue_msgs_received"], 1)
        self.assertEqual(result[1]["queue_msgs_received"], 2)

    def test_falls_back_to_per_tick_for_old_logs(self):
        rows = [
            _old_format_tick({"metrics_available": True, "queue_msgs_received": 1}),
            _old_format_tick({"metrics_available": True, "queue_msgs_received": 2}),
            _old_format_tick({"metrics_available": True, "queue_msgs_received": 3}),
        ]
        result = _imu_pipeline_rows(rows)
        self.assertEqual(len(result), 3)
        self.assertEqual([r["queue_msgs_received"] for r in result], [1, 2, 3])

    def test_no_imu_pipeline_anywhere_returns_empty(self):
        rows = [_new_format_tick(), _new_format_tick()]
        self.assertEqual(_imu_pipeline_rows(rows), [])


class TestMainReadsDualFormatLogs(unittest.TestCase):

    def _run_main(self, log_path):
        buf = io.StringIO()
        with patch.object(sys, "argv", ["imu_pipeline_analyzer.py", "--log", str(log_path)]):
            with redirect_stdout(buf):
                rc = main()
        return rc, buf.getvalue()

    def test_new_format_log_reports_slow_line_metrics(self):
        lines = [
            {"type": "session_header", "schema": 1},
            _new_format_tick(loop_dt_ms=15),
            _slow_line({
                "metrics_available": True,
                "queue_msgs_received": 100, "queue_msgs_consumed": 99,
                "queue_msgs_dropped": 0, "queue_drain_count": 10,
            }),
            _new_format_tick(loop_dt_ms=16),
            _slow_line({
                "metrics_available": True,
                "queue_msgs_received": 200, "queue_msgs_consumed": 198,
                "queue_msgs_dropped": 0, "queue_drain_count": 20,
            }),
        ]
        with self._temp_log(lines) as path:
            rc, out = self._run_main(path)
        self.assertEqual(rc, 0)
        self.assertIn("imu_pipeline=rows:2", out)
        self.assertIn("recv_delta:100.0", out)  # 200 - 100

    def test_old_format_log_still_reports_metrics(self):
        lines = [
            _old_format_tick({"metrics_available": True, "queue_msgs_received": 10}, loop_dt_ms=15),
            _old_format_tick({"metrics_available": True, "queue_msgs_received": 30}, loop_dt_ms=16),
        ]
        with self._temp_log(lines) as path:
            rc, out = self._run_main(path)
        self.assertEqual(rc, 0)
        self.assertIn("imu_pipeline=rows:2", out)
        self.assertIn("recv_delta:20.0", out)  # 30 - 10

    def test_log_with_no_imu_pipeline_at_all(self):
        lines = [_new_format_tick(loop_dt_ms=15), _new_format_tick(loop_dt_ms=16)]
        with self._temp_log(lines) as path:
            rc, out = self._run_main(path)
        self.assertEqual(rc, 0)
        self.assertIn("imu_pipeline=none", out)

    @contextmanager
    def _temp_log(self, rows):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".log", delete=False) as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")
            path = Path(f.name)
        try:
            yield path
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
