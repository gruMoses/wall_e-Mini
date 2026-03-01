#!/usr/bin/env python3
"""
Run a timed FOLLOW_ME probe via web API and summarize outcomes.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path


def _toggle(base_url: str) -> tuple[int, dict]:
    req = urllib.request.Request(f"{base_url}/api/follow_me", method="POST")
    with urllib.request.urlopen(req, timeout=5) as resp:
        payload = json.loads(resp.read().decode("utf-8"))
        return resp.status, payload


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-url", default="http://127.0.0.1:8080")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--log", type=Path, default=Path("/home/pi/wall_e-Mini/logs/latest.log"))
    ap.add_argument("--analyzer", type=Path, default=Path("/home/pi/wall_e-Mini/tools/perf/follow_me_analyze.py"))
    ap.add_argument("--out-window", type=Path, default=None)
    args = ap.parse_args()

    start = time.time()
    print(f"start_epoch={start}")
    try:
        st_on, resp_on = _toggle(args.base_url)
        print(f"toggle_on_status={st_on} response={resp_on}")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="ignore")
        print(f"toggle_on_http_error={e.code} body={body}")
        return 2
    except Exception as e:
        print(f"toggle_on_error={type(e).__name__}: {e}")
        return 2

    time.sleep(max(0.0, float(args.seconds)))

    end = time.time()
    print(f"end_epoch={end}")
    try:
        st_off, resp_off = _toggle(args.base_url)
        print(f"toggle_off_status={st_off} response={resp_off}")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="ignore")
        print(f"toggle_off_http_error={e.code} body={body}")
    except Exception as e:
        print(f"toggle_off_error={type(e).__name__}: {e}")

    if args.out_window is not None:
        args.out_window.write_text(json.dumps({"start_epoch": start, "end_epoch": end}))
        print(f"window_saved={args.out_window}")

    if not args.log.exists():
        print(f"analyze_skipped_missing_log={args.log}")
        return 0
    if not args.analyzer.exists():
        print(f"analyze_skipped_missing_analyzer={args.analyzer}")
        return 0

    cmd = [
        "python3",
        str(args.analyzer),
        "--log",
        str(args.log),
        "--start-epoch",
        str(start),
        "--end-epoch",
        str(end),
    ]
    print("analyzer_cmd=" + " ".join(cmd))
    subprocess.run(cmd, check=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
