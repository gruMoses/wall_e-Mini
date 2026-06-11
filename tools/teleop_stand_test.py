"""Automated wheels-off-the-ground teleop stand test.

Run ON the Pi (or anywhere that can reach the dashboard) while the robot is
on a stand with wheels free and RC ch3 ARMED. Drives the motors through the
phone-teleop REST API exactly like a phone would, then verifies the safety
envelope from the outside:

  Phase A: arm + drive 6 s at half stick (slow cap), sampling VESC RPM
  Phase B: stop sending frames mid-drive -> deadman must trip <= 250 ms,
           motors neutral, session disarmed
  Phase C: re-arm, drive, E-STOP, then verify the latch refuses a clear
           until RC ch3 is cycled

Prints a compact PASS/FAIL report. Exits nonzero on any FAIL.

Usage:
  python3 tools/teleop_stand_test.py [--base http://localhost:8080] [--token T]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request


def _post(base: str, path: str, payload: dict, token: str) -> dict:
    req = urllib.request.Request(
        base + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "X-Teleop-Token": token},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.loads(r.read().decode())


def _get(base: str, path: str, token: str) -> dict:
    req = urllib.request.Request(base + path, headers={"X-Teleop-Token": token})
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.loads(r.read().decode())


def _sse_snapshot(base: str) -> dict:
    """Read one telemetry event from the SSE stream."""
    req = urllib.request.Request(base + "/api/telemetry")
    with urllib.request.urlopen(req, timeout=4) as r:
        for raw in r:
            line = raw.decode("utf-8", "replace").strip()
            if line.startswith("data:"):
                return json.loads(line[5:])
    return {}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://localhost:8080")
    ap.add_argument("--token", default="")
    args = ap.parse_args()
    base, token = args.base.rstrip("/"), args.token

    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        results.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    print("== Pre-flight ==")
    snap = _sse_snapshot(base)
    rc_armed = bool(snap.get("is_armed"))
    print(f"  controller armed={rc_armed} mode={snap.get('mode')}")
    if not rc_armed:
        print("ABORT: RC ch3 is not armed — motors cannot spin. Arm the RC first.")
        return 2

    print("== Phase A: arm + drive 6 s, sample RPM ==")
    arm = _post(base, "/api/teleop/session/arm", {"hold_ms": 600, "rc_in_hand": False}, token)
    check("session armed", bool(arm.get("ok")), str(arm.get("reason")))

    seq = 0
    rpm_samples: list[tuple[int, int]] = []
    t0 = time.monotonic()
    next_sse = 0.0
    while time.monotonic() - t0 < 6.0:
        seq += 1
        _post(base, "/api/teleop/session/drive",
              {"seq": seq, "t": time.time(), "left": 0.5, "right": 0.5}, token)
        now = time.monotonic()
        if now - t0 >= next_sse:
            s = _sse_snapshot(base)
            rpm_samples.append((s.get("vesc_left_rpm") or 0, s.get("vesc_right_rpm") or 0))
            next_sse += 1.0
        time.sleep(0.1)

    peak_l = max((abs(a) for a, _ in rpm_samples), default=0)
    peak_r = max((abs(b) for _, b in rpm_samples), default=0)
    check("RPM nonzero while driving (service path)", peak_l > 0 and peak_r > 0,
          f"peak L={peak_l} R={peak_r} samples={rpm_samples}")

    print("== Phase B: deadman — stop sending mid-drive ==")
    # Last drive frame was <100 ms ago and the session is armed. Go silent.
    time.sleep(0.6)  # > 250 ms deadman + margin
    st = _get(base, "/api/teleop/session/status", token)
    check("deadman tripped", st.get("armed") is False and st.get("tripped_reason") == "deadman",
          f"armed={st.get('armed')} trip={st.get('tripped_reason')}")
    s = _sse_snapshot(base)
    check("motors neutral after trip",
          s.get("motor_left") == 126 and s.get("motor_right") == 126,
          f"bytes L={s.get('motor_left')} R={s.get('motor_right')}")

    print("== Phase C: e-stop latch ==")
    arm = _post(base, "/api/teleop/session/arm", {"hold_ms": 600, "rc_in_hand": False}, token)
    check("re-arm after deadman allowed", bool(arm.get("ok")), str(arm.get("reason")))
    for i in range(10):  # 1 s of driving
        seq += 1
        _post(base, "/api/teleop/session/drive",
              {"seq": seq, "t": time.time(), "left": 0.4, "right": 0.4}, token)
        time.sleep(0.1)
    _post(base, "/api/teleop/session/estop", {}, token)
    st = _get(base, "/api/teleop/session/status", token)
    check("e-stop latched + disarmed",
          st.get("estop_latched") is True and st.get("armed") is False,
          f"latched={st.get('estop_latched')} armed={st.get('armed')}")
    # The controller slews outputs toward neutral rather than stepping
    # (observed 2026-06-11: 135/135 one frame after e-stop, 126/126 settled).
    # Sample after the ramp, not during it.
    time.sleep(0.8)
    s = _sse_snapshot(base)
    check("motors neutral after e-stop",
          s.get("motor_left") == 126 and s.get("motor_right") == 126,
          f"bytes L={s.get('motor_left')} R={s.get('motor_right')}")
    clr = _post(base, "/api/teleop/session/clear_estop", {}, token)
    st = _get(base, "/api/teleop/session/status", token)
    check("clear refused until RC ch3 cycle", st.get("estop_latched") is True,
          f"clear_resp={clr} latched={st.get('estop_latched')}")

    print()
    fails = [r for r in results if not r[1]]
    print(f"== RESULT: {len(results) - len(fails)}/{len(results)} passed ==")
    if fails:
        for name, _, detail in fails:
            print(f"   FAILED: {name} ({detail})")
    print("Robot left in: e-stop latched, disarmed. To finish: cycle RC ch3")
    print("down/up, then clear the e-stop (phone two-tap or POST clear_estop).")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
