#!/usr/bin/env python3
"""Bench tests for the walle-power watcher on the X1202 (runs on the Mac).

Imports Bench from x1202_cycle_test.py (same directory) and drives the outlet
and the Pi the same way. The watcher (walle-power.service) does the halt; this
script never issues poweroff. Design: docs/x1202_power_watcher_design.md.

Subcommands: status, short, long, window, hang, resume.
Every run of a test appends one JSON line to --results and prints one summary
line. Every test ends with the outlet ON and the Pi reachable (one outlet
cycle if it is not; still down = PiDown, all testing stops).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from x1202_cycle_test import Bench, PiDown, now  # noqa: E402

STATUS = "/run/walle-power/status.json"
VCELL_MIN = 3.60
KEYS = ("HALT", "halt step", "cancel", "FAULT")
PROBE = ("cat /proc/sys/kernel/random/boot_id; "
         "systemctl show -p ActiveEnterTimestampMonotonic --value wall-e.service; "
         "systemctl is-active walle-power; systemctl is-active wall-e; echo ---; "
         "tr -d '\\n' < %s" % STATUS)


def probe(b):
    """One ssh: boot id, robot start stamp, unit states, status file. None if ssh fails."""
    rc, out, _ = b.ssh(PROBE, timeout=10, connect=4)
    if rc in (124, 255) or "---" not in out:
        return None
    head, _, tail = out.partition("---")
    h = head.split()
    if len(h) < 4:
        return None
    try:
        st = json.loads(tail.strip())
    except ValueError:
        st = {}
    return {"boot_id": h[0], "robot_ts": h[1], "watcher": h[2], "robot": h[3], "st": st, "t": now()}


def precheck(b, rec, wait_s=60):
    """Reason string when the test must not start, else None. Retries for wait_s."""
    t0, why = now(), "no ssh"
    while True:
        p = probe(b)
        if p is None:
            why = "Pi not reachable by ssh"
        else:
            st = p["st"]
            v = st.get("vcell_v")
            if p["watcher"] != "active":
                why = "walle-power is %s" % p["watcher"]
            elif st.get("boot_id") != p["boot_id"]:
                why = "status boot_id %s != boot id" % st.get("boot_id")
            elif st.get("state") != "run":
                why = "state is %s" % st.get("state")
            elif st.get("input_good") is not True:
                why = "input_good is %s" % st.get("input_good")
            elif v is None or v < VCELL_MIN:
                why = "vcell_v %s < %.2f" % (v, VCELL_MIN)
            else:
                rec["pre"] = p
                return None
        if now() - t0 >= wait_s:
            return why
        time.sleep(5)


class Stream:
    """Status file sampled on the Pi every 0.25 s over one ssh; stamped on the Mac."""

    def __init__(self, b):
        cmd = "while :; do tr -d '\\n' < %s 2>/dev/null; echo; sleep 0.25; done" % STATUS
        self.p = subprocess.Popen(["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
                                   "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", b.pi, cmd],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self.s, self.lock = [], threading.Lock()
        threading.Thread(target=self._rd, daemon=True).start()
        time.sleep(1.0)

    def _rd(self):
        for ln in self.p.stdout:
            try:
                r = json.loads(ln)
            except ValueError:
                continue
            r["_mac"] = now()
            with self.lock:
                self.s.append(r)

    def samples(self):
        with self.lock:
            return list(self.s)

    def stop(self):
        try:
            self.p.terminate()
        except Exception:  # noqa: BLE001
            pass

    def summary(self, t_off):
        s = [r for r in self.samples() if r["_mac"] >= t_off - 1]
        lows = [r.get("low_s") or 0 for r in s]
        bad = [r for r in s if r.get("input_good") is False and r["_mac"] >= t_off]
        hal = [r for r in s if r.get("state") == "halting"]
        return {"samples": len(s), "max_low_s": max(lows) if lows else None,
                "t_input_bad_s": round(bad[0]["_mac"] - t_off, 2) if bad else None,
                "t_halting_s": round(hal[0]["_mac"] - t_off, 2) if hal else None,
                "halting_seen": bool(hal),
                "vcell_min": min([r["vcell_v"] for r in s if r.get("vcell_v") is not None] or [None])}


def prev_journal(b, n=25, keys=KEYS):
    rc, out, _ = b.ssh("journalctl -b -1 -u walle-power --no-pager -o short-monotonic | tail -n %d" % n, timeout=25)
    return [ln[:220] for ln in out.splitlines() if any(k.lower() in ln.lower() for k in keys)] if rc == 0 else None


def ensure_up(b, rec, t_ref, force=False):
    """Pi must answer ssh within 180 s of t_ref; else one outlet cycle (off 30 s, on, wait 300 s)."""
    if not force:
        if b.contact():
            return
        _, c = b.wait_ssh(t_ref, limit=180)
        if c:
            return
    print("  Pi not answering: cycling the outlet once", flush=True)
    rec["recovered_by_cycle"] = True
    rec["cycle_off"] = b.outlet_off_verified()
    time.sleep(30)
    t2 = b.outlet_on_verified()
    rec["cycle_on"] = t2
    _, c = b.wait_ssh(t2, limit=300)
    if not c:
        rec["result"] = "FAIL"
        rec["reason"] = "Pi still down after the recovery cycle"
        raise PiDown("Pi down after the recovery cycle")


def restore(b, rec):
    if b.outlet_state() != "on":
        rec["restore_on"] = b.outlet_on_verified()
    ensure_up(b, rec, rec.get("restore_on") or rec.get("t_on") or now(), force=rec.pop("force_cycle", False))


def run_test(b, rec, body):
    why = precheck(b, rec)
    if why:
        rec["result"], rec["reason"] = "SKIP", why
        b.emit(rec)
        return rec
    rec["boot_id_before"] = rec["pre"]["boot_id"]
    rec["pre"] = {k: rec["pre"]["st"].get(k) for k in ("vcell_v", "low_s", "state", "input_good")}
    return b.step(rec, body)


def fail(rec, why):
    rec["result"], rec["reason"] = "FAIL", why


# ------------------------------------------------------------------ tests
def cmd_status(b, a):
    sh = ("systemctl is-active walle-power wall-e x1202-bench-logger walle-ups; echo ---; cat %s; echo; echo ---; "
          "pinctrl get 6,16; echo ---; journalctl -u walle-power --no-pager -n 5" % STATUS)
    print("outlet:", json.dumps(b.outlet_attrs(), default=str))
    print(b.ssh(sh, timeout=20)[1])


def short(b, a, rep):
    rec = {"test": "WB_SHORT", "rep": rep, "off_s": a.off_s or 3.0}

    def body(rec):
        ts0, m = rec["pre"], None
        p0 = probe(b)
        rec["robot_ts_before"] = p0["robot_ts"] if p0 else None
        try:
            m = Stream(b)
            rec["t_off"] = t_off = b.outlet_off_verified()
            time.sleep(max(0.0, rec["off_s"] - (now() - t_off)))
            rec["t_on"] = b.outlet_on_verified()
            time.sleep(max(0.0, 20 - (now() - rec["t_on"])))
            rec.update(m.summary(t_off))
        finally:
            if m:
                m.stop()
            restore(b, rec)
        p1 = probe(b)
        rec["boot_id_after"] = p1["boot_id"] if p1 else None
        rec["robot_ts_after"] = p1["robot_ts"] if p1 else None
        if rec["boot_id_after"] != rec["boot_id_before"]:
            fail(rec, "boot id changed")
        elif rec["robot_ts_after"] != rec["robot_ts_before"]:
            fail(rec, "wall-e.service was restarted")
        elif rec.get("halting_seen"):
            fail(rec, "state halting seen")
    return run_test(b, rec, body)


def long_(b, a, rep):
    off_s = a.off_s or 60.0
    rec = {"test": "WB_LONG", "rep": rep, "off_s": off_s}

    def body(rec):
        m, c = None, None
        try:
            m = Stream(b)
            rec["t_off"] = t_off = b.outlet_off_verified()
            while now() - t_off < off_s:
                if "t_ping_fail" not in rec and not b.ping():
                    rec["t_ping_fail"] = now()
                time.sleep(0.5)
            rec["t_on"] = t_on = b.outlet_on_verified()
            rec.update(m.summary(t_off))
            rec["t_down_s"] = round(rec["t_ping_fail"] - t_off, 2) if "t_ping_fail" in rec else None
            m.stop()
            fp, c = b.wait_ssh(t_on, limit=180, exclude_boot=rec["boot_id_before"])
            rec["first_ping"] = fp
            if c:
                rec["contact"] = c
                b.post_boot(rec, c, t_on)
        finally:
            if m:
                m.stop()
            restore(b, rec)
        rec["journal_prev"] = prev_journal(b)
        if rec.get("recovered_by_cycle"):
            fail(rec, "needed the recovery cycle")
        elif rec["t_down_s"] is None or not 6 <= rec["t_down_s"] <= 45:
            fail(rec, "Pi went down at %s s (need 6 to 45)" % rec["t_down_s"])
        elif not c or c["boot_id"] == rec["boot_id_before"]:
            fail(rec, "no new boot id")
    return run_test(b, rec, body)


def window(b, a, rep, off_s):
    rec = {"test": "WB_WINDOW", "rep": rep, "off_s": off_s, "result": "INFO"}

    def body(rec):
        m = None
        try:
            m = Stream(b)
            rec["robot_ts_before"] = (probe(b) or {}).get("robot_ts")
            rec["t_off"] = t_off = b.outlet_off_verified()
            time.sleep(max(0.0, off_s - (now() - t_off)))
            rec["t_on"] = t_on = b.outlet_on_verified()
            last, cls = None, None
            while now() - t_on < 150 and cls is None:
                p = probe(b)
                last = p if p else last
                if p and p["boot_id"] != rec["boot_id_before"]:
                    cls = "halted_and_restarted"
                    rec["t_new_boot_ssh_s"] = round(p["t"] - t_on, 2)
                    rec["boot_id"] = p["boot_id"]
                elif p and now() - t_on >= 30 and p["st"].get("state") == "run" and p["st"].get("input_good") \
                        and p["robot"] == "active":
                    cls = "rode_through" if p["robot_ts"] == rec["robot_ts_before"] else "cancelled"
                    rec["t_settled_s"] = round(p["t"] - t_on, 2)
                time.sleep(1.0)
            if cls is None:
                if last and last["t"] > now() - 15:
                    cls = "unsettled"
                    rec["last_probe"] = {"state": last["st"].get("state"), "robot": last["robot"]}
                else:
                    cls = "stayed_off"
                    rec["force_cycle"] = True
            rec["class"] = cls
            rec.update(m.summary(t_off))
        finally:
            if m:
                m.stop()
            restore(b, rec)
        if rec["class"] == "cancelled":
            rec["cancel_lines"] = b.ssh("journalctl -b 0 -u walle-power --no-pager -o short-monotonic | "
                                        "grep -i cancel | tail -n 5", timeout=20)[1].splitlines()
        if rec["class"] == "halted_and_restarted":
            rec["journal_prev"] = prev_journal(b)
        rec["reason"] = rec["class"]
    return run_test(b, rec, body)


def hang(b, a):
    rec = {"test": "WB_HANG", "rep": 1}

    def body(rec):
        pid = b.ssh("systemctl show -p MainPID --value walle-power", timeout=10)[1].strip()
        if not pid.isdigit() or pid == "0":
            raise RuntimeError("no MainPID: %r" % pid)
        rec["pid_old"], g16, t_gone, t_back, fails = pid, [], None, None, 0
        try:
            rec["t_stop"] = t0 = now()
            b.ssh("sudo -n kill -STOP %s" % pid, timeout=8)
            while now() - t0 < 60 and t_back is None:
                rc, out, _ = b.ssh("systemctl show -p ActiveState -p MainPID walle-power; pinctrl get 16", timeout=8)
                if rc not in (0, 1):
                    fails += 1
                else:
                    kv = dict(ln.split("=", 1) for ln in out.splitlines() if "=" in ln)
                    g16.append("op dh" in out)
                    gone = kv.get("ActiveState") != "active" or kv.get("MainPID") != pid
                    if gone and t_gone is None:
                        t_gone = now() - t0
                    if kv.get("ActiveState") == "active" and kv.get("MainPID") not in (pid, "0"):
                        t_back, rec["pid_new"] = now() - t0, kv.get("MainPID")
                time.sleep(1.0)
        finally:
            b.ssh("sudo -n kill -CONT %s 2>/dev/null; true" % pid, timeout=8)
        rec.update(t_not_active_s=round(t_gone, 1) if t_gone else None, t_active_again_s=round(t_back, 1) if t_back else None,
                   g16_samples=len(g16), g16_all_dh=all(g16) and bool(g16), poll_failures=fails)
        rec["g16_final"] = b.ssh("pinctrl get 16", timeout=8)[1].strip()[:80]
        p = probe(b)
        rec["boot_id_after"] = p["boot_id"] if p else None
        if t_back is None:
            fail(rec, "watcher not active again with a new MainPID within 60 s")
        elif "op dh" not in rec["g16_final"] or not rec["g16_all_dh"]:
            fail(rec, "GPIO 16 was not op dh throughout")
        elif rec["boot_id_after"] != rec["boot_id_before"]:
            fail(rec, "the Pi rebooted")
    return run_test(b, rec, body)


def resume(b, a):
    rec = {"test": "WB_RESUME", "rep": 1}

    def body(rec):
        pid = b.ssh("systemctl show -p MainPID --value walle-power", timeout=10)[1].strip()
        if not pid.isdigit() or pid == "0":
            raise RuntimeError("no MainPID: %r" % pid)
        rec["pid_old"], m, down = pid, None, False
        try:
            m = Stream(b)
            rec["t_off"] = t_off = b.outlet_off_verified()
            while now() - t_off < 60 and "t_kill" not in rec:
                if any(r.get("state") == "halting" and r["_mac"] >= t_off for r in m.samples()):
                    rec["t_kill"] = now()
                    rec["kill_rc"] = b.ssh("sudo -n kill -9 %s" % pid, timeout=8)[0]
                time.sleep(0.1)
            while "t_kill" in rec and now() - t_off < 90:
                if not b.ping():
                    rec["t_ping_fail"], down = now(), True
                    break
                time.sleep(0.5)
            rec["t_down_s"] = round(rec["t_ping_fail"] - t_off, 2) if down else None
            if down:
                time.sleep(15)
        finally:
            if m:
                m.stop()
            rec["t_on"] = t_on = b.outlet_on_verified()
        _, c = b.wait_ssh(t_on, limit=180, exclude_boot=rec["boot_id_before"] if down else None)
        if c:
            rec["contact"] = c
            rec["boot_id"] = c["boot_id"]
        restore(b, rec)
        if down:
            rec["journal_prev"] = prev_journal(b, 60, ("continuing halt", "HALT", "halt step"))
        if "t_kill" not in rec:
            fail(rec, "state never became halting within 60 s")
        elif not down:
            fail(rec, "Pi still up 90 s after the off command")
        elif rec.get("recovered_by_cycle") or not c or c["boot_id"] == rec["boot_id_before"]:
            fail(rec, "did not come back by itself with a new boot id")
        elif not any("continuing halt" in ln for ln in rec["journal_prev"] or []):
            fail(rec, "no 'continuing halt' line in the previous boot journal")
    return run_test(b, rec, body)


def main(argv=None):
    ap = argparse.ArgumentParser(description="X1202 watcher bench tests")
    ap.add_argument("--pi", required=True, help="ssh target, e.g. pi@host")
    ap.add_argument("--outlet-url", required=True, help="device base URL, as for x1202_cycle_test.py")
    ap.add_argument("--results", default="x1202_watcher_results.jsonl")
    ap.add_argument("cmd", choices=("status", "short", "long", "window", "hang", "resume"))
    ap.add_argument("--reps", type=int, default=None)
    ap.add_argument("--off-s", type=float, default=None, help="short: 3, long: 60")
    ap.add_argument("--offs", default="8,12,16,20", help="window: off times, s")
    ap.add_argument("--gap-s", type=float, default=None, help="between repeats (short 30, long 60, window 60)")
    a = ap.parse_args(argv)
    b = Bench(a)
    gap = a.gap_s if a.gap_s is not None else {"short": 30, "long": 60}.get(a.cmd, 60)
    try:
        if a.cmd == "status":
            cmd_status(b, a)
        elif a.cmd == "short":
            for i in range(a.reps or 5):
                short(b, a, i + 1)
                time.sleep(gap)
        elif a.cmd == "long":
            for i in range(a.reps or 3):
                long_(b, a, i + 1)
                time.sleep(gap)
        elif a.cmd == "window":
            for n in [float(x) for x in a.offs.split(",")]:
                for i in range(a.reps or 1):
                    window(b, a, i + 1, n)
                    time.sleep(gap)
        elif a.cmd == "hang":
            hang(b, a)
        elif a.cmd == "resume":
            resume(b, a)
    except PiDown as e:
        print("STOP: %s (outlet left ON)" % e, flush=True)
        return 2
    finally:
        try:
            if b.outlet_state() != "on":
                b.outlet_on_verified()
        except Exception as e:  # noqa: BLE001
            print("WARNING: final outlet check failed: %s" % e, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
