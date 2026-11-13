#!/usr/bin/env python3
"""Power-cycle bench harness for the Geekworm X1202 (runs on the Mac).

Drives the mains outlet that feeds the X1202 (a home-dashboard device URL,
commands `on` and `off` only) and the Pi over ssh. The Pi must run
x1202_bench_logger.py (unit x1202-bench-logger) from /home/pi/x1202_bench/.

Each test step appends one JSON line to --results and prints one summary line.
Every step ends with the outlet ON (in a finally) and the switch state verified
through the API.

Safety rules enforced here:
  - Before each outlet-off step the rested VCELL (GPIO 16 high, >= 60 s since
    the last load or GPIO 16 change) must be >= 3.60 V (T6: 3.75 V). If it is
    lower: charge (GPIO 16 low) until VCELL >= 4.15 V for 3 min, disable,
    rest 2 min, re-check.
  - On cells: abort (stop the load, outlet on) if VCELL < 3.35 V,
    EXT5V < 4.95 V, or get_throttled != 0x0.
  - If the Pi does not answer ssh within 5 min after an outlet-on: outlet off
    30 s, on, wait 5 more min, once. Still down: raise PiDown (stop testing,
    outlet left ON).

Subcommands: status, t0, t1, t2, t3, t4, t5, t6, t7, rest, halt-off, on-wait.
For a soak: `halt-off` (clean halt with the outlet off) and later `on-wait`
(outlet on and wait for ssh).
"""
from __future__ import annotations

import argparse
import json
import shlex
import statistics
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request

V_REST_MIN = 3.60
V_REST_MIN_LOAD = 3.75
V_ABORT = 3.35
EXT5V_ABORT = 4.95
CHARGE_TARGET = 4.15
CHARGE_HOLD_S = 180
SSH_WAIT_S = 300
BENCH = "/home/pi/x1202_bench"

SNAP_PY = r'''
import json, os, glob, time
d = "/home/pi/x1202_bench/logs"
bt = time.clock_gettime(time.CLOCK_BOOTTIME)
bid = open("/proc/sys/kernel/random/boot_id").read().strip()
fs = sorted(glob.glob(d + "/x1202_%s_*.jsonl" % bid[:8]), key=os.path.getmtime)
out = {"bt": bt, "boot_id": bid, "files": len(fs)}
if fs:
    f = fs[-1]
    size = os.path.getsize(f)
    with open(f, "rb") as fh:
        head = fh.readline()
        fh.seek(max(0, size - 3000000))
        data = fh.read().decode(errors="replace").splitlines()
    try:
        h = json.loads(head); out["header"] = {k: h.get(k) for k in ("uptime_s", "gpio16_found", "gpio6_found", "vcell", "bt")}
        last_change = h.get("bt", 0)
    except Exception:
        last_change = 0
    last_low = 0; recs = []
    for ln in data:
        try: r = json.loads(ln)
        except Exception: continue
        if "g6" not in r: continue
        recs.append(r)
        if r.get("g6") == 0: last_low = r["bt"]
        if r.get("event") in ("gpio16_change", "charge_set"): last_change = max(last_change, r["bt"])
    if recs:
        L = recs[-1]
        vs = [r["vcell"] for r in recs[-8:] if r.get("vcell") is not None]
        slow = [r for r in recs[-12:] if "ext5v" in r]
        out.update(last=L, age=bt - L["bt"], vcell_avg=(sum(vs) / len(vs)) if vs else None,
                   rested_s=bt - max(last_low, last_change), since_low_s=(bt - last_low) if last_low else None,
                   ext5v=slow[-1]["ext5v"] if slow else None, thr=slow[-1]["thr"] if slow else None,
                   temp=slow[-1]["temp"] if slow else None)
out["svc"] = os.popen("systemctl is-active x1202-bench-logger").read().strip()
out["uptime"] = float(open("/proc/uptime").read().split()[0])
print(json.dumps(out))
'''


class PiDown(Exception):
    pass


class Abort(Exception):
    pass


def now():
    return time.time()


class Bench:
    def __init__(self, a):
        self.pi = a.pi
        self.url = a.outlet_url.rstrip("/")
        self.results = a.results
        self.host = a.pi.split("@")[-1]
        self.dry = False
        self.level_log = []

    # ------------------------------------------------------------ outlet
    def outlet(self, cmd: str) -> float:
        assert cmd in ("on", "off")
        t = now()
        req = urllib.request.Request(self.url + "/command/" + cmd, method="POST")
        with urllib.request.urlopen(req, timeout=10) as r:
            r.read()
        return t

    def outlet_state(self):
        return self.outlet_attrs().get("switch")

    def outlet_attrs(self):
        """Attributes of the device as {name: value}. The dashboard returns a
        list of {name, currentValue}; a plain dict is accepted too."""
        base = self.url.split("/api/devices/")[0] + "/api/devices"
        hub, dev = [urllib.parse.unquote(x) for x in self.url.rstrip("/").split("/")[-2:]]
        with urllib.request.urlopen(base, timeout=10) as r:
            data = json.loads(r.read())
        for d in data:
            if str(d.get("id")) == dev and d.get("_hub", hub) == hub:
                at = d.get("attributes") or {}
                if isinstance(at, list):
                    return {x.get("name"): x.get("currentValue") for x in at}
                return at
        return {}

    def outlet_on_verified(self):
        """Outlet on, then verify switch == on through the API (retry 3x).
        Then check the dimmer level (see check_level)."""
        t = None
        for i in range(3):
            try:
                t = self.outlet("on")
            except Exception as e:  # noqa: BLE001
                print("  outlet on error: %s" % e, flush=True)
            for _ in range(10):
                time.sleep(0.5)
                try:
                    if self.outlet_state() == "on":
                        self.check_level(t)
                        return t
                except Exception:
                    pass
        raise RuntimeError("outlet did not verify ON")

    def check_level(self, t_on):
        """The outlet is a Z-Wave dimmer. A few seconds after each `on`, read
        `level`; if it is not 99, send setLevel 99 once (the only value this
        harness ever sends) and re-read. Every check is logged to
        self.level_log and appended to the results file."""
        time.sleep(max(0.0, 3.0 - (now() - t_on)))
        entry = {"test": "LEVEL", "t_on": t_on, "t_read": now()}
        try:
            entry["level_after_on"] = self.outlet_attrs().get("level")
            if entry["level_after_on"] != 99:
                req = urllib.request.Request(self.url + "/command/setLevel?value=99", method="POST")
                with urllib.request.urlopen(req, timeout=10) as r:
                    r.read()
                entry["setlevel_99_sent"] = now()
                time.sleep(3)
                entry["level_after_set"] = self.outlet_attrs().get("level")
        except Exception as e:  # noqa: BLE001
            entry["err"] = str(e)[:120]
        self.level_log.append(entry)
        try:
            with open(self.results, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except OSError:
            pass
        if entry.get("level_after_on") != 99:
            print("  dimmer level %s after on -> set 99 -> %s" % (entry.get("level_after_on"),
                                                                  entry.get("level_after_set")), flush=True)

    def outlet_off_verified(self):
        t = self.outlet("off")
        for _ in range(10):
            time.sleep(0.5)
            try:
                if self.outlet_state() == "off":
                    return t
            except Exception:
                pass
        return t

    # ------------------------------------------------------------ pi
    def ssh(self, cmd: str, timeout=15.0, connect=5):
        try:
            r = subprocess.run(["ssh", "-o", "ConnectTimeout=%d" % connect, "-o", "BatchMode=yes",
                                "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                                self.pi, cmd], capture_output=True, text=True, timeout=timeout)
            return r.returncode, r.stdout, r.stderr
        except subprocess.TimeoutExpired:
            return 124, "", "timeout"

    def ping(self) -> bool:
        r = subprocess.run(["ping", "-c", "1", "-W", "800", self.host], capture_output=True)
        return r.returncode == 0

    def snap(self):
        rc, out, err = self.ssh("python3 -c " + shlex.quote(SNAP_PY), timeout=15)
        if rc != 0:
            return None
        try:
            return json.loads(out.strip().splitlines()[-1])
        except Exception:
            return None

    def contact(self):
        """First-contact info: uptime + boot id, mapped to Mac time."""
        t = now()
        rc, out, _ = self.ssh("cat /proc/uptime /proc/sys/kernel/random/boot_id", timeout=10, connect=4)
        if rc != 0:
            return None
        lines = out.split()
        up = float(lines[0])
        return {"t_ssh": t, "uptime": up, "boot_id": lines[2], "boot_start_mac": t - up}

    def journal_warnings(self):
        rc, out, _ = self.ssh("journalctl -b -p warning --no-pager | grep -iE "
                              "'volt|power|i2c|nvme|ext4|fsck|throttl|brownout|pmic|filesystem' | "
                              "grep -v walle-ups | tail -5", timeout=20)
        return [ln[:200] for ln in out.splitlines()] if rc == 0 else None

    def wait_ssh(self, t_on, limit=SSH_WAIT_S, exclude_boot=None):
        """Return (first_ping, contact) or (first_ping, None). A contact with
        boot id exclude_boot (the boot that is going down) does not count."""
        first_ping = None
        while now() - t_on < limit:
            if first_ping is None and self.ping():
                first_ping = now()
            if first_ping is not None:
                c = self.contact()
                if c and c["boot_id"] != exclude_boot:
                    return first_ping, c
                if c:
                    first_ping = None   # still the old boot
            time.sleep(0.5)
        return first_ping, None

    def recover_on(self, t_on, rec):
        """Rule 4 around an outlet-on at t_on. Returns contact or raises PiDown."""
        ex = rec.get("boot_id_before")
        fp, c = self.wait_ssh(t_on, exclude_boot=ex)
        rec["first_ping"] = fp
        if c:
            rec["contact"] = c
            return c
        rec["rule4"] = "no ssh 5 min after on: outlet off 30 s, on"
        print("  RULE 4: no ssh within 5 min; cycling outlet once", flush=True)
        rec["rule4_off"] = self.outlet_off_verified()
        time.sleep(30)
        t2 = self.outlet_on_verified()
        rec["rule4_on"] = t2
        fp, c = self.wait_ssh(t2, exclude_boot=ex)
        rec["rule4_first_ping"] = fp
        if c:
            rec["contact"] = c
            rec["recovered_by_rule4"] = True
            return c
        rec["rule4_result"] = "still down; testing stopped, outlet left ON"
        raise PiDown("Pi down after rule-4 recovery")

    # ------------------------------------------------------------ charge
    def set_charge_leave(self, seconds):
        cmd = ("bid=$(cat /proc/sys/kernel/random/boot_id); up=$(cut -d' ' -f1 /proc/uptime); "
               "echo \"$bid $(echo \"$up + %d\" | bc)\" > %s/CHARGE_LEAVE.tmp && mv %s/CHARGE_LEAVE.tmp %s/CHARGE_LEAVE"
               % (seconds, BENCH, BENCH, BENCH))
        return self.ssh(cmd)[0] == 0

    def charge(self, enable: bool):
        if enable:
            if not self.set_charge_leave(7200):
                raise RuntimeError("cannot write CHARGE_LEAVE")
            time.sleep(0.5)
            return self.ssh("pinctrl set 16 op dl")[0] == 0
        ok = self.ssh("pinctrl set 16 op dh")[0] == 0
        self.ssh("rm -f %s/CHARGE_LEAVE" % BENCH)
        return ok

    def wait_rested(self, max_wait=300):
        """Wait until the logger says rested >= 60 s; return the snapshot."""
        t0 = now()
        while True:
            s = self.snap()
            if s and s.get("rested_s") is not None and s["rested_s"] >= 60 and s.get("last", {}).get("g16") == 1 \
                    and s.get("age", 99) < 5:
                return s
            if now() - t0 > max_wait:
                return s
            time.sleep(5)

    def ensure_rested_v(self, vmin, rec):
        """Rule 3. Returns the rested snapshot that passed."""
        for attempt in range(3):
            s = self.wait_rested()
            if s is None or s.get("vcell_avg") is None:
                raise RuntimeError("no logger snapshot for the rested check")
            v = s["vcell_avg"]
            rec.setdefault("rested_checks", []).append({"v": round(v, 4), "rested_s": round(s["rested_s"], 1)})
            if s.get("thr") not in ("0x0", None):
                rec.setdefault("notes", []).append("throttled %s before step" % s.get("thr"))
            if v >= vmin:
                return s
            print("  rested VCELL %.3f < %.2f: charging (rule 3)" % (v, vmin), flush=True)
            self.charge_until(CHARGE_TARGET, CHARGE_HOLD_S, rec)
            self.charge(False)
            time.sleep(120)
        raise RuntimeError("rested VCELL did not reach %.2f after 3 charges" % vmin)

    def charge_until(self, target, hold_s, rec, timeout=5400):
        t0 = now()
        self.charge(True)
        good_since = None
        try:
            while now() - t0 < timeout:
                s = self.snap()
                v = s.get("vcell_avg") if s else None
                if v is not None and v >= target:
                    good_since = good_since or now()
                    if now() - good_since >= hold_s:
                        rec.setdefault("charges", []).append({"s": round(now() - t0), "v": round(v, 3)})
                        return
                else:
                    good_since = None
                time.sleep(10)
            raise RuntimeError("charge did not reach %.2f V" % target)
        finally:
            self.charge(False)

    # ------------------------------------------------------------ monitor
    def start_monitor(self):
        """tail -F the newest logger file; parse lines in a thread."""
        cmd = ("f=$(ls -t %s/logs/x1202_$(cut -c1-8 /proc/sys/kernel/random/boot_id)_*.jsonl | head -1); "
               "exec stdbuf -oL tail -n0 -F \"$f\"" % BENCH)
        p = subprocess.Popen(["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
                              "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", self.pi, cmd],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        m = {"p": p, "lines": [], "lock": threading.Lock()}

        def rd():
            for ln in p.stdout:
                try:
                    r = json.loads(ln)
                except Exception:
                    continue
                r["_mac"] = now()
                with m["lock"]:
                    m["lines"].append(r)
        threading.Thread(target=rd, daemon=True).start()
        time.sleep(1.0)
        return m

    @staticmethod
    def stop_monitor(m):
        try:
            m["p"].terminate()
        except Exception:
            pass

    @staticmethod
    def lines_since(m, t):
        with m["lock"]:
            return [r for r in m["lines"] if r["_mac"] >= t]

    def check_abort(self, m, t_from):
        """Raise Abort when a rule-3 limit is crossed in lines after t_from."""
        for r in self.lines_since(m, t_from):
            v = r.get("vcell")
            if v is not None and v < V_ABORT:
                raise Abort("VCELL %.3f < %.2f" % (v, V_ABORT))
            e = r.get("ext5v")
            if e is not None and e < EXT5V_ABORT:
                raise Abort("EXT5V %.3f < %.2f" % (e, EXT5V_ABORT))
            th = r.get("thr")
            if th not in (None, "0x0"):
                raise Abort("throttled %s" % th)

    def watch(self, m, seconds, t_from, step=0.25):
        t_end = now() + seconds
        while now() < t_end:
            self.check_abort(m, t_from)
            time.sleep(step)

    @staticmethod
    def summarize(lines, t_off=None, t_on=None):
        out = {}
        vs = [r["vcell"] for r in lines if r.get("vcell") is not None]
        es = [r["ext5v"] for r in lines if r.get("ext5v") is not None]
        ts = [r["temp"] for r in lines if r.get("temp") is not None]
        th = sorted({r["thr"] for r in lines if r.get("thr")})
        if vs:
            out["vcell_min"], out["vcell_max"] = min(vs), max(vs)
        if es:
            out["ext5v_min"], out["ext5v_max"] = min(es), max(es)
        if ts:
            out["temp_max"] = max(ts)
        out["throttled"] = th
        if t_off is not None:
            fall = [r for r in lines if r["_mac"] >= t_off and r.get("g6") == 0]
            if fall:
                out["g6_fall_delay_s"] = round(fall[0]["wall"] - t_off, 3)
                out["g6_fall_delay_mac_s"] = round(fall[0]["_mac"] - t_off, 3)
        if t_on is not None:
            rise = [r for r in lines if r["_mac"] >= t_on and r.get("g6") == 1]
            if rise:
                out["g6_rise_delay_s"] = round(rise[0]["wall"] - t_on, 3)
                out["g6_rise_delay_mac_s"] = round(rise[0]["_mac"] - t_on, 3)
        kern = [r for r in lines if r.get("event") == "gpio6_kernel_edge"]
        out["kernel_edges"] = len(kern)
        return out

    # ------------------------------------------------------------ results
    def emit(self, rec):
        rec["t_end"] = now()
        with open(self.results, "a") as f:
            f.write(json.dumps(rec, default=str) + "\n")
        keys = ("test", "rep", "result", "reason", "boot_ms", "t_first_ping_s", "t_first_ssh_s")
        print("%s" % " ".join("%s=%s" % (k, rec[k]) for k in keys if k in rec), flush=True)

    def step(self, rec, body):
        """Run body(rec); always end with the outlet ON and verified."""
        rec.setdefault("result", "PASS")
        try:
            body(rec)
        except Abort as e:
            rec["result"] = "ABORT"
            rec["reason"] = str(e)
        except PiDown as e:
            rec["result"] = "FAIL"
            rec["reason"] = str(e)
            raise
        except Exception as e:  # noqa: BLE001
            rec["result"] = "ERROR"
            rec["reason"] = "%s: %s" % (type(e).__name__, e)
        finally:
            # Outlet ON first: an abort must restore the input before anything else.
            try:
                rec["final_on"] = self.outlet_on_verified()
                rec["final_switch"] = self.outlet_state()
            except Exception as e:  # noqa: BLE001
                rec["final_switch"] = "UNKNOWN: %s" % e
            if rec.get("test") == "T6":
                # [x] keeps pkill -f from matching its own shell.
                self.ssh("pkill -f '[x]1202load'; sudo -n pkill -f '[d]d if=/dev/nvme0n1'", timeout=8)
            if rec.get("test") == "T6":
                time.sleep(3)   # let GPIO 6 go high before the guard is live again
                self.ssh("rm -f %s/GUARD_INHIBIT" % BENCH, timeout=8)
            self.emit(rec)
        return rec

    # ------------------------------------------------------------ boot measure
    def halt_and_measure_down(self, rec, cmd="poweroff"):
        """Send a clean poweroff/reboot; record last ssh and first failed ping."""
        rc, _, _ = self.ssh("sudo -n systemd-run --on-active=1 --no-block /bin/systemctl %s" % cmd, timeout=10)
        rec["t_cmd"] = now()
        rec["cmd_rc"] = rc
        if rc != 0:
            raise RuntimeError("could not schedule %s (rc %s)" % (cmd, rc))
        rec["t_last_ssh"] = rec["t_cmd"]
        t0 = now()
        ssh_ok = True
        while now() - t0 < 45:
            if not self.ping():
                rec["t_ping_fail"] = now()
                return True
            if ssh_ok:
                ssh_ok = self.contact() is not None
                if ssh_ok:
                    rec["t_last_ssh"] = now()
            time.sleep(0.3)
        return False

    def post_boot(self, rec, c, t_ref):
        rec["boot_id"] = c["boot_id"]
        rec["t_first_ssh_s"] = round(c["t_ssh"] - t_ref, 2)
        if rec.get("first_ping"):
            rec["t_first_ping_s"] = round(rec["first_ping"] - t_ref, 2)
        rec["boot_start_after_ref_s"] = round(c["boot_start_mac"] - t_ref, 2)
        time.sleep(3)
        s = self.snap()
        if s:
            rec["header"] = s.get("header")
            rec["logger_svc"] = s.get("svc")
        rec["journal"] = self.journal_warnings()


# ---------------------------------------------------------------- tests
def t0(b: Bench, a):
    """Baseline: 10 min GPIO16 high, 3 min low (charging), 25 min high."""
    rec = {"test": "T0", "rep": 1, "t_start": now()}

    def body(rec):
        st = b.outlet_state()
        rec["switch_at_start"] = st
        if st != "on":
            b.outlet_on_verified()
        b.charge(False)
        rec["t_phase1"] = now()
        time.sleep(a.t0_high1)
        rec["t_charge_on"] = now()
        b.charge(True)
        time.sleep(a.t0_low)
        b.charge(False)
        rec["t_charge_off"] = now()
        time.sleep(a.t0_high2)
        rec["t_done"] = now()
        s = b.snap()
        rec["boot_id"] = s.get("boot_id") if s else None
    return b.step(rec, body)


def t1(b, a, rep):
    rec = {"test": "T1", "rep": rep}

    def body(rec):
        b.ensure_rested_v(V_REST_MIN, rec)
        m = b.start_monitor()
        try:
            t_off = b.outlet_off_verified()
            rec["t_off"] = t_off
            b.watch(m, a.t1_off_s - (now() - t_off), t_off)
            t_on = b.outlet_on_verified()
            rec["t_on"] = t_on
            time.sleep(8)
            rec.update(b.summarize(b.lines_since(m, t_off - 2), t_off, t_on))
            b.check_abort(m, t_off)
        finally:
            Bench.stop_monitor(m)
        if rec.get("g6_fall_delay_s") is None:
            rec["result"] = "FAIL"
            rec["reason"] = "GPIO 6 did not fall"
    return b.step(rec, body)


def full_cycle(b, a, rec, on_after_cmd_s=None):
    """outlet off; 10 s; poweroff; then outlet on (after ping fail + 30 s,
    or on_after_cmd_s after the command for the race test)."""
    b.ensure_rested_v(V_REST_MIN, rec)
    rec["boot_id_before"] = (b.contact() or {}).get("boot_id")
    m = b.start_monitor()
    try:
        return _full_cycle_inner(b, rec, m, on_after_cmd_s)
    finally:
        Bench.stop_monitor(m)


def _full_cycle_inner(b, rec, m, on_after_cmd_s):
    t_off = b.outlet_off_verified()
    rec["t_off"] = t_off
    b.watch(m, 10 - (now() - t_off), t_off)
    down = None
    if on_after_cmd_s is None:
        down = b.halt_and_measure_down(rec)
        rec.update({"on_cells": b.summarize(b.lines_since(m, t_off - 1), t_off)})
        if not down:
            raise RuntimeError("Pi still pings 45 s after poweroff")
        time.sleep(30)
    else:
        rc, _, _ = b.ssh("sudo -n systemd-run --on-active=1 --no-block /bin/systemctl poweroff", timeout=10)
        rec["t_cmd"] = now()
        rec["cmd_rc"] = rc
        if rc != 0:
            raise RuntimeError("could not schedule poweroff")
        wait = on_after_cmd_s - (now() - rec["t_cmd"])
        if wait > 0:
            time.sleep(wait)
        rec.update({"on_cells": b.summarize(b.lines_since(m, t_off - 1), t_off)})
    t_on = b.outlet_on_verified()
    rec["t_on"] = t_on
    if on_after_cmd_s is not None:
        # Did the Pi go down at all?
        t0_ = now()
        while now() - t0_ < 60:
            if not b.ping():
                rec["t_ping_fail"] = now()
                break
            time.sleep(0.3)
    c = b.recover_on(t_on, rec)
    b.post_boot(rec, c, t_on)
    if rec.get("boot_id_before") and c["boot_id"] == rec["boot_id_before"]:
        rec["result"] = "FAIL"
        rec["reason"] = "same boot id: Pi did not power off"
    if rec.get("recovered_by_rule4"):
        rec["result"] = "FAIL"
        rec["reason"] = "needed the rule-4 outlet cycle"


def t2(b, a, rep):
    rec = {"test": "T2", "rep": rep}
    return b.step(rec, lambda r: full_cycle(b, a, r))


def t3(b, a, rep, delay):
    rec = {"test": "T3", "rep": rep, "on_after_cmd_s": delay}
    return b.step(rec, lambda r: full_cycle(b, a, r, on_after_cmd_s=delay))


def t4(b, a, rep):
    rec = {"test": "T4", "rep": rep}

    def body(rec):
        if b.outlet_state() != "on":
            b.outlet_on_verified()
        rec["boot_id_before"] = (b.contact() or {}).get("boot_id")
        down = b.halt_and_measure_down(rec)
        if not down:
            raise RuntimeError("Pi still pings 45 s after poweroff")
        fp, c = b.wait_ssh(rec["t_ping_fail"], limit=180, exclude_boot=rec["boot_id_before"])
        if c and c["boot_id"] != rec["boot_id_before"]:
            rec["self_restart"] = True
            rec["first_ping"] = fp
            b.post_boot(rec, c, rec["t_cmd"])
            return
        rec["self_restart"] = False
        rec["t_off"] = b.outlet_off_verified()
        time.sleep(30)
        t_on = b.outlet_on_verified()
        rec["t_on"] = t_on
        c = b.recover_on(t_on, rec)
        b.post_boot(rec, c, t_on)
        if rec.get("recovered_by_rule4"):
            rec["result"] = "FAIL"
            rec["reason"] = "outlet cycle did not restart it; rule-4 second cycle did"
    return b.step(rec, body)


def t5(b, a, rep, on_cells):
    rec = {"test": "T5", "rep": rep, "on_cells": on_cells}

    def body(rec):
        m = None
        try:
            if on_cells:
                b.ensure_rested_v(V_REST_MIN, rec)
                m = b.start_monitor()
                rec["t_off"] = b.outlet_off_verified()
                b.watch(m, 3, rec["t_off"])
            rec["boot_id_before"] = (b.contact() or {}).get("boot_id")
            down = b.halt_and_measure_down(rec, cmd="reboot")
            if m:
                rec["on_cells_sum"] = b.summarize(b.lines_since(m, rec["t_off"] - 1), rec["t_off"])
        finally:
            if m:
                Bench.stop_monitor(m)
        if not down:
            rec["notes"] = ["ping never failed during reboot"]
        limit = 90 if on_cells else SSH_WAIT_S
        fp, c = b.wait_ssh(rec["t_cmd"], limit=limit, exclude_boot=rec["boot_id_before"])
        rec["first_ping"] = fp
        if on_cells:
            rec["t_on"] = b.outlet_on_verified()
            if not c:
                rec["result"] = "FAIL"
                rec["reason"] = "no ssh within 90 s of reboot on cells"
                c = b.recover_on(rec["t_on"], rec)
        elif not c:
            # Input on, 5 min passed: rule 4 (outlet cycle once) right away.
            rec["result"] = "FAIL"
            rec["reason"] = "reboot with input on did not come back in 5 min"
            c = b.recover_on(rec["t_cmd"], rec)
        b.post_boot(rec, c, rec["t_cmd"])
        if c["boot_id"] == rec.get("boot_id_before"):
            rec["result"] = "FAIL"
            rec["reason"] = "same boot id"
    return b.step(rec, body)


def t7(b, a, rep, off_s):
    """Minimum input-off time that restarts a Pi halted with the input present."""
    rec = {"test": "T7", "rep": rep, "off_s": off_s}

    def body(rec):
        if b.outlet_state() != "on":
            b.outlet_on_verified()
        rec["boot_id_before"] = (b.contact() or {}).get("boot_id")
        if not b.halt_and_measure_down(rec):
            raise RuntimeError("Pi still pings 45 s after poweroff")
        time.sleep(max(0.0, 60 - (now() - rec["t_ping_fail"])))
        rec["t_off"] = b.outlet_off_verified()
        time.sleep(max(0.0, off_s - (now() - rec["t_off"])))
        t_on = b.outlet_on_verified()
        rec["t_on"] = t_on
        rec["off_actual_s"] = round(t_on - rec["t_off"], 2)
        fp, c = b.wait_ssh(t_on, limit=180, exclude_boot=rec["boot_id_before"])
        rec["first_ping"] = fp
        if not c:
            rec["result"] = "FAIL"
            rec["reason"] = "no ssh 3 min after a %.0f s off" % off_s
            # Rule 4: off 30 s, on, once (recover_on waits 5 min from t_on first; skip that wait)
            c = b.recover_on(t_on - SSH_WAIT_S, rec)
        b.post_boot(rec, c, t_on)
    return b.step(rec, body)


LOAD_SH = r'''
n=%d; d=%d
for i in $(seq 1 $n); do timeout $d python3 -c 'while 1: pass' x1202load & done
if [ "%s" = dd ]; then
  end=$((SECONDS+d)); while [ $SECONDS -lt $end ]; do sudo -n timeout $((end-SECONDS+1)) dd if=/dev/nvme0n1 of=/dev/null bs=1M count=2000 iflag=direct 2>&1 | tail -1; done
fi
wait
'''


def t6(b, a):
    rec = {"test": "T6", "rep": 1}

    def body(rec):
        s = b.ensure_rested_v(V_REST_MIN_LOAD, rec)
        rec["rested_v"] = s["vcell_avg"]
        # Inhibit the bench guard for this boot for 420 s (load steps > 180 s on cells).
        b.ssh("bid=$(cat /proc/sys/kernel/random/boot_id); up=$(cut -d' ' -f1 /proc/uptime); "
              "echo \"$bid $(echo \"$up + 420\" | bc)\" > %s/GUARD_INHIBIT" % BENCH)
        m = b.start_monitor()
        steps = [("1core", 1, ""), ("2core", 2, ""), ("4core", 4, ""), ("4core+dd", 4, "dd")]
        rec["steps"] = []
        try:
            rec["t_off"] = b.outlet_off_verified()
            b.watch(m, 5, rec["t_off"])
            idle = b.summarize(b.lines_since(m, rec["t_off"] + 1))
            rec["steps"].append(dict(name="idle", **idle))
            for name, n, dd in steps:
                t_s = now()
                p = subprocess.Popen(["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", b.pi,
                                      "bash -c " + shlex.quote(LOAD_SH % (n, a.t6_step_s, dd))],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                try:
                    b.watch(m, a.t6_step_s + 1, rec["t_off"])
                except BaseException:
                    p.kill()          # abort: no wait here, step() puts the outlet on next
                    raise
                try:
                    out = p.communicate(timeout=10)[0]
                except subprocess.TimeoutExpired:
                    p.kill()
                    out = ""
                sm = b.summarize(b.lines_since(m, t_s + 10))  # settled part of the step
                sm_all = b.summarize(b.lines_since(m, t_s))
                sm["name"] = name
                sm["vcell_min_all"] = sm_all.get("vcell_min")
                sm["ext5v_min_all"] = sm_all.get("ext5v_min")
                if dd:
                    sm["dd"] = [ln for ln in out.splitlines() if "copied" in ln][-3:]
                rec["steps"].append(sm)
                print("  %s: %s" % (name, {k: sm.get(k) for k in ("vcell_min", "ext5v_min", "temp_max", "throttled")}),
                      flush=True)
        finally:
            Bench.stop_monitor(m)
            # Do NOT remove GUARD_INHIBIT here: the outlet is still off, GPIO 6
            # has been low > 180 s, and the guard would halt the Pi into the
            # outlet-on race (happened 2026-11-13 00:03:49). step() removes it
            # after the outlet is verified on.
    return b.step(rec, body)


def cmd_rest(b, a):
    rec = {"test": "REST", "rep": 1}

    def body(rec):
        b.charge(False)
        time.sleep(a.rest_s)
        s = b.snap()
        rec["vcell_avg"] = s.get("vcell_avg") if s else None
        rec["rested_s"] = s.get("rested_s") if s else None
    return b.step(rec, body)


def cmd_halt_off(b, a):
    """Soak start: rested check, outlet off, 10 s, clean poweroff. The outlet
    stays OFF afterwards (the only subcommand that leaves it off), and only if
    the Pi stopped answering ping."""
    rec = {"test": "HALT_OFF", "rep": 1}
    rec_ok = {"done": False}
    try:
        b.ensure_rested_v(V_REST_MIN, rec)
        rec["boot_id_before"] = (b.contact() or {}).get("boot_id")
        rec["t_off"] = b.outlet_off_verified()
        time.sleep(10)
        if not b.halt_and_measure_down(rec):
            raise RuntimeError("Pi still pings 45 s after poweroff")
        time.sleep(20)
        if b.ping():
            raise RuntimeError("Pi answers ping again after the halt")
        rec_ok["done"] = True
        rec["result"] = "PASS"
        rec["final_switch"] = b.outlet_state()
    except Exception as e:  # noqa: BLE001
        rec["result"] = "ERROR"
        rec["reason"] = "%s: %s" % (type(e).__name__, e)
    finally:
        if not rec_ok["done"]:
            rec["final_on"] = b.outlet_on_verified()
            rec["final_switch"] = b.outlet_state()
        b.emit(rec)


def cmd_on_wait(b, a):
    rec = {"test": "ON_WAIT", "rep": 1}

    def body(rec):
        t_on = b.outlet_on_verified()
        rec["t_on"] = t_on
        c = b.recover_on(t_on, rec)
        b.post_boot(rec, c, t_on)
    return b.step(rec, body)


def main(argv=None):
    ap = argparse.ArgumentParser(description="X1202 power-cycle bench harness")
    ap.add_argument("--pi", required=True, help="ssh target, e.g. pi@host")
    ap.add_argument("--outlet-url", required=True,
                    help="device base URL, e.g. http://dash:8888/api/devices/<hub>/<id>")
    ap.add_argument("--results", default="x1202_results.jsonl")
    ap.add_argument("cmd", choices=("status", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
                                    "rest", "halt-off", "on-wait"))
    ap.add_argument("--reps", type=int, default=None)
    ap.add_argument("--delays", default="2,10,30")
    ap.add_argument("--offs", default="3,10", help="T7 outlet-off times, s")
    ap.add_argument("--t0-high1", type=float, default=600)
    ap.add_argument("--t0-low", type=float, default=180)
    ap.add_argument("--t0-high2", type=float, default=1500)
    ap.add_argument("--t1-off-s", type=float, default=20)
    ap.add_argument("--t6-step-s", type=int, default=60)
    ap.add_argument("--rest-s", type=float, default=600)
    a = ap.parse_args(argv)
    b = Bench(a)
    try:
        if a.cmd == "status":
            print(json.dumps({"outlet": b.outlet_attrs(), "snap": b.snap()}, default=str))
        elif a.cmd == "t0":
            t0(b, a)
        elif a.cmd == "t1":
            for i in range(a.reps or 3):
                t1(b, a, i + 1)
        elif a.cmd == "t2":
            for i in range(a.reps or 10):
                t2(b, a, i + 1)
        elif a.cmd == "t3":
            for d in [float(x) for x in a.delays.split(",")]:
                for i in range(a.reps or 3):
                    t3(b, a, i + 1, d)
        elif a.cmd == "t4":
            for i in range(a.reps or 2):
                t4(b, a, i + 1)
        elif a.cmd == "t5":
            for i in range(a.reps if a.reps is not None else 3):
                t5(b, a, i + 1, False)
            t5(b, a, 1, True)
        elif a.cmd == "t6":
            t6(b, a)
        elif a.cmd == "t7":
            for n in [float(x) for x in a.offs.split(",")]:
                for i in range(a.reps or 2):
                    t7(b, a, i + 1, n)
        elif a.cmd == "rest":
            cmd_rest(b, a)
        elif a.cmd == "halt-off":
            cmd_halt_off(b, a)
        elif a.cmd == "on-wait":
            cmd_on_wait(b, a)
    except PiDown as e:
        print("STOP: %s (outlet left ON)" % e, flush=True)
        return 2
    finally:
        try:
            if a.cmd != "halt-off" and b.outlet_state() != "on":
                b.outlet_on_verified()
        except Exception as e:  # noqa: BLE001
            print("WARNING: final outlet check failed: %s" % e, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
