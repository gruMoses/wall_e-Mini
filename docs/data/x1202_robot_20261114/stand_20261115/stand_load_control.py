#!/usr/bin/env python3
"""Stand test load control, 2026-11-15. Runs on the Pi as a transient root unit. It reads the walle-power
status file only, and it starts or stops a CPU + NVMe load. It sends no command to the motors.
Phases: wait for input loss -> load on the cells (max 200 s) -> input back: load off, wait for a charge
burst -> load on during the burst -> load off. Repeats the burst part until the end time or /tmp/stand_stop."""
import json, os, signal, subprocess, sys, time
END = time.time() + float(sys.argv[1])
LOG = open("/tmp/stand_orch.log", "a")
def say(*a):
    LOG.write(json.dumps({"t": round(time.time(), 1), "msg": " ".join(str(x) for x in a)}) + "\n"); LOG.flush()
def st():
    try: return json.load(open("/run/walle-power/status.json"))
    except Exception: return {}
def temp():
    try: return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
    except Exception: return None
procs = []
def load_on(why):
    if procs: return
    for _ in range(4):
        procs.append(subprocess.Popen(["nice", "-n", "19", "sh", "-c", "while :; do :; done"], start_new_session=True))
    procs.append(subprocess.Popen(["nice", "-n", "19", "sh", "-c",
        "while :; do dd if=/dev/nvme0n1 of=/dev/null bs=4M count=1500 iflag=direct 2>/dev/null; done"], start_new_session=True))
    say("load ON:", why)
def load_off(why):
    if not procs: return
    for p in procs:
        try: os.killpg(p.pid, signal.SIGKILL)
        except Exception: pass
    for p in procs:
        try: p.wait(timeout=3)
        except Exception: pass
    procs.clear(); say("load OFF:", why)
def stop_asked(): return os.path.exists("/tmp/stand_stop") or time.time() > END
signal.signal(signal.SIGTERM, lambda *a: (load_off("SIGTERM"), sys.exit(0)))
try:
    say("start; waiting for input loss", st().get("vcell_v"), st().get("faults"))
    while not stop_asked() and st().get("input_good") is not False: time.sleep(0.5)
    if stop_asked(): raise SystemExit
    t_off = time.time(); say("input LOST; on the cells", st().get("vcell_v")); load_on("on the cells")
    hot = False
    while not stop_asked():
        s = st(); el = time.time() - t_off; tc = temp()
        if s.get("input_good") is True: break
        v = s.get("vcell_v"); e5 = s.get("ext5v_v")
        if el > 200: load_off("200 s on the cells")
        elif v is not None and v < 3.45: load_off("VCELL %.3f < 3.45" % v)
        elif e5 is not None and e5 < 4.70: load_off("EXT5V %.2f < 4.70" % e5)
        elif tc is not None and tc > 82: load_off("temp %.1f" % tc)
        time.sleep(0.5)
    s = st(); say("input BACK after %.0f s; loaded vcell %s ext5v %s temp %s" % (time.time() - t_off, s.get("vcell_v"), s.get("ext5v_v"), temp()))
    load_off("input back; rest until the burst")
    n = 0
    while not stop_asked():
        s = st()
        if s.get("charge") == "burst":
            n += 1; t_b = time.time(); say("burst %d START; vcell %s" % (n, s.get("vcell_v"))); load_on("burst %d" % n)
            while not stop_asked() and st().get("charge") == "burst":
                tc = temp()
                if tc is not None and tc > 83: load_off("temp %.1f in burst" % tc)
                time.sleep(0.5)
            s = st(); say("burst %d END after %.0f s; vcell %s ext5v %s temp %s faults %s" % (n, time.time() - t_b, s.get("vcell_v"), s.get("ext5v_v"), temp(), s.get("faults")))
            load_off("burst end")
        time.sleep(0.5)
    say("end; bursts seen:", n)
finally:
    load_off("exit")
