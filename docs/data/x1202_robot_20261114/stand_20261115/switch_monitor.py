#!/usr/bin/env python3
"""Robot switch checks, 2026-11-15: watch the Pi from the Mac. Read-only. Modes:
cycle: wait for the Pi to go down and come back with a new boot id and the web page up; print the times.
blip:  wait for a short input loss with no halt in the journal of this boot; print the lines."""
import json, subprocess, sys, time, urllib.request
PI = "pi@192.168.86.54"; MODE = sys.argv[1]; CAP = float(sys.argv[2]); LOG = open(sys.argv[3], "a")
def say(*a):
    line = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a); print(line, flush=True); LOG.write(line + "\n"); LOG.flush()
def ssh(cmd, timeout=7):
    try:
        r = subprocess.run(["ssh", "-o", "ConnectTimeout=4", "-o", "BatchMode=yes", PI, cmd], capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout
    except Exception: return 255, ""
def web():
    try: return urllib.request.urlopen("http://192.168.86.54:8080/", timeout=3).status == 200
    except Exception: return False
def probe():
    rc, o = ssh("cat /proc/sys/kernel/random/boot_id; cut -d' ' -f1 /proc/uptime; systemctl is-active wall-e walle-power | tr '\\n' ' '")
    if rc != 0 or not o.strip(): return None
    ls = o.strip().splitlines()
    return {"boot": ls[0][:8], "up": float(ls[1]), "svc": ls[2] if len(ls) > 2 else ""}
t0 = time.time(); p = probe()
if not p: say(MODE, "Pi not reachable at start"); sys.exit(2)
boot0 = p["boot"]; say(MODE, "start: boot", boot0, "uptime %.0f s" % p["up"], p["svc"])
if MODE == "cycle":
    last_up = time.time(); down = None; back = None
    while time.time() - t0 < CAP:
        p = probe(); now = time.time()
        if p and p["boot"] == boot0 and down is None: last_up = now
        elif p is None and down is None: down = now; say("Pi stopped answering (last answer %s)" % time.strftime("%H:%M:%S", time.localtime(last_up)))
        elif p and p["boot"] != boot0:
            if back is None: back = now; say("Pi answers again: boot", p["boot"], "uptime %.1f s" % p["up"], p["svc"]); up_at_back = p["up"]
            if web():
                say("web page up; Pi uptime now %.1f s" % probe()["up"]); break
        time.sleep(2)
    else: say("cap reached; down=%s back=%s" % (down, back))
    if down and back: say("down for %.0f s (Mac clock, from the first missed answer)" % (back - down))
    rc, o = ssh("tail -n 5 /var/lib/walle-power/events.jsonl; echo ---; cat /run/walle-power/status.json; echo; echo ---; systemctl show wall-e -p ActiveEnterTimestampMonotonic -p NRestarts; systemd-analyze 2>/dev/null | head -1; echo ---; sudo journalctl -u walle-power -b -1 --no-pager -o short-iso 2>/dev/null | grep -v FAULT | tail -8 | cut -c1-150; echo ---; sudo journalctl -u wall-e -b -1 --no-pager -o short-iso -a 2>/dev/null | grep -i -E 'emergency|shutdown|e-stop|estop|ch5' | tail -5 | cut -c1-200", timeout=25)
    print(o, flush=True); LOG.write(o + "\n")
else:
    while time.time() - t0 < CAP:
        rc, o = ssh("sudo journalctl -u walle-power -b --no-pager -o short-iso | grep -e 'good:' -e 'state:' -e 'halt' | tail -8 | cut -c1-140; cat /proc/sys/kernel/random/boot_id")
        if rc == 0 and "good: False -> True" in o:
            time.sleep(9); rc, o = ssh("sudo journalctl -u walle-power -b --no-pager -o short-iso | grep -e 'good:' -e 'state:' -e 'halt' | tail -8 | cut -c1-140; cat /proc/sys/kernel/random/boot_id; cat /run/walle-power/status.json; echo; systemctl is-active wall-e walle-power; tail -n 2 /var/lib/walle-power/events.jsonl")
            say("short loss seen; same boot:", boot0 in o); print(o, flush=True); LOG.write(o + "\n"); break
        if rc != 0: say("Pi not answering (a halt?)")
        time.sleep(3)
    else: say("cap reached, no short loss seen")
