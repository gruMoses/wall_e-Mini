import json, subprocess, sys, time
end = time.time() + float(sys.argv[1]); out = open(sys.argv[2], "w"); n = 0
def run(c):
    try: return subprocess.run(c, capture_output=True, text=True, timeout=2).stdout
    except Exception: return ""
while time.time() < end:
    t = time.time(); n += 1
    p6 = run(["pinctrl", "get", "6"])
    r = {"t": round(t, 2), "g6": "hi" if "| hi" in p6 else "lo" if "| lo" in p6 else "?"}
    if n % 4 == 1:
        e = run(["vcgencmd", "pmic_read_adc", "EXT5V_V"])
        try: r["ext5v"] = float(e.split("=")[1].strip().rstrip("V"))
        except Exception: pass
        r["thr"] = run(["vcgencmd", "get_throttled"]).strip().split("=")[-1]
        try: r["temp"] = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
        except Exception: pass
        try:
            s = json.load(open("/run/walle-power/status.json"))
            r.update(vcell=s.get("vcell_v"), charge=s.get("charge"), good=s.get("input_good"), state=s.get("state"))
        except Exception: pass
    out.write(json.dumps(r) + "\n"); out.flush()
    time.sleep(max(0.0, 0.25 - (time.time() - t)))
