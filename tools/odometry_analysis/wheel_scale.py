"""Wheel-speed (VESC eRPM -> m/s, config kinematics) vs GPS over 3-epoch (~2 s) straight, steady windows.
Steady: wheel speed change across the window < 0.15 m/s. Straight: |gyro dh| < 3 deg, both eRPM > 1500."""
import json, gzip, sys, math, statistics as st
R = 6371000.0
def wrap(a): return (a + 180) % 360 - 180
res = {}; asym = []
for n in sys.argv[1:]:
    rows = [r for r in (json.loads(l) for l in gzip.open(f"ext/{n}.log.jsonl.gz","rt")) if "m" in r]
    ep = [r for r in rows if r["ng"] and r["lat"] is not None]
    for a, c in zip(ep, ep[2:]):
        if not (1.7 < c["ts"] - a["ts"] < 2.6) or a["fx"] != c["fx"] or a["fx"] < 2: continue
        mid = [r for r in rows if a["ts"] < r["ts"] <= c["ts"] and r["h"] is not None and r["v"] is not None]
        if len(mid) < 8 or not all(r["lr"] and r["rr"] and r["lr"] > 1500 and r["rr"] > 1500 for r in mid): continue
        if abs(wrap(mid[-1]["h"] - mid[0]["h"])) > 3 or abs(mid[-1]["v"] - mid[0]["v"]) > 0.15: continue
        dx = math.radians(c["lon"] - a["lon"]) * R * math.cos(math.radians(a["lat"])); dy = math.radians(c["lat"] - a["lat"]) * R
        g = math.hypot(dx, dy) / (c["ts"] - a["ts"]); w = st.mean(r["v"] for r in mid)
        if g < 0.3: continue
        key = (a["fx"], "lt1.0" if w < 1.0 else "ge1.0")
        res.setdefault(key, []).append(w / g)
        sogs = [r["sog"] for r in (a, c) if r["sog"]]
        res.setdefault((a["fx"], "sog"), []).append(w / st.mean(sogs) if sogs else None)
        asym.append(st.mean((r["lr"] - r["rr"]) / ((r["lr"] + r["rr"]) / 2) for r in mid))
for k in sorted(res, key=str):
    v = sorted(x for x in res[k] if x)
    print(k, "n", len(v), "median", round(st.median(v), 3), "p25", round(v[len(v)//4], 3), "p75", round(v[3*len(v)//4], 3))
asym.sort(); print("L-R eRPM asymmetry on straight steady windows: median", round(st.median(asym)*100, 2), "%  p25", round(asym[len(asym)//4]*100,2), "p75", round(asym[3*len(asym)//4]*100,2), "n", len(asym))
