"""Gyro heading vs GPS course. Course from (a) receiver cog_deg, (b) positions over a 2-epoch
baseline. Straight forward motion only (both rpm > 1500, |gyro dh| < 4 deg over the baseline)."""
import json, gzip, sys, math, statistics as st, datetime as dt
R = 6371000.0
def wrap(a): return (a + 180) % 360 - 180
def T(ts): return dt.datetime.fromtimestamp(ts).strftime("%m-%d %H:%M")
allpts = []
for n in sys.argv[1:]:
    rows = [r for r in (json.loads(l) for l in gzip.open(f"ext/{n}.log.jsonl.gz","rt")) if "m" in r]
    ep = [r for r in rows if r["ng"] and r["lat"] is not None]
    cogstale = 0; cogn = 0; pts = []
    for a, b, c in zip(ep, ep[1:], ep[2:]):
        if c["ts"] - a["ts"] > 2.6 or a["fx"] not in (4, 5) or c["fx"] not in (4, 5): continue
        mid = [r for r in rows if a["ts"] <= r["ts"] <= c["ts"] and r["h"] is not None]
        if not mid or not all(r["lr"] and r["rr"] and r["lr"] > 1500 and r["rr"] > 1500 for r in mid): continue
        if abs(wrap(mid[-1]["h"] - mid[0]["h"])) > 4: continue
        dx = math.radians(c["lon"] - a["lon"]) * R * math.cos(math.radians(a["lat"])); dy = math.radians(c["lat"] - a["lat"]) * R
        if math.hypot(dx, dy) < 1.0: continue
        course = math.degrees(math.atan2(dx, dy)) % 360
        hm = mid[len(mid)//2]["h"]
        cogn += 1
        if b["cog"] == a["cog"]: cogstale += 1
        pts.append((b["ts"], wrap(course - hm), wrap(b["cog"] - hm) if b["cog"] is not None else None, wrap(course - b["cog"]) if b["cog"] is not None else None))
    if not pts: print(n, "no straight RTK-float/fixed segments"); continue
    d = [p[1] for p in pts]; m0 = st.median(d)
    q = [p[3] for p in pts if p[3] is not None]
    print(f"{n} {T(pts[0][0])}: n={len(pts)} course-gyro median {m0:+.1f}  IQR {sorted(d)[len(d)//4]:+.1f}..{sorted(d)[3*len(d)//4]:+.1f};  receiver cog-gyro median {st.median([p[2] for p in pts if p[2] is not None]):+.1f};  pos-course minus cog median {st.median(q):+.1f};  cog unchanged vs previous epoch {cogstale}/{cogn}")
    allpts += [(p[0], p[1]) for p in pts]
# drift across the 7d40084 process
if allpts:
    allpts.sort(); base = allpts[0][0]
    by = {}
    for t, v in allpts: by.setdefault(T(t), []).append(v)
    print("per-session medians:", {k: round(st.median(v), 1) for k, v in by.items()})
