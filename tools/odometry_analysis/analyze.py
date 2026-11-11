"""Local analysis of extracted rows (ext/*.jsonl.gz from extract.py). Usage: analyze.py FILE..."""
import json, gzip, sys, math, statistics as st, datetime as dt
R = 6371000.0
def wrap(a): return (a + 180) % 360 - 180
def enu(lat, lon, lat0, lon0):
    return (math.radians(lon - lon0) * R * math.cos(math.radians(lat0)), math.radians(lat - lat0) * R)
def utc(s):
    try: return dt.datetime.strptime(s, "%Y-%m-%dT%H:%M:%SZ").timestamp()
    except Exception: return None
def pct(xs, p):
    xs = sorted(xs); return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None
def med(xs): return round(st.median(xs), 3) if xs else None
for path in sys.argv[1:]:
    rows = [json.loads(l) for l in gzip.open(path, "rt")]
    rows = [r for r in rows if "ts" in r and "m" in r]
    if not rows: continue
    name = path.split("/")[-1].replace(".log.jsonl.gz", "")
    moving = lambda r: (r["lr"] is not None and abs(r["lr"]) > 100) or (r["rr"] is not None and abs(r["rr"]) > 100)
    # wheel distance + time moving
    wd = 0.0; tmov = 0.0
    for a, b in zip(rows, rows[1:]):
        d = b["ts"] - a["ts"]
        if 0 < d < 0.3 and moving(b) and b["v"] is not None:
            wd += abs(b["v"]) * d; tmov += d
    # GPS epochs
    ep = [r for r in rows if r["ng"] and r["lat"] is not None]
    fixes = {}
    for r in ep: fixes[r["fx"]] = fixes.get(r["fx"], 0) + 1
    fixes_mov = {}
    for r in ep:
        if moving(r): fixes_mov[r["fx"]] = fixes_mov.get(r["fx"], 0) + 1
    lat0, lon0 = (ep[0]["lat"], ep[0]["lon"]) if ep else (0, 0)
    gd = 0.0; segs = []; jumps = []
    idx = {id(r): i for i, r in enumerate(rows)}
    for a, b in zip(ep, ep[1:]):
        ua, ub = utc(a["utc"]), utc(b["utc"])
        if ua is None or ub is None: continue
        ddt = ub - ua
        if not (0.5 <= ddt <= 1.5) or a["fx"] < 1 or b["fx"] < 1: continue
        xa, ya = enu(a["lat"], a["lon"], lat0, lon0); xb, yb = enu(b["lat"], b["lon"], lat0, lon0)
        dd = math.hypot(xb - xa, yb - ya)
        ia, ib = idx[id(a)], idx[id(b)]
        tk = [r for r in rows[ia + 1:ib + 1] if r["v"] is not None]
        if not tk: continue
        vw = st.mean(abs(r["v"]) for r in tk)
        if moving(b): gd += dd
        wdist = vw * ddt
        if a["fx"] != b["fx"] or dd > 3.0:
            jumps.append((b["ts"], a["fx"], b["fx"], round(dd, 2), round(wdist, 2), b.get("da")))
        hs = [r["h"] for r in tk if r["h"] is not None]
        dh = abs(wrap(hs[-1] - hs[0])) if len(hs) > 1 else 99
        straight = dh < 3.0 and all(r["lr"] and r["rr"] and r["lr"] > 0 and r["rr"] > 0 for r in tk)
        if straight and vw > 0.5 and a["fx"] == b["fx"] and len(tk) >= 5:
            segs.append(dict(fx=b["fx"], gps=dd / ddt, wheel=vw, sog=b["sog"], lr=st.mean(r["lr"] for r in tk), rr=st.mean(r["rr"] for r in tk), ts=b["ts"]))
    # heading vs COG
    hc = []
    for i, r in enumerate(rows):
        if not (r["ng"] and r["cog"] is not None and r["sog"] and r["sog"] > 0.5 and r["h"] is not None and r["lr"] and r["rr"] and r["lr"] > 300 and r["rr"] > 300):
            continue
        # heading rate over the previous ~1 s
        prev = [q for q in rows[max(0, i - 15):i] if q["h"] is not None and r["ts"] - q["ts"] <= 1.2]
        if not prev: continue
        rate = wrap(r["h"] - prev[0]["h"]) / max(r["ts"] - prev[0]["ts"], 0.1)
        if abs(rate) > 4: continue
        hc.append((r["ts"], wrap(r["cog"] - r["h"]), r["fx"], wrap(r["cog"] - r["ch"]) if r["ch"] is not None else None, r["lk"]))
    modes = {}
    for r in rows: modes[r["m"]] = modes.get(r["m"], 0) + 1
    fm = [r for r in rows if r["m"] == "FOLLOW_ME"]
    out = {"t0": dt.datetime.fromtimestamp(rows[0]["ts"]).strftime("%m-%d %H:%M"), "t1": dt.datetime.fromtimestamp(rows[-1]["ts"]).strftime("%H:%M"),
           "moving_s": round(tmov), "wheel_m": round(wd), "gps_m_moving": round(gd), "fix_epochs": fixes, "fix_epochs_moving": fixes_mov,
           "modes_logged_rows": modes, "pm": sorted({r["pm"] for r in fm}, key=str), "os": sorted({r["os"] for r in fm}, key=str),
           "locked_rows": sum(1 for r in rows if r["lk"]), "lock_src": sorted({r["lsrc"] for r in rows}, key=str)}
    if segs:
        by = {}
        for s in segs: by.setdefault(s["fx"], []).append(s)
        out["wheel_over_gps"] = {f: dict(n=len(v), med=med([s["wheel"] / s["gps"] for s in v if s["gps"] > 0.2]),
                                       p10=pct([round(s["wheel"] / s["gps"],3) for s in v if s["gps"] > 0.2], .1),
                                       p90=pct([round(s["wheel"] / s["gps"],3) for s in v if s["gps"] > 0.2], .9),
                                       wheel_over_sog=med([s["wheel"] / s["sog"] for s in v if s["sog"] and s["sog"] > 0.2]),
                                       lr_asym=med([(s["lr"] - s["rr"]) / ((s["lr"] + s["rr"]) / 2) for s in v]),
                                       slip_gt1p3=sum(1 for s in v if s["gps"] > 0.2 and s["wheel"] / s["gps"] > 1.3)) for f, v in by.items()}
    if hc:
        d = [x[1] for x in hc]; m0 = st.median(d)
        dd = [m0 + wrap(x - m0) for x in d]; ts = [x[0] for x in hc]
        slope = None
        if len(hc) > 5 and ts[-1] - ts[0] > 60:
            tm = st.mean(ts); dm = st.mean(dd)
            slope = sum((t - tm) * (y - dm) for t, y in zip(ts, dd)) / sum((t - tm) ** 2 for t in ts) * 60
        out["cog_minus_gyro"] = dict(n=len(hc), med=round(m0, 1), p10=round(pct(dd, .1), 1), p90=round(pct(dd, .9), 1),
                                     drift_deg_per_min=round(slope, 2) if slope is not None else None, span_min=round((ts[-1] - ts[0]) / 60, 1),
                                     fix=sorted({x[2] for x in hc}))
    if fm:
        res = [wrap(r["oth"] - (90 - r["h"])) for r in fm if r["oth"] is not None and r["h"] is not None]
        oxs = [(r["ts"], r["ox"], r["oy"]) for r in fm if r["ox"] is not None]
        ch = [(a, b) for a, b in zip(oxs, oxs[1:]) if (a[1], a[2]) != (b[1], b[2])]
        steps = [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in ch]
        gaps = [b[0] - a[0] for a, b in ch]
        out["gps_odom"] = dict(rows=len(fm), oth_minus_90_minus_h_med=med(res), oth_res_maxabs=round(max(map(abs, res)), 2) if res else None,
                               pos_updates=len(ch), upd_gap_med_s=med(gaps), upd_gap_max_s=round(max(gaps), 2) if gaps else None,
                               step_med_m=med(steps), step_max_m=round(max(steps), 2) if steps else None,
                               gps_speed_med=med([r["gsp"] for r in fm if r["gsp"] is not None]),
                               wheel_v_med=med([r["v"] for r in fm if r["v"] is not None]),
                               trail_len_max=max([r["tl"] or 0 for r in fm]))
    out["fix_change_or_big_jumps"] = jumps[:12]
    out["diff_age_max"] = max([r["da"] or 0 for r in ep]) if ep else None
    print("=" * 10, name); print(json.dumps(out, default=str))
