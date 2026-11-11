"""Compare trail-odometry candidates on the reduced extracts of the real drives.

Usage: python3 compare.py EXTDIR name1 [name2 ...]

Frames: gyro frame = x along boot heading 0, y along boot heading 90 (CW),
heading h (deg, CW-positive).  GPS frame = (N, E) metres, compass course.
A DR displacement in the gyro frame maps to (N, E) by a rotation of phi
(true = h + phi), fitted per drive.
"""
import gzip, json, math, sys, statistics as st, bisect

R = 6371000.0
NEUTRAL = 126


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def load(extdir, name):
    rows = [json.loads(l) for l in gzip.open(f"{extdir}/{name}.log.jsonl.gz", "rt")]
    rows = [r for r in rows if "m" in r and r.get("ts") is not None]
    rows.sort(key=lambda r: r["ts"])
    return rows


def byte_speed(mL, mR):
    if mL is None or mR is None:
        return 0.0
    if (mL > NEUTRAL and mR < NEUTRAL) or (mL < NEUTRAL and mR > NEUTRAL):
        return 0.0
    return (max(0, mL - NEUTRAL) + max(0, mR - NEUTRAL)) / 2.0


def integrate(rows, speed_fn):
    """Cumulative DR position (gyro frame, unit scale) at each row, code-like (MAX_DT 0.2)."""
    xs, ys, ts, dist = [0.0], [0.0], [rows[0]["ts"]], [0.0]
    x = y = d = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b["ts"] - a["ts"]
        h = b["h"]
        if 0 < dt <= 0.2 and h is not None:
            v = speed_fn(b)
            th = math.radians(h)
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            d += abs(v) * dt
        xs.append(x); ys.append(y); ts.append(b["ts"]); dist.append(d)
    return ts, xs, ys, dist


def interp(ts, arr, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return arr[0]
    if i >= len(ts):
        return arr[-1]
    t0, t1 = ts[i - 1], ts[i]
    f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return arr[i - 1] + f * (arr[i] - arr[i - 1])


def epochs(rows):
    ep = []
    lat0 = lon0 = None
    for r in rows:
        if not r.get("ng") or r.get("lat") is None or (r.get("fx") or 0) < 1:
            continue
        if lat0 is None:
            lat0, lon0 = r["lat"], r["lon"]
        n = math.radians(r["lat"] - lat0) * R
        e = math.radians(r["lon"] - lon0) * R * math.cos(math.radians(lat0))
        t = r["ts"] - (r.get("ga") or 0.0)
        ep.append({"t": t, "n": n, "e": e, "fx": r["fx"], "cog": r.get("cog"), "sog": r.get("sog")})
    return ep


def fit_rot_scale(pairs):
    """pairs: list of ((dx, dy) gyro-frame DR, (dn, de) GPS). Fit CW rotation phi and scale s.
    True compass = h + phi, so (n, e) = s * (x cos phi - y sin phi, x sin phi + y cos phi)."""
    sc = ss = nd = 0.0
    for (x, y), (n, e) in pairs:
        sc += x * n + y * e
        ss += x * e - y * n
        nd += x * x + y * y
    phi = math.atan2(ss, sc)
    s = math.hypot(sc, ss) / nd if nd > 0 else 1.0
    return phi, s


def rot(x, y, phi):
    return x * math.cos(phi) - y * math.sin(phi), x * math.sin(phi) + y * math.cos(phi)


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")


def main():
    extdir = sys.argv[1]
    names = sys.argv[2:]
    speed_fns = {
        "byte": lambda r: byte_speed(r.get("mL"), r.get("mR")),
        "vesc": lambda r: (r.get("v") or 0.0),
    }
    data = {}
    for n in names:
        rows = [r for r in load(extdir, n)]
        ep = epochs(rows)
        if len(ep) < 40:
            continue
        data[n] = (rows, ep, {k: integrate(rows, f) for k, f in speed_fns.items()})

    # 1) global scale fit on 10 s windows, same fix both ends, moving
    for src in speed_fns:
        pairs_all = []
        per = {}
        for n, (rows, ep, dr) in data.items():
            ts, xs, ys, dd = dr[src]
            pairs = []
            for i, a in enumerate(ep):
                for b in ep[i + 1:]:
                    if b["t"] - a["t"] >= 10.0:
                        break
                else:
                    continue
                if abs(b["t"] - a["t"] - 10.0) > 1.5 or a["fx"] != b["fx"]:
                    continue
                dxx = interp(ts, xs, b["t"]) - interp(ts, xs, a["t"])
                dyy = interp(ts, ys, b["t"]) - interp(ts, ys, a["t"])
                if math.hypot(dxx, dyy) < 0.5 / (0.01 if src == "byte" else 1.0):
                    continue
                pairs.append(((dxx, dyy), (b["n"] - a["n"], b["e"] - a["e"])))
            if len(pairs) >= 5:
                per[n] = fit_rot_scale(pairs)
                pairs_all += pairs
        print(f"[{src}] per-drive fit (phi deg, scale):",
              {n: (round(math.degrees(p), 1), round(s, 5)) for n, (p, s) in per.items()})
        data_fit = per
        # Leave-one-out global scale for evaluation
        for tau in (3.0, 10.0, 30.0):
            errs, rel, gpsd = [], [], []
            jump_errs = []
            for n, (rows, ep, dr) in data.items():
                if n not in per:
                    continue
                phi = per[n][0]
                others = [per[m][1] for m in per if m != n]
                s = st.median(others) if others else per[n][1]
                ts, xs, ys, dd = dr[src]
                j = 0
                for i, a in enumerate(ep):
                    while j < len(ep) and ep[j]["t"] - a["t"] < tau - 0.5:
                        j += 1
                    if j >= len(ep):
                        break
                    b = ep[j]
                    if abs(b["t"] - a["t"] - tau) > 1.5:
                        continue
                    seg = [e["fx"] for e in ep[i:j + 1]]
                    dxx = interp(ts, xs, b["t"]) - interp(ts, xs, a["t"])
                    dyy = interp(ts, ys, b["t"]) - interp(ts, ys, a["t"])
                    pn, pe = rot(dxx * s, dyy * s, phi)
                    gn, ge = b["n"] - a["n"], b["e"] - a["e"]
                    gd = math.hypot(gn, ge)
                    path = (interp(ts, dd, b["t"]) - interp(ts, dd, a["t"])) * s
                    if path < 0.3 * tau:  # moving at least 0.3 m/s average
                        continue
                    err = math.hypot(pn - gn, pe - ge)
                    if len(set(seg)) == 1:
                        errs.append(err); rel.append(err / path); gpsd.append(gd)
                    else:
                        jump_errs.append(err)
            print(f"  tau {tau:4.0f}s clean n={len(errs):4d} err med {pct(errs, .5):.2f} m p90 {pct(errs, .9):.2f} m"
                  f"  rel med {100*pct(rel, .5):.1f}% p90 {100*pct(rel, .9):.1f}%  (gps disp med {pct(gpsd, .5):.1f} m)"
                  f" | windows with a fix change n={len(jump_errs)} err med {pct(jump_errs, .5):.2f} p90 {pct(jump_errs, .9):.2f}")
        print(f"  global scale (median of drives): {st.median([s for _, s in per.values()]):.5f}")

    # 2) GPS noise at short lag (1 s), DR as reference, and fix-change steps
    print("\n[gps] epoch-to-epoch residual vs vesc DR (1 s lag), by fix; fix-change steps")
    res = {}
    steps = []
    park_steps = []
    for n, (rows, ep, dr) in data.items():
        if n not in data_fit:
            pass
        ts, xs, ys, dd = dr["vesc"]
        # use vesc per-drive fit
        pairs = []
        for a, b in zip(ep, ep[1:]):
            dxx = interp(ts, xs, b["t"]) - interp(ts, xs, a["t"])
            dyy = interp(ts, ys, b["t"]) - interp(ts, ys, a["t"])
            pairs.append(((dxx, dyy), (b["n"] - a["n"], b["e"] - a["e"]), a, b,
                          interp(ts, dd, b["t"]) - interp(ts, dd, a["t"])))
        good = [(p[0], p[1]) for p in pairs if p[2]["fx"] == p[3]["fx"] and math.hypot(*p[0]) > 0.3]
        if len(good) < 10:
            continue
        phi, s = fit_rot_scale(good)
        for d, g, a, b, path in pairs:
            if not (0.6 < b["t"] - a["t"] < 1.4):
                continue
            pn, pe = rot(d[0] * s, d[1] * s, phi)
            err = math.hypot(pn - g[0], pe - g[1])
            if a["fx"] == b["fx"]:
                res.setdefault((a["fx"], "moving" if path > 0.2 else "parked"), []).append(err)
            else:
                steps.append((a["fx"], b["fx"], err, path))
    for k in sorted(res):
        v = res[k]
        print(f"  fix {k[0]} {k[1]:7s} n={len(v):5d} 1-s residual med {pct(v, .5):.2f} p90 {pct(v, .9):.2f} p99 {pct(v, .99):.2f} m")
    st_errs = [e for _, _, e, _ in steps]
    print(f"  fix-change epochs n={len(steps)} step error med {pct(st_errs, .5):.2f} p90 {pct(st_errs, .9):.2f} max {max(st_errs) if st_errs else 0:.2f} m")
    trans = {}
    for a, b, e, _ in steps:
        trans.setdefault(f"{a}->{b}", []).append(e)
    print("  by transition:", {k: (len(v), round(pct(v, .5), 2), round(max(v), 2)) for k, v in sorted(trans.items())})

    # 3) heading: COG - gyro offset on straight forward epochs, scatter and drift within 30 s
    print("\n[heading] receiver/position course minus gyro, straight forward, speed > 0.8 m/s")
    for n, (rows, ep, dr) in data.items():
        rts = [r["ts"] for r in rows]
        offs = []
        for a, b in zip(ep, ep[1:]):
            if not (0.6 < b["t"] - a["t"] < 1.4) or a["fx"] != b["fx"]:
                continue
            i0 = bisect.bisect_left(rts, a["t"]); i1 = bisect.bisect_right(rts, b["t"])
            mid = [r for r in rows[i0:i1] if r["h"] is not None]
            if len(mid) < 5 or not all((r.get("v") or 0) > 0.8 for r in mid):
                continue
            if abs(wrap(mid[-1]["h"] - mid[0]["h"])) > 4:
                continue
            dn, de = b["n"] - a["n"], b["e"] - a["e"]
            if math.hypot(dn, de) < 0.6:
                continue
            course = math.degrees(math.atan2(de, dn))
            offs.append((b["t"], wrap(course - mid[len(mid) // 2]["h"]), b["fx"]))
        if len(offs) < 5:
            print(f"  {n}: n={len(offs)} too few")
            continue
        med = st.median(o for _, o, _ in offs)
        dev = [wrap(o - med) for _, o, _ in offs]
        # within-30 s change: pair medians of first/last 10 s of each 30 s span
        drifts = []
        for t0, _, _ in offs:
            A = [o for t, o, _ in offs if t0 <= t < t0 + 10]
            B = [o for t, o, _ in offs if t0 + 20 <= t < t0 + 30]
            if len(A) >= 3 and len(B) >= 3:
                drifts.append(abs(wrap(st.median(B) - st.median(A))))
        fx = sorted(set(f for _, _, f in offs))
        print(f"  {n}: n={len(offs)} fix {fx} offset median {med:+.1f} deg, per-epoch scatter |dev| med {pct([abs(d) for d in dev], .5):.1f} p90 {pct([abs(d) for d in dev], .9):.1f} deg;"
              f" 30 s change (10 s medians) med {pct(drifts, .5):.1f} p90 {pct(drifts, .9):.1f} deg n={len(drifts)}")


if __name__ == "__main__":
    main()
