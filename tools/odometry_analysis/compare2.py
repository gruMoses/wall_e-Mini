"""Shape comparison on 30 s windows: rotation-free (a constant frame rotation is
irrelevant to a trail, because points and pose share the frame).

For each window (start every ~5 s, 30 s long, robot moving, single fix quality):
  fit one rotation (and free translation) of the DR track onto the GPS epochs,
  report RMS / max residual and the end-to-end displacement error.
Also simulates option (a)'s online heading-offset estimate (complementary
filter on COG - gyro, alpha 0.85, straight forward epochs only) and reports its
error against the window-fitted rotation, plus time-to-first-estimate.
And a GPS time-offset scan (does the DR fit improve if GPS time is shifted?).
"""
import math, sys, statistics as st, bisect
sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from compare import load, epochs, integrate, interp, byte_speed, wrap, pct

EXT = sys.argv[1]
NAMES = sys.argv[2:]
TAU = 30.0


def window_fit(P, G):
    """P, G: lists of (n, e). Rotation phi (CW: (x,y) -> (x c - y s, x s + y c)) + translation."""
    pn = sum(p[0] for p in P) / len(P); pe = sum(p[1] for p in P) / len(P)
    gn = sum(g[0] for g in G) / len(G); ge = sum(g[1] for g in G) / len(G)
    sc = ss = 0.0
    for (x, y), (n, e) in zip(P, G):
        x -= pn; y -= pe; n -= gn; e -= ge
        sc += x * n + y * e
        ss += x * e - y * n
    phi = math.atan2(ss, sc)
    c, s = math.cos(phi), math.sin(phi)
    res = []
    for (x, y), (n, e) in zip(P, G):
        x -= pn; y -= pe
        res.append(math.hypot(x * c - y * s - (n - gn), x * s + y * c - (e - ge)))
    # end-to-end displacement error
    dx, dy = P[-1][0] - P[0][0], P[-1][1] - P[0][1]
    rn, re = dx * c - dy * s, dx * s + dy * c
    end = math.hypot(rn - (G[-1][0] - G[0][0]), re - (G[-1][1] - G[0][1]))
    return phi, res, end


def run(scale_by_src, shift=0.0, verbose=True):
    out = {}
    for src, k in scale_by_src.items():
        fn = (lambda r: byte_speed(r.get("mL"), r.get("mR")) * k) if src == "byte" else (lambda r, k=k: (r.get("v") or 0.0) * k)
        rms_all, max_all, end_all, path_all, gd_all = [], [], [], [], []
        phis = {}
        for n in NAMES:
            rows = load(EXT, n)
            ep = epochs(rows)
            if len(ep) < 40:
                continue
            ts, xs, ys, dd = integrate(rows, fn)
            i = 0
            last_start = -1e9
            while i < len(ep):
                a = ep[i]
                if a["t"] - last_start < 5.0:
                    i += 1; continue
                W = [e for e in ep[i:] if e["t"] - a["t"] <= TAU]
                if W[-1]["t"] - a["t"] < TAU - 1.5 or len(set(e["fx"] for e in W)) != 1:
                    i += 1; continue
                path = interp(ts, dd, W[-1]["t"] + shift) - interp(ts, dd, a["t"] + shift)
                if path < 0.3 * TAU:
                    i += 1; continue
                P = [(interp(ts, xs, e["t"] + shift), interp(ts, ys, e["t"] + shift)) for e in W]
                G = [(e["n"], e["e"]) for e in W]
                phi, res, end = window_fit(P, G)
                rms_all.append(math.sqrt(sum(r * r for r in res) / len(res)))
                max_all.append(max(res)); end_all.append(end); path_all.append(path)
                gd_all.append(math.hypot(G[-1][0] - G[0][0], G[-1][1] - G[0][1]))
                phis.setdefault(n, []).append((a["t"], math.degrees(phi), W[0]["fx"]))
                last_start = a["t"]
                i += 1
        out[src] = (rms_all, max_all, end_all, path_all, gd_all, phis)
        if verbose:
            print(f"[{src} k={k}] shift {shift:+.2f}s windows n={len(rms_all)}  path med {pct(path_all, .5):.1f} m"
                  f" | RMS resid med {pct(rms_all, .5):.2f} p90 {pct(rms_all, .9):.2f} | max resid med {pct(max_all, .5):.2f} p90 {pct(max_all, .9):.2f}"
                  f" | end-to-end err med {pct(end_all, .5):.2f} p90 {pct(end_all, .9):.2f} m"
                  f" ({100*pct([e/p for e, p in zip(end_all, path_all)], .5):.1f}% / {100*pct([e/p for e, p in zip(end_all, path_all)], .9):.1f}% of path)")
    return out


if __name__ == "__main__":
    print("== time-offset scan (vesc, k=0.79) ==")
    for sh in (-0.6, -0.3, 0.0, 0.3, 0.6):
        o = run({"vesc": 0.79}, shift=sh, verbose=False)["vesc"]
        print(f"  shift {sh:+.1f}s RMS med {pct(o[0], .5):.3f}  end med {pct(o[2], .5):.3f}")
    print("== scale scan, vesc ==")
    for k in (0.70, 0.75, 0.79, 0.83, 0.87, 1.0):
        o = run({"vesc": k}, verbose=False)["vesc"]
        print(f"  k {k:.2f} RMS med {pct(o[0], .5):.3f} p90 {pct(o[0], .9):.3f} end med {pct(o[2], .5):.3f}")
    print("== scale scan, byte ==")
    for k in (0.0075, 0.0085, 0.0095, 0.0100, 0.0105):
        o = run({"byte": k}, verbose=False)["byte"]
        print(f"  k {k:.4f} RMS med {pct(o[0], .5):.3f} p90 {pct(o[0], .9):.3f} end med {pct(o[2], .5):.3f}")
    print("== main ==")
    out = run({"byte": 0.0100, "vesc": 0.79})
    out2 = run({"byte": 0.0080})
    # heading-offset drift between overlapping windows 30 s apart
    phis = out["vesc"][5]
    drifts = []
    for n, lst in phis.items():
        for t, p, fx in lst:
            for t2, p2, fx2 in lst:
                if 27 <= t2 - t <= 33:
                    drifts.append(abs(wrap(p2 - p)))
    print(f"fitted frame rotation change between windows 30 s apart: med {pct(drifts, .5):.1f} p90 {pct(drifts, .9):.1f} deg n={len(drifts)}")
    for n, lst in phis.items():
        ps = [p for _, p, _ in lst]
        print(f"  {n}: windows {len(ps)} fitted rotation range {min(ps):+.1f}..{max(ps):+.1f} deg fix {sorted(set(f for *_, f in lst))}")
