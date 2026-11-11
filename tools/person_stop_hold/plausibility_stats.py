"""Plausibility numbers for the person-stop hold chain (one object between sightings).

Pairs: consecutive logged lines <= 0.6 s apart with exactly one stop-tier box of
the same group in both. Trusted = depth "ok", z > 0, box >= 0.05 from left and
right edge. Reports |dz| - v*dt excess (range test margin) and the area ratio
against the 1/z^2 bounds with z0 = previous trusted range.
"""
import json, sys

V = float(sys.argv[1]); M = float(sys.argv[2]); J = float(sys.argv[3]); ZMIN = 0.2
files = sys.argv[4:]
needJ = []
EM = 0.05


def boxes(d):
    out = []
    for x in d.get("detections") or []:
        out.append(("person", x))
    for x in d.get("animals") or []:
        if x.get("tier") == "stop":
            out.append(("animal", x))
    return out


def trusted(x):
    bb = x.get("bbox")
    return (x.get("depth_status", "ok") == "ok" and (x.get("z_m") or 0) > 0 and bb
            and bb[0] >= EM and bb[2] <= 1 - EM)


def area(bb):
    return max(0, bb[2] - bb[0]) * max(0, bb[3] - bb[1])


rng = []; ar = []; viol_r = []; viol_a = []
for f in files:
    prev = None
    for line in open(f):
        d = json.loads(line)
        if d.get("type"):
            continue
        b = boxes(d)
        if len(b) != 1 or not b[0][1].get("bbox"):
            prev = None
            continue
        cur = (d["ts"], b[0][0], b[0][1], d["ts_iso"][11:23], d["mode"])
        if prev is not None and prev[1] == cur[1]:
            dt = cur[0] - prev[0]
            if 0 < dt <= 0.6:
                p, c = prev[2], cur[2]
                if trusted(p) and trusted(c):
                    dz = abs(c["z_m"] - p["z_m"])
                    ex = dz - V * dt
                    rng.append(ex)
                    if ex > M:
                        viol_r.append((cur[3], cur[4], cur[1], round(p["z_m"], 2), round(c["z_m"], 2), round(dt, 3)))
                    z0 = p["z_m"]; r = V * dt + M
                    lo = (z0 / (z0 + r)) ** 2
                    lo0 = lo; lo /= J
                    hi = (z0 / max(z0 - r, ZMIN)) ** 2
                    hi0 = hi; hi *= J
                    a0, a1 = area(p["bbox"]), area(c["bbox"])
                    if a0 > 0:
                        q = a1 / a0
                        ar.append(q)
                        needJ.append(max(lo0 / q, q / hi0))
                        if not (lo <= q <= hi):
                            viol_a.append((cur[3], cur[4], cur[1], round(z0, 2), round(c["z_m"], 2), round(q, 3), round(lo, 3), round(hi, 3)))
        prev = cur

rng.sort()
print("V=%.2f M=%.2f trusted pairs %d" % (V, M, len(rng)))
for p in (50, 99, 99.9, 100):
    print("  excess |dz|-V*dt p%s = %.3f" % (p, rng[min(len(rng) - 1, int(p / 100 * len(rng)))]))
print("  range violations (> M):", len(viol_r), viol_r[:10])
ar.sort()
print("  area ratio min %.3f p0.1 %.3f p99.9 %.3f max %.3f" % (ar[0], ar[int(0.001 * len(ar))], ar[int(0.999 * len(ar))], ar[-1]))
print("  area violations:", len(viol_a), viol_a[:10])
needJ.sort(); print("  J needed max %.3f p99.9 %.3f" % (needJ[-1], needJ[int(0.999*len(needJ))]))
