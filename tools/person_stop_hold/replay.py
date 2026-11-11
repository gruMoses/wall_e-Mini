"""Replay the one-chain person-stop hold over JSON-lines arm logs.

Usage: python3 tools/person_stop_hold/replay.py <repo> <log> [--trace t0 t1]
Each logged line (~10 Hz) is one depth poll; fresh = oak.det_seq advanced and
not vision_stale. Time is the logged ts (real gaps). Neutral proxy (MANUAL):
bt L/R when present, else the logged motor bytes when the old stop was off,
else RC ch1/ch2 within 25 us of 1500. Operator release: disarmed and ch3 low.
"""
import json, math, sys
from collections import Counter
from types import SimpleNamespace

repo = sys.argv[1]
sys.path.insert(0, repo)
from config import config  # noqa: E402
from pi_app.control.person_stop_hold import PersonStopHold, NeutralTimer  # noqa: E402
from pi_app.hardware.oak_depth import OakDepthReader  # noqa: E402

log = sys.argv[2]
trace = None
if "--trace" in sys.argv:
    i = sys.argv.index("--trace"); trace = (sys.argv[i + 1], sys.argv[i + 2])
oc = config.obstacle_avoidance
hold = PersonStopHold(
    release_distance_m=oc.person_stop_hold_release_m,
    release_polls=oc.person_stop_hold_release_polls,
    min_valid_pct=oc.person_stop_hold_min_valid_pct,
    edge_margin=oc.person_stop_hold_edge_margin,
    in_manual=oc.person_stop_hold_in_manual,
    carry_in_s=oc.person_stop_hold_carry_in_s,
    chain_gap_s=oc.person_stop_hold_chain_gap_s,
    chain_gate=oc.person_stop_hold_chain_gate,
    resolve_polls=oc.person_stop_hold_resolve_polls,
    stop_radius_m=oc.safety_stop_radius_m,
    plausible_speed_mps=oc.person_stop_hold_plausible_speed_mps,
    plausible_margin_m=oc.person_stop_hold_plausible_margin_m,
    plausible_area_factor=oc.person_stop_hold_plausible_area_factor,
)
nt = NeutralTimer(oc.person_stop_hold_neutral_band_byte, oc.person_stop_hold_neutral_hold_s)
R, W = oc.safety_stop_radius_m, oc.safety_stop_bbox_width
rows = []; last_seq = None; mismatch = 0
for line in open(log):
    d = json.loads(line)
    if d.get("type"):
        continue
    ob = d.get("obstacle") or {}
    dist = ob.get("distance_m")
    if dist is None:
        continue
    dets = [SimpleNamespace(safety_tier="stop", label_name="person", z_m=x.get("z_m") or 0.0,
                            bbox=x.get("bbox"), depth_status=x.get("depth_status", "ok"))
            for x in d.get("detections") or []]
    animals = [SimpleNamespace(safety_tier=x.get("tier", "stop"), label_name=x.get("label"),
                               z_m=x.get("z_m") or 0.0, bbox=x.get("bbox"),
                               depth_status=x.get("depth_status", "ok"))
               for x in d.get("animals") or []]
    dets += animals
    seq = (d.get("oak") or {}).get("det_seq")
    fresh = seq is not None and seq != last_seq and not d.get("vision_stale")
    last_seq = seq
    cvp = ob.get("corridor_valid_pct") or 0.0
    sup = ob.get("corridor_support_px") or 0
    p5 = ob.get("depth_p5_mm")
    if not math.isfinite(float(dist)) or p5 is None:
        rejected = True
    else:
        count = sup / (cvp / 100.0) if cvp > 0 else 0
        rejected = cvp < oc.min_valid_pct or sup < max(oc.corridor_min_support_px,
                                                         oc.corridor_min_support_frac * count)
    cmm = float("inf") if rejected else float(p5)
    _, stop_det = OakDepthReader._apply_safety_tier_override(dets, cmm, R, safety_stop_bbox_width=W)
    trig_dets = [x for x in dets if x.safety_tier == "stop" and OakDepthReader._apply_safety_tier_override(
        [x], float("inf"), R, safety_stop_bbox_width=W)[1] is not None] if stop_det is not None else []
    old_stop = float(dist) == 0.0
    mismatch += (stop_det is not None) != old_stop
    rc = d.get("rc") or {}; bt = d.get("bt") or {}
    mot = d.get("motor") or {}
    fwd = (mot.get("L", 126) + mot.get("R", 126)) / 2.0 - 126
    armed = bool((d.get("safety") or {}).get("armed"))
    manual = d["mode"] == "MANUAL"
    if bt.get("L") is not None:
        lr = (bt["L"], bt["R"])
    elif not old_stop:
        lr = (mot.get("L"), mot.get("R"))
    elif abs((rc.get("ch1") or 0) - 1500) <= 25 and abs((rc.get("ch2") or 0) - 1500) <= 25:
        lr = (126, 126)
    else:
        lr = (None, None)
    neutral = nt.update(d["ts"], manual, lr[0], lr[1])
    held = hold.update(now=d["ts"], triggered=stop_det is not None, detections=dets,
                       fresh_detections=fresh, corridor_mm=cmm, corridor_rejected=rejected,
                       corridor_valid_pct=cvp, stop_det=stop_det, trigger_dets=trig_dets,
                       operator_release=(not armed) and (rc.get("ch3") or 2000) <= 1200,
                       autonomous=not manual, forward_neutral=manual and neutral)
    s = hold.snapshot()
    rows.append(dict(ts=d["ts"], iso=d["ts_iso"][11:], mode=d["mode"], old=old_stop, new=held,
                     reason=s.reason, oo=s.operator_only, oor=s.operator_only_reason, fwd=fwd,
                     animals=len([a for a in animals if a.safety_tier == "stop"]), armed=armed))
    if trace and trace[0] <= rows[-1]["iso"] <= trace[1]:
        print(rows[-1]["iso"], d["mode"][:4], "old", int(old_stop), "trig", int(stop_det is not None),
              "HOLD", int(held), s.reason, "ev", s.evidence_polls, "chain", s.chain,
              "pres", int(s.chain_present), "streak", s.beyond_streak, "res", int(s.resolved),
              "OO", s.operator_only_reason or "-", "fresh", int(fresh), "rej", int(rejected),
              "cmm", cmm, "cvp", cvp, "fwd", fwd, "neu", int(manual and neutral),
              [(x.label_name, x.depth_status, round(x.z_m, 2), [round(v, 3) for v in x.bbox])
               for x in dets if x.safety_tier == "stop"])

print("== %s: lines %d, trigger/log mismatches %d, modes %s" % (
    log.split("/")[-1], len(rows), mismatch, dict(Counter(r["mode"] for r in rows))))
an_fm = [r for r in rows if r["mode"] == "FOLLOW_ME" and r["animals"]]
print("   FOLLOW_ME lines with stop-tier animal boxes: %d (held %d, operator-only %d)" % (
    len(an_fm), sum(r["new"] for r in an_fm), sum(r["oo"] for r in an_fm)))
for mode in ("FOLLOW_ME", "MANUAL"):
    print("  [%s] old-stop lines %d, new-held lines %d, held-and-old-off lines %d, of them fwd>4 %d" % (
        mode, sum(1 for r in rows if r["mode"] == mode and r["old"]),
        sum(1 for r in rows if r["mode"] == mode and r["new"]),
        sum(1 for r in rows if r["mode"] == mode and r["new"] and not r["old"]),
        sum(1 for r in rows if r["mode"] == mode and r["new"] and not r["old"] and r["fwd"] > 4)))
eps, start = [], None
for i, r in enumerate(rows):
    if r["new"] and start is None:
        start = i
    if not r["new"] and start is not None:
        eps.append((start, i)); start = None
if start is not None:
    eps.append((start, len(rows)))
print("  holds: start, length s, modes, engaged, released by, operator-only (why, at), extra held lines F/M, fwd-blocked F/M, animals in view")
for a, b in eps:
    seg = rows[a:b]
    end = rows[b]["reason"] if b < len(rows) else "end-of-log"
    dur = (rows[b]["ts"] if b < len(rows) else rows[-1]["ts"]) - rows[a]["ts"]
    ex = {m: sum(1 for r in seg if not r["old"] and r["mode"] == m) for m in ("FOLLOW_ME", "MANUAL")}
    fb = {m: sum(1 for r in seg if not r["old"] and r["mode"] == m and r["fwd"] > 4) for m in ("FOLLOW_ME", "MANUAL")}
    oo = next((r for r in seg if r["oo"]), None)
    print("   %s %6.2f %-9s %-8s %-15s %-28s extra %d/%d fwd %d/%d animals %d" % (
        rows[a]["iso"], dur, "/".join(sorted(set(r["mode"][:4] for r in seg))), seg[0]["reason"], end,
        ("%s @%s" % (oo["oor"], oo["iso"][:8])) if oo else "-",
        ex["FOLLOW_ME"], ex["MANUAL"], fb["FOLLOW_ME"], fb["MANUAL"],
        sum(1 for r in seg if r["animals"])))
