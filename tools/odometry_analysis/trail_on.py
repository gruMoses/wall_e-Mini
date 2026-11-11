"""Trail-ON replay of real follow-me drives (replay only; the config default stays False).

Usage: python3 trail_on.py REPO_ROOT SOURCE SCENARIO LOG [LOG ...]
  SOURCE   dead_reckon | gps | auto   (auto = whatever the tree does by itself)
  SCENARIO asis   - default distances (trail pursuit only beyond 5.2 m)
           forced - direct_pursuit_distance_m 1.0 (trail pursuit beyond 1.3 m)
           lost   - forced, plus detections hidden for 3.5 s every 15 s
Prints per-log mode counts and steer-sign agreement with the person's logged
camera bearing (target_x_m, + = right; for hidden frames the person the
controller could not see).
"""
import dataclasses, json, math, sys, collections
root = sys.argv[1]
sys.path.insert(0, root)
from tools.follow_me_analyze import parse_follow_me_runs
from tools.replay_follow_me_log import _detections_from_frame, _heading_deg, _gps_feed
from config import config
from pi_app.control.follow_me import FollowMeController
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
import pi_app.control.follow_me as fm_mod

SRC, SCEN = sys.argv[2], sys.argv[3]
BAD = {"stale", "init", "gap_freeze", "regressed", "restart"}


def cfg_for():
    kw = dict(trail_follow_enabled=True)
    if SCEN in ("forced", "lost"):
        kw["direct_pursuit_distance_m"] = 1.0
    fields = {f.name for f in dataclasses.fields(config.follow_me)}
    if SRC != "auto":
        assert "trail_odometry_source" in fields, "tree has no source switch"
        kw["trail_odometry_source"] = SRC
    return dataclasses.replace(config.follow_me, **kw)


def run(frames):
    ctrl = FollowMeController(cfg_for())
    t = {"t": 0.0}
    real = fm_mod.time.monotonic
    fm_mod.time.monotonic = lambda: float(t["t"])
    prev_l = prev_r = int(CENTER_OUTPUT_VALUE)
    t0 = float(frames[0]["ts"])
    last_utc = None; gps_ts = None
    out = []
    try:
        for fr in frames:
            t["t"] = float(fr["ts"]) - t0
            hdg = _heading_deg(fr)
            st_imu = ((fr.get("imu") or {}).get("oak_imu") or {}).get("integrate_status")
            hv = st_imu not in BAD and (fr.get("imu") or {}).get("heading_deg") is not None
            try:
                ctrl.update_pose(hdg, prev_l, prev_r, t["t"], heading_valid=hv)
            except TypeError:
                ctrl.update_pose(hdg, prev_l, prev_r, t["t"])
            g = _gps_feed(fr)
            utc = (fr.get("gps") or {}).get("utc")
            if g is not None:
                if utc != last_utc:
                    last_utc = utc; gps_ts = t["t"]
                ctrl.update_gps(g[0], g[1], g[2], gps_ts)
            dets = _detections_from_frame(fr)
            hidden = SCEN == "lost" and (t["t"] % 15.0) > 11.5
            fed = [] if hidden else dets
            ctrl.compute(fed)
            st = ctrl.get_status()
            fmz = fr.get("follow_me") or {}
            out.append(dict(t=t["t"], mode=st.get("follow_me_pursuit_mode"), steer=st.get("follow_me_steer_offset"),
                            speed=st.get("follow_me_speed_offset"), src=st.get("odom_source"),
                            hidden=hidden, x=fmz.get("target_x_m") if dets else None,
                            z=fmz.get("target_z_m") if dets else None,
                            resets=st.get("trail_frame_resets"), gref=st.get("gps_heading_reference_known")))
            m = fr.get("motor") or {}
            prev_l = int(m["L"]) if m.get("L") is not None else prev_l
            prev_r = int(m["R"]) if m.get("R") is not None else prev_r
    finally:
        fm_mod.time.monotonic = real
    return out


for p in sys.argv[4:]:
    frames = [f for r in parse_follow_me_runs(p) for f in r]
    rows = run(frames)
    modes = collections.Counter(r["mode"] for r in rows)
    srcs = collections.Counter(r["src"] for r in rows)
    res = {}
    for key, sel in (("trail_seen", lambda r: r["mode"] == "trail" and not r["hidden"]),
                     ("trail_hidden", lambda r: r["mode"] == "trail" and r["hidden"]),
                     ("search_hidden", lambda r: r["mode"] == "search" and r["hidden"])):
        rr = [r for r in rows if sel(r) and r["x"] is not None and abs(r["x"]) > 0.3 and r["steer"] and abs(r["steer"]) > 0.5]
        agree = sum(1 for r in rr if (r["steer"] > 0) == (r["x"] > 0))
        res[key] = f"{agree}/{len(rr)}"
    n_tr = sum(1 for r in rows if r["mode"] == "trail")
    first_valid = next((r["t"] for r in rows if r["src"] not in (None, "none")), None)
    print(f"{p.rsplit('/',1)[-1]} src={SRC} scen={SCEN}: frames {len(rows)} modes {dict(modes)} odom {dict(srcs)}"
          f" first-valid-pose t={first_valid if first_valid is None else round(first_valid,1)}s"
          f" frame-resets {rows[-1]['resets']} | steer sign agrees with person bearing (|x|>0.3 m): {res}")
