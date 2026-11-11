"""Truth check of trail pursuit on the real drives (replay only, trail forced ON).

The replay is open loop: the robot moved as it did in the log (direct
pursuit), so a trail-pursuit steer that points back to the person's PATH need
not point at the person NOW. The right question is whether the point the code
steers to is really on the side the code thinks. Truth here = GPS track
(1 Hz, linearly interpolated) + gyro heading rotated by the per-drive fitted
gyro-to-true offset (compare.py), with the person placed from the logged
camera coordinates at the time each trail point was recorded.

For every trail-pursuit tick (forced: direct_pursuit_distance_m 1.0) and every
hidden-target tick (lost scenario) the script takes the raw trail point nearest
to the lookahead, and compares its lateral (right) coordinate in the code's
pose with the same physical point in the truth pose.

Usage: python3 truth.py REPO_ROOT SOURCE SCENARIO LOG PHI_DEG
"""
import bisect, dataclasses, math, sys, collections
root = sys.argv[1]; sys.path.insert(0, root)
from tools.follow_me_analyze import parse_follow_me_runs
from tools.replay_follow_me_log import _detections_from_frame, _heading_deg, _gps_feed
from config import config
from pi_app.control.follow_me import FollowMeController
import pi_app.control.follow_me as fm_mod

SRC, SCEN, LOG, PHI = sys.argv[2], sys.argv[3], sys.argv[4], math.radians(float(sys.argv[5]))
R = 6371000.0
kw = dict(trail_follow_enabled=True, direct_pursuit_distance_m=1.0)
if SRC != "auto":
    kw["trail_odometry_source"] = SRC
ctrl = FollowMeController(dataclasses.replace(config.follow_me, **kw))
frames = [f for r in parse_follow_me_runs(LOG) for f in r]
t0 = frames[0]["ts"]

# GPS truth track (new epochs only)
gt, gn, ge = [], [], []
lat0 = lon0 = None; lu = None
for fr in frames:
    g = fr.get("gps") or {}
    if g.get("lat") is None or (g.get("fix") or 0) < 1 or g.get("utc") == lu:
        continue
    lu = g.get("utc")
    if lat0 is None:
        lat0, lon0 = g["lat"], g["lon"]
    gt.append(fr["ts"] - t0 - (g.get("age_s") or 0.0))
    gn.append(math.radians(g["lat"] - lat0) * R)
    ge.append(math.radians(g["lon"] - lon0) * R * math.cos(math.radians(lat0)))


def tpos(t):
    i = bisect.bisect_left(gt, t)
    if i <= 0: return gn[0], ge[0]
    if i >= len(gt): return gn[-1], ge[-1]
    f = (t - gt[i - 1]) / (gt[i] - gt[i - 1])
    return gn[i - 1] + f * (gn[i] - gn[i - 1]), ge[i - 1] + f * (ge[i] - ge[i - 1])


heads = {}   # rel time -> gyro heading (deg)
cams = {}    # rel time -> (x_m, z_m) of the fed detection
t = {"t": 0.0}; fm_mod.time.monotonic = lambda: t["t"]
pl = pr = 126; lu = None; gts = None
stats = collections.Counter(); errs = []
hid_stats = collections.Counter()
for fr in frames:
    t["t"] = fr["ts"] - t0
    h = _heading_deg(fr); heads[round(t["t"], 4)] = h
    st_imu = ((fr.get("imu") or {}).get("oak_imu") or {}).get("integrate_status")
    try:
        ctrl.update_pose(h, pl, pr, t["t"], heading_valid=st_imu not in {"stale", "init", "gap_freeze", "regressed", "restart"})
    except TypeError:
        ctrl.update_pose(h, pl, pr, t["t"])
    g = _gps_feed(fr); utc = (fr.get("gps") or {}).get("utc")
    if g:
        if utc != lu: lu = utc; gts = t["t"]
        ctrl.update_gps(g[0], g[1], g[2], gts)
    dets = _detections_from_frame(fr)
    hidden = SCEN == "lost" and (t["t"] % 15.0) > 11.5
    fed = [] if hidden else dets
    if fed:
        cams[round(t["t"], 4)] = (fed[0].x_m, fed[0].z_m)
    ctrl.compute(fed)
    st = ctrl.get_status()
    m = fr.get("motor") or {}
    pl = int(m.get("L", pl) or pl); pr = int(m.get("R", pr) or pr)
    mode = st.get("follow_me_pursuit_mode")
    steer = st.get("follow_me_steer_offset") or 0.0
    if hidden and dets and mode in ("trail", "search") and abs(dets[0].x_m) > 0.3 and abs(steer) > 0.5:
        hid_stats[mode + "_n"] += 1
        hid_stats[mode + "_agree_person"] += (steer > 0) == (dets[0].x_m > 0)
    if mode != "trail":
        continue
    odom = ctrl._peek_odometry()  # read-only (get_status uses the same)
    if odom is None:
        continue
    raw = ctrl._trail.get_trail()
    lx, ly = ctrl._pursuit_lookahead_x, ctrl._pursuit_lookahead_y
    pt = min(raw, key=lambda p: (p.x - lx) ** 2 + (p.y - ly) ** 2)
    tp = round(pt.timestamp, 4)
    if tp not in cams or tp not in heads:
        stats["unmatched"] += 1; continue
    p = odom.pose
    code_r = -(pt.x - p.x) * math.sin(p.theta) + (pt.y - p.y) * math.cos(p.theta)
    # truth
    xc, zc = cams[tp]
    th_p = math.radians(heads[tp]) + PHI
    n0, e0 = tpos(tp)
    pn = n0 + zc * math.cos(th_p) - xc * math.sin(th_p)
    pe = e0 + zc * math.sin(th_p) + xc * math.cos(th_p)
    th = math.radians(h) + PHI
    n1, e1 = tpos(t["t"])
    true_r = -(pn - n1) * math.sin(th) + (pe - e1) * math.cos(th)
    errs.append(abs(code_r - true_r))
    if abs(true_r) > 0.3:
        stats["n"] += 1
        stats["side_agree"] += (code_r > 0) == (true_r > 0)
        if abs(steer) > 0.5:
            stats["steer_n"] += 1
            stats["steer_agree_truth"] += (steer > 0) == (true_r > 0)
errs.sort()
med = errs[len(errs) // 2] if errs else float("nan"); p90 = errs[int(0.9 * len(errs))] if errs else float("nan")
print(f"{LOG.rsplit('/',1)[-1]} {SRC} {SCEN}: trail ticks with truth {len(errs)}; lateral error of steered-to point vs truth med {med:.2f} p90 {p90:.2f} m;"
      f" side agrees {stats['side_agree']}/{stats['n']}; steer sign agrees with truth {stats['steer_agree_truth']}/{stats['steer_n']};"
      f" hidden-person agreement {dict(hid_stats)}; unmatched {stats['unmatched']}")
