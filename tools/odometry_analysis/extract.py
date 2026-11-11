"""Run on the robot: compact extraction of odometry-related fields from arm logs.
Usage: python3 extract.py OUTDIR log1 [log2 ...]  (writes OUTDIR/<name>.jsonl.gz + summary)"""
import json, sys, gzip, os, collections
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
def g(d, *ks):
    for k in ks:
        if not isinstance(d, dict): return None
        d = d.get(k)
    return d
summ = {}
for path in sys.argv[2:]:
    name = os.path.basename(path)
    s = collections.Counter(); first = last = None
    vio = collections.Counter(); hdr = None
    lastutc = None; n_out = 0
    with open(path, errors="replace") as f, gzip.open(os.path.join(out, name + ".jsonl.gz"), "wt") as o:
        for line in f:
            try: d = json.loads(line)
            except Exception: s["bad"] += 1; continue
            t = d.get("type", "tick")
            s["t_" + t] += 1
            if t == "session_header": hdr = {"git_sha": d.get("git_sha"), "ts_iso": d.get("ts_iso")}; continue
            if t == "slow":
                vio["slow_vio_status_" + str(g(d, "oak_camera_health", "vio_status"))] += 1
                if d.get("vio") is not None: vio["slow_vio_nonnull"] += 1
                if (g(d, "oak_camera_health", "vio_poses") or 0) > 0: vio["slow_vio_poses_pos"] += 1
                continue
            if t == "event":
                o.write(json.dumps({"ev": d.get("event"), "ts": d.get("ts"), "d": {k: v for k, v in d.items() if k not in ("type",)}}) + "\n"); continue
            ts = d.get("ts")
            if ts is None: continue
            if first is None: first = ts
            last = ts
            if d.get("vio") is not None: vio["tick_vio_nonnull"] += 1
            mode = d.get("mode"); s["mode_" + str(mode)] += 1
            fm = d.get("follow_me") or {}
            pm = fm.get("pursuit_mode"); osrc = fm.get("odom_source")
            if mode == "FOLLOW_ME":
                s["pm_" + str(pm)] += 1; s["os_" + str(osrc)] += 1
            gp = d.get("gps") or {}
            lr = g(d, "vesc", "left_rpm"); rr = g(d, "vesc", "right_rpm")
            mL = g(d, "motor", "L"); mR = g(d, "motor", "R")
            moving = (lr is not None and abs(lr) > 100) or (rr is not None and abs(rr) > 100) or (mL not in (None, 126)) or (mR not in (None, 126))
            newgps = gp.get("utc") != lastutc
            if newgps:
                lastutc = gp.get("utc"); s["gps_fix_" + str(gp.get("fix"))] += 1
                if moving: s["gps_fix_moving_" + str(gp.get("fix"))] += 1
            if moving: s["moving_ticks"] += 1
            if not (moving or newgps or mode != "MANUAL" or d.get("events")): continue
            ha = d.get("heading_align") or {}
            row = {"ts": ts, "m": mode, "src": d.get("src"), "ar": g(d, "safety", "armed"),
                   "mL": mL, "mR": mR, "lr": lr, "rr": rr, "v": g(d, "vesc", "speed_mps"),
                   "h": g(d, "imu", "heading_deg"), "yr": g(d, "imu", "yaw_rate_dps"),
                   "zupt": g(d, "imu", "oak_imu", "integrate_status"),
                   "ch": d.get("corrected_heading_deg"), "lk": ha.get("locked"), "off": ha.get("offset_deg"),
                   "lsrc": ha.get("lock_source"), "rel": ha.get("relock_count"), "cve": ha.get("cog_verify_error_deg"),
                   "lat": gp.get("lat"), "lon": gp.get("lon"), "fx": gp.get("fix"), "sat": gp.get("sats"),
                   "hd": gp.get("hdop"), "da": gp.get("diff_age_s"), "ga": gp.get("age_s"), "utc": gp.get("utc"),
                   "cog": gp.get("cog_deg"), "sog": gp.get("sog_mps"), "ng": newgps,
                   "pm": pm, "os": osrc, "ox": fm.get("odom_x"), "oy": fm.get("odom_y"), "oth": fm.get("odom_theta_deg"),
                   "gsp": fm.get("gps_speed_mps"), "trk": fm.get("tracking"), "tx": fm.get("target_x_m"), "tz": fm.get("target_z_m"),
                   "tl": fm.get("trail_length"), "so": fm.get("steer_offset"), "spo": fm.get("speed_offset"),
                   "twx": fm.get("target_world_x"), "twy": fm.get("target_world_y"),
                   "ev": d.get("events") or None}
            o.write(json.dumps(row, separators=(",", ":")) + "\n"); n_out += 1
    summ[name] = {"hdr": hdr, "first": first, "last": last, "dur_s": (last - first) if first else None, "counts": dict(s), "vio": dict(vio), "rows_out": n_out}
    print(name, json.dumps(summ[name]), flush=True)
json.dump(summ, open(os.path.join(out, "summary.json"), "w"), indent=1)
