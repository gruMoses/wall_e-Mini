"""Flag-off equivalence: replay_run with the DEFAULT config on the three drives.
Usage: python3 tools/odometry_analysis/flag_off_equivalence.py REPO_ROOT OUT.json LOG [LOG ...]"""
import json, sys
root = sys.argv[1]
sys.path.insert(0, root)
from tools.follow_me_analyze import parse_follow_me_runs
from tools.replay_follow_me_log import replay_run
from config import config
assert config.follow_me.trail_follow_enabled is False
out = {}
for p in sys.argv[3:]:
    frames = [f for run in parse_follow_me_runs(p) for f in run]
    rows = replay_run(frames, config.follow_me)
    out[p.rsplit("/", 1)[-1]] = [[r["replay_L"], r["replay_R"], r["replay_steer_offset"], r["replay_speed_offset"], r["pursuit_mode"]] for r in rows]
json.dump(out, open(sys.argv[2], "w"))
print({k: len(v) for k, v in out.items()}, "total", sum(len(v) for v in out.values()))
