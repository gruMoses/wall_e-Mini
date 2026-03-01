# Perf Harness Quick Start

## Files

- `run_scenario.sh` - collects CPU/memory/disk/thermal stats for a timed run.
- `oak_timing_probe.py` - captures OAK frame timing, drops, and host-latency CSV.
- `follow_me_analyze.py` - summarizes FOLLOW_ME lock/reacquisition + loop timing from a log window.
- `follow_me_probe.py` - toggles FOLLOW_ME for a timed window and runs analysis automatically.
- `loop_tail_analyze.py` - profiles loop tail-latency conditions from run logs.

## Typical Usage

```bash
cd /home/pi/wall_e-Mini

# 60s idle baseline
./tools/perf/run_scenario.sh idle 60

# 120s app run baseline
./tools/perf/run_scenario.sh full_stack 120 "python3 -u -m app.main --pid-debug"

# 60s camera timing probe
python3 ./tools/perf/oak_timing_probe.py \
  --seconds 60 \
  --fps 30 \
  --width 1280 \
  --height 720 \
  --out ./logs/perf/oak_probe_720p30.csv

# Analyze a follow-me window by epoch bounds
python3 ./tools/perf/follow_me_analyze.py \
  --log ./logs/latest.log \
  --start-epoch 1772392033.66 \
  --end-epoch 1772392078.66

# Or analyze last 45 seconds of FOLLOW_ME rows only
python3 ./tools/perf/follow_me_analyze.py \
  --log ./logs/latest.log \
  --last-seconds 45 \
  --follow-only

# Analyze tail-latency conditions (default threshold 30ms)
python3 ./tools/perf/loop_tail_analyze.py \
  --log ./logs/latest.log \
  --threshold-ms 30

# End-to-end follow-me probe (toggle on -> wait -> toggle off -> analyze)
python3 ./tools/perf/follow_me_probe.py \
  --seconds 45 \
  --log ./logs/latest.log
```

Outputs are written to `logs/perf/`.
