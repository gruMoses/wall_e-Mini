# Perf Harness Quick Start

## Files

- `run_scenario.sh` - collects CPU/memory/disk/thermal stats for a timed run.
- `oak_timing_probe.py` - captures OAK frame timing, drops, and host-latency CSV.

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
```

Outputs are written to `logs/perf/`.
