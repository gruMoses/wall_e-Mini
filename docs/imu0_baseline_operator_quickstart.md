# IMU-0 Baseline + Instrumentation Quickstart

Use this when running IMU-0 (baseline + instrumentation only).

## 1) Prep

```bash
cd /home/pi/wall_e-Mini
sudo systemctl stop wall-e.service || true
```

## 2) Baseline runs (2x nominal, 2x stress, 1x soak)

Nominal (run twice):

```bash
./tools/perf/run_scenario.sh imu0_nominal_60s_r1 60 "cd /home/pi/wall_e-Mini && python3 -u -m pi_app.app.main"
./tools/perf/run_scenario.sh imu0_nominal_60s_r2 60 "cd /home/pi/wall_e-Mini && python3 -u -m pi_app.app.main"
```

Stress (run twice). Start stress load in another terminal before each run:

```bash
stress-ng --cpu 4 --cpu-method all --timeout 75s
```

Then run:

```bash
./tools/perf/run_scenario.sh imu0_stress_60s_r1 60 "cd /home/pi/wall_e-Mini && python3 -u -m pi_app.app.main"
./tools/perf/run_scenario.sh imu0_stress_60s_r2 60 "cd /home/pi/wall_e-Mini && python3 -u -m pi_app.app.main"
```

Soak:

```bash
./tools/perf/run_scenario.sh imu0_soak_30m 1800 "cd /home/pi/wall_e-Mini && python3 -u -m pi_app.app.main"
```

## 3) Generate instrumentation snapshot inputs

Loop tail summary (existing tool):

```bash
python3 ./tools/perf/loop_tail_analyze.py --log ./logs/latest.log --threshold-ms 30 \
  | tee "./docs/imu0_loop_tail_$(date +%Y%m%d_%H%M%S).txt"
```

IMU pipeline counters + loop guardrails (new analyzer):

```bash
python3 ./tools/imu_pipeline_analyzer.py --log ./logs/latest.log \
  | tee "./docs/imu0_instrumentation_extract_$(date +%Y%m%d_%H%M%S).txt"
```

## 4) Fill the report

Create report from template:

```bash
cp ./docs/imu_instrumentation_snapshot_template.md "./docs/imu0_instrumentation_snapshot_$(date +%Y%m%d).md"
```

Fill it using:

- `logs/perf/<timestamp>_imu0_*` directories from `run_scenario.sh`
- `docs/imu0_loop_tail_*.txt`
- `docs/imu0_instrumentation_extract_*.txt`
- baseline comparison doc (for example `docs/performance_baseline_report_20260301.md`)

## 5) Cleanup

```bash
sudo systemctl start wall-e.service || true
```

## Outputs (where files go)

- Scenario bundles: `logs/perf/<timestamp>_<scenario>/`
- App run log symlink: `logs/latest.log` (points to `logs/run_YYYYMMDD_HHMMSS.log`)
- IMU extraction text: `docs/imu0_instrumentation_extract_*.txt`
- Loop tail summary text: `docs/imu0_loop_tail_*.txt`
- Final snapshot report: `docs/imu0_instrumentation_snapshot_YYYYMMDD.md`
