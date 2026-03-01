#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <scenario_name> <duration_sec> [app_command]"
  exit 1
fi

SCENARIO="$1"
DURATION_SEC="$2"
APP_CMD="${3:-}"

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_DIR="$ROOT_DIR/logs/perf/$(date +%Y%m%d_%H%M%S)_${SCENARIO}"
mkdir -p "$OUT_DIR"

has_cmd() {
  command -v "$1" >/dev/null 2>&1
}

echo "scenario=$SCENARIO" | tee "$OUT_DIR/meta.txt"
echo "duration_sec=$DURATION_SEC" | tee -a "$OUT_DIR/meta.txt"
echo "start_iso=$(date -Iseconds)" | tee -a "$OUT_DIR/meta.txt"

# System monitors
MPSTAT_PID=""
if has_cmd mpstat; then
  (mpstat -P ALL 1 > "$OUT_DIR/mpstat.log") &
  MPSTAT_PID=$!
else
  echo "warn=mpstat_not_found" | tee -a "$OUT_DIR/meta.txt"
fi

(vmstat 1 > "$OUT_DIR/vmstat.log") &
VMSTAT_PID=$!

IOSTAT_PID=""
if has_cmd iostat; then
  (iostat -xz 1 > "$OUT_DIR/iostat.log") &
  IOSTAT_PID=$!
else
  echo "warn=iostat_not_found" | tee -a "$OUT_DIR/meta.txt"
fi

# Thermal/throttle monitor
(
  for ((i=0; i<DURATION_SEC; i++)); do
    ts=$(date +%s.%N)
    temp=$(vcgencmd measure_temp | cut -d= -f2)
    thr=$(vcgencmd get_throttled | cut -d= -f2)
    echo "$ts,temp=$temp,throttled=$thr"
    sleep 1
  done
) > "$OUT_DIR/thermal.log" &
THERM_PID=$!

APP_PID=""
APP_PGID=""
PIDSTAT_PID=""
if [[ -n "$APP_CMD" ]]; then
  # Start app in its own process group so we can terminate the full tree.
  setsid bash -lc "$APP_CMD" > "$OUT_DIR/app.stdout.log" 2> "$OUT_DIR/app.stderr.log" &
  APP_PID=$!
  APP_PGID="$APP_PID"
  echo "app_pid=$APP_PID" | tee -a "$OUT_DIR/meta.txt"

  if has_cmd pidstat; then
    (pidstat -rudh -p "$APP_PID" 1 > "$OUT_DIR/pidstat.log") &
    PIDSTAT_PID=$!
  else
    echo "warn=pidstat_not_found" | tee -a "$OUT_DIR/meta.txt"
  fi
fi

sleep "$DURATION_SEC"

if [[ -n "$APP_PID" ]]; then
  kill -INT "-$APP_PGID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "$APP_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  kill -TERM "-$APP_PGID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "$APP_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  kill -KILL "-$APP_PGID" 2>/dev/null || true
  wait "$APP_PID" 2>/dev/null || true
fi

if [[ -n "$PIDSTAT_PID" ]]; then
  kill "$PIDSTAT_PID" 2>/dev/null || true
  wait "$PIDSTAT_PID" 2>/dev/null || true
fi

if [[ -n "$MPSTAT_PID" ]]; then
  kill "$MPSTAT_PID" 2>/dev/null || true
fi
kill "$VMSTAT_PID" "$THERM_PID" 2>/dev/null || true
if [[ -n "$IOSTAT_PID" ]]; then
  kill "$IOSTAT_PID" 2>/dev/null || true
fi

if [[ -n "$MPSTAT_PID" ]]; then
  wait "$MPSTAT_PID" 2>/dev/null || true
fi
wait "$VMSTAT_PID" "$THERM_PID" 2>/dev/null || true
if [[ -n "$IOSTAT_PID" ]]; then
  wait "$IOSTAT_PID" 2>/dev/null || true
fi

echo "end_iso=$(date -Iseconds)" | tee -a "$OUT_DIR/meta.txt"
echo "out_dir=$OUT_DIR" | tee -a "$OUT_DIR/meta.txt"
echo "$OUT_DIR"
