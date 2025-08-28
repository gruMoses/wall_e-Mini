#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/pi/wall_e-Mini"
VENV_DIR="$PROJECT_ROOT/.venv"
VENV_PY="$VENV_DIR/bin/python3"
APP_DIR="$PROJECT_ROOT/pi_app"
LOCK_FILE="/tmp/wall_e_mini_main.lock"

echo "[run_main] Ensuring previous instances are stopped..."

# Find and terminate any running main module processes (match exact module invocation)
mapfile -t MAIN_PIDS < <(ps -eo pid,cmd | awk '/python3 .* -m app\.main/ {print $1}' || true)

# Find and terminate any Python processes using this venv's interpreter
mapfile -t VENV_PIDS < <(ps -eo pid,cmd | awk -v vpy="$VENV_DIR/bin/python" '$2==vpy || index($0,vpy)>0 {print $1}' || true)

# Merge and de-duplicate PIDs
declare -A PID_SET=()
for pid in "${MAIN_PIDS[@]:-}"; do [[ -n "${pid:-}" ]] && PID_SET[$pid]=1; done
for pid in "${VENV_PIDS[@]:-}"; do [[ -n "${pid:-}" ]] && PID_SET[$pid]=1; done

if (( ${#PID_SET[@]} > 0 )); then
  echo "[run_main] Terminating PIDs: ${!PID_SET[@]}"
  # First try graceful termination
  for pid in "${!PID_SET[@]}"; do
    if [[ "$pid" != "$$" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  # Force kill remaining
  for pid in "${!PID_SET[@]}"; do
    if ps -p "$pid" >/dev/null 2>&1; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
else
  echo "[run_main] No existing processes found."
fi

# Clean up stale lock file if present
if [[ -f "$LOCK_FILE" ]]; then
  echo "[run_main] Removing stale lock file $LOCK_FILE"
  rm -f "$LOCK_FILE" || true
fi

echo "[run_main] Activating venv and starting main..."

if [[ ! -x "$VENV_PY" ]]; then
  echo "[run_main] ERROR: venv Python not found at $VENV_PY" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

cd "$APP_DIR"
exec python3 -u -m app.main --pid-debug


