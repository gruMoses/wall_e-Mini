#!/usr/bin/env bash
# WALL-E auto-deploy. Pull origin/main and restart wall-e.service when it is safe.
#
# Runs from cron every 5 minutes as user pi (see docs/auto_deploy.md).
# Guards, in order:
#   1. Nothing to do: local HEAD already equals origin/main.
#   2. Kill switch: a file named .deploy_hold in the repo root holds all deploys.
#   3. Divergence: refuse unless origin/main is a fast-forward of local HEAD.
#   4. Local edits to tracked files: refuse (someone is working on the Pi).
#   4a. walle-ups retirement: hold a change that deletes or renames bin/walle_ups.py
#      while a walle-ups unit is still installed (walle_ups_hold below;
#      docs/x1202_power_watcher_design.md, sections 7 and 8).
#   5. Arm guard: never restart while the robot reports is_armed=true. If the
#      service is up but telemetry cannot be read, hold; after 3 consecutive
#      unreadable checks (15 min) the service is treated as hung and restarted.
#   6. Smoke test after the pull: byte-compile + import the control stack. On
#      failure, roll the checkout back to the previous SHA and do not restart.
# Every decision is appended to logs/deploy.log.
set -uo pipefail

REPO=/home/pi/wall_e-Mini
BRANCH=main
SERVICE=wall-e.service
LOG="$REPO/logs/deploy.log"
LOCK=/tmp/walle_auto_deploy.lock
STATE=/tmp/walle_auto_deploy.state
TELEMETRY_URL=http://127.0.0.1:8080/api/telemetry
MAX_UNKNOWN_HOLDS=3

log() { printf '%s %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*" >> "$LOG"; }

# Inputs of walle_ups_hold, overridable for pi_app/tests/test_auto_deploy_hold.py.
SYSTEMCTL=${SYSTEMCTL:-systemctl}
UNIT_DIR=${UNIT_DIR:-/etc/systemd/system}

# walle_ups_hold LOCAL REMOTE
# Returns 0 (HOLD) when LOCAL (the checked-out HEAD) has bin/walle_ups.py and
# REMOTE (the fetched origin/main) does not -- a delete or a rename, decided
# from the two trees (a rename is an R, not a D, in `git diff`) -- and the old
# units may still be installed. Returns 1 (no hold) otherwise. Fails closed: a
# git error, a systemctl error or any doubt holds.
# Why: walle-ups.service has Restart=always, RestartSec=2 and no start limit;
# with its script gone under an enabled unit it restarts every 2 s forever.
# `systemctl is-enabled` is not used: for a missing unit it prints an error and
# returns 1 on this systemd, the same as for a disabled one.
WALLE_UPS_PATH=bin/walle_ups.py
WALLE_UPS_UNITS="walle-ups.service walle-ups-wakecheck.service"

# _tree_has REV PATH: prints "yes" or "no"; returns non-zero on a git error.
_tree_has() {
  local listed
  git rev-parse --verify --quiet "$1^{commit}" > /dev/null || return 1
  listed=$(git ls-tree --full-tree --name-only "$1" "$2") || return 1
  if [ "$listed" = "$2" ]; then echo yes; else echo no; fi
}

walle_ups_hold() {
  local local_rev=$1 remote_rev=$2 had has out err rc errf unit
  had=$(_tree_has "$local_rev" "$WALLE_UPS_PATH") || return 0
  has=$(_tree_has "$remote_rev" "$WALLE_UPS_PATH") || return 0
  if [ "$had" != yes ] || [ "$has" != no ]; then
    return 1
  fi
  # Measured on the Pi (systemd 252): no match prints nothing and returns 1;
  # matches print one line each and return 0. Any stdout = installed; any
  # stderr, a status above 1, or no systemctl = doubt. Both hold.
  errf=$(mktemp) || return 0
  out=$("$SYSTEMCTL" list-unit-files 'walle-ups*' --no-legend 2>"$errf")
  rc=$?
  err=$(cat "$errf" 2>/dev/null) || err="unreadable"
  rm -f "$errf"
  [ -n "${out//[[:space:]]/}" ] && return 0
  [ -n "$err" ] && return 0
  [ "$rc" -gt 1 ] && return 0
  # Exact names only: a leftover walle-ups.service.bak or a
  # walle-ups.service.d drop-in directory must not hold every deploy forever.
  for unit in $WALLE_UPS_UNITS; do
    if [ -e "$UNIT_DIR/$unit" ] || [ -L "$UNIT_DIR/$unit" ]; then
      return 0
    fi
  done
  return 1
}

# Sourced (by the test): stop here, with only the functions defined.
# AUTO_DEPLOY_FUNCTIONS_ONLY=1 on a real run also stops here, but never
# silently: it logs one line.
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
  return 0
fi
if [ -n "${AUTO_DEPLOY_FUNCTIONS_ONLY:-}" ]; then
  mkdir -p "$(dirname "$LOG")" 2>/dev/null
  log "functions-only: no deploy" 2>/dev/null
  echo "functions-only: no deploy" >&2
  exit 0
fi

mkdir -p "$(dirname "$LOG")"
exec 9>"$LOCK"
flock -n 9 || exit 0

cd "$REPO" || exit 0
git fetch -q origin "$BRANCH" || { log "WARN fetch failed"; exit 0; }
LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse "origin/$BRANCH")
[ "$LOCAL" = "$REMOTE" ] && exit 0

if [ -e "$REPO/.deploy_hold" ]; then
  log "HOLD .deploy_hold present; origin/$BRANCH=$REMOTE not deployed"
  exit 0
fi
if ! git merge-base --is-ancestor "$LOCAL" "$REMOTE"; then
  log "REFUSE local $LOCAL is not an ancestor of origin/$BRANCH $REMOTE (diverged); fix by hand"
  exit 0
fi
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  log "REFUSE tracked files modified on the Pi; commit or discard them first"
  exit 0
fi
# Before the pull and before the docs-only decision.
if walle_ups_hold "$LOCAL" "$REMOTE"; then
  log "HOLD walle-ups still installed"
  exit 0
fi

# Arm guard.
armed=unknown
service_up=false
if systemctl is-active --quiet "$SERVICE"; then
  service_up=true
  first=$(curl -s -N -m 4 "$TELEMETRY_URL" 2>/dev/null | head -n 1 || true)
  case "$first" in
    *'"is_armed": true'*)  armed=true ;;
    *'"is_armed": false'*) armed=false ;;
  esac
fi
if [ "$armed" = true ]; then
  log "HOLD robot is armed; $REMOTE deferred"
  exit 0
fi
if [ "$service_up" = true ] && [ "$armed" = unknown ]; then
  # Count consecutive unreadable checks for this target SHA.
  count=0
  if [ -r "$STATE" ]; then
    read -r prev_sha prev_count < "$STATE" || true
    [ "${prev_sha:-}" = "$REMOTE" ] && count=${prev_count:-0}
  fi
  count=$((count + 1))
  printf '%s %s\n' "$REMOTE" "$count" > "$STATE"
  if [ "$count" -lt "$MAX_UNKNOWN_HOLDS" ]; then
    log "HOLD service up but arm state unreadable ($count/$MAX_UNKNOWN_HOLDS); $REMOTE deferred"
    exit 0
  fi
  log "WARN arm state unreadable $count times; treating service as hung"
fi
rm -f "$STATE"

# Docs-only changes update the checkout without a service restart.
code_changed=$(git diff --name-only "$LOCAL" "$REMOTE" | grep -vE '^(docs/|[^/]*\.md$)' | head -n 1)
if [ -z "$code_changed" ]; then
  if git pull -q --ff-only origin "$BRANCH"; then
    log "DOCS-ONLY $LOCAL -> $REMOTE pulled; service not restarted"
  else
    log "FAIL pull --ff-only failed on docs-only change; nothing changed"
  fi
  exit 0
fi

# Deploy.
if ! git pull -q --ff-only origin "$BRANCH"; then
  log "FAIL pull --ff-only failed; nothing changed"
  exit 0
fi
# Bytecode goes to a scratch prefix: a root-owned __pycache__ left in the
# tree (2026-09-19: pi_app/io/__pycache__) made compileall fail with a
# PermissionError and rolled back a good push. The error text is kept.
export PYTHONPYCACHEPREFIX=/tmp/walle_auto_deploy_pycache
SMOKE_OUT=$(
  { python3 -m compileall -q pi_app config.py bin \
    && python3 -c 'import config, pi_app.control.controller, pi_app.control.follow_me'; } 2>&1
)
SMOKE_RC=$?
if [ "$SMOKE_RC" -ne 0 ]; then
  git reset -q --hard "$LOCAL"
  SMOKE_TAIL=$(printf '%s\n' "$SMOKE_OUT" | tail -n 3 | tr '\n' ' ')
  log "FAIL smoke test (rc=$SMOKE_RC) at $REMOTE; rolled back to $LOCAL; service not restarted; error: $SMOKE_TAIL"
  exit 0
fi
if sudo -n systemctl restart "$SERVICE"; then
  log "DEPLOYED $LOCAL -> $REMOTE (armed=$armed) and restarted $SERVICE"
else
  log "FAIL restart of $SERVICE after deploying $REMOTE; restart it by hand"
fi
