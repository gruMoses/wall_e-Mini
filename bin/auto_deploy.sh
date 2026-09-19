#!/usr/bin/env bash
# WALL-E auto-deploy. Pull origin/main and restart wall-e.service when it is safe.
#
# Runs from cron every 5 minutes as user pi (see docs/auto_deploy.md).
# Guards, in order:
#   1. Nothing to do: local HEAD already equals origin/main.
#   2. Kill switch: a file named .deploy_hold in the repo root holds all deploys.
#   3. Divergence: refuse unless origin/main is a fast-forward of local HEAD.
#   4. Local edits to tracked files: refuse (someone is working on the Pi).
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
  { python3 -m compileall -q pi_app config.py \
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
