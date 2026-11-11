#!/bin/bash
# Wait until the robot has pulled a commit and wall-e runs again, then print its health.
#
# Usage:
#   WALLE_SSH=pi@<robot address> [WALLE_JUMP=<user>@<jump host>] tools/wait_deploy.sh <commit>
#
# The robot pulls origin/main with cron each 5 minutes (bin/auto_deploy.sh) and holds
# while armed. This script checks the ANCESTRY of the commit, not HEAD equality, because
# later commits can be on top of it. It prints: the robot's HEAD, the service state and
# start time, the last deploy-log lines, the count of tracebacks since the start, and the
# last status line. Exit code 1 if the commit did not arrive in 12 minutes.
#
# Read-only on the robot: git, systemctl show/is-active, journalctl, tail.
set -u
SHA="${1:?usage: WALLE_SSH=pi@<robot> [WALLE_JUMP=user@host] tools/wait_deploy.sh <commit>}"
TARGET="${WALLE_SSH:?set WALLE_SSH, for example pi@<robot address>}"
REPO="${WALLE_REPO:-/home/pi/wall_e-Mini}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=12)
if [ -n "${WALLE_JUMP:-}" ]; then SSH+=(-J "$WALLE_JUMP"); fi

out=""
for _ in $(seq 1 36); do
  out=$("${SSH[@]}" "$TARGET" "cd $REPO && git merge-base --is-ancestor $SHA HEAD && echo LANDED" 2>/dev/null)
  [ "$out" = "LANDED" ] && break
  sleep 20
done
echo "ancestry: ${out:-NOT LANDED after 12 min}"
[ "$out" = "LANDED" ] || exit 1

# Give the service time to start the camera before the health read.
sleep 40
"${SSH[@]}" "$TARGET" "REPO=$REPO bash -s" <<'REMOTE'
cd "$REPO"
echo "robot HEAD: $(git log -1 --format="%h %s" | cut -c1-80)"
START=$(systemctl show wall-e -p ActiveEnterTimestamp --value)
echo "service: $(systemctl is-active wall-e), started $START, restarts $(systemctl show wall-e -p NRestarts --value), now $(date +%H:%M:%S)"
tail -n 2 logs/deploy.log | cut -c1-200
echo "tracebacks since start: $(journalctl -u wall-e -a --since "$START" --no-pager 2>/dev/null | grep -c -E "Traceback|Error:|Exception")"
journalctl -u wall-e -a --since "$START" --no-pager 2>/dev/null | grep -E "Traceback|Error:|Exception" | head -5 | cut -c1-200
journalctl -u wall-e -a --since "-12 s" --no-pager | tail -n 1 | cut -c1-240
REMOTE
