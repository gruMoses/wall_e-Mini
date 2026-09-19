# Auto-deploy for the WALL-E Pi

`bin/auto_deploy.sh` pulls `origin/main` on the Pi and restarts `wall-e.service` when it is safe. Cron runs it every 5 minutes as user `pi`. Before 2026-09-19 the Pi did not auto-deploy; each deploy was a manual pull and restart.

## What the script does

1. It fetches `origin/main`. If the local `HEAD` equals `origin/main`, it exits.
2. If the file `.deploy_hold` exists in the repo root, it holds all deploys.
3. If the local `HEAD` is not an ancestor of `origin/main`, it refuses. Fix the checkout by hand.
4. If tracked files are modified on the Pi, it refuses.
5. It reads `is_armed` from the first event of `http://127.0.0.1:8080/api/telemetry`. If the robot is armed, it holds. If the service is up but the telemetry cannot be read, it holds. After 3 consecutive unreadable checks (15 minutes) it treats the service as hung and continues.
6. It runs `git pull --ff-only`, then byte-compiles `pi_app` and `config.py`, then imports `config`, `pi_app.control.controller` and `pi_app.control.follow_me`. If any step fails, it resets the checkout to the previous SHA and does not restart the service.
7. It runs `sudo -n systemctl restart wall-e.service`.

Each decision is one line in `logs/deploy.log`.

## Install

Run this on the Pi one time:

```bash
cd /home/pi/wall_e-Mini && git pull --ff-only origin main
chmod +x bin/auto_deploy.sh
( crontab -l 2>/dev/null | grep -v auto_deploy.sh; echo '*/5 * * * * /home/pi/wall_e-Mini/bin/auto_deploy.sh >/dev/null 2>&1' ) | crontab -
crontab -l
sudo systemctl restart wall-e.service
```

NOTE: The manual `git pull` moves the checkout to `origin/main` before cron runs. The script then sees nothing to deploy, so the running service keeps the old code. The last line restarts the service one time. Every later push to `main` deploys through the script.

`sudo -n systemctl restart wall-e.service` must work without a password for user `pi`. `run_main.sh` already uses `sudo`, so this is the case on this Pi.

## Pause and resume

- Pause: `touch /home/pi/wall_e-Mini/.deploy_hold`
- Resume: `rm /home/pi/wall_e-Mini/.deploy_hold`

Use the pause during a field test, or when a push to `main` must not restart the robot yet.

## Verify a deploy

```bash
ssh pi@192.168.86.54 'tail -5 /home/pi/wall_e-Mini/logs/deploy.log; git -C /home/pi/wall_e-Mini log --oneline -1'
```

WARNING: The restart happens at most 5 minutes after a push to `main` when the robot is disarmed. Tell Kevin before you push a change that restarts the service (global rule).

## Appendix: Technical Names

`auto_deploy.sh`, `wall-e.service`, `origin/main`, `HEAD`, cron, crontab, `is_armed`, `.deploy_hold`, `deploy.log`, `git pull --ff-only`, `sudo -n`, `systemctl`, SHA, Pi, `run_main.sh`.
