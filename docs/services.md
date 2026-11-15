# Services and Startup Flow

This project is commonly run through systemd services on the Pi.

## Primary Services

- `wall-e.service`: main control application launcher
- `wall-e-spp.service`: Bluetooth SPP diagnostics server. Its control writes
  are gated OFF by default since 2026-07-17 (commit `a2628f0`, environment
  variable `WALL_E_SPP_CONTROL_ENABLED`). It is not the phone drive path.
  Refer to "Phone Command File and Bluetooth SPP" below.
- `walle-power.service`: power watcher for the Geekworm X1202 board. It runs
  as root (`bin/walle_power.py`). It halts the Pi when the board input is lost
  (GPIO 6 low for 5 s within 7.5 s). It forces a power-off when the cells are
  low. It keeps charging disabled (GPIO 16) except for the hold bursts
  (`--hold`). Design and test results: `docs/x1202_power_watcher_design.md`.
- `i2c-bus-guard.service`: I2C bus 1 auto-unlock guard
  (`bin/i2c_bus_guard.py`).
- `wifi-watchdog.service`: Wi-Fi reconnect watchdog (`bin/wifi_watchdog.py`).

The unit files of `wall-e.service` and `wall-e-spp.service` are installed under
`/etc/systemd/system` on the target Pi and are not versioned in this
repository. The unit files of the other three services are versioned in `bin/`.

Auto-deploy does NOT install or restart those three services. Install each one
by hand on the Pi. Run its install script again after a change to its code or
its unit file:

- `walle-power.service`: `sudo bash bin/install_walle_power.sh`
- `i2c-bus-guard.service`: `sudo bash bin/install_i2c_bus_guard.sh`
- `wifi-watchdog.service`: `sudo bash bin/install_wifi_watchdog.sh`

For the power board and the watcher design, refer to
`docs/pi_power_architecture.md` (section 7) and
`docs/x1202_power_watcher_design.md`. The Waveshare UPS HAT (B) and its
watcher `walle-ups` are retired (2026-11-14); refer to `docs/ups_hat_b.md`.

## `run_main.sh` Behavior

`run_main.sh` is the production entrypoint used by `wall-e.service`.

At startup it:

1. Ensures `wall-e-spp.service` is running.
2. Terminates stale `python3 -m app.main` / venv Python processes.
3. Removes stale lock file `/tmp/wall_e_mini_main.lock`.
4. Launches `python3 -u -m app.main` from `pi_app/`.

`run_main.sh` does not start the power watcher. `walle-power.service` is its own
systemd service.

## Phone Command File and Bluetooth SPP

- `pi_app/web/teleop.py` writes the phone command file
  `/tmp/wall_e_bt_latest.json` (class `FileCommandSink`). It is the
  session-gated writer: arming ceremony, latched e-stop, and 250 ms deadman.
  Refer to `docs/teleop.md`.
- Main loop (`pi_app/app/main.py`) reads that file and applies the override only
  when the command age is 0 to 0.6 s on the monotonic clock and the command is
  from this boot (`pi_app/app/phone_override.py`, 2026-11-10). Each command
  carries `last_update_mono_s` and `boot_id`. A command with no stamp, a
  future stamp, or a missing or different `boot_id` is stale. RC arming still
  gates the motors. Refer to `docs/teleop.md`.
- `wall-e-spp.service` runs `pi_app/cli/spp_server.py`. This is a diagnostics
  server: it accepts connections and parses, acknowledges, and logs commands.
  Its control writes to `/tmp/wall_e_bt_latest.json` are gated OFF by default
  since 2026-07-17 (commit `a2628f0`). To enable them, set
  `WALL_E_SPP_CONTROL_ENABLED=1` on the service unit. Do this only for
  supervised bench testing. Refer to `docs/spp_writer_audit.md`.

## Power Status File

`walle-power.service` writes `/run/walle-power/status.json` (schema
`walle-power/1`). The `/debug` page shows the file through `/api/ups`
(`pi_app/web/oak_viewer.py`). `i2c-bus-guard.service` uses the field
`bus_ok_mono` of this file as its witness that the bus reader can see the I2C
bus. It uses the file only when its `boot_id` is the boot id of this boot.
When the watcher halts the Pi, it also writes a halt record to
`/var/lib/walle-power/events.jsonl`. Refer to
`docs/x1202_power_watcher_design.md`.

## Logs and Status

Useful commands:

```bash
systemctl status wall-e.service --no-pager
systemctl status wall-e-spp.service --no-pager
systemctl status walle-power.service --no-pager
systemctl status i2c-bus-guard.service --no-pager
systemctl status wifi-watchdog.service --no-pager
journalctl -u wall-e.service -n 200 --no-pager
journalctl -u wall-e-spp.service -n 200 --no-pager
journalctl -u walle-power.service -n 200 --no-pager
journalctl -u i2c-bus-guard.service -n 200 --no-pager
journalctl -u wifi-watchdog.service -n 200 --no-pager
cat /run/walle-power/status.json
tail /var/lib/walle-power/events.jsonl
```

Runtime logs are also written under `logs/` by the main app (including
`latest.log` symlink and PID CSV traces when enabled).
