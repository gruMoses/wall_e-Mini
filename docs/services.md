# Services and Startup Flow

This project is commonly run through systemd services on the Pi.

## Primary Services

- `wall-e.service`: main control application launcher
- `wall-e-spp.service`: Bluetooth SPP diagnostics server. Its control writes
  are gated OFF by default since 2026-07-17 (commit `a2628f0`, environment
  variable `WALL_E_SPP_CONTROL_ENABLED`). It is not the phone drive path.
  Refer to "Phone Command File and Bluetooth SPP" below.
- `walle-ups.service`: UPS watcher for the Waveshare UPS HAT (B). It runs as
  root (`bin/walle_ups.py run`). It halts the Pi when mains has been absent for
  5 seconds. A sample counts as absent only while the pack discharges and reads below the supply setpoint minus 0.30 V (7.20 V), so a pack above that line holds the Pi up first. Before the halt, it arms the RTC alarm, so the Pi wakes to check
  the mains again.
- `walle-ups-wakecheck.service`: early-boot oneshot. It powers the Pi off
  again if mains is still absent.
- `i2c-bus-guard.service`: I2C bus 1 auto-unlock guard
  (`bin/i2c_bus_guard.py`).
- `wifi-watchdog.service`: Wi-Fi reconnect watchdog (`bin/wifi_watchdog.py`).

The unit files of `wall-e.service` and `wall-e-spp.service` are installed under
`/etc/systemd/system` on the target Pi and are not versioned in this
repository. The unit files of the other four services are versioned in `bin/`.

Auto-deploy does NOT install or restart those four services. Install each one
by hand on the Pi. Run its install script again after a change to its code or
its unit file:

- `walle-ups.service` and `walle-ups-wakecheck.service` (one script installs
  both): `sudo bash bin/install_walle_ups.sh`
- `i2c-bus-guard.service`: `sudo bash bin/install_i2c_bus_guard.sh`
- `wifi-watchdog.service`: `sudo bash bin/install_wifi_watchdog.sh`

For the UPS design, the bench readings, and the bootloader settings, refer to
`docs/ups_hat_b.md`.

## `run_main.sh` Behavior

`run_main.sh` is the production entrypoint used by `wall-e.service`.

At startup it:

1. Ensures `wall-e-spp.service` is running.
2. Terminates stale `python3 -m app.main` / venv Python processes.
3. Removes stale lock file `/tmp/wall_e_mini_main.lock`.
4. Launches `python3 -u -m app.main` from `pi_app/`.

`run_main.sh` does not start the UPS watcher. `walle-ups.service` is its own
systemd service.

## Phone Command File and Bluetooth SPP

- `pi_app/web/teleop.py` writes the phone command file
  `/tmp/wall_e_bt_latest.json` (class `FileCommandSink`). It is the
  session-gated writer: arming ceremony, latched e-stop, and 250 ms deadman.
  Refer to `docs/teleop.md`.
- Main loop (`pi_app/app/main.py`) reads that file and applies the override only
  when command age is `<= 0.6s`. RC arming still gates the motors.
- `wall-e-spp.service` runs `pi_app/cli/spp_server.py`. This is a diagnostics
  server: it accepts connections and parses, acknowledges, and logs commands.
  Its control writes to `/tmp/wall_e_bt_latest.json` are gated OFF by default
  since 2026-07-17 (commit `a2628f0`). To enable them, set
  `WALL_E_SPP_CONTROL_ENABLED=1` on the service unit. Do this only for
  supervised bench testing. Refer to `docs/spp_writer_audit.md`.

## UPS Status File

`walle-ups.service` writes `/tmp/ups_status.json` (schema `walle-ups/1`) on each
poll. The field `ts` is the time of the last successful INA219 read, so a
watcher that cannot read the sensor looks stale. The field `ts_loop` is the time
of the last poll. The `/debug` page shows the file through `/api/ups`
(`pi_app/web/oak_viewer.py`). `i2c-bus-guard.service` uses the age of `ts` as
its witness that the UPS reader can see the I2C bus.

## Logs and Status

Useful commands:

```bash
systemctl status wall-e.service --no-pager
systemctl status wall-e-spp.service --no-pager
systemctl status walle-ups.service --no-pager
systemctl status walle-ups-wakecheck.service --no-pager
systemctl status i2c-bus-guard.service --no-pager
systemctl status wifi-watchdog.service --no-pager
journalctl -u wall-e.service -n 200 --no-pager
journalctl -u wall-e-spp.service -n 200 --no-pager
journalctl -u walle-ups.service -n 200 --no-pager
journalctl -u walle-ups-wakecheck.service -n 200 --no-pager
journalctl -u i2c-bus-guard.service -n 200 --no-pager
journalctl -u wifi-watchdog.service -n 200 --no-pager
cat /tmp/ups_status.json
```

Runtime logs are also written under `logs/` by the main app (including
`latest.log` symlink and PID CSV traces when enabled).
