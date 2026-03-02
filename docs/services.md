# Services and Startup Flow

This project is commonly run through systemd services on the Pi.

## Primary Services

- `wall-e.service`: main control application launcher
- `wall-e-spp.service`: Bluetooth SPP server that writes BT command state file
- `upsplus-power.service`: UPS watchdog (if UPS hardware is present)

Service unit files are typically installed under `/etc/systemd/system` on the
target Pi and are not versioned in this repository.

## `run_main.sh` Behavior

`run_main.sh` is the production entrypoint used by `wall-e.service`.

At startup it:

1. Checks I2C for UPS MCU address (`0x17` or `0x18`) and starts
   `upsplus-power.service` when detected.
2. Ensures `wall-e-spp.service` is running.
3. Terminates stale `python3 -m app.main` / venv Python processes.
4. Removes stale lock file `/tmp/wall_e_mini_main.lock`.
5. Launches `python3 -u -m app.main --pid-debug` from `pi_app/`.

## Bluetooth Data Path

- `wall-e-spp.service` runs `pi_app/cli/spp_server.py`.
- Server writes latest command to `/tmp/wall_e_bt_latest.json`.
- Main loop (`pi_app/app/main.py`) reads that file and applies BT override only
  when command age is `<= 0.6s`.

## Logs and Status

Useful commands:

```bash
systemctl status wall-e.service --no-pager
systemctl status wall-e-spp.service --no-pager
systemctl status upsplus-power.service --no-pager
journalctl -u wall-e.service -n 200 --no-pager
journalctl -u wall-e-spp.service -n 200 --no-pager
journalctl -u upsplus-power.service -n 200 --no-pager
```

Runtime logs are also written under `logs/` by the main app (including
`latest.log` symlink and PID CSV traces when enabled).
