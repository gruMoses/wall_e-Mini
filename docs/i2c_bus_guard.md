# I2C bus 1 auto-unlock guard

`bin/i2c_bus_guard.py` watches I2C bus 1 for a stuck bus. It runs the
standard I2C bus-clear recovery when the bus is stuck. It runs on the Pi
as the systemd service `i2c-bus-guard`.

## Background: the 2026-09-25 incident

On 2026-09-25 at 21:36:16 the kernel logged this line:

```
i2c_designware 1f00074000.i2c: i2c_dw_handle_tx_abort: SDA stuck at low
```

A device on I2C bus 1 froze mid-transfer. The device held the SDA line
low. Two devices share bus 1: the RTK GPS co-processor (address 0x20) and
the UPSPlus HAT. Both devices went dark for 17 hours. A human power-cycled
the robot to clear the fault.

The UPS daemon (`upsplus-power.service`) logged an I2C read error every 10
seconds during the fault. Its status file (`/tmp/ups_status.json`) went
stale. Nothing on the Pi 5 cleared the stuck bus on its own. The Pi 5
kernel has no GPIO bus-recovery configured for this controller. This
guard clears the bus without a human.

## What it detects

The guard reads two signals every 5 seconds (`--interval`):

1. The age of the UPS daemon's status file. A missing or unreadable file
   counts as an unknown age, not a stale one. The UPS daemon may simply be
   stopped.
2. The level of SDA (GPIO2), read with `pinctrl get 2`.

The guard calls the bus stuck only when both of these are true:

- The UPS status age is more than 30 seconds (`--stale-s`).
- SDA reads low for 3 consecutive samples, about 1 second apart
  (`--low-samples`).

Both conditions are required. Normal I2C traffic holds SDA low for only a
moment, so it never produces 3 consecutive low samples. A stopped UPS
daemon alone never triggers a recovery, because its status file's age
does not by itself mean the bus is stuck.

## What it does on a stuck bus

The guard runs the standard I2C bus-clear procedure:

1. It unbinds the `i2c_designware` driver from the bus.
2. It sets SCL (GPIO3) as an output and SDA (GPIO2) as an input with a
   pull-up resistor.
3. It pulses SCL low then high, up to 16 times. It checks SDA after each
   pulse. It stops as soon as SDA reads high.
4. It sends a STOP condition: SDA goes from low to high while SCL is
   high.
5. It restores SCL and SDA to their normal I2C function.
6. It rebinds the `i2c_designware` driver.
7. It waits 2 seconds, then checks SDA one more time. A high reading
   means the recovery succeeded.

WARNING: step 5 and step 6 always run, even when an earlier step fails.
A failed recovery attempt must never leave the pins in a non-I2C mode or
leave the driver unbound.

The guard runs at most 6 recoveries per rolling hour (`--max-per-hour`).
Past that limit, it logs one error and keeps monitoring. It does not try
again until the hour window frees up.

## Install

The install step is manual. Nothing installs this service automatically.

Run this on the Pi, with sudo:

```bash
sudo bash bin/install_i2c_bus_guard.sh
```

The script copies `bin/i2c-bus-guard.service` to `/etc/systemd/system`,
reloads systemd, enables the service, starts it, and prints its status.

## Check it

```bash
systemctl status i2c-bus-guard --no-pager
cat /tmp/i2c_guard_status.json
journalctl -u i2c-bus-guard -n 200 --no-pager
```

The status file holds one JSON object:

| Field | Meaning |
|---|---|
| `ts` | Time of this status, in seconds since the epoch. |
| `sda_level` | The last SDA reading this cycle: `hi`, `lo`, or `unknown`. |
| `ups_age_s` | Age of the UPS status file, in seconds, or `null` if unknown. |
| `stuck` | `true` if this cycle called the bus stuck. |
| `recoveries_total` | Count of recovery attempts since the guard started. |
| `last_recovery_ts` | Time of the last recovery attempt, or `null`. |
| `last_result` | `ok`, `failed`, or `null` if no recovery has run yet. |
| `last_error` | The error from the last failed recovery, or `null`. |

## Disable it

```bash
sudo systemctl stop i2c-bus-guard
sudo systemctl disable i2c-bus-guard
```

## Manual testing

Run one cycle by hand. Log the actions without touching hardware:

```bash
python3 bin/i2c_bus_guard.py --once --dry-run
```

Drop `--dry-run` to let a single cycle act for real. Add `--status-file`
or `--ups-status-file` to point at test paths instead of the real ones.

## Appendix: Technical Names

`i2c_bus_guard.py`, `i2c-bus-guard.service`, `install_i2c_bus_guard.sh`,
`i2c-bus-guard`, I2C, SDA, SCL, GPIO2, GPIO3, `pinctrl`, `i2c_designware`,
`1f00074000.i2c`, RTK GPS, UPSPlus, HAT, `upsplus-power.service`,
`/tmp/ups_status.json`, `/tmp/i2c_guard_status.json`, `UPS_STATUS_FILE`,
STOP condition, Raspberry Pi 5, Pi 5, systemd, `systemctl`, `journalctl`,
JSON, sudo, `--interval`, `--stale-s`, `--low-samples`, `--max-per-hour`,
`--status-file`, `--ups-status-file`, `--dry-run`, `--once`.
