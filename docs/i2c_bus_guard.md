# I2C bus 1 auto-unlock guard

`bin/i2c_bus_guard.py` watches I2C bus 1 for a stuck bus. It runs the
standard I2C bus-clear recovery when the bus is stuck, it repairs itself
after a kill signal cuts a recovery short, and it warns (and sometimes
restarts the UPS daemon) whenever the UPS daemon's status goes stale, for
any reason. It runs on the Pi as the systemd service `i2c-bus-guard`.

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

Every cycle (`--interval`, default 5 seconds), the guard reads:

1. The age of the UPS daemon's status file. A missing or unreadable file
   counts as an unknown age, not a stale one. The UPS daemon may simply be
   stopped.
2. SDA (GPIO2) and SCL (GPIO3) together, in one `pinctrl get 2,3` call.

The guard calls the bus stuck only when both of these are true:

- The UPS status age is more than 30 seconds (`--stale-s`).
- Every one of 10 consecutive joint samples, about 100ms apart
  (`--sample-count`, `--sample-interval-s`), shows SDA low AND SCL high.

Both conditions are required. SDA low with SCL high, held for a full
second, is the fingerprint of a wedged slave: it holds SDA down while the
clock sits idle. A live transfer toggles SCL, and a released bus reads
SDA high -- either one breaks the streak on its very next sample, so
ordinary I2C traffic is never mistaken for a stuck bus. A stopped UPS
daemon alone never triggers a recovery either, because its status file's
age does not by itself mean the bus is stuck.

WARNING: a stuck bus is only one way the UPS daemon can go blind. The
guard also checks the UPS status age by itself, every cycle, regardless
of SDA/SCL. When that age is stale for any reason, it logs a warning:
safe shutdown on power loss is not active. This warning repeats at most
once every 5 minutes. A missing or unreadable status file is different
from a stale one: the guard logs that once, at a lower level, and does
not repeat it -- the UPS hardware may simply not be fitted to this robot.

## What it does on a stuck bus

The guard runs the standard I2C bus-clear procedure. Every step below
drives a line low or releases it -- never drives a line high. A wedged
device can still be holding a line low. Driving that same line high
would fight the wedged device on one wire. Releasing the line instead
lets the pull-up resistor pull it high only when nothing else holds it
down.

1. It unbinds the `i2c_designware` driver from the bus.
2. It releases SCL (GPIO3) and SDA (GPIO2): both become inputs, both with
   a pull-up resistor.
3. It pulses SCL: drive low, then release, up to 16 times. It checks SDA
   after each release. It stops as soon as SDA reads high.
4. It sends a STOP condition, in this exact order: drive SCL low; drive
   SDA low; release SCL; release SDA. SDA then goes from low to high
   while SCL is high. That transition is the STOP condition.
5. It restores SCL and SDA to their normal I2C function.
6. It rebinds the `i2c_designware` driver.
7. It waits 2 seconds, then samples SDA 3 times, 200ms apart. 2 or more
   high readings means the recovery succeeded -- a single low sample can
   land squarely inside a live transfer the instant a client (the RTK GPS
   reader, say) reopens its handle and starts talking again, and that
   alone must not look like a failed recovery.

WARNING: step 5 and step 6 always run, even when an earlier step fails.
A failed recovery attempt must never leave the pins in a non-I2C mode or
leave the driver unbound.

The guard runs at most 6 recoveries per rolling hour (`--max-per-hour`).
Past that limit, it logs one error and keeps monitoring. It does not try
again until the hour window frees up.

## Self-heal: recovering from a kill

Steps 5 and 6 above run in a `finally` block in the code, which survives
almost anything -- an exception at any earlier step, a bad read, a bad
write. It does NOT survive the process being killed outright:

- A SIGTERM (what `systemctl stop` sends first) or a SIGINT (Ctrl-C) is
  caught by this guard. The signal only sets a flag; the guard finishes
  its current cycle -- including a `finally` block already running --
  and only then exits. A recovery in progress always completes cleanly.
- A SIGKILL (`systemctl kill -s KILL`, or the OOM killer) cannot be
  caught by any process, ever. If that happens mid-recovery, the pins can
  be left as plain GPIO and the driver can be left unbound.

To cover the SIGKILL case, every cycle -- including the very first one at
startup -- starts with a cheap self-heal check: is the driver bound, and
are both pins on the I2C (`a3`) alt function? If not, the guard restores
both pins to `a3 pu`, binds the driver if it was not already bound, and
logs a warning. This is idempotent: running it against an already-healthy
bus does nothing beyond the one check. It never unbinds anything -- if
the driver is already bound and only a pin drifted for some unrelated
reason, unbinding first would needlessly interrupt live traffic to fix
something that does not need it.

## After a recovery or self-heal

Unbinding and rebinding the `i2c_designware` driver breaks any I2C
connection a running program already had open on that bus, regardless of
whether the bus recovery itself succeeded.

- **The UPS daemon** (`upsplus-power.service`) opens its connection once,
  at startup. It never reopens that connection on its own. So this guard
  restarts that service -- unless you pass `--no-restart-ups` -- after
  ANY bus recovery attempt (success or failure) and after any self-heal
  that had to rebind the driver, as soon as the driver is bound again.
  Self-heal that only fixes a drifted pin, without ever unbinding, does
  NOT restart the UPS daemon -- its I2C handle was never touched. The
  guard logs every restart as a warning, because it is a real, visible
  action on the robot. A failed restart is logged as an error and does
  not count as a failed bus recovery -- the bus is already clear by that
  point.
- **`wall-e.service` is never restarted by this guard.** Its RTK GPS
  reader behaves in one of two ways:
  - If `wall-e.service` was already running when the bus got stuck, its
    GPS reader keeps retrying. It reopens its own connection on its own,
    after 10 consecutive read errors. No action is needed.
  - If `wall-e.service` started while the bus was still stuck, its GPS
    reader fails once, at startup, and then gives up. It does not retry
    again, ever, even after this guard clears the bus. GPS stays
    disabled until a person restarts `wall-e.service` by hand:
    `sudo systemctl restart wall-e.service`. Check `journalctl -u
    wall-e -n 200 --no-pager` for "RTK GPS init failed" to confirm this
    is what happened.

## UPS safety net

A stuck bus is not the only way the UPS daemon goes blind -- its own
process can simply crash, or never start. If the UPS status has been
stale for more than 120 seconds AND the bus itself reads healthy (SDA
high, SCL high -- confirmed not stuck), a wedged bus cannot be the cause,
so the guard restarts the UPS daemon anyway, on the chance that fixes it.
This is rate-limited to once every 15 minutes, separately from the
`--max-per-hour` bus-recovery limit, and logged as a warning each time it
fires. A missing status file never triggers this -- an unknown age is
not a stale one.

## Install

The install step is manual. Nothing installs this service automatically,
and nothing restarts it automatically after a code update either.

Run this on the Pi, with sudo:

```bash
sudo bash bin/install_i2c_bus_guard.sh
```

The script copies `bin/i2c-bus-guard.service` to `/etc/systemd/system`,
reloads systemd, enables the service, restarts it, and prints its status.
Run it again, the same way, after pulling any change to the guard --
`main` auto-deploys the checkout, but it never restarts this service on
its own. Restarting by hand, or re-running the installer, is what makes
the new code take effect:

```bash
sudo systemctl restart i2c-bus-guard
```

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
| `scl_level` | The last SCL reading this cycle: `hi`, `lo`, or `unknown`. |
| `ups_age_s` | Age of the UPS status file, in seconds, or `null` if unknown. |
| `ups_stale` | `true` if the UPS status age is over the stale threshold, for any reason. |
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
Add `--no-restart-ups` to skip every UPS-daemon restart trigger (a bus
recovery, a self-heal that rebinds the driver, and the stale-UPS safety
net), for example while you are also testing the UPS daemon itself.

## Appendix: Technical Names

`i2c_bus_guard.py`, `i2c-bus-guard.service`, `install_i2c_bus_guard.sh`,
`i2c-bus-guard`, I2C, SDA, SCL, GPIO2, GPIO3, `pinctrl`, `i2c_designware`,
`1f00074000.i2c`, RTK GPS, `RtkGpsReader`, UPSPlus, HAT,
`upsplus-power.service`, `wall-e.service`, `/tmp/ups_status.json`,
`/tmp/i2c_guard_status.json`, `UPS_STATUS_FILE`, open-drain, push-pull,
pull-up, STOP condition, START condition, SIGTERM, SIGINT, SIGKILL,
Ctrl-C, OOM killer, Raspberry Pi 5, Pi 5, systemd, `systemctl`,
`journalctl`, JSON, sudo, `--interval`, `--stale-s`, `--sample-count`,
`--sample-interval-s`, `--max-per-hour`, `--status-file`,
`--ups-status-file`, `--no-restart-ups`, `--dry-run`, `--once`.
