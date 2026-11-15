# I2C bus 1 auto-unlock guard

`bin/i2c_bus_guard.py` watches I2C bus 1 for a stuck bus. It runs the
standard I2C bus-clear recovery when the bus is stuck, it repairs itself
after a kill signal cuts a recovery short, and it warns (and sometimes
restarts the power watcher) whenever the watcher's bus witness goes stale,
for any reason. It runs on the Pi as the systemd service `i2c-bus-guard`.

## Background: the 2026-09-25 incident

On 2026-09-25 at 21:36:16 the kernel logged this line:

```
i2c_designware 1f00074000.i2c: i2c_dw_handle_tx_abort: SDA stuck at low
```

A device on I2C bus 1 froze mid-transfer. The device held the SDA line
low. Two devices share bus 1: the RTK GPS co-processor (address 0x20) and
the UPS board of that date. Both devices went dark for 17 hours. A human power-cycled
the robot to clear the fault.

CORRECTION: on 2026-09-25 the UPS was the UPSPlus HAT and its watcher was `upsplus-power.service`. Both were retired on 2026-11-03. From 2026-11-03 to 2026-11-14 the UPS was the Waveshare UPS HAT (B) and its watcher was `walle-ups.service`. Since 2026-11-14 the power board is the Geekworm X1202 and the watcher is `walle-power.service` (`bin/walle_power.py`). The same bus failure applies to it. The guard now watches `walle-power.service`.

In the incident, the UPS daemon of that date logged an I2C read error every
10 seconds during the fault. Its status file went stale. Nothing on the Pi 5 cleared the stuck bus on its own. The Pi 5
kernel has no GPIO bus-recovery configured for this controller. This
guard clears the bus without a human.

## What it detects

Every cycle (`--interval`, default 5 seconds), the guard reads:

1. The age of the last good bus-1 read of the power watcher. The guard
   takes it from the status file `/run/walle-power/status.json`: the clock
   `CLOCK_BOOTTIME` now, minus the field `bus_ok_mono`. It uses the file only
   when the field `boot_id` is the boot id of this boot. A missing,
   unreadable or other-boot file counts as an unknown age, not a stale one.
   The watcher may simply be stopped. This is the only witness. (Until
   2026-11 the status file of the old `walle-ups` watcher, `/tmp/ups_status.json`,
   was a second witness. It is retired.)
2. SDA (GPIO2) and SCL (GPIO3) together, in one `pinctrl get 2,3` call.

The guard calls the bus stuck only when both of these are true:

- The age of the last good bus-1 read is more than 30 seconds (`--stale-s`).
- Every one of 10 consecutive joint samples, about 100ms apart
  (`--sample-count`, `--sample-interval-s`), shows SDA low AND SCL high.

Both conditions are required. SDA low with SCL high, held for a full
second, is the fingerprint of a wedged slave: it holds SDA down while the
clock sits idle. A live transfer toggles SCL, and a released bus reads
SDA high -- either one breaks the streak on its very next sample, so
ordinary I2C traffic is never mistaken for a stuck bus. A stopped
watcher alone never triggers a recovery either, because its status file's
age does not by itself mean the bus is stuck.

WARNING: a stuck bus is only one way the power watcher can go blind. The
guard also checks the witness age by itself, every cycle, regardless
of SDA/SCL. When that age is stale for any reason, it logs a warning:
safe shutdown on power loss is not active. This warning repeats at most
once every 5 minutes. A missing or unreadable status file is different
from a stale one: the guard logs that once, at a lower level, and does
not repeat it -- the watcher may simply be stopped.

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

- **The power watcher** (`walle-power.service`) opens the bus for each
  read, so a live watcher needs no restart after an unbind. The guard
  restarts it only when it is not alive. The guard never restarts it
  while its heartbeat `hb_mono` is fresh or unknown, or while the status
  file is absent or from another boot. In those cases it only reports.
  When the state in the file is "halting" and the heartbeat is fresh, a
  restart would interrupt the halt, so the guard does not do it. A stale
  heartbeat allows the restart in any state. The restarted watcher
  continues the halt from its marker. The guard reads the status file
  again at the moment of the restart, so it uses the heartbeat of that
  moment. The restart happens after ANY bus recovery attempt (success or
  failure) and after any self-heal that had to rebind the driver, as soon
  as the driver is bound again, unless you pass `--no-restart-ups`.
  Self-heal that only fixes a drifted pin, without ever unbinding, does
  NOT restart the watcher. The guard logs every restart as a warning,
  because it is a real, visible action on the robot. A failed restart is
  logged as an error and does not count as a failed bus recovery -- the
  bus is already clear by that point. (Until 2026-11 the target was
  `walle-ups.service`, which reopened its bus handle only after 10
  consecutive errors, so the restart was a second layer. It is retired.)
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

## Watcher safety net

A stuck bus is not the only way the power watcher goes blind -- its own
process can simply crash, or never start. If the witness age has been
stale for more than 120 seconds AND the bus itself reads healthy (SDA
high, SCL high -- confirmed not stuck), a wedged bus cannot be the cause,
so the guard tries to restart the watcher anyway, on the chance that
fixes it. The same heartbeat rule applies as above: a watcher with a fresh
or unknown heartbeat is not restarted. This is rate-limited to once every
15 minutes, separately from the `--max-per-hour` bus-recovery limit, and
logged as a warning each time it fires. A blocked restart does not use the
rate limit. A missing status file never triggers this -- an unknown age is
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
| `ups_age_s` | Age of the last good bus-1 read of the power watcher, in seconds, or `null` if unknown. |
| `ups_stale` | `true` if that age is over the stale threshold, for any reason. |
| `witness` | The witness in use this cycle: `walle-power`, or `null` if none. |
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
or `--power-status-file` to point at test paths instead of the real ones.
Add `--no-restart-ups` to skip every watcher restart trigger (a bus
recovery, a self-heal that rebinds the driver, and the watcher safety
net), for example while you are also testing the watcher itself.

## Appendix: Technical Names

`i2c_bus_guard.py`, `i2c-bus-guard.service`, `install_i2c_bus_guard.sh`,
`i2c-bus-guard`, I2C, SDA, SCL, GPIO2, GPIO3, `pinctrl`, `i2c_designware`,
`1f00074000.i2c`, RTK GPS, `RtkGpsReader`, UPSPlus, HAT,
`walle-power.service`, `bin/walle_power.py`, `/run/walle-power/status.json`,
`bus_ok_mono`, `hb_mono`, `boot_id`, `wall-e.service`,
`/tmp/i2c_guard_status.json`, `walle-ups.service` (retired),
`/tmp/ups_status.json` (retired), open-drain, push-pull,
pull-up, STOP condition, START condition, SIGTERM, SIGINT, SIGKILL,
Ctrl-C, OOM killer, Raspberry Pi 5, Pi 5, systemd, `systemctl`,
`journalctl`, JSON, sudo, `--interval`, `--stale-s`, `--sample-count`,
`--sample-interval-s`, `--max-per-hour`, `--status-file`,
`--power-status-file`, `--no-restart-ups`, `--dry-run`, `--once`.
