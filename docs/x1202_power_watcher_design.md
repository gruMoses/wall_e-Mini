# X1202 power watcher: design and install procedure (DRAFT, not approved)

Status: draft 3, 2026-11-13. Nothing is installed or pushed. Grok and an independent Opus reviewer examined this design in three rounds (refer to "Review history"). The bench results that this document calls "the bench" are in `docs/pi_power_architecture.md`, sections 7.5 to 7.8. The install is gated on the bench go/no-go in `docs/pi_power_architecture.md` section 7.6. Kevin decides the items in section 10. "Verify on the bench" marks each item that is not proven on this Raspberry Pi OS image.

## 1. Scope and non-goals

Scope: a script `bin/walle_power.py` (120 to 150 lines) and its unit `walle-power.service`; a `poweroff.target` drop-in; a new witness for `bin/i2c_bus_guard.py`; a new `/debug` reader; two changes to `bin/auto_deploy.sh`; the retirement of the old UPS watcher and of the bench logger.

Non-goals: no RTC alarm and no wake poll; no gauge state of charge; no second witness for an input loss; no late check that turns a power-off into a reboot; no active discharge; no change to `wall-e.service`, the control loop or the e-stop path.

| Feature | Lines (approximate) |
|---|---|
| GPIO 6 input with pull-down and read-back, 4 Hz, 10 s cumulative debounce | 20 |
| Low-voltage halt, tiers (a) and (b) | 15 |
| Halt sequence, re-check, cancel, power-off ladder, `halting` marker | 35 |
| Gauge VCELL read (lazy `smbus2` import, bus opened for each read) | 15 |
| Hold rule and fail-safe for GPIO 16 | 25 |
| Status file, `sd_notify`, journal warnings, expiring `nohalt` | 30 |
| Start checks (`POWER_OFF_ON_HALT`, pin read-back) | 10 |

## 2. Behaviour

The loop reads GPIO 6 at 4 Hz and VCELL at 1 Hz, and writes the status file at 1 Hz. All timers use `CLOCK_BOOTTIME`. Each subprocess has a 2 s timeout. An I2C read is a blocking `ioctl`, and Python cannot time it out: the kernel `i2c_designware` adapter timeout bounds it (about 1 s per transfer; verify on the bench), and the systemd watchdog is the backstop.

| State or event | Action |
|---|---|
| Input lost | 10 s of GPIO 6 low in the last 15 s: input-loss halt. |
| Input back during the debounce | The low time ages out. No halt. |
| Input back after the halt decision | The re-check before `poweroff` cancels, starts `wall-e`, clears the state. |
| Input back after `poweroff` | The Pi stays off. Operator rule (section 4). |
| Halt with the input present (e-stop, manual `poweroff`) | The Pi stays off. Operator rule. |
| Reboot | ExecStartPre sets both pins again. The board keeps power. |
| Tier (a) | GPIO 6 low and VCELL 3.30 V or less for 5 s: forced low-voltage halt. |
| Tier (b) | VCELL 3.10 V or less on 3 samples in a row, any GPIO 6 state: forced low-voltage halt. |
| Charge burst | Tier (b) stays active. A burst ends at once if VCELL falls below its value at the burst start. |
| Flat cells with the input present | Tier (b) halts at boot. Special operator rule (section 4). |
| Gauge unreadable | 3 failures: GPIO 16 high. After 60 s: FAULT each minute and on `/debug`. |
| GPIO 6 low with the input present | Halt (fail-off). The `nohalt` file is for fault-finding. It turns off the input-loss halt only, and it expires after 3 starts (section 7). |
| GPIO 6 pogo contact lost | The pull-down reads low: halt. |
| GPIO 6 stuck high, input lost | Tier (b) halts at 3.10 V. |
| GPIO 6 and gauge both lost | The pull-down reads low: halt. |
| Cells absent | Gauge fails or reads outside 2.5 V to 4.5 V: charge disabled, FAULT. |
| `pinctrl` fails for GPIO 16 | The pin stays an input and the board charges. Residual. FAULT `charge_pin` on `/debug`. |
| Script hang | `WatchdogSec=15` kills it; ExecStopPost sets GPIO 16 high; restart after 15 s. |
| Restart while `halting` | The new instance reads `/run/walle-power/halting` and continues the halt (section 3). |
| Start-up error (no `/dev/i2c-1`, no `smbus2`) | The GPIO 6 path runs without I2C. Gauge counts as unreadable. |
| Script not running | No halt on power loss. Hard cut when the cells are flat. Stale heartbeat on `/debug`. |

## 3. The halt rule

Debounce: 10 s of low in the last 15 s. The bench needs about 1 s to reject glitches. The other 9 s ride through a switch bounce or a quick off and on. A chattering input cannot reset a cumulative window.

GPIO 6 alone decides the input-loss halt. GPIO 6 pull: ExecStartPre and the script start set `pinctrl set 6 ip pd`. The script reads the pin back and shows FAULT `gpio6_cfg` if the pull is not set. The board drives the pin high against the pull-down (bench finding 5), so a lost contact reads low. Verify on the bench: measure the pin voltage with the input on, with the pull-down on and off, and with the Pi pressed down and released (a marginal contact). Pass: the high level stays well above the RP1 input threshold (for example 2.6 V or more) with the pull-down on. If it does not, use no pull (`ip pn`) and accept the residual that a lost contact can read high.

Input-loss halt:

1. Write `/run/walle-power/halting` (reason, start time, `poweroff_at`), then log the reason, VCELL and uptime.
2. `systemctl kill --signal=SIGKILL wall-e.service`, then `systemctl stop --no-block wall-e.service`. Poll `is-active` each 0.5 s for at most 15 s. The stop is necessary: `wall-e.service` has `Restart=on-failure` with a 5 s delay, and a kill alone starts it again. The stop of a dead unit returns at once and cancels the restart. There is no SIGTERM grace, as in the old watcher: the robot has no motor power, and the camera teardown is the code that can hang until the stop timeout (30 s for `wall-e.service` on the Pi, read on 2026-11-13).
3. Read GPIO 6 on 2 samples. If both are high, cancel: delete the marker, `systemctl start wall-e.service`, clear the state.
4. `systemctl poweroff --no-wall`, started as a child process. The script then enters the power-off ladder.

There is no explicit `sync`. A `sync` stuck in the kernel cannot be timed out, and the shutdown syncs anyway.

Low-voltage halt (tiers (a) and (b)): write the marker, then `systemctl poweroff --force` at once, then the ladder. `--force` skips the unit stop jobs. `systemd-shutdown` still kills all processes, syncs, and unmounts or remounts the file systems read-only. There is no cancel.

Power-off ladder: after the first call, the script does not wait passively. Its loop continues: it sends `WATCHDOG=1`, it keeps both tiers active (a tier during an input-loss ladder jumps to `--force` at once), and it retries. 30 s after the first call it runs `systemctl poweroff --force`. At 60 s it runs `systemctl poweroff --force --force` (this calls `reboot(2)` directly, with no unmount). The ladder covers a `poweroff` call that fails or times out before PID 1 queues a job: then `JobTimeoutSec` never starts. If the shutdown does run, it stops the script, and the ladder ends with it.

Watchdog during the halt: every wait in the halt path is a loop of at most 0.5 s steps with a `WATCHDOG=1` ping, and each subprocess has a 2 s bound. No blocking call without a bound sits between two pings.

Restart while `halting`: the new instance continues the halt. A crash or a watchdog kill cannot be prevented, and normal operation needs `Restart=always`. `/run` is cleared at each boot, so the marker cannot outlive the boot. With `poweroff_at` empty, an input-loss halt continues at step 2; otherwise the ladder continues from `poweroff_at`. A cancel deletes the marker.

Forced power-off in PID 1: `/etc/systemd/system/poweroff.target.d/10-walle-timeout.conf` sets `JobTimeoutSec=60s` and `JobTimeoutAction=poweroff-force`. On the Pi (systemd 252, read on 2026-11-13), `poweroff.target` has `JobTimeoutUSec=30min` and `JobTimeoutAction=poweroff-force`. The drop-in needs only `daemon-reload`. The timer starts when the job is queued. Verify on the bench: `systemctl show poweroff.target -p JobTimeoutUSec,JobTimeoutAction`, then a test unit with `ExecStop=/bin/sleep 600` and `TimeoutStopSec=infinity`. The Pi must go dark about 60 s after `poweroff`.

Hardware watchdog: not in the first install. Read on the Pi on 2026-11-13: the device is the Broadcom BCM2835 watchdog, its timeout is 15 s, its `max_timeout` attribute is 0, and it is inactive. `RuntimeWatchdogSec` is 0 and `RebootWatchdogSec` is the default of 10 min. A shutdown watchdog at that value can reset the Pi in a slow but healthy final stage, and with the input present the board then turns the power-off into a reboot. Bench test 10 decides it (decision 5).

Bound without the hardware watchdog: the userspace stop is bounded by `JobTimeoutSec` (input-loss halt) or skipped (`--force`). The ladder bounds a call that queued no job. The final stage (`systemd-shutdown`) has its own kill wait and a limited number of unmount retries; verify on the bench that the kill wait is the 30 s of the existing `DefaultTimeoutStopSec` drop-in. A flush to the NVMe that never finishes has no software bound. That is a residual.

Low-voltage thresholds: 3.30 V loaded (tier (a)) and 3.10 V (tier (b)). With the present cells, 4 cores pull a rested 3.96 V down to 3.42 V. With good cells the sag is 0.03 V to 0.05 V.

Time and energy budget from the start of the event to the board's cut:

| Step | Input-loss halt, typical / worst | Low-voltage halt, typical / worst |
|---|---|---|
| Detection | 10.6 s / 15.6 s | 3 s to 5 s / 5 s |
| `wall-e` kill and stop, re-check | 1 s / 16 s | none |
| Userspace shutdown | 3 s to 8 s / 60 s (`JobTimeoutSec`) | none (`--force`) |
| Final stage | about 2 s / 30 s (verify) | about 2 s / 30 s (verify) |
| Board cut after the PMIC turns off | 3 s | 3 s |
| Total | about 20 s to 25 s / about 125 s | about 10 s / about 38 s |
| Energy at 7 W (worst) | 875 J (0.24 Wh) | 270 J (0.07 Wh) |

A queued-job failure adds at most 60 s (the ladder). Two good 3 Ah cells hold about 0.6 Wh to 1.1 Wh at a loaded 3.30 V and about 0.2 Wh at 3.10 V (typical NMC curves, not measured). Both budgets close. The 7 W load is pessimistic after `wall-e` and the OAK stop.

## 4. The stay-off condition

The Pi stays off when the input returns before the board cuts its output, or after any halt with the input present. Recovery: input off for 4 s or more. The design is option 1, with operator rules:

1. "If WALL-E does not start within 60 s after the main switch goes on, set the main switch to off, wait 5 s, and set it to on."
2. "After an e-stop, set the main switch to off within some minutes." Else the board charges the cells to 4.2 V with the Pi off.
3. Flat cells (tier (b) at boot): "If WALL-E does not start and the board shows one or no battery LED, leave the main switch on for at least 15 min, then set it to off for 5 s and to on." A 5 s off time does not charge the cells above 3.10 V. With the Pi off and the input present, the board charges the cells. Verify on the bench: what the battery LEDs show at about 3.1 V with the Pi off, and the charge time from 3.0 V to a rested 3.4 V.

With the 10 s debounce and the re-check, rule 1 applies after an input-off time of about 12 s to 30 s.

Option 2 (a hook in `systemd-shutdown` that turns the power-off into a reboot) was dropped. Its marker survived a cancelled halt, so a later e-stop became a reboot; a flapping input could cause a reboot loop; the mechanism was not verified; and it did not remove the operator rules.

## 5. Charge policy

The requirement "cells never at 100 percent" is only partly met. Software holds the level while Linux runs. With the Pi off and the input present (e-stop, stay-off), the board charges to 4.2 V, and only hardware can change that. On the robot the input is absent while parked, so this state lasts only until the operator cycles the switch.

The hold rule ships OFF (GPIO 16 always high, driven as an output) until the soak result is read. Interim result, 2026-11-13 01:38: with GPIO 16 high and the input on, the rested VCELL decreased from 3.959 V to 3.921 V in 54 min, and the rate decreased (7 mV in 10 min at the start, 4 mV in 10 min at the end). The two branches:

- Branch A (the input carries the Pi with charging disabled): a burst starts when the input is good, the cells have rested 120 s, and VCELL is below 3.75 V. A burst lasts 120 s. It ends at once on an input loss, a gauge error, or a VCELL below its value at the burst start (charging must raise VCELL; a fall means the input is not there). Tier (b) stays active during a burst.
- Branch B (the cells carry the Pi with charging disabled): VCELL is a loaded reading. The hold stays off, and a manual 10 min burst is the only top-up. Tier (b) stays active during that burst, and the burst ends at once if VCELL falls below its value at the burst start. If the board does not move back to the input before the cells are low, branch B is not acceptable, and the hold goes back to Kevin. Note: with these cells (0.23 ohm), a loaded 3.10 V is a rested 3.6 V, so tier (b) could halt with the input present. New cells are a go/no-go item.

I2C fail-safe: 3 failed gauge reads in a row force GPIO 16 high. A burst does not resume after the gauge comes back.

Temperature: no cell temperature exists. The gauge has none and the board has no sensor. The 45 C requirement is NOT met. The smallest real option is a sensor on the cells, for example a DS18B20 (a part Kevin buys). It controls only the software bursts, not the boot-window charge or the charge with the Pi off. Until then, the default is "charge disabled".

Boot window (optional, not in the first install): `gpio=16=op,dh` in `config.txt`. It is not verified that the Pi 5 firmware applies `gpio=` to the RP1 header pins, or that the kernel keeps the state. The alternative is a `gpio-hog` device-tree overlay. Bench check: rest the cells at about 3.8 V with charging disabled, remove the input, apply it to a cold Pi, and log VCELL from the earliest unit (`DefaultDependencies=no`, which also runs `pinctrl get 16`). Compare 3 boots with the line and 3 without. A meter on GPIO 16 from power-on is the direct check.

Cycles: each boot charges for about 13 s and each halt on the cells discharges for about 20 s. These roughly cancel. If the first VCELL after a start is above 3.95 V, `/debug` shows "cells high".

State left behind: ExecStopPost sets GPIO 16 high after a stop, a crash, a watchdog kill and a failed start. With the Pi off, the board pull-down enables charging.

## 6. Status file and the bus-guard witness

Path: `/run/walle-power/status.json`, written atomically at 1 Hz and at each halt step.

Schema `walle-power/1`: `boot_id`, `hb_mono` (loop heartbeat), `bus_ok_mono` (last good bus-1 read), `bus_ok_ts` (wall time of that read, for people), `input_good`, `low_s`, `vcell_v`, `ext5v_v`, `charge` (`disabled`, `burst`, `failsafe`), `hold` (`off`, `on`), `state` (`run`, `debounce`, `halting`), `halt_reason`, `faults` (`gauge`, `nohalt`, `eeprom`, `cells_high`, `charge_pin`, `gpio6_cfg`), `last_error`. The two `_mono` values use `CLOCK_BOOTTIME`. `bus_ok_mono` does not change on a failed read.

Changes in `bin/i2c_bus_guard.py`:

- New witness: `bus_ok_mono` against the guard's `CLOCK_BOOTTIME`, when `boot_id` matches. A mismatch counts as absent.
- Old witness, until push 2: the old file `/tmp/ups_status.json`, field `ts` (wall clock, `time.time()`), age `now_wall - ts`.
- Choice: the guard uses the fresher of the two witnesses. If the new file is missing, it uses the old file. If both are missing, the age is unknown (today's behaviour). A stale new file with a missing old file stays stale.
- The guard never restarts `walle-power` while `hb_mono` is fresh. It only reports. The script opens the bus for each read, so an unbind and rebind need no restart. `UPS_SERVICE_NAME` becomes `walle-power.service` for a restart when the heartbeat is stale.

`/debug` (`pi_app/web/oak_viewer.py`): the same path order. Age from `hb_mono` with `boot_id`. A banner for each fault. The hint names `walle-power.service`.

## 7. Packaging

Push 1 files:

- `bin/walle_power.py`. The installer copies it to `/usr/local/lib/walle-power/`. The unit runs the copy, so an auto-deploy pull cannot change the running watcher.
- `bin/walle-power.service` and `bin/walle-poweroff-timeout.conf`.
- `bin/install_walle_power.sh`: copies the script and the unit, installs the drop-in and the existing 30 s and 10 s stop-timeout drop-ins, runs `daemon-reload` (`daemon-reexec` only when a `system.conf.d` file changes), enables and starts the unit, and stops with an error if `rpi-eeprom-config` does not show `POWER_OFF_ON_HALT=1`. It does not edit `config.txt`.
- `bin/retire_walle_ups.sh`: disables and stops both old units, removes their unit files, the hook `walle-ups-alarm`, `/var/lib/walle-ups`, `/run/walle-ups` and `/tmp/ups_status.json`, writes 0 to `/sys/class/rtc/rtc0/wakealarm`, and runs `daemon-reload`. It keeps the stop-timeout drop-ins.
- `bin/auto_deploy.sh`: (1) a HOLD for push 2, before the docs-only check. When `git diff --diff-filter=D --name-only "$LOCAL" "$REMOTE"` lists `bin/walle_ups.py`, the script holds unless `systemctl list-unit-files 'walle-ups*' --no-legend` succeeds and prints no line, and no `walle-ups*` file exists in `/etc/systemd/system`. A failed `systemctl` holds. The log line is `HOLD walle-ups still installed`. Read on the Pi on 2026-11-13: `systemctl is-enabled` for a missing unit prints an error and returns 1, and it does not print `not-found`; thus the HOLD does not use `is-enabled`. Verify on the bench: the diff direction, with a dry run on a test branch. (2) `bin` in the `compileall` smoke test.

Unit:

```ini
[Unit]
Description=WALL-E power watcher (Geekworm X1202)
After=local-fs.target
StartLimitIntervalSec=0

[Service]
Type=notify
TimeoutStartSec=10
WatchdogSec=15
User=root
RuntimeDirectory=walle-power
RuntimeDirectoryPreserve=yes
StateDirectory=walle-power
ExecStartPre=-/usr/bin/pinctrl set 16 op dh
ExecStartPre=-/usr/bin/pinctrl set 6 ip pd
ExecStart=/usr/bin/python3 /usr/local/lib/walle-power/walle_power.py
ExecStopPost=-/usr/bin/pinctrl set 16 op dh
Restart=always
RestartSec=15

[Install]
WantedBy=multi-user.target
```

- `Type=notify`: the script sends `READY=1` after its first loop pass and `WATCHDOG=1` on each pass, with a datagram to `$NOTIFY_SOCKET` (an address that starts with `@` is abstract). No library is needed. A watchdog timeout sends SIGABRT, runs ExecStopPost and restarts the script.
- `-` on the `pinctrl` lines: a failed `pinctrl` must not stop the watcher, because the halt matters more than the charge state. The script sets both pins again and reads them back (FAULT `charge_pin` or `gpio6_cfg`). ExecStopPost also runs after a failed start.
- No start limit, 15 s delay: a stopped watcher means a hard cut; a restart loop costs only journal lines.
- `RuntimeDirectoryPreserve=yes` keeps `/run/walle-power/halting` across a restart.
- `/usr/bin/python3` has `smbus2` (read on 2026-11-13; the bus guard uses the same interpreter). `pinctrl`, `wdctl` and `rpi-eeprom-config` are in `/usr/bin`.
- Verify on the bench: ExecStopPost after a watchdog kill (`journalctl -u walle-power`); the `READY=1` within 10 s.

`nohalt` override: `/var/lib/walle-power/nohalt` holds a boot count (default 3). It turns off the input-loss halt only. Tiers (a) and (b) stay on. The script decrements the count at each start and deletes the file at 0. A file that does not parse counts as 1. It gives a journal warning each minute and a `/debug` banner.

Tests (`pi_app/tests/test_walle_power.py`, pytest, fake hardware and clock) cover: the cumulative debounce; both tiers with every GPIO 6 state and during a burst, `--force`, no cancel; the halt order with pings and the cancel; the ladder with a fake `systemctl` that fails first; the `halting` marker after a restart; the `nohalt` count; start without I2C or `smbus2`; subprocess timeouts; pin read-back; the charge fail-safe, branch A and the burst end; the frozen `bus_ok_mono`; the guard witness choice and its no-restart rule; the auto-deploy HOLD with a failing `systemctl`.

Old tests: `test_walle_ups.py` goes in push 2. About 17 cases in `test_i2c_bus_guard.py` and 9 in `test_debug_board.py` change in push 1.

## 8. Retirement and install procedure

Gate: Grok and an independent Opus reviewer agree on this draft (refer to "Review history"). A change to the design needs a new review. Kevin approves push 1, each hand step and push 2. Steps 2 to 7 are on the bench. Keep the input on, and do not reboot, between steps 2 and 6.

Facts: on the X1202 the old watcher is inert. Its INA219 is gone, it never drives GPIO 16 and it never halts. Its wake check fails open. The bench logger is the only process that halts the Pi today.

1. Push 1. It restarts `wall-e.service`. Check: `git merge-base --is-ancestor <sha> HEAD && echo LANDED` on the Pi; `systemctl is-active wall-e`. Rollback: revert push 1 (allowed only before step 5).
2. Stop the bench logger, but keep it: `sudo systemctl stop x1202-bench-logger`, then `mv /home/pi/x1202_bench/ENABLED /home/pi/x1202_bench/ENABLED.off`. Without that file the logger is inert after a reboot. Restore: `mv /home/pi/x1202_bench/ENABLED.off /home/pi/x1202_bench/ENABLED && sudo systemctl start x1202-bench-logger`.
3. `sudo bash bin/install_walle_power.sh`. Check: `systemctl is-active walle-power`; `pinctrl get 6,16` shows `ip pd | hi` and `op dh`; `hb_mono` within 2 s of `/proc/uptime`; `systemctl show poweroff.target -p JobTimeoutUSec` shows 1 min. Rollback: step 3 rollback below, then the step 2 restore.
4. Bench tests 1 to 9 (section 9). On a failure: rollback.
5. `sudo bash bin/retire_walle_ups.sh`. Check: `systemctl list-unit-files 'walle-ups*'` lists nothing; no `walle-ups-alarm` in `/usr/lib/systemd/system-shutdown/`; `/sys/class/rtc/rtc0/wakealarm` is empty. From this step on, do NOT revert push 1: a revert brings back a guard that reads only the deleted old file, and it removes code that the installed watcher and the guard depend on. Fix forward, or roll back by hand.
6. Restart the guard only now, when the new witness exists: `sudo systemctl restart i2c-bus-guard`. Check: `/tmp/i2c_guard_status.json` shows a small `ups_age_s` and `ups_stale: false`.
7. Delete the bench logger unit and `/home/pi/x1202_bench/ENABLED.off`. Bench tests 10 to 15.
8. Buck, before the robot: confirm that the switched buck feeds ONLY the UPS board input and no other load. Record its current rating. Set it to about 12 V (the board accepts 6 V to 18 V). Use the 2-pin locking input connector, not the barrel jack. Restore for a return to the HAT (B): set the buck to 7.5 V before the HAT (B) is connected. Then the stand test (section 9).
9. Mount the Pi on the robot. Robot tests.
10. Push 2, after 7 days with no fault: delete `bin/walle_ups.py`, the old units, hook and installer, `test_walle_ups.py`, and the old-path code in the guard and the viewer. Update the stale documents. The auto-deploy HOLD blocks it while an old unit is installed.

Rollback by hand: stop the guard first (`sudo systemctl stop i2c-bus-guard`). Then `sudo systemctl disable --now walle-power`; remove the unit, the copy and the `poweroff.target` drop-in; `daemon-reload`. Start the guard only when a witness exists. `install_walle_ups.sh` is not a working rollback on the X1202. The working rollbacks are the bench logger (bench only) or the HAT (B) hardware with `install_walle_ups.sh` and the buck at 7.5 V.

## 9. Test plan

Bench, before retirement (relay outlet, not the dimmer):

1. Unit tests pass on the Mac and on the Pi.
2. GPIO 6 pin voltage with the pull-down on and off, input on, Pi pressed and released (section 3).
3. Input off 5 s, then on: no halt, 5 of 5.
4. Input off 60 s: halt, cut, input on, boot, 10 of 10. Record each step against the budget table. After the halt decision `wall-e` stays `inactive`.
5. Input off 15 s, 20 s and 25 s: record which cases stay off.
6. `poweroff.target` test unit: dark in about 60 s.
7. Ladder: a fake `systemctl` first in `PATH` for the script only, failing on the first call. Record `--force` at 30 s.
8. Watcher hang: `kill -STOP` the script during a manual burst. Within 15 s the watchdog kills it, GPIO 16 reads `op dh`, and the script restarts.
9. Restart while `halting`: `kill -9` the script during step 2 of a halt. The new instance continues the halt.

Bench, after retirement:

10. Hardware watchdog: `wdctl`, `max_timeout`, current values, and the 25 s SIGTERM test (section 3).
11. GPIO 16 toggles on the cells: 20 toggles, uptime and boot id unchanged.
12. Tier (a): run on the cells to 3.30 V. Also the low-cell boot (7.7 step 6) and the flat-cell LED check (section 4).
13. Soak result: choose branch A or B, with tier (b) in mind (section 5). Branch A: 24 h with the input on and the CPU load logged; rested VCELL stays 3.70 V to 3.85 V.
14. Storage drain (7.7 step 3). This is a go/no-go item. Above about 0.2 mA, add an operator rule: "for storage longer than N weeks, remove the cells", with N from the measurement.
15. `POWER_OFF_ON_HALT=0` once, with a meter on the 5 V output after a halt. This is an EEPROM write: a separate hand step with Kevin's approval, then set it back to 1 and read it back with `rpi-eeprom-config`.

Stand, before driving: a charge burst plus 4 cores plus the OAK streaming plus motor load, 10 min, GPIO 6 logged at 4 Hz, a meter on the buck output. Pass: no GPIO 6 low sample.

Robot, after the install:

1. Main switch off: dark in about 25 s. Switch on: `wall-e` up within about 60 s.
2. Main switch off 3 s, then on: no halt.
3. ch5 e-stop: the Pi stays off. Operator rule 1 starts it.
4. `/debug` shows the new fields and no banner. The guard is fresh.
5. RTK fix quality with the board in operation.
6. 7 days with no FAULT.

## 10. Risks, open questions, and Kevin's decisions

Risks, most serious first:

1. No cell temperature. The 45 C requirement is not met. A sensor cannot control the charge with the Pi off.
2. Storage: no hardware cutoff and an unknown drain (bench test 14 is a gate).
3. A false GPIO 6 low halts the robot. The Pi stays off until a person acts.
4. A watcher that is not running gives a hard cut. The alarms are a stale heartbeat and a guard warning.
5. A final stage that never ends (a stuck NVMe flush) has no software bound.
6. The power path with charging disabled is unknown until the soak. The hold stays off.
7. The present cells fail the load test (EXT5V 4.896 V), and their sag can fire tier (b) under branch B.
8. The wall clock is wrong at each start. The new code does not use it. Log names and the recorder clean-up do.

Kevin's decisions:

1. Option 1 and the three operator rules.
2. The debounce: 10 s in 15 s, or longer.
3. Fail-off on GPIO 6 (D1) and tier (b) unconditional (D2).
4. A cell temperature sensor (a part to buy), or no temperature rule.
5. The hardware watchdog (`RebootWatchdogSec`, `RuntimeWatchdogSec`), after bench test 10.
6. The hold branch, after the soak.
7. The `config.txt` or `gpio-hog` line, after the bench check.
8. An RTC cell for the Pi 5 (a part to buy).
9. The buck at about 12 V and the 2-pin connector.
10. The EEPROM test (bench test 15).
11. Approval of push 1, each hand step and push 2.

## Reviewer disagreements

D1 and D2 are closed. D3 is open for Kevin. D1 (a GPIO 6 that reads low with the input present): round 1 split; Grok wanted an alarm only, Opus wanted a halt. The ruling was fail-off, with an override file. In round 2 both reviewers agree with that ruling. Draft 3 adds an expiry to the override file. D2 (a low-voltage halt with GPIO 6 high): Grok wanted it unconditional, Opus wanted a probe on that branch. The ruling was tier (b) unconditional at 3.10 V. In round 2 both reviewers accept it. Opus still prefers the probe, but agrees that the cost is one operator cycle, not a safety cost. Opus finding 9 (a frozen gauge counts as unreadable) is not taken; Opus agrees in round 2.

D3 (the `nohalt` file and tier (a)): in round 3, Grok asked that tier (a) is off while the file is present, so that a fault-finding session with GPIO 6 stuck low cannot halt at 3.30 V. This draft keeps tier (a) on. Reason: a forgotten file together with a real input loss then halts at 3.30 V and not at 3.10 V, and a false GPIO 6 low alone cannot make VCELL low. Kevin can change this.

Clarification of the witness choice in section 6: "if the new file is stale, use the old file" would make a stale new file with no old file look unknown, and so never stale. Draft 3 uses the fresher witness and keeps "stale" in that case.

## Review history

- Round 1 on draft 1: Grok sessions 8ad0ce4e (sections 2, 3, 10) and 8c736335 (sections 4, 5, 6, 8), and an independent Opus reviewer (23 findings).
- Dropped in round 1: option 2 (the late-check hook), the charge-enable probe, the early-halt counter, the SoC temperature proxy.
- Added in round 1: the GPIO 6 pull-down, two low-voltage tiers, the `wall-e` stop, the `poweroff.target` drop-in, `Type=notify`, the installed copy, the auto-deploy HOLD, the buck step.
- Round 2 on draft 2: the same three reviewers. Result "not yet", with a short list.
- Added in round 2: the power-off ladder, the `halting` marker, `--force` for low-voltage halts, no explicit `sync`, the expiring `nohalt`, tier (b) during bursts, the bench logger kept until proven, the corrected rollbacks.
- Opus round 2: with the ladder done, no third round is needed.
- Round 3 on draft 3 (confirmation, Grok only): session 8ad0ce4e agrees, and asked for one sentence about the `nohalt` lifetime (added in section 2; refer to D3). Session 8c736335 agreed after one sentence about the manual burst in branch B (added in section 5).
- Read from the Pi on 2026-11-13, with no change to the Pi: the watchdog device, the `poweroff.target` values, the stop timeout of `wall-e.service`, the interpreter and the tools.

## Appendix: Technical Names

Geekworm X1202, SupTronics, Waveshare UPS HAT (B), INA219, MAX17040, IP2312, Raspberry Pi 5, RP1, PMIC, EEPROM, NVMe, OAK, RTK, Broadcom, NMC, 18650, DS18B20, GPIO, I2C, VCELL, EXT5V, `CLOCK_BOOTTIME`, `pinctrl`, `wdctl`, `rpi-eeprom-config`, `gpio-hog`, device-tree overlay, `POWER_OFF_ON_HALT`, `ioctl`, systemd, `systemd-shutdown`, PID 1, D-Bus, `sd_notify`, `reboot(2)`, tmpfs, `boot_id`, pytest, auto-deploy, HOLD, bench logger, pogo pin, pull-down, debounce, tier, burst, soak, ladder, buck, e-stop, Grok, Opus.
