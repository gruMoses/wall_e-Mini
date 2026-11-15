# X1202 power watcher: design and install procedure (DRAFT, not approved)

Status: draft 3, 2026-11-13. Nothing is installed or pushed. Grok and an independent Opus reviewer examined this design in three rounds (refer to "Review history"). The bench results that this document calls "the bench" are in `docs/pi_power_architecture.md`, sections 7.5 to 7.8. NOTE, 2026-11-13: the results of the night are for the USB Type-C input of the board with a 5 V supply. The power-cycle tests were done again on the 6 V to 18 V input with a 12 V supply the same morning, with the same results (7.5, "Results on the 12 V input"). The install is gated on the bench go/no-go in `docs/pi_power_architecture.md` section 7.6. Kevin decides the items in section 10. "Verify on the bench" marks each item that is not proven on this Raspberry Pi OS image.

## 1. Scope and non-goals

Scope: a script `bin/walle_power.py` (120 to 150 lines) and its unit `walle-power.service`; a `poweroff.target` drop-in; a new witness for `bin/i2c_bus_guard.py`; a new `/debug` reader; two changes to `bin/auto_deploy.sh`; the retirement of the old UPS watcher and of the bench logger.

Non-goals: no RTC alarm and no wake poll; no gauge state of charge; no second witness for an input loss; no late check that turns a power-off into a reboot; no active discharge; no change to `wall-e.service`, the control loop or the e-stop path.

| Feature | Lines (approximate) |
|---|---|
| GPIO 6 input with pull-down and read-back, 4 Hz, 5 s cumulative debounce | 20 |
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
| Input lost | 5 s of GPIO 6 low in the last 7.5 s: input-loss halt. |
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

Debounce: 5 s of low in the last 7.5 s. Kevin decided this value on 2026-11-13, after the reviews (the reviewed draft had 10 s in 15 s). It is the same time as in the old watcher. The bench needs about 1 s to reject glitches. The other 4 s ride through a switch bounce or a quick off and on. A chattering input cannot reset a cumulative window. Kevin's reason for a short time: the converter from the main pack operates down to 30 V or less, thus a low pack under load does not look like an input loss. A shorter time is possible for the signal, but it moves the stay-off window to shorter interruptions (section 4).

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
| Detection | 6.2 s / 8.7 s (GPIO 6 falls 1.2 s after the loss on the 12 V input) | 3 s to 5 s / 5 s |
| `wall-e` kill and stop, re-check | 1 s / 16 s | none |
| Userspace shutdown | 3 s to 8 s / 60 s (`JobTimeoutSec`) | none (`--force`) |
| Final stage | about 2 s / 30 s (verify) | about 2 s / 30 s (verify) |
| Board cut after the PMIC turns off | 3 s | 3 s |
| Total | about 15 s to 20 s / about 118 s | about 10 s / about 38 s |
| Energy at 7 W (worst) | 830 J (0.23 Wh) | 270 J (0.07 Wh) |

A queued-job failure adds at most 60 s (the ladder). Two good 3 Ah cells hold about 0.6 Wh to 1.1 Wh at a loaded 3.30 V and about 0.2 Wh at 3.10 V (typical NMC curves, not measured). Both budgets close. The 7 W load is pessimistic after `wall-e` and the OAK stop.

## 4. The stay-off condition

The Pi stays off when the input returns before the board cuts its output, or after any halt with the input present. Recovery: input off for 4 s or more. The design is option 1, with operator rules:

1. "If WALL-E does not start within 60 s after the main switch goes on, set the main switch to off, wait 5 s, and set it to on."
2. "After an e-stop, set the main switch to off within some minutes." Else the board charges the cells to 4.2 V with the Pi off.
3. Flat cells (tier (b) at boot): "If WALL-E does not start and the board shows one or no battery LED, leave the main switch on for at least 15 min, then set it to off for 5 s and to on." A 5 s off time does not charge the cells above 3.10 V. With the Pi off and the input present, the board charges the cells. Verify on the bench: what the battery LEDs show at about 3.1 V with the Pi off, and the charge time from 3.0 V to a rested 3.4 V.
4. Storage (Kevin, 2026-11-15): "If WALL-E stays parked for more than 6 weeks, remove the two cells from the board." With the Pi off and the input off, the board takes 0.85 mA from the cells (bench test 14), and the board has no low-voltage cutoff.

With the 5 s debounce and the re-check, rule 1 applies after an input-off time of about 7 s to 25 s. An input-off time of less than 5 s causes no halt.

Option for Kevin, added on 2026-11-13 after the reviews: an external push button on the 2-pin button connector of the board. On the bench, the button on the board started a halted Pi while the 12 V input was present. A button on the body of the robot can thus start the Pi after an e-stop, and the main switch stays on. Not tested: the external connector, and a push while the Pi is in operation (the vendor says that the button does the same as the power button of the Pi, thus it can start a shutdown).

Option 2 (a hook in `systemd-shutdown` that turns the power-off into a reboot) was dropped. Its marker survived a cancelled halt, so a later e-stop became a reboot; a flapping input could cause a reboot loop; the mechanism was not verified; and it did not remove the operator rules.

## 5. Charge policy

The requirement "cells never at 100 percent" is only partly met. Software holds the level while Linux runs. With the Pi off and the input present (e-stop, stay-off), the board charges to 4.2 V, and only hardware can change that. On the robot the input is absent while parked, so this state lasts only until the operator cycles the switch.

The hold rule was OFF (GPIO 16 always high, driven as an output) until Kevin's decision of 2026-11-15. It is now ON: `bin/walle-power.service` starts the watcher with `--hold`. The default in the code stays off, thus a start without the option keeps the charge disabled at all times. The Pi gets a change of the unit only when `bin/install_walle_power.sh` runs again (a hand step). Soak result, 2026-11-13 (`docs/pi_power_architecture.md`, 7.5, findings 13 and 14): branch A applies. With GPIO 16 high and the input on, the input carries the Pi. The rested VCELL decreased from 3.981 V to 3.911 V in 1.67 h, and the rate decreased from 50 mV in 1 h to less than 10 mV in 1 h. Kevin decided on 2026-11-15, after bench tests 13 and 14 (decision 6): the hold is on. The result of 2026-11-13 was added after the reviews; it selects a branch that the reviewed draft already contains. The two branches:

- Branch A (the input carries the Pi with charging disabled): a burst starts when the input is good, the cells have rested 120 s, and VCELL is below 3.75 V. A burst lasts 120 s. It ends at once on an input loss, a gauge error, or a VCELL below its value at the burst start (charging must raise VCELL; a fall means the input is not there). Tier (b) stays active during a burst.
- Branch B (the cells carry the Pi with charging disabled): VCELL is a loaded reading. The hold stays off, and a manual 10 min burst is the only top-up. Tier (b) stays active during that burst, and the burst ends at once if VCELL falls below its value at the burst start. If the board does not move back to the input before the cells are low, branch B is not acceptable, and the hold goes back to Kevin. Note: with these cells (0.23 ohm), a loaded 3.10 V is a rested 3.6 V, so tier (b) could halt with the input present. New cells are a go/no-go item.

I2C fail-safe: 3 failed gauge reads in a row force GPIO 16 high. A burst does not resume after the gauge comes back.

Temperature: no cell temperature exists. The gauge has none and the board has no sensor. The 45 C requirement is NOT met. The smallest real option is a sensor on the cells, for example a DS18B20 (a part Kevin buys). It controls only the software bursts, not the boot-window charge or the charge with the Pi off. Kevin turned the hold on with this limit known (2026-11-15): a burst can charge warm cells. A burst is 120 s, and it starts only when the rested VCELL is below 3.75 V.

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
ExecStart=/usr/bin/python3 /usr/local/lib/walle-power/walle_power.py --hold
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
8. Buck, before the robot: confirm that the switched buck feeds ONLY the UPS board input and no other load. Record its current rating. Set it to about 12 V (the board accepts 6 V to 18 V). WARNING: do not make a 12 V lead with an XH 2-pin plug. Each 2-pin socket of the X1202 is an XH2.54 socket (the 12 V input, the two 5 V outputs and the button), and the 5 V inputs of the Pi 5 shield are the same. A 12 V XH plug fits the 5 V sockets and puts 12 V on the 5 V rail of the Pi. Kevin's decision, 2026-11-15: the 12 V goes into the barrel jack of the X1202 (5.5 mm x 2.1 mm), and the plug is held in the jack with a cable tie or is a right-angle plug. (Before 2026-11-15 this step said to use the 2-pin locking input.) Examine the polarity of the plug before the first power-on. If the plug comes out, the input is lost: the watcher halts the Pi 5 s later, and the robot stops. Restore for a return to the HAT (B): set the buck to 7.5 V before the HAT (B) is connected. Then the stand test (section 9).
9. Mount the Pi on the robot. Robot tests.
10. Push 2, after 7 days with no fault: delete `bin/walle_ups.py`, the old units, hook and installer, `test_walle_ups.py`, and the old-path code in the guard and the viewer. Update the stale documents. The auto-deploy HOLD blocks it while an old unit is installed.

Rollback by hand: stop the guard first (`sudo systemctl stop i2c-bus-guard`). Then `sudo systemctl disable --now walle-power`; remove the unit, the copy and the `poweroff.target` drop-in; `daemon-reload`. Start the guard only when a witness exists. `install_walle_ups.sh` is not a working rollback on the X1202. The working rollbacks are the bench logger (bench only) or the HAT (B) hardware with `install_walle_ups.sh` and the buck at 7.5 V.

## 9. Test plan

Bench, before retirement (relay outlet, not the dimmer):

1. Unit tests pass on the Mac and on the Pi.
2. GPIO 6 pin voltage with the pull-down on and off, input on, Pi pressed and released (section 3).
3. Input off 3 s, then on: no halt, 5 of 5.
4. Input off 60 s: halt, cut, input on, boot, 10 of 10. Record each step against the budget table. After the halt decision `wall-e` stays `inactive`.
5. Input off 8 s, 12 s, 16 s and 20 s: record which cases stay off.
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

1. Main switch off: dark in about 20 s. Switch on: `wall-e` up within about 60 s.
2. Main switch off 3 s, then on: no halt.
3. ch5 e-stop: the Pi stays off. Operator rule 1 starts it.
4. `/debug` shows the new fields and no banner. The guard is fresh.
5. RTK fix quality with the board in operation.
6. 7 days with no FAULT.

## 10. Risks, open questions, and Kevin's decisions

Risks, most serious first:

1. No cell temperature. The 45 C requirement is not met. A sensor cannot control the charge with the Pi off.
2. Storage: no hardware cutoff and an unknown drain (bench test 14 is a gate). Measured on 2026-11-15: 0.85 mA. Operator rule 4 (section 4) applies.
3. A false GPIO 6 low halts the robot. The Pi stays off until a person acts.
4. A watcher that is not running gives a hard cut. The alarms are a stale heartbeat and a guard warning.
5. A final stage that never ends (a stuck NVMe flush) has no software bound.
6. The power path with charging disabled is unknown until the soak. The hold stays off. CLOSED: branch A applies (soak of 2026-11-13), and the hold is on since 2026-11-15 (decision 6). New with the hold on: a burst can start at any time while the Pi runs, and the supply must then carry the Pi and the charge together (approximately 29 W, section 7.10 of `docs/pi_power_architecture.md`). The stand test of section 9 examines this and is necessary before a drive.
7. The present cells fail the load test (EXT5V 4.896 V), and their sag can fire tier (b) under branch B.
8. The wall clock is wrong at each start. The new code does not use it. Log names and the recorder clean-up do.

Kevin's decisions:

1. Option 1 and the three operator rules.
2. The debounce. DECIDED by Kevin on 2026-11-13: 5 s (5 s of low in the last 7.5 s).
3. Fail-off on GPIO 6 (D1) and tier (b) unconditional (D2).
4. A cell temperature sensor (a part to buy), or no temperature rule.
5. The hardware watchdog (`RebootWatchdogSec`, `RuntimeWatchdogSec`), after bench test 10.
6. The hold branch, after the soak. Result, 2026-11-15 (bench test 13): branch A operates correctly in a run of 5.8 h. DECIDED by Kevin on 2026-11-15: the hold is ON (`--hold` in the unit file). Reasons: the storage drain of 0.85 mA (bench test 14) takes approximately 0.5 percent of the cells each day, and with the `config.txt` line of decision 7 nothing else charges the cells while the Pi runs.
7. The `config.txt` or `gpio-hog` line, after the bench check. DECIDED by Kevin on 2026-11-14: add `gpio=16=op,dh` to `/boot/firmware/config.txt`. The line is on the Pi since 2026-11-14 23:00 (backup: `config.txt.bak-20261114-gpio16`). It is a hand step of the installation.
8. An RTC cell for the Pi 5 (a part to buy). Kevin, 2026-11-14: the clock unit (finding 3) stays installed if it passes its bench test. The RTC cell stays open.
9. The buck at about 12 V and the input connector. The connector is DECIDED by Kevin on 2026-11-15 (answer given in the session "Arduino and RC receiver schematics"): the barrel jack with a retention, not the XH input (section 8, step 8). The 12 V setting: Kevin set the buck to approximately 12 V before the stand test of 2026-11-15 (his report; the value was not measured in this session). The buck has a rating of 10 A and is on a 5 A fuse (Kevin, 2026-11-15, after the stand test). The stand test shows a limit on the robot that this rating does not explain ("Stand test on the robot").
10. The EEPROM test (bench test 15).
11. Approval of push 1, each hand step and push 2.
12. The storage rule. DECIDED by Kevin on 2026-11-15, after bench test 14: operator rule 4 (more than 6 weeks parked: remove the cells).

## Implementation and bench status (2026-11-14)

Status: the watcher is installed on the Pi on the test stand (Kevin's approval, 2026-11-14). The code is on `main` (commits `0ad7643` to `285ec62`). The old watcher is retired: `bin/retire_walle_ups.sh` ran on the Pi on 2026-11-14 at 22:08 (Kevin's approval). The I2C bus guard was restarted after that; it runs the new code and its witness is `walle-power`. The bench logger is stopped and kept (`/home/pi/x1202_bench/ENABLED.off`). `bin/walle_ups.py` and the code for the old status file are still in the repository (push 2, Kevin's approval). The charge hold is on since 2026-11-15 at 11:15: `bin/install_walle_power.sh` ran again after commit `ffa35d7` (Kevin's approval), and the status file shows `hold` = `on`.

Code review: Grok (sessions `29454134` for the watcher, `53dea5f0` for the guard, the deploy hold and the retire script) and an independent Opus reviewer examined the code in two rounds. All three agree that it is fit for the bench.

Differences from this design (each one came from a review or from the bench):

- Size: `bin/walle_power.py` has approximately 800 lines, not 120 to 150. The reviews added most of the difference. A shorter script is possible if Kevin removes functions.
- The re-check uses readable samples only. A pin that cannot be read for 2 s in the re-check causes the power-off.
- The debounce counts the time between readable samples (2.5 s maximum for each step). A failed read does not set the count to zero.
- The gauge read comes after the GPIO 6 path in each loop pass. `READY=1` goes out at the end of the start. The longest time between two watchdog messages is 9 s with each tool at its time limit.
- The command runner cannot wait without a limit. A main loop error does not stop the process; 20 failed passes in a row cause an exit and a new start.
- `bus_ok_mono` is the start time of the watcher until the first good read. The plausible VCELL range starts at 2.0 V.
- The `nohalt` file cannot stop the start: a file that cannot be read means "not active".
- The deploy HOLD compares the two file trees. On systemd 252, `systemctl list-unit-files` returns 1 when no unit file matches; the HOLD accepts that.
- New: a halt record. The watcher appends each halt event to `/var/lib/walle-power/events.jsonl` and syncs the file. Reason: on this board the system journal of a halted boot is lost (refer to the finding below).
- New: `cells_high` uses rested readings only and is a notice. The first reading after a start is taken during the charge of the boot.

Bench tests of section 9 with the installed watcher (bench supply at 12.2 V on the relay outlet, the two Molicel cells, the jumper, the camera on a USB port of the Pi; raw data in `docs/data/x1202_robot_20261114/watcher_bench_results.jsonl`):

| Test | n | Result |
|---|---|---|
| 3: input off for 3 s | 5 | No halt, and `wall-e` continues. The low time that the watcher counts is 3.75 s to 4.0 s, because the supply holds its output and GPIO 6 comes back 2.3 s after the input. In one more run the outlet stayed off for 5.7 s; the watcher halted the Pi, which is correct. |
| 4: input off for 40 s to 60 s | 5 | Halt each time. The halt decision is 5.8 s to 6.1 s after the outlet goes off, and the Pi is off the network 8.8 s to 10.8 s after it. The Pi boots with no other action when the input returns. |
| 5: input off for 8 s, 12 s, 16 s, 20 s | 1 each | 8 s and 12 s: the Pi halts and stays off (an input-off of 30 s starts it). 16 s and 20 s: the Pi halts and starts again. |
| 8: watcher stopped with SIGSTOP | 1 | The watchdog ends it after 16 s, the new process is active after 31 s, GPIO 16 stays `op dh`, no reboot. |
| 9: watcher killed during a halt | 2 | The new process continues the halt from the marker and powers off 16 s after the first halt decision. The halt record shows `resume`, `kill`, `stopped`, `poweroff`. |
| 6: `poweroff.target` time limit | 1 | A test unit with a stop command that does not end (`TimeoutStopSec=infinity`), then `systemctl poweroff` with the input on. The stop command stayed alive for 60 s, then systemd forced the power-off. Pass. The Pi then stays off (the stay-off condition); an input-off of 30 s starts it. |
| 7: power-off ladder | 1 | A `systemctl` that fails on the first `poweroff` call, in the `PATH` of the watcher only. Input off: halt decision, `poweroff` 0.5 s later (fails), `poweroff --force` 30.25 s after that. The Pi was off the network 43 s after the outlet went off. Pass. |
| 11: 20 toggles of GPIO 16 on the cells | 1 | The input was off and the Pi was on the cells with the load. The boot ID did not change, the uptime continued (245.62 s to 262.05 s), and `throttled` was `0x0`. Pass. |
| 12: tier (a) on the real cells | 1 | Load: 4 cores at 100 %, `wall-e` and the OAK camera. The input was off and the `nohalt` file disabled the input-loss halt. The rested VCELL at the start was 4.0575 V. VCELL reached 3.30 V after 43.5 min (2612 s) on the cells. The `tier` event shows VCELL 3.2975 V. The Pi was off the network 16.6 s after VCELL reached 3.30 V. On the cells, EXT5V was 5.080 V to 5.128 V, `throttled` was `0x0` in each sample, and the CPU temperature was 73.0 C to 78.5 C. Pass. |
| 12: boot on low cells | 1 | The input came back. The Pi booted with no other action. After the boot the rested VCELL was 3.515 V (3.52 V after 3 min), EXT5V was 5.147 V, `throttled` was `0x0` and the watcher showed no faults. This boot was the first with the `gpio=16=op,dh` line in `config.txt`, so the board did not charge the cells during the boot. |
| 13: charge hold (branch A), 5.8 h | 1 | The watcher ran with `--hold` from 00:10:48 to 06:00:00 on 2026-11-15 (a temporary drop-in; the test removed it at the end). The input was on, `wall-e` was in operation, and there was no added CPU load. The rested VCELL at the start was 3.585 V. 14 bursts of 120 s, with 120 s of rest between them, brought the rested VCELL to 3.7525 V in 56 min (7.5 mV to 21 mV for each burst). Then the watcher made no burst for 3 h 27 min, and the rested VCELL was 3.7500 V to 3.7563 V. One burst at 04:33:40 brought it to 3.77 V, and it was 3.7675 V at 06:00. During a burst VCELL was 3.91 V maximum. EXT5V was 5.112 V to 5.190 V, the watcher showed no faults, `throttled` was `0x0`, and the Pi did not reboot. After the cells reached the target, the rested VCELL stayed in the limits of the test plan (3.70 V to 3.85 V). The test plan asks for 24 h with the CPU load logged. This run is 5.8 h with no load log: a pass for 5.8 h, not the full test. Data: `hold_soak.log` and `hold_soak_samples.jsonl` in `docs/data/x1202_robot_20261114/night/`. |
| 14: storage drain | 1 | Kevin, 2026-11-15, on his bench. The Pi was halted (input-loss halt). The input and all cables of the robot were disconnected. The jumper was in place. One cell was removed. A resistor in series with the other cell (between the positive end of the cell and the holder contact) showed 85 mV on a Fluke 115. The resistor was 100 ohm (Kevin confirmed the value), thus the drain is 0.85 mA. A direct reading of 0.001 A on the 6 A range of the meter agrees with it. The drain is approximately 20 mAh in one day. From the hold level (3.75 V, approximately half of 4 Ah) the cells are empty after approximately 3 months. The limit of the test plan is 0.2 mA, thus the result needs an operator rule. Kevin accepted the rule on 2026-11-15: more than 6 weeks parked, remove the cells (section 4, rule 4). |
| 4, on the barrel jack: input off for 45 s | 2 | 2026-11-15. The bench supply at 12 V goes into the barrel jack (Kevin's decision 9), with the relay outlet in line. Halt each time: the halt decision was 5.8 s and 6.0 s after the outlet went off. The Pi booted with no other action, and the boot started 12.1 s and 12.3 s after the outlet went on (first ssh answer after 19.2 s and 40.0 s; the second boot had a slow Wi-Fi connection). Pass. The behaviour is the same as on the 2-pin input. |
| Charge burst with a full CPU load, on the barrel jack | 1 | 2026-11-15, the hold on. This is the bench part of the stand test (no motors, no camera). Load: 4 cores at 100 % and an NVMe read. First the Pi ran 186 s on the cells with this load (the `nohalt` file was active): the rested VCELL went from 3.766 V to 3.715 V, and EXT5V was 5.11 V to 5.17 V. Then the input came back. The watcher started a burst 122 s later, and the burst lasted the full 120 s. GPIO 6 was sampled at 4 Hz on the Pi: it was low only in the planned time on the cells, and it had no low sample in the 6.4 min after the input came back (approximately 1550 samples, 480 of them during the burst). EXT5V during the burst was 5.10 V to 5.16 V. There was no undervoltage flag. VCELL was 3.915 V at the end of the burst, and the rested VCELL after it was 3.7525 V. The supply current was not read (no camera on the supply; the limit of this supply is 3 A). Pass for the power path. Side finding: `throttled` showed `0x80000` and `0x80008` after approximately 4 min of this load. That is the soft temperature limit of the CPU, not a power fault. The fan turned (3391 rpm after the test). Data: `docs/data/x1202_robot_20261114/jack_20261115/`. |
| 1: unit tests on the Pi | 1 | 134 passed (before the last change; 147 on the Mac). |
| Robot's converter, no motion | 1 | 2026-11-15, 13:45. The Pi is on the robot, on the stand, not armed. The converter of the robot feeds the barrel jack (Kevin: set to approximately 12 V, rating 10 A, on a 5 A fuse). The OAK camera and the Arduino are on the USB ports of the Pi, and `wall-e` is in operation. Load: 4 cores at 100 % and an NVMe read for 177 s, with no charge burst. GPIO 6 at 4 Hz: 708 of 708 samples high. EXT5V was 5.05 V to 5.11 V (5.13 V to 5.20 V before the load). There was no undervoltage flag. The CPU temperature went from 49 C to 78.8 C in 3 min with the fan at 8500 rpm. Pass. Data: `docs/data/x1202_robot_20261114/stand_20261115/noload_samples_4hz.jsonl`. |
| Stand test with the motors (section 9), on the robot's converter | 1 | 2026-11-15, 13:53 to 14:11. FAIL for the pass condition "no GPIO 6 low sample" when the CPU load is full. Pass with the usual load of the robot. Refer to "Stand test on the robot" below. Data: `docs/data/x1202_robot_20261114/stand_20261115/`. |

Not done yet: test 2 (the pin voltage of GPIO 6 with the pull-down; needs a meter), test 10 (the hardware watchdog), the remaining time of test 13 (the hold soak is 24 h in the test plan; 5.8 h are done), test 15 (the EEPROM test; needs Kevin's approval), and the flat-cell LED check of test 12 (needs a person at the board).

Findings from the bench:

1. The stay-off window with the 5 s debounce is an input-off time of approximately 5 s to 14 s on this supply. Shorter: no halt. Longer: the Pi starts again. Operator rule 1 applies.
2. The time from the halt decision to the `poweroff` call is 0.5 s. The Pi is off approximately 4 s later.
3. The system journal of a halted boot is lost. No system journal entry between 2026-11-08 and 2026-11-14 is on the disk, although the Pi was in operation each day. A marker line that was synced to the disk was not there after the next start. The file system itself is clean (no ext4 recovery). Cause, examined on 2026-11-14 in the systemd 252 source and on the Pi (three steps):
   - The X1202 cuts the 5 V and the RTC has no cell, thus each boot starts with the clock at 2025-05-29 (the build date of systemd). `fake-hwclock` then restores a saved time. That time is old: it is saved at a clean stop and one time each hour, and it was 10 min old in the boots that were examined.
   - `journald` opens the journal file of the last boot approximately 1.7 s after the start. The last entry in that file is later than the clock. `journald` does not append to a file "from the future": it renames the file to `system@<clock>.journal~` (the log line is "corrupted or uncleanly shut down, renaming and replacing"). This occurs at each boot, also after a clean halt.
   - The journal directory is at its limit of 100 files (`SystemMaxFiles`, the default; 90 files are one user journal for each boot). `journald` then deletes the oldest files by name. A file that was renamed with an old clock is the oldest. The size limit (4 GB, 1.5 GB used) is not the cause.
   Until a correction is installed, `journalctl -b -1` is not reliable, and the halt record of the watcher is the only sure record of a halt. Corrections (Kevin's decision 8): an RTC cell (the Raspberry Pi RTC battery on connector J5; charging must be enabled with `dtparam=rtc_bbat_vchg=3000000`), or a small unit that sets the clock forward to the newest known time before `journald` opens the file (refer to "Clock unit" below), or the two together.
4. The cell level increases with each power cycle. The board charges for approximately 18 s at each start (12 s before the kernel, then until the watcher starts at 5.8 s), and a halt uses the cells for approximately 10 s. In the tests of 2026-11-14 the rested VCELL went from 3.89 V to 4.02 V in approximately 25 cycles. With the hold off, nothing brings the level down. This is decision 7 (drive GPIO 16 earlier) and decision 6 (the hold). Bench check of decision 7 (2026-11-14, 22:36 to 22:44, two power cycles for each case, a probe unit at 2.7 s to 2.9 s after the kernel start): without a `config.txt` line, GPIO 16 is not driven at that time and VCELL is 4.185 V (the charge is on; the rested value was 4.056 V). With `gpio=16=op,dh` in `config.txt`, GPIO 16 is `op dh` at that time and VCELL is 4.060 V, which is the rested value. The Pi 5 firmware applies the line to the header pin, and the kernel keeps the state until the watcher starts. The rested VCELL did not increase in the two cycles with the line (4.0575 V to 4.0587 V). The charge in the first seconds before the firmware reads `config.txt` stays. The line was removed after the check. Kevin then approved it, and it is in `config.txt` since 23:00 (decision 7).
5. For a low-voltage halt, the halt record has a `tier` line and a `halt` line, and no `poweroff` line (refer to the `events` in `tier_a_discharge.log`). The decision is recorded. The call is not.
6. The stay-off condition occurred one time in the tests of the night. At 23:45 the command "outlet off" got no reply in 10 s from the home dashboard, but the outlet went off. The test script then set the outlet to on. The Pi halted during that interval and stayed off with the input present. An input-off of 34 s started it. This is operator rule 1. The test harness now reads the outlet state and does not trust the reply (`tools/x1202_cycle_test.py`).
7. The cell level with the `gpio=16=op,dh` line: in the 10 power cycles of the clock test the rested VCELL went from 3.5925 V to 3.585 V (no increase). When the Pi is off and the input is on, the board still charges the cells. Between 23:40 and 23:53 the rested VCELL went from 3.52 V to 3.59 V; the probable cause is the 5 min of the stay-off condition above (the three boots in that interval had the line).
8. The board starts the Pi on the cells with no input. On 2026-11-15, on Kevin's bench with the input disconnected, the Pi started two times. Kevin pressed the button of the board by accident (his report; one press is confirmed, the cause of the second start is not recorded). It is not known if a cell that is put back starts the Pi. The watcher halted the first start 17.7 s after the kernel start (the input-loss rule; halt record of boot `d14d716d`). The second start has a `start` line and no halt line in the halt record, and its journal stops after 15 s: the power was cut (probable cause: a cell was removed for the drain test). The journal findings of that cut are in "Clock unit", Limits.

### Stand test on the robot (2026-11-15)

Setup: the Pi with the X1202 is on the robot, on the stand, with the tracks off the ground (Kevin's confirmation before the test). The converter of the robot feeds the barrel jack. The `nohalt` file was active for this boot (Kevin's approval), and it was removed after the test. A sampler on the Pi read GPIO 6 at 4 Hz. A second script on the Pi added a load of 4 busy cores and an NVMe read. The scripts sent no command to the motors: Kevin drove the tracks with the transmitter.

Sequence:

1. Kevin set the main switch to off for 225 s. The Pi stayed in operation on the cells. The script added the full CPU load, and stopped it after 70 s because the loaded VCELL was 3.4475 V (the limit of the script was 3.45 V). With the usual load of the robot, the loaded VCELL was then 3.56 V to 3.58 V. EXT5V on the cells was 5.07 V to 5.18 V.
2. Kevin set the main switch to on, armed the robot with the transmitter on, and drove the tracks for 326 s. The drive was forward at full speed (approximately 20 000 erpm) with few stops. The VESC bus current was -5.8 A to 9.5 A, and the bus voltage was 52.0 V to 55.3 V.
3. The watcher started charge bursts, and the script added the full CPU load during bursts 1, 2 and 3. Burst 4 had no added load.

Results:

| Burst | Motors | CPU load | Result |
|---|---|---|---|
| 1 (14:00:40) | in operation | full | Stopped after 38 s: GPIO 6 went low for less than 0.25 s. |
| 2 (14:03:18) | in operation | full | Stopped after 33 s: GPIO 6 went low for less than 0.25 s. |
| 3 (14:05:52) | stopped | full | Stopped after 34 s: GPIO 6 went low for less than 0.25 s. |
| 4 (14:08:26) | stopped | usual (`wall-e`, the camera, not armed) | The full 120 s. No low sample. VCELL went from 3.72 V to 3.9075 V. EXT5V was 5.14 V to 5.18 V. |

- The watcher was correct each time: it recorded `good: True -> False` and stopped the burst. The low time was much shorter than the debounce of 5 s, thus there was no halt condition (also without the `nohalt` file).
- In bursts 1 to 3, VCELL stayed at 3.82 V to 3.85 V and did not increase. The board decreased the charge current. On the bench supply, with the same CPU load and no camera, VCELL increased continuously to 3.915 V (test "Charge burst with a full CPU load, on the barrel jack").
- With the motors in operation and no burst, GPIO 6 had no low sample. In the 12.9 min after the input came back there were 3097 samples. Two were low (the ends of bursts 2 and 3). The low time at the end of burst 1 was shorter than one sample period; the watcher recorded it.
- EXT5V was 5.07 V to 5.20 V in the full test. `get_throttled` was 0x0. The maximum CPU temperature was 78.8 C.

Conclusions:

1. The motors do not disturb the input of the X1202 (forward, full speed).
2. On the robot, the input cannot carry the Pi with a full CPU load, the camera and the charge at the same time. The burst stops early, and the Pi continues without a fault. With the usual load, the burst is correct.
3. The cause is in the board, not in the supply (meter check below). The rating of the converter is not the cause: it is 10 A on a 5 A fuse (Kevin, after the test), and the estimated input current in the test is approximately 3 A. The 12 V at the barrel plug stays at 12.03 V or more. The probable cause is a load limit of the board, not heat in the board (analysis after Kevin's question, 2026-11-15; not confirmed):
   - Each of the 5 low signals came when the CPU temperature was 74 C to 77 C, although the time in the burst was different (22 s to 38 s). At that temperature the Pi takes its maximum power: a hot CPU takes more current, and the fan goes to full speed at 75 C.
   - The low signal stays until the watcher stops the charge. The watcher reads GPIO 6 each 0.25 s and recorded 5 of 5 low signals. A free signal of 0.1 s would be missed in approximately 60 % of the cases. After the charge stops, the signal is high again in 0.1 s. Thus the short low time is the reaction time of the watcher. It is not a cooling time.
   - With the same CPU temperature and no charge (the test "Robot's converter, no motion"), there was no low signal.
   - A thermal limit of the board is not excluded (no temperature of the board was measured), but the data do not need it. It is also not known if the limit is a stable limit or a short transient when the fan changes speed.

Not examined: a burst with the usual load while the motors are in operation; reverse and hard stops; the temperature of the board.

Meter check (2026-11-15, 15:05 to 15:18, Kevin's approval for the `nohalt` file; the robot was not armed):

- Kevin put a Fluke 115 on the 12 V at the barrel plug that goes into the X1202 (clips on the plug). Before the test: 12.14 V.
- Kevin set the main switch to off for 174 s and then to on. He reset the MIN MAX record of the meter after that. GPIO 6 was sampled at 10 Hz.
- Burst 1 (15:11:25, full CPU load): GPIO 6 went low after 26 s, one sample (approximately 0.1 s). Burst 2 (15:13:51, full CPU load): low after 22 s, one sample.
- The minimum on the meter after the two bursts: 12.03 V. Limit of this result: the low time is approximately 0.1 s, and the MIN record of the meter can be too slow for an event of that length. The stable voltage during the bursts is sure: the supply does not decrease under the load.
- Burst 3 (15:16:14, the usual load, directly after the two bursts above): the full 120 s, no low sample, VCELL to 3.9112 V.
- EXT5V was 5.06 V to 5.20 V. `get_throttled` was 0x0. The maximum CPU temperature was 80.4 C (on the cells, with the full load).
- The `nohalt` file was removed after the test, and the watcher shows no fault.

Totals for the day on the robot: a burst with the full CPU load stopped early 5 times of 5. A burst with the usual load was correct 3 times of 3 (one in the first test, one after it at 14:13, one in the meter check).

DECIDED (Kevin, 2026-11-15): this is accepted as a known limit of the board. A burst with a very high load stops early, and the watcher tries again after 120 s. The charge policy does not change. The alternative (no burst while the robot is armed) was not chosen: the low signal stops when the watcher stops the charge, thus it cannot become a halt condition by this cause; a burst is rare when the cells are at the hold level (approximately one in 5 h); and the watcher would need the arm state of `wall-e`, which adds code and a dependency to the unit that halts the Pi. Not examined, and still open as observations: a burst with the usual load while the motors are in operation, and operation in hot weather (the Pi then gets to its maximum power sooner, and more bursts stop early; the cells then get to the hold level more slowly).


### Robot checks after the install (2026-11-15, 15:46 to 16:11)

These are tests 1 to 4 of "Robot, after the install" in section 9. The Pi is on the robot, on the stand, on the converter of the robot. The `nohalt` file was not active. A script on the Mac recorded when the Pi answered (`switch_checks_monitor.log`). Kevin operated the main switch and the transmitter.

| Test | Result |
|---|---|
| 1. Main switch off, then on | Pass. The watcher made the halt decision after 5.00 s of low input (15:48:10), and called the power-off 0.5 s later. The Pi was off the network 9 s after that. Kevin's report: all lights dark after approximately 20 s to 30 s. Kevin set the switch to on after approximately 43 s. The Pi started with no other action. The web page of `wall-e` answered 17.6 s after the kernel start (approximately 30 s after the switch). No fault. The journal of the halted boot is readable. |
| 2. Main switch off for 3 s, then on | Pass. GPIO 6 was low from 15:50:14 to 15:50:18 (approximately 4 s). The watcher went to `debounce` and back to `run`. The boot id did not change, and there was no halt. The low time was near the limit of 5 s: an off time of 3 s at the switch gives approximately 4 s at the pin. |
| 3. ch5 e-stop | Pass, with one finding. Kevin armed the robot with the transmitter on and set ch5 to e-stop (15:53:56). `wall-e` disarmed and called `shutdown -h now` 1 s later. The Pi was off the network 8 s after the e-stop. Kevin then set the main switch to off and to on. The Pi started, but ch5 was still in the e-stop position: `wall-e` called the shutdown again 19 s after its start. The Pi then stayed off with the input present for approximately 13.5 min, until Kevin set the main switch to off and to on again (with the transmitter off). This is the stay-off condition and operator rule 1. Finding: in those 13.5 min the board charged the cells from 3.75 V to 3.92 V (VCELL 52 s after the start). This is operator rule 2: after an e-stop, set the main switch to off. |
| 4. `/debug` data and the guard | Pass. `/api/ups` (the source of the power panel) shows the source `walle-power`, the status available and not stale, the input present and no fault. `i2c-bus-guard` is active. The page itself was not examined on a screen. |

Notes:

- A boot with no network starts with the clock of the clock unit (the last known time). In the two boots after the e-stop, the clock was 2.5 min and 16 min behind the real time until the time server answered. The journal times of those boots are thus not real times.
- Not recorded: Kevin's confirmation that the Pi stayed dark for 30 s after the first e-stop with the main switch on. The 13.5 min after the second shutdown show the same condition.

Data: `docs/data/x1202_robot_20261114/stand_20261115/switch_checks_monitor.log`, `switch_checks_after_estop.log`.


### Clock unit

The clock unit is a oneshot unit, `walle-clock-floor.service`. It runs `/usr/local/sbin/walle-clock-floor` after `fake-hwclock.service` and before `systemd-journal-flush.service`. The script sets the clock forward, never back, to the newest of three times:

- the `fake-hwclock` file,
- the modification time of the `systemd-timesyncd` clock file,
- the last entry of the system journal plus 1 s.

The script ignores a value that is older than June 2025, or more than 1 h later than the modification time of its file. It always exits with 0. The unit has a time limit of 15 s.

Review: Grok (session `79728951`) examined the first draft. It rejected the order `Before=systemd-journald.service`, because it is not necessary and it delays the start of `journald`. That line was removed. Grok also said that `fake-hwclock load` sets the clock back. The Debian script sets it forward only, so this point was not accepted. `After=fake-hwclock.service` was added because it costs nothing.

Bench result (raw data: `clock_unit_run1_baseline.log` and `clock_unit_run2_10_cycles.log`):

| Case | Cycles | Journal renamed | Time to the first ssh answer |
|---|---|---|---|
| No unit | 2 | 2 of 2 (`ren=1`) | 21.7 s and 21.8 s |
| First version of the unit | 1 | 0 of 1 | 21.7 s |
| Final unit | 10 | 0 of 10 | 18.9 s to 21.8 s (9 of 10 cycles: 21.7 s to 21.8 s) |

With the final unit, the marker of the previous boot was readable in 10 of 10 cycles. It was also readable in the 2 cycles without the unit, so the rename count is the test, not the marker. The number of `.journal~` files was 13 and 14 without the unit and 14 in each cycle with the unit. The first version had no `RemainAfterExit=yes`. It ran 3 times in each boot and moved the clock 1 s forward each time. With `RemainAfterExit=yes` it runs one time in each boot, and it raises the clock by 1 s.

Status: the unit is installed on the Pi since 2026-11-14 at 23:44. Kevin decided on 2026-11-14 that it stays installed if it passes; it passed. Install it with `sudo bash bin/install_walle_clock_floor.sh`. The removal commands are in the header of that script.

Limits:

- The unit does not make the clock correct. On 2026-11-14 at 23:52 the clock of the Pi was 6.5 min behind the real time and `NTPSynchronized` was `no`. The clock is correct only after a time synchronization, or with an RTC cell.
- The test shows that the unit prevents the rename, which is the first step of the loss. The loss itself (a deleted file) did not occur in the 2 cycles without the unit: in those boots `fake-hwclock` restored a time in 2026, and the renamed file was not the oldest file.
- Not tested: a boot with all three time sources missing.
- A power cut without a halt occurred one time, on 2026-11-15 (finding 8). At the next boot `journald` renamed the journal of the cut boot, which is correct for a file that was not closed. The journals of the cut boot and of the two boots before it stayed readable (`journalctl --list-boots`). The unit ran one time and raised the clock by 1935 s (source: the clock file of `systemd-timesyncd`). This is one observation, not a test.
- The RTC cell stays Kevin's decision (decision 8).

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
