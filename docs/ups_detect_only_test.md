# UPS daemon — DETECT-ONLY bench-test mode

Lets you exercise the **entire** charger-loss detection chain of
`scripts/upsPlus_power_daemon.py` — AC state transitions, the 4s loss
debounce, the 4s grace window, and grace expiry — with the daemon's own
shutdown actions suppressed, so you can unplug/replug the charger repeatedly
on the bench.

> 🔪 **The Pi is NOT immune to power loss in this mode.** DETECT-ONLY only
> suppresses the *daemon's* shutdown; it cannot stop the UPS firmware, which
> guillotines its own 5V output ~19-20s after input loss regardless (confirmed
> 2026-07-10 — see the "Firmware guillotine" section below). So an unplug left
> out past ~15-18s **will hard-cut the Pi** (dirty fsck), daemon suppressed or
> not. Keep detect-only unplugs short (replug within ~15s); the whole
> detect→grace-expiry chain you're testing completes by ~8-9s anyway.

> ⚠️ **SAFETY:** DETECT-ONLY *disarms* the robot's power protection. While it
> is enabled the Pi will **not** gracefully shut down on real charger loss and
> the battery could deep-discharge. This is a bench-test mode only. Never
> leave it enabled on a robot in service. The mandatory restore step is at the
> bottom of this doc — do it.

Related: `docs/ups_shutdown_bench_test.md` (the armed-mode validation the
daemon already passed twice) and the module docstring in
`scripts/upsPlus_power_daemon.py`.

---

## What DETECT-ONLY does (and does NOT do)

**⚠ Neither UPS firmware safety is suppressed by DETECT-ONLY — they live in the UPS MCU, not the daemon.** (1) The battery-protect floor (default 3.2 V/cell, regs 17/18) still cuts on deep discharge; while unplugged the Pi drains the cells (~5 A), so watch `batt=` in the log. (2) The **~20s output guillotine** (see below) hard-cuts the Pi on any unplug left out past ~18s. So: keep each detect-only unplug short (replug within ~15s), and after **any** battery stint **wait ~2 minutes** before assuming the UPS's internal clock is cleared — a quick replug does not reliably reset it. (Historical note: a 2026-07-10 shutdown during testing was initially blamed on the battery-protect cutoff but was actually wall-e's VESC low-voltage watchdog reacting to the main battery being switched off — see the VESC plausibility-floor fix. The cautions above remain valid prudence.)

## Firmware guillotine (confirmed 2026-07-10)

The UPSPlus EP-0136 (FW v14) **cuts its own 5V output ~19-20s after input
power is lost, regardless of battery charge** — a graceful-shutdown bridge,
not a ride-through UPS. Hardware-verified: a single 60s input cut with the
pack full (4.2V, trickle), in **this detect-only mode**, with zero register
writes and the countdown register at 0, still killed the output **19.5s**
after the USB-C input collapsed (three earlier deaths at ~28-30s cumulative
battery-time; every survived cut was ≤18s). A quick replug does **not**
reliably reset the internal clock.

**What this means for detect-only testing:** the mode suppresses the *daemon's*
shutdown, but it **cannot** stop this firmware cut. The detection chain you're
here to observe finishes fast — new time budget:

| Elapsed since unplug | Event (detect-only) |
| --- | --- |
| ~1s | instant detection; "would shed USB power (suppressed)" |
| ~4s | debounced "missing" — `Charger absent (debounced over 4.0s)` |
| ~8s | grace expiry — `would begin shutdown sequence NOW (suppressed)` |
| ~20s | **UPS firmware guillotine HARD-CUTS the Pi** (dirty) |

So **replug within ~15s** to see the full chain and repeat cleanly. Any input
cut **>18s WILL hard-cut the Pi** here, because the daemon that would normally
have halted it first is suppressed.


| Behavior | Armed (normal) | DETECT-ONLY |
| --- | --- | --- |
| AC present/absent detection + debounce | ✅ | ✅ identical |
| "state changed", "starting grace window", grace-expiry log lines | ✅ | ✅ identical |
| Startup banner announcing the mode | — (silent) | ✅ loud |
| Stop `wall-e.service` at grace expiry | ✅ | 🚫 suppressed |
| Shed USB load (OAK-D hub power off) | ✅ | 🚫 suppressed |
| Write UPS shutdown-countdown register | ✅ | 🚫 suppressed |
| `sync` + `shutdown -h now` | ✅ | 🚫 suppressed |
| Startup UPS register **writes** (Back-to-AC auto power-on, battery-protect threshold) | ✅ | 🚫 skipped |
| I2C **reads** (voltages, snapshot) | ✅ | ✅ still happen |

DETECT-ONLY is deliberately **strictly read-only against the UPS hardware** —
it performs **zero** register writes of any kind, including the two startup
writes the armed daemon normally makes. This guarantees the bench test cannot
alter UPS configuration. Each suppressed action is guarded at its own call
site, so all of the above hold even if one path is later refactored.

At grace expiry, instead of shutting down, DETECT-ONLY logs one unmistakable
line and keeps monitoring:

```
DETECT-ONLY: AC loss confirmed (grace expired) — would begin shutdown sequence NOW (suppressed). Continuing to monitor; replug to reset and repeat.
```

---

## How to enable on the Pi

**Deploy first:** the detect-only code must actually be at `/home/pi/bin/` before either option — `sudo cp ~/wall_e-Mini/scripts/upsPlus_power_daemon.py /home/pi/bin/upsPlus_power_daemon.py`. If you skip this and run a stale copy, the mandatory MODE banner check below catches it (no banner = stop).


Activated by **either** a CLI flag **or** an environment variable (either one
is sufficient; the daemon ORs them):

- CLI flag: `--detect-only`
- Env var: `UPS_DETECT_ONLY=1` (truthy = `1`/`true`/`yes`/`on`)

There are two ways to run it. **Recommended: Option A (foreground run)** — it
touches nothing persistent, so you cannot accidentally leave the robot
disarmed after a reboot.

### Option A — one-off foreground run (RECOMMENDED)

Stop the armed service, run the daemon by hand with the flag, watch it, then
Ctrl-C and restart the armed service when done.

```bash
# 1. Stop the armed watcher so only one daemon talks to the UPS at a time.
sudo systemctl stop upsplus-power.service

# 2. Run the live daemon in the foreground in DETECT-ONLY mode.
#    (Same interpreter + path the service uses.)
sudo /home/pi/.venv/bin/python3 /home/pi/bin/upsPlus_power_daemon.py --detect-only
#    …or, equivalently, via the env var:
#    sudo UPS_DETECT_ONLY=1 /home/pi/.venv/bin/python3 /home/pi/bin/upsPlus_power_daemon.py

# 3. Do your unplug/replug testing (see "What you'll see" below).
# 4. Ctrl-C to stop the bench run.
# 5. RE-ARM: bring the real service back (see "Restore armed mode").
sudo systemctl start upsplus-power.service
```

Why recommended: nothing survives the Ctrl-C. There is no drop-in to forget
and no reboot hazard — a reboot brings the normal armed service straight back.

### Option B — systemd drop-in with the env var

Use this if you want the mode to persist across the service's own restarts
(e.g. a long soak test) without editing `ExecStart`. The env var exists
precisely so a drop-in can enable the mode without touching the unit file.

```bash
sudo systemctl edit upsplus-power.service
```

In the editor add exactly:

```ini
[Service]
Environment=UPS_DETECT_ONLY=1
```

Then apply and restart:

```bash
sudo systemctl daemon-reload
sudo systemctl restart upsplus-power.service
```

⚠️ A drop-in is **persistent** and survives reboots. If you use Option B you
**must** remove the drop-in afterward (see "Restore armed mode"), or the robot
stays disarmed forever.

### Confirm the mode is active

Right after starting, the log must show the loud banner:

```bash
journalctl -u upsplus-power -n 20 --no-pager    # (Option B / service)
# or just read the foreground terminal              (Option A)
```

Expected:

```
================================================================
MODE: DETECT-ONLY — shutdown actions suppressed.
Bench-test mode: the Pi will NOT shut down on charger loss and NO UPS register WRITES are performed (strictly read-only against the hardware). ...
================================================================
DETECT-ONLY: skipped Back-to-AC auto power-on register write (reg 25) — read-only mode.
DETECT-ONLY: skipped battery-protection threshold register writes (regs 17/18, would be 3400mV) — read-only mode.
```

If you do **not** see `MODE: DETECT-ONLY`, the mode is **not** active — stop
and recheck the flag/env var before testing.

---

## What you'll see during a test (short unplug — replug within ~15s)

Unplug the charger; **replug within ~15s** (past ~18s the firmware guillotine
hard-cuts the Pi — see above). At the daemon's ~1s poll cadence you'll see, in
order:

1. After ~4s of continuous low charger voltage (the loss debounce), the state
   flips to missing and the grace window opens:
   ```
   UPS AC state changed -> missing (typec=…mV microusb=…mV batt=…V …mA)
   Charger absent (debounced over 4.0s); starting 4s glitch grace window.
   ```
2. ~4s later the grace window expires. In armed mode this is where it would
   shut down; in DETECT-ONLY it logs the suppression line and the per-action
   "suppressed …" lines, then **keeps running**:
   ```
   No charger for 4s (debounced); safe shutdown sequence begins. typec=…mV …
   DETECT-ONLY: AC loss confirmed (grace expired) — would begin shutdown sequence NOW (suppressed). Continuing to monitor; replug to reset and repeat.
   DETECT-ONLY: suppressed stop_rover_service() — wall-e left running.
   DETECT-ONLY: suppressed shed_usb_load() — USB power left on.
   DETECT-ONLY: suppressed UPS shutdown-countdown register write (reg 24 would be set to 30s).
   DETECT-ONLY: suppressed sync + '/sbin/shutdown -h now' — Pi stays up.
   ```
   All of this lands by ~8-9s — well inside the ~15s replug window.
3. The Pi stays up **only if you replug before ~18s**. The daemon's
   "would begin shutdown" line fires once per grace expiry (it will not repeat
   for the same unplug), but the UPS firmware will still guillotine the output
   at ~20s if you leave the charger out — the daemon suppression cannot prevent
   that.

### Confirming replug detection + repeatability

Plug the charger back in. Replug is recognized immediately (only *loss* is
debounced):

```
UPS AC state changed -> present (typec=…mV microusb=…mV batt=…V …mA)
Charger present again; resetting grace timer.
```

State is now reset. Unplug again and the whole cycle above repeats — including
a fresh `DETECT-ONLY: … would begin shutdown sequence NOW (suppressed)` line at
the next grace expiry — with **no restart needed**. Repeat as many cycles as
you want.

---

## Restore armed mode (MANDATORY — do not skip)

The robot is unprotected until you do this.

**If you used Option A (foreground run):**

```bash
# Ctrl-C the foreground daemon first, then:
sudo systemctl start upsplus-power.service
```

**If you used Option B (drop-in):**

```bash
sudo systemctl revert upsplus-power.service    # removes the drop-in
sudo systemctl daemon-reload
sudo systemctl restart upsplus-power.service
```

(`systemctl revert` deletes the override created by `systemctl edit`. If you
prefer, `sudo rm /etc/systemd/system/upsplus-power.service.d/override.conf`
then `daemon-reload` does the same.)

### Verification — the banner must be GONE

This is the critical check. Confirm the service is armed and the DETECT-ONLY
banner is absent from a **fresh** start:

```bash
systemctl is-active upsplus-power.service          # -> active
journalctl -u upsplus-power -n 30 --no-pager | grep -i "DETECT-ONLY"
```

The `grep` must return **nothing**. If it prints the banner or any
`DETECT-ONLY:` line from the current run, the mode is still enabled — the
robot is still disarmed. Re-check for a leftover drop-in:

```bash
systemctl show upsplus-power.service -p Environment    # must NOT list UPS_DETECT_ONLY
systemctl cat upsplus-power.service                    # must show no detect-only drop-in
```

Also confirm the armed daemon re-did its startup register writes (proves it's
truly back in armed mode, not silently read-only):

```bash
journalctl -u upsplus-power -n 30 --no-pager | grep -E "Back-to-AC auto power-on enabled|Battery protection threshold"
```

Only when the `DETECT-ONLY` grep is empty **and** you see the armed startup
lines is the robot's power protection restored.
