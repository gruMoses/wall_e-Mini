# UPS shutdown daemon — bench test procedure

Validates the fixed `scripts/upsPlus_power_daemon.py` before/after deploying it
to the Pi. Read this whole doc before touching anything — this daemon controls
a real, physical, graceful-shutdown path for the robot's Pi 5.

## Background: the bug being fixed

Diagnosed live 2026-07-05 from the running `upsplus-power.service` (no
crash-loop — `NRestarts=0`, it just made a wrong decision repeatedly). The
daemon inferred "charger present/absent" from the INA219 **battery current
sign**. On a topped-off pack the charge controller taper-cycles: it stops
pushing current into a full battery, the pack drifts to a slight discharge,
current sign flips negative for several seconds, then charging resumes. This
happens on a roughly 1-2 minute cycle even with the charger permanently
connected.

Live journal (`journalctl -u upsplus-power`), repeated for 2+ hours straight:

```
UPS AC state changed -> missing (typec=5048mV microusb=0mV batt=3.732V -3372.0mA)
Charger appears absent; starting 30s glitch grace window.
UPS AC state changed -> present (typec=4833mV microusb=0mV batt=4.1V +2370.0mA)
Charger present again; resetting grace timer.
```

`typec_mv` (USB-C charger input voltage) reads ~4.7-5.1V **the entire
time** — the charger was never unplugged. Only current sign flipped. Every
"missing" transition armed the 30s shutdown-countdown grace window
(`NO_CHARGE_GRACE_SECONDS`); it happened to always get cancelled ~8-9s later.
A sustained load, or a longer charge-taper window, would let that countdown
complete and **falsely shut down the Pi while genuinely on AC power**.

## The fix (what changed)

File: `scripts/upsPlus_power_daemon.py` (this repo). Full detail in the
module's docstring; summary:

- AC-present detection is now keyed **primarily on charger input voltage**
  (`typec_mv` or `microusb_mv` registers > `AC_PRESENT_VOLTAGE_THRESHOLD_MV`,
  currently **4000mV**) instead of battery current sign.
- Battery current sign is **no longer authoritative** — it's logged only as
  corroborating context.
- A new debounce, `AC_LOSS_DEBOUNCE_S` = **10 seconds**, requires charger
  voltage to stay continuously below threshold before "AC lost" is declared
  at all. A single bad reading or brief glitch cannot arm anything.
- The existing `NO_CHARGE_GRACE_SECONDS` (30s) and
  `UPS_SHUTDOWN_COUNTDOWN_SECONDS` (60s) behavior is **unchanged** — it now
  just triggers off the debounced signal instead of the raw current sign. A
  genuine, sustained power loss still shuts the Pi down cleanly; that
  protection is preserved, not removed.
- I2C read errors during the poll loop no longer risk flipping state on a
  transient fault — a failed read is skipped (with backoff) rather than
  treated as a data point.
- Startup hardware probe (I2C bus + UPS MCU + INA219) is wrapped so that if
  the hardware isn't present, the daemon logs a clear message and exits 0
  instead of crash-looping under `Restart=always`.

## Before you deploy anything

1. **Back up the Pi's boot media first.** This daemon can shut the Pi down
   and, in a bug scenario, could do so at a bad time. Take an image backup
   of the SD card or NVMe before deploying the new daemon:
   - Cleanest: power off the Pi, pull the SD/NVMe, image it from another
     machine (`dd` or Raspberry Pi Imager's "backup" function, or
     `rpi-clone` if you already use it).
   - If you'd rather not power-cycle the robot: at minimum confirm you have
     a recent known-good backup and skip to step 2, but full image-then-test
     is the safer path for a safety-critical daemon change.
2. Confirm you're doing this test with the robot **on the bench**, powered by
   the same charger/UPS setup, not out on a run — you'll be physically
   pulling the charger cable in step 3.
3. Have physical access to the charger cable and to the Pi (for a manual
   power check/recovery) for the whole test.

## Deploying the fixed daemon

Only do this when ready to test — this replaces the live daemon file and
restarts the service (both of which are outside what an assistant may do
unsupervised; a human runs these commands).

```bash
# From a machine with the repo checked out, copy the fixed daemon to the Pi:
scp scripts/upsPlus_power_daemon.py pi@192.168.86.54:/tmp/upsPlus_power_daemon.py

# On the Pi:
ssh pi@192.168.86.54
sudo cp /home/pi/bin/upsPlus_power_daemon.py /home/pi/bin/upsPlus_power_daemon.py.bak-$(date +%Y%m%d)
sudo cp /tmp/upsPlus_power_daemon.py /home/pi/bin/upsPlus_power_daemon.py
sudo systemctl restart upsplus-power.service
systemctl status upsplus-power.service --no-pager
```

Watch the first ~15 seconds of `journalctl -u upsplus-power -f` to confirm it
comes up cleanly: initial snapshot logged, "Back-to-AC auto power-on
enabled," battery protection threshold set, then "UPS AC state initialized:
present" (assuming the charger is connected, which it should be for this
whole test).

## Physical test sequence

### (a) No false "AC missing" flapping while on charger, ~10 minutes

With the charger connected and the Pi running normally (some background
load is fine — this is what exposed the original bug):

```bash
journalctl -u upsplus-power -f
```

Watch for at least 10 minutes. **Expected**: no "UPS AC state changed ->
missing" lines at all. The state should stay "present" continuously
regardless of what the battery current is doing — if you also tail battery
current via `read_ups_snapshot`-style logs or `journalctl | grep batt`,
you should still see the current sign wandering positive/negative on the
taper cycle, exactly like before, but it must **not** cause a state
change anymore. That's the fix working: current noise still exists, it's
just no longer load-bearing.

**Fail condition**: any "missing" transition during this window with the
charger still connected. If you see one, do NOT proceed to step (b) —
stop and re-examine (see Abort path below).

### (b) Deliberate charger pull — confirm real loss still shuts down

1. Start a fresh tail: `journalctl -u upsplus-power -f`
2. Physically disconnect the main charger input (unplug the USB-C/micro-USB
   feeding the UPS HAT — not just an intermediate cable, the actual wall/PSU
   side).
3. Expected timeline:
   - Within ~1-2s: charger voltage registers should drop toward 0.
   - After `AC_LOSS_DEBOUNCE_S` (10s) of continuously-low voltage: log line
     "Charger absent (debounced over 10.0s); starting 30s glitch grace
     window."
   - After a further `NO_CHARGE_GRACE_SECONDS` (30s): log line "No charger
     for 30s (debounced); safe shutdown sequence begins," followed by
     `wall-e.service` stop, USB shedding, UPS shutdown countdown set to
     `UPS_SHUTDOWN_COUNTDOWN_SECONDS` (60s), then `sync` and
     `shutdown -h now`.
   - Total time from physical unplug to shutdown command: roughly 40-45
     seconds (10s debounce + 30s grace + a couple seconds of overhead).
   - Pi actually powers off some time within the 60s UPS countdown window
     after the shutdown command (OS halt itself is typically much faster
     than that — ~10-20s; the 60s is the UPS's own hard-cutoff backstop).
4. **This is the safety behavior working correctly** — a real, sustained
   power cut must still result in a clean shutdown. Do not interpret this as
   a bug.

### (c) Repower and confirm clean boot

1. Reconnect the charger.
2. With "Back-to-AC auto power-on" enabled (the daemon sets this at every
   startup), the UPS should power the Pi back on automatically. If it
   doesn't within a minute or two, use the physical power button on the
   UPS HAT.
3. Confirm the Pi boots cleanly:
   - SSH back in once it's up: `ssh pi@192.168.86.54`
   - Check filesystem health implicitly by confirming normal boot (no fsck
     prompts hanging at a console, services come up).
   - `systemctl status upsplus-power.service wall-e.service --no-pager` —
     both should be active/running.
   - `journalctl -u upsplus-power -n 20 --no-pager` — should show a fresh
     "UPS initial snapshot" and "UPS AC state initialized: present" (charger
     is back), with no errors.

## Abort path

If anything looks wrong at any point:

- **During (a)** — a false "missing" still appears: stop testing, do not
  proceed to (b). Revert to the backed-up daemon:
  ```bash
  ssh pi@192.168.86.54
  sudo cp /home/pi/bin/upsPlus_power_daemon.py.bak-YYYYMMDD /home/pi/bin/upsPlus_power_daemon.py
  sudo systemctl restart upsplus-power.service
  ```
  Then bring the journal excerpt back for further diagnosis.
- **During (b)** — if the countdown does NOT fire after ~45s of sustained
  charger loss (i.e., the fix broke real shutdown protection): reconnect the
  charger immediately, then revert to the backup as above and re-diagnose
  before trying again. Do not leave the robot in a state where genuine power
  loss won't trigger a graceful shutdown.
- **At any point** you're unsure what state the daemon or UPS is in: the
  UPS's own hardware shutdown-countdown register (`REG_SHUTDOWN_COUNTDOWN`,
  register 24) and battery protection threshold are independent hardware
  safety nets that don't depend on the daemon continuing to run once set —
  but if you need to cancel an in-flight countdown, reconnecting the charger
  before the daemon's own AC-present re-check does NOT itself clear an
  already-armed UPS-side countdown register; the safest recovery is to let
  the countdown complete (Pi shuts down, UPS repowers it via Back-to-AC) and
  confirm clean boot per step (c).

## Known-good baseline (for comparison)

`journalctl -u upsplus-power -n 120 --no-pager` captured 2026-07-05 with the
**old** (buggy) daemon shows the flapping pattern roughly every 2 minutes for
the entire ~2.5 hour observed window, e.g.:

```
Jul 05 16:58:33 walle python3[1716]: ... UPS AC state changed -> missing (typec=4989mV microusb=12mV batt=3.744V -4627.8mA)
Jul 05 16:58:33 walle python3[1716]: ... Charger appears absent; starting 30s glitch grace window.
Jul 05 16:58:42 walle python3[1716]: ... UPS AC state changed -> present (typec=4828mV microusb=3mV batt=4.076V 1358.0mA)
Jul 05 16:58:42 walle python3[1716]: ... Charger present again; resetting grace timer.
```

Note `typec` stays ~4.7-5.1V across every single one of these transitions —
that's the smoking gun that voltage was fine and only current sign moved.
After deploying the fix, this pattern (state changes with steady typec)
should disappear entirely; only genuine unplug/replug events should produce
state-change log lines.

## Known non-issue found during investigation (no fix needed)

`run_main.sh`'s `check_and_start_ups()` calls `i2cdetect` to decide whether
to `systemctl enable/start upsplus-power.service`. `i2cdetect` lives at
`/usr/sbin/i2cdetect` on this Pi. Under `wall-e.service`'s systemd
`ExecStart`, the process PATH is systemd's compiled default
(`/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin`), which
includes `/usr/sbin` — so `i2cdetect` resolves and the probe works. Verified
live: the most recent `wall-e.service` journal shows `"UPS hardware detected
at 0x17, starting service"` followed by a successful `enable`+`start`. This
is a non-issue in the current environment; it would only bite if something
ran `run_main.sh` from a shell with a stripped-down `PATH` lacking
`/usr/sbin` (e.g. certain minimal `sudo` or cron contexts) — not the case for
the systemd path this actually runs under today. Regardless,
`upsplus-power.service` is separately `enabled` at the systemd level
(`WantedBy=multi-user.target`), so it starts at every boot independent of
`run_main.sh`'s probe — the probe is redundant belt-and-suspenders, not the
sole path to the service running.
