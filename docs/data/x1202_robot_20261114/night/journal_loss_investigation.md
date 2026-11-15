# Lost previous-boot system journal on the X1202 Pi (investigation + proposal)

> NOTE (2026-11-15): this is the investigation as written on 2026-11-14, before the review and the bench test. Three items changed after it: `Before=systemd-journald.service` was removed (Grok review), `After=fake-hwclock.service` was added, and `RemainAfterExit=yes` was added (without it the unit ran three times in each boot). The files that are installed are `bin/walle-clock-floor` and `bin/walle-clock-floor.service`. The bench result is in `docs/x1202_power_watcher_design.md`.


Date of work: 2026-11-14, about 22:10 to 22:25 CST. Read-only on the Pi (only /tmp used, for a dry run and a unit check). Nothing installed, nothing committed. The Pi power-cycled during the work; readings come from several boots, noted where it matters.

Tags: CONFIRMED = read in docs/source or measured on the Pi. REPORTED = a third party says so. INFERRED = my reasoning from the other facts.

## 0. Short version

The hypothesis in the brief is close but not the real trigger. The chain has three links:

1. The clock at the time journald opens the persistent journal is BEHIND the last entry in the old `system.journal`. journald refuses to append to a file "from the future", so it renames it to `system@<now>-<random>.journal~` ("corrupted or uncleanly shut down, renaming and replacing"). This happens at every boot, even after a clean halt, because the restored time can never be later than the last journal entry (see 2.1). CONFIRMED in source, consistent with every Pi measurement.
2. The renamed file is named with the (wrong, early) clock. When the clock is still 2025 at that moment the name sorts oldest of all.
3. journald's retention then deletes it first. The limit that bites is NOT disk size. It is the file COUNT: `SystemMaxFiles=100`, and the directory sits at exactly 100 files (90 of them are one-per-boot `user-1000@` journals from the night's power cycles). The vacuum deletes the oldest-sorted archived files until the count is 100 again. CONFIRMED (counts measured; limit from docs and source).

Not proven: how often fake-hwclock loses the race against the flush (see 1.7). On the two boots I saw, fake-hwclock won by about 1 ms, the clock was 2026 but 10 minutes stale, the rename still happened, and the renamed file survived (boot -1 had 1640 system entries, boot -2 had 909). So the loss is not 100 percent on every boot. It needs the 2025 clock (or an old name) plus the full count cap.

## 1. Facts from the Pi

### 1.1 journald configuration
`systemd-analyze cat-config systemd/journald.conf` shows only `Storage=persistent`. Everything else is default: `SystemMaxFiles=100`, `SystemMaxUse` capped at 4G (journald logs "max 4.0G, 2.4G free"), `MaxFileSec=1month`, `MaxRetentionSec` off.

### 1.2 Directory (at 22:10 and again at 22:13)
- Kinds at 22:10: 7 `system@...` (2 are `.journal~`), 1 `system.journal`, 92 `user-1000@...`, 1 `user-1000.journal`. About 101 entries; at 22:11 and 22:13 exactly 100 entries (`ls | wc -l` = 100). The count sits on the limit.
- Size is not the limit: 1.5 G used of 4.0 G allowed. Reported size falls each boot (1.5 G, then 1.4 G in journald's own line) because the vacuum removes the oldest big files: no 2026-09 files are left, while the 134 MB user journals of 09-29 and 10-01 were present at 22:10.
- At 22:10 the directory held two 2025 archives (`...-0006364948ce474d.journal`, `...-0006364948ce146e.journal`, head and tail 2025-05-29 12:04:02 to 12:04:05, ARCHIVED, 775 entries each, tail monotonic 5.4 s). One of them belonged to boot ID 4699f56a, which was the CURRENT boot at the time. By 22:11 both were gone (deleted by the next boot's vacuum) and a new `system@00066048b859c70a-....journal~` (8 MB, the previous boot's live file) had appeared. By 22:13 a second `.journal~` (`...b859dc9f...`) was there.
- The last real archive is `system@...e8dbdad-...journal`, last entry 2026-11-08 18:05. Nothing in between survives for 11-09 to 11-14 except the files above.
- The two 2025 archives show the second link: journald started a file at 12:04:02 (2025), the clock jumped to 2026 five seconds later, and `journal_file_rotate_suggested` rotated because the head entry was older than `MaxFileSec` (1 month). So each boot also produces a small 2025-named archive. CONFIRMED in source (2.1).

### 1.3 journald lines of the current boot (`-o short-monotonic`)
Boot at 22:11:
```
1.703 journald: Journal started
1.704 fake-hwclock[326]: Sun 15 Nov 04:01:47 UTC 2026         (= 22:01:47 CST)
1.705 systemd: Starting systemd-journal-flush.service
1.714 journald: Time spent on flushing to /var/log/journal/... 4.5ms for 598 entries
1.718 journald: Received client request to flush runtime journal.
1.720 journald: File .../system.journal corrupted or uncleanly shut down, renaming and replacing.
2.729 systemd: Starting systemd-timesyncd.service
2.763 timesyncd: System clock time unset or jumped backwards, restoring from recorded timestamp: Sat 2026-11-14 22:11:27 CST
```
Boot at 22:12 is the same, with the offset 0.17 s (fake-hwclock at 1.870, flush at 1.871, rename message at 1.881, timesyncd restore at 2.92).

What this shows:
- fake-hwclock ran 1 ms BEFORE the flush on these two boots. There is no ordering between them (see 1.6), so this is a race (the PIDs, 326/329 vs 330/332, fit a start-order accident). In the boots from the brief the order was the other way round: clock 2025 at flush.
- fake-hwclock restored 22:01:47 while the real time was about 22:11:27. That is 10 minutes stale. The saved file was written at the START of an earlier boot or at an earlier shutdown, and the later shutdowns did not run `fake-hwclock save` (power was cut hard, or the unit did not run). `/etc/fake-hwclock.data` still reads 2026-11-15 04:01:47 (22:01:47 CST) at 22:13.
- timesyncd restores a second step at 2.76 s from its clock file. That is much later than the flush at 1.7 s, so it cannot help the journal.

### 1.4 Previous-boot data really is on disk in the good case
At 22:13: `journalctl --system -b 0` 1334 entries, `-b -1` 1640 entries, `-b -2` 909 entries. These came from the `.journal~` files, which journalctl reads. So the rename alone does not lose data; the vacuum does.

### 1.5 Clock chain
- `/sys/class/rtc/rtc0`: name `rpi-rtc soc@107c000000:rpi_rtc`, time 00:00:23, date 1970-01-01, since_epoch 23. The RTC counts from 1970 after every 5 V loss. `timedatectl`: "RTC time: Thu 1970-01-01 00:00:23". `charging_voltage`, `charging_voltage_max`, `charging_voltage_min` and `battery_voltage` exist in sysfs (the driver supports a cell).
- dmesg: `rpi-rtc ...: setting system clock to 1970-01-01T00:00:10 UTC (10)` at 0.29 s.
- `/usr/lib/clock-epoch` does NOT exist. `/usr/lib/systemd/systemd` has mtime 2025-05-29 12:04:02, exactly the boot-time clock value. systemd 252.38-1~deb12u1. So PID 1 bumps the 1970 clock to its compiled-in build epoch `TIME_EPOCH`, 2025-05-29 12:04:02 (the Debian build date of this package). CONFIRMED for the mechanism (`clock_apply_epoch` in `src/shared/clock-util.c` uses the mtime of `/usr/lib/clock-epoch` if present, else `TIME_EPOCH`); the build-date match is by equal timestamp, INFERRED but near certain.
- `/var/lib/systemd/timesync/clock` is a zero-byte file whose mtime is the saved time. At 22:13 it read 22:12:40, which is the time timesyncd restored and re-touched at the start of the boot (touch at restore, then about every 60 s; INFERRED for the interval from the default `SaveIntervalSec`, the code path `manager_save_time_and_rearm` is CONFIRMED).
- `/etc/cron.hourly/fake-hwclock` exists (hourly save). `System clock synchronized: no` on these boots (no NTP reply in the first minute).

### 1.6 Unit ordering
- `fake-hwclock.service`: `DefaultDependencies=no`, `Before=sysinit.target shutdown.target`, `Conflicts=shutdown.target`, `WantedBy=sysinit.target`, ExecStart `fake-hwclock load`, ExecStop `fake-hwclock save`. `After=system.slice systemd-journald.socket` only (the "After" comes from `show`, implied by the default socket dependency on journald logging). NO ordering against `systemd-journald.service` or `systemd-journal-flush.service`.
- `systemd-journal-flush.service`: `After=systemd-journald.service systemd-remount-fs.service`, `Before=systemd-tmpfiles-setup.service`, runs `journalctl --flush`. It runs at about 0.25 s to 1.7 s; critical-chain: flush `+59ms`, journald.service `@250ms +51ms`, journald.socket `@232ms`.
- `systemd-timesyncd.service`: `After=systemd-sysusers.service` (and journald.socket, remount-fs, tmpfiles-setup), `Before=time-set.target sysinit.target`. It starts at about 2.7 s, after the flush.
- Debian's `fake-hwclock` script (0.12+nmu1): `load` sets the clock only if the saved time is later than now; `save` writes `date -u '+%Y-%m-%d %H:%M:%S'` (truncated to whole seconds). CONFIRMED from https://sources.debian.org/data/main/f/fake-hwclock/0.12+nmu1/fake-hwclock.
- A systemd-devel post reports the same missing `Before=systemd-journald.service` on Raspberry Pi: https://lists.freedesktop.org/archives/systemd-devel/2020-September/045243.html (REPORTED; the page returned 403 to my fetch, I only saw the search snippet).

### 1.7 The part I could not prove
I never saw a boot at the 2025 clock on this night, so "fake-hwclock loses the race" is REPORTED by the brief, not measured by me. Both boots I saw had fake-hwclock first. The software fix covers both orders.

## 2. Research

### 2.1 What systemd 252 does (read from the v252 tag, which matches Debian 12's 252.38)
- Opening an existing journal for append (`journal_file_verify_header`, `src/libsystemd/sd-journal/journal-file.c`): state ONLINE returns EBUSY ("Assuming unclean closing"); and "if tail_entry_realtime > now(CLOCK_REALTIME)" it returns ETXTBSY: "Journal file %s is from the future, refusing to append new data to it that'd be older." CONFIRMED. https://github.com/systemd/systemd/blob/v252/src/libsystemd/sd-journal/journal-file.c
- `managed_journal_file_open_reliably` (`src/journal/managed-journal-file.c`): for EBADMSG, ENODATA, EHOSTDOWN, EPROTONOSUPPORT, EBUSY, ESHUTDOWN, EIO, EIDRM and ETXTBSY it logs "File %s corrupted or uncleanly shut down, renaming and replacing." and renames the file. The code comment on ETXTBSY is "File is from the future". CONFIRMED. https://github.com/systemd/systemd/blob/v252/src/journal/managed-journal-file.c
  So a clean halt does not prevent the message. The old file's last entry is later than a clock restored from a once-per-second-truncated time that was saved before journald wrote its last lines. INFERRED link to the Pi, mechanism CONFIRMED.
- Rename name (`journal_file_dispose`): `system@%016x-%016x.journal~` with `now(CLOCK_REALTIME)` and a random 64-bit value. So the name carries the CURRENT clock, not the file's own dates. CONFIRMED. Measured: the 22:11 rename is `system@00066048b859c70a-...`, 0x66048b859c70a is 2026-11-14 22:01:47 CST, the stale restored clock.
- Rotation on the clock: `write_to_journal` rotates with "Time jumped backwards, rotating" when the realtime goes backwards, and when `journal_file_rotate_suggested(..., max_file_usec)` says so. `journal_file_rotate_suggested` returns true if `now > head_entry_realtime + max_file_usec`; the default `MaxFileSec` is one month. A forward jump from 2025 to 2026 therefore rotates the file that started at 2025. This produces the 2025-named archives. CONFIRMED (`journald-server.c`, `journal-file.c`, `DEFAULT_MAX_FILE_USEC USEC_PER_MONTH`).
- Retention (`journal_directory_vacuum`, `src/libsystemd/sd-journal/journal-vacuum.c`), CONFIRMED:
  - Only archived `.journal` and corrupt `.journal~` files are candidates. Active files (`system.journal`, `user-*.journal`) are never deleted. Other names are never touched.
  - Archived names give seqnum and realtime from the name. `.journal~` names give only realtime, from the first hex field (which is the dispose-time clock).
  - `patch_realtime` lowers that realtime to the earliest of the file's ctime, atime, mtime and birth time. A rename changes ctime, so a rename done at the 2025 clock pins the realtime in 2025.
  - Sort (`vacuum_compare`): if both files have a seqnum and the same seqnum ID, by seqnum; otherwise by realtime. This is not a consistent ordering for mixed files, so which old file goes first is not exactly predictable.
  - The loop deletes from the front of the list until `max_retention` (off), `max_use` and `n_max_files` are all satisfied. `left = n_active_files + n_list - i`, so active files count toward the file limit.
  - Empty files are removed regardless of limits.
  - `SystemKeepFree` is not part of this function.
- Docs: `SystemMaxFiles` default 100, "only archived files are deleted to reduce the number of files until this limit is reached". https://www.freedesktop.org/software/systemd/man/252/journald.conf.html (CONFIRMED). The user-1000 files live in the same directory and count (the man page does not say so; the source does, because the vacuum scans the whole directory).
- timesyncd: at start it moves the clock forward to the clock-file mtime (only if that is in the past); with no clock file it uses `/usr/lib/clock-epoch` mtime, else the build time. `time-set.target` is delayed until that is done. https://www.freedesktop.org/software/systemd/man/252/systemd-timesyncd.service.html (CONFIRMED). On this Pi timesyncd does it at 2.7 s, after the journal flush at 1.7 s.
- PID 1 raises a clock earlier than the epoch to the epoch (`clock_apply_epoch`): https://github.com/systemd/systemd/blob/v252/src/shared/clock-util.c (CONFIRMED).
- Known reports of previous-boot logs vanishing on systems with no RTC battery: the closest I found is the Raspberry Pi report above (wrong journal timestamps, missing journald ordering). I did not find a report of exactly this file-count loss. Mark the general pattern as REPORTED at best.

## 3. Software option (recommended)

Goal: make the clock at least as late as the last entry of the old `system.journal` BEFORE journald opens the persistent journal. The persistent file is opened when `systemd-journal-flush.service` asks journald to flush (1.7 s into boot). So the new unit orders itself `Before=` both `systemd-journald.service` and `systemd-journal-flush.service`. Before journald is best (entries get sane timestamps from the start); before flush is what makes the fix hold even if journald wins the start race.

Why not just reorder fake-hwclock? `Before=systemd-journald.service` on fake-hwclock fixes the 2025 case but not this one. The saved time is truncated to a second and saved before the last journal lines, or is stale after a hard cut (22:01:47 against a 22:11 reality, measured). The restored clock is then still earlier than the journal tail and the rename still happens.

Why not just timesyncd? It runs 1 s after the flush.

### Files

`/usr/local/sbin/walle-clock-floor` (mode 0755, owner root). Dry run tested on the Pi (output below):
```sh
#!/bin/sh
# walle-clock-floor: raise (never lower) the system clock to the newest
# trusted timestamp, before journald opens the persistent journal.
# Set WALLE_CLOCK_FLOOR_DRY=1 to print the decision without setting the clock.
MIN_SEC=1750000000          # 2025-06-15: anything older is a bogus value
SLACK=3600                  # a value may exceed its file's mtime by at most 1 h
now=$(date -u +%s)
best=$now
why="current clock"

# consider <seconds> <mtime-seconds-of-source> <label>
consider() {
    v=$1; m=$2; l=$3
    case "$v$m" in ''|*[!0-9]*) return;; esac
    [ "$v" -ge "$MIN_SEC" ] || return
    [ "$v" -le $((m + SLACK)) ] || { echo "ignoring $l: $v is far beyond its file time $m"; return; }
    if [ "$v" -gt "$best" ]; then best=$v; why=$l; fi
}

# A: fake-hwclock (saved at clean shutdown and hourly; seconds are truncated)
f=/etc/fake-hwclock.data
if [ -r "$f" ]; then
    consider "$(date -u -d "$(cat "$f")" +%s 2>/dev/null)" "$(stat -c %Y "$f" 2>/dev/null)" fake-hwclock
fi
# B: timesyncd clock file (its mtime is touched about every 60 s)
f=/var/lib/systemd/timesync/clock
if [ -e "$f" ]; then
    m=$(stat -c %Y "$f" 2>/dev/null)
    consider "$m" "$m" timesyncd-clock-file
fi
# C: last entry of the active system journal (the file journald will try to append to)
f=/var/log/journal/$(cat /etc/machine-id 2>/dev/null)/system.journal
if [ -r "$f" ]; then
    us=$(timeout 8 journalctl --no-pager --file="$f" -n 1 -o json 2>/dev/null \
         | sed -n 's/.*"__REALTIME_TIMESTAMP" *: *"\([0-9]*\)".*/\1/p' | head -n 1)
    case "$us" in ''|*[!0-9]*) ;; *) consider "$((us / 1000000 + 1))" "$(stat -c %Y "$f" 2>/dev/null)" journal-tail;; esac
fi

if [ "$best" -gt "$now" ]; then
    echo "raising clock by $((best - now)) s to $(date -u -d @"$best" '+%F %T') UTC (source: $why)"
    [ "${WALLE_CLOCK_FLOOR_DRY:-0}" = 1 ] || date -u -s "@$best" >/dev/null
else
    echo "clock already at or past every known timestamp (now $(date -u -d @"$now" '+%F %T') UTC)"
fi
exit 0
```

`/etc/systemd/system/walle-clock-floor.service`:
```ini
[Unit]
Description=Raise the clock to the newest known time before the persistent journal opens
Documentation=file:///usr/local/sbin/walle-clock-floor
DefaultDependencies=no
Before=systemd-journald.service systemd-journal-flush.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/walle-clock-floor
StandardOutput=kmsg
StandardError=kmsg
TimeoutStartSec=15

[Install]
WantedBy=sysinit.target
```
Enable with `systemctl enable walle-clock-floor.service` (symlink into `sysinit.target.wants`). Output goes to kmsg so that it does not need the journal socket before journald is up; journald then picks the line up as a kernel message.

Optional belt (see the last question of this section), `/etc/systemd/journald.conf.d/10-walle.conf`:
```ini
[Journal]
SystemMaxFiles=1000
```

### Tested on paper and on the Pi
- Dry run on the Pi at 22:13 (`sudo WALLE_CLOCK_FLOOR_DRY=1 sh`, script piped over ssh, nothing written): `raising clock by 1 s to 2026-11-15 04:13:00 UTC (source: journal-tail)`. The parse of the last journal entry works on a live ONLINE file. The 1 s is the "+1 so that the clock is never behind the tail" step.
- On the boots I observed (stale fake-hwclock 22:01:47, real time 22:11:27, journal tail about 22:10:26 to 22:11:27) candidates B and C would both lift the clock by about 10 minutes before the flush, so `tail_entry_realtime > now` is false and journald appends to the existing file.
- `systemd-analyze verify default.target` with the unit linked into a temp `sysinit.target.wants` in /tmp (`SYSTEMD_UNIT_PATH=/tmp/units:`): no ordering cycle reported. The only warning was that `/usr/local/sbin/walle-clock-floor` does not exist yet. The grep for "cycle" came back empty. (A weak check; the paper argument is the stronger one.)
- No cycle on paper: the new unit has no `After=` and no `Requires=`. A cycle needs an edge from something that is after the new unit back to something it must be after. `Before=systemd-journald.service` and `Before=systemd-journal-flush.service` only add edges out of the new unit. `WantedBy=sysinit.target` adds no ordering. journald is already `Before=sysinit.target`, so the order floor, then journald, then sysinit is consistent.
- `DefaultDependencies=no` is needed so the unit does not get `After=sysinit.target basic.target` (which would create a cycle with the Before lines).
- The unit reads `/etc`, `/var/lib` and `/var/log` while root may still be mounted read-only. Reading is enough; nothing is written (the journalctl call is read only).

### What happens on the first boot after installing
The boot after install: previous `system.journal` was written under the old regime and its tail is the previous boot's last entry (a 2026 time). The script lifts the clock to tail plus 1 s, journald's flush opens that file without the rename, and the old log continues in the same file. A few early entries from the runtime journal (stamped before the lift, if journald won the race) are older than the file's tail; journald copies them with their original timestamps. That can leave a handful of entries slightly out of order in the file. journald accepts them (no check on realtime ordering in the append path of v252: I found no `EREMCHG` or similar in `journal-file.c`; INFERRED, untested). The very first boot after install can still show one `.journal~` if the file was already disposed earlier. Old disposed files stay readable.

### If the saved time is wrong or missing
- Missing files (no fake-hwclock.data, no clock file, no journal): the script does nothing, the clock stays at the systemd epoch and the old behaviour returns, no worse than today.
- A saved time too EARLY: ignored, the script only raises the clock.
- A saved time too LATE (garbage in a file): each candidate must be at most 1 h later than its own file's mtime, and not older than June 2025; otherwise it is ignored with a log line. If a file had a future mtime AND a future value (a system that once ran with a wrong clock), the clock is set into the future until NTP steps it back (timesyncd steps; journald then logs "Time jumped backwards, rotating", which only rotates). No NTP in the field means a wrong date stays. This is the same trust that fake-hwclock already gets.
- The clock can end up up to 1 s ahead of real time; harmless.
- journalctl missing or hangs: `timeout 8` bounds it, the unit's `TimeoutStartSec=15` bounds the whole script, and the script always `exit 0`, so it can never block boot.

### Undo
```
sudo systemctl disable walle-clock-floor.service
sudo rm /etc/systemd/system/walle-clock-floor.service /usr/local/sbin/walle-clock-floor
sudo rm -f /etc/systemd/journald.conf.d/10-walle.conf
sudo systemctl daemon-reload
```
No reboot is needed to undo; no existing file is changed.

### Would raising `SystemMaxFiles` or setting `SystemMaxUse` help?
- `SystemMaxUse`: no. Usage is 1.5 G against a 4.0 G cap. The size limit is not what deletes files. Raising it does nothing; lowering it makes the loss worse.
- `SystemMaxFiles=1000` (or similar): it stops today's deletions, because the 100-file cap is the trigger, and it is a good safety net (also against the 90+ user journals of a night of power cycles). It only HIDES the cause. The rename on every boot would continue, each boot would leave a `.journal~` file with a misdated name and one 2025-named archive, and the file count would grow with every power cycle. Use it only together with the clock fix. A drop-in with `SplitMode=none` would also stop the one-user-journal-per-boot growth; I did not check what wall-e logs through the user journal, so I do not propose it.
- `MaxFileSec=0` would stop the 2025-archive rotation but not the rename. Not needed once the clock is right.

## 4. Hardware option

- Connector: J5 (BAT) on the Raspberry Pi 5, near the left corner of the board, right of the USB-C power connector. https://www.raspberrypi.com/documentation/computers/raspberry-pi.html (CONFIRMED, section "Real Time Clock (RTC)").
- Official part: the Raspberry Pi RTC Battery, a rechargeable lithium manganese coin cell, model ML2020, pre-fitted two-pin JST plug and an adhesive pad. https://www.raspberrypi.com/products/rtc-battery/ (CONFIRMED)
- The docs say it powers the RTC when the main supply is disconnected and that retention is "measured in months". Non-rechargeable lithium cells are not recommended and Lithium Ion cells must not be used. (CONFIRMED)
- Charging is OFF by default. It is a constant-current (3 mA) and constant-voltage charger. To enable, add `dtparam=rtc_bbat_vchg=3000000` to `/boot/firmware/config.txt` and reboot (3.0 V; limits 1.3 V to 4.4 V; check `/sys/class/rtc/rtc0/charging_voltage`, `charging_voltage_max`, `charging_voltage_min`). This Pi already exposes those sysfs files, so the driver supports it. (CONFIRMED)
- The RTC works without a battery but resets to 1970 on every 5 V loss, which is exactly what `timedatectl` shows now. (CONFIRMED)
- Weeks with the 5 V rail fully cut: the cell is the only supply then, so it must keep the RTC alive from its own charge. "Months" per the docs is the figure; a charged ML2020 over several weeks is within that (INFERRED). It only charges while 5 V is present. A cell that has gone flat after a long time off will leave the clock at 1970 again until it recharges, which the software fix still covers. I did not find a measured retention figure for this board with a cell at 5 V cut.
- Does it fix the journal loss alone? With the RTC right at boot the clock is later than the journal tail. A cut on the X1202 that lasts a few seconds loses no time. INFERRED. But it only works after the cell is fitted, charging is enabled and the first sync has set the RTC (the kernel writes the RTC when NTP is synced and at shutdown; the Pi OS handles that with its hwclock sync; I did not check this Pi's setup). It does not cover a flat cell.
- No purchase recommendation beyond the name above.

## 5. Bench verification (for either fix)

Run this on the stand with the power watcher in whatever state you want to test.

Before (baseline, 1 cycle): note `ls /var/log/journal/*/ | wc -l` (is it at 100?), `journalctl -b -1 --system | wc -l`.

Per power cycle:
1. `logger -t JTEST "marker N $(date +%s)"`, then `journalctl --sync`, then a clean halt (`sudo systemctl poweroff` or whatever the stand's script does), then power returns.
2. After the boot, check:
   - `journalctl -b 0 -u systemd-journald -o short-monotonic | grep -c "renaming and replacing"` must be 0 (it is 1 today).
   - `journalctl -b -1 --system | grep -c "marker N"` must be 1.
   - `journalctl -b -1 --system | wc -l` is in the hundreds, not 0 and not 775 (775 is the 5 s 2025 stub).
   - `ls -la --time-style=full-iso /var/log/journal/*/ | grep -v user-1000@`: no new `.journal~` file, no archive with a 2025 name (`-000636...` in the last field), the `system.journal` head timestamp is the previous boot's start, not this boot's.
   - `journalctl -k | grep walle-clock` (or `dmesg | grep -i clock-floor`): the line `raising clock by N s ... (source: ...)` or `already at or past`.
   - Boot order: `journalctl -b 0 -o short-monotonic -u fake-hwclock -u systemd-journald -u systemd-journal-flush -u walle-clock-floor` shows the floor line before "Received client request to flush runtime journal".
3. Count of cycles: at least 12 clean cycles in a row.

Extra cases (these are the ones that fail today):
- 3 hard cuts: remove the supply with no halt (Hubitat switch 98 or the stand's script). Check the same list for the boot after each. This is the case where `fake-hwclock.data` is stale and only the clock file or the journal tail can help.
- Stale-save case: after a clean boot, shut down cleanly, then check that `fake-hwclock.data` is not later than the journal tail (it should be earlier or equal by about a second); that is the case the "+1 s" handles.
- File-count case: keep the directory at its cap (at or near 100 files; it is there now) and run 10 more cycles. The previous-boot count must not fall to 0 for any boot. With the clock fix alone the cap should still trim only the oldest by real date (Oct and Sep files), not the previous boot. If you also add `SystemMaxFiles=1000`, the file count should grow instead and nothing should be deleted.
- Fail-open case: temporarily move `/etc/fake-hwclock.data` and the timesync clock file out of the way on the stand and confirm the boot still completes and the unit exits 0 (journal tail alone should still protect the journal).
- Failure signs to watch for: the unit shows "failed" in `systemctl status walle-clock-floor`; the clock jumps by more than a minute against a known good time; boot is slower than before by more than about 0.2 s (journalctl on the live file is fast; the dry run was quick).
- Undo test: remove the files with the undo steps above and confirm the old symptom returns (rename line back), to prove the fix is what changed the behaviour.

## Appendix: other observations
- `fake-hwclock` is only saved on a CLEAN stop (and hourly by cron). In the data I saw, `/etc/fake-hwclock.data` was 10 minutes stale after hard cuts. The X1202 stand's cuts therefore make fake-hwclock a weak source on its own, which is why the journal-tail source matters.
- The listing at 22:10 showed ~101 entries and later 100; I counted by name kind with `ls | wc -l` and kind buckets. The exact deleted file each time depends on the non-consistent sort noted in section 2.1.
- Collection scripts and raw output are in the scratchpad: `pi_facts.txt`, `walle-clock-floor.sh`, `walle-clock-floor.service`.
