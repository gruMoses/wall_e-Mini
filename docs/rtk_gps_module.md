# RTK GPS module (DFRobot KIT0198)

This document describes the register map and read protocol
`pi_app/hardware/rtk_gps.py` uses, and the fields it now logs
(2026-09-19, Commit D of the logging audit).

## Hardware

The receiver is a DFRobot GNSS-RTK rover kit (KIT0198): a Quectel LC29HDA
GNSS module behind an I2C co-processor at address `0x20`. The reader
talks to the co-processor's register interface, not the LC29HDA directly.

## Source of the register map

The register map and the chunked-sentence read protocol are vendored from
DFRobot's own Python driver:
`DFRobot/DFRobot_RTK_LoRa`, file
`python/raspberrypi/DFRobot_RTK_LoRa.py`
(<https://github.com/DFRobot/DFRobot_RTK_LoRa>). `pi_app/hardware/rtk_gps.py`
reimplements the relevant registers directly against `smbus2` rather than
importing the vendor file, so it stays dependency-light and testable
without the vendor package installed.

## Register map (the subset this driver uses)

| Register(s) | Field | Notes |
|---|---|---|
| 0-1 | UTC year (hi, lo) | `year = reg0*256 + reg1`; `0` means no time fix yet |
| 2 | UTC month | |
| 3 | UTC date | |
| 4 | UTC hour | |
| 5 | UTC minute | |
| 6 | UTC second | Used to dedupe polls onto GPS epochs — see below |
| 7-12 | Latitude | `dd`, `mm`, 3-byte `mmmmm` (big-endian), direction char (`N`/`S`) |
| 13-18 | Longitude | Same shape as latitude, direction char `E`/`W` |
| 19 | Fix quality | `0`=invalid, `1`=GPS, `2`=DGPS, `4`=RTK fixed, `5`=RTK float |
| 20 | Satellites used | |
| 21-22 | HDOP | integer + fractional/100 |
| 23-25 | Altitude | sign in high bit of byte 23; `(h*256 + reg24 + reg25/100) * sign` |
| 26-28 | Geoid separation | Same sign/scale shape as altitude (23-25) |
| 29-30 | Differential age (s) | integer + fractional/100 |
| 31-32 | Station ID | `reg31*256 + reg32`; read register 32 **before** the 0-31 block (see below) |
| 50 | I2C device ID | readback for `detect()`; should equal the bus address |
| 80 | Data-flush flag | vendor "new data available" flag — **not used for gating** here (see below) |
| 83, 84 | RMC sentence length / data | see "Sentence read protocol" |
| 87, 88 | VTG sentence length / data | course/speed fallback when RMC's course field is empty |
| 93 | Operation mode | `10` = LoRa. Written only when it does not already read `10`; read back to confirm |

## Read order: register 32 before 0-31

Station ID splits across two registers: the high byte (31) sits inside
the single 32-byte block read (registers 0-31), and the low byte (32)
needs a separate transaction (`read_i2c_block_data` on this bus caps a
block read at 32 bytes). Reading 32 **first**, then the 0-31 block,
keeps the pair from tearing across a device-side register update that
lands between the two reads.

## Sentence read protocol (RMC, VTG)

The RMC and VTG NMEA sentences are read through a length register and a
chunked data register (83/84 for RMC, 87/88 for VTG), mirroring the
vendor driver's `get_gnss_message()` I2C-mode branch exactly:

1. Read the length register (1 byte) — the sentence's total length.
2. For each 32-byte chunk: write the running byte offset back to the
   *length* register (it is overloaded as an offset-write register in
   this direction), then read that many bytes from the *data* register.
3. Concatenate the chunks and decode each byte as a character.

`pi_app.hardware.rtk_gps.RtkGpsReader._read_gnss_sentence()` implements
this loop; `parse_rmc_sentence()` / `parse_vtg_sentence()` are pure
functions (checksum-validated) that turn the resulting string into
`(cog_deg, sog_mps, mode)` / `(cog_deg, sog_mps)`.

## Publishing: every fix quality, deduped on UTC second

Two behavior changes from before this commit:

- **Every fix quality is published.** The reader no longer drops
  low-quality epochs itself; `WaypointNavController.accepts_fix_quality()`
  and `GpsHeadingAligner`'s `fix_quality != min_fix_quality` check already
  gate on quality where it matters for control, so a blanket reader-side
  drop only hid low-quality fixes from anything else that might want to
  see them (e.g. troubleshooting, this logging work).
- **Polling (5 Hz) and publishing (once per GPS epoch) are separate.**
  The receiver updates at roughly 1 Hz; polling at the old 1 Hz rate and
  gating a publish on the data-flush register (80) aliased against the
  receiver's own update cadence -- ticks could double-publish the same
  epoch or miss one. The reader now polls at `GpsConfig.poll_hz` (default
  5 Hz) and publishes a new `GpsReading` only when the UTC second
  (registers 0-6) changes from the last publish. The data-flush register
  is not read at all for this gate.

## Course and speed over ground (logging only)

`GpsReading.cog_deg` / `.sog_mps` / `.nmea_mode` come from the RMC
sentence, read every `GpsConfig.cog_read_every` epochs (default: every
epoch). When RMC's course field is empty (a common "stationary" signal),
the reader falls back to VTG for course and speed. A parse or checksum
failure never breaks the position read — it increments `rmc_errors`
(visible in `get_health()`) and leaves `cog_deg`/`sog_mps`/`nmea_mode` at
`None` for that epoch.

**As of this commit, `cog_deg` is not fed into the heading aligner or any
control path — it is logged only**, pending a separate hardware register
check.

## Health

`RtkGpsReader.get_health()` returns `reconnect_count`,
`poll_error_count`, `consecutive_i2c_errors`, `last_poll_age_s`,
`rmc_errors`, and `mode_register_value` (the raw value last read from
register 93). Ten or more consecutive I2C errors close and reopen the bus
with a short backoff, matching the pattern the OAK-D supervisor already
uses (`pi_app/hardware/oak_depth.py`).

## Logging

See `docs/logging_troubleshooting.md`'s "Log format (2026-09-19)" section
for where these fields land in the structured log: the per-tick `gps`
block gets `utc`/`cog_deg`/`sog_mps`/`nmea_mode`/`geoid_sep_m`/`age_s`;
the 1 Hz slow line gets the whole `get_health()` dict as `gps_health`; and
every fix-quality transition is logged separately at WARNING with UTC,
satellite count, HDOP, differential age, and station ID.
