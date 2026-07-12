# GPS Course-Over-Ground Heading Alignment

The OAK-D Lite BMI270 has no magnetometer. Integrated-gyro heading is relative to
startup orientation and drifts over time. **GPS heading alignment** learns a
scalar offset from RTK course-over-ground (COG) while the robot is actually
moving, so:

```text
corrected_heading_deg = (raw_imu_heading_deg + offset_deg) mod 360
```

is referenced to true north. The aligner never commands motors — it only
maintains this offset for consumers (waypoint nav, straight-drive heading hold).

## When the offset locks

All gates must pass on **distinct GPS samples** (deduped by `GpsReading.timestamp`):

| Gate | Default (`config.gps_heading_align`) | Meaning |
|------|--------------------------------------|---------|
| `enabled` | `True` | Feature on |
| `min_fix_quality` | `4` | **RTK fixed only** (not float/GPS/DGPS) |
| `min_distance_m` | `0.8` | Displacement in rolling window |
| `min_speed_mps` | `0.12` | COG speed from GPS sample timestamps |
| `history_seconds` | `8.0` | Rolling GPS window |

Fix quality in this codebase (`rtk_gps.py`): `0=none`, `1=GPS`, `2=DGPS`,
`4=RTK fixed`, `5=RTK float`.

After lock, `alpha` (default `0.1`) EMA-refines the offset on subsequent RTK-fixed
samples. RTK dropout (quality ≠ 4) clears history but **preserves** the last
learned offset until the armed session ends. Out-of-order or duplicate GPS
timestamps are ignored.

## Armed session lifecycle

`GpsHeadingAligner.reset()` runs on every transition that **ends an armed
session**:

- Manual RC disarm (ch3 low)
- RC link stale (>1 s)
- Emergency stop (ch5 latched)

A new arm starts fresh — drive forward on RTK fixed to re-lock before waypoint
missions.

## Waypoint navigation gate (field test)

When heading alignment is **enabled** and both GPS + IMU are available,
`/api/nav/start` and `/api/nav/go` return **409** with
`heading_alignment_not_locked` until the aligner has locked.

Independently of the heading-align gate, waypoint activation and every runtime
navigation update require exactly fix quality **4** when
`waypoint_nav.min_rtk_quality = 4`. RTK float (quality **5**) returns
`gps_quality_not_trusted` at activation and commands zero motion if quality
drops during a mission. The existing stale-GPS timeout remains in force.

When alignment is disabled (`gps_heading_align.enabled = False`) or the GPS/IMU
stack is absent, waypoint nav behaves as before.

The `/navigate` pre-run validation panel shows **GPS heading alignment locked**
when the gate applies.

## Operator observability

| Field | Where | Meaning |
|-------|-------|---------|
| `heading_offset_deg` | SSE, JSON log, MCAP | Learned IMU→true-north offset |
| `heading_offset_locked` | SSE, JSON log, MCAP | `True` once COG lock succeeded |
| `corrected_heading_deg` | SSE, JSON log, MCAP | True-north heading from raw IMU |
| `heading_align.last_cog_deg` | SSE, JSON log | Last trusted GPS track bearing |
| `heading_align.last_speed_mps` | Controller telemetry dict | Last COG speed used |
| CLI `HDG_OFF(+12.3° L)` | journalctl heartbeat | Offset and lock (`L` = locked) |

Dashboard heading card shows `raw° → corrected°` when locked.

## Field gate checklist (upcoming validation)

1. Arm on level ground with RTK fixed (fix quality **4**).
2. Drive straight ≥1 m — confirm `heading_offset_locked=true` in SSE or CLI `HDG_OFF(… L)`.
3. Start waypoint nav — must succeed only after lock.
4. Disarm — offset clears (`HDG_OFF(… -)`); re-arm requires a new drive to lock.
5. Run a short waypoint mission — compare cross-track error vs pre-alignment baseline.

Disable the gate temporarily with `gps_heading_align.enabled = False` in
`config.py` (service restart required).
