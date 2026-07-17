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
| manual forward-straight intent | required | Both RC tracks must command forward; equal reverse never qualifies |
| `max_lock_yaw_rate_dps` | `3.0` | Curved/turning motion clears lock history |

Fix quality in this codebase (`rtk_gps.py`): `0=none`, `1=GPS`, `2=DGPS`,
`4=RTK fixed`, `5=RTK float`.

After lock, the offset is **frozen** until the armed session ends. Continuous
EMA refinement is disabled: the 2026-07-12 field trace proved that GPS COG
during a turn is not the robot body heading and drove the offset through nearly
200°, producing a self-sustaining circle.

Before lock, RTK dropout (quality ≠ 4) clears history on a new sample.
Once locked, `update()` is a pure no-op until `reset()`, preserving the frozen
offset and history. Duplicate GPS timestamps are ignored. Out-of-order
timestamps clear unlocked history (fail-closed).

## Armed session lifecycle

`GpsHeadingAligner.reset()` runs on every transition that **ends an armed
session**:

- Manual RC disarm (ch3 low)
- RC link stale (>1 s)
- Emergency stop (ch5 latched)

A new arm starts fresh — drive forward with equal manual RC commands on RTK fixed to
re-lock before waypoint missions.

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
| `heading_offset_frozen` | SSE, JSON log, MCAP | `True` after the one-shot lock |
| `heading_offset_refining` | SSE, JSON log, MCAP | Always `False` in this tranche |
| `corrected_heading_deg` | SSE, JSON log, MCAP | True-north heading from raw IMU |
| `heading_align.last_cog_deg` | SSE, JSON log | Last trusted GPS track bearing |
| `heading_align.last_speed_mps` | Controller telemetry dict | Last COG speed used |
| CLI `HDG_OFF(+12.3° F)` | journalctl heartbeat | Offset and frozen state |

Dashboard heading card shows `raw° → corrected°` when locked.

## Field gate checklist (upcoming validation)

1. Arm on level ground with RTK fixed (fix quality **4**).
2. Drive forward straight ≥1 m with equal manual RC commands — confirm
   `heading_offset_locked=true`, `heading_offset_frozen=true`, and CLI
   `HDG_OFF(… F)`.
3. Command a short turn; confirm the offset does not change.
4. Start waypoint nav — it must succeed only after lock.
5. Disarm — offset clears (`HDG_OFF(… -)`); re-arm requires a new drive to lock.
6. Run a short waypoint mission — compare cross-track error vs pre-alignment baseline.

Disable the gate temporarily with `gps_heading_align.enabled = False` in
`config.py` (service restart required).
