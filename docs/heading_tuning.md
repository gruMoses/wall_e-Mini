# Heading Tuning (OAK-D IMU)

Field-proven procedure for diagnosing and calibrating OAK-D Lite BMI270 yaw after
chalk tests. Production defaults were **field-validated 2026-07-12**: pin
`gyro_y` at scale `1.0`. **Never restore `auto` / `0.46`** — that pair hid
host-side sample loss behind an empirical multiplier.

## Architecture (lossless producer yaw)

```
BMI270 @ 100 Hz ──► DepthAI host IMU queue (maxSize=512 msgs, nonblocking)
                         │
                         ▼
                    OakDepthReader._poll_imu (shared vision thread)
                         │
                         ├─ drain ALL queue messages (consumed/batched, not dropped)
                         ├─ sort packets by device timestamp
                         └─ ImuYawProducer.ingest(every packet; cap=512 packets)
                                │
                                ├─ cum_yaw_x / y / z / gravity (unscaled rad)
                                ├─ latest raw body rates (diagnostics)
                                └─ counters: integrated / duplicate / gap / backlog
                                         │
                                         ▼
                              OakImuReader.read()
                                │
                                ├─ select axis (gyro_x/y/z / auto / gravity)
                                ├─ Δcum × scale  (scale applied exactly once)
                                ├─ preserve unread Δcum across generation bumps
                                │    (incl. +turn then -turn returning cum≈0)
                                └─ reseed without jump only on counter rewind
```

### Why this exists

1. **Host-side packet loss (primary under-report)** — OAK requests 100 Hz, but
   the shared vision loop historically drained the IMU queue and kept only the
   newest packet (`latest` mode). `OakImuReader` then reconstructed angle from
   sparse snapshots; intervals >0.15 s **gap-froze** and lost rotation. Direct
   light-pipeline harness at fixed `gyro_y` scale=1 was accurate (~90→92,
   ~180→180–183) while full service scale=1 read ~73/81 for physical 90°.
2. **Host queue depth** — DepthAI default `createOutputQueue()` is
   `maxSize=16` nonblocking. Full-vision stalls can overwrite samples before
   the host ever sees them. Production uses **maxSize=512 message slots**,
   nonblocking. At batch threshold ≈1 this is multi-second headroom, but it is
   **not** a hard 5.1 s wall-clock guarantee: one queue message may hold
   multiple packets. **Nonblocking overwrite loss is not directly observable**
   through `tryGet` — never invent occupancy or drop counts from drain size.
3. **Wrong / unstable axis (`auto`)** — can pick X/Z under vibration.
4. **Phantom yaw from duplicate samples** — fixed earlier (6950e14).
5. **Generation-reseed loss** — a single jittered/regressed device timestamp
   bumps producer generation; older consumers discarded all unread cum (~27°
   in simulation). Consumer now applies continuous cum deltas across generation
   bumps. True freeze only on **integrated-packet counter rewind** (producer
   replacement) — never a near-zero cum magnitude heuristic (a legitimate
   +turn then -turn returns channels to ~0).

## Current production baseline (config.py)

| Key | Value | Notes |
| --- | --- | --- |
| `oak_yaw_rate_source` | `"gyro_y"` | Field-validated 2026-07-12; **never restore `auto`** |
| `oak_yaw_rate_scale` | `1.0` | Neutral; **never restore `0.46`** as a loss mask |
| `oak_use_gravity_projected_yaw_rate` | `False` | |
| `oak_nmni_enabled` | `True` | Applied per-packet on the producer |
| `oak_nmni_threshold_dps` | `0.3` | |
| `oak_bias_adapt_enabled` | `False` | |
| IMU host queue | `maxSize=512` msgs, `blocking=False` | message slots, not seconds |
| Producer drain cap | `max_packets_per_drain=512` | aligned with host msg capacity @ ~1 pkt/msg |

Legacy `auto`+`0.46` is retired. If chalk later shows systematic magnitude error
*after* lossless proof, fit a new scale for the **pinned** axis only — do not
bring back auto axis switching without new field evidence.

## Observability

`OakImuReader.get_health()` and extra `read()` keys expose:

- Body triad: `gx_body_dps`, `gy_body_dps`, `gz_body_dps`
- Selected path: `yaw_rate_source_cfg`, `yaw_rate_source_selected`, `yaw_rate_scale`, `yaw_rate_sign`
- Integration path: `integration_path` (`producer` vs `legacy_snapshot`)
- Sample identity: `sample_age_s`, `device_timestamp_s`, `last_dt_s`
- Consumer counters: `count_duplicate`, `count_stale`, `count_regressed`,
  `count_restart`, `count_gap_freeze`, `count_integrated`,
  `count_producer_packets`, `count_generation_change`, `count_cum_reset`
- Producer / host counters (**truthful loss accounting**):
  - `producer_packets_received` / `drained` / `parsed` / `integrated`
  - `producer_packets_duplicate` / `gap_freeze` / `regressed` / `restart` / `backlog_dropped`
  - `packets_coalesced` — legacy selection drops; stays 0 **by structure** on
    the lossless path (never force-assigned as a proof)
  - `producer_cadence_avg_s` / `producer_cadence_max_s` (healthy ~0.01 s avg)
  - `producer_cum_yaw_x_deg` / `y` / `z` / `grav` (unscaled free yaw)
  - `last_batch_packets`, `queue_drain_count`
  - `queue_msgs_received` / `queue_msgs_consumed` — successfully drained
    messages are **consumed/batched**, never counted as dropped
  - `queue_msgs_dropped` — always 0 (overwrite loss not observable)
  - `queue_msgs_overwrite_observable` — always `false`
  - Host config: `host_queue_max_size`, `host_queue_blocking`, `max_packets_per_drain`
  - **Drain-batch observability** (messages drained per poll — **not** occupancy):
    `drain_batch_high_water_msgs`, `drain_batch_large_events`,
    `drain_batch_full_size_events`
- OAK pipeline: `oak_connected`, `oak_reconnect_count`, `oak_pipeline_running`

Live service: `controller.get_imu_status()["oak_imu"]` (logged under `imu` in arm logs).

### Metric invariants (loss proof)

During a clean chalk turn, **do not** treat a hard-coded zero as proof. Require:

| Invariant | Healthy |
| --- | --- |
| `integrated ≈ received − restart_seeds − duplicate − gap_freeze − …` | within a few packets |
| `packets_backlog_dropped` | 0 (full configured backlog must integrate) |
| `packets_gap_freeze` | rare (cadence_max mostly ≪ 0.15 s) |
| `queue_msgs_dropped` | 0 (drained ≠ dropped; overwrite not observable) |
| `drain_batch_high_water_msgs` | informative only — large drain ≠ queue full |
| scale=1 best axis `\|Δ\|` | within pass band of chalk angle |
| `count_cum_reset` | 0 unless integrated-counter rewind (true replacement) |

`packets_coalesced` remaining 0 only means selection mode is not dropping.
**Never** infer host-queue overflow from drain-batch high-water / large / full-size
events — those measure messages successfully retrieved in one poll, not remaining
queue depth or overwritten samples.

### Residual limitations

- **DepthAI nonblocking overwrite loss is not directly observable** through this
  API. If the device overwrites unread host-queue messages during a long stall,
  those samples never appear in `tryGet` and cannot be counted as drops.
- Host queue is a **message-slot** budget. At batch≈1 / 100 Hz it is roughly
  multi-second, but multi-packet messages make wall-clock headroom shorter than
  `maxSize/rate`. Do not claim a hard 5.1 s guarantee.
- Producer cap is in **packets** (aligned to 512 at ~1 pkt/msg). Pathological
  multi-packet bursts above the cap soft-drop oldest samples
  (`packets_backlog_dropped`) to keep CPU/memory bounded.

## Safe chalk harness (disarmed)

Stop the service so it does not own the camera, then:

```bash
# on the Pi
sudo systemctl stop wall-e    # unit name may vary
cd /home/pi/wall_e-Mini
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --stream
# then
python3 -m pi_app.cli.oak_yaw_chalk_test --expected 180 --stream
```

The harness uses the **full shared OakDepthReader pipeline** (same producer path as
service) and prints producer + drain-batch metrics.

### Exact procedure

1. Flat ground. Mark chalk **0°** on floor and a matching mark on the chassis.
2. Mark chalk **90°** and **180°** CW (looking down) with a square/protractor or known board.
3. Robot **disarmed**, no motor drive from this tool (harness never commands motors).
4. Start harness; wait for bias collection and `READY`.
5. Align chassis to 0°. Enter → **MARK START**.
6. Rotate slowly by hand (or carefully with RC only if you accept extra vibration) to the
   chalk target. Prefer hand-rotate for axis ID.
7. Enter → **MARK END**.
8. Read the report:
   - triad from **producer cum** at scale=1 (`gyro_x/y/z`)
   - production path (`gyro_y` × 1.0)
   - producer `recv` / `integrated` / `gap` / `backlog_drop` / cadence
   - drain-batch high-water (msgs per poll) vs maxSize — not occupancy

### Choosing axis and scale (from evidence only)

1. Among `gyro_x`, `gyro_y`, `gyro_z` at scale=1, pick the axis whose **|Δ|** is closest to
   the chalk angle and whose **sign is consistent** CW vs CCW.
2. Production is already pinned to `gyro_y` @ 1.0 from 2026-07-12 field validation.
3. Only if magnitude is systematically off **after** lossless proof:
   `new_scale = expected_deg / |measured_deg|` for that pinned axis.
4. Re-run 90° and 180° both CW and CCW with the candidate settings via:
   ```bash
   python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 \
     --production-source gyro_y --production-scale 1.0
   ```
5. Only then edit `config.py` and redeploy. **Do not reintroduce auto/0.46.**

### Pass criteria

For a single chalk turn after bias, with the **chosen pinned axis** at its fitted scale
(or scale=1 if magnitude already matches):

| Check | Pass |
| --- | --- |
| Magnitude | `||measured| − expected| ≤ max(8°, 10% of expected)` |
| 90° and 180° | Both pass; 180° error should not be ~2× worse than 90° (rules out wrong axis) |
| CW vs CCW | |Δ| within the same band; opposite free-yaw sign |
| No packet loss | integrated tracks received; backlog_drop=0 |
| No jumps | Heading freezes on stale/duplicate; counter-rewind freeze only on true replacement |
| Integration | During the turn, `count_integrated` / `count_producer_packets` increase |

Fail examples from the pre-producer chalk run: 90° CW → ~72° / ~69° (wrong axis and/or
sparse snapshot gap-freeze). Service scale=1 at ~73/81 for physical 90° with load while
light pipeline was accurate — classic host-side loss.

## Exact hardware validation (post-fix)

On the robot, with service path (or chalk harness using full OakDepthReader):

1. `sudo systemctl stop wall-e`
2. `python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 --production-source gyro_y --production-scale 1.0 --stream`
3. Hand-rotate 90° CW; confirm:
   - triad `gyro_y` PASS at scale=1
   - production free-yaw ≈ triad for gyro_y
   - backlog_drop=0, cadence_avg≈0.01 s; drain-batch stats are informational only
4. Repeat 180° CW and 90° CCW.
5. Restart service under normal vision load; arm; log `imu.oak_imu` during a slow pivot:
   - `integration_path=producer`
   - `yaw_rate_source_selected=gyro_y`
   - heading tracks physical turn without a scale fudge
6. Only if magnitude still systematically off after lossless proof, fit scale from chalk
   and keep the axis pinned — never “guess” a multiplier to hide packet loss.

## If regression happens in service

1. Confirm IMU source: startup log `source: oak_d`.
2. Inspect arm log `imu.oak_imu` (or status JSON):
   - `integration_path` is `producer`?
   - `yaw_rate_source_selected` is `gyro_y` (not auto flapping)?
   - `packets_backlog_dropped` / `gap_freeze` rising during motion
   - large `drain_batch_*` alone is **not** overflow proof (overwrite not observable)
   - `count_cum_reset` during motion without USB reconnect → investigate counter rewind
   - `count_duplicate` rising while idle is expected; rising `count_stale` during motion is not
   - `sample_age_s` should stay well under 0.5 s when healthy
3. Re-run chalk harness before changing scale again.

## Unit tests

```bash
python3 -m unittest pi_app.tests.test_oak_imu_yaw_producer pi_app.tests.test_oak_imu -v
python3 -m unittest discover -s pi_app/tests -p "test_*.py"
```

Coverage includes: sparse undercount reproduction, full-batch accuracy under delayed
consumer reads, full 512-packet backlog without drop, duplicates, ordering,
generation-jitter 27° preservation, exact +10°/−10° with generation (no false
cum_reset), true counter-rewind reset freeze, reconnect, bounded soft backlog cap,
scale-once, host-queue / producer-cap alignment asserts, axis isolation, and
controller health embedding.

## Related docs

- `pi_app/hardware/oak_imu_yaw_producer.py` — pure integrator
- `docs/imu_tuning_session_20260308.md` — mount / gravity-projection history
- `docs/IMU_TROUBLESHOOTING.md` — external IMU paths
- `docs/gps_heading_alignment.md` — absolute north alignment (separate from relative gyro yaw)
