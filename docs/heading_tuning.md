# Heading Tuning (OAK-D IMU)

Field-proven procedure for diagnosing and calibrating OAK-D Lite BMI270 yaw after
chalk tests. **Do not change production `oak_yaw_rate_source` / `oak_yaw_rate_scale`
until measured evidence from the chalk harness picks them.**

## Current production baseline (config.py)

In `ImuSteeringConfig` (treat as *last known*, not necessarily correct after mount changes):

| Key | Value | Notes |
| --- | --- | --- |
| `oak_yaw_rate_source` | `"auto"` | Can lock the wrong axis under vibration — prefer pinned `gyro_x/y/z` after chalk |
| `oak_yaw_rate_scale` | `0.46` | Empirical; only valid for the axis auto actually used when fitted |
| `oak_use_gravity_projected_yaw_rate` | `False` | |
| `oak_nmni_enabled` | `True` | |
| `oak_nmni_threshold_dps` | `0.3` | |
| `oak_bias_adapt_enabled` | `False` | |

## Root causes seen in chalk field test (2026)

1. **Wrong / unstable axis (`auto`)** — rough 90° CW chalk turns reported ~72° / ~69° and
   precise segments were inconsistent. `auto` re-selects the dominant body axis while
   turning; if X or Z spikes, integration uses the wrong component and scale 0.46 no longer
   matches.
2. **Phantom yaw from duplicate/stale samples** — `OakImuReader.read()` used to fall back
   to host controller `dt` when the device timestamp did not advance, so multiple
   controller polls of the same cached packet re-integrated the same gyro rate (and
   `get_heading_deg()` / `update()` double-reads made it worse).

Hardening (software): each distinct fresh sample is integrated **at most once**. Stale,
duplicate, regressed (reconnect), invalid, or oversized-gap timestamps **freeze** heading
(no jump) and report zero yaw rate for that tick.

## Observability

`OakImuReader.get_health()` and extra `read()` keys expose:

- Body triad: `gx_body_dps`, `gy_body_dps`, `gz_body_dps`
- Selected path: `yaw_rate_source_cfg`, `yaw_rate_source_selected`, `yaw_rate_scale`, `yaw_rate_sign`
- Sample identity: `sample_age_s`, `device_timestamp_s`, `last_integrated_device_ts_s`, `last_dt_s`
- Counters: `count_duplicate`, `count_stale`, `count_regressed`, `count_restart`, `count_gap_freeze`, `count_integrated`
- OAK pipeline (when available): `oak_connected`, `oak_reconnect_count`, `oak_pipeline_running`

Live service: `controller.get_imu_status()["oak_imu"]` (logged under `imu` in arm logs when status is present).

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

### Exact procedure

1. Flat ground. Mark chalk **0°** on floor and a matching mark on the chassis.
2. Mark chalk **90°** and **180°** CW (looking down) with a square/protractor or known board.
3. Robot **disarmed**, no motor drive from this tool (harness never commands motors).
4. Start harness; wait for bias collection and `READY`.
5. Align chassis to 0°. Enter → **MARK START**.
6. Rotate slowly by hand (or carefully with RC only if you accept extra vibration) to the
   chalk target. Prefer hand-rotate for axis ID.
7. Enter → **MARK END**.
8. Read the report: `gyro_x` / `gyro_y` / `gyro_z` at **scale=1**, plus the production path
   (config source × scale).

### Choosing axis and scale (from evidence only)

1. Among `gyro_x`, `gyro_y`, `gyro_z` at scale=1, pick the axis whose **|Δ|** is closest to
   the chalk angle and whose **sign is consistent** CW vs CCW.
2. Recommended magnitude scale:
   `new_scale = expected_deg / |measured_deg|` (for that pinned axis at scale=1).
3. Pin production: `oak_yaw_rate_source = "gyro_y"` (example) and set `oak_yaw_rate_scale`
   to the fitted value. **Do not leave `auto` if chalk shows axis flapping.**
4. Re-run 90° and 180° both CW and CCW with the candidate settings via:
   ```bash
   python3 -m pi_app.cli.oak_yaw_chalk_test --expected 90 \
     --production-source gyro_y --production-scale 1.0
   ```
5. Only then edit `config.py` and redeploy.

### Pass criteria

For a single chalk turn after bias, with the **chosen pinned axis** at its fitted scale
(or scale=1 if magnitude already matches):

| Check | Pass |
| --- | --- |
| Magnitude | `||measured| − expected| ≤ max(8°, 10% of expected)` |
| 90° and 180° | Both pass; 180° error should not be ~2× worse than 90° (rules out wrong axis) |
| CW vs CCW | |Δ| within the same band; opposite free-yaw sign |
| No jumps | Heading freezes on stale/duplicate; reconnect increments `count_regressed` / `count_restart` without a step change |
| Integration | During the turn, `count_integrated` increases; idle double-polls only increase `count_duplicate` |

Fail examples from the pre-hardening chalk run: 90° CW → ~72° / ~69° with inconsistent
segments (wrong axis and/or phantom integration).

## If regression happens in service

1. Confirm IMU source: startup log `source: oak_d`.
2. Inspect arm log `imu.oak_imu` (or status JSON):
   - `yaw_rate_source_selected` stable during a pure yaw turn?
   - `count_duplicate` rising while idle is expected; rising `count_stale` / `count_regressed` during motion is not.
   - `sample_age_s` should stay well under 0.5 s when healthy.
3. Re-run chalk harness before changing scale again:
   `new_scale = old_scale * (expected / |measured|)` only after the axis is pinned.

## Related docs

- `docs/imu_tuning_session_20260308.md` — mount / gravity-projection history
- `docs/IMU_TROUBLESHOOTING.md` — external IMU paths
- `docs/gps_heading_alignment.md` — absolute north alignment (separate from relative gyro yaw)
