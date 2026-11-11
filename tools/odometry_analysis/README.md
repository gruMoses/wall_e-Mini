# Odometry analysis tools

These scripts were written for the odometry study and the odometry repair of 2026-11-10 (`docs/status_audit_2026-11-10.md`, sections 8 and 9.5). They are kept as they were used. They read armed logs; they do not run on the robot's control path.

## Extract and measure

| Script | Use |
|---|---|
| `extract.py OUTDIR log1 [log2 ...]` | Run it on the robot or on log copies. It writes a small `.jsonl.gz` extract for each armed log (heading, motor bytes, wheel speed, GPS, follow-me odometry fields). |
| `analyze.py FILE...` | Inventory of the extracts: modes, distance, fix quality, odometry source. |
| `wheel_scale.py FILE...` | Wheel speed against GPS ground speed on straight, steady windows. |
| `heading.py FILE...` | Gyro heading against GPS course while the robot moves forward. |

## Compare the odometry sources

| Script | Use |
|---|---|
| `compare.py EXTDIR name1 [name2 ...]` | Dead reckoning and GPS odometry against the GPS track on 30 s windows. |
| `compare2.py EXTDIR name1 [name2 ...]` | The same windows without a constant frame rotation (the shape of the path only). It imports `compare.py`. |

## Replay with trail following

`trail_follow_enabled` is False in `config.py`. These scripts set it to True in the replay only.

| Script | Use |
|---|---|
| `trail_on.py REPO_ROOT SOURCE SCENARIO LOG [LOG ...]` | Replays real follow-me drives with the trail on. `SOURCE` is `dead_reckon` or `gps`. |
| `truth.py REPO_ROOT SOURCE SCENARIO LOG PHI_DEG` | Checks if trail pursuit steers to the true side of the point that it steers to. The replay is open loop. |
| `flag_off_equivalence.py REPO_ROOT OUT.json LOG [LOG ...]` | Replays with the default config and writes the motor bytes. Run it on two checkouts and compare the two files: with the switch off they must be the same. |

Run each script with no arguments or read its first lines for the full usage.

## Before the trail switch goes on

1. A stall check for dead reckoning (it integrates the commanded bytes, so a blocked robot still "moves").
2. A straight run of approximately 20 m on a tape (scale and gyro drift).
3. A run in an empty area: pursuit beyond 5.2 m and a lost target, steering each way.
4. A Grok review of the diff, then Kevin's approval.

## Technical Names

dead reckoning, GPS odometry, trail pursuit, open loop, armed log, extract.
