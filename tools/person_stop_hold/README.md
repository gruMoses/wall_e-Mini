# Person stop hold: replay tools

These scripts check the person stop hold (`pi_app/control/person_stop_hold.py`) against armed logs. They were written during the work of 2026-11-10 and are kept as they were used. They do not run on the robot's control path.

## replay.py

Replays the hold on one armed log. Each log line (approximately 10 Hz) is one depth poll.

```bash
python3 tools/person_stop_hold/replay.py <repo> <armed log> [--trace HH:MM:SS HH:MM:SS]
```

The output gives, for each hold: the start time, the length, the modes, what engaged it, what released it, the reason and the time if it became operator-only, the held lines that the old stop did not have, and the forward ticks that it blocked. FOLLOW_ME and MANUAL are reported separately. Use it after a drive to see how frequently the hold becomes operator-only, and before you change a threshold.

Limits: the log has 10 lines each second and the robot polls at 15 each second. The log has only the detections above the follow-me confidence.

## plausibility_stats.py

Measures how fast the range and the box area of one object change between two sightings in the logs. The plausibility limits of the hold (1.5 m/s, 0.25 m, area factor 1.5) come from these numbers.

```bash
python3 tools/person_stop_hold/plausibility_stats.py <armed log> [<armed log> ...]
```

## Technical Names

FOLLOW_ME, MANUAL, operator-only, armed log, depth poll.
