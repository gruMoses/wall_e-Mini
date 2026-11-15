Moved out of CLAUDE.md on 2026-11-14 (text kept as it was).

### Camera calibration — READ THIS BEFORE TOUCHING ANY CAMERA GEOMETRY

Geometry comes from the device's per-unit factory EEPROM, **not** from a
hand-entered field of view. `OakDepthReader.get_intrinsics(w, h)` is the single
source; the obstacle corridor, person position and the recorder overlay all go
through it. Never re-derive a focal length from an FOV constant — that is
exactly how the overlay ended up drawing different geometry from the corridor.

Measured on this unit (`python3 -m pi_app.cli.oak_intrinsics`, read-only; stop
`wall-e.service` first because the OAK allows one process at a time):

| | |
|---|---|
| CAM_A @ 640×400 | fx = 456.89, cx = 334.95 |
| Horizontal FOV | **70.01°** (EEPROM spec 68.794°) |
| Principal point | **+14.9 px off centre** — never assume cx = w/2 |
| VFOV at the 640×352 detection frame | **42.13°** |

Startup logs the numbers in use at WARNING level (the app installs no logging
handler, so INFO is dropped). A silent fallback to the config constant changes
safety geometry, so it is logged loudly.

**Three defects this replaced (2026-07-26), all from one wrong constant:**
`camera_hfov_deg` was 81.0 — the *diagonal* FOV in a horizontal field.
1. Corridor threshold 18% too small: guarded a ~0.67 m robot instead of 0.82 m.
2. `detect_camera_vfov_deg` 65.3 → 42.13. Implied heights had been inflated
   1.66×, so the 1.20 m "reject short ground blobs" gate was really 0.72 m.
3. DetectionFilter Rule 3 now **skips boxes clipped by the top frame edge** —
   height is unmeasurable on a truncated box. Bottom contact is deliberately
   NOT excused, because ground animals rest against the frame bottom.

WARNING: (1) and (2) were cancelling each other. Fixing either alone makes the
robot worse — correcting the FOV without (3) filters the operator out at
`follow_distance_m`. Expect this pattern elsewhere; see
`~/Documents/screenshots/WALLE-constant-audit.md`.

Checked 2026-11-14 (all armed logs 2026-11-06 to 11-11 on the Pi, Rule 3
recomputed from the logged raw boxes): `detect_min_person_height_m = 1.20` is
right; keep it. Upright adults measure 1.55-1.75 m (p25-p50 at 3.5-5 m). In
FOLLOW_ME, 94 % of person boxes are top-clipped, so the rule skips them. On
the 26 FOLLOW_ME ticks with a height reject, every rejected box was a
non-target track of about 1 m (goats and dogs labelled person, with sheep or
dog detections beside them), and no tick lost all its detections. The speed
cap (`_nearest_person_range_m`) and the 0.8 m stop ignore the rule. Residual:
a crouched or half-hidden operator beyond about 3 m reads about 1 m and is
dropped, and the robot then stops. `tools/replay_follow_me_log.py` cannot
test the rule (it uses a dummy box); recompute from the `detections` block.

