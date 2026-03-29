# Mower Mode Plan

**Status:** Planning
**Date:** 2026-03-29

Autonomous electric lawn mowing mode for WALL-E Mini. This document covers what we reuse, what we build, and how we sequence the work.

---

## Overview

WALL-E Mini already has the core autonomy stack needed for mowing: RTK GPS, depth camera with neural inference, IMU, VESC motor controllers, pure pursuit path following, and obstacle avoidance. The gap is mission-level software — boundary management, coverage path planning, a blade motor, and the sensor fusion upgrades to make GPS-denied areas survivable.

The strategy is RTK GPS + boustrophedon (back-and-forth) path planning as the primary approach, with OAK-D depth + neural net as the primary safety sensor. This is the same stack the best open-source and commercial mowers use.

---

## Reference Projects

| Project | Notes |
|---|---|
| [OpenMower](https://github.com/ClemensElflein/OpenMower) | Most popular open-source autonomous mower. ublox F9P RTK, custom mainboard, ROS. Good reference for overall architecture. |
| [OAK-Mower](https://alemamm.github.io/oakmower) | OpenMower fork with OAK-D spatial AI obstacle detection. Closest match to our hardware. Follow closely. |
| [Kenny Trussell's ArduRover mower](https://ktrussell.com/) | PixHawk + ZED-F9P, cutting 5–18 acre fields for 2+ years. Most battle-tested DIY build. Good source for practical lessons. |
| [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) | ROS2-compatible coverage path planning library. Boustrophedon decomposition, Dubins curves, route optimization. Use this. |
| [Greenzie planner](https://github.com/greenzieai) | Commercial autonomous mowing company, boustrophedon planner open-sourced (archived). Reference for coverage logic. |

---

## What We Reuse

Everything on this list already works and just needs to be wired into the mower mission context.

| Component | Current State | Mower Use |
|---|---|---|
| RTK GPS (DFRobot GNSS-RTK, I2C) | Active | Boundary recording, localization |
| OAK-D Lite (depth + RGB + VPU) | Active | Obstacle detection, visual odometry |
| IMU (external I2C + OAK-D BMI270) | Active | Sensor fusion, tilt/lift interlock |
| VESC over CAN | Active | Drive motors; read wheel encoder telemetry for odometry |
| Pure pursuit path follower (`trail_follow`) | Active | Execute mowing lines, no rewrite needed |
| Depth-based obstacle avoidance | Active | Integrate with mower safety tiers |
| Person detection (MobileNet on OAK-D) | Active | Will be replaced by YOLOv8n multi-class |
| Web dashboard | Active | Add boundary editor, coverage map, mower status |
| Safety state machine (arm/disarm, e-stop) | Active | Extend with blade interlocks |
| `config.py` dataclass pattern | Active | Mower-specific tuning params live here |

---

## What's New

| Component | Description |
|---|---|
| Boundary recorder | RC-drive the perimeter while logging RTK waypoints → save as GeoJSON polygon |
| Boustrophedon path planner | Fields2Cover integration: decompose boundary into convex sub-regions → parallel mowing lines with 5–10% overlap |
| Coverage tracker | Track which cells/strips have been mowed; support multi-session resume |
| Multi-zone support | Multiple named boundaries (front yard, back yard, etc.) |
| Exclusion zones | GPS-tagged permanent obstacles (trees, sprinkler heads) to skip in path planning |
| Dual-EKF sensor fusion | RTK (5 Hz) + IMU (50 Hz) + VESC wheel odometry. Use `robot_localization` or custom EKF |
| YOLOv8n obstacle detector | Replaces MobileNet. Multi-class: people, children, pets, balls, toys, hoses, rocks. Runs on OAK-D Myriad X VPU at 20–25 FPS |
| Tiered safety response | STOP / SLOW+NAVIGATE / MAP+AVOID based on object class (see below) |
| Blade motor control | ESC or relay output from Pi GPIO; blade start/stop tied to state machine |
| Blade safety interlocks | Tilt, lift, obstacle proximity — blade cuts power automatically |
| Return-to-dock | Navigate home on low battery or mission complete |
| Battery-aware planning | Estimate remaining coverage from battery %; split missions if needed |

---

## Navigation Architecture

### Localization

Dual-EKF fusion stack:

```
RTK GPS (5 Hz)  ──┐
IMU (50 Hz)     ──┼──▶  EKF (robot_localization)  ──▶  pose estimate
VESC odometry   ──┘
(OAK-D visual odometry — Phase 2 stretch)
```

Under tree canopy where RTK degrades, IMU + wheel odometry dead-reckons through the gap. OAK-D visual odometry is a Phase 2 addition if dead-reckoning drift is unacceptable in testing.

**RTK base station:** Own base station (~$400) is strongly recommended over NTRIP. No internet dependency, lower latency, better uptime. If we're already paying for RTK corrections via NTRIP, evaluate reliability first.

### Path Planning (Fields2Cover)

1. Load boundary GeoJSON polygon
2. Decompose into convex sub-regions (boustrophedon decomposition handles L-shapes, irregular yards)
3. Generate parallel mowing lines at `config.mower.row_spacing` with 5–10% overlap
4. Optimize route (minimize turns, respect Dubins curve turning radius)
5. Inject exclusion zones as hard obstacles
6. Output as ordered waypoint list → feed to pure pursuit

Fields2Cover is the right tool here. It has a Python API, is ROS2-compatible, and is what OpenMower-derived projects use.

### GPS Dropout Handling

Tree canopy is the universal failure mode. Plan:
- EKF continues on IMU + odometry when GPS fix quality drops below threshold
- Flag GPS-denied zones on the coverage map
- If drift exceeds threshold (TBD in testing), pause and wait for RTK re-acquisition rather than guess
- Log all GPS dropout events with timestamps + locations for analysis

---

## Obstacle Detection Architecture

### Model: YOLOv8n on OAK-D Myriad X VPU

- 20–25 FPS on VPU; Pi CPU stays near 0% during inference
- Export to OpenVINO IR format for OAK-D deployment
- Better than MobileNet-SSD for multi-class detection

**Critical OAK-D config:** Enable `Extended Disparity` mode. Normal mode has a 0.7m minimum depth — anything closer is invisible. Extended Disparity halves this to ~0.35m, which matters for low-profile obstacles on the ground.

### Detection Pipeline

```
RGB frame + stereo depth ──▶  YOLOv8n (VPU)
                                    │
                              confidence filter
                                    │
                           depth validation (stereo)
                           (shadows → no depth → filtered)
                                    │
                           temporal tracking (N frames)
                                    │
                           size validation
                                    │
                           tiered safety response
```

Depth validation is the key false-positive killer: shadows produce no stereo depth signal, so they don't survive the filter. This is the main reason to use stereo over monocular for this task.

### Safety Tiers

| Tier | Objects | Response |
|---|---|---|
| STOP IMMEDIATELY | People, children, pets | Single confident detection → blade off, full stop, alarm |
| SLOW + NAVIGATE | Balls, toys, unknown objects | 3-frame confirmation → slow, attempt to route around |
| MAP + AVOID | Rocks, sprinkler heads, permanent obstacles | Add to exclusion zone map, update path plan |

### Training Data

No good public dataset exists for residential yard obstacles. FieldSAFE (grass mowing with labeled obstacles) is the closest starting point. Plan:
- Collect 200–500 images per custom class from the actual yard
- Augment: lighting variation, wet grass, shadows, partial occlusion
- Label with Roboflow or similar
- Fine-tune YOLOv8n from COCO checkpoint

---

## Blade Motor Control

Hardware TBD (see Open Questions), but the software interface is:

```python
class BladeController:
    def enable(self): ...   # start blade motor
    def disable(self): ...  # stop blade motor
    def is_safe(self) -> bool: ...  # check interlocks
```

Interlocks that force blade off:
- Tilt angle exceeds threshold (IMU)
- Robot lifted (current spike pattern or dedicated lift sensor)
- Obstacle within `config.mower.blade_stop_distance`
- E-stop active
- GPS fix lost for > N seconds

---

## Dashboard Extensions

The web dashboard needs two new views for mower mode:

1. **Boundary Editor** — Draw/record yard boundaries on map. Place exclusion zones. Select active zone for mowing.
2. **Coverage Map** — Real-time view of planned path vs. completed strips. Battery estimate. Current mission status. GPS quality indicator.

Both can be layered onto the existing dashboard map component rather than built from scratch.

---

## Phases

### Phase 0: Boundary & Map

- [ ] Build boundary recording tool: RC-drive perimeter, record RTK waypoints at ~0.5m intervals
- [ ] Save/load boundary as GeoJSON polygon
- [ ] Visualize boundary on dashboard map
- [ ] Add exclusion zone drawing (point + radius, or polygon)
- [ ] Validate RTK accuracy — measure actual vs. measured perimeter on a known shape

**Exit criteria:** Can record a yard boundary, save it, reload it, and view it on the dashboard. Exclusion zones draw correctly.

### Phase 1: Path Planning

- [ ] Integrate Fields2Cover (Python bindings or subprocess call)
- [ ] Generate boustrophedon mowing lines from boundary polygon
- [ ] Respect exclusion zones in path generation
- [ ] Visualize planned path on dashboard
- [ ] Tune `row_spacing` for actual blade width + overlap
- [ ] Validate path continuity through turning radius constraints

**Exit criteria:** Given a boundary polygon, produce a valid ordered waypoint list. Visualize it. Feed to pure pursuit and drive a strip manually (no blade).

### Phase 2: Sensor Fusion Upgrade

- [ ] Read VESC wheel encoder telemetry via CAN (check what's already available in CAN telemetry read-back)
- [ ] Implement or integrate dual-EKF (RTK + IMU + wheel odometry)
- [ ] Validate localization accuracy: drive known path, measure drift
- [ ] Simulate GPS dropout (disable RTK fix): measure dead-reckoning drift over 30s / 60s
- [ ] Enable OAK-D Extended Disparity mode
- [ ] Stretch: add OAK-D visual odometry as EKF input if dead-reckoning is insufficient

**Exit criteria:** Robot can maintain <0.2m cross-track error on a straight mowing line. GPS dropout of 30s causes <0.5m drift.

### Phase 3: Obstacle Detection Upgrade

- [ ] Collect yard obstacle dataset (people, pets, balls, toys, hoses, rocks, sprinkler heads)
- [ ] Fine-tune YOLOv8n from COCO checkpoint
- [ ] Export to OpenVINO IR, deploy to OAK-D VPU
- [ ] Implement depth-validated detection pipeline
- [ ] Implement temporal tracking (3-frame confirmation for non-critical classes)
- [ ] Implement tiered safety response
- [ ] Validate false-positive rate on shadows, leaves, fence patterns
- [ ] Validate detection rate on all target classes

**Exit criteria:** <1 false positive per minute in normal yard conditions. Detects a toy/ball at 3m. Detects a person at 8m+. Single-frame stop response < 200ms.

### Phase 4: Mower Integration

- [ ] Wire up blade motor control hardware (ESC or relay)
- [ ] Implement `BladeController` with safety interlocks
- [ ] Integrate blade control into mower state machine
- [ ] Implement coverage tracking (mark strips complete as driven)
- [ ] Implement return-to-dock on low battery
- [ ] Implement mission resume after charge
- [ ] Battery-aware mission planning (estimate coverage time from current %)
- [ ] Add mower status to dashboard (blade on/off, coverage %, ETA)

**Exit criteria:** Full autonomous mow of a bounded test zone with blade running. Auto-stops on obstacle detection. Returns to dock on low battery.

### Phase 5: Field Testing

- [ ] Boundary accuracy test: compare recorded vs. actual perimeter
- [ ] Path following accuracy: measure cross-track error over full mowing run
- [ ] Obstacle detection validation: test all detection classes in realistic conditions
- [ ] GPS dropout test: drive under tree canopy, measure position error on exit
- [ ] Full-yard autonomous mow: complete a real yard without intervention
- [ ] Multi-session coverage: stop mid-mow, recharge, resume
- [ ] Slope test: characterize traction and blade angle on max-grade areas in yard

**Exit criteria:** Successfully mow target yard 3 times consecutively without false stops, missed zones, or intervention.

---

## Key Configuration Parameters

These will live in `config.py` under a `MowerConfig` dataclass:

```python
@dataclass
class MowerConfig:
    row_spacing: float = 0.28        # meters; set to blade_width * (1 - overlap)
    overlap_fraction: float = 0.08   # 8% strip overlap
    turn_radius: float = 0.4         # min Dubins curve turn radius (meters)
    blade_stop_distance: float = 1.0 # stop blade if obstacle within this distance
    gps_dropout_timeout: float = 30.0  # seconds before pausing mission
    dead_reckoning_max_drift: float = 0.5  # meters before requiring re-acquisition
    battery_return_threshold: float = 0.2  # return to dock at 20% battery
    coverage_cell_size: float = 0.1  # meters, coverage map resolution
```

---

## Open Questions

These need answers before or during Phase 0/1 to unblock downstream decisions.

1. **Blade motor type** — What motor and deck are we using? Brushless spindle with ESC, or relay-switched AC motor? This determines the blade control interface entirely.
2. **RTK base station** — Do we have one, or are we using NTRIP corrections? Own base station is strongly recommended for reliability. ~$400 for a second F9P module.
3. **Mowing height** — Manual adjust or actuated? If actuated, needs its own control channel.
4. **Charging dock** — Manual plug-in or autonomous docking? Autonomous docking is complex (sub-5cm alignment needed); manual plug-in is fine for v1.
5. **Rain detection** — Abort mowing in rain? Wet grass clumps, increases slip, and accelerates blade wear. Simple: moisture sensor on deck + weather API check before mission start.
6. **Max slope in yard** — Tracks help with traction, but blade angle matters for cut quality and safety. What's the steepest section?
7. **Operating hours** — Daytime only? Any local noise ordinances to respect?
8. **Boundary wire fallback** — Pure RTK boundary, or do we want wire as a physical backup for GPS-challenged areas? Husqvarna EPOS supports both. Adds cost and install effort but is more reliable in heavy canopy.

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| RTK dropout under tree canopy | High | Medium | Dead-reckoning EKF; GPS quality threshold; pause-and-wait logic |
| False positive stops (shadows, leaves) | Medium | Low | Depth validation; temporal confirmation; tune confidence threshold |
| Missed obstacle detection | Low | High | Blade interlock by proximity; slow approach speed near cover; test all classes |
| Blade motor integration complexity | Medium | Medium | Start with relay (simpler) before ESC; define interface early (Phase 0) |
| Coverage tracker drift causing missed strips | Medium | Low | 8–10% row overlap as buffer; cross-track error monitoring |
| Battery runs out mid-field | Low | Low | 20% return threshold; battery-aware pre-mission estimate |
