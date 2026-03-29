# WALL-E Mini — Mower Mode Plan

*Created: 2026-03-29*

## Overview

Add an autonomous lawn mowing mode to WALL-E Mini. The robot already has the core hardware — RTK GPS, OAK-D Lite stereo camera, IMU, VESC motor controllers over CAN bus, differential drive tracks, and a web dashboard. This plan covers what we need to build on top of that.

**Navigation strategy:** RTK GPS systematic mowing with boustrophedon (back-and-forth) path planning. No random bouncing.

**Obstacle avoidance:** OAK-D Lite neural net + stereo depth fusion. Tiered response — hard stop for people/pets, slow+navigate for objects, map+avoid for permanent obstacles.

## What We Already Have

These existing WALL-E systems carry over directly:

- **RTK GPS** — DFRobot GNSS-RTK on I2C (0x20)
- **OAK-D Lite** — stereo depth + RGB + Myriad X neural compute (MobileNet person detection already running)
- **IMU** — external I2C with OAK-D BMI270 fallback
- **VESC motor controllers** — CAN bus with telemetry read-back capability (wheel speed feedback)
- **Pure pursuit path following** — trail_follow already implemented and field-tested
- **Depth-based obstacle avoidance** — already implemented
- **Safety state machine** — arm/disarm, e-stop, RC override
- **Web dashboard** — Flask + SSE telemetry
- **config.py dataclass pattern** — all tuning parameters centralized

## What's New

### Navigation
- Boundary mapping tool (RC drive perimeter, record RTK waypoints → GeoJSON polygon)
- Boustrophedon path planner (Fields2Cover library or custom implementation)
- Coverage tracker (which areas have been mowed, resume after charge)
- Multi-zone support (front yard, back yard, side strips)
- Exclusion zones (permanent obstacles mapped via GPS)
- Return-to-dock navigation

### Obstacle Detection
- Upgrade from MobileNet to YOLOv8n (multi-class, 20-25 FPS on Myriad X VPU)
- Custom yard obstacle dataset (200-500 images per class from our lawn)
- Depth-validated detection pipeline (shadows have no depth signal → zero false positives)
- Tiered safety response system

### Hardware
- Mower deck + blade motor control (ESC or relay — TBD)
- Blade safety interlocks (tilt, lift, obstacle proximity)
- Battery-aware mission planning

### Sensor Fusion
- Dual-EKF: RTK GPS (5Hz) + IMU (50Hz) + VESC wheel encoders (100Hz)
- Dead-reckoning for GPS dropout under tree canopy
- Optional: OAK-D visual odometry as additional input

## Reference Projects

| Project | What It Is | Why It Matters |
|---------|-----------|----------------|
| [OpenMower](https://github.com/ClemensElflein/OpenMower) | Most popular open-source RTK mower, ROS-based | Largest community, ublox F9P RTK, custom mainboard |
| [OAK-Mower](https://alemamm.github.io/oakmower/) | Spatial AI obstacle detection on OAK-D for OpenMower | Closest to our exact hardware stack |
| [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) | Path planning library for agricultural robots | Boustrophedon decomposition, Dubins curves, ROS2-compatible |
| Kenny Trussell's ArduRover mower | PixHawk + ZED-F9P, 5-18 acre fields | Most battle-tested DIY build, 2+ years of real mowing |
| [Greenzie](https://greenzie.com) | Commercial autonomous mowing | Boustrophedon planner was open-sourced (archived) |

## Commercial Mower Lessons

Key takeaways from studying Mammotion Luba, Husqvarna EPOS, Segway Navimow, and others:

- **Tree canopy is the universal weakness.** Every commercial RTK mower struggles under trees. Sensor fusion (IMU + wheel odometry dead-reckoning) is mandatory for bridging GPS gaps.
- **Multi-sensor fusion wins.** No single sensor is reliable enough. RTK + depth camera + IMU is the minimum viable stack. Mammotion's Luba 3 added LiDAR because RTK-only missed dark objects on the ground.
- **Own RTK base station > NTRIP** for residential. ~$400, set-and-forget, no internet dependency.
- **Husqvarna EPOS** achieves 1-2cm accuracy with local base station. That's our target.
- **Segway Navimow X3** has the best obstacle AI (300° FOV, 200+ object types via VSLAM). Worth studying their detection tiers.
- **App/connectivity issues** are the #1 complaint across all brands. Keep the dashboard simple and local.

## Obstacle Detection Strategy

### Model
**YOLOv8n** deployed on OAK-D Myriad X VPU. 20-25 FPS, Pi CPU stays near 0% during inference. Significantly better multi-class detection than current MobileNet-SSD.

### Detection Pipeline
1. Neural net detection (YOLOv8n on Myriad X)
2. Confidence threshold filtering
3. Depth validation (reject detections with no depth signal — kills shadow false positives)
4. Temporal tracking (persistence across frames)
5. Size validation (reject noise)

### Safety Tiers

| Tier | Trigger | Response | Confirmation |
|------|---------|----------|--------------|
| **STOP** | Person, child, pet | Immediate halt, blade stop | Single confident frame |
| **SLOW** | Ball, toy, unknown object | Reduce speed, navigate around | 3-frame confirmation |
| **MAP** | Rock, sprinkler head | Add to exclusion zone, avoid permanently | GPS-tagged, persisted |

### Critical OAK-D Note
Normal stereo mode has 0.7m minimum depth. **Must enable Extended Disparity mode** to halve to ~0.35m, or ground-level obstacles within 70cm are invisible to depth sensing.

### Custom Training Required
No public dataset covers garden obstacles well. FieldSAFE (agricultural mowing) is the closest. Plan to collect 200-500 images per class from our own yard:
- People, children (various poses, distances)
- Dogs, cats
- Balls, toys
- Garden hoses, sprinkler heads
- Rocks, sticks
- Lawn furniture

## Implementation Phases

### Phase 0: Boundary & Map
- Build boundary recording tool (RC drive perimeter while recording RTK waypoints)
- Save/load boundary as GeoJSON polygon
- Visualize boundary on dashboard map view
- Define and persist exclusion zones
- **Exit criteria:** Recorded boundary matches physical yard within 10cm

### Phase 1: Path Planning
- Integrate Fields2Cover or build custom boustrophedon planner
- Generate parallel mowing lines from boundary polygon (5-10% overlap)
- Visualize planned path on dashboard
- Handle multi-zone decomposition for complex yard shapes
- **Exit criteria:** Generated path covers >95% of boundary area

### Phase 2: Sensor Fusion Upgrade
- Add VESC wheel encoder feedback via CAN telemetry read-back (P1 on existing roadmap)
- Implement dual-EKF (RTK + IMU + wheel odometry)
- Test dead-reckoning accuracy during simulated GPS dropout
- Enable OAK-D Extended Disparity mode
- **Exit criteria:** <0.5m position drift over 30s GPS dropout

### Phase 3: Obstacle Detection Upgrade
- Train YOLOv8n on custom yard obstacle dataset
- Convert and deploy to OAK-D Myriad X VPU (.blob format)
- Implement tiered safety response (stop/slow/map)
- Depth-validated detection pipeline
- False positive filtering (shadow rejection, temporal confirmation)
- **Exit criteria:** Zero missed people/pets over 100 test encounters, <5% false positive rate

### Phase 4: Mower Integration
- Blade motor control (hardware integration — ESC or relay TBD)
- Blade safety interlocks (tilt sensor, lift detection, obstacle proximity)
- Return-to-dock on low battery
- Coverage tracking and resume-after-charge
- **Exit criteria:** Complete autonomous mow of one zone, dock, resume, finish

### Phase 5: Field Testing
- Boundary accuracy testing (GPS vs physical measurement)
- Path following accuracy (cross-track error during mow)
- Obstacle detection validation (all classes, various lighting)
- GPS dropout recovery testing (mow under tree canopy)
- Full-yard autonomous mow (multi-zone, multi-session)
- **Exit criteria:** 3 consecutive full-yard mows with zero safety incidents

## Open Questions

1. **Blade motor type** — spindle with brushless ESC? Belt-driven from a separate motor? What deck?
2. **RTK base station** — do we have one, or currently using NTRIP? Own base station recommended (~$400).
3. **Mowing height adjustment** — manual set-and-forget, or actuated for different zones?
4. **Charging dock** — manual plug-in or autonomous docking with alignment?
5. **Rain detection** — abort mowing in rain? Wet grass clumps, traction loss on slopes.
6. **Slope handling** — max grade in the yard? Tracks help but blade angle matters.
7. **Operating hours** — daytime only? Noise restrictions from neighbors?
8. **Boundary wire fallback** — pure RTK boundary, or wire as safety backup for GPS-challenged areas (Husqvarna EPOS does this)?
