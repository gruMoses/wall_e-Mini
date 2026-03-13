# Trail-Following Strategies for Ground-Based Rovers

**Research Date:** 2026-03-13
**Goal:** Find the best approach for Wall-E to follow the path a person walked (breadcrumb trail), not a direct beeline to the person.

---

## The Problem

Person walks a path → robot records their positions as breadcrumbs → person disappears or moves on → robot follows the exact path that was walked. This is the **Teach and Repeat** paradigm.

Our current implementation uses basic pure pursuit with a fixed lookahead on raw breadcrumb points. This research identifies what the state of the art looks like and what upgrades would make the biggest impact.

---

## Key GitHub Repos (Ranked by Relevance)

### Tier 1: Directly Applicable

| Repo | Stars | What It Does | Key Takeaway |
|---|---|---|---|
| [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | 23,000+ | Clean Python implementations of pure pursuit, Stanley, MPC, DWA, path smoothing, and more | **Best single reference for prototyping.** Port algorithms directly. |
| [Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller) | 2,000+ | Production pure pursuit with curvature-based velocity scaling, proximity slowdown, and collision detection | **Best production-quality path follower.** The three regulation layers are exactly what we need. |
| [nickcharron/waypoint_nav](https://github.com/nickcharron/waypoint_nav) | ~200 | ROS1 "collect GPS waypoints by driving, then replay them" with move_base | **Exact same breadcrumb concept.** Record-and-replay pattern with obstacle avoidance. |

### Tier 2: Good References

| Repo | Stars | What It Does | Key Takeaway |
|---|---|---|---|
| [ArduPilot Rover](https://github.com/ArduPilot/ardupilot) | Massive | Production rover firmware with Follow mode + waypoint recording via RC switch | Most mature platform, but requires ArduPilot hardware stack |
| [Geonhee-LEE/mpc_ros](https://github.com/Geonhee-LEE/mpc_ros) | 374 | Nonlinear MPC for differential drive, works as move_base plugin | If we ever want MPC, this is the reference |
| [CL2-UWaterloo/f1tenth_ws](https://github.com/CL2-UWaterloo/f1tenth_ws) | — | Pure Pursuit + Stanley combined for waypoint following with obstacle avoidance | Shows how to blend two controllers |
| [fcaponetto/robot-follow-path-uwb](https://github.com/fcaponetto/robot-follow-path-uwb) | — | UWB-based person following with particle filter | Alternative to GPS/vision for person tracking |

### Tier 3: Algorithm References

| Repo | What It Does |
|---|---|
| [larics/pure_pursuit](https://github.com/larics/pure_pursuit) | Clean ROS1 pure pursuit reference |
| [Atharva-05/stanley_control](https://github.com/Atharva-05/stanley_control) | Stanley with visualization |
| [winstxnhdw/FullStanleyController](https://github.com/winstxnhdw/FullStanleyController) | Complete Stanley abstraction in Python |
| [alexliniger/MPCC](https://github.com/alexliniger/MPCC) | Model Predictive Contouring Controller |

### Key Research Paper

**"Algorithms and Sensors for Small Robot Path Following" (JPL/NASA, 2002)**
- URL: https://www-robotics.jpl.nasa.gov/media/documents/ICRALeaderFollower2002.pdf
- Directly addresses leader-follower path tracking (not direct pursuit)
- Tests pure pursuit, PI controller, and a blend
- Results: ~50cm max deviation over 40m paths
- Conclusion: pure pursuit was simplest and effective

---

## Algorithm Comparison

### Pure Pursuit (What We Have)
- **How it works:** Find a lookahead point on the path, compute the curvature arc from robot to that point, steer along the arc.
- **Our version:** Fixed lookahead, no speed regulation, global closest-point search.
- **Weaknesses:** Cuts corners on tight turns, no speed adaptation, can snap backward on crossing trails.

### Regulated Pure Pursuit (Recommended Upgrade)
Three regulation layers on top of basic pure pursuit:

1. **Adaptive lookahead:** `lookahead = clamp(velocity * lookahead_time, min_dist, max_dist)`. Look further ahead at speed, shorter when slow. Prevents oscillation at high speed and corner-cutting at low speed.

2. **Curvature-based velocity scaling:** Compute the turning radius from the pure pursuit arc. If radius < threshold, reduce speed proportionally. Minimum speed floor prevents stalling.
   ```
   if radius < regulated_min_radius:
       speed *= radius / regulated_min_radius
       speed = max(speed, min_speed)
   ```

3. **Collision detection:** Project the robot's footprint forward along the current velocity command for N seconds. If it intersects an obstacle in the costmap, stop. This is NOT obstacle avoidance — it's a safety stop.

**This is the single highest-impact upgrade we can make.** It addresses our three biggest weaknesses (fixed lookahead, no speed regulation, no obstacle awareness) with minimal complexity increase.

### Stanley Controller
- **How it works:** Uses front-axle position. Combines heading error + cross-track error via `steer = heading_error + arctan(k * crosstrack / velocity)`.
- **Problem for us:** Designed for bicycle/Ackermann steering. Awkward mapping to differential drive. Requires smooth paths — raw breadcrumbs cause instability.
- **Verdict:** Skip. Pure pursuit maps more naturally to differential-drive kinematics.

### Model Predictive Control (MPC)
- **How it works:** Optimizes a sequence of control inputs over a prediction horizon, minimizing tracking error subject to constraints.
- **Pi 5 feasibility:** Yes with C/C++ solvers (acados: sub-ms solve times). Python: 10-15 Hz.
- **Verdict:** Overkill for now. Consider only if regulated pure pursuit isn't accurate enough. The complexity jump is significant.

### Dynamic Window Approach (DWA)
- **How it works:** Samples (v, ω) velocity pairs, simulates short trajectories, scores each by: obstacle distance + path progress + heading alignment. Executes the best.
- **Use case:** Local obstacle avoidance layer, not primary path tracking.
- **Verdict:** Good future add-on for obstacle avoidance during trail following. Not a replacement for pure pursuit.

---

## Path Smoothing (Critical Missing Piece)

Raw breadcrumbs from GPS/odometry are noisy. Smoothing before feeding to the path follower makes a huge difference.

### Savitzky-Golay Filter (Recommended)
- Fits successive windows of points to a polynomial via least-squares
- Preserves path shape better than moving average
- Nav2 Savitzky-Golay Smoother runs in < 1ms
- Tuning: window size (larger = smoother) and polynomial order
- **Best for our case:** Fast, removes sensor noise, works with pure pursuit (which tolerates remaining roughness)

### Cubic Spline
- Piecewise cubics with continuous first and second derivatives
- Guarantees curvature continuity (useful for speed profiling)
- Can overshoot between widely-spaced points
- More computation than Savitzky-Golay but still trivial on Pi 5

### Recommendation
Apply Savitzky-Golay to the raw breadcrumbs. This alone will significantly improve path quality. If we later need curvature-continuous paths for speed profiling, add cubic spline as a second pass.

---

## Speed Profiling (Important Missing Piece)

Currently we use a fixed speed along the trail. We should vary speed based on path curvature.

### Curvature-Based Velocity Scaling
1. Compute curvature at each trail point using the three-point circle method:
   ```
   curvature = 2 * cross(p2-p1, p3-p2) / (|p2-p1| * |p3-p2| * |p3-p1|)
   ```
2. Set target velocity: `v = clamp(v_max / (1 + alpha * |curvature|), v_min, v_max)`
3. Apply acceleration limits to smooth transitions

### Look-Ahead Curvature
Don't just look at current curvature — look N points ahead so the robot starts decelerating BEFORE entering a turn. Critical for a robot with any mass/inertia.

---

## Recommended Upgrade Path

### Phase 1: Adaptive Lookahead + Path Smoothing
- **Adaptive lookahead:** `lookahead = clamp(speed * time_scale, min_dist, max_dist)`
- **Savitzky-Golay smoothing** on breadcrumb trail points
- **Running closest-point index** (don't search globally — search forward from last known position)
- Impact: Eliminates corner-cutting, reduces oscillation, handles speed changes gracefully

### Phase 2: Curvature-Based Velocity Scaling
- Compute curvature along the smoothed trail
- Scale speed inversely with curvature
- Add look-ahead curvature check (decelerate before turns)
- Impact: Smoother following, better tracking on tight turns

### Phase 3: Obstacle-Aware Trail Following
- Project robot footprint forward along current trajectory
- If obstacle detected within N seconds of travel, stop or slow
- If we add range sensors (ultrasonic/depth), consider DWA for local avoidance with path rejoin
- Impact: Safety during autonomous trail following

### Phase 4 (Optional): MPC
- Only if phases 1-3 don't meet accuracy requirements
- Use acados for C-generated solver on Pi 5
- Prediction horizon: 10-20 steps at 10 Hz = 1-2 seconds ahead

---

## Comparison to Our Current Implementation

| Feature | Current | After Upgrades |
|---|---|---|
| Lookahead | Fixed | Adaptive (speed-scaled) |
| Path smoothing | None (raw breadcrumbs) | Savitzky-Golay |
| Speed along trail | Fixed | Curvature-scaled |
| Closest point search | Global (can snap backward) | Forward-only running index |
| Collision detection | None | Footprint projection (Phase 3) |
| Path representation | Raw deque of points | Smoothed + curvature-annotated |
| Lookahead measurement | From closest trail point | From robot position (corrected) |

---

## Key Sources

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) — Algorithm implementations
- [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html) — Production controller docs
- [Biomimetic Adaptive Pure Pursuit (2024)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10813636/) — Adaptive lookahead paper
- [PP-DSC Enhanced Pure Pursuit (2025)](https://www.nature.com/articles/s41598-026-38695-1) — 68-82% improvement
- [JPL Leader/Follower (2002)](https://www-robotics.jpl.nasa.gov/media/documents/ICRALeaderFollower2002.pdf) — Classic reference
- [Path Smoothing Survey (Ravankar et al.)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6165411/) — Smoothing techniques
- [DWA + Path Following Fusion](https://pmc.ncbi.nlm.nih.gov/articles/PMC11057753/) — Obstacle avoidance
- [waypoint_nav](https://github.com/nickcharron/waypoint_nav) — Record-and-replay GPS waypoints
- [Visual Teach and Repeat 3](https://utiasasrl.github.io/vtr3/) — State-of-art visual teach and repeat
