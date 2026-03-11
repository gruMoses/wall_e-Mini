# Trail-Following Pure Pursuit — Implementation Plan

## Problem

The current Follow Me controller uses **direct pursuit**: it steers straight toward
the person's current position. If the person walks around an obstacle (tree, corner
of a building), the robot cuts the corner and collides with it.

## Goal

The robot should follow **the path the person walked**, not a straight line to where
they are now. If the person goes around a tree, the robot goes around the tree.

---

## Architecture Overview

```
 Person detected at (x_cam, z_cam)
       │
       ▼
 ┌─────────────────┐     ┌──────────────────┐
 │  Odometry Module │────▶│  Trail Manager   │
 │  (IMU + motors)  │     │  (breadcrumb     │
 │  → robot pose    │     │   deque in world │
 │    (x, y, θ)     │     │   coordinates)   │
 └─────────────────┘     └────────┬─────────┘
                                  │
                                  ▼
                          ┌──────────────────┐
                          │  Pure Pursuit     │
                          │  Controller       │
                          │  → (left, right)  │
                          │    motor bytes    │
                          └──────────────────┘
```

Three new modules, one modified module:

| Module | File | Responsibility |
|--------|------|----------------|
| **Odometry** | `pi_app/control/odometry.py` | Dead-reckoning robot pose from IMU heading + motor speed |
| **Trail Manager** | `pi_app/control/trail.py` | Accumulate person waypoints in world frame, prune old ones |
| **Pure Pursuit** | `pi_app/control/pure_pursuit.py` | Follow a trail of waypoints, output motor bytes |
| **FollowMeController** (modified) | `pi_app/control/follow_me.py` | Orchestrate the above; fall back to direct pursuit when appropriate |

---

## Phase 1: Dead-Reckoning Odometry

### Why dead reckoning?

- We already have IMU heading (`heading_deg`) and motor commands.
- The trail is short-lived (robot consumes waypoints within seconds), so drift is
  negligible over the ~5–15 second lifespan of any waypoint.
- No new dependencies or hardware required.
- If drift proves problematic on dirt (wheel slip), we can upgrade to VIO later
  without changing the trail/pursuit modules.

### Implementation: `pi_app/control/odometry.py`

```python
@dataclass
class RobotPose:
    x: float       # meters, world frame
    y: float       # meters, world frame
    theta: float   # radians, world frame heading (0 = initial forward)
    timestamp: float

class DeadReckonOdometry:
    def __init__(self, speed_scale: float):
        """
        speed_scale: converts motor byte offset from neutral (e.g. 40)
                     to linear velocity in m/s. Measured empirically.
        """

    def update(self, heading_deg: float, motor_l: int, motor_r: int,
               timestamp: float) -> RobotPose:
        """Called every control loop (~10 Hz).

        Uses IMU heading directly (not integrated gyro) to avoid yaw drift.
        Forward speed estimated from average motor byte offset × speed_scale.
        """

    def reset(self):
        """Zero the pose. Called on mode entry."""

    @property
    def pose(self) -> RobotPose:
        """Current estimated pose."""
```

**Key design decisions:**

- **Heading from IMU, not gyro integration.** The OAK-D IMU gives absolute heading
  via its rotation vector. This avoids gyro drift entirely for yaw.
- **Forward speed from motor commands.** `v = (motor_l + motor_r - 252) / 2 * speed_scale`.
  This assumes motor bytes map roughly linearly to wheel speed — not perfect on dirt,
  but good enough for short-lived waypoints.
- **`speed_scale` calibration.** Measure by driving straight at a known motor byte for
  a known time, measure distance traveled. One constant. Can be done with the
  calibration tool or manually.
- **Position update:** `x += v * cos(θ) * dt`, `y += v * sin(θ) * dt`, where θ comes
  from IMU heading.

### Data needed from controller.py

- `heading_deg` — already available from `ImuSteeringCompensator` / `get_imu_status()`
- `motor_l`, `motor_r` — the commanded motor bytes (post-slew-limiter)
- `timestamp` — `time.monotonic()`

All available in the existing `controller.process()` loop.

---

## Phase 2: Trail Manager

### Implementation: `pi_app/control/trail.py`

```python
@dataclass(frozen=True)
class TrailPoint:
    x: float          # world-frame meters
    y: float          # world-frame meters
    timestamp: float  # when the person was at this position
    speed_hint: float # robot speed at time of recording (for speed control)

class TrailManager:
    def __init__(self, config: TrailConfig):
        self._trail: deque[TrailPoint] = deque(maxlen=config.max_trail_points)
        self._min_spacing_m: float  # minimum distance between consecutive points
        self._max_age_s: float      # prune points older than this

    def add_detection(self, person_x_cam: float, person_z_cam: float,
                      robot_pose: RobotPose, speed_offset: float):
        """Transform person position from camera frame to world frame
        and append to trail if far enough from the last point."""

    def prune(self, robot_pose: RobotPose):
        """Remove waypoints that the robot has already passed
        (behind the robot) or that are too old."""

    def get_trail(self) -> list[TrailPoint]:
        """Return the current trail for Pure Pursuit."""

    def clear(self):
        """Reset trail on mode exit or target loss timeout."""

    @property
    def length(self) -> int:
        """Number of active trail points."""
```

**Camera-to-world transform:**

```
# Person in camera frame: (x_cam, z_cam) where x=right, z=forward
# Robot pose: (rx, ry, rθ) in world frame

world_x = rx + z_cam * cos(rθ) + x_cam * sin(rθ)   [VERIFY SIGN]
world_y = ry + z_cam * sin(rθ) - x_cam * cos(rθ)   [VERIFY SIGN]
```

Note: sign convention depends on whether heading increases CW or CCW. Must verify
against the OAK-D IMU's convention during implementation.

**Trail spacing:** Only add a new point if it's ≥ `min_spacing_m` (e.g., 0.3m) from the
last point. This prevents flooding the deque when the person is stationary and keeps
the trail clean for Pure Pursuit.

**Pruning strategy:**

1. **Age-based:** Remove points older than `max_age_s` (e.g., 30s).
2. **Behind-robot:** Remove points that are behind the robot's current position
   (dot product of point-to-robot vector with robot heading is negative).
3. **Closest-point advance:** When the robot reaches within `consume_radius_m` of
   the oldest point, pop it.

---

## Phase 3: Pure Pursuit Controller

### Implementation: `pi_app/control/pure_pursuit.py`

```python
@dataclass(frozen=True)
class PursuitCommand:
    speed_byte: float    # forward speed offset from neutral
    steer_byte: float    # steering differential offset
    lookahead_x: float   # world coords of current lookahead target (for logging)
    lookahead_y: float
    trail_remaining: int # how many waypoints left

class PurePursuitController:
    def __init__(self, config: PurePursuitConfig):
        self._lookahead_m: float    # lookahead distance along trail
        self._max_speed_byte: float
        self._wheelbase_m: float    # effective track width for curvature→steer

    def compute(self, robot_pose: RobotPose,
                trail: list[TrailPoint]) -> PursuitCommand | None:
        """Given robot pose and trail, compute motor commands.

        Returns None if trail is empty (fall back to direct pursuit).
        """
```

**Pure Pursuit math (well-established):**

1. Find the point on the trail closest to the robot.
2. Walk forward along the trail from that point by `lookahead_m` meters.
   This is the **lookahead point** `(Lx, Ly)`.
3. Transform lookahead point to robot-local frame:
   ```
   dx = Lx - rx
   dy = Ly - ry
   local_x =  dx * cos(rθ) + dy * sin(rθ)   # forward
   local_y = -dx * sin(rθ) + dy * cos(rθ)   # lateral
   ```
4. Compute curvature: `κ = 2 * local_y / L²` where `L = lookahead_m`.
5. Convert curvature to steering differential:
   ```
   steer_byte = κ * wheelbase_m * speed_byte / 2
   ```
   Clamp to `max_steer_offset_byte`.

**Speed control:** Use the same proportional-to-distance-error logic as today, but
measured to the **end of the trail** (the person's current position), not the
nearest waypoint.

**Lookahead distance tuning:**
- Too short → oscillation, especially on dirt
- Too long → cuts corners (defeats the purpose)
- Start with 0.8–1.2m, tune from there
- Could scale with speed: `lookahead = base + k * speed`

---

## Phase 4: Integration into FollowMeController

### Modified `follow_me.py` flow

```
compute(detections) →
  1. select_target(detections) → person or None
  2. If person detected:
       a. Update trail with person's world-frame position
       b. Compute speed_offset from distance error (same as today)
  3. If trail has ≥ 2 points:
       a. Pure Pursuit on trail → steer_offset
  4. Else (trail empty or too short):
       a. Direct pursuit (current PD controller) → steer_offset
  5. Combine speed_offset + steer_offset → (left, right)
```

**Fallback to direct pursuit:** When the trail is very short (person just appeared,
or person is close and centered), Pure Pursuit doesn't help — you're essentially
right behind them. Fall back to the existing PD controller for the "last mile."

**Threshold:** Use direct pursuit when:
- Trail has < 2 points, OR
- Person is within `direct_pursuit_distance_m` (e.g., 2.0m) AND lateral offset < 0.3m

This gives a smooth blend: Pure Pursuit for the curvy parts, direct pursuit for
the final approach.

### Lost target behavior

- Keep the trail alive during `lost_target_timeout_s`.
- Continue following the trail via Pure Pursuit (the robot will drive along the
  last known path toward where the person was).
- This replaces the current "spin toward last known x" behavior — much better,
  because the robot drives toward where you WERE along the path, not just turns.
- After timeout, clear the trail and stop.

---

## Phase 5: Configuration

### New config dataclass: `TrailFollowConfig`

```python
@dataclass(frozen=True)
class TrailFollowConfig:
    enabled: bool = True               # Feature flag; False = use direct pursuit only

    # Odometry
    speed_scale_mps_per_byte: float = 0.01  # Calibrate: m/s per motor byte offset

    # Trail
    max_trail_points: int = 100        # Ring buffer size
    min_spacing_m: float = 0.3         # Min distance between consecutive points
    max_age_s: float = 30.0            # Prune points older than this
    consume_radius_m: float = 0.4      # Pop waypoint when robot is this close

    # Pure Pursuit
    lookahead_base_m: float = 1.0      # Base lookahead distance
    lookahead_speed_scale: float = 0.005  # Additional lookahead per speed byte
    wheelbase_m: float = 0.28          # Track width (wheel-to-wheel) for curvature calc

    # Fallback thresholds
    direct_pursuit_distance_m: float = 2.0  # Use direct pursuit when person is closer
    direct_pursuit_lateral_m: float = 0.3   # ...and lateral offset is small
    min_trail_points_for_pursuit: int = 2   # Need at least this many points
```

### Feature flag

`TrailFollowConfig.enabled = True` activates trail following.
`enabled = False` keeps the current direct-pursuit behavior unchanged.
This lets us A/B test easily.

---

## Phase 6: Telemetry & Debugging

Add to the structured JSON logs:

```json
{
  "trail": {
    "length": 12,
    "oldest_age_s": 4.2,
    "pursuit_mode": "trail",         // "trail" or "direct"
    "lookahead_x": 1.23,
    "lookahead_y": 0.45,
    "closest_trail_dist_m": 0.15
  },
  "odometry": {
    "x": 2.1,
    "y": -0.3,
    "theta_deg": 45.2
  }
}
```

This lets us visualize the trail and debug pursuit behavior from logs, the same
way we diagnosed steering issues tonight.

---

## Calibration: speed_scale

The one new constant that needs measuring: `speed_scale_mps_per_byte`.

**Method:** Drive straight at a known motor byte (e.g., NEUTRAL + 50 = 176) for
5 seconds on dirt. Measure distance traveled. `speed_scale = distance / (5 * 50)`.

Can add this as Phase 0 in the calibration tool, or just measure with a tape measure.

---

## Implementation Order

| Step | What | Risk | Notes |
|------|------|------|-------|
| 1 | `odometry.py` + unit tests | Low | Pure math, no hardware needed to test |
| 2 | `trail.py` + unit tests | Low | Data structure + coordinate transform |
| 3 | `pure_pursuit.py` + unit tests | Low | Well-known algorithm, testable with synthetic trails |
| 4 | Integration into `follow_me.py` | Medium | Wire up odometry feed from controller, feature-flagged |
| 5 | Config additions | Low | New dataclass, defaults that match current behavior |
| 6 | Telemetry additions | Low | Extra fields in log dict |
| 7 | `speed_scale` calibration | Low | Manual measurement or calibration tool phase |
| 8 | Field test: straight line | Low | Verify no regression from current behavior |
| 9 | Field test: walk around obstacle | Medium | The real test |
| 10 | Tune lookahead & trail params | Medium | Iterative, data-driven |

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Odometry drift on dirt (wheel slip) | Trail curves in wrong direction | IMU heading is absolute (no yaw drift); forward speed drift is bounded by short trail lifespan. Can add GPS fusion later. |
| Pure Pursuit oscillation | Jerky steering | Lookahead distance is the primary damper. Start conservative (1.0m+). The existing `max_steer_offset_byte` cap still applies. |
| Stale trail after long pause | Robot follows phantom path | Age-based pruning (30s) + clear on mode exit handles this. |
| Camera-to-world sign error | Trail mirrored or rotated | Unit test with known poses and detections. Verify on first live run. |
| CPU overhead of trail management | Slows control loop | Trail is a deque of floats — negligible. Pure Pursuit is O(N) on trail length, N ≤ 100. |
| Person walks backward toward robot | Trail doubles back on itself | Prune behind-robot points aggressively. Direct pursuit fallback handles close range. |

---

## What We're NOT Doing (Yet)

- **Visual-Inertial Odometry (VIO):** Overkill for now. Dead reckoning is sufficient
  for short-lived trails. If dirt drift is bad, VIO via Spectacular AI is the upgrade.
- **Global path planning / costmap:** We don't have a map. Pure Pursuit on a
  breadcrumb trail is reactive, not planned. Good enough for obstacle avoidance
  by following the human's path.
- **Multi-person trail disambiguation:** We track one person. The trail belongs
  to the tracked target.
- **Reverse driving:** If the robot overshoots a waypoint, it prunes it rather
  than backing up.

---

## Dependencies

- No new Python packages required.
- No new hardware required.
- Uses existing IMU heading, motor commands, and person detections.
