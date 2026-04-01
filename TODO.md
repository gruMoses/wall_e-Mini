# WALL-E Mini TODO

Last updated: 2026-03-30
Scope: Bug fixes and features tracked here; see `wall_e-Mini/docs/performance_optimization_todo.md` for the full engineering roadmap.

---

## Bug Fixes

- [ ] **Reverse obstacle avoidance is wrong**
  - The camera faces forward, so forward-facing obstacle detections must NOT inhibit reverse motion — only forward motion should be gated by camera-detected obstacles.
  - Current behavior: reverse direction is blocked when a front-camera obstacle is detected, which is nonsensical since the camera cannot see behind the robot.
  - Fix: remove any obstacle-based restriction on reverse commands; limit camera obstacle gating strictly to the forward direction.

---

## Safety

- [ ] **Inhibit motion when charger is connected**
  - If the charger is connected, the robot must refuse all drive commands.
  - Prevents driving while tethered, which could damage the cable or charger and create a hazard.
  - Detect charger state (GPIO, VESC telemetry, or BMS flag) and gate motion commands at the controller level before they reach the motors.

---

## Features

### P1 (High Priority)

- [ ] **Graceful shutdown on low battery (VESC voltage)**
  - Monitor VESC voltage readings and trigger a safe Pi shutdown sequence before the battery cuts out.
  - Use VESC telemetry rather than BMS data — the Daly BMS tends to go to sleep after a while, making it unreliable as a shutdown trigger.
  - Define a low-voltage threshold (with hysteresis) and initiate a clean OS shutdown so the Pi doesn't lose state abruptly.

### P2 (Medium Priority)

- [ ] **Daly BMS Bluetooth communication**
  - Add direct BMS communication over the Pi 5's built-in Bluetooth (the Pi at 192.168.86.54 running Wally).
  - Expose real-time cell voltages, state of charge (SOC), pack temperature, and other BMS telemetry to the web dashboard.
  - Use the standard Daly BMS serial-over-Bluetooth protocol; surface data via the existing telemetry/dashboard pipeline.

- [ ] **Segment log files by arm/disarm cycle**
  - Start a new log file each time WALL-E is armed or disarmed, instead of one continuous log.
  - Makes it much easier to analyze individual sessions.
