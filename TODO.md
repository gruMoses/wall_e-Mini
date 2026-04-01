# WALL-E Mini TODO

Last updated: 2026-04-01
Scope: Bug fixes and features tracked here; see `wall_e-Mini/docs/performance_optimization_todo.md` for the full engineering roadmap.

---

## Bug Fixes

- [x] **Reverse obstacle avoidance is wrong** *(fixed in 2002092)*
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

- [x] **Graceful shutdown on low battery (VESC voltage)**
  - VESC broadcasts STATUS_5 frames (CAN_PACKET_STATUS_5, packet id 27) containing pack input voltage.
  - Background CAN RX thread in VescCanDriver parses STATUS / STATUS_4 / STATUS_5 frames from both motors.
  - Configurable threshold (default 22.4 V) with a 10 s sustained-low-voltage delay before triggering.
  - Hysteresis: voltage must stay below threshold for the full delay; recovery resets the timer. Once triggered, the latch never clears.
  - On trigger: motors stopped, stop-event set, then `sudo shutdown -h now` via a separate thread (avoids RX-thread deadlock).
  - Config: `VescConfig.voltage_shutdown_threshold_v`, `voltage_shutdown_delay_s`, `left_can_id`, `right_can_id`.
  - Tests: `pi_app/tests/test_vesc_telemetry.py` covers frame parsing, threshold/delay/hysteresis/latch, and RX thread integration.

### P2 (Medium Priority)

- [ ] **Daly BMS Bluetooth communication**
  - Add direct BMS communication over the Pi 5's built-in Bluetooth (the Pi at 192.168.86.54 running Wally).
  - Expose real-time cell voltages, state of charge (SOC), pack temperature, and other BMS telemetry to the web dashboard.
  - Use the standard Daly BMS serial-over-Bluetooth protocol; surface data via the existing telemetry/dashboard pipeline.

- [x] **Segment log files by arm/disarm cycle** *(done in 7d39655)*
  - Start a new log file each time WALL-E is armed or disarmed, instead of one continuous log.
  - Makes it much easier to analyze individual sessions.
