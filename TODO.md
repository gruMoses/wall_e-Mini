# WALL-E Mini TODO

Last updated: 2026-04-01 (BMS + charger inhibit complete)
Scope: Bug fixes and features tracked here; see `wall_e-Mini/docs/performance_optimization_todo.md` for the full engineering roadmap.

---

## Bug Fixes

- [x] **Reverse obstacle avoidance is wrong** *(fixed in 2002092)*
  - The camera faces forward, so forward-facing obstacle detections must NOT inhibit reverse motion — only forward motion should be gated by camera-detected obstacles.
  - Current behavior: reverse direction is blocked when a front-camera obstacle is detected, which is nonsensical since the camera cannot see behind the robot.
  - Fix: remove any obstacle-based restriction on reverse commands; limit camera obstacle gating strictly to the forward direction.

---

## Safety

- [x] **Inhibit motion when charger is connected**
  - Implemented via Daly BMS Bluetooth: `BmsService.is_charging()` (charge FET on + positive current) feeds `controller.set_charger_inhibit()` each main-loop tick.
  - Fail-open: if BMS is unreachable for >30 s the inhibit clears automatically, so a dropped BLE link never bricks the robot.
  - See `pi_app/hardware/bms.py` and `pi_app/control/controller.py`.

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

- [x] **Daly BMS Bluetooth communication**
  - `pi_app/hardware/bms.py`: threaded `BmsService` connects to Daly SPIM08HP over BLE (bleak), polls SOC / cell voltages / temperature / MOSFET status / errors every 8 s.
  - Auto-reconnect with exponential backoff (5 s → 60 s cap). BLE failures never block the main control loop.
  - `BmsConfig` in `config.py`: `bms_mac_address`, `bms_poll_interval_s`, `bms_timeout_s`, `charger_inhibit_enabled`.
  - BMS state surfaced in the structured JSON telemetry log each loop tick.
  - Set `config.bms.enabled = True` and `bms_mac_address` to your BMS MAC to activate.

- [x] **Segment log files by arm/disarm cycle** *(done in 7d39655)*
  - Start a new log file each time WALL-E is armed or disarmed, instead of one continuous log.
  - Makes it much easier to analyze individual sessions.
