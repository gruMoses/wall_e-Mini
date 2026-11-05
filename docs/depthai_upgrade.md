# depthai upgrade: 3.3.0 to 3.6.1 or later

Status: prepared 2026-11-05, not done. The robot runs depthai 3.3.0. This
procedure is for Kevin and Claude together, with the robot on the test stand.

## 1. Why upgrade

- depthai 3.3.0 aborts the process when the OAK drops off USB during a
  session (`Resource deadlock avoided` in `PipelineImpl::wait`). systemd
  restarts wall-e. If the camera is not back yet, wall-e starts without it
  ("OAK-D Lite not detected") and runs with no obstacle avoidance and no
  follow-me until the next restart. This happened on 2026-11-05 at 14:59.
- The fix is in depthai 3.6.0 and later (PR luxonis/depthai-core#1723). Refer
  to `docs/NEXT_SESSION.md`, the depthai teardown item.
- Later versions add record and replay with calibration (3.6.1), newer YOLO
  parsers, and an `Odometry` message for VIO (3.11). Refer to section 0 of
  `docs/oak_d_lite_capability_audit.md`.

## 2. The two silent changes, and what the code does about them

| Change in depthai 3.6.1 | Effect on WALL-E | Protection in the code |
|---|---|---|
| The `RAW` IMU reports come directly from the IMU. Before, they were rotated into the RDF frame. `UNCALIBRATED` is the old behaviour. | The yaw channel (`gyro_y`, +Y down) and the boot gravity check assume RDF. With `RAW` on 3.6.1, the yaw axis can move or change sign. | `oak_depth.imu_report_names()` requests `RAW` before 3.6.1 and `UNCALIBRATED` from 3.6.1. Startup logs the choice. |
| AutoCalibration runs at pipeline start by default. | It can change the calibration that the corridor and the person geometry use. | `oak_depth.py` sets `DEPTHAI_AUTOCALIBRATION=OFF` unless the variable is already set. Startup logs the value. |

Source: the depthai v3.6.1 release notes and the 3.6.1 `IMUSensor` stubs.
3.3.0 has no `ACCELEROMETER_UNCALIBRATED`, so 3.3.0 must keep `RAW`.

`requirements.txt` pins `depthai==3.3.0`. `bin/auto_deploy.sh` does not run
`pip install`, so the pin does not change the robot by itself.

## 3. Startup line to look for

Each OAK session logs one line at WARNING level:

```
OAK device: depthai 3.3.0, USB SUPER, IMU reports ACCELEROMETER_RAW + GYROSCOPE_RAW, DEPTHAI_AUTOCALIBRATION=OFF
```

Read it with `sudo journalctl -u wall-e -b | grep "OAK device"`.

## 4. Procedure

CAUTION: The OAK allows one process at a time. Each step that runs a tool
needs `wall-e.service` stopped. Get Kevin's permission to stop it, with the
robot disarmed.

1. Record the baseline on 3.3.0. Stop wall-e. Run
   `python3 tools/depthai_upgrade_check.py` with the robot at rest on a level
   floor. Keep the output. The expected result is PASS: fx 456.89, cx 334.95,
   and the `RAW` family with gravity on -Y.
2. Install the new version for the user that runs wall-e (it uses the
   system `python3` with the user site-packages):
   `python3 -m pip install --user --break-system-packages depthai==3.6.1`
   (the system Python is marked externally managed). Do not use 3.12.0 for the
   first step. To go back, install `depthai==3.3.0` the same way.
   Before this step, test 3.6.1 without changing what wall-e uses: a copy
   is staged in `/home/pi/depthai-3.6.1` (installed 2026-11-05 with
   `pip install --no-deps --target`; it imports, and it has `BasaltVIO` and
   the `UNCALIBRATED` reports). Run a tool on it with
   `PYTHONPATH=/home/pi/depthai-3.6.1 python3 tools/depthai_upgrade_check.py`.
   wall-e still imports 3.3.0 from the user site-packages.
3. Run `python3 tools/depthai_upgrade_check.py` again. The expected result is
   PASS, with the `UNCALIBRATED` family showing gravity on -Y, and the same
   fx and cx. If the robot's family does not show gravity on -Y, stop and go
   back to 3.3.0.
4. Start wall-e. Read the "OAK device" line (section 3). It must show
   `UNCALIBRATED` and `DEPTHAI_AUTOCALIBRATION=OFF`.
5. Run the chalk SIGN test (`docs/heading_tuning.md`). A right turn must
   increase the heading. If the sign is wrong, fix it at the reader. Do not
   add an inversion downstream.
6. Check vision on the stand: camera health HEALTHY, `oak.det_latency_s` near
   0.20 s at idle, person detections, depth corridor, no colour-stall
   restarts in 10 minutes.
7. Unplug and replug the OAK once, with the robot disarmed. wall-e must stay
   up (no abort in the journal), and the camera must come back by itself.
8. Change the pin in `requirements.txt` to the version that passed. Record
   the results in this document.

### Results, 2026-11-05 17:09 (steps 1 and 3, robot at rest on the stand)

`tools/depthai_upgrade_check.py`, wall-e stopped, the 3.6.1 run from the
staged copy. Both runs: USB SUPER, DEPTHAI_AUTOCALIBRATION=OFF, CAM_A
fx 456.89 cx 334.95, RESULT PASS.

| depthai | Report | Accelerometer (g) | Gravity | Gyro at rest (deg/s) |
|---|---|---|---|---|
| 3.3.0 | RAW (the robot's) | (-0.009, -0.988, -0.046) | -Y | (+0.232, -0.643, +0.127) |
| 3.3.0 | CALIBRATED | refused: "BMI270 supports only ACCELEROMETER_RAW and/or GYROSCOPE_RAW" | | |
| 3.6.1 | RAW | (-0.009, +0.988, +0.047) | +Y | (+0.210, +0.598, -0.161) |
| 3.6.1 | UNCALIBRATED (the robot's) | (-0.009, -0.988, -0.048) | -Y | (+0.218, -0.546, +0.146) |
| 3.6.1 | CALIBRATED | (-0.009, -0.988, -0.048) | -Y | (+0.227, -0.523, +0.134) |

The 3.6.1 RAW frame has Y and Z reversed against the old RAW (a half turn
about X), so `gyro_y` changes sign. Without the version gate the heading
would turn the wrong way, or be saved only by the inverted-mount sign from
the gravity check. 3.6.1 UNCALIBRATED matches 3.3.0 RAW. 3.6.1 also accepts
CALIBRATED on the BMI270, which 3.3.0 refused; it is a later option.

Still to do: steps 4 to 8 (wall-e on 3.6.1, the chalk SIGN test, the
vision checks, the unplug test).

## 5. Visual-inertial odometry (VIO) probe

`tools/vio_probe.py` runs depthai's `BasaltVIO` node and only logs the pose
to `logs/vio_probe_*.csv`. It does not control the motors. `BasaltVIO` is in
the Python API of both 3.3.0 and 3.6.1 for the Pi (aarch64 wheels, checked
2026-11-05). It runs on the Pi, not on the camera.

CAUTION: Do not give VIO control authority until the logged runs show that it
is better than the gyro. A featureless view or netting over the lens can
blind it.

Runs to do (wall-e stopped, robot moved by hand or on the stand):

1. Parked for 60 s: the yaw and position drift at rest.
2. A 90 degree right turn on the chalk marks: the yaw change and its sign.
3. A straight push along a tape: the distance against the tape.
4. The CPU figure that the tool prints, at 30 fps and at 60 fps.

## 6. Open items

- Grok review 2026-11-05 (session `01a1adba`): the boot gravity check in
  `oak_imu` only logs a warning. A wrong IMU frame then gives a wrong
  heading without a stop. Make a failed gravity check (gravity not on -Y,
  or a magnitude far from 1 g) disable the heading-dependent features
  (MANUAL heading hold, waypoint navigation) until a restart. This is a
  behaviour change: get Kevin's decision first.
- Adopted from the same review: an unparseable depthai version now selects
  the IMU report from the enum, not RAW.
- When the OAK is missing at startup, wall-e runs without the camera until
  the next restart. Decide with Kevin whether wall-e must start the camera
  reader anyway and let its supervisor wait for the device.

## Appendix: Technical Names

depthai, OAK, OAK-D Lite, BasaltVIO, VIO, RDF, IMU, `RAW`, `UNCALIBRATED`,
AutoCalibration, `DEPTHAI_AUTOCALIBRATION`, wall-e, `wall-e.service`,
systemd, YOLO, aarch64, fx, cx, CSV, pip.
