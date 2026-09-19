# OAK-D Lite capability audit

Date: 2026-09-19. Device: OAK-D Lite (RVC2), depthai 3.3.0, Python 3.11 on the Pi 5. This document answers two questions from Kevin: "Should the OAK-D Lite heading have worked without host-side work?" and "Which OAK-D features do we not use, or use incorrectly?" A 41-agent research pass fetched the Luxonis documentation and compared it with `pi_app/hardware/oak_depth.py`. Each claim below has a source URL in section 7. Claims marked VERIFY need a check on the unit before you act on them.

## 1. Short answer

1. The heading could not "just work" on this camera. The OAK-D Lite has a BMI270. The BMI270 is a 6-axis part: gyroscope and accelerometer, no magnetometer, and no on-chip sensor fusion. Luxonis lists ROTATION_VECTOR and GAME_ROTATION_VECTOR as "No" for the BMI270. Absolute heading from this IMU is not possible. Host-side gyro integration is the correct design for this part.
2. The pipeline does not use the calibrated IMU reports. It requests `GYROSCOPE_RAW` and `ACCELEROMETER_RAW`. The Luxonis v3 documentation lists `GYROSCOPE_CALIBRATED` and `ACCELEROMETER_CALIBRATED` as supported on the BMI270. The calibrated path applies the factory bias, scale and cross-axis terms and rotates the sample into the Luxonis RDF frame (+X right, +Y down, +Z forward). VERIFY: confirm on this unit that the calibrated reports arrive and that the extrinsics are not identity.
3. The detection path has one root mistake. The YOLOv8n blob comes from a plain ultralytics ONNX export through blobconverter. That blob has no NN Archive `heads` metadata, so the on-device parsers have nothing to decode. That is why `DetectionNetwork`, `SpatialDetectionNetwork` and `ObjectTracker` returned nothing, and why the host does NMS, depth ROI math and tracklets today. The supported path is the Luxonis conversion tool, which produces an NN Archive that the device parses.

## 2. IMU facts for the BMI270

| Report | BMI270 | BNO08x |
|---|---|---|
| ACCELEROMETER_RAW / CALIBRATED | Yes | Yes |
| GYROSCOPE_RAW / CALIBRATED | Yes | Yes |
| MAGNETOMETER_RAW | No | Yes |
| ROTATION_VECTOR, GAME_ROTATION_VECTOR | No | Yes |
| LINEAR_ACCELERATION, GRAVITY | No (and deprecated on all parts) | Yes |

- Rates on the BMI270 round DOWN to the next supported rate. 100 Hz is inside the safe band. Above 400 Hz the gyroscope can jitter.
- Datasheet budget: zero-rate offset ±0.5 °/s at 25 °C; offset change ±0.015 °/s per kelvin. The robot showed a walk of −1.2 °/s over 35 minutes after a good boot calibration (refer to `docs/heading_tuning.md`). That is larger than the typical figure, so a host-side stationary bias tracker stays necessary with or without the calibrated reports.
- The Bosch CRT trim corrects gyroscope gain, not zero-rate offset, and depthai does not expose it. Do not plan on on-chip bias correction.
- Kickstarter OAK-D Lite units have no IMU. This unit reports IMU data, so it is a retail unit.
- Luxonis states the IMU is not synchronized with the cameras. Use `getTimestampDevice()` for integration dt (the code does this). Use the Sync node only if you correlate IMU with frames.

## 3. Feature gaps

Status values: USED = used correctly; UNUSED = available and not used; MISUSED = used in a way the documentation does not support; N/A = not available on the Lite.

| Feature | Status | Impact | Effort | Heading? | Action |
|---|---|---|---|---|---|
| GYROSCOPE_CALIBRATED / ACCELEROMETER_CALIBRATED | MISUSED (RAW requested) | high | small | yes | Enable the CALIBRATED reports beside RAW, log both at rest and in a turn, then switch. VERIFY. |
| IMU extrinsics (`getImuToCameraExtrinsics`) | UNUSED | high | small | yes | Read and log the matrix at boot beside the intrinsics line. Derive the yaw axis from it. Fall back to the pinned axis with a loud log. VERIFY. |
| On-device fusion (ROTATION_VECTOR) | N/A | – | – | no | Do not try it. Record the "No" in CLAUDE.md. |
| `getChipTemperature()` | UNUSED | medium | small | yes | Log the average temperature in the IMU metrics each health tick. Later, index the bias estimate by temperature. |
| Device diagnostics: `getConnectedIMU`, `getIMUFirmwareVersion`, `getUsbSpeed`, memory usage | UNUSED | medium | small | no | Log once per device session at WARNING. USB2 vs USB3 changes what the pipeline can carry. |
| FeatureTracker (camera-aided yaw) | UNUSED | medium | large | yes | Prototype after the calibrated-report change. It costs shaves that YOLO and stereo share. |
| YOLO through NN Archive + `DetectionNetwork` | MISUSED (raw blob, host NMS) | high | medium | no | Convert the model with the Luxonis tool. Then `SpatialDetectionNetwork` gives per-box depth on the device and `ObjectTracker` gives track ids on the device. This removes the host NMS, the host depth-ROI median and the host tracklet layer. |
| `SpatialDetectionNetwork` for person distance | UNUSED | medium | medium | no | Blocked by the NN Archive change. Do it after. |
| `ObjectTracker` | UNUSED on the YOLO path | high | medium | no | Blocked by the NN Archive change. Do it after. |
| Hand landmarks on device (Luxonis zoo) | UNUSED (MediaPipe on the Pi CPU) | low | large | no | Last. Shave budget is shared. |
| StereoDepth ROBOTICS preset | Unknown (DENSITY set) | medium | small | no | Do not change blind. The obstacle corridor is tuned to the current depth output. Test on a recording first. |
| Subpixel and extended disparity both enabled | MISUSED (VERIFY) | medium | small | no | v3 defaults subpixel ON; the code sets extended ON. The documentation says the two are exclusive on RVC2. Log the effective StereoDepth config and choose extended (0.35 m minimum range). |
| StereoDepth post-processing filters | UNUSED | medium | small | no | Enable the speckle filter first and measure. Median is hardware-accelerated. |
| SpatialLocationCalculator (obstacle corridor) | USED | – | – | no | Keep. |
| EEPROM intrinsics | USED | – | – | – | Keep. |
| Holistic RecordReplay | UNUSED | medium | medium | no | Keep the custom recorder for the operator. Add holistic record behind a developer flag for offline replay. |
| HostNode API | UNUSED | low | large | no | Do not refactor now. |
| Device auto-reconnect | USED (custom supervisor) | – | – | no | Keep. The library mechanism does not give the depth-age safety property. |
| Recording suppressed when gestures are on | MISUSED (silent) | medium | small | no | Log at WARNING and show "recording: suppressed" in health. |

Refuted during verification (no action): IMU batching (`setBatchReportThreshold(1)`, `setMaxBatchReports(10)`) matches the Luxonis examples; the Sync node is not needed for gyro integration.

## 4. Recommended sequence

1. Land the heading fixes (sign, live yaw rate, stationary bias tracker) on the RAW path. Field data validated the RAW path. Refer to `RESULT.md`.
2. Add boot diagnostics: `getConnectedIMU`, `getIMUFirmwareVersion`, `getUsbSpeed`, `getImuToCameraExtrinsics`, `getChipTemperature`. One log line each at WARNING.
3. Enable `GYROSCOPE_CALIBRATED` and `ACCELEROMETER_CALIBRATED` beside the RAW reports. Log both streams. Compare the rest mean and a 90-degree turn. Switch the integrator to CALIBRATED when the data agrees with the frame the extrinsics describe.
4. Convert YOLOv8n through the Luxonis tool to an NN Archive. Move detection, spatial location and tracking onto the device. This is the largest single improvement to the follow-me path.
5. Log the effective StereoDepth configuration. Resolve subpixel versus extended disparity. Try the speckle filter.
6. Later: FeatureTracker as a yaw-drift brake; hand landmarks on the device.

## 5. Hardware options for absolute heading

| Option | What you get | Trade-off |
|---|---|---|
| Keep the Lite; calibrated reports + GPS-COG lock | Correct frame and factory bias for free; absolute heading only from GPS | No magnetometer; heading stays relative between GPS locks |
| External BNO085 breakout on the Pi I2C bus (~USD 30) | On-chip 9-axis fusion, absolute heading, automatic gyro bias calibration | A new driver branch in `imu_reader.py`, a mount whose orientation you control, magnetic interference near the motors |
| OAK-D Pro (BNO086 + active IR stereo) | ROTATION_VECTOR through the same IMU node; IR helps depth in low texture | The most expensive way to fix a heading bug |
| OAK 4 D (RVC4) | 9-axis IMU, far more compute | ~USD 1,049; RVC2 blobs do not transfer |

The breakout is the cheap, correct route to absolute heading. It keeps the camera.

## 6. What this changes in the heading work

- The RAW path stays the production path until step 3 in section 4 is measured. The sign fix is validated on RAW by field data.
- The stationary bias tracker stays necessary in every option except the BNO085 breakout, because the calibrated reports remove a static bias, not the thermal walk.

## 7. Sources

- IMU node (v3): https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu
- IMU hardware page: https://docs.luxonis.com/hardware/platform/sensors/imu
- IMU properties header: https://raw.githubusercontent.com/luxonis/depthai-core/main/include/depthai/properties/IMUProperties.hpp
- Rotation vector example ("supported only by BNO086"): https://github.com/luxonis/depthai-python/blob/main/examples/IMU/imu_rotation_vector.py
- Ultralytics conversion (v3): https://docs.luxonis.com/software-v3/ai-inference/integrations/ultralytics
- SpatialDetectionNetwork: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/spatial_detection_network
- ObjectTracker: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/object_tracker
- StereoDepth: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth
- FeatureTracker: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/feature_tracker
- Holistic record and replay: https://docs.luxonis.com/software-v3/depthai/tutorials/holistic-record-replay/
- Host nodes: https://docs.luxonis.com/software-v3/depthai/depthai-components/host_nodes/
- ChipTemperature header: https://raw.githubusercontent.com/luxonis/depthai-core/main/include/depthai/common/ChipTemperature.hpp
- BMI270 datasheet: https://cdn.sparkfun.com/assets/9/a/2/9/6/bst-bmi270-ds000.pdf
- OAK-D Pro: https://docs.luxonis.com/hardware/products/OAK-D%20Pro
- BNO085 breakout: https://www.adafruit.com/product/4754
- OAK 4 D: https://shop.luxonis.com/products/oak-4-d

## Appendix: Technical Names

OAK-D Lite, OAK-D Pro, OAK 4 D, RVC2, RVC4, depthai, Luxonis, BMI270, BNO085, BNO086, LSM6DSV, AK09919, Pi 5, I2C, IMU, NMS, RDF, EEPROM, YOLOv8n, ONNX, blobconverter, NN Archive, `DetectionNetwork`, `SpatialDetectionNetwork`, `ObjectTracker`, `SpatialLocationCalculator`, `StereoDepth`, `FeatureTracker`, `HostNode`, `RecordReplay`, MediaPipe, `getChipTemperature`, `getConnectedIMU`, `getIMUFirmwareVersion`, `getUsbSpeed`, `getImuToCameraExtrinsics`, `getTimestampDevice`, `GYROSCOPE_RAW`, `GYROSCOPE_CALIBRATED`, `ACCELEROMETER_RAW`, `ACCELEROMETER_CALIBRATED`, `ROTATION_VECTOR`, `GAME_ROTATION_VECTOR`, MCAP, H.265, GPS-COG, shave, ISP, USB2, USB3, WARNING (log level), CRT (Bosch Component ReTrimming).
