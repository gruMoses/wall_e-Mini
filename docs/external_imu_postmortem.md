# External 9DoF IMU postmortem (ISM330DHCX + MMC5983MA)

Date: 2026-09-19. Board: SparkFun 9DoF IMU Breakout, ISM330DHCX (accel + gyro, I2C 0x6B) + MMC5983MA (magnetometer, I2C 0x30), Qwiic. Code path: `pi_app/hardware/imu_reader.py` (`ImuReader`, mode `ISM_MMC`), selected first when `config.imu_source == "auto"`. The board is unplugged today; the service logs "External IMU not found" and uses the OAK BMI270. Kevin asked: "We never really made it work right. What did we get wrong?"

A 22-agent research pass read the driver, the tools, the history, the SparkFun documentation and both datasheets, and ran the heading math offline with the Qwiic drivers mocked (`/Users/kevinmoses/.claude/jobs/1cd9f1d2/tmp/ext_imu/sim_heading.py`; the harness is ready to become `pi_app/tests/test_imu_reader.py`). Findings marked CONFIRMED passed two independent verifiers. Nothing in this document was measured on the board; section 4 is the bench plan.

## 1. Short answer

1. The board never had a fair trial. Its one unique capability is the magnetometer (absolute north). `config.imu_use_magnetometer` has been `False` since 2026-02-28, so the board ran as a gyro-only integrator from zero. That is the same job the OAK BMI270 does, and the OAK path received a year of fixes and tests that the external path never got. The external path has zero unit tests. CONFIRMED.
2. The documented repair tool is dead. `pi_app/cli/calibrate_imu.py` reads `self.imu.imu` and `self.imu.mag`. `ImuReader` renamed those attributes to `.ism` and `.mmc` in the same commit that added the tool (0ff107e). Each sample raises inside a per-sample `except`, so the tool collects zero samples. `pi_app/cli/imu_calibration.json` shows `quality_score: 0.0`. When the compass jumped, the remedy in `docs/IMU_TROUBLESHOOTING.md` did nothing. CONFIRMED.
3. The live calibration is larger than the signal. `imu_calibration.json` hard-iron offsets are `[0.070, −0.281, 0.107]` G, magnitude 0.308 G. The local horizontal Earth field is 0.232 G (WMM2025, Dallas area). If the true hard iron changed at all since 2025-09-02, the compass pins into an 80-degree arc: the simulation reproduces the recorded "20–70 degree variations" from the stale calibration alone. The verifiers refuted the claim that this is the main field mechanism, because the magnetometer is off today; it is the failure mode of the 2025 tests.
4. Motor current at the mount can exceed the Earth field. A conductor at 0.30 m carrying 20 A adds 0.133 G (57 percent of the horizontal field, up to 33 degrees of error); 40 A exceeds the field and the heading pins or jumps by up to 180 degrees. The fusion has no validity gate: `alpha_yaw = 0.95` is a 5-percent-per-sample pull, a 0.33 s time constant at 60 Hz, so the magnetometer dominates the heading within a second. CONFIRMED (gate absence); the current figures are arithmetic, not measurements.
5. The MMC5983MA SET/RESET degauss runs once, inside the driver constructor, and never again. Residual magnetization from a motor pulse then stays in the bridge offset. The driver's last operation is RESET, which inverts the polarity of all three axes: a rigid 180-degree heading error until proven otherwise. CONFIRMED.
6. The magnetometer axis map `('x','y','-z')` is unverified. SparkFun documents only that the Z axes are opposite. Two right-handed frames cannot differ by one axis flip, so either the MMC frame is left-handed (map correct) or X or Y is also flipped (mirrored compass, which turns heading-hold into positive feedback). A 60-second bench turn decides it. CONFIRMED.
7. Second-order defects: the tilt-compensation formula is malformed (up to 4.7 degrees at 15/15 degrees pitch/roll); no declination term (2.5 degrees E here); one magnetometer timeout raises `TypeError` out of `read()` and ten of them latch the IMU unavailable; each magnetometer read blocks 8–12 ms on the shared I2C bus inside the 60 Hz control path; no presence check on the ISM330DHCX and no 0x6A probe; two calibration files with two schemas; no accel/gyro axis map to the robot frame; the physical mounting orientation is documented nowhere.

The PID derivative-sign finding from this pass applies to the external reader only: it returns `gz_dps` in the raw (counter-clockwise) sense while its heading is clockwise-positive, so the heading-hold D-term is anti-damping on that path. The OAK path is corrected by the 2026-09-19 heading fix. Do not change the external reader's sign without the board on the bench.

## 2. Verdict

Salvageable in one role only: an absolute-north corrector for the OAK gyro heading, using only the MMC5983MA. Do not use the ISM330DHCX as the gyro or as a backup; the OAK path is field-validated and tested, the external gyro path is not. The magnetometer does for heading what RTK does for position: it removes unbounded drift. It complements the GPS-COG aligner (which needs RTK fix 4, a straight run, and locks once per session); a compass works while stationary and in turns.

The mount decides everything. Target 0.4 m or more from any conductor carrying more than 20 A (VESC phase leads, pack leads, CAN run), on non-magnetic standoffs, with the Qwiic cable routed away from the power leads. Acceptance: bench step 9 shows Δ|B| < 0.02 G and Δheading < 5 degrees between motors-off and motors-at-load. If the mount fails that test, no code fixes it.

## 3. Code plan (after the bench, in order)

1. `pi_app/hardware/mag_heading.py`: MMC5983MA-only reader on its own thread at 10 Hz; the control loop reads a cached sample.
2. Per-sample SET/RESET: H = (M_set − M_reset) / 2. This removes the polarity ambiguity, the thermal bridge offset and the residual magnetization in one step.
3. Validity gates before fusion: |B| within 0.477 G ± 8 percent; dip angle within 61 ± 5 degrees using the OAK accelerometer for the horizontal; per-sample delta limit; hard mute while motor command bytes or VESC current exceed a threshold. Mute on failure; never blend a suspect sample.
4. Replace the 0.95 per-sample pull with a slew-limited offset observer on the OAK heading (0.5–2 degrees per second).
5. Fix `_tilt_compensated_heading` to the canonical inverse rotation and add `pi_app/tests/test_imu_reader.py` from the harness.
6. Add `config.imu_mag_declination_deg` (+2.54 for Dallas) so magnetic and GPS headings share one frame.
7. Re-fit hard iron in situ with an ellipsoid fit, motors off; store |B|, dip angle, date and temperature with it; refuse a calibration older than N days without an override.
8. Fix or delete `pi_app/cli/calibrate_imu.py`; one calibration file, one schema, one loader; log the loaded values at WARNING.
9. Make the boot probe loud: print the exception, the addresses tried (0x6B and 0x6A), the chip that initialized; put `imu_source_used` in telemetry.
10. Treat a falsy magnetometer return as a dropped sample; log once when availability flips.

## 4. Bench protocol (read-only, when the board is plugged in)

WARNING: Steps 4–12 need `wall-e.service` stopped (the OAK allows one process, and the IMU shares I2C bus 1 with the RTK GPS and the UPS daemon). Stopping the service needs Kevin's explicit go in the moment. Wheels off the ground for any step that commands a motor. No config edits, no calibration writes.

1. Record the environment: `pip3 show sparkfun-qwiic-mmc5983ma sparkfun-qwiic-ism330dhcx qwiic-i2c`; `grep -n i2c /boot/firmware/config.txt`; `python3 -m pi_app.cli.calibrate_imu --validate-only` (expected: nothing useful, which confirms finding 2 on the Pi).
2. `i2cdetect -y 1` with the board unplugged. Write down the baseline (expected: GPS, UPS MCU, INA219).
3. Plug in the Qwiic cable, scan again. Expected new addresses: 0x30 and 0x6B. 0x30 only → check 0x6A (address jumper). 0x6B only → magnetometer dead or unpowered. Neither → cable, connector or 3V3.
4. Static raw dump, board level, motors off, at least 1 m from the VESCs and the pack, 5 s. Expected: |a| = 1000 ± 20 mg with az ≈ +1000 (Z up); gyro within ±1 °/s of the stored bias; |B| = 0.477 G, accept 0.453–0.501. |B| in 0.20–0.33 G means the MMC5983MA is out of spec (possible soldering damage; the stored soft-iron matrix already implies 0.20/0.22/0.33 G semi-axes).
5. Z-axis check: with Z up, the raw MMC z must read about −0.417 G (field points down at 61 degrees). +0.417 G means the mag Z really is inverted and the map's `-z` is right on that axis.
6. Mirror check (decisive): level, rotate slowly clockwise through 0/90/180/270/360 degrees, log raw (mx, my) and `heading_deg` with `use_magnetometer=True` and the calibration bypassed (offsets 0, scales 1). Expected: (mx, my) traces a 0.232 G circle centred within 0.01 G of the origin; heading increases monotonically. Heading DEcreasing → a horizontal axis is flipped in the map. Constant ~180 degrees off → RESET polarity (step 7) or an X flip.
7. SET/RESET polarity: read; `perform_set_operation()`; read. All three components flip if the driver left the part in RESET. The correct polarity gives |B| ≈ 0.477 G with mz opposite to gravity. Do the datasheet procedure: SET, M_set; RESET, M_reset; (M_set − M_reset)/2 must have |·| ≈ 0.477 G; (M_set + M_reset)/2 is the bridge offset.
8. Hard iron in situ: board on its standoffs, motors off, rotate the whole robot 360 degrees on flat ground, log (mx, my). Acceptance for a usable mount: circle centre |c| < 0.05 G. The stored calibration implies 0.308 G; a value near that means the mount is the problem.
9. Motors off versus on, same heading, wheels off the ground: idle, then each side at about 25 percent and 60 percent byte offset; cross-reference `vesc_left_rpm`/`vesc_right_rpm` and VESC current. Acceptance: Δ|B| < 0.02 G and Δheading < 5 degrees at the highest current you use. Repeat at two or three candidate mounts; this chooses the mount.
10. Dip-angle gate: from the same logs compute the angle between the field vector and the accelerometer horizontal. Expected 61 ± 2 degrees motors off; confirm it leaves the band under load. This is the best runtime validity gate available.
11. Loop cost: time 200 `ImuReader.read()` calls with the magnetometer on and 200 off. Expected 8–12 ms versus < 1 ms; this is the evidence for the separate thread.
12. Bus health: with the service stopped and the UPS daemon and GPS running, loop raw reads for 60 s, count exceptions and timeouts, then `i2cdetect -y 1` again. Only if errors appear, consider `dtparam=i2c_arm_baudrate=100000`.
13. Write the numbers and the physical mounting orientation and distances into this document; they are the regression baseline.

## 5. Sources

- SparkFun 9DoF IMU Breakout hookup guide and product page (ISM330DHCX + MMC5983MA, Qwiic)
- MMC5983MA datasheet (SET/RESET, polarity, bridge offset, 16384 counts per gauss)
- ISM330DHCX datasheet (zero-rate level ±1 °/s, temperature drift ±0.005 °/s per °C; no orientation fusion)
- `sparkfun-qwiic-mmc5983ma` and `sparkfun-qwiic-ism330dhcx` Python drivers v2.0.0 (constructor SET/RESET, `is_connected()` returning `None`, `set_shadow_bit` local-variable defect, `get_measurement_xyz()` returning `False` on timeout)
- BGS WMM2025 field values for 32.78 N, 96.80 W on 2026-09-19: total 0.477 G, horizontal 0.232 G, inclination 60.96 degrees, declination +2.54 degrees E
- Repository history: 2221501 (2025-08), 0ff107e (2025-09-02), 8528c77 (2025-09-04), `imu_use_magnetometer=False` since 2026-02-28; `docs/IMU_TROUBLESHOOTING.md`; `docs/imu_tuning_session_20260308.md`

## Appendix: Technical Names

ISM330DHCX, MMC5983MA, SparkFun, Qwiic, I2C, `ImuReader`, `imu_reader.py`, `calibrate_imu.py`, `imu_calibration.json`, `imu_use_magnetometer`, `imu_mag_axis_map`, `alpha_yaw`, SET/RESET, hard iron, soft iron, dip angle, declination, WMM2025, BGS, OAK, BMI270, VESC, CAN, UPS, INA219, RTK, GPS-COG, `i2cdetect`, `dtparam`, `wall-e.service`, `perform_set_operation`, `get_measurement_xyz_gauss`, `is_connected`, `set_shadow_bit`, TypeError, WARNING (log level), Pi.
