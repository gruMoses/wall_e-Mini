# RTK float investigation (2026-09-19)

The rover reported RTK float (fix quality 5) and never RTK fixed (4) until 15:27 on this date; the seven JSON logs on the Pi before that (oldest 2026-07-30) contain no epoch with quality 4 (section 2.1). Kevin has seen a fix in the past. This document gives the hardware identity, the evidence, the ranked causes, the decisive test, and the base-station checklist. A 13-agent research pass produced the causes; a Grok cross-check corrected one of them.

## 1. Hardware

- Kit: DFRobot Gravity GNSS-RTK High Precision Positioning Kit with LoRa, SKU KIT0198 (US 915 MHz variant for Texas). Rover: Quectel LC29HDA behind a DFRobot I2C co-processor at address 0x20 (`pi_app/hardware/rtk_gps.py`). Base: Quectel LC29HBS. Radio: Semtech LLCC68, 22 dBm.
- Bands: GPS/QZSS L1 C/A + L5; GLONASS L1 only; Galileo E1 + E5a; BDS B1I + B2a. Spec: 5 cm + 1 ppm; convergence < 10 s; range up to 1.5 km in the open.
- Base: on the barn roof, about 100 yards (90 m) from the yard. Kevin plugged it in without configuration. Power-cycle history unknown. The base has only a power LED; its state is invisible from the robot.
- The rover LED shows the fix state: 1 Hz blink = state 5, steady = state 4, off = other.

## 2. Evidence (robot parked, disarmed)

| Item | Value | Meaning |
|---|---|---|
| Fix quality | 5 continuously since 12:01; 1 (no corrections) from 11:12 to 12:01 | Corrections started when the robot moved at 12:01 |
| Satellites used | 28–36; HDOP 0.5–0.87 | The code solution is healthy; this says nothing about carrier-phase quality or L5 |
| Differential age | 1.0 s on 119 of 120 epochs, 2.0 s on 1 | Some correction message arrives each second |
| Station id | 3335 | RTCM 1005/1006 from the base decodes |
| Raw `$GNGGA` (register 81/82) | `...,5,30,0.71,38.075,M,-23.838,M,1.0,3335*57` | The receiver itself reports quality 5; the driver decode is correct |
| Raw `$GNRMC` | `...,0.067,076.52,190926,,,F,V*24` | Course 076.52°, speed 0.067 kn, mode F = RTK float; course and speed are available every epoch |
| Register 93 (operation) | 10 | LoRa mode confirmed |
| Register 94 (LoRa serial baud index) | 4 | 19,200 baud between the LoRa radio and the receiver; the air rate is not visible from the rover |
| Register 113 (connect state) | 0 | Meaning undocumented in LoRa mode |
| OAK-D Lite USB link | 480 Mb/s (USB 2) | No USB 3 harmonic near L1/L5 |

### 2.1 First fix, 15:27 (after this document was started)

Kevin drove the robot about 23 m west of its usual parking spot and stopped at 15:07. At 15:27:34, after 20 minutes stationary, the rover reported quality 4 for the first time in any log. It fell back to 5 at 15:27:49, fixed again at 15:34:30, fell to 5 at 15:47:08, and fixed at 15:47:18; it then held 4 with `$GNRMC` mode R, course 324.1, position scatter about 1 cm. Conclusions: the base, the link and the rover can produce a fixed solution; the convergence time (20 minutes against a specification of under 10 seconds) and the drop-outs say the corrections or the carrier-phase quality are marginal, which points at causes 1, 2 and 4 in section 3; the old parking spot never fixed in hours, so it is worse (obstruction, multipath, or LoRa margin) than the new one. The short-baseline test in section 4 and the base checklist in section 5 stay the next steps; the GPS heading aligner can now lock during a straight manual run for the first time.

## 3. Ranked causes

1. HIGH. Incomplete correction epochs on the air. RTCM 1005 is about 20 bytes and keeps the differential age at 1 s by itself, while the MSM observation blocks (about 900 B/s for four constellations at MSM4) are truncated or dropped by a LoRa air rate set for range, not throughput (the LLCC68 goes down to 1.76 kb/s). Confirm with the short-baseline test (section 4) and by decoding the RTCM stream at the base output. Fix: raise the air rate (Emlid recommends about 9 kb/s for RTK), and cut the payload to GPS + Galileo, L1 + L5/E5a, MSM4 at 0.5–1 Hz, station coordinates at 0.1 Hz.
2. HIGH. L1-only intersection. GLONASS has no L5. If the base sends only L1/E1/B1I observations, or an antenna at either end is an L1-only patch, the rover resolves ambiguities on one frequency, which commonly ends in permanent float. 30+ satellites used is what four-constellation L1 tracking looks like. Confirm with per-satellite signal ids (GSV) from the full NMEA buffer or a UART tap; fix with L1/L5 antennas at both ends and an L5/E5a MSM set.
3. MEDIUM. Base coordinate not frozen. The LC29HBS default survey-in is 43,200 epochs (12 hours) and the base transmits while it surveys. CORRECTION from the Grok cross-check: a running mean moves the coordinate by about 0.3 mm per epoch after two hours, which cannot block a fix by itself; a frozen but wrong coordinate still gives quality 4. The only way this blocks a fix is an engine that restarts on each coordinate change. The fix is the same and takes five minutes at the base UART: freeze the coordinate (`$PQTMCFGSVIN` with a fixed position, or a 300-epoch survey-in that then locks). Do not wait 12 hours.
4. MEDIUM. Rover carrier-phase quality: GNSS antenna without a ground plane, multipath from the robot, broadband noise from the Pi, the VESCs and the BMS radio in the same enclosure. Satellite count and HDOP cannot show this. Confirm with the short-baseline test with the compute powered down.
5. MEDIUM. Base antenna sky view and multipath (metal roof, eaves). Confirm with the base's own NMEA on its UART and a sky photo.
6. LOW. Message set gaps (no 1230 with GLONASS MSM; infrequent 1005), MSM7 versus MSM4, receiver mode, or firmware. Confirm by decoding the RTCM stream.
7. RULED OUT. Driver decode (the raw GGA agrees with register 19). Packet loss at the parking spot (119/120 epochs on time). USB 3 interference (the OAK runs at USB 2).

The gates stay as they are: `WaypointNavConfig.min_rtk_quality = 4` and the aligner's `== 4` check are correct. Float is decimetre-to-metre accuracy and must not establish heading lock. Consequence: the GPS heading aligner has never locked, so the robot has run on gyro-only heading with no north reference since at least July.

## 4. Decisive test: short baseline

WARNING: This test moves the robot. Kevin does it, and says when he starts.

1. Look at the rover LED. A 1 Hz blink confirms state 5 from the module itself.
2. Carry or drive the robot to within 5–10 m of the base antenna, with clear sky at both ends. If possible, power down the Pi, the OAK and the VESCs, or lift the GNSS antenna onto a metal disc away from the robot.
3. Wait 15 minutes. Watch the LED (or the dashboard `gps_fix`).
4. Result A: fixed near the base, float at the parking spot → baseline, link margin or robot noise. Raise the LoRa air rate; check the LoRa antennas; move the GNSS antenna onto a ground plane.
5. Result B: still float near the base → base configuration, base antenna, or the message set. Go to section 5.

## 5. Base-station checklist

1. Power LED on? When was it last power-cycled? Has it ever run for hours without interruption?
2. Photograph the sky from the antenna position. Note metal within 2 m, the roof, the eaves, and whether the antenna sits on a ground plane.
3. Confirm both LoRa antennas are screwed on, vertical, and clear of metal. Confirm both radios are the 915 MHz US variant.
4. Get a USB-TTL adapter on the base UART (LC29HBS). Read `$PQTMVERNO` (firmware) and `$PQTMCFGSVIN,R` (survey-in mode and stored position). Log the transmitted 1005/1006 coordinate for several minutes: frozen or moving?
5. Freeze the coordinate: `$PQTMCFGSVIN,W,1,300,15,0,0,0` (300-epoch survey-in, then lock) or a fixed position. Save and power-cycle to confirm it persists.
6. Decode the RTCM stream (RTKLIB `str2str`, `pyrtcm`): list every message number and rate. Required: 1005 or 1006; MSM for each constellation in use; 1230 if GLONASS MSM is sent.
7. Size the stream against the radio: measure bytes per second; read the spreading factor, bandwidth and air rate at both ends; they must match, and the air rate must exceed the payload with margin.
8. Make it permanent: bolt the antenna, record the coordinate, give the base an always-on supply, and stop power-cycling it.
9. Rover side: put the GNSS antenna on a 100 mm metal ground plane, as far from the USB run, the VESCs and the Pi as the harness allows.

## 6. Software changes (in progress, branch fable-follow-fixes)

- Log every fix-quality transition at WARNING with UTC, satellites, HDOP, differential age and station id.
- Parse UTC and geoid separation from the bytes already read; read course and speed over ground from the RMC sentence (registers 83/84) and log them; do not feed them into control yet.
- Publish low-quality epochs instead of discarding them, so a fix loss is visible.
- Poll at 5 Hz and dedupe on the UTC second (the 1 Hz poll aliases against the 1 Hz output).
- Verify the LoRa-mode register instead of writing it blind; log at WARNING.
- Health counters and bus reopen on repeated I2C errors.

## 7. Sources

- DFRobot wiki KIT0198: https://wiki.dfrobot.com/kit0198-eu/ and https://wiki.dfrobot.com/GNSS_RTK_High_Precision_Positioning_Kit_SKU_KIT0198_EN
- DFRobot product page (US 915 MHz): https://www.dfrobot.com/product-2970.html
- Vendor library: https://github.com/DFRobot/DFRobot_RTK_LoRa
- Quectel forum, LC29HBS float to fixed: https://forums.quectel.com/t/float-rtk-mode-to-fixed-rtk-using-lc29hbs-and-lc29hda/37451
- RTCM MSM bandwidth reference: https://www.kalmixtech.com/blogs/blog/an002-rtcm-3-frame-structure-msm-reference

## Appendix: Technical Names

DFRobot, KIT0198, Gravity, Quectel, LC29HDA, LC29HBS, Semtech, LLCC68, LoRa, RTCM, MSM4, MSM7, RTK, GNSS, GPS, GLONASS, Galileo, BDS, QZSS, L1, L5, E1, E5a, B1I, B2a, HDOP, NMEA, `$GNGGA`, `$GNRMC`, `$GNVTG`, GSV, `$PQTMCFGSVIN`, `$PQTMVERNO`, RTKLIB, `str2str`, `pyrtcm`, Emlid, I2C, UART, USB-TTL, USB 2, USB 3, OAK-D Lite, VESC, BMS, Pi, `rtk_gps.py`, `min_rtk_quality`, survey-in, ground plane, Kevin, Grok.
