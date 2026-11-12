# Stand test 2026-11-11: VESC part A and the pack guard

Status: done 2026-11-11 21:03-21:05, robot on the stand, wheels free, pack
about 53.8 V (BMS lowest cell 4.143 V). Kevin drove with the RC
transmitter. Code on the robot: `2c193c1` (part A applied at 20:26, the pack
guard deployed at 20:45).

Data on the robot: `logs/standtest_20261111/candump2.log` (`candump -L can0`,
21:03-21:33), the armed logs `logs/arm_20261111_210328.log` and
`logs/arm_20261111_210417.log`, and the journal of `wall-e.service`. The
analysis scripts were throwaway scripts; this document has their results.

## 1. Sequence

1. Arm with the switch.
2. Full forward for 3 s, then release. Three times.
3. Full reverse for 3 s, then release. One time.
4. Full forward for 3 s, then full reverse for 1 s, then release.
5. Transmitter off for 3 s with the arm switch up, then on.
6. Disarm with the switch.

## 2. Results

| Item | Left (id 2) | Right (id 1) | Limit or expectation |
|---|---|---|---|
| Highest regen (battery) current | -5.4 A | -6.4 A (one sample) | -5 A per VESC (part A) |
| Highest drive (battery) current | 11.0 A | 12.0 A | 15 A per VESC (part A) |
| Bus voltage before / highest during | 53.6 / 53.7 V | 54.7 / 54.8 V | Small rise |
| Stop after stick release, from about 15,000 eRPM | about 0.75 s | about 0.75 s | 0.5-1.0 s on 2026-11-05 |
| VESC fault after the test | none | none | none |
| MOSFET temperature after the test | 31 C | 30.5 C | |

The regen limit works. The peaks above 5 A are single samples of the current
controller. Braking on the stand is the same as before part A.

Transmitter dropout: the robot disarmed at once (journal 21:04:16,
`armed=N`) and armed again after the transmitter came back with the arm
switch up (21:04:21, `armed=Y`). The wheels stayed still. This is the
behaviour of the code: a dropout does not set the re-arm latch.

## 3. Finding: the pack guard compares readings from different moments

The pack guard compares the two VESC bus readings. During the spin-ups the
guard logged "pack voltage UNTRUSTED (disagree)" many times (for example
left 52.5 V, right 54.2 V). The old limit was 1.5 V.

Cause (independent Opus review, then checked again on the same data):

- Readings taken within 5 ms of each other differ by 0.8-1.3 V (12,105
  pairs, mean 1.02 V). That is the known offset of the right VESC
  (finding 7 of `docs/battery_settings.md`). The current-dependent part is
  about 0.01 ohm per side, which is negligible. There is no wiring drop
  worth counting.
- The guard compares the LATEST reading of each VESC. Those two can be
  0-40 ms apart (each VESC sends STATUS_5 at about 48 Hz, and frames are
  sometimes lost).
- With the wheels free at 17,000-20,000 eRPM, the VESC speed loop hunts: the
  input current changes between +10 A and -3 A from one frame to the next,
  and the bus voltage changes by about 1 V in 20 ms.
- So the compared pair reached 2.1 V. Each such gap lasted one frame.

A first analysis in this session paired each left frame with the nearest
older right frame (up to 50 ms) and fitted a "wiring drop" of 0.064 ohm left
and 0.037 ohm right. That fit was an artifact of the time skew. It is wrong;
do not use it.

Replay of the stand data through the real `VescCanDriver`, with a check after
every frame (as the receive loop does):

| Limit | "Disagree" episodes | Longest untrusted time | RESTRICTED |
|---|---|---|---|
| 1.5 V | 35 | 4.3 s | no (but a drive would reach it) |
| 2.0 V | 2 | | no |
| 2.5 V and 3.0 V | 0 | | no |
| 3.0 V, swings scaled x2.5 | 4 (one frame each) | 3.3 s | no |

Change (2026-11-11, Kevin's choice; Grok session
`01a1cdbe-e87c-7af0-9df9-0baa7ea360c0` and an independent Opus review agree):
`VescConfig.pack_voltage_max_disagree_v` is 3.0 V (was 1.5 V). The change
cannot make the latch late or release it wrongly, because the guard always
uses the lower reading, and release needs that reading at 46.0 V or more.

Cost of the change: a left reader that reads 1.5-3 V low now gives an early
latch (limp home still works) instead of RESTRICTED, and a frozen reader is
found after 3 V of pack movement instead of 1.5 V.

Not changed by it (also true at 1.5 V): if the LEFT reader sticks high, the
minimum falls to the right reading, which is about 1 V high, so the latch
can be up to 1 V late (a true pack of about 43.0 V).

## 4. Open items

1. Better fix: put both readings through the same 0.25 s moving average
   before the comparison. In the replay the gap then stayed 0.96-1.22 V
   (0.88-1.53 V with the swings scaled x2.5). Then tighten the limit to about
   2.0 V.
2. A lone right reading is corrected by a fixed 1.0 V
   (`pack_voltage_right_alone_offset_v`). It is used only when the left VESC
   is silent.
3. The drive-current cap (15 A) was not reached on free wheels. Check it on
   the ground.
4. The speed loop hunts at full speed with the wheels free (current +10 A to
   -3 A frame to frame). Check whether it also hunts on the ground.

## Appendix: Technical Names

VESC, CAN, eRPM, regen, RC, MOSFET, BMS, RESTRICTED, Grok, Opus, `candump`,
`wall-e.service`, `VescConfig`, `VescCanDriver`, MANUAL, STATUS_5, speed loop,
moving average.
