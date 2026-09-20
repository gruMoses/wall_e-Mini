# Hand-gesture review (2026-09-20)

This document records a code review of the hand-gesture path. The
review verdict is: the chain is wired, but gestures do not work as
deployed.

## 1. Findings

1. FIVE (open palm) stop is only checked in `GestureStateMachine` phase
   `ACTIVE`. RC-started FOLLOW_ME never enters that phase. An open palm
   is ignored on a normal RC run, even at 0.5 m.
2. A 3-4-3 start is cancelled one tick later. The start sets follow-me
   on. The next control tick, RC channel 4 low forces
   `FOLLOW_ME_EXITED`.
3. MediaPipe Hands on the 640×480 stream sees a hand that is too small
   beyond about 0.5 m (section 2).
4. `hold_frames` (config 12) counts 30 Hz control ticks that re-read a
   cached result, not new camera frames.
5. No gesture logging existed at the start of this review. A per-tick
   `gesture` block now exists (section 5).

Better detection alone does not fix items 1, 2, or 4.

## 2. Hand size versus distance

MediaPipe Hands runs on a 256 px canvas of the 640×480 preview. It
needs about 40-60 px of hand. A standing operator at
`follow_distance_m` 1.5 m is already below that.

| Range | Hand on MediaPipe 256 px canvas | Versus 40-60 px need |
|---|---|---|
| 1.5 m | about 22 px | too small |
| 3.0 m | about 11 px | too small |

Nothing works beyond about 0.5 m on the current full-frame crop.

On-device palm/landmark blobs exist in `pi_app/models/hand/`
(`palm_detection_sh4.blob`, `hand_landmark_lite_sh4.blob`). The live
path does not use them. Host-side MediaPipe Hands uses the 640×480
preview.

## 3. Owner decision

Fix the gestures. Do not remove them. Do not disable them by default.

Until the fix is complete, the RC switch stays the authority and the
real stop. Operator: "that's too scary for our current level of
control on this guy."

End state: hand gestures and the phone are the primary control,
including when the robot is not armed. That end state is later. It
needs its own safety review. Treat "gesture can start motion with
nobody on the RC" as a motion-safety change.

## 4. Agreed fix order

1. Mode ownership between RC channel 4 and gestures. RC-entered
   follow-me must put the state machine in `ACTIVE`, so FIVE is
   checked. A gesture start must survive channel 4 low, or be refused
   with a logged reason.
2. Count holds on new hand results or on wall-clock, not on 30 Hz
   control ticks that re-read a cache.
3. Range. Crop the hand from the person bbox at higher resolution, or
   use the on-device palm/landmark blobs in `pi_app/models/hand/`.
4. Only then unarmed operation, with its own safety review.

## 5. Logging that now exists

Per-tick line, block `gesture` (`pi_app/app/log_gating.py`
`build_log_obj`):

`hand_detected`, `hand_span_px`, `finger_count`, `label`, `streak`,
`phase`, `seq_idx`, `event`, `event_reason`, `hand_poll_enabled`.

1 Hz slow line, block `gesture` (`build_slow_obj`):

`hand_poll_enabled`, `hand_poll_ms`, `mp_loaded`, `hand_detect_rate`.

`GestureConfig.hand_poll_in_follow_me` default False: MediaPipe Hands
does not run in RC-started FOLLOW_ME (the machine is in `IDLE`, so
Hands cannot act). Set True to restore the old always-when-armed poll.

## Appendix: Technical Names

FOLLOW_ME, RC, MediaPipe Hands, FIVE, `GestureConfig`,
`hold_frames`, `hand_poll_in_follow_me`, `gesture`,
`pi_app/models/hand/`.
