"""Log-only pump-gesture detector. Synthetic 15 Hz sequences.

Intrinsics are the NN frame this robot's EEPROM reports at 640 px wide
(fx=456.89, cx=334.95). Metric width is recovered from the normalised
box with that same z, so a walk-in does not look like a pump.
"""

import unittest

from config import PumpGestureConfig, config as default_config
from pi_app.control.pump_gesture import PumpDetector, PumpObservation


FX = 456.89
CX = 334.95
FRAME_W = 640
# fy / cy are unused by the width math; they complete the (fx, fy, cx, cy) tuple.
INTR = (FX, 456.89, CX, 176.0)
DT = 1.0 / 15.0
REST_W = 0.52
PUMP_W = 1.6
Z = 3.0

KEYS = (
    "state", "accepted", "reject_reason",
    "w_m", "c_m", "rest_w", "ratio",
    "out", "out_run", "peaks",
    "start_event", "stop_event", "stop_reason",
    "active_s", "track_id", "would_start_count",
)


def bbox_from_edges(l_m, r_m, z_m):
    xmin = (l_m * FX / z_m + CX) / FRAME_W
    xmax = (r_m * FX / z_m + CX) / FRAME_W
    return (xmin, 0.2, xmax, 0.8)


def bbox_for(w_m, c_m, z_m):
    return bbox_from_edges(c_m - w_m / 2.0, c_m + w_m / 2.0, z_m)


def obs(ts, w_m, z_m=Z, c_m=0.0, depth_ok=True, track_id=1, bbox=None):
    if bbox is None:
        bbox = bbox_for(w_m, c_m, z_m)
    return PumpObservation(
        ts=ts, bbox=bbox, z_m=z_m, depth_ok=depth_ok, track_id=track_id,
    )


class TestPumpGesture(unittest.TestCase):
    def setUp(self):
        self.det = PumpDetector(PumpGestureConfig())

    def step(self, t, observation, intrinsics=INTR):
        result = self.det.update(observation, intrinsics, FRAME_W, t)
        self.assertEqual(set(result), set(KEYS))
        return result

    def _rest(self, t0=0.0, n=45, w=REST_W, z=Z, track=1):
        last = None
        for i in range(n):
            t = t0 + i * DT
            last = self.step(t, obs(t, w, z_m=z, track_id=track))
        return last, t0 + (n - 1) * DT

    def _pump(self, t0, track=1, w_wide=PUMP_W, w_rest=REST_W):
        """4 wide frames then 3 rest frames. Returns (events, next_t)."""
        events = []
        t = t0
        for _ in range(4):
            events.append((t, self.step(t, obs(t, w_wide, track_id=track))))
            t += DT
        for _ in range(3):
            events.append((t, self.step(t, obs(t, w_rest, track_id=track))))
            t += DT
        return events, t

    def _two_peaks(self):
        """3 s of rest, then two pumps. Start lands on the 2nd peak."""
        self._rest()
        t = 45 * DT
        first, t = self._pump(t)
        second, t = self._pump(t)
        return first, second, t

    def test_config_defaults_and_field(self):
        cfg = PumpGestureConfig()
        self.assertIs(cfg.enabled, True)
        self.assertEqual(cfg.min_range_m, 1.0)
        self.assertEqual(cfg.max_range_m, 5.0)
        self.assertEqual(cfg.edge_margin, 0.01)
        self.assertEqual(cfg.min_body_w_m, 0.3)
        self.assertEqual(cfg.max_body_w_m, 2.2)
        self.assertEqual(cfg.rest_window_s, 3.0)
        self.assertEqual(cfg.rest_min_samples, 8)
        self.assertEqual(cfg.rest_min_span_s, 1.5)
        self.assertEqual(cfg.freeze_ratio, 1.4)
        self.assertEqual(cfg.out_ratio, 1.5)
        self.assertEqual(cfg.edge_out_frac, 0.25)
        self.assertEqual(cfg.max_centre_shift_m, 0.15)
        self.assertEqual(cfg.min_out_frames, 3)
        self.assertEqual(cfg.peak_end_ratio, 1.4)
        self.assertEqual(cfg.start_peaks, 2)
        self.assertEqual(cfg.start_window_s, 3.0)
        self.assertEqual(cfg.continue_window_s, 0.7)
        self.assertEqual(cfg.no_detection_stop_s, 0.3)
        self.assertEqual(cfg.max_active_s, 8.0)
        self.assertEqual(cfg.cooldown_s, 2.0)
        self.assertEqual(cfg.rearm_rest_ratio, 1.3)
        self.assertEqual(cfg.rearm_rest_s, 1.0)
        self.assertIs(cfg.log_nearest_outside_follow_me, True)
        self.assertEqual(default_config.pump, cfg)

    def test_three_pumps_start_on_second_and_hands_down(self):
        # The normalised box for a centred 0.52 m body at 3 m must not
        # touch the frame, or the whole sequence is rejected as an edge.
        box = bbox_for(REST_W, 0.0, Z)
        self.assertGreater(box[0], 0.01)
        self.assertLess(box[2], 0.99)

        last, _ = self._rest()
        self.assertAlmostEqual(last["rest_w"], REST_W, places=4)
        self.assertFalse(last["out"])
        self.assertEqual(last["would_start_count"], 0)
        self.assertEqual(last["state"], "idle")

        t0 = 45 * DT
        first, t = self._pump(t0)
        # Peak completes on the first frame back at rest width.
        peak1_t, peak1 = first[4]
        # _pump accumulates t += DT, so compare within float rounding.
        self.assertAlmostEqual(peak1_t, t0 + 4 * DT, places=9)
        self.assertEqual(peak1["peaks"], 1)
        self.assertFalse(peak1["start_event"])
        self.assertEqual(peak1["state"], "idle")
        self.assertAlmostEqual(peak1["rest_w"], REST_W, places=4)

        second, t = self._pump(t)
        peak2_t, peak2 = second[4]
        self.assertTrue(peak2["start_event"])
        self.assertEqual(peak2["state"], "active")
        self.assertEqual(peak2["peaks"], 2)
        self.assertEqual(peak2["would_start_count"], 1)
        self.assertFalse(peak2["out"])
        self.assertAlmostEqual(peak2["rest_w"], REST_W, places=4)

        third, t = self._pump(t)
        for _, frame in third:
            self.assertEqual(frame["state"], "active")
            self.assertFalse(frame["start_event"])
            self.assertFalse(frame["stop_event"])
        last_out_t = third[3][0]
        self.assertTrue(third[3][1]["out"])

        # 3 scripted rest frames are only 0.2 s. Keep hands down until 0.7 s.
        prev = third[-1][1]
        stopped = None
        t_cursor = third[-1][0]
        for _ in range(20):
            t_cursor += DT
            frame = self.step(t_cursor, obs(t_cursor, REST_W))
            if frame["stop_event"]:
                stopped = frame
                break
            prev = frame
        self.assertIsNotNone(stopped)
        self.assertEqual(stopped["stop_reason"], "hands_down")
        self.assertEqual(stopped["state"], "cooldown")
        self.assertEqual(prev["state"], "active")
        self.assertFalse(prev["stop_event"])
        gap = t_cursor - last_out_t
        self.assertGreaterEqual(gap, 0.7 - 1e-6)
        self.assertLess(gap, 0.7 + DT + 1e-6)
        self.assertEqual(stopped["would_start_count"], 1)

    def test_one_frame_flicker_twice_does_not_start(self):
        self._rest()
        t = 45 * DT
        flickers = []
        for _ in range(2):
            frame = self.step(t, obs(t, REST_W * 3.0))
            flickers.append(frame)
            self.assertTrue(frame["out"])
            self.assertEqual(frame["out_run"], 1)
            self.assertEqual(frame["peaks"], 0)
            t += DT
            # Back to rest before a 3-frame run can form.
            for _k in range(8):
                frame = self.step(t, obs(t, REST_W))
                t += DT
            self.assertEqual(frame["out_run"], 0)
            self.assertEqual(frame["peaks"], 0)
        self.assertEqual(len(flickers), 2)
        self.assertEqual(frame["state"], "idle")
        self.assertFalse(frame["start_event"])
        self.assertEqual(frame["would_start_count"], 0)
        self.assertLess(t - 45 * DT, 3.0)

    def test_walk_in_constant_metric_width_is_not_a_pump(self):
        far = bbox_for(REST_W, 0.0, 4.0)
        near = bbox_for(REST_W, 0.0, 2.0)
        self.assertGreater(
            (near[2] - near[0]),
            (far[2] - far[0]) * 1.8,
        )
        n = 45
        saw_out = False
        last = None
        for i in range(n):
            t = i * DT
            # 4 m -> 2 m across the same 3 s the rest window covers.
            z = 4.0 + (2.0 - 4.0) * (i / (n - 1))
            last = self.step(t, obs(t, REST_W, z_m=z))
            if last["out"]:
                saw_out = True
        self.assertFalse(saw_out)
        self.assertEqual(last["state"], "idle")
        self.assertEqual(last["would_start_count"], 0)
        self.assertEqual(last["peaks"], 0)
        self.assertAlmostEqual(last["rest_w"], REST_W, places=3)
        self.assertAlmostEqual(last["ratio"], 1.0, places=3)

    def test_one_sided_wave_is_not_out(self):
        self._rest()
        # Right edge only. Width clears 1.5x, the left edge and the centre do not.
        l_m = -REST_W / 2.0
        r_m = REST_W / 2.0 + 0.80
        box = bbox_from_edges(l_m, r_m, Z)
        self.assertGreater(box[0], 0.01)
        self.assertLess(box[2], 0.99)
        t = 45 * DT
        for _ in range(6):
            frame = self.step(t, obs(t, REST_W, bbox=box))
            self.assertTrue(frame["accepted"])
            self.assertFalse(frame["out"])
            self.assertEqual(frame["out_run"], 0)
            self.assertGreater(frame["w_m"], REST_W * 1.5)
            t += DT
        self.assertEqual(frame["state"], "idle")
        self.assertEqual(frame["peaks"], 0)
        self.assertEqual(frame["would_start_count"], 0)

    def test_reject_edge_depth_range_and_intrinsics(self):
        t = 1.0
        good = bbox_for(REST_W, 0.0, Z)
        edge = self.step(t, obs(t, REST_W, bbox=(0.0, 0.2, 0.4, 0.8)))
        self.assertFalse(edge["accepted"])
        self.assertEqual(edge["reject_reason"], "edge")

        depth = self.step(
            t + DT, obs(t + DT, REST_W, bbox=good, depth_ok=False),
        )
        self.assertFalse(depth["accepted"])
        self.assertEqual(depth["reject_reason"], "depth")

        near = bbox_for(REST_W, 0.0, 0.8)
        self.assertGreater(near[0], 0.01)
        self.assertLess(near[2], 0.99)
        ranged = self.step(t + 2 * DT, obs(t + 2 * DT, REST_W, z_m=0.8, bbox=near))
        self.assertFalse(ranged["accepted"])
        self.assertEqual(ranged["reject_reason"], "range")

        missing = self.step(t + 3 * DT, obs(t + 3 * DT, REST_W), intrinsics=None)
        self.assertFalse(missing["accepted"])
        self.assertEqual(missing["reject_reason"], "no_intrinsics")

    def test_held_wide_arms_hit_time_limit_not_hands_down(self):
        _first, second, _t = self._two_peaks()
        start = second[4][1]
        start_t = second[4][0]
        self.assertTrue(start["start_event"])
        prev = start
        stopped = None
        stop_t = None
        for n in range(1, 200):
            t = start_t + n * DT
            frame = self.step(t, obs(t, PUMP_W))
            if frame["stop_event"]:
                stopped = frame
                stop_t = t
                break
            self.assertEqual(frame["state"], "active", n)
            self.assertTrue(frame["out"])
            prev = frame
        self.assertIsNotNone(stopped)
        self.assertEqual(stopped["stop_reason"], "time_limit")
        self.assertEqual(stopped["state"], "cooldown")
        self.assertEqual(prev["state"], "active")
        self.assertGreaterEqual(stopped["active_s"], 8.0 - 1e-4)
        self.assertLess(stopped["active_s"], 8.0 + DT + 1e-3)
        self.assertAlmostEqual(stopped["rest_w"], REST_W, places=4)
        self.assertGreater(stop_t, start_t)

    def test_pump_during_cooldown_does_not_start_until_rest(self):
        # Stop, then one pump (would be a peak in IDLE), then hands down.
        _first, second, t_next = self._two_peaks()
        start_t = second[4][0]
        # Finish the second pump's remaining rest frames (t_next is past them),
        # then one more pump while ACTIVE, then wait out hands_down.
        third, t_next = self._pump(t_next)
        self.assertTrue(all(f["state"] == "active" for _, f in third))
        last_out_t = third[3][0]
        t = third[-1][0]
        stopped = None
        for _ in range(20):
            t += DT
            frame = self.step(t, obs(t, REST_W))
            if frame["stop_event"]:
                stopped = frame
                break
        self.assertEqual(stopped["stop_reason"], "hands_down")
        stop_t = t
        self.assertGreaterEqual(stop_t - last_out_t, 0.7 - 1e-6)

        # A pump immediately after the stop must not start.
        pumped, t_after = self._pump(stop_t + DT)
        for _, frame in pumped:
            self.assertFalse(frame["start_event"])
            self.assertEqual(frame["state"], "cooldown")
            self.assertEqual(frame["would_start_count"], 1)

        # The pump's wide frames clear rearm. Its first rest frame starts it.
        # Rearm (1.0 s) finishes before the 2.0 s cooldown. _pump already
        # consumed that rest frame; continue from the next tick.
        t_down0 = stop_t + DT + 4 * DT
        saw_rearm_only = False
        left_t = None
        t = t_after
        guard = 0
        while guard < 80:
            guard += 1
            frame = self.step(t, obs(t, REST_W))
            rearm_met = (t - t_down0) >= 1.0 - 1e-6
            cooldown_met = (t - stop_t) >= 2.0 - 1e-6
            if frame["state"] == "idle":
                left_t = t
                self.assertTrue(cooldown_met)
                self.assertTrue(rearm_met)
                self.assertEqual(frame["peaks"], 0)
                self.assertFalse(frame["start_event"])
                break
            self.assertEqual(frame["state"], "cooldown")
            self.assertFalse(frame["start_event"])
            if rearm_met and not cooldown_met:
                saw_rearm_only = True
            t += DT
        self.assertTrue(saw_rearm_only)
        self.assertIsNotNone(left_t)

        # A fresh two-pump gesture can start again. Rest baseline is kept.
        again, _ = self._pump(left_t + DT)
        self.assertFalse(again[4][1]["start_event"])
        self.assertEqual(again[4][1]["peaks"], 1)
        again2, _ = self._pump(left_t + DT + 7 * DT)
        self.assertTrue(again2[4][1]["start_event"])
        self.assertEqual(again2[4][1]["state"], "active")
        self.assertEqual(again2[4][1]["would_start_count"], 2)
        self.assertAlmostEqual(again2[4][1]["rest_w"], REST_W, places=3)

    def test_track_id_change_resets(self):
        self._rest()
        t = 45 * DT
        first, t = self._pump(t)
        self.assertEqual(first[4][1]["peaks"], 1)
        self.assertEqual(first[4][1]["state"], "idle")
        switched = self.step(t, obs(t, REST_W, track_id=2))
        self.assertTrue(switched["accepted"])
        self.assertEqual(switched["track_id"], 2)
        self.assertEqual(switched["state"], "idle")
        self.assertEqual(switched["peaks"], 0)
        self.assertEqual(switched["out_run"], 0)
        self.assertFalse(switched["out"])
        self.assertIsNone(switched["rest_w"])
        t += DT
        wide = self.step(t, obs(t, PUMP_W, track_id=2))
        self.assertTrue(wide["accepted"])
        self.assertFalse(wide["out"])
        self.assertEqual(wide["peaks"], 0)
        self.assertEqual(wide["would_start_count"], 0)
        self.assertEqual(wide["state"], "idle")

    def test_repeated_timestamp_does_not_count(self):
        self._rest()
        t = 45 * DT
        seen = obs(t, PUMP_W)
        first = self.det.update(seen, INTR, FRAME_W, t)
        self.assertTrue(first["accepted"])
        self.assertTrue(first["out"])
        self.assertEqual(first["out_run"], 1)
        for k in range(1, 6):
            again = self.det.update(seen, INTR, FRAME_W, t + k * 0.05)
            self.assertFalse(again["accepted"])
            self.assertFalse(again["out"])
            self.assertEqual(again["out_run"], 1)
            self.assertEqual(again["peaks"], 0)
            self.assertFalse(again["start_event"])
        t2 = t + DT
        dropped = self.step(t2, obs(t2, REST_W))
        self.assertEqual(dropped["out_run"], 0)
        self.assertEqual(dropped["peaks"], 0)
        for i in range(1, 4):
            t2 += DT
            counted = self.step(t2, obs(t2, PUMP_W))
            self.assertTrue(counted["accepted"])
            self.assertEqual(counted["out_run"], i)
        self.assertEqual(counted["peaks"], 0)

    def test_no_detection_stops_before_hands_down(self):
        _first, second, _t = self._two_peaks()
        self.assertTrue(second[4][1]["start_event"])
        # _pump feeds two more accepted rest frames after the start; the
        # no-detection clock runs from the last accepted frame.
        last_t = second[-1][0]
        self.assertEqual(second[-1][1]["state"], "active")
        early = self.det.update(None, INTR, FRAME_W, last_t + 0.2)
        self.assertEqual(early["state"], "active")
        self.assertFalse(early["stop_event"])
        stopped = self.det.update(None, INTR, FRAME_W, last_t + 0.3)
        self.assertTrue(stopped["stop_event"])
        self.assertEqual(stopped["stop_reason"], "no_detection")
        self.assertEqual(stopped["state"], "cooldown")

    def test_rest_freezes_so_a_wide_run_cannot_move_the_median(self):
        # 8 samples spanning > 1.5 s, then 8 samples between the freeze
        # ratio and the out ratio. Median of an even mix would move;
        # the frozen rest must stay at the standing width.
        times = [i * 0.25 for i in range(8)]
        self.assertGreaterEqual(times[-1] - times[0], 1.5)
        last = None
        for t in times:
            last = self.step(t, obs(t, REST_W))
        self.assertAlmostEqual(last["rest_w"], REST_W, places=6)
        wide = REST_W * 1.45  # above 1.4x, below 1.5x
        t = times[-1]
        for _ in range(8):
            t += DT
            last = self.step(t, obs(t, wide))
            self.assertFalse(last["out"])
            self.assertEqual(last["state"], "idle")
        self.assertAlmostEqual(last["rest_w"], REST_W, places=6)
        self.assertEqual(last["peaks"], 0)

    def _pump_with_band_descent(self, t, n_out=4):
        """n_out wide frames, one frame in the 1.4x-1.5x band, two rest frames."""
        band = REST_W * 1.45
        last = None
        for w in [PUMP_W] * n_out + [band, REST_W, REST_W]:
            last = self.step(t, obs(t, w))
            t += DT
        return last, t

    def test_band_frame_on_the_descent_keeps_the_peak(self):
        # A real pump's descent can land one frame between peak_end_ratio
        # and out_ratio. That frame must not throw the peak away.
        self._rest()
        t = 45 * DT
        first, t = self._pump_with_band_descent(t)
        self.assertEqual(first["peaks"], 1)
        self.assertEqual(first["state"], "idle")
        second, t = self._pump_with_band_descent(t)
        self.assertEqual(second["state"], "active")
        self.assertEqual(second["would_start_count"], 1)

    def test_band_frame_before_the_run_is_armed_still_resets(self):
        # Only min_out_frames consecutive out frames arm a peak. Two out
        # frames, a band frame, then one more out frame is not a peak.
        self._rest()
        t = 45 * DT
        band = REST_W * 1.45
        last = None
        for _ in range(2):
            for w in [PUMP_W, PUMP_W, band, PUMP_W, REST_W, REST_W]:
                last = self.step(t, obs(t, w))
                t += DT
        self.assertEqual(last["peaks"], 0)
        self.assertEqual(last["state"], "idle")
        self.assertEqual(last["would_start_count"], 0)

    def test_rest_relearns_after_a_lasting_width_change(self):
        # Standing, then a lasting symmetric widening (a coat, a carried
        # board): no start, no false peak when the rest catches up, and
        # the rest ends at the new width instead of staying stale.
        self._rest()
        t = 45 * DT
        wide = 0.9
        last = None
        for _ in range(int(10.0 / DT)):
            last = self.step(t, obs(t, wide))
            self.assertFalse(last["start_event"])
            self.assertEqual(last["peaks"], 0)
            t += DT
        self.assertEqual(last["state"], "idle")
        self.assertAlmostEqual(last["rest_w"], wide, places=6)
        # A real pump against the new rest still starts (2x, inside the
        # 2.2 m body-width limit).
        for _ in range(2):
            for w in [wide * 2.0] * 4 + [wide] * 3:
                last = self.step(t, obs(t, w))
                t += DT
        self.assertEqual(last["state"], "active")

    def _armed_then(self, gap_frames, tail=(REST_W, REST_W)):
        """Rest, 4 out frames, the given gap observations, then tail widths."""
        self._rest()
        t = 45 * DT
        last = None
        for _ in range(4):
            last = self.step(t, obs(t, PUMP_W))
            t += DT
        self.assertGreaterEqual(last["out_run"], 4)
        for make in gap_frames:
            last = self.step(t, make(t))
            t += DT
        for w in tail:
            last = self.step(t, obs(t, w))
            t += DT
        return last

    def test_edge_reject_inside_an_armed_run_breaks_the_peak(self):
        edge_box = (0.005, 0.2, 0.5, 0.8)
        last = self._armed_then(
            [lambda t: obs(t, PUMP_W, bbox=edge_box)] * 2
        )
        self.assertEqual(last["peaks"], 0)

    def test_depth_dropout_inside_an_armed_run_is_bridged(self):
        last = self._armed_then([lambda t: obs(t, PUMP_W, depth_ok=False)])
        self.assertEqual(last["peaks"], 1)

    def test_slow_drift_through_the_band_is_not_a_peak(self):
        # 8 band frames (> max_hold_s at 15 Hz), then rest: not a pump.
        band = REST_W * 1.45
        last = self._armed_then([lambda t: obs(t, band)] * 8)
        self.assertEqual(last["peaks"], 0)

    def test_max_hold_default(self):
        self.assertEqual(PumpGestureConfig().max_hold_s, 0.4)


if __name__ == "__main__":
    unittest.main()
