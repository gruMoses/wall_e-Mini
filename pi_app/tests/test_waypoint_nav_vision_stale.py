"""WAYPOINT_NAV must not run on a stale detection stream (2026-11-10 audit, item 4).

FOLLOW_ME leaves when the detection stream is older than
``OakDetectionConfig.vision_stale_s`` (``OakDepthReader.get_detection_freshness``).
WAYPOINT_NAV did not: with the colour camera stalled, the run continued on the
depth corridor alone and the person stop was blind. It now uses the same
freshness source and the same exit: mode MANUAL before the tick's command, the
sticks are the target, the slew limiter ramps (never bypassed). The stored
detection list is NOT cleared (a stale close person must still force the stop).
"""

import unittest

from pi_app.control.follow_me import PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.tests.test_waypoint_nav_heading_guard import _clock, _Rig


def _fresh(age, seq=7, stuck=0):
    return {"age_s": age, "stale": age > 1.5, "seq": seq, "seq_stuck_packets": stuck}


FROZEN_PERSON = PersonDetection(x_m=0.0, z_m=0.6, confidence=0.9, bbox=(0.4, 0.0, 0.6, 0.9))


class TestWaypointNavVisionStale(unittest.TestCase):
    def _driving_rig(self, clock, fresh):
        rig = _Rig(clock)
        rig.ctrl.set_vision_freshness_getter(lambda: fresh["v"])
        self.assertIsNone(rig.ctrl.activate_waypoint_nav())
        for _ in range(10):
            cmd, _, telem = rig.tick()
        self.assertEqual(telem["nav_state"], "DRIVE")
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)
        return rig, cmd.left_byte

    def test_stale_stream_leaves_waypoint_nav_through_the_slew(self):
        clock = {"t": 1000.0}
        fresh = {"v": _fresh(0.1)}
        with _clock(clock):
            rig, driving = self._driving_rig(clock, fresh)
            fresh["v"] = _fresh(1.6)
            cmd, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "MANUAL")
        self.assertEqual(telem["mode"], "MANUAL")
        self.assertEqual(telem["waypoint_nav_exit_reason"], "vision_stale")
        self.assertTrue(telem["vision_stale"])
        # Ramp, not a jump: the slew limiter is not bypassed.
        self.assertLess(cmd.left_byte, driving)
        self.assertGreater(cmd.left_byte, CENTER_OUTPUT_VALUE)

    def test_fresh_stream_keeps_running(self):
        clock = {"t": 1000.0}
        fresh = {"v": _fresh(0.1)}
        with _clock(clock):
            rig, _ = self._driving_rig(clock, fresh)
            fresh["v"] = _fresh(1.5)  # at the limit: not stale
            _, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")
        self.assertNotIn("waypoint_nav_exit_reason", telem)

    def test_stale_exit_does_not_clear_the_stored_detections(self):
        clock = {"t": 1000.0}
        fresh = {"v": _fresh(0.1)}
        with _clock(clock):
            rig, _ = self._driving_rig(clock, fresh)
            rig.ctrl.set_person_detections([FROZEN_PERSON])
            fresh["v"] = _fresh(30.0)
            rig.tick()
        self.assertEqual(rig.ctrl._mode, "MANUAL")
        self.assertEqual(rig.ctrl._person_detections, [FROZEN_PERSON])

    def test_getter_failure_is_stale(self):
        clock = {"t": 1000.0}
        fresh = {"v": _fresh(0.1)}
        with _clock(clock):
            rig, _ = self._driving_rig(clock, fresh)

            def boom():
                raise RuntimeError("reader gone")

            rig.ctrl.set_vision_freshness_getter(boom)
            _, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "MANUAL")
        self.assertEqual(telem["waypoint_nav_exit_reason"], "vision_stale")

    def test_activation_refused_on_a_stale_stream(self):
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            rig.ctrl.set_vision_freshness_getter(lambda: _fresh(float("inf")))
            self.assertEqual(rig.ctrl.activate_waypoint_nav(), "vision_stale")
        self.assertEqual(rig.ctrl._mode, "MANUAL")

    def test_no_getter_keeps_the_old_behaviour(self):
        # Same as FOLLOW_ME: with no freshness getter (no OAK reader) the
        # gate is inactive. main.py registers it whenever the OAK exists.
        clock = {"t": 1000.0}
        with _clock(clock):
            rig = _Rig(clock)
            self.assertIsNone(rig.ctrl.activate_waypoint_nav())
            _, _, telem = rig.tick()
        self.assertEqual(rig.ctrl._mode, "WAYPOINT_NAV")
        self.assertNotIn("vision_stale", telem)


if __name__ == "__main__":
    unittest.main()
