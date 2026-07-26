"""FIX 3 — PID dt measures time since last compute() call, not last output.

At 30fps with a 15Hz output hold, dt was being derived from _last_output_time,
so the PID derivative terms saw alternating dt of ~0 and ~66ms. The fix tracks
a separate _last_compute_time so dt reflects the per-frame compute cadence.
"""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.follow_me import FollowMeController, PersonDetection


def _person(x_m=0.0, z_m=3.0, confidence=0.9,
            bbox=(0.4, 0.0, 0.6, 0.8), track_id=None) -> PersonDetection:
    return PersonDetection(x_m=x_m, z_m=z_m, confidence=confidence,
                           bbox=bbox, track_id=track_id)


class TestPidComputeDt(unittest.TestCase):
    def test_dt_is_compute_cadence_never_zero(self):
        fm = FollowMeController(FollowMeConfig(follow_output_rate_hz=15.0))

        captured = []
        orig = fm._speed.compute

        def spy(*a, **kw):
            captured.append(kw.get("dt"))
            return orig(*a, **kw)

        fm._speed.compute = spy

        base = 1000.0
        with patch("pi_app.control.follow_me.time") as mt:
            for i in range(8):
                # Fixed 33ms cadence (≈30fps).
                mt.monotonic.return_value = base + i * 0.033
                fm.compute([_person(z_m=3.0)])

        self.assertEqual(len(captured), 8)
        # First call falls back (no prior compute timestamp). Every subsequent
        # call must see ~33ms — never the ~0 a 15Hz output hold would produce.
        for dt in captured[1:]:
            self.assertAlmostEqual(dt, 0.033, places=3)
            self.assertGreater(dt, 0.02)

    def test_output_hold_keeps_independent_timing(self):
        """The 15Hz output hold still uses _last_output_time, unaffected by the
        new compute-cadence dt."""
        fm = FollowMeController(FollowMeConfig(follow_output_rate_hz=15.0))
        base = 2000.0
        with patch("pi_app.control.follow_me.time") as mt:
            mt.monotonic.return_value = base
            fm.compute([_person(z_m=3.0)])
            first_output = fm._last_output_time
            # A second compute only 10ms later (< 66ms output interval) must NOT
            # refresh the output timestamp, even though _last_compute_time did.
            mt.monotonic.return_value = base + 0.010
            fm.compute([_person(z_m=3.0)])
        self.assertEqual(fm._last_output_time, first_output)
        self.assertAlmostEqual(fm._last_compute_time, base + 0.010, places=4)


if __name__ == "__main__":
    unittest.main()
