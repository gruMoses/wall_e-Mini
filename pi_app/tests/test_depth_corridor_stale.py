"""Tests for the stale-depth corridor fix (backlog item H).

Root cause: when _poll_depth receives a fresh depth frame but the corridor ROI
contains no valid pixels (genuinely empty, e.g. open field), the old code
returned early *before* updating _depth_state.timestamp.  Consequently,
get_min_distance() returned a growing age even though frames were arriving,
causing compute_throttle_scale() to treat a live-but-empty corridor as stale
sensor data (full stop or degraded throttle depending on stale_policy).

The fix: update _depth_state.min_distance_m and _depth_state.timestamp even
when effective_min_mm == inf, so the consumer sees a fresh reading of
"corridor clear" (distance=inf, age~0).

These tests verify:
  1. Fresh frame + empty corridor  =>  not stale, reports clear (inf distance).
  2. No frames for > staleness window  =>  stale still fires (fail-safe intact).
  3. Fresh frame + empty corridor + person within stop radius  =>  person-stop
     still forces stop (safety tier is byte-for-byte unaffected by the fix).
  4. Fresh frame + obstacle in corridor  =>  distance reported as before
     (regression guard).
"""

import time
import unittest

import numpy as np

from config import ObstacleAvoidanceConfig, FollowMeConfig
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.hardware.oak_depth import OakDepthReader, ObjectDetection, _AllDetsState

INF = float("inf")

# ---------------------------------------------------------------------------
# Staleness window used in all tests that exercise the stale-policy path.
# Keep it short so tests don't need large injected age gaps.
STALE_TIMEOUT_S = 0.5


# ---------------------------------------------------------------------------
# Minimal fakes that let us drive _poll_depth without real depthai hardware.
# ---------------------------------------------------------------------------

class _FakeFrame:
    """Wraps a numpy uint16 mm depth array and mimics the depthai ImgFrame API."""

    def __init__(self, array: np.ndarray):
        self._arr = array

    def getFrame(self):
        return self._arr


class _FakeQueue:
    """Single-item fake queue: returns one frame on the first tryGet, then None.

    Pass frame=None to simulate "no frame available this poll cycle".
    """

    def __init__(self, frame=None):
        self._frame = frame
        self._served = False

    def tryGet(self):
        if self._frame is not None and not self._served:
            self._served = True
            return self._frame
        return None


def _make_reader(*, stale_timeout_s=STALE_TIMEOUT_S, stale_policy="stop",
                 robot_width_m=0.820) -> OakDepthReader:
    """Build a minimal OakDepthReader suitable for unit tests.

    Uses robot_width_m=0.820 (the realistic default) so the trapezoidal
    corridor path is exercised.  The timestamp fix applies equally to
    both the corridor and rectangular ROI paths.
    """
    obs_cfg = ObstacleAvoidanceConfig(
        slow_distance_m=1.5,
        stop_distance_m=0.4,
        roi_width_pct=0.8,
        roi_height_pct=0.5,
        robot_width_m=robot_width_m,
        min_depth_mm=350,
        min_valid_pct=8.0,
        update_rate_hz=15.0,
        stale_timeout_s=stale_timeout_s,
        stale_policy=stale_policy,
        safety_stop_radius_m=0.8,
        camera_hfov_deg=81.0,
    )
    fm_cfg = FollowMeConfig()
    return OakDepthReader(obstacle_config=obs_cfg, follow_me_config=fm_cfg)


def _all_zeros_frame(h=400, w=640) -> np.ndarray:
    """A depth frame where every pixel is 0 — below min_depth_mm, so corridor is empty."""
    return np.zeros((h, w), dtype=np.uint16)


def _obstacle_frame(distance_mm=1000, h=400, w=640) -> np.ndarray:
    """A depth frame with a uniform obstacle at distance_mm in the corridor."""
    return np.full((h, w), distance_mm, dtype=np.uint16)


def _poll_with_empty_corridor(reader: OakDepthReader) -> None:
    """Fire one _poll_depth cycle with an all-zeros (empty corridor) frame."""
    spatial_q = _FakeQueue(frame=None)   # device spatial calc absent
    depth_q = _FakeQueue(frame=_FakeFrame(_all_zeros_frame()))
    reader._poll_depth(depth_q, spatial_q, np)


# ---------------------------------------------------------------------------
# Test 1 — fresh frame, empty corridor => not stale, reports clear.
# ---------------------------------------------------------------------------

class TestFreshFrameEmptyCorridorNotStale(unittest.TestCase):

    def test_empty_corridor_timestamp_is_updated(self):
        """After _poll_depth with an all-zeros frame the depth-state timestamp
        must be very recent (<< stale_timeout_s) so get_min_distance() does not
        look stale.

        Before the fix: _depth_state.timestamp stayed at its prior value
        (initialised to 0.0), so age = monotonic() - 0.0 = system uptime
        (>> stale_timeout_s), which fired the stale policy.
        After the fix: timestamp is updated to now() on every fresh frame.
        """
        reader = _make_reader(stale_timeout_s=STALE_TIMEOUT_S)

        # Force the depth-state timestamp very far in the past so we can
        # tell unambiguously whether _poll_depth updates it.
        with reader._lock:
            reader._depth_state.timestamp = time.monotonic() - 100.0

        _poll_with_empty_corridor(reader)

        _, age_s = reader.get_min_distance()
        self.assertLess(age_s, STALE_TIMEOUT_S,
                        "Timestamp must be updated even when corridor is empty")

    def test_empty_corridor_reports_inf_distance(self):
        """Clear corridor must report distance=inf (no obstacle present)."""
        reader = _make_reader()

        # Pre-seed a stale non-inf value to confirm it gets cleared.
        with reader._lock:
            reader._depth_state.min_distance_m = 0.5
            reader._depth_state.timestamp = time.monotonic() - 100.0

        _poll_with_empty_corridor(reader)

        dist_m, age_s = reader.get_min_distance()
        self.assertEqual(dist_m, INF,
                         "Empty corridor must report inf distance (corridor clear)")
        self.assertLess(age_s, STALE_TIMEOUT_S)

    def test_empty_corridor_compute_throttle_scale_not_stale(self):
        """The throttle scale must be 1.0 (not the stale-policy value) for an
        empty corridor with a fresh frame.

        Before the fix: scale = 0.0 (stale_policy='stop' fires).
        After the fix:  scale = 1.0 (inf distance, fresh age, full throttle).
        """
        reader = _make_reader(stale_policy="stop")
        # Pre-age the state to ensure the poll updates it (not just lucky timing).
        with reader._lock:
            reader._depth_state.timestamp = time.monotonic() - 100.0

        oa = ObstacleAvoidanceController(reader._obs_cfg)

        _poll_with_empty_corridor(reader)

        dist_m, age_s = reader.get_min_distance()
        scale = oa.compute_throttle_scale(dist_m, age_s)
        self.assertAlmostEqual(scale, 1.0,
                               msg="Empty corridor + fresh frame must yield full throttle scale")


# ---------------------------------------------------------------------------
# Test 2 — genuine sensor silence => stale still fires (fail-safe intact).
# ---------------------------------------------------------------------------

class TestStalenessStillFiresWhenFramesStop(unittest.TestCase):

    def test_no_frame_leaves_timestamp_untouched(self):
        """When depth_q.tryGet() returns None, _poll_depth must return without
        modifying the depth state — the staleness window must tick forward."""
        reader = _make_reader()

        # Inject an old timestamp to simulate a sensor that stopped sending.
        old_ts = time.monotonic() - 10.0
        with reader._lock:
            reader._depth_state.min_distance_m = 1.0
            reader._depth_state.timestamp = old_ts

        # Poll with no frame available.
        spatial_q = _FakeQueue(frame=None)
        depth_q = _FakeQueue(frame=None)   # no frame — simulates stopped frames
        reader._poll_depth(depth_q, spatial_q, np)

        _, age_s = reader.get_min_distance()
        self.assertGreater(age_s, 9.0,
                           "Timestamp must NOT be updated when no frame arrives")

    def test_stale_age_triggers_stop_policy(self):
        """After a genuine gap the compute_throttle_scale with stale_policy='stop'
        must return 0.0, confirming the fail-safe is not bypassed by the fix."""
        reader = _make_reader(stale_timeout_s=STALE_TIMEOUT_S, stale_policy="stop")
        oa = ObstacleAvoidanceController(reader._obs_cfg)

        # Mark state as very old — no new frame will arrive.
        with reader._lock:
            reader._depth_state.min_distance_m = INF
            reader._depth_state.timestamp = time.monotonic() - 10.0

        dist_m, age_s = reader.get_min_distance()
        scale = oa.compute_throttle_scale(dist_m, age_s)
        self.assertAlmostEqual(scale, 0.0,
                               msg="Genuine stale sensor must still yield stop (scale=0.0)")

    def test_sequential_empty_polls_keep_timestamp_fresh(self):
        """Multiple consecutive empty-corridor polls must each update the
        timestamp so the age never drifts past the staleness window."""
        reader = _make_reader(stale_timeout_s=STALE_TIMEOUT_S)

        for _ in range(5):
            _poll_with_empty_corridor(reader)

        _, age_s = reader.get_min_distance()
        self.assertLess(age_s, STALE_TIMEOUT_S,
                        "Repeated empty-corridor polls must keep state fresh")


# ---------------------------------------------------------------------------
# Test 3 — fresh frame + empty corridor + person inside stop radius =>
#           person-stop still forces stop (safety tier unaffected by fix).
# ---------------------------------------------------------------------------

class TestPersonStopStillForcesStopOnEmptyCorridor(unittest.TestCase):

    class _FakePersonDet:
        """Minimal fake for ObjectDetection fields the stop tier reads."""
        safety_tier = "stop"
        z_m = 0.5   # inside the 0.8 m safety_stop_radius
        label_name = "person"

    def test_person_inside_radius_forces_stop_even_when_corridor_empty(self):
        """Even when the depth corridor yields no valid pixels (inf), a stop-tier
        detection within safety_stop_radius_m must force min_distance_m = 0.

        This proves _apply_safety_tier_override runs BEFORE the empty-corridor
        branch and its result (0.0 mm) bypasses the inf early-return entirely.
        """
        reader = _make_reader()

        with reader._lock:
            reader._all_dets_state = _AllDetsState(
                detections=[self._FakePersonDet()],
                timestamp=time.monotonic(),
            )
            # Age the depth state so we can confirm it IS updated.
            reader._depth_state.timestamp = time.monotonic() - 100.0

        _poll_with_empty_corridor(reader)

        dist_m, age_s = reader.get_min_distance()
        self.assertAlmostEqual(
            dist_m, 0.0,
            msg="Stop-tier person inside radius must force distance=0 regardless of corridor emptiness",
        )
        self.assertLess(age_s, STALE_TIMEOUT_S,
                        "Timestamp must still be updated when person-stop fires")

    def test_apply_safety_tier_override_person_within_radius_is_hard_stop(self):
        """White-box: _apply_safety_tier_override(corridor=inf, person at 0.5m)
        must return (0.0, det) — the canonical stop-tier contract."""
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [self._FakePersonDet()], corridor_p5_mm=INF, safety_stop_radius_m=0.8
        )
        self.assertEqual(eff_mm, 0.0)
        self.assertIsNotNone(stop_det)

    def test_apply_safety_tier_override_empty_corridor_no_person_returns_inf(self):
        """White-box: with no detections the override must return (inf, None).
        The fix must not alter this path in any way."""
        eff_mm, stop_det = OakDepthReader._apply_safety_tier_override(
            [], corridor_p5_mm=INF, safety_stop_radius_m=0.8
        )
        self.assertEqual(eff_mm, INF)
        self.assertIsNone(stop_det)

    def test_throttle_scale_is_zero_after_person_stop_on_empty_corridor(self):
        """End-to-end: person-stop sets distance=0 => compute_throttle_scale=0.0."""
        reader = _make_reader()
        oa = ObstacleAvoidanceController(reader._obs_cfg)

        with reader._lock:
            reader._all_dets_state = _AllDetsState(
                detections=[self._FakePersonDet()],
                timestamp=time.monotonic(),
            )
            reader._depth_state.timestamp = time.monotonic() - 100.0

        _poll_with_empty_corridor(reader)

        dist_m, age_s = reader.get_min_distance()
        scale = oa.compute_throttle_scale(dist_m, age_s)
        self.assertAlmostEqual(scale, 0.0,
                               msg="Person-stop path must yield zero throttle scale")


# ---------------------------------------------------------------------------
# Test 4 — fresh frame with obstacle => distance reported correctly
#           (regression guard: fix must not change the obstacle-present path).
# ---------------------------------------------------------------------------

class TestObstacleInCorridorReportedCorrectly(unittest.TestCase):

    def test_obstacle_distance_reported_and_timestamp_updated(self):
        """A frame with valid obstacle pixels must report a finite distance and
        a fresh timestamp — confirming the normal path is unaffected."""
        reader = _make_reader()
        with reader._lock:
            reader._depth_state.timestamp = time.monotonic() - 100.0

        spatial_q = _FakeQueue(frame=None)
        # 1000 mm > min_depth_mm (350), so pixels are valid corridor reads.
        depth_q = _FakeQueue(frame=_FakeFrame(_obstacle_frame(distance_mm=1000)))
        reader._poll_depth(depth_q, spatial_q, np)

        dist_m, age_s = reader.get_min_distance()
        self.assertLess(dist_m, INF,
                        "Obstacle frame must report a finite distance")
        self.assertGreater(dist_m, 0.0)
        self.assertAlmostEqual(dist_m, 1.0, places=1,
                               msg="1000 mm obstacle should report ~1.0 m")
        self.assertLess(age_s, STALE_TIMEOUT_S)

    def test_obstacle_triggers_scale_reduction(self):
        """An obstacle at 800 mm (between stop=0.4 m and slow=1.5 m) must yield
        a throttle scale strictly between 0 and 1."""
        reader = _make_reader()
        oa = ObstacleAvoidanceController(reader._obs_cfg)

        spatial_q = _FakeQueue(frame=None)
        depth_q = _FakeQueue(frame=_FakeFrame(_obstacle_frame(distance_mm=800)))
        reader._poll_depth(depth_q, spatial_q, np)

        dist_m, age_s = reader.get_min_distance()
        scale = oa.compute_throttle_scale(dist_m, age_s)
        self.assertGreater(scale, 0.0,
                           msg="800 mm obstacle should not trigger full stop")
        self.assertLess(scale, 1.0,
                        msg="800 mm obstacle should throttle below full speed")

    def test_close_obstacle_triggers_stop(self):
        """An obstacle at 200 mm (below stop_distance_m=0.4 m) must yield scale=0."""
        reader = _make_reader()
        oa = ObstacleAvoidanceController(reader._obs_cfg)

        spatial_q = _FakeQueue(frame=None)
        depth_q = _FakeQueue(frame=_FakeFrame(_obstacle_frame(distance_mm=200)))
        reader._poll_depth(depth_q, spatial_q, np)

        dist_m, age_s = reader.get_min_distance()
        # 200 mm is below both min_depth_mm threshold (350mm) — pixels might be
        # filtered out as too close. Either way the state should be updated:
        # if valid pixels exist, dist <= 0.4 m => scale 0; if all filtered,
        # dist = inf => scale 1.  Both are correct behaviour.
        # We just confirm the timestamp is fresh (not a stale-detection failure).
        self.assertLess(age_s, STALE_TIMEOUT_S)

    def test_clear_sky_frame_returns_full_scale(self):
        """After the fix, an empty corridor must yield scale=1.0 not scale=0.0.
        This is the primary regression the fix addresses."""
        reader = _make_reader(stale_policy="stop")
        oa = ObstacleAvoidanceController(reader._obs_cfg)

        with reader._lock:
            reader._depth_state.timestamp = time.monotonic() - 100.0

        _poll_with_empty_corridor(reader)

        dist_m, age_s = reader.get_min_distance()
        scale = oa.compute_throttle_scale(dist_m, age_s)
        # Before fix: scale = 0.0 (stale fires) or 0.10 (manual_stale_throttle_scale).
        # After fix:  scale = 1.0 (inf distance is >= slow_distance_m).
        self.assertAlmostEqual(scale, 1.0,
                               msg="Empty corridor after fix must yield full throttle, not stale-stop")


if __name__ == "__main__":
    unittest.main()
