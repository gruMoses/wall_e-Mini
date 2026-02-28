import unittest

from config import ObstacleAvoidanceConfig
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController


class TestObstacleAvoidanceController(unittest.TestCase):

    def _make(self, **overrides) -> ObstacleAvoidanceController:
        cfg = ObstacleAvoidanceConfig(**overrides)
        return ObstacleAvoidanceController(cfg)

    # --- Distance-based scaling ---

    def test_beyond_slow_distance_returns_1(self):
        oa = self._make(slow_distance_m=1.5, stop_distance_m=0.4)
        self.assertAlmostEqual(oa.compute_throttle_scale(2.0, age_s=0.0), 1.0)

    def test_at_slow_distance_returns_1(self):
        oa = self._make(slow_distance_m=1.5, stop_distance_m=0.4)
        self.assertAlmostEqual(oa.compute_throttle_scale(1.5, age_s=0.0), 1.0)

    def test_at_stop_distance_returns_0(self):
        oa = self._make(slow_distance_m=1.5, stop_distance_m=0.4)
        self.assertAlmostEqual(oa.compute_throttle_scale(0.4, age_s=0.0), 0.0)

    def test_below_stop_distance_returns_0(self):
        oa = self._make(slow_distance_m=1.5, stop_distance_m=0.4)
        self.assertAlmostEqual(oa.compute_throttle_scale(0.1, age_s=0.0), 0.0)

    def test_midpoint_returns_half(self):
        oa = self._make(slow_distance_m=1.5, stop_distance_m=0.5)
        mid = (1.5 + 0.5) / 2.0  # 1.0
        scale = oa.compute_throttle_scale(mid, age_s=0.0)
        self.assertAlmostEqual(scale, 0.5)

    def test_quarter_point(self):
        oa = self._make(slow_distance_m=2.0, stop_distance_m=0.0)
        scale = oa.compute_throttle_scale(0.5, age_s=0.0)
        self.assertAlmostEqual(scale, 0.25)

    # --- Stale data handling ---

    def test_stale_stop_policy_returns_0(self):
        oa = self._make(stale_timeout_s=0.5, stale_policy="stop")
        scale = oa.compute_throttle_scale(10.0, age_s=1.0)
        self.assertAlmostEqual(scale, 0.0)

    def test_stale_clear_policy_returns_1(self):
        oa = self._make(stale_timeout_s=0.5, stale_policy="clear")
        scale = oa.compute_throttle_scale(10.0, age_s=1.0)
        self.assertAlmostEqual(scale, 1.0)

    def test_fresh_data_not_treated_as_stale(self):
        oa = self._make(stale_timeout_s=0.5, stale_policy="stop")
        scale = oa.compute_throttle_scale(10.0, age_s=0.3)
        self.assertAlmostEqual(scale, 1.0)

    # --- Status ---

    def test_get_status_after_compute(self):
        oa = self._make()
        oa.compute_throttle_scale(1.0, age_s=0.0)
        status = oa.get_status()
        self.assertIn("obstacle_distance_m", status)
        self.assertIn("obstacle_throttle_scale", status)
        self.assertAlmostEqual(status["obstacle_distance_m"], 1.0)


if __name__ == "__main__":
    unittest.main()
