"""Tests for the structured-log content added by the 2026-09-19 logging audit:
the session header, the per-tick log object builder, and the console
heartbeat gate.

Like test_log_gating.py, this avoids importing pi_app.app.main (it pulls in
the hardware stack at module load time) by exercising the pure helpers in
pi_app/app/log_gating.py directly, with plain SimpleNamespace/dict fakes
standing in for the live telemetry/reading objects main.py would pass.

Written as unittest.TestCase (rather than test_log_gating.py's bare pytest
functions) so `python3 -m unittest discover` actually collects and runs it --
bare functions are silently NOT discovered by unittest's loader, only
pytest's, and the project's mandated regression check is unittest discover.
"""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from pi_app.app.log_gating import (
    build_log_obj,
    build_slow_obj,
    should_print_console_line,
    should_write_slow_line,
    to_int,
    round1,
    round_floats,
    _session_header,
    _filter_oak_imu_for_log,
    _filter_imu_status_for_log,
)
from config import config as real_config


def _rc(ch1=1500, ch2=1500, ch3=1500, ch4=1500, ch5=1000):
    return SimpleNamespace(ch1_us=ch1, ch2_us=ch2, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5)


def _cmd(left=126, right=126, armed=True, emergency=False):
    return SimpleNamespace(left_byte=left, right_byte=right, is_armed=armed, emergency_active=emergency)


def _base_kwargs(**overrides):
    kwargs = dict(
        now_ts=1000.0,
        src="RC",
        s=_rc(),
        bt_override=None,
        bt_age=None,
        imu_status=None,
        telem={},
        oak_depth_stats=None,
        oak_persons=[],
        gps_reading=None,
        bms_state=None,
        bms_charging=None,
        recording_state=None,
        cmd=_cmd(),
        loop_dt_ms=33,
        imu_dt_ms=33,
        imu_motion_witness_still=None,
        events=[],
    )
    kwargs.update(overrides)
    return kwargs


class ValueRounderTests(unittest.TestCase):
    def test_to_int_rounds_nested_numbers_and_ignores_bools(self):
        self.assertEqual(
            to_int({"a": 1.6, "b": [2.4, True], "c": None}),
            {"a": 2, "b": [2, True], "c": None},
        )

    def test_round1_rounds_to_one_decimal(self):
        self.assertEqual(round1({"a": 1.23, "b": [4.567]}), {"a": 1.2, "b": [4.6]})

    def test_round_floats_handles_tuples_unlike_round1(self):
        self.assertEqual(round_floats((1.23456, 2.3456), 3), (1.235, 2.346))
        # Tuples pass through round1 unchanged (only dict/list are recursed).
        self.assertEqual(round1((1.23456,)), (1.23456,))


class BuildLogObjVescBlockTests(unittest.TestCase):
    def test_vesc_block_carries_new_fields(self):
        telem = {
            "vesc_left_rpm": 500, "vesc_right_rpm": 505, "vesc_actual_speed_mps": 0.8,
            "vesc_rpm_plausible": False, "vesc_rpm_gate_trips": 3,
            "vesc_left_temp_c": 41.2, "vesc_right_temp_c": 42.5,
            "vesc_left_motor_temp_c": 55.1, "vesc_right_motor_temp_c": 56.3,
            "vesc_left_duty": 0.31, "vesc_right_duty": 0.33,
        }
        obj = build_log_obj(**_base_kwargs(telem=telem))
        vesc = obj["vesc"]
        self.assertIs(vesc["rpm_plausible"], False)
        self.assertEqual(vesc["gate_trips"], 3)
        self.assertEqual(vesc["l_temp_c"], 41.2)
        self.assertEqual(vesc["r_temp_c"], 42.5)
        self.assertEqual(vesc["l_motor_temp_c"], 55.1)
        self.assertEqual(vesc["r_motor_temp_c"], 56.3)
        self.assertEqual(vesc["l_duty"], 0.31)
        self.assertEqual(vesc["r_duty"], 0.33)


class BuildLogObjImuSteeringBlockTests(unittest.TestCase):
    def test_imu_steering_block_carries_new_fields(self):
        telem = {"correction_blend": 0.75, "speed_gain_scale": 0.9}
        imu_status = {"saturated": True}
        obj = build_log_obj(**_base_kwargs(telem=telem, imu_status=imu_status))
        self.assertEqual(obj["imu_steering"]["correction_blend"], 0.75)
        self.assertEqual(obj["imu_steering"]["speed_gain_scale"], 0.9)
        self.assertIs(obj["imu_steering"]["saturated"], True)
        # The full imu_status (with "saturated") is also nested under "imu".
        self.assertIs(obj["imu"]["saturated"], True)

    def test_saturated_none_when_no_imu_status(self):
        obj = build_log_obj(**_base_kwargs(imu_status=None))
        self.assertIsNone(obj["imu_steering"]["saturated"])
        self.assertIsNone(obj["imu"])


class BuildLogObjGpsBlockTests(unittest.TestCase):
    def test_age_s_from_controller_telemetry(self):
        gps_reading = SimpleNamespace(
            latitude=1.0, longitude=2.0, altitude_m=3.0, fix_quality=4,
            satellites_used=12, hdop=0.9, diff_age_s=1.1, station_id=7,
        )
        obj = build_log_obj(**_base_kwargs(telem={"gps_age_s": 0.42}, gps_reading=gps_reading))
        self.assertEqual(obj["gps"]["age_s"], 0.42)
        self.assertEqual(obj["gps"]["fix"], 4)

    def test_age_s_none_without_reading(self):
        obj = build_log_obj(**_base_kwargs(telem={"gps_age_s": 0.42}, gps_reading=None))
        self.assertIsNone(obj["gps"]["age_s"])
        self.assertIsNone(obj["gps"]["fix"])


class BuildLogObjTopLevelTests(unittest.TestCase):
    def test_straight_intent(self):
        obj = build_log_obj(**_base_kwargs(telem={"straight_intent": True}))
        self.assertIs(obj["straight_intent"], True)
        obj2 = build_log_obj(**_base_kwargs(telem={"straight_intent": False}))
        self.assertIs(obj2["straight_intent"], False)

    def test_motor_and_safety_from_cmd(self):
        obj = build_log_obj(**_base_kwargs(cmd=_cmd(left=100, right=150, armed=True, emergency=True)))
        self.assertEqual(obj["motor"], {"L": 100, "R": 150})
        self.assertEqual(obj["safety"], {"armed": True, "emergency": True})


class SessionHeaderTests(unittest.TestCase):
    def test_shape_and_git_fields(self):
        log_path = "/tmp/wall_e_test_logs/run_20260919_000000.log"
        with patch("pi_app.app.log_gating._git_sha", return_value="abc1234"), \
             patch("pi_app.app.log_gating._git_dirty", return_value=True):
            hdr = _session_header(real_config, log_path)

        self.assertEqual(hdr["type"], "session_header")
        self.assertEqual(hdr["schema"], 1)
        self.assertEqual(hdr["git_sha"], "abc1234")
        self.assertIs(hdr["git_dirty"], True)
        self.assertEqual(hdr["file"], log_path)
        self.assertIsInstance(hdr["ts"], float)

        cfg = hdr["config"]
        self.assertEqual(cfg["imu_steering"]["kp"], real_config.imu_steering.kp)
        self.assertEqual(
            cfg["imu_steering"]["oak_yaw_rate_source"], real_config.imu_steering.oak_yaw_rate_source
        )
        self.assertEqual(cfg["follow_me"]["speed_kp"], real_config.follow_me.speed_kp)
        self.assertEqual(cfg["follow_me"]["follow_distance_m"], real_config.follow_me.follow_distance_m)
        self.assertEqual(
            cfg["vesc"]["rpm_plausibility_min_erpm"], real_config.vesc.rpm_plausibility_min_erpm
        )
        self.assertEqual(
            cfg["waypoint_nav"]["min_rtk_quality"], real_config.waypoint_nav.min_rtk_quality
        )
        self.assertEqual(
            cfg["gps_heading_align"]["max_lock_yaw_rate_dps"],
            real_config.gps_heading_align.max_lock_yaw_rate_dps,
        )
        self.assertEqual(cfg["imu_source"], real_config.imu_source)

    def test_git_helpers_fall_back_on_failure(self):
        # subprocess.run raising (e.g. git missing, timeout, not a repo) must
        # not blow up header construction -- fall back to "unknown" / None.
        with patch("pi_app.app.log_gating.subprocess.run", side_effect=OSError("no git")):
            hdr = _session_header(real_config, "/tmp/wall_e_test_logs/arm_x.log")
        self.assertEqual(hdr["git_sha"], "unknown")
        self.assertIsNone(hdr["git_dirty"])

    def test_file_none_when_path_none(self):
        with patch("pi_app.app.log_gating._git_sha", return_value="abc"), \
             patch("pi_app.app.log_gating._git_dirty", return_value=False):
            hdr = _session_header(real_config, None)
        self.assertIsNone(hdr["file"])


class ConsoleGateTests(unittest.TestCase):
    def test_tty_always_prints(self):
        # A TTY prints every tick regardless of elapsed time.
        self.assertTrue(should_print_console_line(True, now=100.0, last_print_t=99.999))
        self.assertTrue(should_print_console_line(True, now=100.0, last_print_t=0.0))

    def test_non_tty_throttled_to_5s(self):
        self.assertFalse(should_print_console_line(False, now=100.0, last_print_t=96.0))
        self.assertTrue(should_print_console_line(False, now=100.0, last_print_t=95.0))

    def test_non_tty_respects_custom_interval(self):
        self.assertFalse(should_print_console_line(False, now=10.0, last_print_t=9.0, min_interval_s=2.0))
        self.assertTrue(should_print_console_line(False, now=11.0, last_print_t=9.0, min_interval_s=2.0))


# ─────────────────────────────────────────────────────────────────────────────
# Commit C: imu_pipeline/oak_camera_health demoted to the "slow" line,
# imu.oak_imu duplicate-counter drop + per-tick rounding, flat
# heading_offset_* removal.
# ─────────────────────────────────────────────────────────────────────────────

class SlowLineCadenceTests(unittest.TestCase):
    def test_fires_at_or_after_one_second(self):
        self.assertTrue(should_write_slow_line(now=101.0, last_write_t=100.0))
        self.assertFalse(should_write_slow_line(now=100.9, last_write_t=100.0))

    def test_unconditional_on_arm_state(self):
        # No is_armed parameter at all -- unlike should_log_tick, this gate
        # fires the same way whether the robot is armed or not.
        self.assertTrue(should_write_slow_line(now=50.0, last_write_t=48.9))

    def test_respects_custom_interval(self):
        self.assertFalse(should_write_slow_line(now=10.0, last_write_t=9.0, interval_s=2.0))
        self.assertTrue(should_write_slow_line(now=11.0, last_write_t=9.0, interval_s=2.0))


class BuildSlowObjTests(unittest.TestCase):
    def test_shape_and_content(self):
        obj = build_slow_obj(
            now_ts=1000.0,
            imu_pipeline={"metrics_available": True, "queue_msgs_received": 42},
            oak_camera_health={"is_stale": False},
            chip_temp_c=41.2345,
        )
        self.assertEqual(obj["type"], "slow")
        self.assertEqual(obj["ts"], 1000.0)
        self.assertIn("ts_iso", obj)
        self.assertEqual(obj["imu_pipeline"], {"metrics_available": True, "queue_msgs_received": 42})
        self.assertEqual(obj["oak_camera_health"], {"is_stale": False})
        # Full precision -- unlike the per-tick "imu" block, the slow line is
        # never rounded by build_slow_obj itself (chip_temp_c comes in
        # pre-rounded from oak_depth.get_health(), but build_slow_obj must
        # not re-round or drop precision on its own).
        self.assertEqual(obj["oak"]["chip_temp_c"], 41.2345)

    def test_none_values_pass_through(self):
        obj = build_slow_obj(now_ts=1.0, imu_pipeline=None, oak_camera_health=None, chip_temp_c=None)
        self.assertIsNone(obj["imu_pipeline"])
        self.assertIsNone(obj["oak_camera_health"])
        self.assertIsNone(obj["oak"]["chip_temp_c"])


class OakImuFilterTests(unittest.TestCase):
    def test_drops_producer_queue_drain_cadence_host_queue_keys(self):
        oak_imu = {
            "yaw_rate_source_cfg": "gyro_y",
            "heading_deg": 12.0,
            "count_read": 100,
            "producer_packets_received": 500,
            "producer_cum_yaw_y_deg": 1.5,
            "queue_msgs_received": 10,
            "queue_drain_count": 3,
            "drain_batch_high_water_msgs": 4,
            "cadence_avg_s": 0.016,
            "host_queue_max_size": 512,
            "max_packets_per_drain": 512,
            "last_batch_packets": 2,
            "zupt_engage_count": 1,
            "bias_updates": 5,
            "bias_gx_dps": 0.01,
            "window_gyro_std_dps": 0.02,
            "last_bias_update_host_ts": 123.0,
            "stationary_tracking_enabled": True,
            "zupt_enabled": True,
        }
        filtered = _filter_oak_imu_for_log(oak_imu)
        # Kept: OakImuReader's own state, not a re-export of get_imu_metrics().
        self.assertEqual(filtered["yaw_rate_source_cfg"], "gyro_y")
        self.assertEqual(filtered["heading_deg"], 12.0)
        self.assertEqual(filtered["count_read"], 100)
        # Dropped: everything that duplicates the imu_pipeline block.
        for key in (
            "producer_packets_received", "producer_cum_yaw_y_deg",
            "queue_msgs_received", "queue_drain_count",
            "drain_batch_high_water_msgs", "cadence_avg_s",
            "host_queue_max_size", "max_packets_per_drain",
            "last_batch_packets", "zupt_engage_count", "bias_updates",
            "bias_gx_dps", "window_gyro_std_dps", "last_bias_update_host_ts",
            "stationary_tracking_enabled", "zupt_enabled",
        ):
            self.assertNotIn(key, filtered, f"{key} should have been dropped")

    def test_keeps_stationary_and_zupt_active_top_level(self):
        # "stationary"/"zupt_active" (OakImuReader's own last-read snapshot)
        # are explicitly kept even though the audit's rationale is dedup --
        # they're cheap booleans, not counters, and useful inline.
        oak_imu = {"stationary": True, "zupt_active": False}
        filtered = _filter_oak_imu_for_log(oak_imu)
        self.assertEqual(filtered, {"stationary": True, "zupt_active": False})

    def test_non_dict_passes_through(self):
        self.assertIsNone(_filter_oak_imu_for_log(None))


class FilterImuStatusForLogTests(unittest.TestCase):
    def test_filters_nested_oak_imu_without_mutating_input(self):
        imu_status = {
            "heading_deg": 5.0,
            "oak_imu": {"heading_deg": 5.0, "producer_packets_received": 10},
        }
        original_oak_imu = imu_status["oak_imu"]
        filtered = _filter_imu_status_for_log(imu_status)
        self.assertNotIn("producer_packets_received", filtered["oak_imu"])
        self.assertEqual(filtered["heading_deg"], 5.0)
        # Original dict passed in must be untouched.
        self.assertIn("producer_packets_received", original_oak_imu)
        self.assertIn("producer_packets_received", imu_status["oak_imu"])

    def test_no_oak_imu_key_passes_through_unchanged(self):
        imu_status = {"heading_deg": 5.0}
        self.assertEqual(_filter_imu_status_for_log(imu_status), imu_status)

    def test_none_passes_through(self):
        self.assertIsNone(_filter_imu_status_for_log(None))


class BuildLogObjCommitCTests(unittest.TestCase):
    def test_per_tick_has_no_imu_pipeline_or_oak_camera_health_keys(self):
        obj = build_log_obj(**_base_kwargs())
        self.assertNotIn("imu_pipeline", obj)
        self.assertNotIn("oak_camera_health", obj)

    def test_per_tick_has_no_flat_heading_offset_keys(self):
        telem = {
            "heading_offset_deg": 3.456, "heading_offset_locked": True,
            "heading_offset_frozen": False, "heading_offset_refining": True,
            "heading_align": {
                "offset_deg": 3.456, "locked": True, "frozen": False, "refining": True,
            },
        }
        obj = build_log_obj(**_base_kwargs(telem=telem))
        for key in (
            "heading_offset_deg", "heading_offset_locked",
            "heading_offset_frozen", "heading_offset_refining",
        ):
            self.assertNotIn(key, obj)
        # The same values remain available via heading_align.
        self.assertEqual(obj["heading_align"]["offset_deg"], 3.5)  # round1
        self.assertIs(obj["heading_align"]["locked"], True)
        self.assertIs(obj["heading_align"]["frozen"], False)
        self.assertIs(obj["heading_align"]["refining"], True)
        # corrected_heading_deg is unaffected (not part of this drop).
        self.assertIn("corrected_heading_deg", obj)

    def test_imu_floats_rounded_to_3_decimals(self):
        imu_status = {
            "heading_deg": 12.345678,
            "yaw_rate_dps": -1.23456789,
            "oak_imu": {"gx_body_dps": 0.0123456, "heading_deg": 12.345678},
        }
        obj = build_log_obj(**_base_kwargs(imu_status=imu_status))
        self.assertEqual(obj["imu"]["heading_deg"], 12.346)
        self.assertEqual(obj["imu"]["yaw_rate_dps"], -1.235)
        self.assertEqual(obj["imu"]["oak_imu"]["gx_body_dps"], 0.012)
        self.assertEqual(obj["imu"]["oak_imu"]["heading_deg"], 12.346)

    def test_imu_oak_imu_producer_keys_dropped_from_per_tick(self):
        imu_status = {
            "heading_deg": 1.0,
            "oak_imu": {
                "heading_deg": 1.0,
                "producer_packets_received": 500,
                "queue_msgs_received": 10,
            },
        }
        obj = build_log_obj(**_base_kwargs(imu_status=imu_status))
        self.assertNotIn("producer_packets_received", obj["imu"]["oak_imu"])
        self.assertNotIn("queue_msgs_received", obj["imu"]["oak_imu"])
        self.assertEqual(obj["imu"]["oak_imu"]["heading_deg"], 1.0)


if __name__ == "__main__":
    unittest.main()
