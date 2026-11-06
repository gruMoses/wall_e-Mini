"""Log-only VIO: pose bookkeeping, IMU report choice, the y/z flip, and the
fallbacks that keep the camera session up (2026-11-06)."""

import math
from types import SimpleNamespace

import pytest

from config import FollowMeConfig, ObstacleAvoidanceConfig, OakDetectionConfig
from pi_app.app import log_gating
from pi_app.hardware import oak_depth
from pi_app.hardware.oak_depth import (
    OakDepthReader,
    imu_report_names,
    imu_session_reports,
    vio_decision,
)
from pi_app.hardware.vio_track import VioTracker, quaternion_to_rpy_deg


def _q_yaw(deg):
    """Quaternion for a counter-clockwise yaw of ``deg`` about world Z."""
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _q_roll(deg):
    h = math.radians(deg) / 2.0
    return (math.sin(h), 0.0, 0.0, math.cos(h))


# --- VioTracker -----------------------------------------------------------

def test_right_turn_is_positive_heading_like_the_imu():
    t = VioTracker()
    t.new_session()
    t.update(0, 0, 0, *_q_yaw(10.0), now_s=0.0)
    # Basalt: counter-clockwise positive; a right turn lowers yaw by 90.
    t.update(0, 0, 0, *_q_yaw(-80.0), now_s=1.0)
    assert t.heading_deg == pytest.approx(90.0, abs=1e-6)


def test_heading_unwraps_across_the_seam():
    t = VioTracker()
    t.new_session()
    for i, yaw in enumerate([170.0, 179.0, -179.0, -170.0]):
        t.update(0, 0, 0, *_q_yaw(yaw), now_s=float(i))
    assert t.heading_deg == pytest.approx(-20.0, abs=1e-6)  # 20 deg left


def test_path_and_position():
    t = VioTracker()
    t.new_session()
    t.update(0, 0, 0, *_q_yaw(0), now_s=0.0)
    t.update(3, 4, 0, *_q_yaw(0), now_s=1.0)
    t.update(3, 4, 0, *_q_yaw(0), now_s=2.0)
    snap = t.snapshot(2.0)
    assert snap["vio_path_m"] == pytest.approx(5.0)
    assert (snap["vio_x_m"], snap["vio_y_m"]) == (3.0, 4.0)
    assert snap["vio_poses"] == 3


def test_upside_down_world_marks_the_frame_bad_and_it_stays_bad():
    t = VioTracker()
    t.new_session()
    t.update(0, 0, 0, *_q_roll(-179.0), now_s=0.0)  # measured with UNCALIBRATED
    assert t.frame_ok is False
    t.update(0, 0, 0, *_q_roll(0.0), now_s=1.0)
    assert t.frame_ok is False
    t2 = VioTracker()
    t2.new_session()
    t2.update(0, 0, 0, *_q_roll(-1.6), now_s=0.0)  # measured with RAW
    assert t2.frame_ok is True


def test_new_session_resets_and_counts():
    t = VioTracker()
    t.new_session()
    t.update(1, 1, 1, *_q_yaw(30), now_s=0.0)
    t.new_session()
    snap = t.snapshot(0.0)
    assert snap["vio_session"] == 2
    assert snap["vio_poses"] == 0 and snap["vio_heading_deg"] is None
    assert snap["vio_age_s"] is None


def test_non_finite_pose_is_ignored():
    t = VioTracker()
    t.new_session()
    t.update(float("nan"), 0, 0, *_q_yaw(0), now_s=0.0)
    assert t.poses == 0


def test_rate_counts_the_window():
    t = VioTracker()
    t.new_session()
    for i in range(31):
        t.update(0, 0, 0, *_q_yaw(0), now_s=i / 15.0)
    assert t.rate_hz(2.0) == pytest.approx(15.0, abs=1.0)


def test_rpy_identity():
    assert quaternion_to_rpy_deg(0, 0, 0, 1) == (0.0, 0.0, 0.0)


# --- report choice and decision ---------------------------------------------

def test_vio_off_keeps_todays_reports_without_flip():
    for version in ("3.3.0", "3.6.1"):
        accel, gyro, flip = imu_session_reports(version, vio=False)
        assert (accel, gyro) == imu_report_names(version)
        assert flip is False


def test_vio_on_requests_raw_and_flips():
    assert imu_session_reports("3.6.1", vio=True) == (
        "ACCELEROMETER_RAW", "GYROSCOPE_RAW", True)


def test_vio_decision():
    node = SimpleNamespace(BasaltVIO=object)
    assert vio_decision(False, "3.6.1", node) == (False, "off")
    assert vio_decision(True, "3.6.1", node) == (True, "on")
    assert vio_decision(True, "3.3.0", node)[0] is False
    assert vio_decision(True, "3.6.0", node)[0] is False
    assert vio_decision(True, "garbage", node)[0] is False
    assert vio_decision(True, "3.6.1", SimpleNamespace())[0] is False
    on, reason = vio_decision(True, "3.6.1", node, "build failed (x)")
    assert on is False and "build failed" in reason


def test_config_default_is_off():
    assert OakDetectionConfig().vio_enabled is False


# --- the reader: flip, latch, health ----------------------------------------

def _reader(vio_enabled=False):
    det = OakDetectionConfig(vio_enabled=vio_enabled)
    return OakDepthReader(obstacle_config=ObstacleAvoidanceConfig(),
                          follow_me_config=FollowMeConfig(),
                          detection_config=det)


class _Q:
    def __init__(self, msgs):
        self._msgs = list(msgs)

    def tryGet(self):
        return self._msgs.pop(0) if self._msgs else None


def _imu_msg(accel, gyro):
    v = lambda t: SimpleNamespace(x=t[0], y=t[1], z=t[2])  # noqa: E731
    pkt = SimpleNamespace(acceleroMeter=v(accel), gyroscope=v(gyro))
    return SimpleNamespace(packets=[pkt])


def _captured_packets(reader, monkeypatch):
    seen = []
    orig = reader._imu_yaw_producer.ingest

    def spy(parsed, host_now_s):
        seen.extend(parsed)
        return orig(parsed, host_now_s=host_now_s)

    monkeypatch.setattr(reader._imu_yaw_producer, "ingest", spy)
    return seen


@pytest.mark.parametrize("flip", [False, True])
def test_poll_imu_flip_reproduces_uncalibrated(monkeypatch, flip):
    """RAW (3.6.1, bench 2026-11-05) flipped == UNCALIBRATED, sign for sign."""
    reader = _reader()
    seen = _captured_packets(reader, monkeypatch)
    reader._imu_flip_yz = flip
    raw_accel = (-0.009, +0.988, +0.047)
    raw_gyro = (+0.210, +0.598, -0.161)
    reader._poll_imu(_Q([_imu_msg(raw_accel, raw_gyro)]))
    assert len(seen) == 1
    p = seen[0]
    if flip:
        assert (p.ax_mss, p.ay_mss, p.az_mss) == pytest.approx((-0.009, -0.988, -0.047))
        assert (p.gx_rads, p.gy_rads, p.gz_rads) == pytest.approx((+0.210, -0.598, +0.161))
    else:
        assert (p.ax_mss, p.ay_mss, p.az_mss) == pytest.approx(raw_accel)
        assert (p.gx_rads, p.gy_rads, p.gz_rads) == pytest.approx(raw_gyro)


def test_poll_vio_feeds_the_tracker():
    reader = _reader(vio_enabled=True)
    reader._vio_tracker.new_session()
    reader._vio_active = True
    msg = SimpleNamespace(
        getTranslation=lambda: SimpleNamespace(x=1.0, y=2.0, z=0.0),
        getQuaternion=lambda: SimpleNamespace(qx=0.0, qy=0.0, qz=0.0, qw=1.0),
    )
    reader._poll_vio(_Q([msg, msg]))
    h = reader.get_health()
    assert h["vio_status"] == "on"
    assert h["vio_poses"] == 2
    assert h["vio_x_m"] == 1.0


def test_poll_vio_error_is_counted_not_raised():
    reader = _reader(vio_enabled=True)

    class Bad:
        def tryGet(self):
            raise RuntimeError("boom")

    reader._poll_vio(Bad())
    assert reader.get_health()["vio_errors"] == 1


def test_two_short_vio_sessions_disable_vio_until_restart():
    reader = _reader(vio_enabled=True)
    for _ in range(2):
        reader._vio_active = True
        reader._note_vio_session_end(5.0)
    assert reader._vio_disabled_reason
    on, _ = vio_decision(True, "3.6.1", SimpleNamespace(BasaltVIO=object),
                         reader._vio_disabled_reason)
    assert on is False
    assert reader.get_health()["vio_status"].startswith("disabled")


def test_a_long_session_resets_the_short_count():
    reader = _reader(vio_enabled=True)
    reader._vio_active = True
    reader._note_vio_session_end(5.0)
    reader._vio_active = True
    reader._note_vio_session_end(600.0)
    reader._vio_active = True
    reader._note_vio_session_end(5.0)
    assert not reader._vio_disabled_reason


def test_session_end_without_vio_counts_nothing():
    reader = _reader(vio_enabled=True)
    reader._vio_active = False
    reader._note_vio_session_end(1.0)
    reader._note_vio_session_end(1.0)
    assert reader._vio_short_sessions == 0


def test_health_and_log_block_when_off():
    reader = _reader(vio_enabled=False)
    h = reader.get_health()
    assert h["vio_status"] == "off"
    assert h["imu_flip_yz"] is False
    assert log_gating._vio_block(h) is None


def test_log_block_when_on():
    oak = {"vio_status": "on", "vio_heading_deg": 12.5, "vio_x_m": 1.0,
           "vio_y_m": 0.0, "vio_z_m": 0.0, "vio_path_m": 1.0, "vio_age_s": 0.05,
           "vio_frame_ok": True, "vio_session": 1, "vio_poses": 30,
           "vio_rate_hz": 15.0, "vio_roll_deg": 0.5, "vio_pitch_deg": -2.0,
           "imu_flip_yz": True, "vio_errors": 0, "last_vio_error": None}
    tick = log_gating._vio_block(oak)
    assert tick["hdg"] == 12.5 and "roll" not in tick
    slow = log_gating._vio_block(oak, slow=True)
    assert slow["imu_flip_yz"] is True and slow["hz"] == 15.0


def test_module_constant():
    assert oak_depth.VIO_MIN_DEPTHAI == (3, 6, 1)
