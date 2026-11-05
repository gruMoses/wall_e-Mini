"""Host-side checks for the depthai upgrade bench tools (2026-11-05).

tools/depthai_upgrade_check.py and tools/vio_probe.py need the OAK, so only
their pure helpers are tested here: the gravity-axis label, the VIO heading
from a quaternion and its unwrap across +/-180, and that both tools take the
robot's own IMU report choice.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

_TOOLS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "tools"))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import depthai_upgrade_check as uc  # noqa: E402
import vio_probe as vp  # noqa: E402
from pi_app.hardware.oak_depth import imu_report_names  # noqa: E402


def _quat_about_z(deg: float):
    half = math.radians(deg) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


@pytest.mark.parametrize("deg", [0.0, 30.0, 90.0, -90.0, 179.0, -179.0])
def test_yaw_from_a_pure_z_rotation(deg):
    assert vp.yaw_deg_from_quaternion(*_quat_about_z(deg)) == pytest.approx(deg, abs=1e-6)


def test_unwrap_continues_across_the_seam():
    # 170 -> -170 is +20 degrees, not -340.
    assert vp.unwrap_deg(170.0, 170.0, -170.0) == pytest.approx(190.0)
    # -170 -> 170 is -20 degrees.
    assert vp.unwrap_deg(-170.0, -170.0, 170.0) == pytest.approx(-190.0)
    # Two full right turns in 10 degree steps add up to 720.
    unwrapped = wrapped = 0.0
    for step in range(1, 73):
        new = (step * 10.0 + 180.0) % 360.0 - 180.0
        unwrapped = vp.unwrap_deg(unwrapped, wrapped, new)
        wrapped = new
    assert unwrapped == pytest.approx(720.0)


def test_gravity_axis_label():
    assert uc._gravity_axis([0.02, -0.99, 0.05]) == "-Y"
    assert uc._gravity_axis([0.98, 0.10, 0.0]) == "+X"
    assert uc._gravity_axis([0.0, 0.1, -1.0]) == "-Z"


def test_the_robot_family_is_one_the_check_samples():
    for version in ("3.3.0", "3.6.1", "3.12.0"):
        accel, gyro = imu_report_names(version)
        assert any(accel in a and gyro in g for a, g in uc.FAMILIES.values()), version


def test_vio_probe_report_choices_match_the_robot_names():
    assert vp.REPORTS["raw"] == imu_report_names("3.3.0")
    assert vp.REPORTS["uncalibrated"] == imu_report_names("3.6.1")


def test_both_tools_default_autocalibration_off():
    assert os.environ.get("DEPTHAI_AUTOCALIBRATION") == "OFF"
