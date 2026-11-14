"""Host build of the Arduino RC sketch (safety findings 1 and 5).

The no-pulse timeout and the valid range live in
``arduino_rc_reader/rc_channel.h`` with no Arduino calls, so a host C++
compiler can test them: ``tools/arduino_rc_reader_host_test/``. The second
test only checks that the sketch parses as C++ against stub headers; it does
not build it for the AVR (that needs the Arduino toolchain). Skipped when no
C++ compiler is installed.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SKETCH_DIR = ROOT / "arduino_rc_reader"
HOST_TEST_DIR = ROOT / "tools" / "arduino_rc_reader_host_test"
CXX = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")

pytestmark = pytest.mark.skipif(CXX is None, reason="no C++ compiler on this machine")


def _run(cmd, timeout):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def test_rc_channel_logic_host_test(tmp_path):
    exe = tmp_path / "test_rc_channel"
    build = _run(
        [
            CXX, "-std=gnu++11", "-Wall", "-Wextra",
            "-I", str(SKETCH_DIR),
            str(HOST_TEST_DIR / "test_rc_channel.cpp"),
            "-o", str(exe),
        ],
        timeout=120,
    )
    assert build.returncode == 0, build.stderr
    run = _run([str(exe)], timeout=60)
    assert run.returncode == 0, run.stderr + run.stdout
    assert "checks passed" in run.stdout


def test_sketch_parses_as_cpp_against_stubs():
    check = _run(
        [
            CXX, "-std=gnu++11", "-fsyntax-only", "-Wall", "-Wextra",
            "-I", str(HOST_TEST_DIR / "stubs"),
            "-I", str(SKETCH_DIR),
            "-include", "Arduino.h",
            "-x", "c++", str(SKETCH_DIR / "arduino_rc_reader.ino"),
        ],
        timeout=120,
    )
    assert check.returncode == 0, check.stderr
