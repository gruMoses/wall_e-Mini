"""IMU report choice across the depthai 3.6.1 upgrade (2026-11-05).

depthai 3.6.1 changed what the IMU RAW reports mean: before, RAW was
pre-rotated into the Luxonis RDF frame; from 3.6.1 RAW is the sensor's own
frame and UNCALIBRATED is the old RDF behaviour (v3.6.1 release notes). The
yaw channel (gyro_y, +Y down) and the boot gravity check assume RDF, so an
upgrade that kept requesting RAW could move or flip the yaw axis silently,
the same failure as the pre-2026-09-19 sign bug. 3.3.0 has no
ACCELEROMETER_UNCALIBRATED, so older versions must keep RAW.
"""

from __future__ import annotations

import os
import unittest

from pi_app.hardware import oak_depth
from pi_app.hardware.oak_depth import _depthai_version_tuple, imu_report_names

RAW = ("ACCELEROMETER_RAW", "GYROSCOPE_RAW")
UNCAL = ("ACCELEROMETER_UNCALIBRATED", "GYROSCOPE_UNCALIBRATED")


class TestDepthaiVersionTuple(unittest.TestCase):
    def test_parses_release_versions(self):
        self.assertEqual(_depthai_version_tuple("3.3.0"), (3, 3, 0))
        self.assertEqual(_depthai_version_tuple("3.12.0"), (3, 12, 0))

    def test_ignores_local_and_dev_suffixes(self):
        self.assertEqual(_depthai_version_tuple("3.6.1+abc123"), (3, 6, 1))
        self.assertEqual(_depthai_version_tuple("3.7.0.dev0"), (3, 7, 0))

    def test_unparseable_is_none(self):
        self.assertIsNone(_depthai_version_tuple(""))
        self.assertIsNone(_depthai_version_tuple("unknown"))
        self.assertIsNone(_depthai_version_tuple(None))


class TestImuReportNames(unittest.TestCase):
    def test_versions_before_3_6_1_keep_raw(self):
        for version in ("3.3.0", "3.5.0", "3.6.0", "2.30.0.0"):
            with self.subTest(version=version):
                self.assertEqual(imu_report_names(version), RAW)

    def test_3_6_1_and_later_use_uncalibrated(self):
        for version in ("3.6.1", "3.7.1", "3.10.0", "3.12.0", "4.0.0"):
            with self.subTest(version=version):
                self.assertEqual(imu_report_names(version), UNCAL)

    def test_unparseable_version_falls_back_to_the_enum(self):
        class OldEnum:  # 3.3.0: no ACCELEROMETER_UNCALIBRATED
            ACCELEROMETER_RAW = GYROSCOPE_RAW = GYROSCOPE_UNCALIBRATED = object()

        class NewEnum(OldEnum):  # 3.6.1+
            ACCELEROMETER_UNCALIBRATED = object()

        for version in ("", "v3.6.1", "3.6", "unknown", None):
            with self.subTest(version=version):
                self.assertEqual(imu_report_names(version, NewEnum), UNCAL)
                self.assertEqual(imu_report_names(version, OldEnum), RAW)
                self.assertEqual(imu_report_names(version), RAW)

    def test_a_parsed_version_wins_over_the_enum(self):
        class NewEnum:
            ACCELEROMETER_UNCALIBRATED = object()

        # 3.6.0 had no RAW change, whatever its enum holds.
        self.assertEqual(imu_report_names("3.6.0", NewEnum), RAW)
        self.assertEqual(imu_report_names("3.6.1.dev0", NewEnum), UNCAL)

    def test_names_exist_in_the_depthai_enum_when_installed(self):
        try:
            import depthai as dai
        except ImportError:
            self.skipTest("depthai is not installed here")
        # Other tests install a stand-in types.ModuleType("depthai"); only the
        # real wheel (a file on disk with a version) can answer this.
        if not getattr(dai, "__file__", None) or not hasattr(dai, "__version__"):
            self.skipTest("a test stand-in replaced depthai")
        for name in imu_report_names(getattr(dai, "__version__", ""), dai.IMUSensor):
            self.assertTrue(hasattr(dai.IMUSensor, name), name)


class TestAutocalibrationDefault(unittest.TestCase):
    def test_module_import_turns_autocalibration_off_by_default(self):
        # oak_depth sets it with setdefault at import; an explicit value wins.
        self.assertIn("DEPTHAI_AUTOCALIBRATION", os.environ)
        self.assertTrue(hasattr(oak_depth, "imu_report_names"))
        if os.environ.get("WALLE_TEST_EXPLICIT_AUTOCAL") is None:
            self.assertEqual(os.environ["DEPTHAI_AUTOCALIBRATION"], "OFF")


if __name__ == "__main__":
    unittest.main()
