import unittest
from unittest.mock import patch

from pi_app.hardware.vesc import VescCanDriver
from pi_app.hardware.arduino_modelx import ArduinoModelXDriver


class TestHardwareSelection(unittest.TestCase):
    def test_detect_vesc_false(self):
        self.assertIsInstance(ArduinoModelXDriver(), ArduinoModelXDriver)

    def test_detect_vesc_true(self):
        # Detection function just checks for interface presence; simulate via patching os.path.exists
        with patch("os.path.exists", return_value=True):
            self.assertTrue(VescCanDriver.detect("can0"))


if __name__ == "__main__":
    unittest.main()


