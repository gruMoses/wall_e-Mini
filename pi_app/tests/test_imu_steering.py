import unittest
from pi_app.control.imu_steering import ImuSteeringCompensator, ImuSteeringConfig


class FakeImuReader:
    def __init__(self):
        self.fail = False

    def calibrate_gyro(self, duration_s):
        pass

    def calibrate_mag_hard_iron(self, duration_s):
        pass

    def read(self):
        if self.fail:
            raise RuntimeError("read failure")
        return {
            'heading_deg': 0.0,
            'gz_dps': 0.0,
            'roll_deg': 0.0,
            'pitch_deg': 0.0,
        }


class TestImuSteering(unittest.TestCase):
    def test_error_count_resets_after_success(self):
        imu_reader = FakeImuReader()
        config = ImuSteeringConfig()
        compensator = ImuSteeringCompensator(config, imu_reader)

        # Trigger an error during update
        imu_reader.fail = True
        compensator.update(0.0, 0.1)
        self.assertEqual(compensator.get_status().error_count, 1)

        # Successful read should reset the error counter
        imu_reader.fail = False
        compensator.update(0.0, 0.1)
        self.assertEqual(compensator.get_status().error_count, 0)


if __name__ == "__main__":
    unittest.main()
