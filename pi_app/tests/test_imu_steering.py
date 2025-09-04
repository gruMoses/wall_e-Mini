import unittest
import itertools
import time
from threading import Thread
from unittest.mock import patch

from pi_app.control.imu_steering import ImuSteeringCompensator, ImuSteeringConfig


class FakeImuReader:
    def __init__(self, headings=None, delay: float = 0.0):
        self.headings = iter(headings) if headings is not None else itertools.repeat(0.0)
        self.fail = False
        self.delay = delay
        self.use_mag = True

    def calibrate_gyro(self, duration_s):
        pass

    def calibrate_mag_hard_iron(self, duration_s):
        pass

    def read(self):
        if self.fail:
            raise RuntimeError("read failure")
        if self.delay:
            time.sleep(self.delay)
        try:
            heading = next(self.headings)
        except StopIteration:
            heading = 0.0
        return {
            "heading_deg": heading,
            "gz_dps": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
        }


class TestImuSteering(unittest.TestCase):

    def test_neutral_dwell_updates_target_heading(self):
        headings = [0.0, 5.0, 10.0, 20.0]
        imu_reader = FakeImuReader(headings=headings)
        cfg = ImuSteeringConfig(neutral_dwell_s=0.5)
        comp = ImuSteeringCompensator(cfg, imu_reader)

        # Leave neutral state
        comp.update(0.2, 0.1)

        times = [1.0, 1.0, 1.0, 1.6, 1.6]
        with patch("pi_app.control.imu_steering.time.monotonic", side_effect=times):
            # Enter neutral and start dwell
            comp.update(0.0, 0.1)
            self.assertEqual(comp.get_status().target_heading_deg, 0.0)

            # After dwell expires, target heading should update
            comp.update(0.0, 0.1)
            self.assertAlmostEqual(comp.get_status().target_heading_deg, 20.0)

    def test_error_count_resets_after_success(self):
        imu_reader = FakeImuReader()
        cfg = ImuSteeringConfig()
        comp = ImuSteeringCompensator(cfg, imu_reader)

        imu_reader.fail = True
        comp.update(0.0, 0.1)
        self.assertEqual(comp.get_status().error_count, 1)

        imu_reader.fail = False
        comp.update(0.0, 0.1)
        self.assertEqual(comp.get_status().error_count, 0)

    def test_update_is_thread_safe(self):
        imu_reader = FakeImuReader(delay=0.001)
        cfg = ImuSteeringConfig()
        comp = ImuSteeringCompensator(cfg, imu_reader)

        errors = []

        def worker():
            for _ in range(5):
                try:
                    comp.update(0.0, 0.1)
                except Exception as e:
                    errors.append(e)

        threads = [Thread(target=worker) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(errors, [])
        self.assertEqual(comp.get_status().error_count, 0)


if __name__ == "__main__":
    unittest.main()

