import time
import math
import csv
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

try:
    # Allow running as a script or module
    from pi_app.hardware.imu_reader import ImuReader
    from pi_app.control.imu_steering import ImuSteeringCompensator
    from pi_app.hardware.vesc import VescCanDriver
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from config import config
except Exception:
    import sys
    from pathlib import Path as _Path
    sys.path.append(str(_Path(__file__).resolve().parents[2]))
    from pi_app.hardware.imu_reader import ImuReader  # type: ignore
    from pi_app.control.imu_steering import ImuSteeringCompensator  # type: ignore
    from pi_app.hardware.vesc import VescCanDriver  # type: ignore
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver  # type: ignore
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from config import config  # type: ignore


def wrap_deg180(angle_deg: float) -> float:
    e = (angle_deg + 180.0) % 360.0 - 180.0
    return e


def wait_for_arm(rc_reader: ArduinoRCReader, high_threshold_us: int = 1800, debounce_s: float = 0.3) -> None:
    print(f"Waiting for ARM (ch3 >= {high_threshold_us} µs for {debounce_s:.1f}s)...")
    high_since: Optional[float] = None
    while True:
        s = rc_reader.get_state()
        now = time.monotonic()
        if s.ch3_us >= high_threshold_us:
            if high_since is None:
                high_since = now
            if (now - high_since) >= debounce_s:
                print("ARM detected. Starting auto-tune.")
                return
        else:
            high_since = None
        time.sleep(0.02)


class PIDAutoTuner:
    def __init__(
        self,
        imu_reader: ImuReader,
        motor_driver,
        steering_config,
        base_duty: int = 140,
        relay_amplitude: int = 60,
        tuning_duration_s: float = 6.0,
        sample_rate_hz: float = 80.0,
        switch_deadband_deg: float = 2.0,
        transient_discard_s: float = 1.0,
        log_csv_path: Optional[Path] = None,
    ) -> None:
        self.imu_reader = imu_reader
        self.motor_driver = motor_driver
        self.config = steering_config
        # Constructing compensator performs IMU calibration and initial read
        self.compensator = ImuSteeringCompensator(steering_config, imu_reader)
        self.base_duty = int(base_duty)
        self.relay_amplitude = int(relay_amplitude)
        self.tuning_duration_s = float(tuning_duration_s)
        self.sample_period_s = 1.0 / float(sample_rate_hz)
        self.switch_deadband_deg = float(switch_deadband_deg)
        self.transient_discard_s = float(transient_discard_s)
        self.heading_history: list[float] = []
        self.error_history: list[float] = []
        self.time_history: list[float] = []
        self.direction_history: list[int] = []
        self.duty_history: list[Tuple[int, int]] = []
        self.log_csv_path = log_csv_path

    def _write_tracks(self, left: int, right: int) -> None:
        l = max(0, min(255, int(left)))
        r = max(0, min(255, int(right)))
        self.motor_driver.set_tracks(l, r)

    def start_tuning(self) -> Optional[Tuple[float, float, float]]:
        # Initial heading as target
        data0 = self.imu_reader.read()
        target_heading = float(data0['heading_deg'])

        # Brief stabilization at base
        self._write_tracks(self.base_duty, self.base_duty)
        time.sleep(0.5)

        direction = 1  # toggled by relay logic
        start = time.monotonic()
        next_tick = start
        end_time = start + self.tuning_duration_s

        csv_fh = None
        csv_writer = None
        if self.log_csv_path is not None:
            try:
                csv_fh = open(self.log_csv_path, "w", newline="", encoding="utf-8")
                csv_writer = csv.writer(csv_fh)
                csv_writer.writerow(["t_s", "heading_deg", "error_deg", "direction", "left", "right"])
            except Exception:
                csv_fh = None
                csv_writer = None

        try:
            while time.monotonic() < end_time:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(max(0.0, next_tick - now))
                next_tick += self.sample_period_s

                data = self.imu_reader.read()
                heading = float(data['heading_deg'])
                error = wrap_deg180(target_heading - heading)
                t_rel = now - start

                # Relay switching with hysteresis
                if abs(error) > self.switch_deadband_deg and (error > 0) != (direction > 0):
                    direction = -direction

                left_duty = self.base_duty + direction * self.relay_amplitude
                right_duty = self.base_duty - direction * self.relay_amplitude
                self._write_tracks(left_duty, right_duty)

                # Log histories
                self.heading_history.append(heading)
                self.error_history.append(error)
                self.time_history.append(t_rel)
                self.direction_history.append(direction)
                self.duty_history.append((left_duty, right_duty))
                if csv_writer is not None:
                    csv_writer.writerow([f"{t_rel:.4f}", f"{heading:.4f}", f"{error:.4f}", direction, left_duty, right_duty])
        finally:
            # Always stop motors
            try:
                self._write_tracks(126, 126)
                self.motor_driver.stop()
            except Exception:
                pass
            if csv_fh is not None:
                try:
                    csv_fh.flush()
                    csv_fh.close()
                except Exception:
                    pass

        # Analyze oscillations after discarding transient
        return self._analyze_and_compute_gains(target_heading)

    def _analyze_and_compute_gains(self, target_heading: float) -> Optional[Tuple[float, float, float]]:
        if not self.time_history:
            print("No samples collected.")
            return None

        times = np.array(self.time_history)
        errors = np.array([wrap_deg180(target_heading - h) for h in self.heading_history])

        # Discard beginning transient
        mask = times >= self.transient_discard_s
        if not np.any(mask):
            mask = slice(None)
        times = times[mask]
        errors = errors[mask]

        # Zero crossings for half-periods
        signs = np.sign(errors)
        signs[signs == 0] = 1
        crossings = np.where(np.diff(signs) != 0)[0]
        if crossings.size < 3:
            print("Insufficient oscillations. Increase relay_amplitude or duration.")
            return None

        # Estimate period Tu: 2x mean half-period
        half_periods = np.diff(times[crossings])
        Tu = float(np.mean(half_periods) * 2.0)

        # Estimate oscillation amplitude using peak-to-peak within mid segments
        segment_amplitudes = []
        for i in range(1, len(crossings)):
            s = crossings[i - 1]
            e = crossings[i] + 1
            seg = errors[s:e]
            if seg.size >= 2:
                amp = (float(np.max(seg)) - float(np.min(seg))) / 2.0
                if amp > 0:
                    segment_amplitudes.append(amp)
        if not segment_amplitudes:
            print("Failed to estimate oscillation amplitude.")
            return None
        osc_amp = float(np.median(segment_amplitudes))

        # Relay describing function: Ku = 4*d / (pi*a)
        # Here the steering input toggles left/right difference by ±(2*relay_amplitude)
        d_eff = 2.0 * float(self.relay_amplitude)
        Ku = 4.0 * d_eff / (math.pi * max(osc_amp, 0.1))

        # Ziegler–Nichols for PID
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / max(Tu, 1e-3)
        Kd = 0.075 * Ku * Tu

        print(f"Tuned PID values: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f} (Tu={Tu:.2f}s, a={osc_amp:.2f}°)")
        return (Kp, Ki, Kd)


def main() -> None:
    # Prepare output directory for CSV logs
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    try:
        logs_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    csv_path = logs_dir / f"auto_tune_{int(time.time())}.csv"

    # Initialize IMU
    imu = ImuReader(calibration_path=config.imu_calibration_path,
                    use_magnetometer=config.imu_use_magnetometer)

    # Always start Arduino RC reader to monitor arming (ch3) [[memory:3361651]]
    rc_reader: Optional[ArduinoRCReader] = ArduinoRCReader()
    try:
        port = rc_reader.start()
        print(f"Arduino RC detected on {port}")
    except Exception as e:
        print(f"Failed to start Arduino RC reader: {e}")
        return

    # Choose motor driver
    motor_driver = None
    if VescCanDriver.detect():
        print("VESC over CAN detected; using VESC driver")
        motor_driver = VescCanDriver()
    else:
        print("VESC not detected; using Arduino Model X motor driver (via Arduino)")
        motor_driver = ArduinoModelXDriver(rc_reader=rc_reader)

    # Wait for ARMED on ch3 before starting motion
    wait_for_arm(rc_reader)

    tuner = PIDAutoTuner(
        imu_reader=imu,
        motor_driver=motor_driver,
        steering_config=config.imu_steering,
        base_duty=140,
        relay_amplitude=60,
        tuning_duration_s=15.0,
        sample_rate_hz=80.0,
        switch_deadband_deg=2.0,
        transient_discard_s=1.0,
        log_csv_path=csv_path,
    )
    try:
        values = tuner.start_tuning()
        if values is None:
            print("Auto-tuning did not converge; no values written.")
            return
        Kp, Ki, Kd = values
        out_path = logs_dir / "tuned_pid.txt"
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(f"Kp = {Kp}\nKi = {Ki}\nKd = {Kd}\n")
        print(f"Wrote tuned values to {out_path}")
    finally:
        try:
            if rc_reader is not None:
                rc_reader.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()


