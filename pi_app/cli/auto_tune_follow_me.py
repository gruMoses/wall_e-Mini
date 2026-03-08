"""
Auto-tune Follow Me lateral steering gains (Kp_steer, Kd_steer).

Uses a relay feedback test with Ziegler–Nichols PD rules — the same proven
approach used by auto_tune_pid.py for the IMU heading controller.

How it works
------------
1. The OAK-D camera tracks a person standing still in front of the rover.
2. A relay controller toggles the rover's steering left/right based on the
   sign of x_m (lateral offset of the person from camera center).
3. The system oscillates around x_m = 0 as the rover swings past the person.
4. From the oscillation amplitude (a) and period (Tu), the ultimate gain is
   computed:  Ku = 4·d / (π·a)  where d is the relay amplitude.
5. Ziegler–Nichols PD rules give:  Kp = 0.8·Ku,  Kd = 0.10·Ku·Tu

Setup
-----
- Stand 2-3 m in front of the rover (within Follow Me range) and stay still.
- ARM the rover via RC (ch3 high).
- The auto-tuner will oscillate the rover for ~8 seconds, then stop and print
  the recommended gains.

Usage
-----
    sudo systemctl stop wall-e.service
    python -m pi_app.cli.auto_tune_follow_me [--relay-amp 30] [--duration 8] [--apply]
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from config import config, FollowMeConfig
from pi_app.control.follow_me import PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE


def _wait_for_arm(rc_reader, high_us: int = 1800, debounce_s: float = 0.3) -> None:
    print(f"Waiting for ARM (ch3 >= {high_us} µs for {debounce_s:.1f}s) ...")
    high_since: Optional[float] = None
    while True:
        s = rc_reader.get_state()
        now = time.monotonic()
        if s.ch3_us >= high_us:
            if high_since is None:
                high_since = now
            if (now - high_since) >= debounce_s:
                print("ARM detected.")
                return
        else:
            high_since = None
        time.sleep(0.02)


def _wait_for_person(oak_reader, fm_cfg: FollowMeConfig, timeout_s: float = 30.0) -> Optional[PersonDetection]:
    """Block until a person is detected within Follow Me range."""
    print("Waiting for a person detection (stand 2-3 m in front of the rover) ...")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        persons = oak_reader.get_person_detections()
        for p in persons:
            if (p.confidence >= fm_cfg.detection_confidence
                    and fm_cfg.min_distance_m <= p.z_m <= fm_cfg.max_distance_m):
                print(f"  Person detected at z={p.z_m:.2f}m, x={p.x_m:.2f}m (conf={p.confidence:.0%})")
                return p
        time.sleep(0.1)
    return None


class FollowMeSteeringTuner:
    """Relay feedback auto-tuner for Follow Me lateral PD steering gains."""

    def __init__(
        self,
        oak_reader,
        motor_driver,
        fm_cfg: FollowMeConfig,
        relay_amplitude: int = 30,
        tuning_duration_s: float = 8.0,
        sample_rate_hz: float = 30.0,
        switch_deadband_m: float = 0.05,
        transient_discard_s: float = 1.5,
        log_csv_path: Optional[Path] = None,
    ) -> None:
        self._oak = oak_reader
        self._motor = motor_driver
        self._fm_cfg = fm_cfg
        self._relay_amp = int(relay_amplitude)
        self._duration_s = float(tuning_duration_s)
        self._sample_period_s = 1.0 / float(sample_rate_hz)
        self._deadband_m = float(switch_deadband_m)
        self._transient_s = float(transient_discard_s)
        self._csv_path = log_csv_path

        self._times: list[float] = []
        self._x_m_history: list[float] = []
        self._direction_history: list[int] = []
        self._duty_history: list[Tuple[int, int]] = []

    def _write_tracks(self, left: int, right: int) -> None:
        l = max(0, min(255, int(left)))
        r = max(0, min(255, int(right)))
        self._motor.set_tracks(l, r)

    def _get_x_m(self) -> Optional[float]:
        """Get latest lateral offset from best person detection."""
        persons = self._oak.get_person_detections()
        best: Optional[PersonDetection] = None
        best_score = -1.0
        for p in persons:
            if p.confidence < self._fm_cfg.detection_confidence:
                continue
            if p.z_m < self._fm_cfg.min_distance_m or p.z_m > self._fm_cfg.max_distance_m:
                continue
            cx = (p.bbox[0] + p.bbox[2]) / 2.0
            score = 1.0 - abs(cx - 0.5) * 2.0
            if score > best_score:
                best_score = score
                best = p
        return best.x_m if best is not None else None

    def run(self) -> Optional[Tuple[float, float]]:
        """Execute relay test and return (Kp_steer, Kd_steer) or None."""
        print(f"\n{'='*60}")
        print(f"  Follow Me Steering Auto-Tune")
        print(f"  Relay amplitude: ±{self._relay_amp} bytes")
        print(f"  Duration: {self._duration_s:.0f}s  |  Deadband: {self._deadband_m:.2f}m")
        print(f"{'='*60}\n")

        csv_fh = None
        csv_writer = None
        if self._csv_path is not None:
            try:
                csv_fh = open(self._csv_path, "w", newline="", encoding="utf-8")
                csv_writer = csv.writer(csv_fh)
                csv_writer.writerow(["t_s", "x_m", "direction", "left", "right"])
            except Exception:
                csv_fh = None

        direction = 1
        start = time.monotonic()
        next_tick = start
        end_time = start + self._duration_s
        missed_count = 0

        try:
            while time.monotonic() < end_time:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(max(0.0, next_tick - now))
                next_tick += self._sample_period_s

                x_m = self._get_x_m()
                t_rel = time.monotonic() - start

                if x_m is None:
                    missed_count += 1
                    if missed_count > 30:
                        print("ERROR: Lost person for >1s during relay test. Aborting.")
                        return None
                    continue
                missed_count = 0

                # Relay switching with hysteresis
                if abs(x_m) > self._deadband_m and (x_m > 0) != (direction > 0):
                    direction = -direction

                left = CENTER_OUTPUT_VALUE + direction * self._relay_amp
                right = CENTER_OUTPUT_VALUE - direction * self._relay_amp
                self._write_tracks(left, right)

                self._times.append(t_rel)
                self._x_m_history.append(x_m)
                self._direction_history.append(direction)
                self._duty_history.append((left, right))

                if csv_writer is not None:
                    csv_writer.writerow([f"{t_rel:.4f}", f"{x_m:.4f}", direction, left, right])

                # Live feedback every ~1s
                if len(self._times) % int(1.0 / self._sample_period_s) == 0:
                    print(f"  t={t_rel:5.1f}s  x_m={x_m:+.2f}  dir={'R' if direction > 0 else 'L'}  "
                          f"L={left} R={right}")

        finally:
            self._write_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
            try:
                self._motor.stop()
            except Exception:
                pass
            if csv_fh is not None:
                try:
                    csv_fh.flush()
                    csv_fh.close()
                except Exception:
                    pass

        return self._analyze()

    def _analyze(self) -> Optional[Tuple[float, float]]:
        if len(self._times) < 10:
            print("Not enough samples collected.")
            return None

        times = np.array(self._times)
        x_vals = np.array(self._x_m_history)

        # Discard initial transient
        mask = times >= self._transient_s
        if not np.any(mask):
            mask = slice(None)
        times = times[mask]
        x_vals = x_vals[mask]

        # Zero crossings → half-period estimates
        signs = np.sign(x_vals)
        signs[signs == 0] = 1
        crossings = np.where(np.diff(signs) != 0)[0]
        if crossings.size < 3:
            print("Insufficient oscillations detected. Try increasing relay amplitude or duration.")
            return None

        half_periods = np.diff(times[crossings])
        Tu = float(np.mean(half_periods) * 2.0)

        # Oscillation amplitude from half-cycle peak-to-peak
        seg_amps: list[float] = []
        for i in range(1, len(crossings)):
            seg = x_vals[crossings[i - 1]:crossings[i] + 1]
            if seg.size >= 2:
                amp = (float(np.max(seg)) - float(np.min(seg))) / 2.0
                if amp > 0.01:
                    seg_amps.append(amp)
        if not seg_amps:
            print("Failed to estimate oscillation amplitude.")
            return None
        osc_amp = float(np.median(seg_amps))

        # Relay describing function: Ku = 4·d / (π·a)
        # d_eff = relay amplitude in the same units as the controller output.
        # The controller's "scale" converts x_m to byte-offset:
        #   steer_bytes = x_m * Kp_steer * scale
        # so Ku is in the "Kp_steer" units (dimensionless gain that gets
        # multiplied by scale inside the controller).
        scale = self._fm_cfg.max_follow_speed_byte / max(self._fm_cfg.max_distance_m, 0.1)
        d_eff = float(self._relay_amp)
        Ku_bytes = 4.0 * d_eff / (math.pi * max(osc_amp, 0.01))
        Ku = Ku_bytes / scale

        # Ziegler–Nichols PD rules
        Kp = 0.8 * Ku
        Kd = 0.10 * Ku * Tu

        n_crossings = crossings.size
        print(f"\n{'='*60}")
        print(f"  RESULTS")
        print(f"{'='*60}")
        print(f"  Oscillation period  Tu = {Tu:.3f}s")
        print(f"  Oscillation amplitude a = {osc_amp:.3f}m")
        print(f"  Zero crossings         = {n_crossings}")
        print(f"  Ultimate gain       Ku = {Ku:.3f}")
        print(f"")
        print(f"  Ziegler-Nichols PD:")
        print(f"    steering_gain            = {Kp:.3f}")
        print(f"    steering_derivative_gain = {Kd:.3f}")
        print(f"{'='*60}\n")
        return (Kp, Kd)


def main() -> None:
    parser = argparse.ArgumentParser(description="Auto-tune Follow Me lateral steering gains")
    parser.add_argument("--relay-amp", type=int, default=30,
                        help="Relay amplitude in byte-offset from neutral (default: 30)")
    parser.add_argument("--duration", type=float, default=8.0,
                        help="Relay test duration in seconds (default: 8)")
    parser.add_argument("--deadband", type=float, default=0.05,
                        help="x_m deadband for relay switching in meters (default: 0.05)")
    parser.add_argument("--apply", action="store_true",
                        help="Write tuned values into config.py")
    args = parser.parse_args()

    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.hardware.vesc import VescCanDriver
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.oak_depth import OakDepthReader

    # RC reader for arming check
    rc_reader = ArduinoRCReader()
    try:
        port = rc_reader.start()
        print(f"Arduino RC on {port}")
    except Exception as e:
        print(f"Failed to start RC reader: {e}")
        return

    # Motor driver
    if VescCanDriver.detect():
        print("VESC detected")
        motor_driver = VescCanDriver()
    else:
        print("VESC not found; using Arduino motor driver")
        motor_driver = ArduinoModelXDriver(rc_reader=rc_reader)

    # OAK-D reader (needs obstacle + follow_me configs for pipeline setup)
    if not OakDepthReader.detect():
        print("ERROR: No OAK-D camera detected.")
        return
    oak_reader = OakDepthReader(
        obstacle_config=config.obstacle_avoidance,
        follow_me_config=config.follow_me,
    )
    print("Starting OAK-D pipeline ...")
    oak_reader.start()
    time.sleep(3.0)

    # Wait for ARM
    _wait_for_arm(rc_reader)

    # Wait for person
    person = _wait_for_person(oak_reader, config.follow_me, timeout_s=20.0)
    if person is None:
        print("ERROR: No person detected within timeout. Exiting.")
        oak_reader.stop()
        rc_reader.stop()
        return

    # Prepare log
    logs_dir = _PROJECT_ROOT / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    csv_path = logs_dir / f"auto_tune_follow_me_{int(time.time())}.csv"

    tuner = FollowMeSteeringTuner(
        oak_reader=oak_reader,
        motor_driver=motor_driver,
        fm_cfg=config.follow_me,
        relay_amplitude=args.relay_amp,
        tuning_duration_s=args.duration,
        switch_deadband_m=args.deadband,
        log_csv_path=csv_path,
    )

    try:
        result = tuner.run()
        if result is None:
            print("Auto-tune did not converge.")
            return

        Kp, Kd = result

        # Write results file
        out_path = logs_dir / "tuned_follow_me_steering.txt"
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(f"steering_gain = {Kp}\n")
            fh.write(f"steering_derivative_gain = {Kd}\n")
        print(f"Wrote results to {out_path}")
        print(f"CSV log at {csv_path}")

        if args.apply:
            _apply_to_config(Kp, Kd)

    finally:
        oak_reader.stop()
        try:
            rc_reader.stop()
        except Exception:
            pass


def _apply_to_config(kp: float, kd: float) -> None:
    """Patch config.py with the new steering gains."""
    cfg_path = _PROJECT_ROOT / "config.py"
    text = cfg_path.read_text(encoding="utf-8")
    import re

    new_text = text
    new_text = re.sub(
        r"(steering_gain:\s*float\s*=\s*)[0-9.]+",
        rf"\g<1>{kp:.3f}",
        new_text,
        count=1,
    )
    new_text = re.sub(
        r"(steering_derivative_gain:\s*float\s*=\s*)[0-9.]+",
        rf"\g<1>{kd:.3f}",
        new_text,
        count=1,
    )
    if new_text != text:
        cfg_path.write_text(new_text, encoding="utf-8")
        print(f"Applied to {cfg_path}:")
        print(f"  steering_gain = {kp:.3f}")
        print(f"  steering_derivative_gain = {kd:.3f}")
    else:
        print("WARNING: Could not find gain fields in config.py to patch.")


if __name__ == "__main__":
    main()
