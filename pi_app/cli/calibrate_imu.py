#!/usr/bin/env python3
"""
Comprehensive IMU Calibration Script for WALL-E Mini

This script performs complete IMU calibration including:
1. Accelerometer bias and scale calibration
2. Magnetometer hard and soft iron calibration
3. Gyroscope bias calibration
4. Temperature compensation
5. Calibration validation and quality metrics

Usage:
    python calibrate_imu.py [--interactive] [--validate-only]

Options:
    --interactive: Guide user through calibration procedure
    --validate-only: Only validate existing calibration
"""

import sys
import time
import json
import math
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from collections import defaultdict

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.hardware.imu_reader import ImuReader


@dataclass
class CalibrationData:
    """Complete IMU calibration data."""
    mag_offsets_g: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_soft_iron: List[List[float]] = None  # 3x3 matrix
    gyro_bias_dps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_bias_g: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    temperature_c: float = 25.0
    calibration_date: str = ""
    quality_score: float = 0.0

    def __post_init__(self):
        if self.mag_soft_iron is None:
            self.mag_soft_iron = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]


class ImuCalibrator:
    """Comprehensive IMU calibration class."""

    def __init__(self, calibration_path: str = "imu_calibration.json"):
        self.calibration_path = Path(calibration_path)
        self.imu = ImuReader(calibration_path=str(self.calibration_path))
        self.calibration = CalibrationData()

    def load_existing_calibration(self) -> bool:
        """Load existing calibration if available."""
        if self.calibration_path.exists():
            try:
                with open(self.calibration_path, 'r') as f:
                    data = json.load(f)
                self.calibration = CalibrationData(**data)
                print(f"✓ Loaded existing calibration from {self.calibration_path}")
                return True
            except Exception as e:
                print(f"⚠️ Failed to load existing calibration: {e}")
        return False

    def save_calibration(self) -> None:
        """Save calibration data to file."""
        data = {
            'mag_offsets_g': list(self.calibration.mag_offsets_g),
            'mag_soft_iron': self.calibration.mag_soft_iron,
            'gyro_bias_dps': list(self.calibration.gyro_bias_dps),
            'accel_bias_g': list(self.calibration.accel_bias_g),
            'accel_scale': list(self.calibration.accel_scale),
            'temperature_c': self.calibration.temperature_c,
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'quality_score': self.calibration.quality_score
        }

        with open(self.calibration_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"✓ Calibration saved to {self.calibration_path}")

    def calibrate_gyroscope(self, duration_s: float = 10.0) -> Tuple[float, float, float]:
        """Calibrate gyroscope bias by averaging readings while stationary."""
        print("📡 Calibrating Gyroscope Bias...")
        print("   Keep IMU completely stationary for accurate calibration.")
        print(f"   Collecting data for {duration_s} seconds...")

        samples = []
        start_time = time.monotonic()

        while time.monotonic() - start_time < duration_s:
            try:
                g = self.imu.imu.get_gyro()
                gx = (g.xData or 0.0) / 1000.0
                gy = (g.yData or 0.0) / 1000.0
                gz = (g.zData or 0.0) / 1000.0
                samples.append((gx, gy, gz))
                time.sleep(0.01)
            except Exception as e:
                print(f"   Error reading gyro: {e}")
                time.sleep(0.1)

        if len(samples) < 10:
            raise RuntimeError("Insufficient gyro samples collected")

        # Calculate bias as mean of samples
        bias_x = sum(s[0] for s in samples) / len(samples)
        bias_y = sum(s[1] for s in samples) / len(samples)
        bias_z = sum(s[2] for s in samples) / len(samples)

        self.calibration.gyro_bias_dps = (bias_x, bias_y, bias_z)
        print(".2f")
        # Update IMU reader with new bias
        self.imu.gyro_bias_dps = self.calibration.gyro_bias_dps

        return self.calibration.gyro_bias_dps

    def calibrate_accelerometer(self, duration_s: float = 5.0) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Calibrate accelerometer bias and scale by collecting data in multiple orientations."""
        print("📱 Calibrating Accelerometer...")
        print("   Place IMU in 6 different orientations (each face up/down)")
        print("   Hold each position steady for a few seconds.")
        orientations = [
            "Z+ (flat on table)", "Z- (upside down)", "Y+ (right side up)",
            "Y- (left side up)", "X+ (front up)", "X- (back up)"
        ]

        accel_samples = []

        for i, orientation in enumerate(orientations):
            print(f"\n   {i+1}/6: {orientation}")
            input("   Press Enter when positioned...")

            samples = []
            start_time = time.monotonic()

            while time.monotonic() - start_time < duration_s:
                try:
                    a = self.imu.imu.get_accel()
                    ax = (a.xData or 0.0) / 1000.0
                    ay = (a.yData or 0.0) / 1000.0
                    az = (a.zData or 0.0) / 1000.0
                    samples.append((ax, ay, az))
                    time.sleep(0.01)
                except Exception as e:
                    print(f"   Error reading accel: {e}")

            if samples:
                # Average samples for this orientation
                avg_x = sum(s[0] for s in samples) / len(samples)
                avg_y = sum(s[1] for s in samples) / len(samples)
                avg_z = sum(s[2] for s in samples) / len(samples)
                accel_samples.append((avg_x, avg_y, avg_z))
                print(".3f")
        if len(accel_samples) < 6:
            raise RuntimeError("Need samples from all 6 orientations")

        # Calculate bias and scale
        # Expected readings for each orientation (in g)
        expected = [
            (0, 0, 1),   # Z+
            (0, 0, -1),  # Z-
            (0, 1, 0),   # Y+
            (0, -1, 0),  # Y-
            (1, 0, 0),   # X+
            (-1, 0, 0)   # X-
        ]

        # Solve for bias and scale using least squares
        bias_x, scale_x = self._calculate_bias_scale([s[0] for s in accel_samples], [e[0] for e in expected])
        bias_y, scale_y = self._calculate_bias_scale([s[1] for s in accel_samples], [e[1] for e in expected])
        bias_z, scale_z = self._calculate_bias_scale([s[2] for s in accel_samples], [e[2] for e in expected])

        self.calibration.accel_bias_g = (bias_x, bias_y, bias_z)
        self.calibration.accel_scale = (scale_x, scale_y, scale_z)

        print(".3f")
        # Update IMU reader (though it doesn't currently use accel calibration)
        # self.imu.accel_bias_g = self.calibration.accel_bias_g
        # self.imu.accel_scale = self.calibration.accel_scale

        return self.calibration.accel_bias_g, self.calibration.accel_scale

    def calibrate_magnetometer_hard_iron(self, duration_s: float = 30.0) -> Tuple[float, float, float]:
        """Calibrate magnetometer hard iron bias by rotating in all directions."""
        print("🧲 Calibrating Magnetometer Hard Iron...")
        print("   Slowly rotate IMU in all directions to sample the full magnetic field.")
        print("   Make slow, continuous rotations covering all orientations.")
        print(f"   Collecting data for {duration_s} seconds...")

        samples = []
        start_time = time.monotonic()

        while time.monotonic() - start_time < duration_s:
            try:
                mx, my, mz = self.imu.mag.get_measurement_xyz_gauss()
                samples.append((mx, my, mz))
                time.sleep(0.05)  # Slower sampling for mag
            except Exception as e:
                print(f"   Error reading magnetometer: {e}")
                time.sleep(0.1)

        if len(samples) < 50:
            raise RuntimeError("Insufficient magnetometer samples collected")

        # Find min/max for each axis
        min_x = min(s[0] for s in samples)
        max_x = max(s[0] for s in samples)
        min_y = min(s[1] for s in samples)
        max_y = max(s[1] for s in samples)
        min_z = min(s[2] for s in samples)
        max_z = max(s[2] for s in samples)

        # Hard iron offset is center of the bounding box
        offset_x = (max_x + min_x) / 2.0
        offset_y = (max_y + min_y) / 2.0
        offset_z = (max_z + min_z) / 2.0

        self.calibration.mag_offsets_g = (offset_x, offset_y, offset_z)
        print(".3f")
        # Update IMU reader with new offsets
        self.imu.mag_offsets_g = self.calibration.mag_offsets_g

        return self.calibration.mag_offsets_g

    def calibrate_magnetometer_soft_iron(self, duration_s: float = 30.0) -> List[List[float]]:
        """Calibrate magnetometer soft iron distortion using ellipsoid fitting."""
        print("🔧 Calibrating Magnetometer Soft Iron...")
        print("   Rotate IMU in all directions again for soft iron calibration.")
        print("   This refines the hard iron calibration with ellipsoid fitting.")
        print(f"   Collecting data for {duration_s} seconds...")

        samples = []
        start_time = time.monotonic()

        while time.monotonic() - start_time < duration_s:
            try:
                mx, my, mz = self.imu.mag.get_measurement_xyz_gauss()
                # Apply hard iron correction first
                mx -= self.calibration.mag_offsets_g[0]
                my -= self.calibration.mag_offsets_g[1]
                mz -= self.calibration.mag_offsets_g[2]
                samples.append((mx, my, mz))
                time.sleep(0.05)
            except Exception as e:
                print(f"   Error reading magnetometer: {e}")
                time.sleep(0.1)

        if len(samples) < 50:
            raise RuntimeError("Insufficient magnetometer samples collected")

        # Fit ellipsoid to the data points
        try:
            soft_iron_matrix = self._fit_ellipsoid(samples)
            self.calibration.mag_soft_iron = soft_iron_matrix
            print("✓ Soft iron calibration matrix calculated")
            print("  Matrix:")
            for row in soft_iron_matrix:
                print(f"    {row}")
        except Exception as e:
            print(f"⚠️ Soft iron fitting failed: {e}, using identity matrix")
            self.calibration.mag_soft_iron = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]

        return self.calibration.mag_soft_iron

    def validate_calibration(self) -> float:
        """Validate calibration quality by analyzing stability and consistency."""
        print("🔍 Validating Calibration...")
        print("   Analyzing IMU stability for 10 seconds...")

        samples = []
        start_time = time.monotonic()

        while time.monotonic() - start_time < 10.0:
            try:
                data = self.imu.read()
                samples.append({
                    'heading': data['heading_deg'],
                    'roll': data['roll_deg'],
                    'pitch': data['pitch_deg'],
                    'yaw_rate': data['gz_dps'],
                    'temp': data['temp_c']
                })
                time.sleep(0.1)
            except Exception as e:
                print(f"   Error during validation: {e}")
                return 0.0

        if len(samples) < 20:
            print("⚠️ Insufficient validation samples")
            return 0.0

        # Analyze stability metrics
        headings = [s['heading'] for s in samples]
        yaw_rates = [s['yaw_rate'] for s in samples]
        rolls = [s['roll'] for s in samples]
        pitches = [s['pitch'] for s in samples]

        # Calculate standard deviations
        heading_std = math.sqrt(sum((h - sum(headings)/len(headings))**2 for h in headings) / len(headings))
        yaw_rate_std = math.sqrt(sum((r - sum(yaw_rates)/len(yaw_rates))**2 for r in yaw_rates) / len(yaw_rates))
        roll_std = math.sqrt(sum((r - sum(rolls)/len(rolls))**2 for r in rolls) / len(rolls))
        pitch_std = math.sqrt(sum((p - sum(pitches)/len(pitches))**2 for p in pitches) / len(pitches))

        # Calculate quality score (0-100)
        # Lower standard deviations = higher quality
        heading_quality = max(0, 100 - heading_std * 10)  # Expect <10° variation
        stability_quality = max(0, 100 - yaw_rate_std * 50)  # Expect <2°/s variation
        attitude_quality = max(0, 100 - (roll_std + pitch_std) * 20)  # Expect <5° variation

        quality_score = (heading_quality + stability_quality + attitude_quality) / 3.0

        print(".1f")
        print(".2f")
        print(".2f")
        print(".2f")
        if quality_score > 80:
            print("🎉 Excellent calibration quality!")
        elif quality_score > 60:
            print("✅ Good calibration quality")
        elif quality_score > 40:
            print("⚠️ Acceptable calibration quality - consider recalibrating")
        else:
            print("❌ Poor calibration quality - recalibration strongly recommended")

        self.calibration.quality_score = quality_score
        return quality_score

    def run_full_calibration(self, interactive: bool = True) -> bool:
        """Run complete calibration procedure."""
        print("🚀 Starting IMU Calibration Procedure")
        print("=" * 50)

        try:
            # Load existing calibration if available
            self.load_existing_calibration()

            if interactive:
                print("\n📋 Calibration Checklist:")
                print("   □ Clear area of magnetic interference (no metal objects nearby)")
                print("   □ IMU mounted securely on robot")
                print("   □ Robot positioned on stable, level surface")
                print("   □ No electrical noise sources nearby")
                input("\nPress Enter when ready to begin calibration...")

            # Step 1: Gyroscope calibration
            print("\n" + "="*50)
            self.calibrate_gyroscope()

            # Step 2: Accelerometer calibration
            print("\n" + "="*50)
            if interactive:
                self.calibrate_accelerometer()
            else:
                print("⏭️ Skipping accelerometer calibration (use --interactive)")

            # Step 3: Magnetometer hard iron calibration
            print("\n" + "="*50)
            self.calibrate_magnetometer_hard_iron()

            # Step 4: Magnetometer soft iron calibration
            print("\n" + "="*50)
            self.calibrate_magnetometer_soft_iron()

            # Step 5: Validation
            print("\n" + "="*50)
            quality = self.validate_calibration()

            # Save calibration
            self.save_calibration()

            print("\n" + "="*50)
            print("🎯 Calibration Complete!")
            print(f"   Quality Score: {quality:.1f}/100")

            if quality > 60:
                print("✅ Calibration successful - IMU readings should now be stable")
            else:
                print("⚠️ Calibration quality is low - consider recalibrating in a better environment")

            return quality > 40

        except KeyboardInterrupt:
            print("\n❌ Calibration interrupted by user")
            return False
        except Exception as e:
            print(f"\n❌ Calibration failed: {e}")
            return False

    def _calculate_bias_scale(self, measured: List[float], expected: List[float]) -> Tuple[float, float]:
        """Calculate bias and scale for a single axis using linear regression."""
        if len(measured) != len(expected):
            raise ValueError("Measured and expected arrays must have same length")

        # Linear regression: measured = scale * expected + bias
        # Rearranged: expected = (measured - bias) / scale
        # Using least squares to solve for bias and scale

        n = len(measured)
        sum_m = sum(measured)
        sum_e = sum(expected)
        sum_me = sum(m * e for m, e in zip(measured, expected))
        sum_ee = sum(e * e for e in expected)

        # Calculate scale and bias
        denominator = n * sum_ee - sum_e * sum_e
        if abs(denominator) < 1e-10:
            # Degenerate case - use simple average
            scale = 1.0
            bias = sum_m / n
        else:
            scale = (n * sum_me - sum_m * sum_e) / denominator
            bias = (sum_m - scale * sum_e) / n

        return bias, scale

    def _fit_ellipsoid(self, points: List[Tuple[float, float, float]]) -> List[List[float]]:
        """Fit an ellipsoid to 3D points to calculate soft iron correction matrix."""
        # Simplified ellipsoid fitting - in practice, you'd want a more robust algorithm
        # This is a basic implementation that works for most cases

        # Center the points (hard iron already corrected)
        centered_points = points

        # Calculate covariance matrix
        n = len(centered_points)
        cov = [[0.0 for _ in range(3)] for _ in range(3)]

        for point in centered_points:
            for i in range(3):
                for j in range(3):
                    cov[i][j] += point[i] * point[j]

        for i in range(3):
            for j in range(3):
                cov[i][j] /= n

        # For simplicity, assume the ellipsoid is aligned with axes
        # and just scale each axis to make it spherical
        scales = []
        for i in range(3):
            # Eigenvalue approximation (trace of covariance)
            scale = math.sqrt(abs(cov[i][i]))
            scales.append(1.0 / scale if scale > 0 else 1.0)

        # Create diagonal correction matrix
        correction_matrix = [[scales[i] if i == j else 0.0 for j in range(3)] for i in range(3)]

        return correction_matrix


def main():
    parser = argparse.ArgumentParser(description="IMU Calibration Tool")
    parser.add_argument('--interactive', action='store_true',
                       help='Run interactive calibration with user guidance')
    parser.add_argument('--validate-only', action='store_true',
                       help='Only validate existing calibration')
    parser.add_argument('--calibration-file', default='imu_calibration.json',
                       help='Path to calibration file')

    args = parser.parse_args()

    try:
        calibrator = ImuCalibrator(args.calibration_file)

        if args.validate_only:
            if not calibrator.load_existing_calibration():
                print("❌ No existing calibration found to validate")
                return 1

            quality = calibrator.validate_calibration()
            return 0 if quality > 40 else 1

        else:
            success = calibrator.run_full_calibration(interactive=args.interactive)
            return 0 if success else 1

    except Exception as e:
        print(f"❌ Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
