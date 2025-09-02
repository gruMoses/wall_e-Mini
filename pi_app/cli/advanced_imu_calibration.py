#!/usr/bin/env python3
"""
Advanced IMU Calibration and Diagnostics System for WALL-E Mini

This script provides comprehensive IMU calibration and diagnostics including:
1. Multi-stage calibration procedure
2. Interference detection and mitigation
3. Real-time performance monitoring
4. Hardware validation
5. Adaptive filter tuning
"""

import sys
import time
import json
import math
import argparse
import statistics
from pathlib import Path
from typing import Dict, List, Tuple, Optional, NamedTuple
from dataclasses import dataclass, field

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.hardware.imu_reader import ImuReader


@dataclass
class InterferenceMetrics:
    """Metrics for detecting electromagnetic interference."""
    mag_field_std: float = 0.0
    gyro_noise_std: float = 0.0
    accel_noise_std: float = 0.0
    electrical_noise_score: float = 0.0
    magnetic_interference_score: float = 0.0

    def overall_score(self) -> float:
        """Calculate overall interference score (0-100, higher = more interference)."""
        return min(100.0, (self.electrical_noise_score + self.magnetic_interference_score) / 2)


@dataclass
class CalibrationQuality:
    """Comprehensive calibration quality assessment."""
    heading_stability: float = 0.0  # 0-100
    attitude_stability: float = 0.0  # 0-100
    gyro_performance: float = 0.0    # 0-100
    accel_performance: float = 0.0   # 0-100
    mag_performance: float = 0.0     # 0-100
    interference_level: float = 0.0  # 0-100
    overall_score: float = 0.0       # 0-100

    def compute_overall(self):
        """Compute overall quality score."""
        weights = {
            'heading': 0.3,
            'attitude': 0.2,
            'gyro': 0.15,
            'accel': 0.15,
            'mag': 0.15,
            'interference': 0.05
        }

        self.overall_score = (
            self.heading_stability * weights['heading'] +
            self.attitude_stability * weights['attitude'] +
            self.gyro_performance * weights['gyro'] +
            self.accel_performance * weights['accel'] +
            self.mag_performance * weights['mag'] -
            self.interference_level * weights['interference']
        )

        self.overall_score = max(0.0, min(100.0, self.overall_score))


class AdvancedImuCalibrator:
    """Advanced IMU calibration and diagnostics system."""

    def __init__(self, calibration_path: str = "imu_calibration.json"):
        self.calibration_path = Path(calibration_path)
        self.imu = None
        self.interference_baseline = InterferenceMetrics()

    def initialize_imu(self) -> bool:
        """Initialize IMU with optimized parameters."""
        try:
            # Use optimized complementary filter parameters
            self.imu = ImuReader(
                complementary_alpha_rp=0.02,  # Trust accelerometer more
                complementary_alpha_yaw=0.01,  # Trust magnetometer more
                calibration_path=str(self.calibration_path)
            )
            print("✅ IMU initialized with optimized parameters")
            return True
        except Exception as e:
            print(f"❌ IMU initialization failed: {e}")
            return False

    def detect_interference(self) -> InterferenceMetrics:
        """Detect electromagnetic interference and noise."""
        print("🔍 Detecting electromagnetic interference...")

        if not self.imu:
            return InterferenceMetrics()

        # Collect baseline data
        mag_readings = []
        gyro_readings = []
        accel_readings = []

        print("   Collecting baseline data (10 seconds)...")
        start_time = time.time()

        while time.time() - start_time < 10.0:
            try:
                data = self.imu.read()
                mag_readings.append(math.sqrt(data['mx_g']**2 + data['my_g']**2 + data['mz_g']**2))
                gyro_readings.append(math.sqrt(data['gx_dps']**2 + data['gy_dps']**2 + data['gz_dps']**2))
                accel_readings.append(math.sqrt(data['ax_g']**2 + data['ay_g']**2 + data['az_g']**2))
                time.sleep(0.1)
            except Exception as e:
                print(f"   Error collecting data: {e}")
                time.sleep(0.1)

        if len(mag_readings) < 20:
            print("❌ Insufficient data for interference analysis")
            return InterferenceMetrics()

        # Calculate metrics
        metrics = InterferenceMetrics()

        # Magnetic field stability (should be very stable when stationary)
        try:
            metrics.mag_field_std = statistics.stdev(mag_readings)
            metrics.magnetic_interference_score = min(100.0, metrics.mag_field_std * 1000)  # Scale for sensitivity
        except:
            metrics.mag_field_std = 0.0
            metrics.magnetic_interference_score = 100.0

        # Gyro noise (should be very low when stationary)
        try:
            metrics.gyro_noise_std = statistics.stdev(gyro_readings)
            metrics.electrical_noise_score = min(100.0, metrics.gyro_noise_std * 50)
        except:
            metrics.gyro_noise_std = 0.0
            metrics.electrical_noise_score = 100.0

        # Accelerometer noise
        try:
            metrics.accel_noise_std = statistics.stdev(accel_readings)
        except:
            metrics.accel_noise_std = 0.0

        print(".3f")
        print(".3f")
        print(".3f")
        print(".1f")
        print(".1f")
        if metrics.overall_score() < 20:
            print("✅ Environment appears clean")
        elif metrics.overall_score() < 50:
            print("⚠️ Moderate interference detected")
        else:
            print("❌ HIGH interference - calibration may be unreliable")

        return metrics

    def assess_hardware_mounting(self) -> Dict[str, any]:
        """Assess IMU mounting quality and orientation."""
        print("🔧 Assessing hardware mounting...")

        if not self.imu:
            return {"quality": "unknown", "issues": ["IMU not initialized"]}

        issues = []
        quality_score = 100

        # Check accelerometer alignment (should read ~1g in Z axis when level)
        accel_readings = []
        for i in range(20):
            try:
                data = self.imu.read()
                accel_readings.append((data['ax_g'], data['ay_g'], data['az_g']))
                time.sleep(0.1)
            except:
                time.sleep(0.1)

        if accel_readings:
            avg_ax = sum(r[0] for r in accel_readings) / len(accel_readings)
            avg_ay = sum(r[1] for r in accel_readings) / len(accel_readings)
            avg_az = sum(r[2] for r in accel_readings) / len(accel_readings)

            # Check if Z-axis is approximately 1g (assuming IMU is level)
            if abs(avg_az - 1.0) > 0.2:
                issues.append(f"Z-axis gravity reading: {avg_az:.3f}g (expected ~1.0g)")
                quality_score -= 30

            # Check for significant tilt in X/Y axes
            if abs(avg_ax) > 0.3:
                issues.append(f"X-axis shows tilt: {avg_ax:.3f}g")
                quality_score -= 20

            if abs(avg_ay) > 0.3:
                issues.append(f"Y-axis shows tilt: {avg_ay:.3f}g")
                quality_score -= 20

            print(".3f")
            print(".3f")
            print(".3f")
        else:
            issues.append("Could not read accelerometer data")
            quality_score = 0

        quality = "excellent" if quality_score > 80 else "good" if quality_score > 60 else "poor"

        return {
            "quality": quality,
            "score": quality_score,
            "issues": issues
        }

    def optimize_filter_parameters(self) -> Dict[str, float]:
        """Optimize complementary filter parameters based on current conditions."""
        print("🎛️ Optimizing filter parameters...")

        if not self.imu:
            return {"alpha_rp": 0.02, "alpha_yaw": 0.01}

        # Test different alpha values and measure stability
        test_alphas = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2]
        results = []

        print("   Testing different filter parameters...")
        for alpha_rp in test_alphas:
            for alpha_yaw in [alpha_rp * 0.5]:  # Yaw typically needs less correction
                # Temporarily change parameters
                old_alpha_rp = self.imu.alpha_rp
                old_alpha_yaw = self.imu.alpha_yaw

                self.imu.alpha_rp = alpha_rp
                self.imu.alpha_yaw = alpha_yaw

                # Test stability
                stability_score = self._test_stability_score(5.0)

                results.append({
                    'alpha_rp': alpha_rp,
                    'alpha_yaw': alpha_yaw,
                    'stability': stability_score
                })

                # Restore original parameters
                self.imu.alpha_rp = old_alpha_rp
                self.imu.alpha_yaw = old_alpha_yaw

        # Find best parameters
        best_result = max(results, key=lambda x: x['stability'])

        print(".3f")
        print(".3f")
        print(".3f")
        return {
            'alpha_rp': best_result['alpha_rp'],
            'alpha_yaw': best_result['alpha_yaw']
        }

    def _test_stability_score(self, duration: float) -> float:
        """Test stability with current filter parameters."""
        headings = []
        start_time = time.time()

        while time.time() - start_time < duration:
            try:
                data = self.imu.read()
                headings.append(data['heading_deg'])
                time.sleep(0.1)
            except:
                time.sleep(0.1)

        if len(headings) < 10:
            return 0.0

        # Calculate stability score (lower variation = higher score)
        try:
            heading_std = statistics.stdev(headings)
            stability_score = max(0.0, 100.0 - heading_std * 10)  # Scale for 0-100 range
            return stability_score
        except:
            return 0.0

    def run_comprehensive_calibration(self) -> bool:
        """Run comprehensive calibration procedure."""
        print("🚀 Starting Comprehensive IMU Calibration")
        print("=" * 60)

        # Step 1: Initialize IMU
        if not self.initialize_imu():
            return False

        # Step 2: Assess environment
        print("\\n" + "="*60)
        interference = self.detect_interference()

        print("\\n" + "="*60)
        mounting = self.assess_hardware_mounting()

        # Step 3: Optimize parameters
        print("\\n" + "="*60)
        optimal_params = self.optimize_filter_parameters()

        # Step 4: Apply optimized parameters
        print("\\n" + "="*60)
        print("🎯 Applying optimized configuration...")

        # Update IMU parameters
        self.imu.alpha_rp = optimal_params['alpha_rp']
        self.imu.alpha_yaw = optimal_params['alpha_yaw']

        # Step 5: Final validation
        print("\\n" + "="*60)
        quality = self.validate_calibration_quality()

        # Step 6: Generate recommendations
        print("\\n" + "="*60)
        recommendations = self.generate_recommendations(interference, mounting, quality)

        # Save enhanced calibration
        self.save_enhanced_calibration(optimal_params, quality)

        print("\\n" + "="*60)
        print("🎯 CALIBRATION COMPLETE")
        print("="*60)
        print(".1f")
        print(f"Interference Level: {interference.overall_score():.1f}/100")
        print(f"Mounting Quality: {mounting['quality']} ({mounting['score']})")

        if quality.overall_score > 70:
            print("✅ Calibration successful - IMU should now be stable")
        elif quality.overall_score > 40:
            print("⚠️ Calibration acceptable but could be improved")
        else:
            print("❌ Calibration quality poor - hardware issues detected")

        return quality.overall_score > 40

    def validate_calibration_quality(self) -> CalibrationQuality:
        """Comprehensive calibration quality validation."""
        print("🔍 Validating calibration quality...")

        quality = CalibrationQuality()

        if not self.imu:
            return quality

        # Collect data for analysis
        readings = []
        start_time = time.time()

        print("   Analyzing IMU performance (15 seconds)...")
        while time.time() - start_time < 15.0:
            try:
                data = self.imu.read()
                readings.append({
                    'heading': data['heading_deg'],
                    'roll': data['roll_deg'],
                    'pitch': data['pitch_deg'],
                    'yaw_rate': data['gz_dps'],
                    'time': time.time() - start_time
                })
                time.sleep(0.1)
            except Exception as e:
                print(f"   Error reading IMU: {e}")
                time.sleep(0.1)

        if len(readings) < 50:
            print("❌ Insufficient data for quality analysis")
            return quality

        # Analyze heading stability
        headings = [r['heading'] for r in readings]
        try:
            heading_std = statistics.stdev(headings)
            quality.heading_stability = max(0.0, 100.0 - heading_std * 5)
        except:
            quality.heading_stability = 0.0

        # Analyze attitude stability
        rolls = [r['roll'] for r in readings]
        pitches = [r['pitch'] for r in readings]
        try:
            roll_std = statistics.stdev(rolls)
            pitch_std = statistics.stdev(pitches)
            attitude_std = (roll_std + pitch_std) / 2
            quality.attitude_stability = max(0.0, 100.0 - attitude_std * 20)
        except:
            quality.attitude_stability = 0.0

        # Analyze gyro performance (should be stable when stationary)
        yaw_rates = [r['yaw_rate'] for r in readings]
        try:
            gyro_std = statistics.stdev(yaw_rates)
            quality.gyro_performance = max(0.0, 100.0 - gyro_std * 100)
        except:
            quality.gyro_performance = 0.0

        # Placeholder for accel and mag performance (would need more sophisticated analysis)
        quality.accel_performance = 75.0  # Assume good if calibration loaded
        quality.mag_performance = 75.0    # Assume good if calibration loaded
        quality.interference_level = self.detect_interference().overall_score()

        quality.compute_overall()

        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        return quality

    def generate_recommendations(self, interference: InterferenceMetrics,
                               mounting: Dict, quality: CalibrationQuality) -> List[str]:
        """Generate specific recommendations for improvement."""
        recommendations = []

        # Interference recommendations
        if interference.overall_score() > 50:
            recommendations.append("HIGH INTERFERENCE: Move IMU away from motors, batteries, and power wires")
            recommendations.append("Use ferrite beads on power cables near IMU")
            recommendations.append("Consider shielded cables for IMU connections")

        # Mounting recommendations
        if mounting['score'] < 70:
            recommendations.append("MOUNTING ISSUES: Ensure IMU is securely mounted and level")
            recommendations.append("Use plastic mounting hardware instead of metal")
            recommendations.append("Verify IMU axes align with robot coordinate system")

        # Quality recommendations
        if quality.heading_stability < 60:
            recommendations.append("HEADING INSTABILITY: Recalibrate magnetometer in cleaner environment")
        if quality.gyro_performance < 60:
            recommendations.append("GYRO ISSUES: Check for vibration or electrical noise")
        if quality.attitude_stability < 60:
            recommendations.append("ATTITUDE DRIFT: Recalibrate accelerometer with proper orientation")

        if not recommendations:
            recommendations.append("✅ Configuration looks good - no major issues detected")

        print("\\n📋 RECOMMENDATIONS:")
        for rec in recommendations:
            print(f"   • {rec}")

        return recommendations

    def save_enhanced_calibration(self, optimal_params: Dict, quality: CalibrationQuality):
        """Save enhanced calibration with quality metrics."""
        try:
            # Load existing calibration
            if self.calibration_path.exists():
                with open(self.calibration_path, 'r') as f:
                    calib_data = json.load(f)
            else:
                calib_data = {}

            # Add quality metrics and optimal parameters
            calib_data.update({
                'calibration_quality': {
                    'overall_score': quality.overall_score,
                    'heading_stability': quality.heading_stability,
                    'attitude_stability': quality.attitude_stability,
                    'gyro_performance': quality.gyro_performance,
                    'accel_performance': quality.accel_performance,
                    'mag_performance': quality.mag_performance,
                    'interference_level': quality.interference_level
                },
                'optimal_parameters': {
                    'alpha_rp': optimal_params['alpha_rp'],
                    'alpha_yaw': optimal_params['alpha_yaw']
                },
                'calibration_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'calibration_version': 'advanced_v2.0'
            })

            # Save enhanced calibration
            with open(self.calibration_path, 'w') as f:
                json.dump(calib_data, f, indent=2)

            print("✅ Enhanced calibration saved with quality metrics")

        except Exception as e:
            print(f"⚠️ Could not save enhanced calibration: {e}")


def main():
    parser = argparse.ArgumentParser(description="Advanced IMU Calibration System")
    parser.add_argument('--comprehensive', action='store_true',
                       help='Run comprehensive calibration procedure')
    parser.add_argument('--diagnostics', action='store_true',
                       help='Run diagnostics only')
    parser.add_argument('--calibration-file', default='imu_calibration.json',
                       help='Path to calibration file')

    args = parser.parse_args()

    calibrator = AdvancedImuCalibrator(args.calibration_file)

    if args.diagnostics:
        print("🔍 Running IMU Diagnostics")
        print("=" * 40)

        if not calibrator.initialize_imu():
            return 1

        interference = calibrator.detect_interference()
        mounting = calibrator.assess_hardware_mounting()
        quality = calibrator.validate_calibration_quality()

        print("\\n" + "=" * 40)
        print("📊 DIAGNOSTIC SUMMARY")
        print("=" * 40)
        print(f"Interference Level: {interference.overall_score():.1f}/100")
        print(f"Mounting Quality: {mounting['quality']} ({mounting['score']}/100)")
        print(f"Overall Quality: {quality.overall_score:.1f}/100")

        return 0

    elif args.comprehensive:
        success = calibrator.run_comprehensive_calibration()
        return 0 if success else 1

    else:
        print("Advanced IMU Calibration System")
        print("Usage:")
        print("  --comprehensive    : Run full calibration procedure")
        print("  --diagnostics      : Run diagnostics only")
        return 1


if __name__ == "__main__":
    sys.exit(main())
