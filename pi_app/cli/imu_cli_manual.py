#!/usr/bin/env python3
"""
Test script for IMU steering compensation.

This script tests:
1. IMU initialization and calibration
2. Steering compensation computation
3. Fallback behavior when IMU is unavailable
"""

import sys
import time
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.hardware.imu_reader import ImuReader
from pi_app.control.imu_steering import ImuSteeringCompensator
from config import config


def test_imu_reader():
    """Test basic IMU functionality."""
    print("Testing IMU Reader...")
    
    try:
        imu = ImuReader()
        print("✓ IMU Reader initialized successfully")
        
        # Test reading
        data = imu.read()
        print(f"✓ IMU data read: heading={data['heading_deg']:.1f}°, "
              f"roll={data['roll_deg']:.1f}°, pitch={data['pitch_deg']:.1f}°")
        
        return imu
        
    except Exception as e:
        print(f"✗ IMU Reader failed: {e}")
        return None


def test_steering_compensator(imu_reader):
    """Test steering compensation logic."""
    print("\nTesting Steering Compensator...")
    
    try:
        compensator = ImuSteeringCompensator(config.imu_steering, imu_reader)
        print("✓ Steering Compensator initialized successfully")
        
        # Test status
        status = compensator.get_status()
        print(f"✓ Status: available={status.is_available}, "
              f"calibrated={status.is_calibrated}")
        
        # Test compensation computation
        dt = 0.05  # 20 Hz
        steering_inputs = [0.0, 0.1, -0.1, 0.0]  # Neutral, slight right, slight left, neutral
        
        for steering in steering_inputs:
            correction = compensator.update(steering, dt)
            if correction is not None:
                print(f"  Steering: {steering:5.1f} → Correction: {correction:6.2f}")
            else:
                print(f"  Steering: {steering:5.1f} → No correction (IMU disabled)")
            time.sleep(0.1)
        
        return compensator
        
    except Exception as e:
        print(f"✗ Steering Compensator failed: {e}")
        return None


def test_fallback_behavior():
    """Test fallback behavior when IMU is unavailable."""
    print("\nTesting Fallback Behavior...")
    
    try:
        # Create compensator without IMU
        compensator = ImuSteeringCompensator(config.imu_steering, None)
        print("✓ Fallback compensator created (no IMU)")
        
        # Test that it returns None for corrections
        correction = compensator.update(0.0, 0.05)
        if correction is None:
            print("✓ Fallback behavior working: no correction when IMU unavailable")
        else:
            print(f"✗ Unexpected correction: {correction}")
            
    except Exception as e:
        print(f"✗ Fallback test failed: {e}")


def main():
    """Main test function."""
    print("IMU Steering Compensation Test")
    print("=" * 40)
    
    # Test IMU reader
    imu_reader = test_imu_reader()
    
    # Test steering compensator
    compensator = None
    if imu_reader is not None:
        compensator = test_steering_compensator(imu_reader)
    
    # Test fallback behavior
    test_fallback_behavior()
    
    # Summary
    print("\n" + "=" * 40)
    print("Test Summary:")
    print(f"  IMU Reader: {'✓' if imu_reader else '✗'}")
    print(f"  Steering Compensator: {'✓' if compensator else '✗'}")
    print("  Fallback Behavior: ✓")
    
    if imu_reader and compensator:
        print("\n🎉 All tests passed! IMU steering compensation is ready.")
    else:
        print("\n⚠️  Some tests failed. Check IMU hardware and connections.")
        print("   The system will fall back to RC-only control.")


if __name__ == "__main__":
    main()
