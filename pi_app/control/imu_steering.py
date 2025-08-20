"""
IMU-based steering compensation for WALL-E Mini robot.

This module implements a PID controller that uses IMU data to:
1. Compensate for bumps and terrain variations that cause unintended turning
2. Maintain straight-line travel when no steering input is given
3. Provide smooth corrections for external disturbances
"""

import time
import math
from dataclasses import dataclass
from typing import Optional, Tuple
from threading import Lock
import sys
from pathlib import Path

# Add parent directory to path for config import
sys.path.append(str(Path(__file__).resolve().parents[2]))

from pi_app.hardware.imu_reader import ImuReader
from config import ImuSteeringConfig


@dataclass
class ImuSteeringState:
    """Current state of IMU steering compensation."""
    
    # IMU data
    heading_deg: float = 0.0
    yaw_rate_dps: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    
    # Control state
    target_heading_deg: float = 0.0
    last_steering_input: float = 0.0  # -1.0 to 1.0, where 0.0 is neutral
    last_update_time: float = 0.0
    
    # PID state
    integral_error: float = 0.0
    last_error: float = 0.0
    
    # Status
    is_calibrated: bool = False
    is_available: bool = False
    error_count: int = 0


class ImuSteeringCompensator:
    """
    PID-based steering compensator using IMU data.
    
    The compensator works by:
    1. Detecting when the robot is turning unintentionally (yaw rate)
    2. Maintaining a target heading when no steering input is given
    3. Applying proportional corrections based on heading error
    4. Using integral term to correct for systematic drift
    5. Using derivative term to dampen oscillations
    """
    
    def __init__(self, config: ImuSteeringConfig, imu_reader: Optional[ImuReader] = None):
        self.config = config
        self.imu_reader = imu_reader
        self.state = ImuSteeringState()
        self.lock = Lock()
        
        # Initialize IMU if provided
        if self.imu_reader is not None:
            self._initialize_imu()
    
    def _initialize_imu(self) -> None:
        """Initialize and calibrate the IMU."""
        try:
            if self.imu_reader is None:
                return
                
            # Try to calibrate gyroscope
            self.imu_reader.calibrate_gyro(duration_s=min(3.0, self.config.calibration_timeout_s))
            
            # Try to calibrate magnetometer
            self.imu_reader.calibrate_mag_hard_iron(duration_s=min(5.0, self.config.calibration_timeout_s))
            
            # Read initial heading
            data = self.imu_reader.read()
            with self.lock:
                self.state.heading_deg = data['heading_deg']
                self.state.target_heading_deg = data['heading_deg']
                self.state.is_calibrated = True
                self.state.is_available = True
                self.state.last_update_time = time.time()
                
        except Exception as e:
            print(f"IMU initialization failed: {e}")
            with self.lock:
                self.state.is_available = False
                self.state.is_calibrated = False
    
    def update(self, steering_input: float, dt: float) -> Optional[float]:
        """
        Update IMU state and compute steering compensation.
        
        Args:
            steering_input: Current steering input from RC/BT (-1.0 to 1.0, 0.0 = neutral)
            dt: Time delta since last update
            
        Returns:
            Steering correction in byte units (-max_correction to +max_correction),
            or None if IMU is not available or compensation is disabled
        """
        if not self.config.enabled or not self.state.is_available:
            return None
            
        try:
            # Update IMU data
            if self.imu_reader is not None:
                data = self.imu_reader.read()
                with self.lock:
                    self.state.heading_deg = data['heading_deg']
                    self.state.yaw_rate_dps = data['gz_dps']  # Use gyroscope z-axis for yaw rate
                    self.state.roll_deg = data['roll_deg']
                    self.state.pitch_deg = data['pitch_deg']
                    self.state.last_update_time = time.time()
            
            # Update target heading when steering input changes significantly
            if abs(steering_input - self.state.last_steering_input) > 0.1:
                with self.lock:
                    self.state.target_heading_deg = self.state.heading_deg
                    self.state.integral_error = 0.0  # Reset integral when steering changes
            
            self.state.last_steering_input = steering_input
            
            # Compute compensation only when near neutral steering
            if abs(steering_input) < 0.1:
                return self._compute_heading_hold_correction(dt)
            else:
                # Reset integral when actively steering
                with self.lock:
                    self.state.integral_error = 0.0
                return None
                
        except Exception as e:
            self.state.error_count += 1
            if self.state.error_count > 10:
                with self.lock:
                    self.state.is_available = False
            return None
    
    def _compute_heading_hold_correction(self, dt: float) -> float:
        """Compute PID-based heading hold correction."""
        with self.lock:
            # Compute heading error (normalized to [-180, 180])
            error_deg = self.state.target_heading_deg - self.state.heading_deg
            while error_deg > 180:
                error_deg -= 360
            while error_deg < -180:
                error_deg += 360
            
            # Apply deadband
            if abs(error_deg) < self.config.deadband_deg:
                return 0.0
            
            # PID control
            # Proportional term
            p_term = self.config.kp * error_deg
            
            # Integral term (with anti-windup)
            self.state.integral_error += error_deg * dt
            self.state.integral_error = max(-self.config.max_integral, 
                                         min(self.config.max_integral, self.state.integral_error))
            i_term = self.config.ki * self.state.integral_error
            
            # Derivative term (yaw rate damping)
            d_term = -self.config.kd * self.state.yaw_rate_dps
            
            # Combine terms
            correction = p_term + i_term + d_term
            
            # Clamp to maximum correction
            max_corr = float(self.config.max_correction)
            correction = max(-max_corr, min(max_corr, correction))
            
            return correction
    
    def get_status(self) -> ImuSteeringState:
        """Get current IMU steering state."""
        with self.lock:
            return ImuSteeringState(
                heading_deg=self.state.heading_deg,
                yaw_rate_dps=self.state.yaw_rate_dps,
                roll_deg=self.state.roll_deg,
                pitch_deg=self.state.pitch_deg,
                target_heading_deg=self.state.target_heading_deg,
                last_steering_input=self.state.last_steering_input,
                last_update_time=self.state.last_update_time,
                integral_error=self.state.integral_error,
                last_error=self.state.last_error,
                is_calibrated=self.state.is_calibrated,
                is_available=self.state.is_available,
                error_count=self.state.error_count
            )
    
    def reset_target_heading(self) -> None:
        """Reset target heading to current heading."""
        with self.lock:
            self.state.target_heading_deg = self.state.heading_deg
            self.state.integral_error = 0.0
    
    def set_target_heading(self, heading_deg: float) -> None:
        """Set a specific target heading."""
        with self.lock:
            self.state.target_heading_deg = heading_deg
            self.state.integral_error = 0.0
