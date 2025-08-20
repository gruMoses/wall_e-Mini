"""
Configuration file for WALL-E Mini robot control system.
"""

from dataclasses import dataclass
from typing import Final


@dataclass(frozen=True)
class ImuSteeringConfig:
    """Configuration for IMU-based steering compensation."""
    
    # Enable/disable IMU steering
    enabled: bool = True
    
    # PID gains for heading control
    kp: float = 2.0      # Proportional gain (heading error to steering correction)
    ki: float = 0.1      # Integral gain (accumulated drift correction)
    kd: float = 0.5      # Derivative gain (yaw rate damping)
    
    # Control parameters
    max_correction: int = 30      # Maximum steering correction in byte units (0-255)
    deadband_deg: float = 2.0    # Minimum heading error to trigger correction (degrees)
    max_integral: float = 50.0   # Maximum integral term to prevent windup
    
    # Timing
    update_rate_hz: float = 20.0  # IMU update rate
    heading_hold_timeout_s: float = 0.5  # Time to hold heading when no steering input
    
    # Fallback behavior
    fallback_on_error: bool = True  # Use RC control if IMU fails
    calibration_timeout_s: float = 5.0  # Timeout for IMU calibration


@dataclass(frozen=True)
class Config:
    """Main configuration class."""
    
    # IMU steering configuration
    imu_steering: ImuSteeringConfig = ImuSteeringConfig()
    
    # File paths
    imu_calibration_path: str = "imu_calibration.json"
    
    # Debug settings
    log_imu_data: bool = False
    log_steering_corrections: bool = True


# Global configuration instance
config = Config()
