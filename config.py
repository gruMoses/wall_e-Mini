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
    kp: float = 2.0     # Proportional gain (heading error to steering correction)
    ki: float = 0.1     # Integral gain (accumulated drift correction)
    kd: float = 0.7     # Derivative gain (yaw rate damping)
    
    # Control parameters
    max_correction: int = 40      # Maximum steering correction in byte units (0-255)
    deadband_deg: float = 0.5    # Minimum heading error to trigger correction (degrees)
    max_integral: float = 25.0   # Maximum integral term to prevent windup
    invert_output: bool = False   # Invert the sign of IMU steering correction (hardware-specific)
    # Steering neutral detection (hysteresis) to lock heading until commanded turn
    steering_neutral_enter: float = 0.08  # |steering_input| below this enters neutral
    steering_neutral_exit: float = 0.15   # |steering_input| above this exits neutral
    neutral_dwell_s: float = 0.0          # Optional dwell before locking target (0 = immediate)
    # Straight-intent detection for dual-throttle skid steer
    straight_equal_tolerance_us: int = 90     # |ch1_us - ch2_us| <= tol -> straight intent
    straight_min_throttle_us: int = 80        # max(|ch1-1500|,|ch2-1500|) to qualify as moving
    # Relative tolerance to allow proportional mismatch at higher throttle
    straight_relative_tolerance_pct: float = 0.25
    # Optional per-side bias applied only during straight intent (bytes)
    straight_bias_left_byte: int = 0
    straight_bias_right_byte: int = 0
    # Hysteresis time to keep straight intent latched despite brief mismatch (seconds)
    straight_disengage_hysteresis_s: float = 0.30
    # Steering-blend: corrections scale down as absolute steering_input grows; zero at this magnitude
    correction_zero_at_steering: float = 0.30
    
    # Debug and logging
    log_steering_corrections: bool = False  # Enable debug logging of steering corrections
    
    # Timing
    update_rate_hz: float = 30.0  # IMU update rate
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
