from dataclasses import dataclass


@dataclass(frozen=True)
class DriveCommand:
    left_byte: int
    right_byte: int
    is_armed: bool
    emergency_active: bool


