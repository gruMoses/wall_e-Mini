from dataclasses import dataclass


@dataclass(frozen=True)
class DriveCommand:
    left_byte: int
    right_byte: int
    is_armed: bool
    emergency_active: bool


@dataclass(frozen=True)
class AutonomyCommand:
    """Mode-agnostic autonomy command represented as motor bytes."""
    source: str
    left_byte: int
    right_byte: int
    steering_input: float
    nav_state: str | None = None

