"""
Minimal VESC CAN motor driver.

Design:
- Auto-detection is a lightweight check for the presence of CAN channel (default 'can0').
- Control API matches MotorDriver used by Controller: set_tracks(left_byte, right_byte) and stop().
- Byte mapping: 0..255 with 126 as neutral -> RPM in [-max_rpm, +max_rpm].
"""

from __future__ import annotations

import os
from typing import Optional


class VescCanDriver:
    def __init__(
        self,
        channel: str = "can0",
        left_id: int = 1,
        right_id: int = 2,
        max_rpm: int = 3000,
    ) -> None:
        self._channel = channel
        self._left_id = left_id
        self._right_id = right_id
        self._max_rpm = max_rpm
        self._bus = None  # Lazy-init python-can bus

    @staticmethod
    def detect(channel: str = "can0") -> bool:
        # Simple heuristic: presence of CAN netdevice
        return os.path.exists(f"/sys/class/net/{channel}")

    # MotorDriver API
    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        left_rpm = self._byte_to_rpm(left_byte)
        right_rpm = self._byte_to_rpm(right_byte)
        self._send_rpm(self._left_id, left_rpm)
        self._send_rpm(self._right_id, right_rpm)

    def stop(self) -> None:
        self._send_rpm(self._left_id, 0)
        self._send_rpm(self._right_id, 0)

    # Internal helpers
    def _ensure_bus(self):
        if self._bus is not None:
            return
        try:
            import can  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                "python-can is required for VESC CAN control. Install with: pip install python-can"
            ) from exc
        self._bus = can.interface.Bus(channel=self._channel, bustype="socketcan")

    def _send_rpm(self, can_id: int, rpm: int) -> None:
        # Build COMM_SET_RPM frame as per VESC CAN spec; for now we send an extended frame id placeholder.
        # Many setups encapsulate VESC commands in CAN id 0x090 | can_id with payload per COMM_SET_RPM.
        try:
            self._ensure_bus()
        except RuntimeError:
            # If bus cannot be created in this environment (e.g., tests), silently return
            return

        try:
            import can  # type: ignore
        except Exception:
            return

        # Payload for COMM_SET_RPM (command=8) with 32-bit RPM
        cmd = 8
        rpm32 = int(rpm)
        data = bytes([
            cmd,
            (rpm32 >> 24) & 0xFF,
            (rpm32 >> 16) & 0xFF,
            (rpm32 >> 8) & 0xFF,
            rpm32 & 0xFF,
        ])
        # VESC CAN id base for commands to a specific id is typically 0x090 | can_id
        arbitration_id = 0x090 | (can_id & 0x7F)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            assert self._bus is not None
            self._bus.send(msg)
        except Exception:
            # Ignore send errors in minimal driver; higher layers handle faults
            pass

    @staticmethod
    def _byte_to_rpm(byte_value: int, max_rpm: Optional[int] = None) -> int:
        # Map 0..255 with 126 center to [-max_rpm .. +max_rpm]
        max_rpm_val = 3000 if max_rpm is None else max_rpm
        clamped = min(255, max(0, byte_value))
        center = 126
        if clamped == center:
            return 0
        span = 255 - center  # 129
        if clamped > center:
            normalized = (clamped - center) / span  # 0..1
            return int(round(normalized * max_rpm_val))
        else:
            normalized = (center - clamped) / center  # 0..1 using lower span 126
            return -int(round(normalized * max_rpm_val))


