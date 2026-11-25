"""Start the OAK reader whether or not the camera is on USB at boot.

Audit 2026-11-25, section 2 item 4. ``OakDepthReader.detect()`` ran once at
startup, and with no camera main.py built no reader at all: no obstacle
controller, no 0.8 m person stop, no IMU, and MANUAL drove at full speed
until the next service restart. A camera that fails AFTER startup instead
leaves MANUAL at the stale-depth floor (``manual_stale_throttle_scale``)
with FOLLOW_ME and WAYPOINT_NAV refused.

Kevin's decision (2026-11-25): start the reader anyway and let it wait for
the camera, so the two cases are the same. The reader then polls USB
(``OakDepthReader._await_first_device``) and opens the session when the
camera enumerates; everything built on it comes up with no restart.

Kept out of main.py so a test can drive it without importing the hardware
stack.
"""

from __future__ import annotations

from typing import Callable


def start_oak_reader(make_reader: Callable[[], object],
                     detect: Callable[[], bool]) -> tuple[object, bool]:
    """Build and start the reader. Returns ``(reader, present_at_boot)``.

    ``detect`` is ``OakDepthReader.detect``. Camera present: ``start()``,
    exactly as before. Camera absent: ``start(wait_for_device=True)``, so
    the worker waits for it and every reading stays stale until then.
    """
    present = bool(detect())
    reader = make_reader()
    if present:
        reader.start()
        print("OAK-D Lite detected and started")
    else:
        reader.start(wait_for_device=True)
        print(
            "OAK-D Lite NOT detected at startup: reader started and waiting for "
            "the camera (no restart needed). Until it connects: MANUAL at the "
            "stale-depth floor, FOLLOW_ME and WAYPOINT_NAV refused, no IMU heading"
        )
    return reader, present
