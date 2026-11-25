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

import threading
from typing import Callable, Optional

# How long the startup USB enumeration may take. ``detect()`` calls
# depthai's getAllAvailableDevices(), which has no timeout of its own.
DETECT_TIMEOUT_S = 10.0


def bounded_detect(detect: Callable[[], bool], timeout_s: float) -> Optional[bool]:
    """``detect()`` in a daemon thread. True / False, or None if it has not
    returned within ``timeout_s`` (the thread is left to finish on its own;
    a call stuck inside depthai cannot be interrupted)."""
    box: dict = {}

    def run() -> None:
        try:
            box["found"] = bool(detect())
        except Exception:
            box["found"] = False

    thread = threading.Thread(target=run, name="OakUsbDetect", daemon=True)
    thread.start()
    thread.join(timeout=max(0.0, float(timeout_s)))
    if thread.is_alive():
        return None
    return bool(box.get("found", False))


def start_oak_reader(make_reader: Callable[[], object],
                     detect: Callable[[], bool],
                     detect_timeout_s: float = DETECT_TIMEOUT_S) -> tuple[object, bool]:
    """Build and start the reader. Returns ``(reader, present_at_boot)``.

    ``detect`` is ``OakDepthReader.detect``, bounded by ``detect_timeout_s``
    so a blocked USB enumeration cannot hang startup. Camera present:
    ``start()``, exactly as before. Camera absent, or the enumeration did
    not return: ``start(wait_for_device=True)``, so the worker waits for it
    and every reading stays stale until then.
    """
    found = bounded_detect(detect, detect_timeout_s)
    present = bool(found)
    reader = make_reader()
    if present:
        reader.start()
        print("OAK-D Lite detected and started")
    else:
        reader.start(wait_for_device=True)
        why = (
            f"USB enumeration did not return in {detect_timeout_s:g} s"
            if found is None else "not on USB"
        )
        print(
            f"OAK-D Lite NOT detected at startup ({why}): reader started and "
            "waiting for the camera (no restart needed). Until it connects: "
            "MANUAL at the stale-depth floor, FOLLOW_ME and WAYPOINT_NAV "
            "refused, no IMU heading"
        )
    return reader, present
