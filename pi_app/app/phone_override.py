"""Freshness of the shared motor-override file (phone teleop / SPP bench).

``/tmp/wall_e_bt_latest.json`` is written by another process (the web
teleop watchdog in ``pi_app/web/teleop.py``, or the opt-in SPP bench server
in ``pi_app/cli/spp_server.py``) and read by the main loop in
``pi_app/app/main.py``. The writer stamps each command with
``time.monotonic()`` in ``last_update_mono_s``; the reader compares that
with its own ``time.monotonic()``. CLOCK_MONOTONIC is one system-wide clock
on Linux, so the two processes share it.

The wall clock is NOT used: this Pi's clock steps at boot (the RTC resets
when 5 V drops, then NTP corrects it), and a backward step made an old
command look fresh. A missing stamp (an old writer that only wrote
``last_update_epoch_s``), a non-finite stamp, or a negative age is stale.

The monotonic clock starts again at each boot, and ``/tmp`` can keep the
file across a reboot. A command left from an earlier boot (for example the
power went off during a phone drive) would look fresh again for 0.6 s when
the uptime of the new boot passes its stamp. So each writer also stores the
kernel's boot ID, and a command from another boot, or with no boot ID, is
stale.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Optional, Tuple

# Unchanged from the old epoch check in main.py: a command older than this
# falls back to the RC sticks.
OVERRIDE_FRESH_S = 0.6

_BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def read_boot_id(path: str = _BOOT_ID_PATH) -> Optional[str]:
    """The kernel's ID of this boot, or None where it cannot be read (not Linux)."""
    try:
        return Path(path).read_text(encoding="ascii").strip() or None
    except (OSError, ValueError):
        return None


# Read once. Writers store it with each command; the reader compares.
BOOT_ID: Optional[str] = read_boot_id()

_USE_MODULE_BOOT_ID = object()


def override_age_s(data: Any, now_mono_s: float) -> Optional[float]:
    """Age of the override command in seconds, or None when it has no valid stamp."""
    if not isinstance(data, dict):
        return None
    stamp = data.get("last_update_mono_s")
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        return None
    age = float(now_mono_s) - float(stamp)
    if not math.isfinite(age):
        return None
    return age


def fresh_override_bytes(
    data: Any,
    now_mono_s: float,
    max_age_s: float = OVERRIDE_FRESH_S,
    boot_id: Any = _USE_MODULE_BOOT_ID,
) -> Tuple[Optional[Tuple[int, int]], Optional[float]]:
    """Return ``((left, right), age_s)`` for a fresh command, else ``(None, age_s)``.

    Fresh means 0 <= age <= ``max_age_s`` AND the command was written in
    this boot. A negative age (the stamp is in this clock's future) is
    stale, never fresh. ``boot_id`` is this process's boot ID (default: the
    one read at import); when it is known, a command with another boot ID or
    with none is stale. When it is not known (not Linux), only the age
    decides.
    """
    age = override_age_s(data, now_mono_s)
    if age is None or age < 0.0 or age > max_age_s:
        return None, age
    reader_boot_id = BOOT_ID if boot_id is _USE_MODULE_BOOT_ID else boot_id
    if reader_boot_id is not None and data.get("boot_id") != reader_boot_id:
        return None, age
    return (data["left_byte"], data["right_byte"]), age
