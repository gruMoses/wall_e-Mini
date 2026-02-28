from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT


@dataclass
class Cmd2:
    left_i: int
    right_i: int
    seq: int
    ts_ms: int
    sn_hex: str
    hmac_hex: str


def parse_cmd2(line: str) -> Optional[Cmd2]:
    # Format: CMD2:<left_i>;<right_i>;<seq>;<ts_ms>;<sn_hex>;<hmac_hex>
    if not line.startswith("CMD2:"):
        return None
    try:
        payload = line[len("CMD2:") :].strip()
        parts = payload.split(";")
        if len(parts) != 6:
            return None
        left_i = int(parts[0])
        right_i = int(parts[1])
        seq = int(parts[2])
        ts_ms = int(parts[3])
        sn_hex = parts[4]
        hmac_hex = parts[5]
        return Cmd2(left_i, right_i, seq, ts_ms, sn_hex, hmac_hex)
    except Exception:
        return None


def accept_cmd2(cmd: Cmd2, last_seq: Optional[int]) -> Tuple[bool, str]:
    """
    Accept any well-formed CMD2 without authentication.
    Enforces strictly increasing sequence numbers if last_seq is provided.
    """
    if last_seq is not None and not (cmd.seq > last_seq):
        return False, "old_seq"
    return True, "ok"


def parse_v1(line: str) -> Optional[Tuple[float, float, int]]:
    """
    Parse lines like: V1:<left_float>;<right_float>;<seq>
    - Floats expected in [-1.0, 1.0]
    - Accepts comma or dot as decimal separator
    - Ignores trailing whitespace/newlines
    """
    if not line.startswith("V1:"):
        return None
    payload = line[3:].strip()
    parts = payload.split(";")
    if len(parts) != 3:
        return None
    try:
        lf = float(parts[0].replace(",", "."))
        rf = float(parts[1].replace(",", "."))
        seq = int(parts[2])
    except Exception:
        return None
    # Clamp to sane range
    lf = max(-1.0, min(1.0, lf))
    rf = max(-1.0, min(1.0, rf))
    return lf, rf, seq


def floats_to_bytes(left_f: float, right_f: float) -> Tuple[int, int]:
    """
    Map normalized floats in [-1, 1] to motor bytes in [0, 254].
    Center 0.0 maps to 126 (system neutral). Endpoints map to 0/254.
    A small deadband around 0.0 forces exact neutral to avoid drift.
    """
    DEAD_BAND = 0.02
    TOP_SNAP = 0.95

    def map_one(v: float) -> int:
        if v > 1.0:
            v = 1.0
        if v < -1.0:
            v = -1.0
        if v >= TOP_SNAP:
            return MAX_OUTPUT
        if v <= -TOP_SNAP:
            return MIN_OUTPUT
        if -DEAD_BAND <= v <= DEAD_BAND:
            return CENTER_OUTPUT_VALUE
        if v > 0.0:
            upper_span = MAX_OUTPUT - CENTER_OUTPUT_VALUE
            mapped = CENTER_OUTPUT_VALUE + int(round(min(1.0, v) * upper_span))
        else:
            lower_span = CENTER_OUTPUT_VALUE
            mapped = CENTER_OUTPUT_VALUE - int(round(min(1.0, abs(v)) * lower_span))
        if mapped < MIN_OUTPUT:
            return MIN_OUTPUT
        if mapped > MAX_OUTPUT:
            return MAX_OUTPUT
        return mapped

    return map_one(left_f), map_one(right_f)



