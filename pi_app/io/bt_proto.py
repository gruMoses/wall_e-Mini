from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


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
    Map normalized floats in [-1, 1] to bytes in [0, 255].
    Center 0.0 maps to ~128. Endpoints map to 0/255.
    """
    def map_one(v: float) -> int:
        # Normalize from [-1,1] -> [0,1]
        normalized = (v + 1.0) * 0.5
        return max(0, min(255, int(round(normalized * 255.0))))

    return map_one(left_f), map_one(right_f)



