#!/usr/bin/env python3
"""
vesc_battery_limits.py - set part A of docs/battery_settings.md (section 4) on
the two drive VESCs over SocketCAN. DRY RUN unless --write is given.

It changes 7 MCCONF fields per VESC and nothing else:
  l_in_current_max, l_in_current_min, l_battery_cut_start, l_battery_cut_end,
  l_battery_regen_cut_start, l_battery_regen_cut_end, si_battery_cells

Method: the same read-modify-write that VESC Tool does for "write motor
configuration". COMM_GET_MCCONF (14) returns the LIVE configuration,
serialized with the FW 6.05 layout (477 bytes, decoded by vesc_info.py). The
tool replaces only the bytes of the 7 fields, proves that every other byte is
unchanged, and sends the result back with COMM_SET_MCCONF (13) as one CAN
buffer (FILL_RX_BUFFER, FILL_RX_BUFFER_LONG, PROCESS_RX_BUFFER with CRC16),
framed exactly as the firmware's comm_can_send_buffer() frames it.

Firmware behaviour (vedderb/bldc release_6_05):
  * commands.c COMM_SET_MCCONF: deserializes into a copy of the live config,
    clamps the current scales, applies the hardware limits, STORES to flash,
    applies, sleeps 200 ms, then replies one byte (13). A bad signature gives
    no reply and changes nothing. It does not check that the motor is still.
  * conf_general.c conf_general_store_mc_configuration(): releases the motor
    and waits up to 2 s for it. COMM_SET_MCCONF ignores the store result, and
    GET_MCCONF returns the LIVE config, so a read-back proves the APPLIED
    values only. At boot a stored config with a bad CRC is replaced by the
    firmware DEFAULTS, and no fault shows in GET_VALUES.
  * So after the live read-back the tool reboots the VESC (COMM_REBOOT, 29:
    NVIC_SystemReset; the VESC re-reads flash at boot) and compares again.
    The FOC offsets are measured again at boot, so those fields are skipped.

Gates (each must pass, for each VESC, again right before every frame burst):
  * wall-e.service is "inactive", and the auto-deploy hold file exists
    (auto_deploy.sh restarts wall-e on any code push, even a stopped one);
  * the VESC reports its command timeout as active: no device on the bus is
    sending it motor commands (catches a hand-run main.py or VESC Tool too);
  * GET_VALUES (second of two reads): |erpm| < 100, |input current| < 1 A,
    no fault, 30 V < v_in < 60 V;
  * HW 75_100_V2, FW 6.05 (no test build), and the UUID pinned for that CAN
    id (the right VESC's +0.9 V targets follow the unit, not just the id);
  * each of the 7 fields holds either its documented pre-change value or its
    target (a VESC that booted on defaults is refused), and the live config
    matches the newest backup of this VESC outside the 7 fields and the boot
    offsets (override that one check with --accept-live);
  * the patched config differs from the live one only inside the 7 fields;
  * a backup of the live config is written (exclusive create), fsynced, and
    read back with its SHA-256 before the first frame.

Usage on the Pi (python-can lives in the venv):
  /home/pi/.venv/bin/python3 tools/vesc_battery_limits.py            # dry run
  /home/pi/.venv/bin/python3 tools/vesc_battery_limits.py --write    # both VESCs
  /home/pi/.venv/bin/python3 tools/vesc_battery_limits.py --restore \
      /home/pi/vesc_backups/<file>.bin --id 2 --write
Exit codes: 0 done (or dry run), 1 refused with nothing written, 2 bus error,
3 a write failed (state uncertain: treat the pair as split), 4 the two VESCs
are (or may be) left with DIFFERENT settings: one at target and one not, one
not checked in this run, or after a one-VESC restore.

Reboot proof: the VESC must stop answering at least once after COMM_REBOOT,
and one of its boot-measured current offsets must change. Anything else is
reported as "stored copy NOT proven", never as VERIFIED.
"""

from __future__ import annotations

import argparse
import datetime
import glob
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import vesc_info as vi  # noqa: E402

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

COMM_SET_MCCONF = 13
COMM_REBOOT = 29
SERVICE = "wall-e.service"
SERVICE_STATES_OK = frozenset({"inactive"})
DEFAULT_DEPLOY_HOLD = "/home/pi/wall_e-Mini/.deploy_hold"
EXPECTED_HW = "75_100_V2"
EXPECTED_FW = (6, 5)

# Read from the units on 2026-11-11 (dry run). A different unit on an id is
# refused: the right VESC's targets carry its +0.9 V reading offset.
EXPECTED_UUID = {2: "1d0031001551343037393236", 1: "43003f001751343036383837"}

BODY_LEN = vi.MCCONF_BODY_LEN_6_05          # 477: signature + fields
WRITE_PAYLOAD_LEN = 1 + BODY_LEN            # 478: command byte + body

MAX_ERPM_STILL = 100.0
MAX_INPUT_CURRENT_STILL_A = 1.0
V_IN_MIN = 30.0
V_IN_MAX = 60.0

ACK_TIMEOUT_S = 6.0          # store: motor release (<= 2 s) + flash + 200 ms
FRAME_GAP_S = 0.001          # pacing; the VESC's CAN receive ring holds 50 frames
SETTLE_S = 0.3
REBOOT_WAIT_S = 20.0
REBOOT_SETTLE_S = 1.5        # after the first answer: let boot calibration finish
REBOOT_POLL_S = 0.5
REBOOT_NO_SILENCE_S = 3.0    # answering all along = the reboot did not happen
QUICK_TIMEOUT_S = 0.25       # one FW_VERSION poll while waiting for the reboot
OFFSETS_WAIT_S = 6.0         # boot calibration must change the offsets by then

DEFAULT_BACKUP_DIR = os.path.join(os.path.expanduser("~"), "vesc_backups")

# docs/battery_settings.md section 4 (part A). The right VESC (id 1) reads the
# bus about 0.9 V high (finding 7), so its voltage values are 0.9 V higher.
# The firmware maps regen from start + 0.1 V to end - 0.1 V.
TARGETS: Dict[int, Dict[str, float]] = {
    2: {  # left
        "l_in_current_max": 15.0,
        "l_in_current_min": -5.0,
        "l_battery_cut_start": 44.2,
        "l_battery_cut_end": 41.6,
        "l_battery_regen_cut_start": 54.4,
        "l_battery_regen_cut_end": 54.9,
        "si_battery_cells": 13,
    },
    1: {  # right
        "l_in_current_max": 15.0,
        "l_in_current_min": -5.0,
        "l_battery_cut_start": 45.1,
        "l_battery_cut_end": 42.5,
        "l_battery_regen_cut_start": 55.3,
        "l_battery_regen_cut_end": 55.8,
        "si_battery_cells": 13,
    },
}
TARGET_FIELDS: Tuple[str, ...] = tuple(TARGETS[2].keys())

# The documented "Now" column (2026-11-05), read again on 2026-11-11.
BEFORE: Dict[int, Dict[str, float]] = {
    2: {"l_in_current_max": 40.0, "l_in_current_min": -40.0,
        "l_battery_cut_start": 40.8, "l_battery_cut_end": 36.0,
        "l_battery_regen_cut_start": 1000.0, "l_battery_regen_cut_end": 1100.0,
        "si_battery_cells": 13},
    1: {"l_in_current_max": 40.0, "l_in_current_min": -40.0,
        "l_battery_cut_start": 40.8, "l_battery_cut_end": 36.0,
        "l_battery_regen_cut_start": 1000.0, "l_battery_regen_cut_end": 1100.0,
        "si_battery_cells": 12},
}

# Proof that a reboot happened: with foc_offsets_cal_on_boot = 1 (both units,
# read 2026-11-11) the boot re-measures these noisy floats (e.g. 2048.726), so
# at least one of them changes bit for bit.
REBOOT_PROOF_FIELDS: Tuple[str, ...] = (
    "foc_offsets_current_0", "foc_offsets_current_1", "foc_offsets_current_2")

# Measured again at every boot (foc_offsets_cal_on_boot); never compared.
BOOT_MEASURED: Tuple[str, ...] = (
    "foc_offsets_current_0", "foc_offsets_current_1", "foc_offsets_current_2",
    "foc_offsets_voltage_0", "foc_offsets_voltage_1", "foc_offsets_voltage_2",
    "foc_offsets_voltage_undriven_0", "foc_offsets_voltage_undriven_1",
    "foc_offsets_voltage_undriven_2",
)

UNITS = {
    "l_in_current_max": "A", "l_in_current_min": "A",
    "l_battery_cut_start": "V", "l_battery_cut_end": "V",
    "l_battery_regen_cut_start": "V", "l_battery_regen_cut_end": "V",
    "si_battery_cells": "",
}


class WriteRefused(Exception):
    """A gate failed; nothing was written to this VESC."""


class WriteGuardViolation(AssertionError):
    """A frame other than the intended ones was about to be sent.

    Raised explicitly, so `python -O` cannot strip the check.
    """


# ---------------------------------------------------------------------------
# Layout and encoding (confgenerator.c / util/buffer.c, release_6_05)
# ---------------------------------------------------------------------------

def field_layout() -> Dict[str, Tuple[int, int, str, Optional[float]]]:
    """name -> (offset in the body, size, kind, scale). Offset 0 = signature."""
    out: Dict[str, Tuple[int, int, str, Optional[float]]] = {}
    off = 4
    for name, kind, scale in vi.MCCONF_FIELDS_6_05:
        size = vi._KIND_SIZES[kind]
        out[name] = (off, size, kind, scale)
        off += size
    if off != BODY_LEN:
        raise RuntimeError(f"MCCONF layout sums to {off}, expected {BODY_LEN}")
    return out


LAYOUT = field_layout()


def float32_auto_encode(number: float) -> bytes:
    """buffer_append_float32_auto()."""
    number = vi._f32(number)
    if abs(number) < 1.5e-38:
        number = 0.0
    sig, e = math.frexp(number)
    sig_abs = abs(sig)
    sig_i = 0
    if sig_abs >= 0.5:
        sig_i = int((sig_abs - 0.5) * 2.0 * 8388608.0)
        e += 126
    res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF)
    if sig < 0:
        res |= 1 << 31
    return struct.pack(">I", res)


def f16_raw(value: float, scale: float) -> int:
    """The int16 we store for a target value (the intended quantum)."""
    raw = int(round(value * scale))
    if not -32768 <= raw <= 32767:
        raise ValueError(f"{value} x {scale} does not fit an int16")
    return raw


def f16_readback_raw(raw: int, scale: float) -> int:
    """What the firmware serializes after it deserialized `raw`.

    buffer_get_float16: (float)int16 / scale (float division);
    buffer_append_float16: (int16_t)(number * scale) (float multiply,
    truncation toward zero).
    """
    s = vi._f32(scale)
    v = vi._f32(raw / s)
    return int(vi._f32(v * s))


def encode_field(name: str, value: float) -> bytes:
    _off, size, kind, scale = LAYOUT[name]
    if kind == "f32a":
        out = float32_auto_encode(float(value))
    elif kind == "f16":
        assert scale is not None
        out = struct.pack(">h", f16_raw(float(value), scale))
    elif kind == "u8":
        iv = int(value)
        if iv != value or not 0 <= iv <= 255:
            raise ValueError(f"{name}: {value!r} is not a u8")
        out = bytes([iv])
    else:
        raise ValueError(f"{name}: kind {kind} is not supported for writing")
    if len(out) != size:
        raise RuntimeError(f"{name}: encoded {len(out)} bytes, expected {size}")
    return out


def decode_body(body: bytes) -> Dict[str, Any]:
    return vi.decode_mcconf(bytes([vi.COMM_GET_MCCONF]) + bytes(body))["fields"]


def field_bytes(body: bytes, name: str) -> bytes:
    off, size, _k, _s = LAYOUT[name]
    return bytes(body[off:off + size])


def check_targets_table() -> None:
    """Static sanity of TARGETS / BEFORE. Raises ValueError on a bad table."""
    for vid, t in TARGETS.items():
        if set(t) != set(TARGET_FIELDS) or set(BEFORE[vid]) != set(TARGET_FIELDS):
            raise ValueError(f"id {vid}: target fields differ")
        if not t["l_in_current_min"] < 0 < t["l_in_current_max"]:
            raise ValueError(f"id {vid}: input current limits out of order")
        if not t["l_battery_cut_end"] < t["l_battery_cut_start"]:
            raise ValueError(f"id {vid}: battery cut end must be below start")
        if not t["l_battery_regen_cut_start"] < t["l_battery_regen_cut_end"]:
            raise ValueError(f"id {vid}: regen cut start must be below end")
        if not t["l_battery_cut_start"] < t["l_battery_regen_cut_start"]:
            raise ValueError(f"id {vid}: cutoff above the regen cutoff")
        if not 10 <= t["si_battery_cells"] <= 20:
            raise ValueError(f"id {vid}: implausible cell count")
        for name in TARGET_FIELDS:
            _o, _s, kind, scale = LAYOUT[name]
            encode_field(name, BEFORE[vid][name])
            if kind == "f16":
                assert scale is not None
                raw = f16_raw(t[name], scale)
                if f16_readback_raw(raw, scale) != raw:
                    raise ValueError(f"id {vid}: {name} would not read back stable")
    for name in ("l_battery_cut_start", "l_battery_cut_end",
                 "l_battery_regen_cut_start", "l_battery_regen_cut_end"):
        if abs((TARGETS[1][name] - TARGETS[2][name]) - 0.9) > 1e-6:
            raise ValueError(f"{name}: right must be left + 0.9 V")
    if len(set(EXPECTED_UUID.values())) != len(EXPECTED_UUID):
        raise ValueError("EXPECTED_UUID has a duplicate")


# ---------------------------------------------------------------------------
# Patch and proof
# ---------------------------------------------------------------------------

def patch_body(body: bytes, targets: Dict[str, float]) -> bytes:
    if len(body) != BODY_LEN:
        raise ValueError(f"body is {len(body)} bytes, expected {BODY_LEN}")
    new = bytearray(body)
    for name, value in targets.items():
        off, size, _k, _s = LAYOUT[name]
        new[off:off + size] = encode_field(name, value)
    return bytes(new)


def changed_fields(old: bytes, new: bytes, skip: Sequence[str] = ()) -> List[str]:
    return [name for name, *_ in vi.MCCONF_FIELDS_6_05
            if name not in skip and field_bytes(old, name) != field_bytes(new, name)]


def prove_patch(old: bytes, new: bytes, targets: Dict[str, float]) -> None:
    """Raise WriteRefused unless new = old with only the target fields changed
    and every target decodes to its intended value."""
    if len(old) != BODY_LEN or len(new) != BODY_LEN:
        raise WriteRefused("body length is not the FW 6.05 length")
    if old[:4] != new[:4] or struct.unpack(">I", new[:4])[0] != vi.MCCONF_SIGNATURE_6_05:
        raise WriteRefused("signature changed or is not FW 6.05")
    allowed = set()
    for name in targets:
        off, size, _k, _s = LAYOUT[name]
        allowed.update(range(off, off + size))
    stray = [i for i in range(BODY_LEN) if old[i] != new[i] and i not in allowed]
    if stray:
        raise WriteRefused(f"patch touches bytes outside the target fields: {stray[:8]}")
    dec = decode_body(new)
    for name, value in targets.items():
        _o, _s, kind, scale = LAYOUT[name]
        got = dec[name]
        if kind == "u8":
            ok = got == int(value)
        elif kind == "f16":
            assert scale is not None
            ok = got == vi._f32(f16_raw(value, scale) / vi._f32(scale))
        else:
            ok = got == vi._f32(value)
        if not ok:
            raise WriteRefused(f"{name}: patched value decodes to {got}, intended {value}")


def predicted_drift(body: bytes) -> List[str]:
    """f16 fields whose stored int16 would read back as a different int16."""
    out = []
    for name, (off, _size, kind, scale) in LAYOUT.items():
        if kind != "f16":
            continue
        assert scale is not None
        raw = struct.unpack(">h", body[off:off + 2])[0]
        if f16_readback_raw(raw, scale) != raw:
            out.append(name)
    return out


def apply_predicted_drift(body: bytes) -> bytes:
    new = bytearray(body)
    for name in predicted_drift(body):
        off, _size, _kind, scale = LAYOUT[name]
        assert scale is not None
        raw = struct.unpack(">h", body[off:off + 2])[0]
        new[off:off + 2] = struct.pack(">h", f16_readback_raw(raw, scale))
    return bytes(new)


def config_refusals(vid: int, live: bytes, backup_dir: str, uuid: str,
                    accept_live: bool) -> List[str]:
    """Is the live config the one we think it is? (Not used for --restore.)"""
    out = []
    for name in TARGET_FIELDS:
        got = field_bytes(live, name)
        if got not in (encode_field(name, BEFORE[vid][name]),
                       encode_field(name, TARGETS[vid][name])):
            val = decode_body(live)[name]
            out.append(f"{name} is {vi._num(val)}, neither the documented "
                       f"{vi._num(BEFORE[vid][name])} nor the target "
                       f"{vi._num(TARGETS[vid][name])} (did it boot on defaults?)")
    newest = newest_backup(backup_dir, vid, uuid)
    if newest is not None and not accept_live:
        ref = newest[0]
        diff = differs_from(ref, live, tuple(TARGET_FIELDS) + BOOT_MEASURED)
        if diff:
            out.append(f"live config differs from the newest backup outside the 7 "
                       f"fields: {', '.join(diff[:8])} (check it; --accept-live to proceed)")
    return out


# ---------------------------------------------------------------------------
# Frames (comm_can.c comm_can_send_buffer) and their guard
# ---------------------------------------------------------------------------

def _pick_reset_crc() -> int:
    """A CRC no 1-byte buffer can have, so the reset frame can never execute."""
    single = {vi.crc16(bytes([b])) for b in range(256)}
    for c in (0xFFFF, 0xFFFE, 0xFFFD):
        if c not in single:
            return c
    raise RuntimeError("no reset CRC")  # pragma: no cover


RESET_CRC = _pick_reset_crc()


def reset_frame(vesc_id: int) -> Tuple[int, bytes]:
    """PROCESS_RX_BUFFER for 1 byte. No buffer is ever filled to offset 1, so
    the firmware resets all 3 receive buffers (comm_can.c 'Something is wrong,
    reset all buffers'); a stale half-written buffer cannot mix into ours."""
    return ((vi.CAN_PACKET_PROCESS_RX_BUFFER << 8) | vesc_id,
            bytes([vi.SENDER_ID, 2, 0, 1, RESET_CRC >> 8, RESET_CRC & 0xFF]))


def reboot_frame(vesc_id: int) -> Tuple[int, bytes]:
    """COMM_REBOOT, no reply (send mode 2)."""
    return ((vi.CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | vesc_id,
            bytes([vi.SENDER_ID, 2, COMM_REBOOT]))


def build_write_frames(vesc_id: int, payload: bytes) -> List[Tuple[int, bytes]]:
    n = len(payload)
    if not 6 < n <= vi.MAX_BUFFER_LEN:
        raise ValueError(f"payload length {n} out of range")
    frames: List[Tuple[int, bytes]] = []
    end_a = 0
    i = 0
    while i < n:
        if i > 255:
            break
        end_a = i + 7
        frames.append(((vi.CAN_PACKET_FILL_RX_BUFFER << 8) | vesc_id,
                       bytes([i]) + payload[i:i + 7]))
        i += 7
    i = end_a
    while i < n:
        frames.append(((vi.CAN_PACKET_FILL_RX_BUFFER_LONG << 8) | vesc_id,
                       bytes([i >> 8, i & 0xFF]) + payload[i:i + 6]))
        i += 6
    crc = vi.crc16(payload)
    frames.append(((vi.CAN_PACKET_PROCESS_RX_BUFFER << 8) | vesc_id,
                   bytes([vi.SENDER_ID, 0, n >> 8, n & 0xFF, crc >> 8, crc & 0xFF])))
    return frames


def receiver_reassemble(frames: Sequence[Tuple[int, bytes]], vesc_id: int) -> bytes:
    """What a VESC with this id would rebuild from the frames (strict model:
    in-order fills, then one PROCESS with matching length and CRC)."""
    buf = bytearray()
    for idx, (arb, data) in enumerate(frames):
        if arb & 0xFF != vesc_id:
            raise WriteGuardViolation(f"frame {idx} is addressed to {arb & 0xFF}")
        pkt = arb >> 8
        data = bytes(data)
        last = idx == len(frames) - 1
        if pkt == vi.CAN_PACKET_FILL_RX_BUFFER and not last:
            off, chunk = data[0], data[1:]
        elif pkt == vi.CAN_PACKET_FILL_RX_BUFFER_LONG and not last:
            off, chunk = (data[0] << 8) | data[1], data[2:]
        elif pkt == vi.CAN_PACKET_PROCESS_RX_BUFFER and last:
            if len(data) != 6 or data[0] != vi.SENDER_ID or data[1] != 0:
                raise WriteGuardViolation(f"bad PROCESS frame {data.hex()}")
            n = (data[2] << 8) | data[3]
            crc = (data[4] << 8) | data[5]
            if n != len(buf) or crc != vi.crc16(bytes(buf)):
                raise WriteGuardViolation("PROCESS length or CRC does not match the fills")
            return bytes(buf)
        else:
            raise WriteGuardViolation(f"frame {idx}: packet type {pkt} not allowed here")
        if off != len(buf) or not chunk or len(data) > 8:
            raise WriteGuardViolation(f"frame {idx}: offset {off} after {len(buf)} bytes")
        buf += chunk
    raise WriteGuardViolation("no PROCESS_RX_BUFFER frame")


def guard_write(vesc_id: int, frames: Sequence[Tuple[int, bytes]], intended_body: bytes) -> None:
    """Final check of a complete write before the first frame goes out."""
    if vesc_id not in TARGETS:
        raise WriteGuardViolation(f"refusing to write to CAN id {vesc_id}")
    payload = receiver_reassemble(frames, vesc_id)
    if len(payload) != WRITE_PAYLOAD_LEN or payload[0] != COMM_SET_MCCONF:
        raise WriteGuardViolation("the frames do not carry one COMM_SET_MCCONF")
    if payload[1:] != bytes(intended_body):
        raise WriteGuardViolation("the frames do not carry the proven config")
    if struct.unpack(">I", payload[1:5])[0] != vi.MCCONF_SIGNATURE_6_05:
        raise WriteGuardViolation("signature is not FW 6.05")


def guard_single(vesc_id: int, frame: Tuple[int, bytes]) -> None:
    """The only two non-buffer frames this tool may send."""
    if vesc_id not in TARGETS:
        raise WriteGuardViolation(f"refusing to send to CAN id {vesc_id}")
    if frame not in (reset_frame(vesc_id), reboot_frame(vesc_id)):
        raise WriteGuardViolation(f"refusing frame {frame[0]:X} {bytes(frame[1]).hex()}")


# ---------------------------------------------------------------------------
# Bus operations
# ---------------------------------------------------------------------------

class Writer:
    """Reads through vesc_info's read-only client; writes one config at a time."""

    def __init__(self, bus: Any, message_factory: Callable[..., Any],
                 timeout_s: float = 1.0, retries: int = 2,
                 sleep: Callable[[float], None] = time.sleep,
                 ack_timeout_s: Optional[float] = None) -> None:
        self.bus = bus
        self.factory = message_factory
        self.client = vi.VescClient(bus, message_factory, timeout_s=timeout_s, retries=retries)
        self.sleep = sleep
        self.ack_timeout_s = ACK_TIMEOUT_S if ack_timeout_s is None else ack_timeout_s

    # -- reads (read-only guarded inside vesc_info) --
    def fw(self, vid: int) -> Dict[str, Any]:
        return vi.parse_fw_version(self.client.request(vid, vi.COMM_FW_VERSION))

    def values(self, vid: int) -> Dict[str, Any]:
        return vi.parse_get_values(self.client.request(vid, vi.COMM_GET_VALUES))

    def still_values(self, vid: int) -> Dict[str, Any]:
        self.values(vid)              # resets the VESC's current average
        self.sleep(SETTLE_S)
        return self.values(vid)

    def body(self, vid: int) -> bytes:
        payload = self.client.request(vid, vi.COMM_GET_MCCONF)
        vi.decode_mcconf(payload)  # raises on a wrong signature / short reply
        if len(payload) != WRITE_PAYLOAD_LEN:
            raise vi.DecodeError(f"MCCONF reply is {len(payload)} bytes, expected "
                                 f"{WRITE_PAYLOAD_LEN}")
        return bytes(payload[1:])

    # -- sends --
    def _send_checked(self, arb: int, data: bytes) -> None:
        msg = self.factory(arbitration_id=arb, data=data, is_extended_id=True)
        if (int(msg.arbitration_id) != arb or bytes(msg.data) != data
                or not bool(getattr(msg, "is_extended_id", True))):
            raise WriteGuardViolation("message object differs from the checked frame")
        self.bus.send(msg)

    def send_config(self, vid: int, body: bytes) -> bool:
        """Reset the VESC's receive buffers, then send COMM_SET_MCCONF with
        `body`. True if the 1-byte ack arrived."""
        payload = bytes([COMM_SET_MCCONF]) + bytes(body)
        frames = build_write_frames(vid, payload)
        guard_write(vid, frames, body)
        reset = reset_frame(vid)
        guard_single(vid, reset)
        self.client._drain()
        self._send_checked(*reset)
        self.sleep(FRAME_GAP_S)
        reasm = vi.Reassembler()
        for arb, data in frames:
            self._send_checked(arb, data)
            self.sleep(FRAME_GAP_S)
        deadline = time.monotonic() + self.ack_timeout_s
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            try:
                res = reasm.feed(int(msg.arbitration_id), bytes(msg.data),
                                 bool(getattr(msg, "is_extended_id", True)))
            except vi.ReassemblyError:
                continue
            if res is not None and res[0] == vid and res[1] == bytes([COMM_SET_MCCONF]):
                return True
        return False

    def reboot_and_wait(self, vid: int, expected_uuid: str) -> str:
        """COMM_REBOOT, then poll FW_VERSION. The VESC must go silent at least
        once (it reset) and then answer as the same unit.

        Returns 'ok', 'no_reboot' (it kept answering: the reboot did not
        happen) or 'timeout' (it went silent and did not come back)."""
        frame = reboot_frame(vid)
        guard_single(vid, frame)
        quick = vi.VescClient(self.bus, self.factory, timeout_s=QUICK_TIMEOUT_S, retries=0)
        self.client._drain()
        self._send_checked(*frame)
        t0 = time.monotonic()
        saw_silence = False
        while True:
            elapsed = time.monotonic() - t0
            if elapsed > REBOOT_WAIT_S or (not saw_silence and elapsed > REBOOT_NO_SILENCE_S):
                return "timeout" if saw_silence else "no_reboot"
            try:
                fw = vi.parse_fw_version(quick.request(vid, vi.COMM_FW_VERSION))
            except (vi.QueryTimeout, vi.DecodeError):
                saw_silence = True
                continue
            if not saw_silence:
                self.sleep(0.05)
                continue
            if fw.get("uuid") == expected_uuid:
                self.sleep(REBOOT_SETTLE_S)
                return "ok"
            self.sleep(REBOOT_POLL_S)

    def body_after_boot(self, vid: int, before: bytes) -> Optional[bytes]:
        """GET_MCCONF until the boot calibration has changed an offset (proof
        of a real reboot). None if no offset changed within OFFSETS_WAIT_S."""
        deadline = time.monotonic() + OFFSETS_WAIT_S
        while True:
            body = self.body(vid)
            if any(field_bytes(body, f) != field_bytes(before, f) for f in REBOOT_PROOF_FIELDS):
                return body
            if time.monotonic() > deadline:
                return None
            self.sleep(REBOOT_POLL_S)


def service_state(runner: Callable[..., Any] = subprocess.run) -> str:
    try:
        r = runner(["systemctl", "is-active", SERVICE], capture_output=True,
                   text=True, timeout=5)
    except (OSError, subprocess.SubprocessError) as exc:
        return f"unknown ({exc})"
    out = (r.stdout or "").strip()
    return out or f"unknown (rc {r.returncode})"


def fw_refusals(vid: int, fw: Dict[str, Any]) -> List[str]:
    out = []
    if fw.get("hw_name") != EXPECTED_HW:
        out.append(f"hardware is {fw.get('hw_name')!r}, expected {EXPECTED_HW}")
    if (fw.get("fw_major"), fw.get("fw_minor")) != EXPECTED_FW:
        out.append(f"firmware is {fw.get('fw_version')}, expected 6.05")
    if fw.get("fw_test_version"):
        out.append(f"firmware is a test build ({fw.get('fw_test_version')})")
    if fw.get("uuid") != EXPECTED_UUID.get(vid):
        out.append(f"unit uuid {fw.get('uuid')} is not the one pinned for id {vid} "
                   f"({EXPECTED_UUID.get(vid)})")
    return out


def values_refusals(vals: Dict[str, Any]) -> List[str]:
    out = []
    if abs(vals["erpm"]) >= MAX_ERPM_STILL:
        out.append(f"motor turning: {vals['erpm']:.0f} erpm")
    if abs(vals["avg_input_current_a"]) >= MAX_INPUT_CURRENT_STILL_A:
        out.append(f"input current {vals['avg_input_current_a']:.2f} A")
    if vals["fault_code"] != 0:
        out.append(f"fault {vals['fault_name']}")
    if not V_IN_MIN < vals["v_in"] < V_IN_MAX:
        out.append(f"bus voltage {vals['v_in']:.1f} V is implausible")
    if vals.get("timeout_active") is not True:
        out.append("the VESC is still receiving motor commands (its command "
                   "timeout is not active)" if "timeout_active" in vals else
                   "GET_VALUES has no status byte (command timeout unknown)")
    return out


def env_refusals(service_probe: Callable[[], str], hold_path: str) -> List[str]:
    out = []
    st = service_probe()
    if st not in SERVICE_STATES_OK:
        out.append(f"{SERVICE} is {st} (stop it first)")
    if not os.path.exists(hold_path):
        out.append(f"auto-deploy hold {hold_path} is missing (touch it first)")
    return out


def gate_now(w: Writer, vid: int, service_probe: Callable[[], str],
             hold_path: str) -> List[str]:
    """Every gate that can change between reads, checked again now."""
    return env_refusals(service_probe, hold_path) + values_refusals(w.still_values(vid))


def live_warnings(vals: Dict[str, Any], targets: Dict[str, float]) -> List[str]:
    out = []
    v = vals["v_in"]
    if "l_battery_regen_cut_start" in targets and v >= targets["l_battery_regen_cut_start"]:
        out.append(f"the pack ({v:.1f} V) is above the new regen cutoff start: "
                   "regen braking is limited until the pack falls")
    if "l_battery_cut_start" in targets and v <= targets["l_battery_cut_start"]:
        out.append(f"the pack ({v:.1f} V) is below the new cutoff start: "
                   "drive current is limited at once")
    return out


def _fmt(v: Any) -> str:
    return vi._num(v)


# ---------------------------------------------------------------------------
# Backups
# ---------------------------------------------------------------------------

def _sha(body: bytes) -> str:
    return hashlib.sha256(bytes(body)).hexdigest()


def _write_excl(path: str, data: bytes) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        os.write(fd, data)
        os.fsync(fd)
    finally:
        os.close(fd)


def write_backup(backup_dir: str, vid: int, fw: Dict[str, Any], body: bytes,
                 kind: str = "pre-write",
                 now: Optional[datetime.datetime] = None) -> str:
    """kind: 'pre-write' (before a part A write), 'pre-restore' or 'pre-verify'.
    Only 'pre-write' backups serve as the reference config (config_refusals)."""
    now = now or datetime.datetime.now()
    os.makedirs(backup_dir, exist_ok=True)
    stem = (f"mcconf_id{vid}_{vi.SIDE_NAMES.get(vid, 'x')}_{fw['uuid']}_"
            f"{now.strftime('%Y%m%d_%H%M%S_%f')}")
    bin_path = os.path.join(backup_dir, stem + ".bin")
    json_path = bin_path[:-4] + ".json"
    meta = {"vesc_id": vid, "side": vi.SIDE_NAMES.get(vid), "uuid": fw["uuid"],
            "hw_name": fw["hw_name"], "fw_version": fw["fw_version"], "kind": kind,
            "signature": vi.MCCONF_SIGNATURE_6_05, "body_len": len(body),
            "sha256": _sha(body), "saved": now.isoformat(timespec="microseconds"),
            "key_fields": {n: decode_body(body)[n] for n, _u, _d in vi.MCCONF_KEY_FIELDS}}
    _write_excl(bin_path, bytes(body))
    _write_excl(json_path, (json.dumps(vi._json_safe(meta), indent=2) + "\n").encode())
    dfd = os.open(backup_dir, os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)
    back, back_meta = load_backup(bin_path)
    if back != bytes(body) or back_meta.get("vesc_id") != vid:
        raise WriteRefused(f"backup {bin_path} did not read back identical")
    return bin_path


def load_backup(path: str) -> Tuple[bytes, Dict[str, Any]]:
    with open(path, "rb") as fh:
        body = fh.read()
    meta_path = path[:-4] + ".json" if path.endswith(".bin") else path + ".json"
    with open(meta_path, "r", encoding="utf-8") as fh:
        meta = json.load(fh)
    if len(body) != BODY_LEN:
        raise WriteRefused(f"backup is {len(body)} bytes, expected {BODY_LEN}")
    if struct.unpack(">I", body[:4])[0] != vi.MCCONF_SIGNATURE_6_05:
        raise WriteRefused("backup signature is not FW 6.05")
    if meta.get("sha256") != _sha(body):
        raise WriteRefused("backup .bin does not match the SHA-256 in its .json")
    decode_body(body)
    return body, meta


def _fields_known(vid: int, body: bytes) -> bool:
    """Each of the 7 fields holds its documented value or its target."""
    return all(field_bytes(body, n) in (encode_field(n, BEFORE[vid][n]),
                                        encode_field(n, TARGETS[vid][n]))
               for n in TARGET_FIELDS)


def newest_backup(backup_dir: str, vid: int,
                  uuid: str) -> Optional[Tuple[bytes, Dict[str, Any], str]]:
    """Newest good reference (body, meta, path): a 'pre-write' backup of this
    unit whose 7 fields were known values (a backup of a defaults boot never
    becomes the reference)."""
    pattern = os.path.join(backup_dir, f"mcconf_id{vid}_*_{uuid}_*.bin")
    for path in sorted(glob.glob(pattern), reverse=True):
        try:
            body, meta = load_backup(path)
        except (OSError, ValueError, WriteRefused):
            continue
        if (meta.get("uuid") == uuid and meta.get("vesc_id") == vid
                and meta.get("kind", "pre-write") == "pre-write"
                and _fields_known(vid, body)):
            return body, meta, path
    return None


def differs_from(ref: bytes, body: bytes, skip: Sequence[str] = ()) -> List[str]:
    """Fields where body differs from ref, allowing the firmware's one-step
    f16 re-quantization of ref."""
    drifted = apply_predicted_drift(ref)
    return [n for n in changed_fields(ref, body, skip)
            if field_bytes(body, n) != field_bytes(drifted, n)]


# ---------------------------------------------------------------------------
# One VESC: plan, gate, write, verify live, reboot, verify flash
# ---------------------------------------------------------------------------

def plan_lines(vid: int, fw: Dict[str, Any], vals: Dict[str, Any], old: bytes,
               new: bytes, targets: Optional[Dict[str, float]]) -> List[str]:
    lines = [f"=== VESC id {vid} ({vi.SIDE_NAMES.get(vid, '?')}): {fw.get('hw_name')} "
             f"FW {fw.get('fw_version')}, uuid {fw.get('uuid')} ===",
             f"  live: v_in {_fmt(vals['v_in'])} V, erpm {_fmt(vals['erpm'])}, "
             f"input current {_fmt(vals['avg_input_current_a'])} A, fault {vals['fault_name']}, "
             f"command timeout {'active' if vals.get('timeout_active') else 'NOT active'}"]
    d_old, d_new = decode_body(old), decode_body(new)
    names = list(targets) if targets is not None else changed_fields(old, new)
    lines.append(f"  {'field':28s} {'now':>10s} {'new':>10s}")
    for name in names:
        unit = UNITS.get(name, "")
        mark = "" if field_bytes(old, name) == field_bytes(new, name) else "   CHANGE"
        lines.append(f"  {name:28s} {_fmt(d_old[name]):>10s} {_fmt(d_new[name]):>10s} "
                     f"{unit:1s}{mark}")
    ch = changed_fields(old, new)
    nbytes = sum(1 for a, b in zip(old, new) if a != b)
    lines.append(f"  {len(ch)} field(s) change, {nbytes} byte(s); every other field is "
                 f"sent back in the exact bytes the VESC reported")
    drift = predicted_drift(new)
    if drift:
        lines.append(f"  NOTE: the firmware will re-quantize {len(drift)} untouched "
                     f"field(s) by one step (VESC Tool does the same): {', '.join(drift)}")
    return lines


def _restore_hint(path: str, vid: int) -> str:
    return (f"  to put the old config back: /home/pi/.venv/bin/python3 "
            f"tools/vesc_battery_limits.py --restore {path} --id {vid} --write")


def _state_now(w: Writer, vid: int, old: bytes, expected: bytes) -> str:
    try:
        now_body = w.body(vid)
    except Exception as exc:  # noqa: BLE001
        return f"read-back also failed: {exc}"
    if not changed_fields(expected, now_body, BOOT_MEASURED):
        return "the VESC now reports the NEW config (live); the stored copy is NOT proven"
    if not changed_fields(old, now_body, BOOT_MEASURED):
        return "the VESC now reports the OLD config (live)"
    return "the VESC now reports a DIFFERENT config (live)"


def process_one(w: Writer, vid: int, *, write: bool, backup_dir: str,
                service_probe: Callable[[], str], hold_path: str,
                accept_live: bool = False,
                restore: Optional[Tuple[bytes, Dict[str, Any]]] = None,
                out: Callable[[str], None] = print) -> Tuple[str, Optional[str]]:
    """Returns (status, backup path). Status: 'ok' (live and stored proven),
    'noop' (dry run, live already at target), 'dry', 'refused' (nothing
    changed), 'fault' (config proven, but the VESC reports a fault) or
    'failed' (changed, or state unknown, or stored copy unproven)."""
    fw = w.fw(vid)
    vals = w.still_values(vid)
    old = w.body(vid)

    reasons = fw_refusals(vid, fw)
    if decode_body(old).get("foc_offsets_cal_on_boot") != 1:
        reasons.append("foc_offsets_cal_on_boot is not 1, so the reboot proof cannot work")
    if restore is not None:
        new, meta = restore
        targets: Optional[Dict[str, float]] = None
        kind = "pre-restore"
        if meta.get("uuid") != fw.get("uuid") or meta.get("vesc_id") != vid:
            out(f"VESC id {vid}: REFUSED: backup is for id {meta.get('vesc_id')} uuid "
                f"{meta.get('uuid')}, this VESC is uuid {fw.get('uuid')}")
            return "refused", None
    else:
        targets = TARGETS[vid]
        kind = "pre-write"
        new = patch_body(old, targets)
        try:
            prove_patch(old, new, targets)
        except WriteRefused as exc:
            out(f"VESC id {vid}: REFUSED: {exc}")
            return "refused", None
        reasons += config_refusals(vid, old, backup_dir, str(fw.get("uuid")), accept_live)

    for ln in plan_lines(vid, fw, vals, old, new, targets):
        out(ln)
    for wmsg in live_warnings(vals, targets or {}):
        out(f"  WARNING: {wmsg}")

    reasons = env_refusals(service_probe, hold_path) + values_refusals(vals) + reasons
    at_target = new == old
    if not write:
        if at_target:
            out("  live config already at target (a --write run proves the stored copy "
                "with a reboot)")
            return "noop", None
        if reasons:
            out("  a write now would be REFUSED: " + "; ".join(reasons))
        else:
            out("  gates pass: a write would go ahead")
        return "dry", None
    if reasons:
        out("  REFUSED: " + "; ".join(reasons))
        return "refused", None
    if at_target:
        out("  live config already at target: no write; proving the stored copy "
            "with a reboot")
        kind = "pre-verify"

    try:
        path = write_backup(backup_dir, vid, fw, old, kind=kind)
    except (OSError, ValueError, WriteRefused) as exc:
        out(f"  REFUSED: backup failed: {exc}")
        return "refused", None
    out(f"  backup: {path}")
    if at_target and restore is None:
        # The pre-verify backup holds the new limits; undo means the newest
        # pre-write backup (the config before part A).
        ref0 = newest_backup(backup_dir, vid, str(fw.get("uuid")))
        path = ref0[2] if ref0 is not None else "<no pre-write backup found>"

    expected = old if at_target else apply_predicted_drift(new)
    live_is_new = at_target
    try:
        back = old
        attempts = 0
        while not at_target:
            attempts += 1
            again = gate_now(w, vid, service_probe, hold_path)
            if again:
                out("  REFUSED just before sending: " + "; ".join(again))
                return "refused", path
            acked = w.send_config(vid, new)
            out(f"  write {attempts}: {'ack received' if acked else 'NO ACK'}")
            w.sleep(SETTLE_S)
            back = w.body(vid)
            if back == expected:
                live_is_new = True
                break
            if back == old and attempts == 1:
                out("  read-back shows the old config: the write did not arrive; "
                    "sending once more")
                continue
            if back == old:
                out("  NOT WRITTEN: the write did not arrive twice; the VESC still has "
                    "its old config (nothing changed)")
                return "refused", path
            out("  FAILED: read-back differs from what was written:")
            d_back, d_exp = decode_body(back), decode_body(expected)
            for name in changed_fields(expected, back):
                out(f"    {name}: wrote {_fmt(d_exp[name])}, reads {_fmt(d_back[name])}")
            out(_restore_hint(path, vid))
            return "failed", path
        if not at_target:
            out("  live config verified; rebooting the VESC to check the stored copy")

        again = gate_now(w, vid, service_probe, hold_path)
        if again:
            out("  FAILED: not rebooting: " + "; ".join(again))
            out("  the live config is the new one, but the stored copy is NOT proven")
            return "failed", path
        rb = w.reboot_and_wait(vid, str(fw.get("uuid")))
        if rb == "no_reboot":
            out("  FAILED: the VESC kept answering, so it did not reboot. The live config "
                "is the new one, but the stored copy is NOT proven.")
            return "failed", path
        if rb != "ok":
            out(f"  FAILED: the VESC went silent and did not answer as uuid {fw.get('uuid')} "
                f"within {REBOOT_WAIT_S:g} s; check its power and CAN id, then run the "
                "dry run")
            return "failed", path
        stored = w.body_after_boot(vid, back)
        if stored is None:
            out("  FAILED: no boot offset changed, so a reboot is not proven. The live "
                "config is the new one, but the stored copy is NOT proven.")
            return "failed", path
        diff = changed_fields(expected, stored, BOOT_MEASURED)
        if diff:
            if not at_target and not changed_fields(old, stored, BOOT_MEASURED):
                out("  NOT STORED: after the reboot the VESC is back on its OLD config: "
                    "the flash store did not take. Nothing changed; run --write again.")
                return "refused", path
            ref = newest_backup(backup_dir, vid, str(fw.get("uuid")))
            if (at_target and restore is None and ref is not None
                    and not differs_from(ref[0], stored, BOOT_MEASURED)):
                out("  NOT STORED: the live config was at target, but the flash still held "
                    "the OLD config (an earlier store did not take); the VESC is back on "
                    "it now. Run --write again.")
                return "refused", path
            out("  FAILED: after the reboot the stored config differs from the expected "
                f"one (defaults loaded?): {', '.join(diff[:10])}")
            out(_restore_hint(path, vid))
            return "failed", path
        vals2 = w.values(vid)
        if vals2["fault_code"] != 0:
            out(f"  FAULT: the config is stored as written (proven by the reboot), but the "
                f"VESC reports {vals2['fault_name']}. Investigate before driving; no "
                "restore is needed for the config.")
            return "fault", path
    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001
        out(_restore_hint(path, vid))
        if isinstance(exc, WriteGuardViolation) and not live_is_new:
            out(f"  REFUSED: the tool's own guard stopped a frame (a bug): {exc}. No "
                "config was applied.")
            return "refused", path
        out(f"  FAILED: state UNKNOWN after {type(exc).__name__}: {exc}")
        out("  " + _state_now(w, vid, old, expected))
        return "failed", path
    out("  VERIFIED: live and stored (after a reboot) config read back as expected")
    return "ok", path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Set part A (battery limits) on the drive VESCs. Dry run unless --write.")
    p.add_argument("--id", type=int, action="append", dest="ids", metavar="CAN_ID",
                   help="VESC to handle (repeatable; default 2 then 1)")
    p.add_argument("--write", action="store_true",
                   help="really write (wall-e stopped, deploy hold set, robot disarmed and still)")
    p.add_argument("--restore", metavar="BACKUP.bin",
                   help="write a backup made by this tool back to its own VESC")
    p.add_argument("--accept-live", action="store_true",
                   help="accept a live config that differs from the newest backup")
    p.add_argument("--backup-dir", default=DEFAULT_BACKUP_DIR)
    p.add_argument("--deploy-hold", default=DEFAULT_DEPLOY_HOLD)
    p.add_argument("--channel", default="can0")
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--retries", type=int, default=2)
    args = p.parse_args(argv)
    args.ids = args.ids or list(vi.DEFAULT_IDS)
    for vid in args.ids:
        if vid not in TARGETS:
            p.error(f"CAN id {vid} is not a drive VESC (1 or 2)")
    if len(set(args.ids)) != len(args.ids):
        p.error("each --id once")
    if args.restore and len(args.ids) != 1:
        p.error("--restore needs exactly one --id")
    return args


def main(argv: Optional[Sequence[str]] = None,
         bus_opener: Callable[[str], Tuple[Any, Any]] = vi.open_bus,
         service_probe: Callable[[], str] = service_state,
         sleep: Callable[[float], None] = time.sleep) -> int:
    args = _parse_args(argv)
    check_targets_table()
    restore = None
    if args.restore:
        try:
            restore = load_backup(args.restore)
        except (OSError, ValueError, WriteRefused) as exc:
            print(f"vesc_battery_limits: cannot use backup: {exc}", file=sys.stderr)
            return 1
    try:
        bus, factory = bus_opener(args.channel)
    except Exception as exc:  # noqa: BLE001
        print(f"vesc_battery_limits: cannot open {args.channel}: {exc}", file=sys.stderr)
        return 2
    results: List[Tuple[int, str, Optional[str]]] = []
    interrupted = False
    try:
        w = Writer(bus, factory, timeout_s=args.timeout, retries=args.retries, sleep=sleep)
        for vid in args.ids:
            try:
                res, path = process_one(w, vid, write=args.write, backup_dir=args.backup_dir,
                                        service_probe=service_probe, hold_path=args.deploy_hold,
                                        accept_live=args.accept_live, restore=restore)
            except Exception as exc:  # noqa: BLE001 - escapes only before any write
                print(f"VESC id {vid}: {type(exc).__name__} before any write, nothing "
                      f"written: {exc}")
                res, path = "refused", None
            results.append((vid, res, path))
            print()
            if args.write and res in ("refused", "failed"):
                if vid != args.ids[-1]:
                    print("stopping: the remaining VESCs were not touched")
                break
    except KeyboardInterrupt:
        print("vesc_battery_limits: interrupted", file=sys.stderr)
        interrupted = True
    finally:
        try:
            bus.shutdown()
        except Exception:  # noqa: BLE001
            pass

    statuses = [r for _v, r, _p in results]
    if not args.write:
        print("DRY RUN: nothing was written.")
        if restore is None and "noop" in statuses and "dry" in statuses:
            print("WARNING: PAIR SPLIT now: one VESC is at target and the other is not. "
                  "Do not drive until both match.")
            return 4
        return 0 if all(r in ("dry", "noop") for r in statuses) else 1

    if restore is not None:
        if statuses and statuses[0] in ("ok", "noop"):
            print("RESTORED. WARNING: the other VESC was not changed, so the two may "
                  "have DIFFERENT battery limits. Do not drive until the dry run shows "
                  "both match.")
            return 4
        return 130 if interrupted else 3 if "failed" in statuses else 1

    written = [(v, p) for v, r, p in results if r in ("ok", "fault")]
    failed = "failed" in statuses
    at_target = {v for v, r, _p in results if r in ("ok", "fault")}
    for v, r, _p in results:
        if r == "fault":
            print(f"WARNING: id {v} has the new limits stored, but reports a fault. "
                  "Do not drive until it is cleared.")
    split = (bool(at_target) and at_target != set(vi.DEFAULT_IDS)) or failed or \
        (bool(written) and interrupted)
    if split:
        print("WARNING: PAIR SPLIT. The two VESCs may now have DIFFERENT battery limits. "
              "Do not drive until both match.")
        for v, p in written:
            print(f"  id {v} has the new limits; to undo it: --restore {p} --id {v} --write")
        for v, r, p in results:
            if r == "failed":
                print(f"  id {v}: write FAILED, state uncertain (see above); backup {p}")
        missing = sorted(set(vi.DEFAULT_IDS) - at_target)
        print(f"  not at target (or not checked in this run): id "
              f"{', '.join(map(str, missing))}")
        return 3 if failed else 4
    if interrupted:
        return 130
    if "fault" in statuses:
        return 3
    if "refused" in statuses:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
