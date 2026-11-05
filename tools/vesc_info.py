#!/usr/bin/env python3
"""
vesc_info.py — READ-ONLY VESC query tool over SocketCAN.

Prints, for each drive VESC (left CAN id 2, right CAN id 1):
  * COMM_FW_VERSION (0): hardware name, firmware version, UUID.
  * COMM_GET_VALUES (4): input voltage, temperatures, input/motor current,
    Ah / Wh (used and charged), fault code.
  * COMM_GET_MCCONF (14): the motor configuration, decoded with the FW 6.05
    field layout, including the voltage limits and the Battery Voltage Regen
    Cutoff (l_battery_regen_cut_start / _end).

SAFETY — this tool must never move a motor or change a setting:
  * The only CAN packet type it ever transmits is
    CAN_PACKET_PROCESS_SHORT_BUFFER (8), arbitration id (8 << 8) | vesc_id,
    extended, data [SENDER_ID, 0, cmd].
  * cmd must be in READ_ONLY_COMMANDS = {0, 4, 14}. `guard_frame()` re-checks
    the packet type, the payload and the allowlist immediately before every
    bus.send() and raises ReadOnlyViolation (an AssertionError that is NOT
    stripped by `python -O`) otherwise. No SET_* CAN packet and no COMM_SET_*
    command exists anywhere in this file's send path.
  * Broadcast id 255 and our own SENDER_ID are refused as targets.

Running beside wall-e.service: CAN arbitrates per frame, so a second process
may send these read-only queries while the control loop owns the bus. The
replies are addressed to SENDER_ID (0x7E), which wall-e ignores (it only
reads STATUS frames from ids 1 and 2). The only side effect on the VESC is
that GET_VALUES resets its averaged-current accumulators (mc_interface
read_reset_avg_*), exactly as VESC Tool's realtime view does; motor control
does not use them.

Protocol source: vedderb/bldc, branch release_6_05 —
  comm/comm_can.c   comm_can_send_buffer(), decode_msg()
  comm/commands.c   COMM_FW_VERSION, COMM_GET_VALUES, commands_send_mcconf()
  confgenerator.c   confgenerator_deserialize_mcconf(), MCCONF_SIGNATURE
  util/buffer.c     buffer_get_float16 / float32 / float32_auto
  util/crc.c        crc16 (CRC16 XMODEM: poly 0x1021, init 0)
Replies longer than 6 bytes arrive as FILL_RX_BUFFER (5, 1-byte offset, for
offsets 0..252), then FILL_RX_BUFFER_LONG (6, 2-byte offset), then
PROCESS_RX_BUFFER (7: src, send, len_hi, len_lo, crc_hi, crc_lo). Replies of
6 bytes or less arrive as one PROCESS_SHORT_BUFFER (8: src, send, data...).

The MCCONF decode refuses to run unless the reply's first 4 bytes equal the
FW 6.05 MCCONF_SIGNATURE (1065524471); on a mismatch it prints the raw reply
length and the signature it got instead.

Usage (on the Pi):
  /home/pi/.venv/bin/python3 tools/vesc_info.py
  /home/pi/.venv/bin/python3 tools/vesc_info.py --id 2 --json
  /home/pi/.venv/bin/python3 tools/vesc_info.py --all      # every MCCONF field
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Protocol constants (bldc release_6_05 datatypes.h / comm_can.c)
# ---------------------------------------------------------------------------

SENDER_ID = 0x7E  # our CAN id; VESC replies are addressed to it
CAN_BROADCAST_ID = 255

CAN_PACKET_FILL_RX_BUFFER = 5
CAN_PACKET_FILL_RX_BUFFER_LONG = 6
CAN_PACKET_PROCESS_RX_BUFFER = 7
CAN_PACKET_PROCESS_SHORT_BUFFER = 8

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_GET_MCCONF = 14

# The ONLY commands this tool may send. Every one is a pure read.
READ_ONLY_COMMANDS = frozenset({COMM_FW_VERSION, COMM_GET_VALUES, COMM_GET_MCCONF})

COMMAND_NAMES = {
    COMM_FW_VERSION: "COMM_FW_VERSION",
    COMM_GET_VALUES: "COMM_GET_VALUES",
    COMM_GET_MCCONF: "COMM_GET_MCCONF",
}

# Firmware RX_BUFFER_SIZE = PACKET_MAX_PL_LEN = 512.
MAX_BUFFER_LEN = 512

# confgenerator.h (release_6_05)
MCCONF_SIGNATURE_6_05 = 1065524471

DEFAULT_IDS = (2, 1)
SIDE_NAMES = {2: "left", 1: "right"}

# datatypes.h mc_fault_code (release_6_05)
FAULT_CODES = (
    "FAULT_CODE_NONE",
    "FAULT_CODE_OVER_VOLTAGE",
    "FAULT_CODE_UNDER_VOLTAGE",
    "FAULT_CODE_DRV",
    "FAULT_CODE_ABS_OVER_CURRENT",
    "FAULT_CODE_OVER_TEMP_FET",
    "FAULT_CODE_OVER_TEMP_MOTOR",
    "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
    "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
    "FAULT_CODE_MCU_UNDER_VOLTAGE",
    "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
    "FAULT_CODE_ENCODER_SPI",
    "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    "FAULT_CODE_FLASH_CORRUPTION",
    "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
    "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
    "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
    "FAULT_CODE_UNBALANCED_CURRENTS",
    "FAULT_CODE_BRK",
    "FAULT_CODE_RESOLVER_LOT",
    "FAULT_CODE_RESOLVER_DOS",
    "FAULT_CODE_RESOLVER_LOS",
    "FAULT_CODE_FLASH_CORRUPTION_APP_CFG",
    "FAULT_CODE_FLASH_CORRUPTION_MC_CFG",
    "FAULT_CODE_ENCODER_NO_MAGNET",
    "FAULT_CODE_ENCODER_MAGNET_TOO_STRONG",
    "FAULT_CODE_PHASE_FILTER",
    "FAULT_CODE_ENCODER_FAULT",
    "FAULT_CODE_LV_OUTPUT_FAULT",
)

# ---------------------------------------------------------------------------
# MCCONF layout — confgenerator_deserialize_mcconf(), bldc release_6_05.
# (name, kind, scale). Kinds:
#   u8   buffer[ind++]                         i8   (int8_t)buffer[ind++]
#   u16  buffer_get_uint16                     i32  buffer_get_int32
#   u32  buffer_get_uint32                     f32a buffer_get_float32_auto
#   f16  buffer_get_float16(buffer, scale)  -> int16 / scale
# Array members are flattened as name_0.._n; "bms.x" is bms_x.
# 196 fields, 477 bytes including the 4-byte signature.
# ---------------------------------------------------------------------------

MCCONF_FIELDS_6_05: Tuple[Tuple[str, str, Optional[float]], ...] = (
    ("pwm_mode", "u8", None),
    ("comm_mode", "u8", None),
    ("motor_type", "u8", None),
    ("sensor_mode", "u8", None),
    ("l_current_max", "f32a", None),
    ("l_current_min", "f32a", None),
    ("l_in_current_max", "f32a", None),
    ("l_in_current_min", "f32a", None),
    ("l_in_current_map_start", "f16", 10000.0),
    ("l_in_current_map_filter", "f16", 10000.0),
    ("l_abs_current_max", "f32a", None),
    ("l_min_erpm", "f32a", None),
    ("l_max_erpm", "f32a", None),
    ("l_erpm_start", "f16", 10000.0),
    ("l_max_erpm_fbrake", "f32a", None),
    ("l_max_erpm_fbrake_cc", "f32a", None),
    ("l_min_vin", "f16", 10.0),
    ("l_max_vin", "f16", 10.0),
    ("l_battery_cut_start", "f16", 10.0),
    ("l_battery_cut_end", "f16", 10.0),
    ("l_battery_regen_cut_start", "f16", 10.0),
    ("l_battery_regen_cut_end", "f16", 10.0),
    ("l_slow_abs_current", "u8", None),
    ("l_temp_fet_start", "u8", None),
    ("l_temp_fet_end", "u8", None),
    ("l_temp_motor_start", "u8", None),
    ("l_temp_motor_end", "u8", None),
    ("l_temp_accel_dec", "f16", 10000.0),
    ("l_min_duty", "f16", 10000.0),
    ("l_max_duty", "f16", 10000.0),
    ("l_watt_max", "f32a", None),
    ("l_watt_min", "f32a", None),
    ("l_current_max_scale", "f16", 10000.0),
    ("l_current_min_scale", "f16", 10000.0),
    ("l_duty_start", "f16", 10000.0),
    ("sl_min_erpm", "f32a", None),
    ("sl_min_erpm_cycle_int_limit", "f32a", None),
    ("sl_max_fullbreak_current_dir_change", "f32a", None),
    ("sl_cycle_int_limit", "f16", 10.0),
    ("sl_phase_advance_at_br", "f16", 10000.0),
    ("sl_cycle_int_rpm_br", "f32a", None),
    ("sl_bemf_coupling_k", "f32a", None),
    ("hall_table_0", "i8", None),
    ("hall_table_1", "i8", None),
    ("hall_table_2", "i8", None),
    ("hall_table_3", "i8", None),
    ("hall_table_4", "i8", None),
    ("hall_table_5", "i8", None),
    ("hall_table_6", "i8", None),
    ("hall_table_7", "i8", None),
    ("hall_sl_erpm", "f32a", None),
    ("foc_current_kp", "f32a", None),
    ("foc_current_ki", "f32a", None),
    ("foc_f_zv", "f32a", None),
    ("foc_dt_us", "f32a", None),
    ("foc_encoder_inverted", "u8", None),
    ("foc_encoder_offset", "f32a", None),
    ("foc_encoder_ratio", "f32a", None),
    ("foc_sensor_mode", "u8", None),
    ("foc_pll_kp", "f32a", None),
    ("foc_pll_ki", "f32a", None),
    ("foc_motor_l", "f32a", None),
    ("foc_motor_ld_lq_diff", "f32a", None),
    ("foc_motor_r", "f32a", None),
    ("foc_motor_flux_linkage", "f32a", None),
    ("foc_observer_gain", "f32a", None),
    ("foc_observer_gain_slow", "f32a", None),
    ("foc_observer_offset", "f16", 1000.0),
    ("foc_duty_dowmramp_kp", "f32a", None),
    ("foc_duty_dowmramp_ki", "f32a", None),
    ("foc_start_curr_dec", "f16", 10000.0),
    ("foc_start_curr_dec_rpm", "f32a", None),
    ("foc_openloop_rpm", "f32a", None),
    ("foc_openloop_rpm_low", "f16", 1000.0),
    ("foc_d_gain_scale_start", "f16", 1000.0),
    ("foc_d_gain_scale_max_mod", "f16", 1000.0),
    ("foc_sl_openloop_hyst", "f16", 100.0),
    ("foc_sl_openloop_time_lock", "f16", 100.0),
    ("foc_sl_openloop_time_ramp", "f16", 100.0),
    ("foc_sl_openloop_time", "f16", 100.0),
    ("foc_sl_openloop_boost_q", "f16", 100.0),
    ("foc_sl_openloop_max_q", "f16", 100.0),
    ("foc_hall_table_0", "u8", None),
    ("foc_hall_table_1", "u8", None),
    ("foc_hall_table_2", "u8", None),
    ("foc_hall_table_3", "u8", None),
    ("foc_hall_table_4", "u8", None),
    ("foc_hall_table_5", "u8", None),
    ("foc_hall_table_6", "u8", None),
    ("foc_hall_table_7", "u8", None),
    ("foc_hall_interp_erpm", "f32a", None),
    ("foc_sl_erpm_start", "f32a", None),
    ("foc_sl_erpm", "f32a", None),
    ("foc_control_sample_mode", "u8", None),
    ("foc_current_sample_mode", "u8", None),
    ("foc_sat_comp_mode", "u8", None),
    ("foc_sat_comp", "f16", 1000.0),
    ("foc_temp_comp", "u8", None),
    ("foc_temp_comp_base_temp", "f16", 100.0),
    ("foc_current_filter_const", "f16", 10000.0),
    ("foc_cc_decoupling", "u8", None),
    ("foc_observer_type", "u8", None),
    ("foc_hfi_voltage_start", "f16", 10.0),
    ("foc_hfi_voltage_run", "f16", 10.0),
    ("foc_hfi_voltage_max", "f16", 10.0),
    ("foc_hfi_gain", "f16", 1000.0),
    ("foc_hfi_max_err", "f16", 1000.0),
    ("foc_hfi_hyst", "f16", 100.0),
    ("foc_sl_erpm_hfi", "f32a", None),
    ("foc_hfi_start_samples", "u16", None),
    ("foc_hfi_obs_ovr_sec", "f32a", None),
    ("foc_hfi_samples", "u8", None),
    ("foc_offsets_cal_on_boot", "u8", None),
    ("foc_offsets_current_0", "f32a", None),
    ("foc_offsets_current_1", "f32a", None),
    ("foc_offsets_current_2", "f32a", None),
    ("foc_offsets_voltage_0", "f16", 10000.0),
    ("foc_offsets_voltage_1", "f16", 10000.0),
    ("foc_offsets_voltage_2", "f16", 10000.0),
    ("foc_offsets_voltage_undriven_0", "f16", 10000.0),
    ("foc_offsets_voltage_undriven_1", "f16", 10000.0),
    ("foc_offsets_voltage_undriven_2", "f16", 10000.0),
    ("foc_phase_filter_enable", "u8", None),
    ("foc_phase_filter_disable_fault", "u8", None),
    ("foc_phase_filter_max_erpm", "f32a", None),
    ("foc_mtpa_mode", "u8", None),
    ("foc_fw_current_max", "f32a", None),
    ("foc_fw_duty_start", "f16", 10000.0),
    ("foc_fw_ramp_time", "f16", 1000.0),
    ("foc_fw_q_current_factor", "f16", 10000.0),
    ("foc_speed_soure", "u8", None),
    ("foc_short_ls_on_zero_duty", "u8", None),
    ("sp_pid_loop_rate", "u8", None),
    ("s_pid_kp", "f32a", None),
    ("s_pid_ki", "f32a", None),
    ("s_pid_kd", "f32a", None),
    ("s_pid_kd_filter", "f16", 10000.0),
    ("s_pid_min_erpm", "f32a", None),
    ("s_pid_allow_braking", "u8", None),
    ("s_pid_ramp_erpms_s", "f32a", None),
    ("s_pid_speed_source", "u8", None),
    ("p_pid_kp", "f32a", None),
    ("p_pid_ki", "f32a", None),
    ("p_pid_kd", "f32a", None),
    ("p_pid_kd_proc", "f32a", None),
    ("p_pid_kd_filter", "f16", 10000.0),
    ("p_pid_ang_div", "f32a", None),
    ("p_pid_gain_dec_angle", "f16", 10.0),
    ("p_pid_offset", "f32a", None),
    ("cc_startup_boost_duty", "f16", 10000.0),
    ("cc_min_current", "f32a", None),
    ("cc_gain", "f32a", None),
    ("cc_ramp_step_max", "f16", 10000.0),
    ("m_fault_stop_time_ms", "i32", None),
    ("m_duty_ramp_step", "f16", 10000.0),
    ("m_current_backoff_gain", "f32a", None),
    ("m_encoder_counts", "u32", None),
    ("m_encoder_sin_amp", "f16", 1000.0),
    ("m_encoder_cos_amp", "f16", 1000.0),
    ("m_encoder_sin_offset", "f16", 1000.0),
    ("m_encoder_cos_offset", "f16", 1000.0),
    ("m_encoder_sincos_filter_constant", "f16", 1000.0),
    ("m_encoder_sincos_phase_correction", "f16", 1000.0),
    ("m_sensor_port_mode", "u8", None),
    ("m_invert_direction", "u8", None),
    ("m_drv8301_oc_mode", "u8", None),
    ("m_drv8301_oc_adj", "u8", None),
    ("m_bldc_f_sw_min", "f32a", None),
    ("m_bldc_f_sw_max", "f32a", None),
    ("m_dc_f_sw", "f32a", None),
    ("m_ntc_motor_beta", "f32a", None),
    ("m_out_aux_mode", "u8", None),
    ("m_motor_temp_sens_type", "u8", None),
    ("m_ptc_motor_coeff", "f32a", None),
    ("m_ntcx_ptcx_res", "f16", 0.1),
    ("m_ntcx_ptcx_temp_base", "f16", 10.0),
    ("m_hall_extra_samples", "u8", None),
    ("m_batt_filter_const", "u8", None),
    ("si_motor_poles", "u8", None),
    ("si_gear_ratio", "f32a", None),
    ("si_wheel_diameter", "f32a", None),
    ("si_battery_type", "u8", None),
    ("si_battery_cells", "u8", None),
    ("si_battery_ah", "f32a", None),
    ("si_motor_nl_current", "f32a", None),
    ("bms_type", "u8", None),
    ("bms_limit_mode", "u8", None),
    ("bms_t_limit_start", "u8", None),
    ("bms_t_limit_end", "u8", None),
    ("bms_soc_limit_start", "f16", 1000.0),
    ("bms_soc_limit_end", "f16", 1000.0),
    ("bms_vmin_limit_start", "f16", 1000.0),
    ("bms_vmin_limit_end", "f16", 1000.0),
    ("bms_vmax_limit_start", "f16", 1000.0),
    ("bms_vmax_limit_end", "f16", 1000.0),
    ("bms_fwd_can_mode", "u8", None),
)

_KIND_SIZES = {"u8": 1, "i8": 1, "u16": 2, "i32": 4, "u32": 4, "f32a": 4, "f16": 2}

# Signature + every field (477 for 6.05). The reply carries one more byte:
# the leading COMM_GET_MCCONF command id.
MCCONF_BODY_LEN_6_05 = 4 + sum(_KIND_SIZES[k] for _, k, _ in MCCONF_FIELDS_6_05)

# Fields printed by default: (name, unit, note).
MCCONF_KEY_FIELDS: Tuple[Tuple[str, str, str], ...] = (
    ("l_current_max", "A", "motor current max"),
    ("l_current_min", "A", "motor current min (braking/regen)"),
    ("l_in_current_max", "A", "battery current max"),
    ("l_in_current_min", "A", "battery current min (regen into the pack)"),
    ("l_abs_current_max", "A", "absolute max current (fault)"),
    ("l_min_erpm", "ERPM", "min ERPM"),
    ("l_max_erpm", "ERPM", "max ERPM"),
    ("l_min_vin", "V", "under-voltage fault"),
    ("l_max_vin", "V", "over-voltage fault"),
    ("l_battery_cut_start", "V", "battery voltage cutoff start"),
    ("l_battery_cut_end", "V", "battery voltage cutoff end"),
    ("l_battery_regen_cut_start", "V", "battery voltage regen cutoff start"),
    ("l_battery_regen_cut_end", "V", "battery voltage regen cutoff end"),
    ("l_temp_fet_start", "C", "MOSFET temp derate start"),
    ("l_temp_fet_end", "C", "MOSFET temp derate end"),
    ("l_temp_motor_start", "C", "motor temp derate start"),
    ("l_temp_motor_end", "C", "motor temp derate end"),
    ("l_watt_max", "W", "max power"),
    ("l_watt_min", "W", "min power (regen)"),
    ("si_battery_cells", "", "series cells"),
    ("si_battery_ah", "Ah", "battery capacity"),
)


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------

class ReadOnlyViolation(AssertionError):
    """A non-read command or a non-query frame was about to be sent.

    Subclasses AssertionError but is raised explicitly, so `python -O`
    cannot strip the check.
    """


class QueryTimeout(Exception):
    """No valid reply arrived within the timeout (after all retries)."""


class ReassemblyError(Exception):
    """A PROCESS_RX_BUFFER frame arrived but the buffered reply was bad."""


class DecodeError(ValueError):
    """A reply was too short or otherwise malformed for its command."""


class McconfSignatureMismatch(DecodeError):
    """GET_MCCONF reply is not the FW 6.05 layout; it must not be decoded."""

    def __init__(self, length: int, signature: Optional[int]) -> None:
        self.length = length
        self.signature = signature
        sig = "none" if signature is None else f"{signature} (0x{signature:08X})"
        super().__init__(
            f"MCCONF signature mismatch: got {sig}, expected "
            f"{MCCONF_SIGNATURE_6_05} (0x{MCCONF_SIGNATURE_6_05:08X}, FW 6.05); "
            f"raw reply length {length} bytes; not decoding"
        )


# ---------------------------------------------------------------------------
# CRC16 (util/crc.c: CRC16 XMODEM, poly 0x1021, init 0, no reflection)
# ---------------------------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Read-only send guard
# ---------------------------------------------------------------------------

def check_read_only(cmd: int) -> None:
    if type(cmd) is not int or cmd not in READ_ONLY_COMMANDS:
        raise ReadOnlyViolation(
            f"refusing to send command {cmd!r}: only read commands "
            f"{sorted(READ_ONLY_COMMANDS)} are allowed"
        )


def check_vesc_id(vesc_id: int) -> None:
    if type(vesc_id) is not int or not 0 <= vesc_id <= 254:
        raise ValueError(f"VESC id {vesc_id!r} out of range 0..254 (255 is broadcast)")
    if vesc_id == SENDER_ID:
        raise ValueError(f"VESC id {vesc_id} is this tool's own sender id")


def build_request(vesc_id: int, cmd: int) -> Tuple[int, bytes]:
    """Arbitration id and payload of one read query. Raises on a non-read cmd."""
    check_read_only(cmd)
    check_vesc_id(vesc_id)
    arb_id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | vesc_id
    return arb_id, bytes([SENDER_ID, 0, cmd])


def guard_frame(arb_id: int, data: bytes, is_extended: bool) -> None:
    """Final check run immediately before every bus.send()."""
    if not is_extended:
        raise ReadOnlyViolation("refusing to send a standard-id frame")
    if (arb_id >> 8) != CAN_PACKET_PROCESS_SHORT_BUFFER:
        raise ReadOnlyViolation(
            f"refusing to send CAN packet type {arb_id >> 8} (arb 0x{arb_id:X}); "
            f"only PROCESS_SHORT_BUFFER ({CAN_PACKET_PROCESS_SHORT_BUFFER}) is allowed"
        )
    target = arb_id & 0xFF
    if target == CAN_BROADCAST_ID or target == SENDER_ID:
        raise ReadOnlyViolation(f"refusing to send to CAN id {target}")
    data = bytes(data)
    if len(data) != 3 or data[0] != SENDER_ID or data[1] != 0:
        raise ReadOnlyViolation(f"refusing to send unexpected payload {data.hex()}")
    check_read_only(data[2])


# ---------------------------------------------------------------------------
# Reply reassembly (mirror of comm_can_send_buffer on the VESC side)
# ---------------------------------------------------------------------------

class Reassembler:
    """Rebuilds VESC replies addressed to SENDER_ID.

    feed() returns (src_vesc_id, payload) when a reply completes, None for a
    frame that is buffered or not ours, and raises ReassemblyError when a
    PROCESS_RX_BUFFER frame closes a buffer that is incomplete or fails CRC.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._buf = bytearray(MAX_BUFFER_LEN)
        self._have = bytearray(MAX_BUFFER_LEN)

    def _store(self, offset: int, chunk: bytes) -> None:
        if offset == 0:
            # The VESC always starts a buffer at offset 0: a new reply.
            self.reset()
        end = offset + len(chunk)
        if end > MAX_BUFFER_LEN:
            return
        self._buf[offset:end] = chunk
        self._have[offset:end] = b"\x01" * len(chunk)

    def feed(self, arb_id: int, data: bytes, is_extended: bool = True) -> Optional[Tuple[int, bytes]]:
        if not is_extended or (arb_id & 0xFF) != SENDER_ID:
            return None
        packet = arb_id >> 8
        data = bytes(data)

        if packet == CAN_PACKET_FILL_RX_BUFFER:
            if len(data) >= 2:
                self._store(data[0], data[1:])
            return None

        if packet == CAN_PACKET_FILL_RX_BUFFER_LONG:
            if len(data) >= 3:
                self._store((data[0] << 8) | data[1], data[2:])
            return None

        if packet == CAN_PACKET_PROCESS_RX_BUFFER:
            if len(data) < 6:
                return None
            src = data[0]
            length = (data[2] << 8) | data[3]
            crc_rx = (data[4] << 8) | data[5]
            buf, have = self._buf, self._have
            self.reset()
            if length == 0 or length > MAX_BUFFER_LEN:
                raise ReassemblyError(f"VESC {src}: bad reply length {length}")
            missing = length - sum(have[:length])
            if missing:
                raise ReassemblyError(
                    f"VESC {src}: reply incomplete, {missing} of {length} bytes "
                    f"missing (dropped CAN frames)"
                )
            payload = bytes(buf[:length])
            crc_calc = crc16(payload)
            if crc_calc != crc_rx:
                raise ReassemblyError(
                    f"VESC {src}: CRC mismatch on {length}-byte reply "
                    f"(got 0x{crc_rx:04X}, computed 0x{crc_calc:04X})"
                )
            return src, payload

        if packet == CAN_PACKET_PROCESS_SHORT_BUFFER:
            if len(data) < 3:
                return None
            return data[0], data[2:]

        return None


# ---------------------------------------------------------------------------
# Decoding helpers (util/buffer.c)
# ---------------------------------------------------------------------------

def _f32(x: float) -> float:
    """Round to IEEE float32, as the firmware's float variables hold it."""
    try:
        return struct.unpack(">f", struct.pack(">f", x))[0]
    except OverflowError:
        return math.copysign(math.inf, x)


def float32_auto_decode(raw: int) -> float:
    """buffer_get_float32_auto()."""
    e = (raw >> 23) & 0xFF
    sig_i = raw & 0x7FFFFF
    neg = bool(raw & (1 << 31))
    sig = 0.0
    if e != 0 or sig_i != 0:
        sig = sig_i / (8388608.0 * 2.0) + 0.5
        e -= 126
    if neg:
        sig = -sig
    return _f32(math.ldexp(sig, e))


class _Reader:
    def __init__(self, data: bytes, what: str) -> None:
        self.data = bytes(data)
        self.ind = 0
        self.what = what

    def remaining(self) -> int:
        return len(self.data) - self.ind

    def _take(self, n: int) -> bytes:
        if self.ind + n > len(self.data):
            raise DecodeError(
                f"{self.what}: reply too short ({len(self.data)} bytes, "
                f"needed {self.ind + n})"
            )
        chunk = self.data[self.ind:self.ind + n]
        self.ind += n
        return chunk

    def u8(self) -> int:
        return self._take(1)[0]

    def i8(self) -> int:
        return struct.unpack(">b", self._take(1))[0]

    def u16(self) -> int:
        return struct.unpack(">H", self._take(2))[0]

    def i16(self) -> int:
        return struct.unpack(">h", self._take(2))[0]

    def u32(self) -> int:
        return struct.unpack(">I", self._take(4))[0]

    def i32(self) -> int:
        return struct.unpack(">i", self._take(4))[0]

    def f16(self, scale: float) -> float:
        # buffer_get_float16: (float)int16 / scale, in float arithmetic.
        return _f32(self.i16() / _f32(scale))

    def f32(self, scale: float) -> float:
        return self.i32() / scale

    def f32_auto(self) -> float:
        return float32_auto_decode(self.u32())

    def cstring(self) -> str:
        end = self.data.find(b"\x00", self.ind)
        if end < 0:
            raise DecodeError(f"{self.what}: unterminated string")
        s = self.data[self.ind:end].decode("ascii", errors="replace")
        self.ind = end + 1
        return s


def _expect_cmd(payload: bytes, cmd: int) -> _Reader:
    r = _Reader(payload, COMMAND_NAMES.get(cmd, str(cmd)))
    got = r.u8()
    if got != cmd:
        raise DecodeError(f"expected reply to command {cmd}, got command {got}")
    return r


# ---------------------------------------------------------------------------
# Reply parsers (comm/commands.c, release_6_05)
# ---------------------------------------------------------------------------

def parse_fw_version(payload: bytes) -> Dict[str, Any]:
    r = _expect_cmd(payload, COMM_FW_VERSION)
    major = r.u8()
    minor = r.u8()
    out: Dict[str, Any] = {
        "fw_major": major,
        "fw_minor": minor,
        "fw_version": f"{major}.{minor:02d}",
        "hw_name": r.cstring(),
    }
    out["uuid"] = r._take(12).hex()
    # Optional tail (present on 6.05): pairing_done, FW_TEST_VERSION_NUMBER,
    # HW_TYPE, custom config count, phase filters, QML HW, QML app,
    # nrf flags, FW_NAME, HW CRC.
    for key in ("pairing_done", "fw_test_version", "hw_type", "custom_config_num",
                "has_phase_filters", "qmlui_hw", "qmlui_app", "nrf_flags"):
        if r.remaining() < 1:
            return out
        out[key] = r.u8()
    if r.remaining() >= 1:
        out["fw_name"] = r.cstring()
    if r.remaining() >= 4:
        out["hw_crc"] = r.u32()
    return out


def parse_get_values(payload: bytes) -> Dict[str, Any]:
    """COMM_GET_VALUES with the full mask, in commands.c order."""
    r = _expect_cmd(payload, COMM_GET_VALUES)
    v: Dict[str, Any] = {}
    v["temp_fet_c"] = r.i16() / 1e1
    v["temp_motor_c"] = r.i16() / 1e1
    v["avg_motor_current_a"] = r.i32() / 1e2
    v["avg_input_current_a"] = r.i32() / 1e2
    v["avg_id_a"] = r.i32() / 1e2
    v["avg_iq_a"] = r.i32() / 1e2
    v["duty"] = r.i16() / 1e3
    v["erpm"] = float(r.i32())
    v["v_in"] = r.i16() / 1e1
    v["amp_hours"] = r.i32() / 1e4
    v["amp_hours_charged"] = r.i32() / 1e4
    v["watt_hours"] = r.i32() / 1e4
    v["watt_hours_charged"] = r.i32() / 1e4
    v["tachometer"] = r.i32()
    v["tachometer_abs"] = r.i32()
    fault = r.u8()
    v["fault_code"] = fault
    v["fault_name"] = FAULT_CODES[fault] if fault < len(FAULT_CODES) else f"UNKNOWN_{fault}"
    # Later fields were appended over firmware versions; read what is there.
    if r.remaining() >= 4:
        v["pid_pos_deg"] = r.i32() / 1e6
    if r.remaining() >= 1:
        v["controller_id"] = r.u8()
    if r.remaining() >= 6:
        v["temp_mos1_c"] = r.i16() / 1e1
        v["temp_mos2_c"] = r.i16() / 1e1
        v["temp_mos3_c"] = r.i16() / 1e1
    if r.remaining() >= 4:
        v["avg_vd_v"] = r.i32() / 1e3
    if r.remaining() >= 4:
        v["avg_vq_v"] = r.i32() / 1e3
    if r.remaining() >= 1:
        status = r.u8()
        v["status"] = status
        v["timeout_active"] = bool(status & 1)
        v["kill_sw_active"] = bool(status & 2)
    return v


def decode_mcconf(payload: bytes) -> Dict[str, Any]:
    """Decode a COMM_GET_MCCONF reply with the FW 6.05 layout.

    Raises McconfSignatureMismatch (with the raw length and signature) when
    the signature is not FW 6.05's, without decoding any field.
    """
    if not payload or payload[0] != COMM_GET_MCCONF:
        raise DecodeError(
            f"expected reply to command {COMM_GET_MCCONF}, got "
            f"{payload[0] if payload else 'empty reply'}"
        )
    if len(payload) < 5:
        raise McconfSignatureMismatch(len(payload), None)
    signature = struct.unpack(">I", payload[1:5])[0]
    if signature != MCCONF_SIGNATURE_6_05:
        raise McconfSignatureMismatch(len(payload), signature)

    r = _Reader(payload, "COMM_GET_MCCONF")
    r.ind = 5
    fields: Dict[str, Any] = {}
    for name, kind, scale in MCCONF_FIELDS_6_05:
        if kind == "u8":
            fields[name] = r.u8()
        elif kind == "i8":
            fields[name] = r.i8()
        elif kind == "u16":
            fields[name] = r.u16()
        elif kind == "i32":
            fields[name] = r.i32()
        elif kind == "u32":
            fields[name] = r.u32()
        elif kind == "f32a":
            fields[name] = r.f32_auto()
        elif kind == "f16":
            assert scale is not None
            fields[name] = r.f16(scale)
        else:  # pragma: no cover - table is static
            raise DecodeError(f"unknown field kind {kind}")
    return {
        "signature": signature,
        "length": len(payload),
        "trailing_bytes": r.remaining(),
        "fields": fields,
    }


# ---------------------------------------------------------------------------
# Bus client
# ---------------------------------------------------------------------------

MessageFactory = Callable[..., Any]


class VescClient:
    """Sends read-only queries and waits for the reassembled replies."""

    def __init__(self, bus: Any, message_factory: MessageFactory,
                 timeout_s: float = 1.0, retries: int = 2) -> None:
        self.bus = bus
        self.message_factory = message_factory
        self.timeout_s = float(timeout_s)
        self.retries = max(0, int(retries))
        self.reassembler = Reassembler()

    def _drain(self) -> None:
        for _ in range(4096):
            if self.bus.recv(timeout=0.0) is None:
                return

    def _send(self, vesc_id: int, cmd: int) -> None:
        arb_id, data = build_request(vesc_id, cmd)
        guard_frame(arb_id, data, True)
        msg = self.message_factory(arbitration_id=arb_id, data=data, is_extended_id=True)
        # Re-check what is actually in the message object being sent.
        guard_frame(int(msg.arbitration_id), bytes(msg.data),
                    bool(getattr(msg, "is_extended_id", True)))
        self.bus.send(msg)

    def request(self, vesc_id: int, cmd: int) -> bytes:
        check_read_only(cmd)
        check_vesc_id(vesc_id)
        name = COMMAND_NAMES[cmd]
        last_err = "no reply"
        attempts = 1 + self.retries
        for _attempt in range(attempts):
            self._drain()
            self.reassembler.reset()
            self._send(vesc_id, cmd)
            deadline = time.monotonic() + self.timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    last_err = f"no reply within {self.timeout_s:g} s"
                    break
                msg = self.bus.recv(timeout=min(remaining, 0.1))
                if msg is None:
                    continue
                try:
                    result = self.reassembler.feed(
                        int(msg.arbitration_id), bytes(msg.data),
                        bool(getattr(msg, "is_extended_id", True)))
                except ReassemblyError as exc:
                    if f"VESC {vesc_id}:" in str(exc):
                        last_err = str(exc)
                        break  # this attempt's reply is broken: retry now
                    continue
                if result is None:
                    continue
                src, payload = result
                if src != vesc_id or not payload or payload[0] != cmd:
                    continue  # someone else's reply, or a stale one
                return payload
        raise QueryTimeout(
            f"VESC {vesc_id}: {name} failed after {attempts} attempt(s): {last_err}")


def open_bus(channel: str) -> Tuple[Any, MessageFactory]:
    """Open SocketCAN with a kernel filter that passes only frames to SENDER_ID."""
    try:
        import can  # noqa: PLC0415 - optional on dev machines
    except ImportError as exc:
        raise RuntimeError(
            "python-can is not installed for this interpreter; on the Pi use "
            "/home/pi/.venv/bin/python3") from exc
    bus = can.Bus(
        channel=channel,
        interface="socketcan",
        can_filters=[{"can_id": SENDER_ID, "can_mask": 0xFF, "extended": True}],
    )
    return bus, can.Message


# ---------------------------------------------------------------------------
# Collection and output
# ---------------------------------------------------------------------------

def collect(client: VescClient, vesc_id: int) -> Dict[str, Any]:
    res: Dict[str, Any] = {"id": vesc_id, "side": SIDE_NAMES.get(vesc_id, "?"),
                           "ok": True, "errors": []}

    def fail(msg: str) -> None:
        res["ok"] = False
        res["errors"].append(msg)

    try:
        res["fw_version"] = parse_fw_version(client.request(vesc_id, COMM_FW_VERSION))
    except (QueryTimeout, DecodeError) as exc:
        fail(str(exc))
        return res  # no FW_VERSION reply: the VESC is not answering

    try:
        res["values"] = parse_get_values(client.request(vesc_id, COMM_GET_VALUES))
    except (QueryTimeout, DecodeError) as exc:
        fail(str(exc))

    try:
        res["mcconf"] = decode_mcconf(client.request(vesc_id, COMM_GET_MCCONF))
    except McconfSignatureMismatch as exc:
        res["mcconf_error"] = {"reason": "signature_mismatch", "length": exc.length,
                               "signature": exc.signature,
                               "expected_signature": MCCONF_SIGNATURE_6_05}
        fail(str(exc))
    except (QueryTimeout, DecodeError) as exc:
        res["mcconf_error"] = {"reason": "error", "message": str(exc)}
        fail(str(exc))
    return res


def _num(v: Any) -> str:
    if isinstance(v, float):
        if math.isinf(v) or math.isnan(v):
            return str(v)
        return f"{v:.7g}"
    return str(v)


def _json_safe(obj: Any) -> Any:
    if isinstance(obj, float):
        if math.isinf(obj) or math.isnan(obj):
            return str(obj)
        return float(f"{obj:.7g}")
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    return obj


def format_report(res: Dict[str, Any], show_all: bool = False) -> str:
    lines: List[str] = [f"=== VESC id {res['id']} ({res['side']}) ==="]
    fw = res.get("fw_version")
    if fw:
        lines.append(f"  HW name            {fw['hw_name']}")
        test = fw.get("fw_test_version")
        test_s = f" (test build {test})" if test else ""
        lines.append(f"  FW version         {fw['fw_version']}{test_s}")
        lines.append(f"  UUID               {fw['uuid']}")
    vals = res.get("values")
    if vals:
        lines.append("  -- GET_VALUES --")
        lines.append(f"  v_in               {_num(vals['v_in'])} V")
        lines.append(f"  temp_fet           {_num(vals['temp_fet_c'])} C")
        lines.append(f"  temp_motor         {_num(vals['temp_motor_c'])} C")
        lines.append(f"  input current      {_num(vals['avg_input_current_a'])} A (avg since last read)")
        lines.append(f"  motor current      {_num(vals['avg_motor_current_a'])} A (avg since last read)")
        lines.append(f"  erpm               {_num(vals['erpm'])}")
        lines.append(f"  Ah / Ah charged    {_num(vals['amp_hours'])} / {_num(vals['amp_hours_charged'])}")
        lines.append(f"  Wh / Wh charged    {_num(vals['watt_hours'])} / {_num(vals['watt_hours_charged'])}")
        lines.append(f"  fault              {vals['fault_code']} {vals['fault_name']}")
    mc = res.get("mcconf")
    if mc:
        trail = f", {mc['trailing_bytes']} trailing bytes" if mc["trailing_bytes"] else ""
        lines.append(f"  -- MCCONF (FW 6.05 layout, signature OK, {mc['length']} bytes{trail}) --")
        f = mc["fields"]
        if show_all:
            for name, _kind, _scale in MCCONF_FIELDS_6_05:
                lines.append(f"  {name:36s} {_num(f[name])}")
        else:
            for name, unit, note in MCCONF_KEY_FIELDS:
                val = f"{_num(f[name])} {unit}".rstrip()
                lines.append(f"  {name:28s} {val:>14s}   {note}")
    err = res.get("mcconf_error")
    if err and err.get("reason") == "signature_mismatch":
        sig = err["signature"]
        sig_s = "none" if sig is None else f"{sig} (0x{sig:08X})"
        lines.append("  -- MCCONF NOT DECODED: signature mismatch --")
        lines.append(f"  raw reply length   {err['length']} bytes")
        lines.append(f"  signature          {sig_s}")
        lines.append(f"  expected (FW 6.05) {MCCONF_SIGNATURE_6_05} (0x{MCCONF_SIGNATURE_6_05:08X})")
    for e in res["errors"]:
        lines.append(f"  ERROR: {e}")
    return "\n".join(lines)


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Read-only VESC query over SocketCAN: FW_VERSION, GET_VALUES, GET_MCCONF.")
    p.add_argument("--id", type=int, action="append", dest="ids", metavar="CAN_ID",
                   help="VESC CAN id to query (repeatable; default: 2 (left) then 1 (right))")
    p.add_argument("--channel", default="can0", help="SocketCAN channel (default can0)")
    p.add_argument("--timeout", type=float, default=1.0,
                   help="seconds to wait for each reply (default 1.0)")
    p.add_argument("--retries", type=int, default=2,
                   help="re-sends of a query after a timeout or a bad reply (default 2)")
    p.add_argument("--json", action="store_true", help="print JSON instead of text")
    p.add_argument("--all", action="store_true", help="print every decoded MCCONF field")
    args = p.parse_args(argv)
    args.ids = args.ids or list(DEFAULT_IDS)
    for vid in args.ids:
        try:
            check_vesc_id(vid)
        except ValueError as exc:
            p.error(str(exc))
    if args.timeout <= 0:
        p.error("--timeout must be > 0")
    if args.retries < 0:
        p.error("--retries must be >= 0")
    return args


def main(argv: Optional[Sequence[str]] = None,
         bus_opener: Callable[[str], Tuple[Any, MessageFactory]] = open_bus) -> int:
    args = _parse_args(argv)
    try:
        bus, factory = bus_opener(args.channel)
    except Exception as exc:  # noqa: BLE001 - report any bus-open failure cleanly
        print(f"vesc_info: cannot open {args.channel}: {exc}", file=sys.stderr)
        return 2

    results: List[Dict[str, Any]] = []
    try:
        client = VescClient(bus, factory, timeout_s=args.timeout, retries=args.retries)
        for vid in args.ids:
            try:
                results.append(collect(client, vid))
            except ReadOnlyViolation:
                raise
            except Exception as exc:  # noqa: BLE001 - e.g. can.CanError on recv/send
                results.append({"id": vid, "side": SIDE_NAMES.get(vid, "?"), "ok": False,
                                "errors": [f"{type(exc).__name__}: {exc}"]})
    except KeyboardInterrupt:
        print("vesc_info: interrupted", file=sys.stderr)
        return 130
    finally:
        try:
            bus.shutdown()
        except Exception:  # noqa: BLE001
            pass

    if args.json:
        print(json.dumps(_json_safe({"channel": args.channel, "sender_id": SENDER_ID,
                                     "vescs": results}), indent=2))
    else:
        print("\n\n".join(format_report(r, show_all=args.all) for r in results))
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
