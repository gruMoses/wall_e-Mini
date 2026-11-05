"""
Tests for tools/vesc_info.py (read-only VESC query tool).

No hardware and no python-can: frames are constructed here the way the
VESC firmware builds them (bldc release_6_05 comm_can_send_buffer,
commands.c, confgenerator_serialize_mcconf, buffer.c encoders), and a fake
bus stands in for SocketCAN.

The MCCONF test buffer is built from MCCONF_SERIALIZE_SPEC below, which was
transcribed from the *serialize* side of confgenerator.c (release_6_05);
the tool's table was transcribed from the *deserialize* side, so the two are
cross-checked here.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
from typing import List, Optional, Tuple

import pytest

_TOOLS_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "tools")
_TOOLS_DIR = os.path.normpath(_TOOLS_DIR)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import vesc_info as vi  # noqa: E402


# ---------------------------------------------------------------------------
# Firmware-side encoders (util/buffer.c)
# ---------------------------------------------------------------------------

def _f32(x: float) -> float:
    return struct.unpack(">f", struct.pack(">f", x))[0]


def enc_float32_auto(number: float) -> bytes:
    """buffer_append_float32_auto()."""
    number = _f32(number)
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


def enc_i16(v: int) -> bytes:
    return struct.pack(">h", v)


def enc_i32(v: int) -> bytes:
    return struct.pack(">i", v)


def enc_u32(v: int) -> bytes:
    return struct.pack(">I", v)


# ---------------------------------------------------------------------------
# Firmware-side CAN framing (comm/comm_can.c comm_can_send_buffer)
# ---------------------------------------------------------------------------

SENDER = 0x7E


def vesc_reply_frames(src_id: int, payload: bytes, dest: int = SENDER,
                      send: int = 1) -> List[Tuple[int, bytes]]:
    """Frames a VESC with id src_id emits to reply `payload` to `dest`."""
    frames: List[Tuple[int, bytes]] = []
    n = len(payload)
    if n <= 6:
        frames.append(((8 << 8) | dest, bytes([src_id, send]) + payload))
        return frames
    end_a = 0
    i = 0
    while i < n:
        if i > 255:
            break
        end_a = i + 7
        frames.append(((5 << 8) | dest, bytes([i]) + payload[i:i + 7]))
        i += 7
    i = end_a
    while i < n:
        frames.append(((6 << 8) | dest, bytes([i >> 8, i & 0xFF]) + payload[i:i + 6]))
        i += 6
    crc = vi.crc16(payload)
    frames.append(((7 << 8) | dest,
                   bytes([src_id, send, n >> 8, n & 0xFF, crc >> 8, crc & 0xFF])))
    return frames


def feed_all(r: "vi.Reassembler", frames):
    out = None
    for arb, data in frames:
        res = r.feed(arb, data, True)
        if res is not None:
            assert out is None, "more than one completed reply"
            out = res
    return out


# ---------------------------------------------------------------------------
# Payload builders (comm/commands.c)
# ---------------------------------------------------------------------------

UUID = bytes(range(0x30, 0x3C))


def fw_version_payload(hw: str = "75_100_V2", major: int = 6, minor: int = 5) -> bytes:
    p = bytes([0, major, minor]) + hw.encode() + b"\x00" + UUID
    p += bytes([0, 0, 0, 1, 0, 0, 0, 0])   # pairing, test ver, hw type, cfg num, ...
    p += b"\x00"                           # FW_NAME ""
    p += enc_u32(0xDEADBEEF)               # main_calc_hw_crc()
    return p


def get_values_payload() -> bytes:
    p = bytes([4])
    p += enc_i16(315)            # temp_fet 31.5
    p += enc_i16(-12)            # temp_motor -1.2
    p += enc_i32(1234)           # avg motor current 12.34
    p += enc_i32(-321)           # avg input current -3.21
    p += enc_i32(50)             # id 0.5
    p += enc_i32(-75)            # iq -0.75
    p += enc_i16(250)            # duty 0.25
    p += enc_i32(-1500)          # rpm
    p += enc_i16(517)            # v_in 51.7
    p += enc_i32(1234)           # Ah 0.1234
    p += enc_i32(567)            # Ah charged 0.0567
    p += enc_i32(65432)          # Wh 6.5432
    p += enc_i32(12345)          # Wh charged 1.2345
    p += enc_i32(12345)          # tachometer
    p += enc_i32(54321)          # tachometer abs
    p += bytes([2])              # fault: UNDER_VOLTAGE
    p += enc_i32(90_000_000)     # pid pos 90.0
    p += bytes([2])              # controller id
    p += enc_i16(301) + enc_i16(302) + enc_i16(303)
    p += enc_i32(1500)           # vd 1.5
    p += enc_i32(-2500)          # vq -2.5
    p += bytes([0])              # status
    return p


# confgenerator_serialize_mcconf(), bldc release_6_05: "kind[:scale] c_name"
MCCONF_SERIALIZE_SPEC = """
    u8    pwm_mode
    u8    comm_mode
    u8    motor_type
    u8    sensor_mode
    f32a  l_current_max
    f32a  l_current_min
    f32a  l_in_current_max
    f32a  l_in_current_min
    f16:10000 l_in_current_map_start
    f16:10000 l_in_current_map_filter
    f32a  l_abs_current_max
    f32a  l_min_erpm
    f32a  l_max_erpm
    f16:10000 l_erpm_start
    f32a  l_max_erpm_fbrake
    f32a  l_max_erpm_fbrake_cc
    f16:10 l_min_vin
    f16:10 l_max_vin
    f16:10 l_battery_cut_start
    f16:10 l_battery_cut_end
    f16:10 l_battery_regen_cut_start
    f16:10 l_battery_regen_cut_end
    u8    l_slow_abs_current
    u8    l_temp_fet_start
    u8    l_temp_fet_end
    u8    l_temp_motor_start
    u8    l_temp_motor_end
    f16:10000 l_temp_accel_dec
    f16:10000 l_min_duty
    f16:10000 l_max_duty
    f32a  l_watt_max
    f32a  l_watt_min
    f16:10000 l_current_max_scale
    f16:10000 l_current_min_scale
    f16:10000 l_duty_start
    f32a  sl_min_erpm
    f32a  sl_min_erpm_cycle_int_limit
    f32a  sl_max_fullbreak_current_dir_change
    f16:10 sl_cycle_int_limit
    f16:10000 sl_phase_advance_at_br
    f32a  sl_cycle_int_rpm_br
    f32a  sl_bemf_coupling_k
    u8    hall_table[0]
    u8    hall_table[1]
    u8    hall_table[2]
    u8    hall_table[3]
    u8    hall_table[4]
    u8    hall_table[5]
    u8    hall_table[6]
    u8    hall_table[7]
    f32a  hall_sl_erpm
    f32a  foc_current_kp
    f32a  foc_current_ki
    f32a  foc_f_zv
    f32a  foc_dt_us
    u8    foc_encoder_inverted
    f32a  foc_encoder_offset
    f32a  foc_encoder_ratio
    u8    foc_sensor_mode
    f32a  foc_pll_kp
    f32a  foc_pll_ki
    f32a  foc_motor_l
    f32a  foc_motor_ld_lq_diff
    f32a  foc_motor_r
    f32a  foc_motor_flux_linkage
    f32a  foc_observer_gain
    f32a  foc_observer_gain_slow
    f16:1000 foc_observer_offset
    f32a  foc_duty_dowmramp_kp
    f32a  foc_duty_dowmramp_ki
    f16:10000 foc_start_curr_dec
    f32a  foc_start_curr_dec_rpm
    f32a  foc_openloop_rpm
    f16:1000 foc_openloop_rpm_low
    f16:1000 foc_d_gain_scale_start
    f16:1000 foc_d_gain_scale_max_mod
    f16:100 foc_sl_openloop_hyst
    f16:100 foc_sl_openloop_time_lock
    f16:100 foc_sl_openloop_time_ramp
    f16:100 foc_sl_openloop_time
    f16:100 foc_sl_openloop_boost_q
    f16:100 foc_sl_openloop_max_q
    u8    foc_hall_table[0]
    u8    foc_hall_table[1]
    u8    foc_hall_table[2]
    u8    foc_hall_table[3]
    u8    foc_hall_table[4]
    u8    foc_hall_table[5]
    u8    foc_hall_table[6]
    u8    foc_hall_table[7]
    f32a  foc_hall_interp_erpm
    f32a  foc_sl_erpm_start
    f32a  foc_sl_erpm
    u8    foc_control_sample_mode
    u8    foc_current_sample_mode
    u8    foc_sat_comp_mode
    f16:1000 foc_sat_comp
    u8    foc_temp_comp
    f16:100 foc_temp_comp_base_temp
    f16:10000 foc_current_filter_const
    u8    foc_cc_decoupling
    u8    foc_observer_type
    f16:10 foc_hfi_voltage_start
    f16:10 foc_hfi_voltage_run
    f16:10 foc_hfi_voltage_max
    f16:1000 foc_hfi_gain
    f16:1000 foc_hfi_max_err
    f16:100 foc_hfi_hyst
    f32a  foc_sl_erpm_hfi
    u16   foc_hfi_start_samples
    f32a  foc_hfi_obs_ovr_sec
    u8    foc_hfi_samples
    u8    foc_offsets_cal_on_boot
    f32a  foc_offsets_current[0]
    f32a  foc_offsets_current[1]
    f32a  foc_offsets_current[2]
    f16:10000 foc_offsets_voltage[0]
    f16:10000 foc_offsets_voltage[1]
    f16:10000 foc_offsets_voltage[2]
    f16:10000 foc_offsets_voltage_undriven[0]
    f16:10000 foc_offsets_voltage_undriven[1]
    f16:10000 foc_offsets_voltage_undriven[2]
    u8    foc_phase_filter_enable
    u8    foc_phase_filter_disable_fault
    f32a  foc_phase_filter_max_erpm
    u8    foc_mtpa_mode
    f32a  foc_fw_current_max
    f16:10000 foc_fw_duty_start
    f16:1000 foc_fw_ramp_time
    f16:10000 foc_fw_q_current_factor
    u8    foc_speed_soure
    u8    foc_short_ls_on_zero_duty
    u8    sp_pid_loop_rate
    f32a  s_pid_kp
    f32a  s_pid_ki
    f32a  s_pid_kd
    f16:10000 s_pid_kd_filter
    f32a  s_pid_min_erpm
    u8    s_pid_allow_braking
    f32a  s_pid_ramp_erpms_s
    u8    s_pid_speed_source
    f32a  p_pid_kp
    f32a  p_pid_ki
    f32a  p_pid_kd
    f32a  p_pid_kd_proc
    f16:10000 p_pid_kd_filter
    f32a  p_pid_ang_div
    f16:10 p_pid_gain_dec_angle
    f32a  p_pid_offset
    f16:10000 cc_startup_boost_duty
    f32a  cc_min_current
    f32a  cc_gain
    f16:10000 cc_ramp_step_max
    i32   m_fault_stop_time_ms
    f16:10000 m_duty_ramp_step
    f32a  m_current_backoff_gain
    u32   m_encoder_counts
    f16:1000 m_encoder_sin_amp
    f16:1000 m_encoder_cos_amp
    f16:1000 m_encoder_sin_offset
    f16:1000 m_encoder_cos_offset
    f16:1000 m_encoder_sincos_filter_constant
    f16:1000 m_encoder_sincos_phase_correction
    u8    m_sensor_port_mode
    u8    m_invert_direction
    u8    m_drv8301_oc_mode
    u8    m_drv8301_oc_adj
    f32a  m_bldc_f_sw_min
    f32a  m_bldc_f_sw_max
    f32a  m_dc_f_sw
    f32a  m_ntc_motor_beta
    u8    m_out_aux_mode
    u8    m_motor_temp_sens_type
    f32a  m_ptc_motor_coeff
    f16:0.1 m_ntcx_ptcx_res
    f16:10 m_ntcx_ptcx_temp_base
    u8    m_hall_extra_samples
    u8    m_batt_filter_const
    u8    si_motor_poles
    f32a  si_gear_ratio
    f32a  si_wheel_diameter
    u8    si_battery_type
    u8    si_battery_cells
    f32a  si_battery_ah
    f32a  si_motor_nl_current
    u8    bms.type
    u8    bms.limit_mode
    u8    bms.t_limit_start
    u8    bms.t_limit_end
    f16:1000 bms.soc_limit_start
    f16:1000 bms.soc_limit_end
    f16:1000 bms.vmin_limit_start
    f16:1000 bms.vmin_limit_end
    f16:1000 bms.vmax_limit_start
    f16:1000 bms.vmax_limit_end
    u8    bms.fwd_can_mode
"""

MCCONF_SIGNATURE = 1065524471

# Realistic values for the fields the tool prints (13S pack).
KEY_VALUES = {
    "l_current_max": 60.0,
    "l_current_min": -60.0,
    "l_in_current_max": 30.0,
    "l_in_current_min": -20.0,
    "l_abs_current_max": 150.0,
    "l_min_erpm": -100000.0,
    "l_max_erpm": 100000.0,
    "l_min_vin": 8.0,
    "l_max_vin": 57.0,
    "l_battery_cut_start": 42.0,
    "l_battery_cut_end": 39.0,
    "l_battery_regen_cut_start": 52.5,
    "l_battery_regen_cut_end": 53.5,
    "l_temp_fet_start": 85,
    "l_temp_fet_end": 100,
    "l_watt_max": 1500000.0,
    "l_watt_min": -1500000.0,
    "si_battery_cells": 13,
    "si_battery_ah": 20.0,
}


def _py_name(c_name: str) -> str:
    return c_name.replace("[", "_").replace("]", "").replace(".", "_")


def build_mcconf_payload(signature: int = MCCONF_SIGNATURE):
    """Return (payload, expected decoded fields).

    Every field gets a distinct value derived from its index so that any
    misalignment in the decoder shows up as a wrong value.
    """
    body = bytearray(enc_u32(signature))
    expected = {}
    spec = [ln.split() for ln in MCCONF_SERIALIZE_SPEC.strip().splitlines()]
    for idx, (kind, c_name) in enumerate(spec):
        name = _py_name(c_name)
        if kind == "u8":
            if name.startswith("hall_table_"):
                raw = -1 if idx % 2 else (idx % 6)   # int8_t in the struct
                body.append(raw & 0xFF)
                expected[name] = raw
            else:
                raw = int(KEY_VALUES.get(name, idx % 251))
                body.append(raw)
                expected[name] = raw
        elif kind == "f32a":
            val = KEY_VALUES.get(name, (idx - 98) * 0.25)
            body += enc_float32_auto(val)
            expected[name] = _f32(val)
        elif kind.startswith("f16:"):
            scale = float(kind.split(":")[1])
            if name in KEY_VALUES:
                raw = int(round(KEY_VALUES[name] * scale))
            else:
                raw = (idx * 37) % 3000 - 1500
            body += enc_i16(raw)
            expected[name] = raw / scale
        elif kind == "u16":
            raw = idx * 3
            body += struct.pack(">H", raw)
            expected[name] = raw
        elif kind == "i32":
            raw = -idx * 1000
            body += enc_i32(raw)
            expected[name] = raw
        elif kind == "u32":
            raw = idx * 100_000
            body += enc_u32(raw)
            expected[name] = raw
        else:
            raise AssertionError(f"unknown kind {kind}")
    return bytes([14]) + bytes(body), expected


# ---------------------------------------------------------------------------
# Fake SocketCAN bus
# ---------------------------------------------------------------------------

class FakeMsg:
    def __init__(self, arbitration_id: int, data: bytes, is_extended_id: bool = True):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id


class FakeBus:
    """Records sends; answers each query like a VESC would."""

    def __init__(self, replies=None, drop_first: int = 0):
        # replies: {(vesc_id, cmd): payload}
        self.replies = replies or {}
        self.sent: List[FakeMsg] = []
        self.rx: List[FakeMsg] = []
        self.drop_first = drop_first   # drop one FILL frame in the first N replies
        self.shutdown_called = False

    def send(self, msg):
        self.sent.append(msg)
        vesc_id = msg.arbitration_id & 0xFF
        cmd = msg.data[2]
        payload = self.replies.get((vesc_id, cmd))
        if payload is None:
            return
        frames = vesc_reply_frames(vesc_id, payload, dest=msg.data[0])
        if self.drop_first > 0 and len(frames) > 2:
            self.drop_first -= 1
            del frames[1]
        # Interleave unrelated traffic: a STATUS frame from the VESC and a
        # SET_RPM from wall-e (neither is addressed to 0x7E).
        self.rx.append(FakeMsg((9 << 8) | vesc_id, bytes(8)))
        self.rx.append(FakeMsg((3 << 8) | vesc_id, bytes(4)))
        self.rx.extend(FakeMsg(a, d) for a, d in frames)

    def recv(self, timeout: Optional[float] = None):
        if self.rx:
            return self.rx.pop(0)
        return None

    def shutdown(self):
        self.shutdown_called = True


def _all_replies(vesc_ids=(2, 1)):
    mcconf, _ = build_mcconf_payload()
    out = {}
    for vid in vesc_ids:
        out[(vid, 0)] = fw_version_payload()
        out[(vid, 4)] = get_values_payload()
        out[(vid, 14)] = mcconf
    return out


def _assert_sent_frames_are_read_only(bus: FakeBus):
    assert bus.sent, "nothing was sent"
    for m in bus.sent:
        assert m.is_extended_id
        assert m.arbitration_id >> 8 == 8, "only PROCESS_SHORT_BUFFER may be sent"
        assert m.arbitration_id & 0xFF in (1, 2)
        assert len(m.data) == 3
        assert m.data[0] == 0x7E and m.data[1] == 0
        assert m.data[2] in (0, 4, 14)


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------

def test_crc16_xmodem_check_value():
    # CRC-16/XMODEM standard check value.
    assert vi.crc16(b"123456789") == 0x31C3
    assert vi.crc16(b"") == 0


def test_crc16_matches_firmware_table_algorithm():
    # util/crc.c: table-driven, table[i] = CRC of byte i with poly 0x1021.
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        table.append(c)
    assert table[1] == 0x1021 and table[2] == 0x2042   # crc16_tab[] head

    def fw_crc(buf):
        ck = 0
        for b in buf:
            ck = table[((ck >> 8) ^ b) & 0xFF] ^ ((ck << 8) & 0xFFFF)
        return ck

    payload, _ = build_mcconf_payload()
    assert vi.crc16(payload) == fw_crc(payload)


# ---------------------------------------------------------------------------
# Reassembly
# ---------------------------------------------------------------------------

def test_reassembly_short_buffer():
    r = vi.Reassembler()
    frames = vesc_reply_frames(2, bytes([4, 1, 2, 3]))
    assert len(frames) == 1 and frames[0][0] >> 8 == 8
    assert feed_all(r, frames) == (2, bytes([4, 1, 2, 3]))


def test_reassembly_fill_and_process():
    payload = get_values_payload()
    frames = vesc_reply_frames(1, payload)
    kinds = {a >> 8 for a, _ in frames}
    assert kinds == {5, 7}
    assert feed_all(vi.Reassembler(), frames) == (1, payload)


def test_reassembly_fill_long_for_mcconf_sized_reply():
    payload, _ = build_mcconf_payload()
    assert len(payload) == 478
    frames = vesc_reply_frames(2, payload)
    kinds = [a >> 8 for a, _ in frames]
    assert 5 in kinds and 6 in kinds and kinds[-1] == 7
    # FILL_RX_BUFFER covers offsets 0..252; LONG starts at 259.
    first_long = next(d for a, d in frames if a >> 8 == 6)
    assert (first_long[0] << 8 | first_long[1]) == 259
    assert feed_all(vi.Reassembler(), frames) == (2, payload)


def test_reassembly_out_of_order_fill_frames():
    payload = get_values_payload()
    frames = vesc_reply_frames(1, payload)
    fills, process = frames[:-1], frames[-1]
    reordered = [fills[0]] + list(reversed(fills[1:])) + [process]
    assert feed_all(vi.Reassembler(), reordered) == (1, payload)


def test_reassembly_crc_mismatch_raises():
    payload = get_values_payload()
    frames = vesc_reply_frames(1, payload)
    arb, data = frames[3]
    frames[3] = (arb, data[:2] + bytes([data[2] ^ 0xFF]) + data[3:])
    r = vi.Reassembler()
    with pytest.raises(vi.ReassemblyError, match="CRC mismatch"):
        feed_all(r, frames)


def test_reassembly_missing_frame_raises():
    payload, _ = build_mcconf_payload()
    frames = vesc_reply_frames(2, payload)
    del frames[40]
    with pytest.raises(vi.ReassemblyError, match="missing"):
        feed_all(vi.Reassembler(), frames)


def test_reassembly_ignores_frames_not_addressed_to_us():
    r = vi.Reassembler()
    payload = get_values_payload()
    other = vesc_reply_frames(1, payload, dest=0x10)
    assert feed_all(r, other) is None
    assert r.feed((9 << 8) | 1, bytes(8), True) is None       # CAN STATUS
    assert r.feed((8 << 8) | SENDER, bytes([1, 1, 4, 0]), False) is None  # std id


# ---------------------------------------------------------------------------
# FW_VERSION / GET_VALUES parsing
# ---------------------------------------------------------------------------

def test_fw_version_parse():
    fw = vi.parse_fw_version(fw_version_payload())
    assert fw["hw_name"] == "75_100_V2"
    assert fw["fw_version"] == "6.05"
    assert (fw["fw_major"], fw["fw_minor"]) == (6, 5)
    assert fw["uuid"] == UUID.hex()
    assert fw["fw_test_version"] == 0
    assert fw["custom_config_num"] == 1
    assert fw["fw_name"] == ""
    assert fw["hw_crc"] == 0xDEADBEEF


def test_fw_version_parse_minimal_reply():
    p = bytes([0, 5, 2]) + b"HW_X\x00" + UUID
    fw = vi.parse_fw_version(p)
    assert fw["fw_version"] == "5.02" and fw["hw_name"] == "HW_X"
    assert "fw_test_version" not in fw


def test_fw_version_rejects_wrong_command():
    with pytest.raises(vi.DecodeError):
        vi.parse_fw_version(bytes([4]) + fw_version_payload()[1:])


def test_get_values_parse():
    v = vi.parse_get_values(get_values_payload())
    assert v["temp_fet_c"] == pytest.approx(31.5)
    assert v["temp_motor_c"] == pytest.approx(-1.2)
    assert v["avg_motor_current_a"] == pytest.approx(12.34)
    assert v["avg_input_current_a"] == pytest.approx(-3.21)
    assert v["duty"] == pytest.approx(0.25)
    assert v["erpm"] == -1500
    assert v["v_in"] == pytest.approx(51.7)
    assert v["amp_hours"] == pytest.approx(0.1234)
    assert v["amp_hours_charged"] == pytest.approx(0.0567)
    assert v["watt_hours"] == pytest.approx(6.5432)
    assert v["watt_hours_charged"] == pytest.approx(1.2345)
    assert v["tachometer"] == 12345 and v["tachometer_abs"] == 54321
    assert v["fault_code"] == 2 and v["fault_name"] == "FAULT_CODE_UNDER_VOLTAGE"
    assert v["pid_pos_deg"] == pytest.approx(90.0)
    assert v["controller_id"] == 2
    assert v["temp_mos3_c"] == pytest.approx(30.3)
    assert v["avg_vd_v"] == pytest.approx(1.5) and v["avg_vq_v"] == pytest.approx(-2.5)
    assert v["timeout_active"] is False


def test_get_values_truncated_raises():
    with pytest.raises(vi.DecodeError, match="too short"):
        vi.parse_get_values(get_values_payload()[:20])


# ---------------------------------------------------------------------------
# MCCONF decode
# ---------------------------------------------------------------------------

def test_mcconf_layout_matches_serialize_side():
    spec = [ln.split() for ln in MCCONF_SERIALIZE_SPEC.strip().splitlines()]
    assert len(spec) == len(vi.MCCONF_FIELDS_6_05) == 196
    for (kind, c_name), (name, tkind, scale) in zip(spec, vi.MCCONF_FIELDS_6_05):
        assert _py_name(c_name) == name
        if kind.startswith("f16:"):
            assert tkind == "f16" and scale == float(kind.split(":")[1])
        elif name.startswith("hall_table_"):
            assert (kind, tkind) == ("u8", "i8")   # int8_t written as a byte
        else:
            assert kind == tkind
    assert vi.MCCONF_BODY_LEN_6_05 == 477
    assert vi.MCCONF_SIGNATURE_6_05 == MCCONF_SIGNATURE


def test_mcconf_decode_6_05():
    payload, expected = build_mcconf_payload()
    mc = vi.decode_mcconf(payload)
    assert mc["signature"] == MCCONF_SIGNATURE
    assert mc["length"] == 478 and mc["trailing_bytes"] == 0
    fields = mc["fields"]
    assert set(fields) == set(expected)
    for name, want in expected.items():
        got = fields[name]
        if isinstance(want, float):
            assert got == pytest.approx(want, rel=1e-6, abs=1e-9), name
        else:
            assert got == want, name
    # The values Kevin needs, spelled out.
    assert fields["l_min_vin"] == pytest.approx(8.0)
    assert fields["l_max_vin"] == pytest.approx(57.0)
    assert fields["l_battery_cut_start"] == pytest.approx(42.0)
    assert fields["l_battery_cut_end"] == pytest.approx(39.0)
    assert fields["l_battery_regen_cut_start"] == pytest.approx(52.5)
    assert fields["l_battery_regen_cut_end"] == pytest.approx(53.5)
    assert fields["l_current_min"] == -60.0
    assert fields["l_in_current_min"] == -20.0
    assert fields["l_max_erpm"] == 100000.0
    assert fields["l_watt_min"] == -1500000.0
    assert fields["l_temp_fet_start"] == 85 and fields["l_temp_fet_end"] == 100
    assert fields["si_battery_cells"] == 13
    assert fields["si_battery_ah"] == 20.0
    assert fields["hall_table_1"] == -1


def test_float32_auto_roundtrip():
    for v in (0.0, 1.0, -1.0, 0.5, 60.0, -60.0, 1e-6, 123456.789, -1.5e6, 3.0e38):
        raw = struct.unpack(">I", enc_float32_auto(v))[0]
        assert vi.float32_auto_decode(raw) == _f32(v)


def test_mcconf_signature_mismatch_refuses_to_decode():
    payload, _ = build_mcconf_payload(signature=0x12345678)
    with pytest.raises(vi.McconfSignatureMismatch) as ei:
        vi.decode_mcconf(payload)
    assert ei.value.length == 478
    assert ei.value.signature == 0x12345678
    assert "478" in str(ei.value) and str(0x12345678) in str(ei.value)


def test_mcconf_signature_mismatch_reported_by_collect():
    replies = _all_replies((2,))
    bad, _ = build_mcconf_payload(signature=999)
    replies[(2, 14)] = bad
    client = vi.VescClient(FakeBus(replies), FakeMsg, timeout_s=0.05, retries=0)
    res = vi.collect(client, 2)
    assert res["ok"] is False
    assert "mcconf" not in res
    assert res["mcconf_error"] == {"reason": "signature_mismatch", "length": 478,
                                   "signature": 999,
                                   "expected_signature": MCCONF_SIGNATURE}
    text = vi.format_report(res)
    assert "signature mismatch" in text and "478 bytes" in text and "999" in text
    assert "l_max_vin" not in text


def test_mcconf_truncated_raises():
    payload, _ = build_mcconf_payload()
    with pytest.raises(vi.DecodeError, match="too short"):
        vi.decode_mcconf(payload[:200])


# ---------------------------------------------------------------------------
# Read-only allowlist
# ---------------------------------------------------------------------------

def test_allowlist_is_exactly_the_read_commands():
    assert vi.READ_ONLY_COMMANDS == frozenset({0, 4, 14})


@pytest.mark.parametrize("cmd", [1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 255, -1])
def test_allowlist_refuses_non_read_command(cmd):
    bus = FakeBus()
    client = vi.VescClient(bus, FakeMsg, timeout_s=0.01, retries=0)
    with pytest.raises(vi.ReadOnlyViolation):
        client.request(2, cmd)
    with pytest.raises(vi.ReadOnlyViolation):
        vi.build_request(2, cmd)
    with pytest.raises(AssertionError):     # it is an assertion, kept under -O
        vi.check_read_only(cmd)
    assert bus.sent == []


def test_guard_refuses_non_query_frames():
    # SET_RPM (3) to VESC 1: the frame wall-e sends. Never from this tool.
    with pytest.raises(vi.ReadOnlyViolation):
        vi.guard_frame((3 << 8) | 1, bytes(4), True)
    # Right packet type, write command (COMM_SET_MCCONF = 13).
    with pytest.raises(vi.ReadOnlyViolation):
        vi.guard_frame((8 << 8) | 1, bytes([SENDER, 0, 13]), True)
    # Broadcast, standard id, extra payload.
    with pytest.raises(vi.ReadOnlyViolation):
        vi.guard_frame((8 << 8) | 255, bytes([SENDER, 0, 0]), True)
    with pytest.raises(vi.ReadOnlyViolation):
        vi.guard_frame((8 << 8) | 1, bytes([SENDER, 0, 0]), False)
    with pytest.raises(vi.ReadOnlyViolation):
        vi.guard_frame((8 << 8) | 1, bytes([SENDER, 0, 0, 1]), True)
    vi.guard_frame((8 << 8) | 1, bytes([SENDER, 0, 14]), True)   # allowed


def test_guard_checks_the_message_object_actually_sent():
    class EvilMsg(FakeMsg):
        def __init__(self, arbitration_id, data, is_extended_id=True):
            super().__init__((3 << 8) | (arbitration_id & 0xFF), data, is_extended_id)

    bus = FakeBus()
    client = vi.VescClient(bus, EvilMsg, timeout_s=0.01, retries=0)
    with pytest.raises(vi.ReadOnlyViolation):
        client.request(2, 0)
    assert bus.sent == []


def test_vesc_id_validation():
    for bad in (255, 0x7E, -1, 256):
        with pytest.raises(ValueError):
            vi.build_request(bad, 0)
    assert vi.build_request(2, 0) == ((8 << 8) | 2, bytes([0x7E, 0, 0]))
    assert vi.build_request(1, 14) == (0x801, bytes([0x7E, 0, 14]))


# ---------------------------------------------------------------------------
# Client / main
# ---------------------------------------------------------------------------

def test_client_request_roundtrip_ignores_other_traffic():
    bus = FakeBus(_all_replies())
    client = vi.VescClient(bus, FakeMsg, timeout_s=0.2, retries=0)
    assert client.request(1, 4) == get_values_payload()
    assert client.request(2, 14) == build_mcconf_payload()[0]
    _assert_sent_frames_are_read_only(bus)


def test_client_ignores_reply_from_wrong_vesc():
    bus = FakeBus({(2, 0): fw_version_payload()})
    orig_send = bus.send

    def send_as_other(msg):
        orig_send(msg)
        # Rewrite every completed reply to look like it came from VESC 9.
        for m in bus.rx:
            if m.arbitration_id >> 8 in (7, 8):
                m.data = bytes([9]) + m.data[1:]

    bus.send = send_as_other
    client = vi.VescClient(bus, FakeMsg, timeout_s=0.05, retries=0)
    with pytest.raises(vi.QueryTimeout):
        client.request(2, 0)


def test_client_timeout_is_clean():
    bus = FakeBus({})
    client = vi.VescClient(bus, FakeMsg, timeout_s=0.05, retries=1)
    with pytest.raises(vi.QueryTimeout, match="2 attempt"):
        client.request(2, 0)
    assert len(bus.sent) == 2
    _assert_sent_frames_are_read_only(bus)


def test_client_retries_after_dropped_frame():
    bus = FakeBus(_all_replies(), drop_first=1)
    client = vi.VescClient(bus, FakeMsg, timeout_s=0.2, retries=2)
    assert client.request(2, 14) == build_mcconf_payload()[0]
    assert len(bus.sent) == 2


def test_main_json(capsys):
    bus = FakeBus(_all_replies())
    rc = vi.main(["--json"], bus_opener=lambda ch: (bus, FakeMsg))
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert [v["id"] for v in out["vescs"]] == [2, 1]
    left = out["vescs"][0]
    assert left["side"] == "left" and left["ok"] is True
    assert left["fw_version"]["hw_name"] == "75_100_V2"
    assert left["fw_version"]["fw_version"] == "6.05"
    assert left["values"]["v_in"] == pytest.approx(51.7)
    assert left["mcconf"]["fields"]["l_battery_regen_cut_start"] == pytest.approx(52.5)
    assert left["mcconf"]["fields"]["l_max_vin"] == pytest.approx(57.0)
    assert bus.shutdown_called
    _assert_sent_frames_are_read_only(bus)


def test_main_text_single_id(capsys):
    bus = FakeBus(_all_replies((1,)))
    rc = vi.main(["--id", "1"], bus_opener=lambda ch: (bus, FakeMsg))
    assert rc == 0
    text = capsys.readouterr().out
    assert "VESC id 1 (right)" in text
    assert "75_100_V2" in text and "6.05" in text
    for name in ("l_current_max", "l_current_min", "l_in_current_max", "l_in_current_min",
                 "l_abs_current_max", "l_max_erpm", "l_min_vin", "l_max_vin",
                 "l_battery_cut_start", "l_battery_cut_end", "l_battery_regen_cut_start",
                 "l_battery_regen_cut_end", "l_temp_fet_start", "l_temp_fet_end",
                 "l_watt_max", "l_watt_min", "si_battery_cells", "si_battery_ah"):
        assert name in text, name
    assert all(m.arbitration_id == 0x801 for m in bus.sent)


def test_main_no_reply_exits_nonzero(capsys):
    bus = FakeBus({})
    rc = vi.main(["--timeout", "0.02", "--retries", "0"],
                 bus_opener=lambda ch: (bus, FakeMsg))
    assert rc == 1
    out = capsys.readouterr().out
    assert "no reply" in out
    # FW_VERSION timed out, so nothing else is asked of that VESC.
    assert [m.data[2] for m in bus.sent] == [0, 0]


def test_main_bus_open_failure(capsys):
    def boom(ch):
        raise OSError("No such device")
    assert vi.main([], bus_opener=boom) == 2
    assert "cannot open can0" in capsys.readouterr().err


def test_main_rejects_broadcast_id():
    with pytest.raises(SystemExit):
        vi.main(["--id", "255"], bus_opener=lambda ch: (FakeBus(), FakeMsg))
