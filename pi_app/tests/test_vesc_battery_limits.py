"""
Tests for tools/vesc_battery_limits.py (part A writer for the drive VESCs).

No hardware: a fake bus plays two VESCs. It answers the read queries the way
vesc_info's tests do, rebuilds a written CAN buffer with the firmware's
offset-matched fill logic (comm_can.c, including the "reset all buffers"
path), checks length and CRC, and on a valid COMM_SET_MCCONF applies the
config, optionally stores it to a separate "flash" copy, and replies the
one-byte ack. COMM_REBOOT reloads the live config from that flash copy and
re-measures the FOC offsets. The f16 re-quantization model it uses was
cross-checked against C float32 arithmetic for every int16 at every scale in
an independent review (2026-11-11).
"""

from __future__ import annotations

import glob
import json
import os
import struct
import sys
from typing import Dict, Iterable, List, Optional

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "tools"))
for _p in (_TOOLS_DIR, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import test_vesc_info as tvi  # noqa: E402
import vesc_battery_limits as vbl  # noqa: E402
import vesc_info as vi  # noqa: E402

SENDER = vi.SENDER_ID


# ---------------------------------------------------------------------------
# Fake VESCs
# ---------------------------------------------------------------------------

def fw_payload(uuid: bytes, hw: str = "75_100_V2", major: int = 6, minor: int = 5,
               test_ver: int = 0) -> bytes:
    p = bytes([0, major, minor]) + hw.encode() + b"\x00" + uuid
    p += bytes([0, test_ver, 0, 1, 0, 0, 0, 0]) + b"\x00" + tvi.enc_u32(0xDEADBEEF)
    return p


def values_payload(erpm: int = 0, current_a: float = 0.0, fault: int = 0,
                   v_in: float = 50.5, timeout_active: Optional[bool] = True,
                   controller_id: int = 2) -> bytes:
    p = bytes([4])
    p += tvi.enc_i16(250) + tvi.enc_i16(200)          # temps
    p += tvi.enc_i32(0)                               # avg motor current
    p += tvi.enc_i32(int(round(current_a * 100)))     # avg input current
    p += tvi.enc_i32(0) + tvi.enc_i32(0)              # id, iq
    p += tvi.enc_i16(0)                               # duty
    p += tvi.enc_i32(erpm)
    p += tvi.enc_i16(int(round(v_in * 10)))
    p += tvi.enc_i32(0) * 4                           # Ah, Ah ch, Wh, Wh ch
    p += tvi.enc_i32(0) * 2                           # tacho, tacho abs
    p += bytes([fault])
    if timeout_active is None:
        return p                                      # old firmware: no tail
    p += tvi.enc_i32(0)                               # pid pos
    p += bytes([controller_id])
    p += tvi.enc_i16(250) * 3                         # mos temps
    p += tvi.enc_i32(0) + tvi.enc_i32(0)              # vd, vq
    p += bytes([1 if timeout_active else 0])          # status
    return p


def defaults_body(like: bytes) -> bytes:
    """What a VESC holds after it booted on firmware defaults."""
    return vbl.patch_body(like, {
        "l_in_current_max": 99.0, "l_in_current_min": -60.0,
        "l_battery_cut_start": 10.0, "l_battery_cut_end": 8.0,
        "l_battery_regen_cut_start": 1000.0, "l_battery_regen_cut_end": 1100.0,
        "si_battery_cells": 3, "l_current_max": 60.0})


class Vesc:
    def __init__(self, vid: int, cells: Optional[int] = None) -> None:
        payload, _ = tvi.build_mcconf_payload()
        before = dict(vbl.BEFORE[vid])
        if cells is not None:
            before["si_battery_cells"] = cells
        self.id = vid
        body = bytearray(vbl.patch_body(payload[1:], before))
        body[vbl.LAYOUT["foc_offsets_cal_on_boot"][0]] = 1   # as on both real units
        self.body = bytes(body)   # live
        self.flash = self.body
        self.uuid = bytes.fromhex(vbl.EXPECTED_UUID[vid])
        self.fw = fw_payload(self.uuid)
        self.values_seq: List[bytes] = [values_payload(controller_id=vid)]
        self.writes = 0           # COMM_SET_MCCONF applied
        self.reboots = 0
        self.lose_writes = 0      # drop the next N writes (as if frames were lost)
        self.drop_acks = 0        # apply, but do not ack, the next N writes
        self.store_fails = False  # apply, ack, but flash keeps the old copy
        self.flash_corrupt = False  # next boot loads defaults
        self.boot_silent = 1      # FW_VERSION queries ignored after a reboot (>= 1)
        self.ignore_reboot = False  # COMM_REBOOT lost
        self.same_offsets = False   # boot calibration gives bit-identical offsets
        self.alter: Optional[Dict[str, float]] = None   # firmware changes on store

    @property
    def values(self) -> bytes:
        return self.values_seq[0] if len(self.values_seq) == 1 else self.values_seq.pop(0)

    @values.setter
    def values(self, v: bytes) -> None:
        self.values_seq = [v]


class FakeBus:
    def __init__(self, vescs: List[Vesc], on_first_write_frame=None,
                 send_error_after: Optional[int] = None) -> None:
        self.vescs = {v.id: v for v in vescs}
        self.sent: List[tvi.FakeMsg] = []
        self.rx: List[tvi.FakeMsg] = []
        self.on_first_write_frame = on_first_write_frame
        self.send_error_after = send_error_after
        self._first_seen = False
        self._buf: Dict[int, bytearray] = {}
        self._off: Dict[int, int] = {}

    def _reply(self, src: int, dest: int, payload: bytes) -> None:
        self.rx.append(tvi.FakeMsg((9 << 8) | src, bytes(8)))   # unrelated STATUS
        self.rx.extend(tvi.FakeMsg(a, d) for a, d in tvi.vesc_reply_frames(src, payload, dest=dest))

    def _reboot(self, v: Vesc) -> None:
        if v.ignore_reboot:
            return
        v.reboots += 1
        before = v.body
        live = bytearray(defaults_body(v.flash) if v.flash_corrupt else v.flash)
        off = vbl.LAYOUT["foc_offsets_current_0"][0]
        if v.same_offsets:
            live[off:off + 4] = before[off:off + 4]
        else:
            live[off:off + 4] = vbl.float32_auto_encode(2048.0 + v.reboots)  # re-measured
        v.body = bytes(live)
        v.boot_silent_left = max(1, v.boot_silent)

    def send(self, msg) -> None:
        if self.send_error_after is not None:
            if self.send_error_after <= 0 and (msg.arbitration_id >> 8) != 8:
                raise OSError("ENOBUFS")
            if (msg.arbitration_id >> 8) != 8:
                self.send_error_after -= 1
        self.sent.append(msg)
        arb, data = int(msg.arbitration_id), bytes(msg.data)
        pkt, target = arb >> 8, arb & 0xFF
        v = self.vescs.get(target)
        if v is None:
            return
        if pkt == 8:
            cmd = data[2]
            if cmd == 29:
                self._reboot(v)
                return
            if cmd == 0 and getattr(v, "boot_silent_left", 0) > 0:
                v.boot_silent_left -= 1
                return
            payload = {0: v.fw, 4: v.values, 14: bytes([14]) + v.body}.get(cmd)
            if payload is not None and data[1] == 0:
                self._reply(v.id, data[0], payload)
            return
        if pkt in (5, 6) and not self._first_seen:
            self._first_seen = True
            if self.on_first_write_frame:
                self.on_first_write_frame()
        if pkt == 5:
            off, chunk = data[0], data[1:]
        elif pkt == 6:
            off, chunk = (data[0] << 8) | data[1], data[2:]
        elif pkt == 7:
            n = (data[2] << 8) | data[3]
            crc = (data[4] << 8) | data[5]
            if self._off.get(target) != n:
                self._off[target] = 0          # "something is wrong, reset all buffers"
                return
            self._off[target] = 0
            buf = bytes(self._buf.get(target, b""))
            if vi.crc16(buf[:n]) != crc:
                return
            payload = buf[:n]
            if payload[0] != 13:
                return
            if struct.unpack(">I", payload[1:5])[0] != vi.MCCONF_SIGNATURE_6_05:
                return
            if v.lose_writes > 0:
                v.lose_writes -= 1
                return
            v.writes += 1
            live = bytearray(vbl.apply_predicted_drift(payload[1:]))
            if v.alter:
                for name, value in v.alter.items():
                    o, s, _k, _sc = vbl.LAYOUT[name]
                    live[o:o + s] = vbl.encode_field(name, value)
            v.body = bytes(live)
            if not v.store_fails:
                v.flash = v.body
            if v.drop_acks > 0:
                v.drop_acks -= 1
                return
            self._reply(v.id, data[0], bytes([13]))
            return
        else:
            return
        if off == 0:
            self._buf[target] = bytearray(512)
            self._off[target] = 0
        if self._off.get(target) != off:
            return
        self._buf[target][off:off + len(chunk)] = chunk
        self._off[target] = off + len(chunk)

    def recv(self, timeout=None):
        return self.rx.pop(0) if self.rx else None

    def shutdown(self) -> None:
        pass

    def write_frames(self) -> List[tvi.FakeMsg]:
        return [m for m in self.sent if (m.arbitration_id >> 8) != 8]

    def config_writes(self, vid: Optional[int] = None) -> List[tvi.FakeMsg]:
        return [m for m in self.sent if (m.arbitration_id >> 8) == 7 and m.data[1] == 0
                and (vid is None or m.arbitration_id & 0xFF == vid)]

    def reboot_frames(self) -> List[tvi.FakeMsg]:
        return [m for m in self.sent if (m.arbitration_id >> 8) == 8 and m.data[2] == 29]


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(vbl, "ACK_TIMEOUT_S", 0.05)
    monkeypatch.setattr(vbl, "REBOOT_WAIT_S", 0.5)
    monkeypatch.setattr(vbl, "REBOOT_NO_SILENCE_S", 0.05)
    monkeypatch.setattr(vbl, "QUICK_TIMEOUT_S", 0.005)
    monkeypatch.setattr(vbl, "OFFSETS_WAIT_S", 0.05)


def run(bus: FakeBus, argv, tmp_path, service="inactive", hold: bool = True) -> int:
    hold_path = tmp_path / "deploy_hold"
    if hold:
        hold_path.touch()
    states: Iterable[str] = [service] if isinstance(service, str) else list(service)
    seq = list(states)

    def probe() -> str:
        return seq.pop(0) if len(seq) > 1 else seq[0]

    return vbl.main(list(argv) + ["--backup-dir", str(tmp_path / "bk"),
                                  "--deploy-hold", str(hold_path), "--timeout", "0.02"],
                    bus_opener=lambda ch: (bus, tvi.FakeMsg),
                    service_probe=probe,
                    sleep=lambda s: None)


def _two():
    return Vesc(2), Vesc(1)


def _backups(tmp_path) -> List[str]:
    return sorted(glob.glob(str(tmp_path / "bk" / "*.bin")))


# ---------------------------------------------------------------------------
# Table, layout, encoding
# ---------------------------------------------------------------------------

def test_targets_match_battery_settings_part_a():
    vbl.check_targets_table()
    assert vbl.TARGETS[2] == {
        "l_in_current_max": 15.0, "l_in_current_min": -5.0,
        "l_battery_cut_start": 44.2, "l_battery_cut_end": 41.6,
        "l_battery_regen_cut_start": 54.4, "l_battery_regen_cut_end": 54.9,
        "si_battery_cells": 13}
    assert vbl.TARGETS[1]["l_battery_cut_start"] == 45.1
    assert vbl.TARGETS[1]["l_battery_regen_cut_end"] == 55.8
    assert vbl.BEFORE[1]["si_battery_cells"] == 12


def test_layout_covers_the_body_and_target_kinds():
    assert vbl.BODY_LEN == 477
    kinds = {n: vbl.LAYOUT[n][2] for n in vbl.TARGET_FIELDS}
    assert kinds["l_in_current_max"] == "f32a"
    assert kinds["l_battery_regen_cut_end"] == "f16"
    assert kinds["si_battery_cells"] == "u8"
    assert all(n in vbl.LAYOUT for n in vbl.BOOT_MEASURED)


@pytest.mark.parametrize("value", [15.0, -5.0, 0.0, 40.0, -40.0, 1e-3, 123456.789])
def test_float32_auto_encoder_matches_firmware(value):
    assert vbl.float32_auto_encode(value) == tvi.enc_float32_auto(value)
    raw = struct.unpack(">I", vbl.float32_auto_encode(value))[0]
    assert vi.float32_auto_decode(raw) == vi._f32(value)


def test_f16_targets_are_exact_quanta_and_read_back_stable():
    for v, raw in ((44.2, 442), (41.6, 416), (54.4, 544), (54.9, 549), (55.8, 558)):
        assert vbl.f16_raw(v, 10.0) == raw
        assert vbl.f16_readback_raw(raw, 10.0) == raw


def test_firmware_f16_round_trip_is_stable_for_scale_10():
    for k in range(-32768, 32768, 7):
        assert vbl.f16_readback_raw(k, 10.0) == k


# ---------------------------------------------------------------------------
# Patch proof and config checks
# ---------------------------------------------------------------------------

def test_patch_changes_only_the_target_fields():
    old = Vesc(1).body
    new = vbl.patch_body(old, vbl.TARGETS[1])
    vbl.prove_patch(old, new, vbl.TARGETS[1])
    assert set(vbl.changed_fields(old, new)) == set(vbl.TARGET_FIELDS)
    dec = vbl.decode_body(new)
    assert dec["si_battery_cells"] == 13
    assert dec["l_in_current_min"] == -5.0
    assert abs(dec["l_battery_regen_cut_start"] - 55.3) < 1e-5


def test_prove_patch_rejects_a_stray_byte_or_signature():
    old = Vesc(2).body
    new = bytearray(vbl.patch_body(old, vbl.TARGETS[2]))
    off = vbl.LAYOUT["l_current_max"][0]
    new[off] ^= 0x01
    with pytest.raises(vbl.WriteRefused):
        vbl.prove_patch(old, bytes(new), vbl.TARGETS[2])
    new2 = bytearray(vbl.patch_body(old, vbl.TARGETS[2]))
    new2[0] ^= 0x01
    with pytest.raises(vbl.WriteRefused):
        vbl.prove_patch(old, bytes(new2), vbl.TARGETS[2])


def test_config_refusals_catch_a_defaults_boot(tmp_path):
    body = defaults_body(Vesc(2).body)
    reasons = vbl.config_refusals(2, body, str(tmp_path), "x", accept_live=False)
    assert any("si_battery_cells" in r for r in reasons)
    assert any("l_in_current_max" in r for r in reasons)
    assert vbl.config_refusals(2, Vesc(2).body, str(tmp_path), "x", False) == []


# ---------------------------------------------------------------------------
# Framing and guard
# ---------------------------------------------------------------------------

def test_write_frames_are_firmware_framing_and_reassemble():
    body = vbl.patch_body(Vesc(2).body, vbl.TARGETS[2])
    payload = bytes([13]) + body
    frames = vbl.build_write_frames(2, payload)
    fw_frames = tvi.vesc_reply_frames(SENDER, payload, dest=2, send=0)
    assert [(a, d) for a, d in frames] == [(a, d) for a, d in fw_frames]
    assert len(frames) == 75
    assert max(d[0] for a, d in frames if a >> 8 == 5) == 252
    assert vbl.receiver_reassemble(frames, 2) == payload
    vbl.guard_write(2, frames, body)


@pytest.mark.parametrize("vid", [0, 3, 255, SENDER])
def test_guard_refuses_other_targets(vid):
    body = Vesc(2).body
    frames = vbl.build_write_frames(vid, bytes([13]) + body)
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_write(vid, frames, body)
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_single(vid, vbl.reboot_frame(vid))


def test_guard_refuses_tampered_or_foreign_payloads():
    body = vbl.patch_body(Vesc(2).body, vbl.TARGETS[2])
    frames = vbl.build_write_frames(2, bytes([13]) + body)
    bad = list(frames)
    a, d = bad[3]
    bad[3] = (a, d[:2] + bytes([d[2] ^ 0xFF]) + d[3:])
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_write(2, bad, body)
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_write(2, vbl.build_write_frames(2, bytes([16]) + body), body)
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_write(2, frames, Vesc(2).body)
    a7, _d7 = vbl.reset_frame(2)
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_single(2, ((8 << 8) | 2, bytes([SENDER, 2, 0])))   # COMM_FW... reboot-like
    with pytest.raises(vbl.WriteGuardViolation):
        vbl.guard_single(2, (a7, bytes([SENDER, 0, 0, 1, 0, 0])))


def test_reset_frame_can_never_execute_a_one_byte_buffer():
    assert all(vi.crc16(bytes([b])) != vbl.RESET_CRC for b in range(256))
    _a, d = vbl.reset_frame(1)
    assert d[1] == 2 and (d[2], d[3]) == (0, 1)


def test_reset_frame_clears_a_stale_half_buffer():
    left = Vesc(2)
    bus = FakeBus([left])
    stale = vbl.build_write_frames(2, bytes([13]) + defaults_body(left.body))[:20]
    for a, d in stale:
        bus.send(tvi.FakeMsg(a, d))
    w = vbl.Writer(bus, tvi.FakeMsg, sleep=lambda s: None)
    new = vbl.patch_body(left.body, vbl.TARGETS[2])
    assert w.send_config(2, new) is True
    assert left.body == vbl.apply_predicted_drift(new)


# ---------------------------------------------------------------------------
# End to end
# ---------------------------------------------------------------------------

def test_dry_run_sends_only_read_queries(tmp_path, capsys):
    left, right = _two()
    for v in (left, right):
        v.values = values_payload(timeout_active=False, controller_id=v.id)
    bus = FakeBus([left, right])
    rc = run(bus, [], tmp_path, service="active", hold=False)
    out = capsys.readouterr().out
    assert rc == 0
    assert "DRY RUN" in out and "would be REFUSED" in out
    assert "wall-e.service is active" in out and "deploy hold" in out
    assert "still receiving motor commands" in out
    assert not bus.write_frames() and not bus.reboot_frames()
    tvi._assert_sent_frames_are_read_only(bus)
    assert not _backups(tmp_path)


def test_write_both_backs_up_first_reboots_and_verifies(tmp_path, capsys):
    left, right = _two()
    seen = {}
    bus = FakeBus([left, right],
                  on_first_write_frame=lambda: seen.setdefault("b", _backups(tmp_path)))
    old_left, old_right = left.body, right.body
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert len(seen["b"]) == 1 and "_id2_" in seen["b"][0]
    assert left.writes == right.writes == 1 and left.reboots == right.reboots == 1
    skip = vbl.BOOT_MEASURED
    want = vbl.apply_predicted_drift(vbl.patch_body(old_left, vbl.TARGETS[2]))
    assert not vbl.changed_fields(left.flash, want, skip)
    assert vbl.decode_body(right.flash)["si_battery_cells"] == 13
    assert out.count("VERIFIED: live and stored") == 2
    backups = _backups(tmp_path)
    assert len(backups) == 2
    assert {open(p, "rb").read() for p in backups} == {old_left, old_right}
    meta = json.load(open(backups[0][:-4] + ".json"))
    assert len(meta["sha256"]) == 64
    # A reset frame precedes each config write.
    seq = [(m.arbitration_id >> 8, m.data[1]) for m in bus.write_frames()]
    assert seq[0] == (7, 2)


def test_second_run_after_success_proves_the_flash_again_without_writing(tmp_path, capsys):
    left, right = _two()
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    capsys.readouterr()
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert out.count("already at target: no write; proving") == 2
    assert not bus.config_writes() and len(bus.reboot_frames()) == 2
    assert out.count("VERIFIED") == 2


def test_dry_run_of_a_finished_pair_is_clean(tmp_path, capsys):
    left, right = _two()
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    capsys.readouterr()
    bus = FakeBus([left, right])
    assert run(bus, [], tmp_path) == 0
    assert capsys.readouterr().out.count("already at target") == 2
    assert not bus.write_frames() and not bus.reboot_frames()


def test_rerun_after_an_unproven_store_finds_the_old_flash(tmp_path, capsys):
    # Review N1: run 1 stores nothing and loses the reboot; run 2 must not exit 0.
    left, right = _two()
    left.store_fails = True
    left.ignore_reboot = True
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 3
    capsys.readouterr()
    left.ignore_reboot = False
    left.store_fails = False
    rc = run(FakeBus([left, right]), ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc != 0, out
    assert "flash still held the OLD config" in out and left.reboots == 1
    assert vbl.decode_body(left.body)["l_in_current_max"] == 40.0
    # Run 3 now writes it properly.
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    assert vbl.decode_body(left.flash)["l_in_current_max"] == 15.0


@pytest.mark.parametrize("service", ["active", "activating", "failed", "unknown (x)"])
def test_write_refused_unless_wall_e_is_inactive(tmp_path, capsys, service):
    left, right = _two()
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path, service=service)
    assert rc == 1
    assert not bus.write_frames()
    assert "remaining VESCs were not touched" in capsys.readouterr().out


def test_write_refused_without_the_deploy_hold(tmp_path, capsys):
    left = Vesc(2)
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path, hold=False)
    assert rc == 1
    assert "deploy hold" in capsys.readouterr().out
    assert not bus.write_frames()


@pytest.mark.parametrize("values,why", [
    (values_payload(erpm=500), "motor turning"),
    (values_payload(current_a=2.5), "input current"),
    (values_payload(fault=2), "fault"),
    (values_payload(v_in=12.0), "implausible"),
    (values_payload(timeout_active=False), "still receiving motor commands"),
    (values_payload(timeout_active=None), "no status byte"),
])
def test_write_refused_when_not_still_or_commanded(tmp_path, capsys, values, why):
    left = Vesc(2)
    left.values = values
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path)
    assert rc == 1
    assert why in capsys.readouterr().out
    assert not bus.write_frames()
    assert not _backups(tmp_path)


def test_gates_are_checked_again_right_before_sending(tmp_path, capsys):
    left = Vesc(2)
    still = values_payload()
    # Initial still check uses reads 1-2; the re-check right before the send
    # reads 3-4 and sees a turning motor.
    left.values_seq = [still, still, still, values_payload(erpm=800)]
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 1, out
    assert "REFUSED just before sending" in out
    assert not bus.write_frames()


def test_service_started_after_the_first_check_is_caught(tmp_path, capsys):
    left = Vesc(2)
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path, service=["inactive", "active"])
    assert rc == 1
    assert "REFUSED just before sending" in capsys.readouterr().out
    assert not bus.write_frames()


@pytest.mark.parametrize("fw,why", [
    (fw_payload(bytes.fromhex(vbl.EXPECTED_UUID[2]), hw="60_MK6"), "hardware"),
    (fw_payload(bytes.fromhex(vbl.EXPECTED_UUID[2]), minor=6), "firmware is"),
    (fw_payload(bytes.fromhex(vbl.EXPECTED_UUID[2]), test_ver=3), "test build"),
    (fw_payload(bytes.fromhex(vbl.EXPECTED_UUID[1])), "not the one pinned"),
])
def test_write_refused_on_other_unit_hardware_or_firmware(tmp_path, capsys, fw, why):
    left = Vesc(2)
    left.fw = fw
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path)
    assert rc == 1
    assert why in capsys.readouterr().out
    assert not bus.write_frames()


def test_a_vesc_on_defaults_is_refused(tmp_path, capsys):
    left = Vesc(2)
    left.body = defaults_body(left.body)
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path)
    assert rc == 1
    assert "did it boot on defaults" in capsys.readouterr().out
    assert not bus.write_frames()


def test_live_config_must_match_the_newest_backup(tmp_path, capsys):
    left, right = _two()
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    # Something else changed a non-target field afterwards.
    left.body = vbl.patch_body(left.body, {"l_current_max": 55.0})
    capsys.readouterr()
    left.body = vbl.patch_body(left.body, vbl.BEFORE[2])     # and the 7 fields back
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 1 and "differs from the newest backup" in out and "l_current_max" in out
    assert not bus.write_frames()
    bus2 = FakeBus([left])
    rc2 = run(bus2, ["--write", "--id", "2", "--accept-live"], tmp_path)
    assert rc2 == 4      # id 2 written, id 1 not checked in this run -> split warning


def test_wrong_mcconf_signature_is_refused_without_writing(tmp_path):
    left = Vesc(2)
    left.body = struct.pack(">I", 12345) + left.body[4:]
    bus = FakeBus([left])
    assert run(bus, ["--write", "--id", "2"], tmp_path) == 1
    assert not bus.write_frames()


def test_lost_ack_but_applied_counts_as_success(tmp_path, capsys):
    left, right = _two()
    left.drop_acks = 1
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "NO ACK" in out and left.writes == 1


def test_lost_write_is_sent_once_more(tmp_path, capsys):
    left, right = _two()
    left.lose_writes = 1
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "sending once more" in out and left.writes == 1
    assert len(bus.config_writes(2)) == 2


def test_write_that_never_lands_stops_after_two_tries_as_not_written(tmp_path, capsys):
    left, right = _two()
    left.lose_writes = 5
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 1, out
    assert "NOT WRITTEN" in out and "PAIR SPLIT" not in out
    assert len(bus.config_writes(2)) == 2 and not bus.config_writes(1)
    assert right.writes == 0 and "remaining VESCs were not touched" in out
    assert not bus.reboot_frames()


def test_lost_reboot_is_never_reported_verified(tmp_path, capsys):
    left, right = _two()
    left.ignore_reboot = True
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "did not reboot" in out and "NOT proven" in out and "VERIFIED" not in out
    assert right.writes == 0


def test_reboot_without_new_offsets_is_not_proof(tmp_path, capsys):
    left, right = _two()
    left.same_offsets = True
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "reboot is not proven" in out and "VERIFIED" not in out


def test_interrupt_mid_write_is_recorded_as_failed(tmp_path, capsys):
    left, right = _two()

    class Boom(FakeBus):
        def send(self, msg):
            if (msg.arbitration_id >> 8) == 6 and not getattr(self, "_boom", False):
                self._boom = True
                raise KeyboardInterrupt()
            super().send(msg)

    bus = Boom([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "state UNKNOWN" in out and "PAIR SPLIT" in out
    assert right.writes == 0


def test_guard_violation_before_sending_is_reported_as_not_applied(tmp_path, capsys, monkeypatch):
    left, right = _two()

    def boom(*a, **k):
        raise vbl.WriteGuardViolation("test")
    monkeypatch.setattr(vbl, "guard_write", boom)
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 1, out
    assert "own guard" in out and left.writes == 0 and not bus.config_writes()


def test_exception_in_the_second_vescs_reads_still_prints_the_split(tmp_path, capsys):
    left, right = _two()

    class Flaky(FakeBus):
        def send(self, msg):
            if (msg.arbitration_id & 0xFF) == 1 and left.reboots >= 1:
                raise OSError("bus off")
            super().send(msg)

    rc = run(Flaky([left, right]), ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 4, out
    assert "OSError before any write" in out and "PAIR SPLIT" in out


def test_cal_on_boot_off_is_refused(tmp_path, capsys):
    left = Vesc(2)
    left.body = vbl.patch_body(left.body, {})
    off = vbl.LAYOUT["foc_offsets_cal_on_boot"][0]
    b = bytearray(left.body)
    b[off] = 0
    left.body = bytes(b)
    bus = FakeBus([left])
    assert run(bus, ["--write", "--id", "2"], tmp_path) == 1
    assert "foc_offsets_cal_on_boot" in capsys.readouterr().out
    assert not bus.write_frames()


def test_fault_after_a_proven_store_does_not_ask_for_a_restore(tmp_path, capsys):
    left, right = _two()

    class FaultAfterBoot(FakeBus):
        def _reboot(self, v):
            super()._reboot(v)
            v.values = values_payload(fault=5, controller_id=v.id)

    rc = run(FaultAfterBoot([left, right]), ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "no restore is needed" in out and "--restore" not in out.split("FAULT:")[1].split("\n")[0]


def test_reference_backup_ignores_restore_and_defaults_backups(tmp_path):
    fw = {"uuid": vbl.EXPECTED_UUID[2], "hw_name": "75_100_V2", "fw_version": "6.05"}
    good = Vesc(2).body
    bad = defaults_body(good)
    d = str(tmp_path)
    vbl.write_backup(d, 2, fw, good, kind="pre-write")
    vbl.write_backup(d, 2, fw, bad, kind="pre-restore")
    vbl.write_backup(d, 2, fw, bad, kind="pre-write")     # a defaults boot backed up
    ref = vbl.newest_backup(d, 2, vbl.EXPECTED_UUID[2])
    assert ref is not None and ref[0] == good


def test_dry_run_reports_an_existing_split(tmp_path, capsys):
    left, right = _two()
    left.body = vbl.patch_body(left.body, vbl.TARGETS[2])
    rc = run(FakeBus([left, right]), [], tmp_path)
    out = capsys.readouterr().out
    assert rc == 4 and "PAIR SPLIT now" in out


def test_firmware_changing_another_field_fails_verification(tmp_path, capsys):
    left = Vesc(2)
    left.alter = {"l_current_max": 55.0}
    bus = FakeBus([left])
    rc = run(bus, ["--write", "--id", "2"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3
    assert "l_current_max" in out and "--restore" in out
    assert "PAIR SPLIT" in out and "write FAILED, state uncertain" in out


def test_silent_store_failure_is_caught_by_the_reboot(tmp_path, capsys):
    left, right = _two()
    left.store_fails = True
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 1, out
    assert "back on its OLD config" in out
    assert left.reboots == 1 and right.writes == 0
    assert vbl.decode_body(left.body)["l_in_current_max"] == 40.0


def test_flash_corruption_is_caught_by_the_reboot(tmp_path, capsys):
    left, right = _two()
    left.flash_corrupt = True
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "defaults loaded?" in out and "--restore" in out
    assert right.writes == 0


def test_boot_offsets_do_not_fail_the_flash_check(tmp_path, capsys):
    left, right = _two()
    bus = FakeBus([left, right])
    assert run(bus, ["--write"], tmp_path) == 0
    off = vbl.LAYOUT["foc_offsets_current_0"][0]
    assert left.body[off:off + 4] == vbl.float32_auto_encode(2049.0)


def test_slow_boot_is_waited_for(tmp_path, capsys):
    left, right = _two()
    left.boot_silent = 3
    bus = FakeBus([left, right])
    assert run(bus, ["--write"], tmp_path) == 0, capsys.readouterr().out


def test_no_answer_after_reboot_fails(tmp_path, capsys, monkeypatch):
    left, right = _two()
    left.boot_silent = 10_000
    monkeypatch.setattr(vbl, "REBOOT_WAIT_S", 0.05)
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3 and "did not answer" in out


def test_partial_run_reports_a_split_pair(tmp_path, capsys):
    left, right = _two()
    right.values = values_payload(erpm=900, controller_id=1)
    bus = FakeBus([left, right])
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 4, out
    assert "PAIR SPLIT" in out and "Do not drive" in out
    assert "id 2 has the new limits" in out and "--restore" in out
    assert left.writes == 1 and right.writes == 0


def test_send_error_mid_write_reports_unknown_state(tmp_path, capsys):
    left, right = _two()
    bus = FakeBus([left, right], send_error_after=30)
    rc = run(bus, ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "state UNKNOWN" in out and "the OLD config" in out and "--restore" in out
    assert right.writes == 0


def test_restore_puts_the_backup_back(tmp_path, capsys):
    left, right = _two()
    original = left.body
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    backup = [p for p in _backups(tmp_path) if "_id2_" in p][0]
    capsys.readouterr()
    bus2 = FakeBus([left, right])
    rc = run(bus2, ["--restore", backup, "--write", "--id", "2"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 4, out            # restored; the other VESC may now differ
    assert "RESTORED" in out and "DIFFERENT" in out
    # Equal up to the boot offsets and the firmware's one-step f16 rounding.
    assert not vbl.changed_fields(vbl.apply_predicted_drift(original), left.body,
                                  vbl.BOOT_MEASURED)


def test_restore_refuses_a_backup_from_the_other_vesc(tmp_path, capsys):
    left, right = _two()
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    left_backup = [p for p in _backups(tmp_path) if "_id2_" in p][0]
    capsys.readouterr()
    bus2 = FakeBus([left, right])
    rc = run(bus2, ["--restore", left_backup, "--write", "--id", "1"], tmp_path)
    assert rc == 1
    assert "REFUSED: backup is for id 2" in capsys.readouterr().out
    assert not bus2.write_frames()


def test_restore_refuses_a_tampered_backup(tmp_path, capsys):
    left, right = _two()
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    backup = [p for p in _backups(tmp_path) if "_id2_" in p][0]
    data = bytearray(open(backup, "rb").read())
    data[100] ^= 0x01
    open(backup, "wb").write(bytes(data))
    rc = run(FakeBus([left, right]), ["--restore", backup, "--write", "--id", "2"], tmp_path)
    assert rc == 1


def test_backups_never_overwrite_each_other(tmp_path):
    import datetime as dt
    body = Vesc(2).body
    fw = {"uuid": vbl.EXPECTED_UUID[2], "hw_name": "75_100_V2", "fw_version": "6.05"}
    t = dt.datetime(2026, 11, 11, 12, 0, 0, 1)
    p1 = vbl.write_backup(str(tmp_path), 2, fw, body, now=t)
    with pytest.raises(FileExistsError):
        vbl.write_backup(str(tmp_path), 2, fw, body, now=t)
    p2 = vbl.write_backup(str(tmp_path), 2, fw, body, now=t.replace(microsecond=2))
    assert p1 != p2


def test_cli_rejects_non_drive_ids(tmp_path):
    with pytest.raises(SystemExit):
        run(FakeBus([]), ["--id", "5"], tmp_path)
    with pytest.raises(SystemExit):
        run(FakeBus([]), ["--restore", "x.bin"], tmp_path)


def test_undo_hint_on_an_at_target_vesc_points_at_the_pre_write_backup(tmp_path, capsys):
    left, right = _two()
    assert run(FakeBus([left, right]), ["--write"], tmp_path) == 0
    pre_write = [p for p in _backups(tmp_path) if "_id2_" in p][0]
    capsys.readouterr()
    left.flash_corrupt = True          # the proof reboot now loads defaults
    rc = run(FakeBus([left, right]), ["--write"], tmp_path)
    out = capsys.readouterr().out
    assert rc == 3, out
    assert f"--restore {pre_write} --id 2" in out
