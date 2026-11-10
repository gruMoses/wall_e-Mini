"""Phone / SPP override freshness on the monotonic clock (2026-11-10).

``main.py`` used ``time.time() - last_update_epoch_s <= 0.6`` with no lower
limit: a backward wall-clock step (this Pi's clock steps at boot) gave a
negative age, and a negative age passed as fresh. The reader now compares the
writer's ``time.monotonic()`` stamp with its own, and a missing, non-finite or
negative age is stale.
"""

import json
import math

import pytest

from pi_app.app.phone_override import (
    OVERRIDE_FRESH_S,
    fresh_override_bytes,
    override_age_s,
)
from pi_app.web.teleop import FileCommandSink


def _payload(mono, epoch=1_000_000.0, left=200, right=210):
    return {"left_byte": left, "right_byte": right,
            "last_update_mono_s": mono, "last_update_epoch_s": epoch}


def test_window_is_unchanged():
    assert OVERRIDE_FRESH_S == 0.6


def test_fresh_command_is_used():
    override, age = fresh_override_bytes(_payload(100.0), 100.3)
    assert override == (200, 210)
    assert age == pytest.approx(0.3)


def test_age_at_the_limit_is_fresh_and_past_it_is_stale():
    assert fresh_override_bytes(_payload(100.0), 100.5)[0] == (200, 210)
    assert fresh_override_bytes(_payload(100.0), 100.61)[0] is None


def test_negative_age_is_stale():
    # Old code: a negative age passed (age <= 0.6). Now stale.
    override, age = fresh_override_bytes(_payload(100.0), 99.0)
    assert override is None
    assert age == pytest.approx(-1.0)


def test_wall_clock_plays_no_part():
    # The epoch stamp says "just now" but the monotonic stamp is 5 s old.
    data = _payload(100.0, epoch=1_000_000.0)
    assert fresh_override_bytes(data, 105.0)[0] is None


@pytest.mark.parametrize(
    "stamp", [float("nan"), float("inf"), float("-inf"), None, "100.0", True]
)
def test_bad_stamp_is_stale(stamp):
    data = _payload(100.0)
    data["last_update_mono_s"] = stamp
    override, age = fresh_override_bytes(data, 100.1)
    assert override is None
    assert age is None or not math.isfinite(age) or age < 0


def test_epoch_only_payload_from_an_old_writer_is_stale():
    data = {"left_byte": 200, "right_byte": 210, "last_update_epoch_s": 100.0}
    assert override_age_s(data, 100.1) is None
    assert fresh_override_bytes(data, 100.1) == (None, None)


def test_non_dict_is_stale():
    assert fresh_override_bytes(["x"], 1.0) == (None, None)


def test_teleop_sink_writes_a_monotonic_stamp(tmp_path, monkeypatch):
    import pi_app.web.teleop as teleop

    monkeypatch.setattr(teleop.time, "monotonic", lambda: 42.0)
    path = tmp_path / "ov.json"
    FileCommandSink(str(path))(150, 160)
    data = json.loads(path.read_text())
    assert data["last_update_mono_s"] == 42.0
    assert fresh_override_bytes(data, 42.2)[0] == (150, 160)


# ── Boot ID: the monotonic clock starts again at each boot ─────────────────

def test_command_from_an_earlier_boot_is_stale_at_the_same_uptime():
    # The power went off during a phone drive; /tmp kept the file. In the
    # next boot the uptime passes the old stamp: the age looks fresh.
    data = {**_payload(5000.0), "boot_id": "boot-A"}
    override, age = fresh_override_bytes(data, 5000.2, boot_id="boot-B")
    assert override is None
    assert age == pytest.approx(0.2)


def test_command_with_no_boot_id_is_stale_when_the_reader_has_one():
    assert fresh_override_bytes(_payload(100.0), 100.1, boot_id="boot-B")[0] is None


def test_command_from_this_boot_is_fresh():
    data = {**_payload(100.0), "boot_id": "boot-B"}
    assert fresh_override_bytes(data, 100.1, boot_id="boot-B")[0] == (200, 210)


def test_without_a_reader_boot_id_only_the_age_decides():
    # Not Linux (the tests on a Mac): there is no boot ID to compare.
    assert fresh_override_bytes(_payload(100.0), 100.1, boot_id=None)[0] == (200, 210)


def test_read_boot_id(tmp_path):
    from pi_app.app.phone_override import read_boot_id

    f = tmp_path / "boot_id"
    f.write_text("0f3c2a9e-1111-2222-3333-444455556666\n")
    assert read_boot_id(str(f)) == "0f3c2a9e-1111-2222-3333-444455556666"
    assert read_boot_id(str(tmp_path / "missing")) is None
    (tmp_path / "empty").write_text("\n")
    assert read_boot_id(str(tmp_path / "empty")) is None


def test_writers_store_the_boot_id(tmp_path, monkeypatch):
    import pi_app.app.phone_override as po
    import pi_app.cli.spp_server as spp

    monkeypatch.setattr(po, "BOOT_ID", "boot-B")
    path = tmp_path / "ov.json"
    FileCommandSink(str(path))(150, 160)
    assert json.loads(path.read_text())["boot_id"] == "boot-B"

    monkeypatch.setenv(spp.SPP_CONTROL_ENV_VAR, "1")
    path2 = tmp_path / "ov2.json"
    assert spp.write_control_override(150, 160, path=str(path2)) is True
    data = json.loads(path2.read_text())
    assert data["boot_id"] == "boot-B"
    # The reader, in the same boot, accepts it.
    assert fresh_override_bytes(data, data["last_update_mono_s"] + 0.1)[0] == (150, 160)
