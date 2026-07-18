"""SPP control-writer retirement tests.

Proves that Bluetooth SPP data can no longer reach the motors unless the
channel is *explicitly* opted in. The SPP server (``pi_app/cli/spp_server.py``)
runs as a standalone systemd service and used to be an uncoordinated THIRD
writer of ``/tmp/wall_e_bt_latest.json`` — the shared override file that
``main.py`` polls (600 ms freshness) and feeds into
``controller.process(bt_override_bytes=...)``. That path bypassed the entire
web-teleop session-safety layer (arming ceremony, latched e-stop, 250 ms
deadman, speed cap). It is now gated OFF by default behind
``WALL_E_SPP_CONTROL_ENABLED``.

These tests use only the file-writer helper — no Bluetooth stack, no hardware.
The fact that ``pi_app.cli.spp_server`` imports here at all is itself part of
the contract: the pybluez import is deferred into ``run_server()`` so the gate
is testable on hosts without a Bluetooth stack.
"""

import json
import time

import pytest

from pi_app.cli import spp_server
from pi_app.cli.spp_server import (
    SPP_CONTROL_ENV_VAR,
    control_output_enabled,
    write_control_override,
)


def _read_bt_override(path, *, now=None):
    """Mirror the freshness read in ``pi_app/app/main.py`` (lines 479-488).

    Returns ``(left_byte, right_byte)`` if the override file exists and is
    fresh within 600 ms, else ``None`` — exactly what the main control loop
    would pass as ``bt_override`` into ``controller.process``.
    """
    now = now if now is not None else time.time()
    try:
        with open(path, "r", encoding="utf-8") as sf:
            data = json.load(sf)
    except Exception:
        return None
    if now - data["last_update_epoch_s"] <= 0.6:
        return (data["left_byte"], data["right_byte"])
    return None


@pytest.fixture(autouse=True)
def _clear_gate_env(monkeypatch):
    """Every test starts with the gate env var unset (i.e. the default)."""
    monkeypatch.delenv(SPP_CONTROL_ENV_VAR, raising=False)


# --------------------------------------------------------------------------
# Gate predicate
# --------------------------------------------------------------------------

def test_disabled_by_default():
    """Unset env var => channel is OFF."""
    assert control_output_enabled() is False


@pytest.mark.parametrize("val", ["1", "true", "TRUE", "yes", "on", " on "])
def test_truthy_values_enable(monkeypatch, val):
    monkeypatch.setenv(SPP_CONTROL_ENV_VAR, val)
    assert control_output_enabled() is True


@pytest.mark.parametrize("val", ["0", "false", "no", "off", "", "2", "enable", "y"])
def test_falsy_values_stay_disabled(monkeypatch, val):
    monkeypatch.setenv(SPP_CONTROL_ENV_VAR, val)
    assert control_output_enabled() is False


# --------------------------------------------------------------------------
# Writer gate — the actual safety property
# --------------------------------------------------------------------------

def test_write_disabled_by_default_creates_nothing(tmp_path):
    """The default (unset) gate: SPP data writes no override file at all."""
    ov = tmp_path / "wall_e_bt_latest.json"
    wrote = write_control_override(200, 210, path=str(ov))
    assert wrote is False
    assert not ov.exists()


def test_write_explicitly_disabled_creates_nothing(monkeypatch, tmp_path):
    monkeypatch.setenv(SPP_CONTROL_ENV_VAR, "0")
    ov = tmp_path / "wall_e_bt_latest.json"
    assert write_control_override(200, 210, path=str(ov)) is False
    assert not ov.exists()


def test_disabled_write_does_not_clobber_existing_file(monkeypatch, tmp_path):
    """A gated SPP write must not even overwrite a legitimate writer's file.

    Simulates the session-gated web-teleop writer (``pi_app/web/teleop.py``)
    having just written a NEUTRAL command; a disabled SPP write must leave it
    byte-for-byte intact so it cannot corrupt or fight the audited path.
    """
    ov = tmp_path / "wall_e_bt_latest.json"
    teleop_payload = {"left_byte": 128, "right_byte": 128, "last_update_epoch_s": 123.0}
    ov.write_text(json.dumps(teleop_payload), encoding="utf-8")

    assert write_control_override(255, 255, path=str(ov)) is False
    assert json.loads(ov.read_text()) == teleop_payload


def test_disabled_write_never_becomes_a_bt_override(tmp_path):
    """End-to-end: default-gated SPP data yields no main.py bt_override.

    This is the retirement claim in main.py's own terms — with the gate OFF,
    there is no fresh override file, so ``controller.process`` receives
    ``bt_override=None`` and the SPP command cannot move the tracks.
    """
    ov = tmp_path / "wall_e_bt_latest.json"
    assert write_control_override(0, 255, path=str(ov)) is False  # full-throttle SPP cmd
    assert _read_bt_override(str(ov)) is None


# --------------------------------------------------------------------------
# Opt-in path still works (supervised bench testing)
# --------------------------------------------------------------------------

def test_write_enabled_produces_consumable_override(monkeypatch, tmp_path):
    monkeypatch.setenv(SPP_CONTROL_ENV_VAR, "1")
    ov = tmp_path / "wall_e_bt_latest.json"

    wrote = write_control_override(120, 140, path=str(ov))
    assert wrote is True

    data = json.loads(ov.read_text())
    assert data["left_byte"] == 120
    assert data["right_byte"] == 140
    assert isinstance(data["last_update_epoch_s"], (int, float))

    # And a fresh enabled write IS seen by the main.py-style reader.
    assert _read_bt_override(str(ov)) == (120, 140)


def test_write_enabled_uses_injected_clock(monkeypatch, tmp_path):
    """The ``now`` seam keeps the write deterministic for freshness tests."""
    monkeypatch.setenv(SPP_CONTROL_ENV_VAR, "1")
    ov = tmp_path / "wall_e_bt_latest.json"

    write_control_override(120, 140, path=str(ov), now=lambda: 1000.0)
    data = json.loads(ov.read_text())
    assert data["last_update_epoch_s"] == 1000.0
    # Stale timestamp => main.py would drop it (age > 0.6 s).
    assert _read_bt_override(str(ov), now=1000.7) is None


# --------------------------------------------------------------------------
# Module contract
# --------------------------------------------------------------------------

def test_module_importable_without_pybluez():
    """Deferred bluetooth import: the gate is reachable with no BT stack."""
    assert not hasattr(spp_server, "bluetooth")
    assert callable(spp_server.write_control_override)
    assert callable(spp_server.control_output_enabled)
