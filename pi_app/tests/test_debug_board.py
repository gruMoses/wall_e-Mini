"""Tests for the read-only /debug status board + its UPS bridge (read side).

Covers:
  * GET /debug            → 200 + the page markers the owner watches.
  * GET /api/ups          → 200 even with a missing/absent status file.
  * read_ups_status()     → graceful handling of missing / stale / corrupt /
                            non-dict files, and correct fresh-vs-stale flagging.
  * /api/telemetry SSE    → the newly-plumbed VESC per-motor status ages,
                            rx_thread_alive, and emergency_active are present
                            in the JSON payload.

Everything runs against the real Flask app from oak_viewer.create_app; no
hardware and no running daemon required.
"""

import json
import time

import pytest

pytest.importorskip("flask")  # declared dep (requirements.txt: flask>=3.0)

from config import OakWebViewerConfig
from pi_app.hardware.oak_recorder import RecordingTelemetry
from pi_app.web.oak_viewer import create_app, read_ups_status

# The old-file (walle-ups) cases pass this as the new file's path, so a real
# /run/walle-power/status.json on a Pi cannot shadow their file.
_NO_POWER = "/nonexistent/walle-power/status.json"


# ---------------------------------------------------------------------------
# /debug route
# ---------------------------------------------------------------------------

@pytest.fixture
def client():
    app = create_app(None, OakWebViewerConfig())
    return app.test_client()


def test_debug_route_serves_200_with_key_markers(client):
    resp = client.get("/debug")
    assert resp.status_code == 200
    assert resp.headers["Content-Type"].startswith("text/html")
    body = resp.get_data(as_text=True)
    # Title + each panel the owner relies on to watch a physical switch flip.
    assert "Debug Status Board" in body
    for marker in ("Safety", "VESC / CAN", "BMS", "UPS", "Event Log",
                   "Temperature History"):
        assert marker in body, marker
    # It subscribes to the existing SSE + polls the UPS bridge.
    assert "/api/telemetry" in body
    assert "/api/ups" in body


def test_debug_page_is_read_only_no_control_posts(client):
    """The board must never command the robot: no fetch/POST to a mutating
    endpoint appears in the page source (only the read-only GETs above)."""
    body = client.get("/debug").get_data(as_text=True)
    assert "method: 'POST'" not in body
    assert "/api/follow_me" not in body
    assert "/api/teleop" not in body


def test_debug_page_temperature_history_markers(client):
    """Temperature panel: canvas graph, named series, bounded buffer, no CDN."""
    body = client.get("/debug").get_data(as_text=True)
    assert 'id="temp-canvas"' in body
    assert "pushTempSample" in body
    assert "TEMP_MAX_POINTS" in body
    assert "TEMP_WINDOW_MS" in body
    # Named series keys match SSE field names.
    for key in (
        "bms_temp_max_c",
        "bms_temp_min_c",
        "vesc_left_temp_c",
        "vesc_right_temp_c",
        "vesc_left_motor_temp_c",
        "vesc_right_motor_temp_c",
        "pi_cpu_temp_c",
    ):
        assert key in body, key
    # No chart libraries / CDNs — self-contained canvas draw.
    assert "cdn." not in body.lower()
    assert "chart.js" not in body.lower()
    assert "googleapis" not in body.lower()
    # Offline-safe: no external <script src=…> (all logic is inline).
    assert "<script src=" not in body.lower()


def test_debug_page_temperature_structural_invariants(client):
    """Retention bounds, gap semantics, a11y hooks, and stale alignment with
    the controller's 0.5 s STATUS-age cutoff must stay in the page source."""
    body = client.get("/debug").get_data(as_text=True)
    # Bounded 10-minute ring buffer hard caps.
    assert "TEMP_WINDOW_MS = 10 * 60 * 1000" in body
    assert "TEMP_MAX_POINTS = 720" in body
    assert "TEMP_MIN_SAMPLE_MS = 900" in body
    # SSE reconnect / tab freeze must not draw a diagonal across a time hole.
    assert "TEMP_GAP_MS = 5000" in body
    assert "(tv - prevT) > TEMP_GAP_MS" in body
    # VESC stale matches controller open-loop fallback (>0.5 s any motor).
    assert "TEMP_VESC_STALE_S = 0.5" in body
    assert "function vescTempsStale" in body
    # Null / disconnected sensors leave gaps (force raw = null).
    assert "bmsOffline" in body
    assert "vescStale" in body
    # Accessibility: toggle state + live summary for screen readers.
    assert 'aria-pressed="' in body
    assert 'aria-live="polite"' in body
    assert 'id="temp-sr"' in body
    assert 'role="group"' in body
    # High-DPI canvas capped at 2× for Pi cost.
    assert "devicePixelRatio" in body
    assert "Math.min(window.devicePixelRatio || 1, 2)" in body
    # Legend updates in place (no full rebuild per sample → no focus steal).
    assert "function updateTempLegend" in body
    assert "legendBuilt" in body


def test_root_dashboard_links_to_debug(client):
    body = client.get("/").get_data(as_text=True)
    assert 'href="/debug"' in body


# ---------------------------------------------------------------------------
# /api/ups route + read_ups_status()
# ---------------------------------------------------------------------------

def test_api_ups_returns_200_when_file_missing(client):
    # Default path (/tmp/ups_status.json) may or may not exist; the route must
    # still return 200 with a JSON object (never 500) so the page can render.
    resp = client.get("/api/ups")
    assert resp.status_code == 200
    body = resp.get_json()
    assert isinstance(body, dict)
    assert "available" in body and "stale" in body


def test_debug_page_ups_panel_matches_walle_ups_bridge(client):
    """The UPS panel carries one cell per field of the walle-ups/1 file AND of
    the walle-power/1 file; the cells of the retired HAT are gone, and every
    u-* id that applyUps() writes to exists in the page. The hint names
    walle-power.service when the new file is in use (source "walle-power")
    and keeps the old names (bin/walle_ups.py, walle-ups.service) otherwise."""
    import re

    body = client.get("/debug").get_data(as_text=True)
    for cell_id in ("u-mains", "u-battv", "u-battma", "u-supply", "u-loss",
                    "u-mode", "u-state", "u-errors", "u-hint",
                    "u-rail", "u-charge", "u-halt", "u-faults"):
        assert f'id="{cell_id}"' in body, cell_id
    for gone in ("u-ac", "u-typec", "u-microusb", "u-protect", "u-swc"):
        assert f'id="{gone}"' not in body, gone
    js = body[body.index("function applyUps"):body.index("function pollUps")]
    # The service named in the hint follows the source of the status.
    assert "u.source === 'walle-power'" in js
    assert "'walle-power.service (bin/walle_power.py)'" in js
    assert "'walle-ups.service (bin/walle_ups.py)'" in js
    assert "Is ' + svc + ' running?" in js
    assert "walle-ups.service running" not in js  # no fixed old name left
    # One banner per fault.
    assert "upsFaults(" in js and "ups-fault" in body
    used = set(re.findall(r"'(u-[a-z]+)'", js))
    assert used, "applyUps() references no u-* ids"
    for ident in used:
        assert f'id="{ident}"' in body, ident


def test_read_ups_status_missing_file(tmp_path):
    st = read_ups_status(str(tmp_path / "nope.json"), power_path=_NO_POWER)
    assert st["available"] is False
    assert st["stale"] is True
    assert st["reason"] == "missing"


def test_read_ups_status_corrupt_json(tmp_path):
    p = tmp_path / "ups_status.json"
    p.write_text("{ this is not valid json ")
    st = read_ups_status(str(p), power_path=_NO_POWER)
    assert st["available"] is False
    assert st["reason"] == "corrupt"


def test_read_ups_status_non_dict_payload(tmp_path):
    p = tmp_path / "ups_status.json"
    p.write_text("[1, 2, 3]")  # valid JSON, wrong shape
    st = read_ups_status(str(p), power_path=_NO_POWER)
    assert st["available"] is False
    assert st["reason"] == "corrupt"


def test_read_ups_status_fresh(tmp_path):
    # Shape of the bin/walle_ups.py status file (schema walle-ups/1).
    p = tmp_path / "ups_status.json"
    now = 1_000_000.0
    p.write_text(json.dumps({
        "schema": "walle-ups/1", "ts": now - 1.0, "ts_loop": now - 0.5,
        "input": "present", "batt_v": 8.35, "batt_ma": 12.0, "soc_pct": 55,
        "overflow": False, "supply_v": 8.4, "absent_s": 0.0, "loss_s": 5.0,
        "state": "monitoring", "mode": "armed", "halt_enabled": True,
        "errors": 0, "errors_run": 0, "last_error": None, "last_loss": None,
        "first_alarm_s": 90,
    }))
    st = read_ups_status(str(p), power_path=_NO_POWER, max_age_s=10.0, now=now)
    assert st["available"] is True
    assert st["stale"] is False
    assert st["input"] == "present"
    assert st["batt_v"] == 8.35
    assert st["batt_ma"] == 12.0
    assert st["soc_pct"] == 55
    assert st["overflow"] is False
    assert st["supply_v"] == 8.4
    assert st["absent_s"] == 0.0
    assert st["loss_s"] == 5.0
    assert st["state"] == "monitoring"
    assert st["mode"] == "armed"
    assert st["halt_enabled"] is True
    assert st["errors"] == 0
    assert st["last_error"] is None
    assert st["last_loss"] is None
    assert st["schema"] == "walle-ups/1"
    assert st["ts_loop"] == now - 0.5
    assert st["age_s"] == pytest.approx(1.0, abs=0.01)
    # The keys of the retired HAT's daemon must not come back.
    for gone in ("typec_mv", "ac_present", "microusb_mv", "protect_mv",
                 "ts_batt", "batt_stale", "detect_only", "seconds_without_charge"):
        assert gone not in st, gone


def test_read_ups_status_stale(tmp_path):
    p = tmp_path / "ups_status.json"
    now = 1_000_000.0
    p.write_text(json.dumps({"ts": now - 30.0, "input": "absent"}))
    st = read_ups_status(str(p), power_path=_NO_POWER, max_age_s=10.0, now=now)
    # Old-but-readable: still "available" so last-known values can be shown
    # greyed out, but flagged stale.
    assert st["available"] is True
    assert st["stale"] is True
    assert st["input"] == "absent"
    assert st["age_s"] == pytest.approx(30.0, abs=0.01)


def test_read_ups_status_missing_ts_is_stale(tmp_path):
    p = tmp_path / "ups_status.json"
    p.write_text(json.dumps({"input": "present"}))  # no ts
    st = read_ups_status(str(p), power_path=_NO_POWER, now=1_000_000.0)
    assert st["available"] is True
    assert st["stale"] is True
    assert st["age_s"] is None


def test_read_ups_status_blind_watcher_is_stale(tmp_path):
    """walle-ups publishes ``ts`` = last SUCCESSFUL INA219 read (None before
    the first one) and ``ts_loop`` = last poll. A watcher that is alive but
    cannot read the sensor must look stale, whatever ts_loop says."""
    p = tmp_path / "ups_status.json"
    now = 1_000_000.0
    p.write_text(json.dumps({
        "schema": "walle-ups/1", "ts": None, "ts_loop": now - 0.2,
        "input": None, "batt_v": None, "batt_ma": None, "soc_pct": None,
        "state": "monitoring", "mode": "armed",
        "errors": 7, "last_error": "OSError: [Errno 121] Remote I/O error",
    }))
    st = read_ups_status(str(p), power_path=_NO_POWER, max_age_s=10.0, now=now)
    assert st["available"] is True
    assert st["stale"] is True
    assert st["age_s"] is None
    assert st["ts_loop"] == now - 0.2
    assert st["input"] is None
    assert st["errors"] == 7
    assert st["last_error"].startswith("OSError")


# ---------------------------------------------------------------------------
# New watcher: /run/walle-power/status.json (schema walle-power/1)
# ---------------------------------------------------------------------------

_BOOT = "11111111-2222-3333-4444-555555555555"
_BOOT_NOW = 5000.0


def _power_file(tmp_path, **over):
    payload = {
        "schema": "walle-power/1", "boot_id": _BOOT,
        "hb_mono": _BOOT_NOW - 0.5, "bus_ok_mono": _BOOT_NOW - 0.8,
        "bus_ok_ts": 1_700_000_000.0, "input_good": True, "low_s": 0.0,
        "vcell_v": 3.912, "ext5v_v": 5.08, "charge": "disabled", "hold": "off",
        "state": "run", "halt_reason": None, "faults": [], "last_error": None,
    }
    payload.update(over)
    p = tmp_path / "power_status.json"
    p.write_text(json.dumps(payload))
    return str(p)


def _read_power(tmp_path, old_path=None, **over):
    return read_ups_status(
        old_path or str(tmp_path / "no_old.json"),
        power_path=_power_file(tmp_path, **over),
        now_boot=_BOOT_NOW, boot_id=_BOOT,
    )


def test_power_status_fresh(tmp_path):
    st = _read_power(tmp_path)
    assert st["available"] is True
    assert st["stale"] is False
    assert st["source"] == "walle-power"
    assert st["age_s"] == pytest.approx(0.5, abs=0.01)
    assert st["bus_ok_age_s"] == pytest.approx(0.8, abs=0.01)
    # Old keys the page reads, filled from the new fields.
    assert st["input"] == "present"
    assert st["batt_v"] == 3.912
    # New fields.
    assert st["input_good"] is True
    assert st["vcell_v"] == 3.912
    assert st["ext5v_v"] == 5.08
    assert st["charge"] == "disabled"
    assert st["hold"] == "off"
    assert st["state"] == "run"
    assert st["halt_reason"] is None
    assert st["faults"] == []
    assert st["low_s"] == 0.0


@pytest.mark.parametrize("good, expected", [
    (True, "present"), (False, "absent"), (None, "unknown"), ("yes", "unknown"),
])
def test_power_status_input_mapping(tmp_path, good, expected):
    assert _read_power(tmp_path, input_good=good)["input"] == expected


def test_power_status_stale_by_heartbeat(tmp_path):
    # The bus read is fresh, but the loop heartbeat is 11 s old: stale.
    st = _read_power(tmp_path, hb_mono=_BOOT_NOW - 11.0, bus_ok_mono=_BOOT_NOW - 1.0)
    assert st["available"] is True
    assert st["stale"] is True
    assert st["age_s"] == pytest.approx(11.0, abs=0.01)


def test_power_status_missing_heartbeat_is_stale(tmp_path):
    st = _read_power(tmp_path, hb_mono=None)
    assert st["stale"] is True
    assert st["age_s"] is None


def test_power_status_from_another_boot_is_stale(tmp_path):
    st = _read_power(tmp_path, boot_id="00000000-dead-beef-0000-000000000000")
    assert st["available"] is True
    assert st["stale"] is True
    assert st["boot_ok"] is False
    assert st["reason"] == "other_boot"
    assert st["age_s"] is None  # another boot's clock is never compared


def test_power_status_with_faults(tmp_path):
    st = _read_power(tmp_path, faults=["gauge", "nohalt"], state="halting",
                     halt_reason="input_loss", charge="failsafe")
    assert st["faults"] == ["gauge", "nohalt"]
    assert st["state"] == "halting"
    assert st["halt_reason"] == "input_loss"
    assert st["charge"] == "failsafe"


def test_power_status_corrupt_does_not_fall_back(tmp_path):
    old = tmp_path / "ups_status.json"
    old.write_text(json.dumps({"ts": time.time(), "input": "present"}))
    p = tmp_path / "power_status.json"
    p.write_text("{ nope")
    st = read_ups_status(str(old), power_path=str(p), now_boot=_BOOT_NOW, boot_id=_BOOT)
    assert st["available"] is False
    assert st["reason"] == "corrupt"
    assert st["source"] == "walle-power"


def test_power_status_missing_falls_back_to_old_file(tmp_path):
    old = tmp_path / "ups_status.json"
    now = 1_000_000.0
    old.write_text(json.dumps({"schema": "walle-ups/1", "ts": now - 2.0,
                               "input": "absent", "batt_v": 7.4}))
    st = read_ups_status(str(old), power_path=str(tmp_path / "no_power.json"),
                         max_age_s=10.0, now=now)
    assert st["available"] is True
    assert st["stale"] is False
    assert st["source"] == "walle-ups"
    assert st["input"] == "absent"
    assert st["batt_v"] == 7.4
    assert st["age_s"] == pytest.approx(2.0, abs=0.01)


def test_power_status_preferred_over_old_file(tmp_path):
    old = tmp_path / "ups_status.json"
    old.write_text(json.dumps({"ts": time.time(), "input": "absent"}))
    st = _read_power(tmp_path, old_path=str(old))
    assert st["source"] == "walle-power"
    assert st["input"] == "present"


def test_api_ups_serves_power_file(tmp_path, monkeypatch):
    import pi_app.web.oak_viewer as viewer

    path = _power_file(tmp_path, boot_id="not-this-boot")
    monkeypatch.setattr(viewer, "POWER_STATUS_FILE", path)
    app = create_app(None, OakWebViewerConfig())
    body = app.test_client().get("/api/ups").get_json()
    assert body["source"] == "walle-power"
    assert body["stale"] is True  # boot id does not match this machine


# ---------------------------------------------------------------------------
# SSE payload — newly-plumbed telemetry fields
# ---------------------------------------------------------------------------

class _OneShotRecorder:
    """Minimal recorder stand-in returning one RecordingTelemetry snapshot."""
    recording_state = "IDLE"

    def __init__(self, telem):
        self._t = telem

    def get_latest_telemetry(self):
        return self._t


def _first_sse_payload(recorder):
    app = create_app(recorder, OakWebViewerConfig())
    client = app.test_client()
    resp = client.get("/api/telemetry")
    try:
        chunk = next(resp.response)  # first `data: {...}\n\n` is yielded immediately
    finally:
        resp.close()
    text = chunk.decode() if isinstance(chunk, bytes) else chunk
    assert text.startswith("data: ")
    return json.loads(text[len("data: "):])


def test_sse_payload_carries_new_vesc_and_emergency_fields():
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=2.0, motor_left=128, motor_right=128, is_armed=True,
        depth_stats=None, person_detections=[],
        emergency_active=True,
        vesc_left_status_age_s=0.12, vesc_right_status_age_s=0.83,
        vesc_rx_thread_alive=True,
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    assert d["emergency_active"] is True
    assert d["vesc_left_status_age_s"] == pytest.approx(0.12)
    assert d["vesc_right_status_age_s"] == pytest.approx(0.83)
    assert d["vesc_rx_thread_alive"] is True


def test_sse_new_fields_present_even_when_none():
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=None, motor_left=128, motor_right=128, is_armed=False,
        depth_stats=None, person_detections=[],
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    # Keys must exist (defaulting to None) so the client can render "n/a"
    # rather than crash on undefined.
    for key in ("vesc_left_status_age_s", "vesc_right_status_age_s",
                "vesc_rx_thread_alive", "emergency_active"):
        assert key in d, key
    assert d["vesc_rx_thread_alive"] is None


def test_sse_payload_carries_bms_link_freshness():
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=2.0, motor_left=126, motor_right=126, is_armed=False,
        depth_stats=None, person_detections=[],
        bms_connected=False, bms_link_state="dark", bms_data_age_s=2591.27,
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    assert d["bms_link_state"] == "dark"
    assert d["bms_data_age_s"] == pytest.approx(2591.3)


def test_sse_bms_link_keys_present_when_none():
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=None, motor_left=126, motor_right=126, is_armed=False,
        depth_stats=None, person_detections=[],
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    assert "bms_link_state" in d and d["bms_link_state"] is None
    assert "bms_data_age_s" in d and d["bms_data_age_s"] is None


_TEMP_SSE_KEYS = (
    "bms_temp_max_c",
    "bms_temp_min_c",
    "vesc_left_temp_c",
    "vesc_right_temp_c",
    "vesc_left_motor_temp_c",
    "vesc_right_motor_temp_c",
    "pi_cpu_temp_c",
)


def test_sse_payload_carries_temperature_fields():
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=2.0, motor_left=128, motor_right=128, is_armed=True,
        depth_stats=None, person_detections=[],
        bms_temp_max_c=32.4, bms_temp_min_c=28.1,
        bms_connected=True,
        vesc_left_temp_c=41.5, vesc_right_temp_c=42.0,
        vesc_left_motor_temp_c=48.2, vesc_right_motor_temp_c=49.0,
        vesc_rx_thread_alive=True,
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    assert d["bms_temp_max_c"] == pytest.approx(32.4)
    assert d["bms_temp_min_c"] == pytest.approx(28.1)
    assert d["vesc_left_temp_c"] == pytest.approx(41.5)
    assert d["vesc_right_temp_c"] == pytest.approx(42.0)
    assert d["vesc_left_motor_temp_c"] == pytest.approx(48.2)
    assert d["vesc_right_motor_temp_c"] == pytest.approx(49.0)
    # Pi CPU is best-effort sysfs; key must always exist.
    assert "pi_cpu_temp_c" in d
    assert d["pi_cpu_temp_c"] is None or isinstance(d["pi_cpu_temp_c"], (int, float))


def test_sse_temperature_keys_present_when_none():
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=None, motor_left=128, motor_right=128, is_armed=False,
        depth_stats=None, person_detections=[],
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    for key in _TEMP_SSE_KEYS:
        assert key in d, key


def test_read_pi_cpu_temp_c_from_sysfs(tmp_path):
    from pi_app.web.oak_viewer import read_pi_cpu_temp_c
    import pi_app.web.oak_viewer as ov

    p = tmp_path / "temp"
    p.write_text("45678\n")  # 45.678 °C → rounded to 45.7
    # Bust cache so the test path is used.
    ov._pi_cpu_temp_cache = None
    val = read_pi_cpu_temp_c(now=time.monotonic(), path=str(p))
    assert val == pytest.approx(45.7)
    # Cache hit returns same without re-read dependency on path still existing.
    p.write_text("99999\n")
    val2 = read_pi_cpu_temp_c(now=time.monotonic(), path=str(p))
    assert val2 == pytest.approx(45.7)
    # After TTL, re-reads.
    ov._pi_cpu_temp_cache = None
    val3 = read_pi_cpu_temp_c(now=time.monotonic() + 10.0, path=str(p))
    assert val3 == pytest.approx(100.0)  # 99.999 → 100.0


def test_read_pi_cpu_temp_c_missing_file(tmp_path):
    from pi_app.web.oak_viewer import read_pi_cpu_temp_c
    import pi_app.web.oak_viewer as ov

    ov._pi_cpu_temp_cache = None
    val = read_pi_cpu_temp_c(now=time.monotonic(), path=str(tmp_path / "nope"))
    assert val is None


def test_read_pi_cpu_temp_c_out_of_range_rejected(tmp_path):
    """Garbage or implausible thermal_zone values must not reach the graph."""
    from pi_app.web.oak_viewer import read_pi_cpu_temp_c
    import pi_app.web.oak_viewer as ov

    p = tmp_path / "temp"
    # 200 °C is above the 125 °C sanity cap.
    p.write_text("200000\n")
    ov._pi_cpu_temp_cache = None
    assert read_pi_cpu_temp_c(now=time.monotonic(), path=str(p)) is None
    # Non-numeric
    p.write_text("not-a-number\n")
    ov._pi_cpu_temp_cache = None
    assert read_pi_cpu_temp_c(now=time.monotonic(), path=str(p)) is None


def test_sse_temperature_fields_reject_non_finite():
    """NaN/Inf in RecordingTelemetry must serialize as null, not JSON null-ish
    numbers that break the canvas finiteOrNull path."""
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=None, motor_left=128, motor_right=128, is_armed=False,
        depth_stats=None, person_detections=[],
        bms_temp_max_c=float("nan"),
        vesc_left_temp_c=float("inf"),
        vesc_right_motor_temp_c=float("-inf"),
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    assert d["bms_temp_max_c"] is None
    assert d["vesc_left_temp_c"] is None
    assert d["vesc_right_motor_temp_c"] is None


def test_sse_bms_temp_min_is_independent_of_max():
    """Min and max must both survive the SSE path (regression: only max was plumbed)."""
    t = RecordingTelemetry(
        timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
        obstacle_distance_m=2.0, motor_left=128, motor_right=128, is_armed=True,
        depth_stats=None, person_detections=[],
        bms_temp_max_c=40, bms_temp_min_c=22,  # ints like Daly BmsState
        bms_connected=True,
    )
    d = _first_sse_payload(_OneShotRecorder(t))
    assert d["bms_temp_max_c"] == pytest.approx(40.0)
    assert d["bms_temp_min_c"] == pytest.approx(22.0)


@pytest.mark.parametrize("text", [
    '{"ts": -1.0}', '{"ts": NaN}', '{"ts": Infinity}', '{"ts": true}',
    '{"ts": %s}' % ("9" * 400),
])
def test_old_file_bad_ts_is_stale_not_error(tmp_path, text):
    p = tmp_path / "ups_status.json"
    p.write_text(text)
    st = read_ups_status(str(p), power_path=_NO_POWER, now=1_000_000.0)
    assert st["available"] is True
    assert st["stale"] is True
    assert st["age_s"] is None


@pytest.mark.parametrize("field", ["hb_mono", "bus_ok_mono"])
@pytest.mark.parametrize("bad", ["-1.0", "NaN", "-Infinity", "true", "1" + "0" * 400])
def test_power_file_bad_numbers(tmp_path, field, bad):
    path = _power_file(tmp_path)
    text = open(path).read()
    data = json.loads(text)
    data[field] = "__BAD__"
    open(path, "w").write(json.dumps(data).replace('"__BAD__"', bad))
    st = read_ups_status(str(tmp_path / "no_old.json"), power_path=path,
                         now_boot=_BOOT_NOW, boot_id=_BOOT)
    assert st["available"] is True
    if field == "hb_mono":
        assert st["stale"] is True and st["age_s"] is None
    else:
        assert st["bus_ok_age_s"] is None


@pytest.mark.parametrize("which", ["new", "old"])
@pytest.mark.parametrize("content", ["[" * 100000 + "]" * 100000, "9" * 400, b"\xff\xfe"])
def test_status_readers_never_raise(tmp_path, which, content):
    p = tmp_path / "status.json"
    if isinstance(content, bytes):
        p.write_bytes(content)
    else:
        p.write_text(content)
    if which == "new":
        st = read_ups_status(str(tmp_path / "no_old.json"), power_path=str(p),
                             now_boot=_BOOT_NOW, boot_id=_BOOT)
    else:
        st = read_ups_status(str(p), power_path=_NO_POWER)
    assert st["available"] is False
    assert st["reason"] == "corrupt"
