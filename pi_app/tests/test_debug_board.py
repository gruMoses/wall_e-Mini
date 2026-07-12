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


def test_read_ups_status_missing_file(tmp_path):
    st = read_ups_status(str(tmp_path / "nope.json"))
    assert st["available"] is False
    assert st["stale"] is True
    assert st["reason"] == "missing"


def test_read_ups_status_corrupt_json(tmp_path):
    p = tmp_path / "ups_status.json"
    p.write_text("{ this is not valid json ")
    st = read_ups_status(str(p))
    assert st["available"] is False
    assert st["reason"] == "corrupt"


def test_read_ups_status_non_dict_payload(tmp_path):
    p = tmp_path / "ups_status.json"
    p.write_text("[1, 2, 3]")  # valid JSON, wrong shape
    st = read_ups_status(str(p))
    assert st["available"] is False
    assert st["reason"] == "corrupt"


def test_read_ups_status_fresh(tmp_path):
    p = tmp_path / "ups_status.json"
    now = 1_000_000.0
    p.write_text(json.dumps({
        "ts": now - 1.0, "ac_present": True, "typec_mv": 5000, "microusb_mv": 0,
        "batt_v": 3.9, "batt_ma": 1200.0, "protect_mv": 3400,
        "detect_only": False, "seconds_without_charge": 0,
    }))
    st = read_ups_status(str(p), max_age_s=10.0, now=now)
    assert st["available"] is True
    assert st["stale"] is False
    assert st["ac_present"] is True
    assert st["typec_mv"] == 5000
    assert st["batt_v"] == 3.9
    assert st["protect_mv"] == 3400
    assert st["age_s"] == pytest.approx(1.0, abs=0.01)


def test_read_ups_status_stale(tmp_path):
    p = tmp_path / "ups_status.json"
    now = 1_000_000.0
    p.write_text(json.dumps({"ts": now - 30.0, "ac_present": False}))
    st = read_ups_status(str(p), max_age_s=10.0, now=now)
    # Old-but-readable: still "available" so last-known values can be shown
    # greyed out, but flagged stale.
    assert st["available"] is True
    assert st["stale"] is True
    assert st["age_s"] == pytest.approx(30.0, abs=0.01)


def test_read_ups_status_missing_ts_is_stale(tmp_path):
    p = tmp_path / "ups_status.json"
    p.write_text(json.dumps({"ac_present": True}))  # no ts
    st = read_ups_status(str(p), now=1_000_000.0)
    assert st["available"] is True
    assert st["stale"] is True
    assert st["age_s"] is None


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
