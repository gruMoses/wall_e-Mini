"""Tests for the streaming connection caps on the OAK web viewer.

The Pi 5 serves MJPEG video (/stream/rgb, /stream/depth) and an SSE telemetry
stream (/api/telemetry), each of which pins a Flask worker thread. Runaway
browser tabs can therefore starve the control loop, so create_app() enforces:

  * a shared MJPEG pool (MAX_MJPEG_CLIENTS) across rgb + depth,
  * an SSE pool (MAX_SSE_CLIENTS) for /api/telemetry,

turning excess clients away with 503 + Retry-After. The counter must be
leak-proof: a slot is reserved before streaming and released in a finally tied
to the generator close / client disconnect, so a client that dies mid-stream
frees its slot. Plain request/response routes (e.g. /api/ups) are NOT capped so
the /debug board keeps working even when the SSE budget is exhausted.

Everything runs against the real Flask app from oak_viewer.create_app; no
hardware required. Caps are monkeypatched small (2) so the tests stay fast and
readable, plus one test exercises the shipped defaults.
"""

import json
import time

import pytest

pytest.importorskip("flask")  # declared dep (requirements.txt: flask>=3.0)

import pi_app.web.oak_viewer as ov
from pi_app.web.oak_viewer import create_app
from config import OakWebViewerConfig
from pi_app.hardware.oak_recorder import RecordingTelemetry


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------

class _FakeRecorder:
    """Recorder stand-in that streams: JPEGs are non-empty (so the MJPEG
    generator yields real frames) and telemetry is a real snapshot (so the SSE
    generator yields immediately — a non-yielding generator would block the
    test client while it waits for the first chunk)."""

    recording_state = "IDLE"

    def __init__(self):
        self._t = RecordingTelemetry(
            timestamp=time.time(), mode="MANUAL", throttle_scale=1.0,
            obstacle_distance_m=2.0, motor_left=128, motor_right=128,
            is_armed=True, depth_stats=None, person_detections=[],
        )

    def get_latest_annotated_jpeg(self):
        return b"\xff\xd8RGBFRAME\xff\xd9"

    def get_latest_depth_jpeg(self):
        return b"\xff\xd8DEPTHFRAME\xff\xd9"

    def get_latest_telemetry(self):
        return self._t

    def set_stream_client_connected(self, name, connected):
        pass


@pytest.fixture
def small_caps(monkeypatch):
    """Shrink both pools to 2 for fast, obvious cap tests. Read at build time
    by create_app, so set BEFORE building the app."""
    monkeypatch.setattr(ov, "MAX_MJPEG_CLIENTS", 2)
    monkeypatch.setattr(ov, "MAX_SSE_CLIENTS", 2)


def _make_client():
    app = create_app(_FakeRecorder(), OakWebViewerConfig())
    return app.test_client()


def _sse_payload(resp):
    """Pull the first `data: {...}` frame off an open SSE response."""
    chunk = next(resp.response)
    txt = chunk.decode() if isinstance(chunk, bytes) else chunk
    assert txt.startswith("data: ")
    return json.loads(txt[len("data: "):])


# ---------------------------------------------------------------------------
# MJPEG cap
# ---------------------------------------------------------------------------

def test_mjpeg_cap_returns_503_with_retry_after(small_caps):
    client = _make_client()
    open_resps = [client.get("/stream/rgb") for _ in range(ov.MAX_MJPEG_CLIENTS)]
    try:
        assert all(r.status_code == 200 for r in open_resps)
        blocked = client.get("/stream/rgb")
        assert blocked.status_code == 503
        assert blocked.headers.get("Retry-After") == str(ov.STREAM_CAP_RETRY_AFTER_S)
        body = json.loads(blocked.get_data(as_text=True))
        assert body["kind"] == "mjpeg"
        assert body["cap"] == ov.MAX_MJPEG_CLIENTS
        assert "cap reached" in body["error"]
    finally:
        for r in open_resps:
            r.close()


def test_mjpeg_pool_is_shared_across_rgb_and_depth(small_caps):
    """One shared budget: rgb + depth together cannot exceed the cap."""
    client = _make_client()
    r_rgb = client.get("/stream/rgb")
    r_depth = client.get("/stream/depth")  # fills the 2-slot pool with rgb+depth
    try:
        assert r_rgb.status_code == 200
        assert r_depth.status_code == 200
        # Both endpoints now see the pool as full.
        assert client.get("/stream/rgb").status_code == 503
        assert client.get("/stream/depth").status_code == 503
    finally:
        r_rgb.close()
        r_depth.close()


def test_mjpeg_slot_freed_after_response_close(small_caps):
    client = _make_client()
    held = [client.get("/stream/rgb") for _ in range(ov.MAX_MJPEG_CLIENTS)]
    try:
        assert client.get("/stream/rgb").status_code == 503  # at cap
        held[0].close()                                       # free one slot
        reopened = client.get("/stream/rgb")
        assert reopened.status_code == 200
        reopened.close()
    finally:
        for r in held[1:]:
            r.close()


def test_mjpeg_leak_proof_client_dies_midstream(small_caps):
    """A client that starts streaming then dies mid-stream must free its slot
    via the generator's finally — not leak it."""
    client = _make_client()
    victim = client.get("/stream/rgb")
    other = client.get("/stream/depth")
    try:
        # Enter the generator body so its try/finally is actually armed.
        first = next(victim.response)
        assert first.startswith(b"--frame")
        assert client.get("/stream/rgb").status_code == 503  # pool full (2/2)
        victim.close()  # simulate mid-stream disconnect -> GeneratorExit
        # Slot must be reclaimed.
        after = client.get("/stream/rgb")
        assert after.status_code == 200
        after.close()
    finally:
        other.close()


def test_recorder_none_stream_does_not_consume_a_slot(small_caps):
    """With no recorder, /stream/rgb returns a single placeholder image (not a
    live stream) and must never 503 or hold a slot."""
    app = create_app(None, OakWebViewerConfig())
    client = app.test_client()
    for _ in range(ov.MAX_MJPEG_CLIENTS + 3):
        r = client.get("/stream/rgb")
        assert r.status_code == 200
        assert r.mimetype == "image/jpeg"  # single frame, not multipart stream


# ---------------------------------------------------------------------------
# SSE cap
# ---------------------------------------------------------------------------

def test_sse_cap_returns_503_with_retry_after(small_caps):
    client = _make_client()
    held = [client.get("/api/telemetry") for _ in range(ov.MAX_SSE_CLIENTS)]
    try:
        assert all(r.status_code == 200 for r in held)
        blocked = client.get("/api/telemetry")
        assert blocked.status_code == 503
        assert blocked.headers.get("Retry-After") == str(ov.STREAM_CAP_RETRY_AFTER_S)
        body = json.loads(blocked.get_data(as_text=True))
        assert body["kind"] == "sse"
        assert body["cap"] == ov.MAX_SSE_CLIENTS
    finally:
        for r in held:
            r.close()


def test_sse_slot_freed_after_response_close(small_caps):
    client = _make_client()
    held = [client.get("/api/telemetry") for _ in range(ov.MAX_SSE_CLIENTS)]
    try:
        assert client.get("/api/telemetry").status_code == 503
        held[0].close()
        reopened = client.get("/api/telemetry")
        assert reopened.status_code == 200
        reopened.close()
    finally:
        for r in held[1:]:
            r.close()


def test_ups_route_not_capped_when_sse_exhausted(small_caps):
    """The /debug board polls /api/ups; that request/response route must keep
    working even when every SSE slot is taken by background dashboards."""
    client = _make_client()
    held = [client.get("/api/telemetry") for _ in range(ov.MAX_SSE_CLIENTS)]
    try:
        assert client.get("/api/telemetry").status_code == 503  # SSE exhausted
        ups = client.get("/api/ups")                            # still fine
        assert ups.status_code == 200
        assert ups.mimetype == "application/json"
    finally:
        for r in held:
            r.close()


# ---------------------------------------------------------------------------
# Normal operation + observability
# ---------------------------------------------------------------------------

def test_full_dashboard_under_default_caps():
    """A single dashboard tab opens 2 MJPEG streams + 1 SSE. Under the shipped
    defaults that must all succeed concurrently."""
    client = _make_client()  # default caps (3 MJPEG, 6 SSE)
    rgb = client.get("/stream/rgb")
    depth = client.get("/stream/depth")
    sse = client.get("/api/telemetry")
    try:
        assert rgb.status_code == 200
        assert depth.status_code == 200
        assert sse.status_code == 200
        payload = _sse_payload(sse)
        sc = payload["stream_clients"]
        assert sc["mjpeg"] == 2 and sc["mjpeg_max"] == ov.MAX_MJPEG_CLIENTS
        assert sc["sse"] == 1 and sc["sse_max"] == ov.MAX_SSE_CLIENTS
    finally:
        rgb.close()
        depth.close()
        sse.close()


def test_stream_clients_gauge_tracks_open_streams(small_caps):
    client = _make_client()
    rgb = client.get("/stream/rgb")
    sse = client.get("/api/telemetry")
    try:
        payload = _sse_payload(sse)
        sc = payload["stream_clients"]
        # rgb open -> mjpeg 1; this sse open -> sse 1.
        assert sc["mjpeg"] == 1
        assert sc["sse"] == 1
        assert sc["mjpeg_max"] == 2
        assert sc["sse_max"] == 2
    finally:
        rgb.close()
        sse.close()
