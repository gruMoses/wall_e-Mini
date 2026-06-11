"""Tests for the runtime Follow-Me param override endpoint.

GET /api/follow_me/params  → current five params
POST /api/follow_me/params → apply a bounds-checked subset (all-or-nothing)

Exercised end-to-end against a real FollowMeController wrapped in a minimal
fake controller, through the real Flask app from oak_viewer.create_app.
"""

import pytest

# Flask is a declared dependency (requirements.txt: flask>=3.0).
pytest.importorskip("flask")

from config import FollowMeConfig, OakWebViewerConfig
from pi_app.control.follow_me import FollowMeController, TUNABLE_PARAM_BOUNDS
from pi_app.web.oak_viewer import create_app


class _FakeController:
    """Wraps a real FollowMeController the way the live Controller does."""
    def __init__(self, follow_me):
        self._follow_me = follow_me


@pytest.fixture
def client_and_fm():
    fm = FollowMeController(FollowMeConfig())
    app = create_app(None, OakWebViewerConfig(), controller=_FakeController(fm))
    return app.test_client(), fm


def test_get_returns_exactly_five_params(client_and_fm):
    client, _ = client_and_fm
    resp = client.get("/api/follow_me/params")
    assert resp.status_code == 200
    body = resp.get_json()
    assert set(body.keys()) == set(TUNABLE_PARAM_BOUNDS.keys())
    # Shipped defaults.
    assert body["pid_lateral_kp"] == pytest.approx(0.4)
    assert body["steer_slew_per_tick"] == pytest.approx(0.1)


def test_post_in_bounds_applies_and_echoes(client_and_fm):
    client, fm = client_and_fm
    resp = client.post("/api/follow_me/params",
                       json={"pid_lateral_kp": 0.55, "steer_deadband_norm": 0.07})
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["ok"] is True
    assert body["applied"] == {"pid_lateral_kp": 0.55, "steer_deadband_norm": 0.07}
    # Applied to the live control objects.
    assert fm._steering._pid.kp == pytest.approx(0.55)
    assert fm._tunable("steer_deadband_norm") == pytest.approx(0.07)


def test_get_reflects_post(client_and_fm):
    client, _ = client_and_fm
    client.post("/api/follow_me/params", json={"target_ema_alpha": 0.8})
    body = client.get("/api/follow_me/params").get_json()
    assert body["target_ema_alpha"] == pytest.approx(0.8)


def test_out_of_bounds_rejected_400_no_partial_application(client_and_fm):
    client, fm = client_and_fm
    before = client.get("/api/follow_me/params").get_json()
    # kp valid, kd out of [0.0, 0.5] — whole request must be rejected.
    resp = client.post("/api/follow_me/params",
                       json={"pid_lateral_kp": 0.6, "pid_lateral_kd": 99.0})
    assert resp.status_code == 400
    assert "out of bounds" in resp.get_json()["error"]
    # Nothing applied — kp unchanged despite being valid in the request.
    after = client.get("/api/follow_me/params").get_json()
    assert after == before
    assert fm._steering._pid.kp == pytest.approx(before["pid_lateral_kp"])


def test_unknown_key_rejected_400(client_and_fm):
    client, _ = client_and_fm
    before = client.get("/api/follow_me/params").get_json()
    resp = client.post("/api/follow_me/params",
                       json={"pid_lateral_kp": 0.5, "speed_kp": 0.8})
    assert resp.status_code == 400
    assert "unknown param" in resp.get_json()["error"]
    # No partial application.
    assert client.get("/api/follow_me/params").get_json() == before


def test_boundary_values_accepted(client_and_fm):
    client, _ = client_and_fm
    lo = {k: b[0] for k, b in TUNABLE_PARAM_BOUNDS.items()}
    hi = {k: b[1] for k, b in TUNABLE_PARAM_BOUNDS.items()}
    assert client.post("/api/follow_me/params", json=lo).status_code == 200
    assert client.post("/api/follow_me/params", json=hi).status_code == 200


def test_empty_body_rejected_400(client_and_fm):
    client, _ = client_and_fm
    assert client.post("/api/follow_me/params", json={}).status_code == 400


def test_nan_rejected_400(client_and_fm):
    client, _ = client_and_fm
    # JSON has no NaN literal; send via the raw string the spec's clients can't,
    # so use a string that float() would parse oddly — non-numeric rejected.
    resp = client.post("/api/follow_me/params", json={"pid_lateral_kp": "abc"})
    assert resp.status_code == 400


def test_503_when_no_controller():
    app = create_app(None, OakWebViewerConfig(), controller=None)
    client = app.test_client()
    assert client.get("/api/follow_me/params").status_code == 503
    assert client.post("/api/follow_me/params", json={"pid_lateral_kp": 0.4}).status_code == 503
