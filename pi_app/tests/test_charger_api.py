"""Tests for the charger plug web API and page markers (pi_app/web/oak_viewer.py)."""

import json

import pytest

pytest.importorskip("flask")

from config import OakWebViewerConfig
from pi_app.web.oak_viewer import create_app


class FakeManager:
    def __init__(self):
        self.mode = "hold"
        self.calls = []

    def get_status(self):
        return {"mode": self.mode, "plug_on": False, "want_on": False, "reason": "hold: 100 % at or over 55 %",
                "soc_pct": 100.0, "pack_v": 54.5, "pack_a": 0.1, "bms_fresh": True, "plug_error": None,
                "last_switch_epoch": None, "switch_count": 0, "updated_epoch": 1.0, "host": "192.0.2.9"}

    def set_mode(self, mode):
        self.calls.append(mode)
        if mode in ("hold", "full", "on", "off"):
            self.mode = mode
            return True, f"mode {mode}"
        return False, "mode must be one of hold, full, on, off"


@pytest.fixture
def with_manager():
    mgr = FakeManager()
    app = create_app(None, OakWebViewerConfig(), charge_manager=mgr)
    return app.test_client(), mgr


@pytest.fixture
def without_manager():
    return create_app(None, OakWebViewerConfig()).test_client()


def test_status_with_manager(with_manager):
    client, _ = with_manager
    resp = client.get("/api/charger")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["available"] is True
    assert body["mode"] == "hold"
    assert body["soc_pct"] == 100.0


def test_status_without_manager(without_manager):
    body = without_manager.get("/api/charger").get_json()
    assert body["available"] is False


def test_set_mode(with_manager):
    client, mgr = with_manager
    resp = client.post("/api/charger/mode", data=json.dumps({"mode": "full"}),
                       content_type="application/json")
    assert resp.status_code == 200
    assert resp.get_json()["ok"] is True
    assert mgr.mode == "full"


def test_set_mode_rejects_bad_payloads(with_manager):
    client, mgr = with_manager
    assert client.post("/api/charger/mode", data="garbage", content_type="text/plain").status_code == 400
    assert client.post("/api/charger/mode", data=json.dumps({"mode": "warp"}),
                       content_type="application/json").status_code == 400
    assert mgr.mode == "hold"


def test_set_mode_without_manager_is_503(without_manager):
    resp = without_manager.post("/api/charger/mode", data=json.dumps({"mode": "full"}),
                                content_type="application/json")
    assert resp.status_code == 503


def test_buttons_on_the_main_page_only(without_manager):
    main = without_manager.get("/").get_data(as_text=True)
    debug = without_manager.get("/debug").get_data(as_text=True)
    assert "/api/charger/mode" in main
    assert 'id="c-btn-full"' in main
    assert "Charger Plug" in debug
    assert "/api/charger/mode" not in debug
    assert 'id="c-btn-full"' not in debug
