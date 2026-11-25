"""Installed-app shell: /drive nav row + battery percent, one manifest scope."""

import json

import pytest

pytest.importorskip("flask")

from config import OakWebViewerConfig
from pi_app.web.oak_viewer import create_app

MANIFEST_LINK = '<link rel="manifest" href="/drive/manifest.json">'


def _drive_app(tmp_path):
    from flask import Flask
    from pi_app.web.teleop import TeleopSession, register_teleop
    app = Flask(__name__)
    s = TeleopSession(command_sink=lambda l, r: None, require_rc_arm=False)
    register_teleop(app, s, token="T0K", token_path=str(tmp_path / "tok"))
    return app


def test_drive_page_has_nav_links_and_battery_poll(tmp_path):
    body = _drive_app(tmp_path).test_client().get("/drive?token=T0K").get_data(as_text=True)
    for href in ('href="/"', 'href="/navigate"', 'href="/map"'):
        assert href in body, href
    assert 'id="c-batt"' in body
    assert "/api/charger" in body and "soc_by_voltage" in body and "soc_pct" in body


def test_manifest_scope_is_site_root(tmp_path):
    m = json.loads(_drive_app(tmp_path).test_client().get("/drive/manifest.json").get_data(as_text=True))
    assert m["scope"] == "/"
    assert m["start_url"] == "/drive"


@pytest.mark.parametrize("path", ["/", "/navigate", "/map"])
def test_other_pages_carry_manifest_and_links(path):
    from pi_app.web.property_map import create_map_blueprint
    from pi_app.web.waypoint_nav_ui import create_nav_blueprint
    app = create_app(None, OakWebViewerConfig())
    app.register_blueprint(create_map_blueprint())
    app.register_blueprint(create_nav_blueprint())
    c = app.test_client()
    resp = c.get(path)
    assert resp.status_code == 200
    body = resp.get_data(as_text=True)
    assert MANIFEST_LINK in body
    assert 'name="theme-color"' in body
    assert 'href="/drive"' in body
    if path != "/":
        assert 'href="/"' in body
