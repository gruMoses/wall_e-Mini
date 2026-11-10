"""The waypoint API limits client speed and radius, and a refused start changes nothing.

Before 2026-11-10 /api/nav/start and /api/nav/go wrote ``cruise_speed`` and
``arrival_radius`` into the nav config with no limit (127 is full speed, a
negative value reverses, and the value stayed for later missions). They also
replaced the route before the activation gate was asked, so a request that
came back 409 had already retargeted a mission that was running.

The navigator in these tests is the real ``WaypointNavController``; only the
controller around it is a fake.
"""
from __future__ import annotations

import json

import pytest

pytest.importorskip("flask")
from flask import Flask  # noqa: E402

from pi_app.control.waypoint_nav import (  # noqa: E402
    NavState,
    Waypoint,
    WaypointNavConfig,
    WaypointNavController,
)
from pi_app.web import waypoint_nav_ui as ui  # noqa: E402


class _Ctl:
    """Fake controller around a real navigator.

    ``block`` is what the activation gate reports before anything is written.
    ``late_block`` / ``raise_on_activate`` model the gate closing (or a fault)
    between that check and the activation itself.
    """

    def __init__(self, block=None, late_block=None, raise_on_activate=False, nav=None):
        self._waypoint_nav = nav if nav is not None else WaypointNavController(WaypointNavConfig())
        self._gps_reading = None
        self.block = block
        self.late_block = late_block
        self.raise_on_activate = raise_on_activate
        self.activated = 0
        self.deactivated = 0

    def waypoint_nav_activation_blocked(self):
        return self.block

    def activate_waypoint_nav(self):
        self.activated += 1
        if self.raise_on_activate:
            raise RuntimeError("boom")
        return self.block or self.late_block

    def deactivate_waypoint_nav(self):
        self.deactivated += 1


def _live_mission():
    """A navigator that is under way to the second of two waypoints."""
    route = [Waypoint(lat=10.0, lon=20.0, name="A"), Waypoint(lat=10.1, lon=20.1, name="B")]
    nav = WaypointNavController(WaypointNavConfig(), route)
    nav._index = 1
    nav._state = NavState.DRIVE
    return nav


def _assert_live_mission_untouched(nav):
    assert [(w.lat, w.lon) for w in nav.waypoints] == [(10.0, 20.0), (10.1, 20.1)]
    assert nav.current_index == 1
    assert nav.state == NavState.DRIVE
    assert not nav.completed
    assert (nav._cfg.cruise_speed_byte, nav._cfg.arrival_radius_m) == (40, 1.0)


def _client(ctl):
    app = Flask(__name__)
    app.register_blueprint(ui.create_nav_blueprint(controller=ctl, estop_check=lambda: False))
    return app.test_client()


def _post(ctl, url, payload):
    # json.dumps allows NaN / Infinity, which is what a hostile client can send.
    return _client(ctl).post(url, data=json.dumps(payload), content_type="application/json")


def _cfg(ctl):
    c = ctl._waypoint_nav._cfg
    return c.cruise_speed_byte, c.arrival_radius_m


WP = {"waypoints": [{"lat": 1.0, "lon": 2.0}]}
GO = {"lat": 1.0, "lon": 2.0}
ROUTES = [("/api/nav/start", WP), ("/api/nav/go", GO)]


# ── Limits ──────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("url,base", ROUTES)
@pytest.mark.parametrize("speed", [0, 0.0, -1, -127, float("inf"), float("nan"), "fast", True, [5]])
def test_bad_speed_refused_and_cfg_untouched(url, base, speed):
    ctl = _Ctl()
    r = _post(ctl, url, {**base, "cruise_speed": speed})
    assert r.status_code == 400
    assert "cruise_speed" in r.get_json()["error"]
    assert ctl.activated == 0
    assert _cfg(ctl) == (40, 1.0)
    assert ctl._waypoint_nav.waypoints == []


@pytest.mark.parametrize("url,base", ROUTES)
@pytest.mark.parametrize("radius", [0, -1, float("inf"), float("nan"), "x"])
def test_bad_radius_refused_and_cfg_untouched(url, base, radius):
    ctl = _Ctl()
    r = _post(ctl, url, {**base, "arrival_radius": radius})
    assert r.status_code == 400
    assert "arrival_radius" in r.get_json()["error"]
    assert ctl.activated == 0
    assert _cfg(ctl) == (40, 1.0)
    assert ctl._waypoint_nav.waypoints == []


@pytest.mark.parametrize("url,base", ROUTES)
def test_values_clamped_to_slider_range(url, base):
    ctl = _Ctl()
    assert _post(ctl, url, {**base, "cruise_speed": 127, "arrival_radius": 50}).status_code == 200
    assert _cfg(ctl) == (ui._SPEED_SLIDER_RANGE[1], ui._RADIUS_SLIDER_RANGE[1])
    # Speed 0 is refused (see the bad-speed test); a small positive speed is
    # raised to the slider minimum.
    assert _post(ctl, url, {**base, "cruise_speed": 1, "arrival_radius": 0.01}).status_code == 200
    assert _cfg(ctl) == (ui._SPEED_SLIDER_RANGE[0], ui._RADIUS_SLIDER_RANGE[0])


@pytest.mark.parametrize("url,base", ROUTES)
def test_in_range_and_omitted_values_unchanged(url, base):
    ctl = _Ctl()
    assert _post(ctl, url, {**base, "cruise_speed": 55, "arrival_radius": 2.0}).status_code == 200
    assert _cfg(ctl) == (55, 2.0)
    assert len(ctl._waypoint_nav.waypoints) == 1
    ctl = _Ctl()
    assert _post(ctl, url, base).status_code == 200
    assert ctl._waypoint_nav._cfg.cruise_speed_byte == 40  # config default, not clamped


def test_page_sliders_use_the_server_bounds():
    html = ui._render_nav_html()
    lo_s, hi_s = ui._SPEED_SLIDER_RANGE
    lo_r, hi_r = ui._RADIUS_SLIDER_RANGE
    assert f'id="speedSlider" min="{lo_s}" max="{hi_s}"' in html
    assert f'id="radiusSlider" min="{lo_r}" max="{hi_r}"' in html
    assert "@@" not in html


# ── A refused or failed start changes nothing ───────────────────────────────

@pytest.mark.parametrize("url,base", ROUTES)
def test_refused_start_writes_nothing(url, base):
    """The gate is asked before anything is written (speed, radius, route)."""
    ctl = _Ctl(block="no_gps")
    assert _post(ctl, url, {**base, "cruise_speed": 70, "arrival_radius": 2.5}).status_code == 409
    assert _cfg(ctl) == (40, 1.0)
    assert ctl._waypoint_nav.waypoints == [], "a refused start must not replace the route"
    assert ctl.activated == 0
    assert ctl.deactivated == 0


@pytest.mark.parametrize("url,base", ROUTES)
def test_refused_start_leaves_a_live_mission_untouched(url, base):
    """A 409 must not retarget a mission that is running."""
    nav = _live_mission()
    ctl = _Ctl(block="heading_alignment_not_locked", nav=nav)
    r = _post(ctl, url, {**base, "cruise_speed": 70, "arrival_radius": 2.5})
    assert r.status_code == 409
    _assert_live_mission_untouched(nav)
    assert ctl.deactivated == 0, "a refusal must not stop the running mission"


@pytest.mark.parametrize("url,base", ROUTES)
def test_gate_closing_after_the_write_restores_the_live_mission(url, base):
    """The gate can close between the check and the activation. By then the
    route was replaced, so the whole mission state comes back from the snapshot."""
    nav = _live_mission()
    ctl = _Ctl(late_block="heading_alignment_not_locked", nav=nav)
    r = _post(ctl, url, {**base, "cruise_speed": 70, "arrival_radius": 2.5})
    assert r.status_code == 409
    assert ctl.activated == 1
    _assert_live_mission_untouched(nav)
    assert ctl.deactivated == 0


@pytest.mark.parametrize("url,base", ROUTES)
def test_fault_during_activation_restores_the_live_mission(url, base):
    nav = _live_mission()
    ctl = _Ctl(raise_on_activate=True, nav=nav)
    r = _post(ctl, url, {**base, "cruise_speed": 70, "arrival_radius": 2.5})
    assert r.status_code == 500
    _assert_live_mission_untouched(nav)
    assert ctl.deactivated == 0


def test_snapshot_and_restore_cover_everything_set_waypoints_changes():
    nav = _live_mission()
    nav._completed = True
    snap = nav.snapshot_mission()
    nav._cfg.cruise_speed_byte = 77
    nav._cfg.arrival_radius_m = 2.9
    nav.set_waypoints([Waypoint(lat=1.0, lon=2.0, name="X")])
    assert nav.current_index == 0 and nav.state == NavState.IDLE and not nav.completed
    nav.restore_mission(snap)
    assert [(w.lat, w.lon) for w in nav.waypoints] == [(10.0, 20.0), (10.1, 20.1)]
    assert nav.current_index == 1
    assert nav.state == NavState.DRIVE
    assert nav.completed
    assert (nav._cfg.cruise_speed_byte, nav._cfg.arrival_radius_m) == (40, 1.0)


# ── Bad request bodies ──────────────────────────────────────────────────────

@pytest.mark.parametrize("url", ["/api/nav/start", "/api/nav/go"])
@pytest.mark.parametrize("body", [[1, 2], "text", 5])
def test_body_that_is_not_an_object_is_refused(url, body):
    ctl = _Ctl()
    r = _post(ctl, url, body)
    assert r.status_code == 400
    assert ctl.activated == 0
    assert _cfg(ctl) == (40, 1.0)


def test_start_with_an_empty_body_is_a_400_not_a_crash():
    ctl = _Ctl()
    r = _client(ctl).post("/api/nav/start", data="null", content_type="application/json")
    assert r.status_code == 400
    assert ctl.activated == 0
