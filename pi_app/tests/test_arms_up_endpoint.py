"""GET/POST /api/arms_up/twitch_test.

Same Flask test-client shape as test_fm_params_endpoint.py. The controller
here is a fake: the safety decisions live in the controller tests.
"""

import unittest

try:
    import flask  # noqa: F401
except ImportError:
    flask = None

if flask is not None:
    from config import OakWebViewerConfig
    from pi_app.web.oak_viewer import create_app


def _state(**overrides):
    state = {
        "latched": False,
        "budget_left": 0,
        "expires_in_s": None,
        "twitch_count": 0,
        "twitch_active": False,
        "twitch_blocked_reason": None,
        "twitch_cancel_reason": None,
    }
    state.update(overrides)
    return state


class _FakeController:
    def __init__(self, accept=True, reason="disarmed"):
        self.accept = accept
        self.reason = reason
        self.calls = []
        self.state = _state()

    def request_twitch_test(self, enabled):
        if not self.accept:
            self.calls.append(("refused", enabled))
            return False, self.reason
        self.calls.append(("applied", enabled))
        self.state["latched"] = bool(enabled)
        if not enabled:
            self.state["twitch_active"] = False
            self.state["twitch_cancel_reason"] = "api"
        return True, "ok"

    def get_twitch_test_state(self):
        return dict(self.state)


@unittest.skipUnless(flask is not None, "flask not installed")
class TestArmsUpTwitchEndpoint(unittest.TestCase):
    def _client(self, controller):
        app = create_app(None, OakWebViewerConfig(), controller=controller)
        return app.test_client()

    def test_get_returns_state(self):
        fake = _FakeController()
        fake.state = _state(latched=True, budget_left=3, expires_in_s=12.0)
        resp = self._client(fake).get("/api/arms_up/twitch_test")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json()["latched"], True)
        self.assertEqual(resp.get_json()["budget_left"], 3)
        self.assertEqual(resp.get_json()["expires_in_s"], 12.0)

    def test_post_true_and_false(self):
        fake = _FakeController()
        client = self._client(fake)
        resp = client.post("/api/arms_up/twitch_test", json={"enabled": True})
        self.assertEqual(resp.status_code, 200)
        body = resp.get_json()
        self.assertIs(body["ok"], True)
        self.assertIs(body["latched"], True)
        self.assertEqual(fake.calls, [("applied", True)])

        resp = client.post("/api/arms_up/twitch_test", json={"enabled": False})
        self.assertEqual(resp.status_code, 200)
        body = resp.get_json()
        self.assertIs(body["ok"], True)
        self.assertIs(body["latched"], False)
        self.assertEqual(body["twitch_cancel_reason"], "api")
        self.assertEqual(fake.calls[-1], ("applied", False))

    def test_bad_payloads_apply_nothing(self):
        fake = _FakeController()
        client = self._client(fake)
        payloads = (
            {},
            {"enabled": True, "budget": 9},
            {"enabled": 1},
            {"enabled": "true"},
            {"enabled": None},
            {"latched": True},
            [],
        )
        for payload in payloads:
            resp = client.post("/api/arms_up/twitch_test", json=payload)
            self.assertEqual(resp.status_code, 400, payload)
        resp = client.post(
            "/api/arms_up/twitch_test",
            data="not-json",
            content_type="application/json",
        )
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(fake.calls, [])
        self.assertFalse(fake.state["latched"])

    def test_refused_is_409_and_changes_nothing(self):
        fake = _FakeController(accept=False, reason="disarmed")
        before = dict(fake.state)
        resp = self._client(fake).post(
            "/api/arms_up/twitch_test", json={"enabled": True},
        )
        self.assertEqual(resp.status_code, 409)
        self.assertEqual(resp.get_json()["reason"], "disarmed")
        self.assertEqual(fake.state, before)
        self.assertEqual(fake.calls, [("refused", True)])

    def test_503_without_controller(self):
        client = self._client(None)
        self.assertEqual(client.get("/api/arms_up/twitch_test").status_code, 503)
        resp = client.post("/api/arms_up/twitch_test", json={"enabled": True})
        self.assertEqual(resp.status_code, 503)
