"""POST /api/debug/depth_snapshot (local-only raw depth capture).

Same Flask test-client shape as test_arms_up_endpoint.py. The reader is a
fake that exposes only the thread-safe getters the endpoint uses.
"""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    import flask  # noqa: F401
except ImportError:
    flask = None

try:
    import numpy as np
except ImportError:
    np = None

if flask is not None and np is not None:
    from config import OakWebViewerConfig
    from pi_app.control.follow_me import PersonDetection
    from pi_app.hardware.oak_depth import DepthStats
    from pi_app.web import oak_viewer
    from pi_app.web.oak_viewer import create_app


class _FakeReader:
    def __init__(self, frame=True):
        if frame:
            self.frame = np.arange(40 * 64, dtype=np.uint16).reshape(40, 64)
        else:
            self.frame = None
        self.persons = [
            PersonDetection(
                x_m=0.1, z_m=2.5, confidence=0.88,
                bbox=(0.40, 0.10, 0.55, 0.90), track_id=7,
            )
        ]
        self.stats = DepthStats(
            min_distance_m=0.62, p5_mm=620.0, corridor_near_px=2100,
        )

    def get_latest_depth_frame(self):
        return self.frame

    def get_intrinsics(self, width, height):
        return (456.89 * width / 640.0, 456.89 * width / 640.0,
                334.95 * width / 640.0, height / 2.0)

    def get_person_detections(self):
        return list(self.persons)

    def get_depth_stats(self):
        return self.stats

    def get_latest_rgb_frame(self):
        return None, 0.0


@unittest.skipUnless(flask is not None and np is not None,
                     "flask or numpy not installed")
class TestDepthSnapshotEndpoint(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.out_dir = Path(self._tmp.name) / "depth_snapshots"
        self._patch = patch.object(oak_viewer, "DEPTH_SNAPSHOT_DIR", self.out_dir)
        self._patch.start()

    def tearDown(self):
        self._patch.stop()
        self._tmp.cleanup()

    def _client(self, reader):
        app = create_app(None, OakWebViewerConfig(), oak_reader=reader)
        return app.test_client()

    def test_local_post_saves_frames_and_context(self):
        reader = _FakeReader()
        resp = self._client(reader).post(
            "/api/debug/depth_snapshot", json={"count": 2, "interval_s": 0.1})
        self.assertEqual(resp.status_code, 200)
        body = resp.get_json()
        self.assertIs(body["ok"], True)
        self.assertEqual(len(body["saved"]), 2)
        self.assertEqual(body["saved"][0]["corridor_near_px"], 2100)
        files = sorted(self.out_dir.glob("*.npz"))
        self.assertEqual(len(files), 2)
        with np.load(files[0]) as data:
            np.testing.assert_array_equal(data["depth"], reader.frame)
            self.assertEqual(data["depth"].dtype, np.uint16)
            meta = json.loads(str(data["meta"]))
        self.assertEqual(meta["shape"], [40, 64])
        self.assertAlmostEqual(meta["intrinsics"][0], 456.89 * 64 / 640.0)
        self.assertEqual(meta["persons"][0]["track_id"], 7)
        self.assertEqual(meta["persons"][0]["bbox"], [0.40, 0.10, 0.55, 0.90])
        self.assertEqual(meta["depth_stats"]["corridor_near_px"], 2100)

    def test_saved_frame_is_a_copy(self):
        reader = _FakeReader()
        original = reader.frame.copy()
        self._client(reader).post("/api/debug/depth_snapshot", json={"count": 1})
        reader.frame[:] = 0
        with np.load(next(self.out_dir.glob("*.npz"))) as data:
            np.testing.assert_array_equal(data["depth"], original)

    def test_empty_body_uses_defaults(self):
        resp = self._client(_FakeReader()).post(
            "/api/debug/depth_snapshot", data="", content_type="application/json")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(len(resp.get_json()["saved"]), 3)

    def test_remote_peer_is_refused(self):
        resp = self._client(_FakeReader()).post(
            "/api/debug/depth_snapshot", json={"count": 1},
            environ_base={"REMOTE_ADDR": "192.168.86.20"})
        self.assertEqual(resp.status_code, 403)
        self.assertFalse(self.out_dir.exists() and any(self.out_dir.iterdir()))

    def test_bad_bodies_are_rejected(self):
        client = self._client(_FakeReader())
        for body in ({"count": 0}, {"count": 11}, {"interval_s": 5},
                     {"interval_s": 0.01}, {"count": "many"}, {"extra": 1},
                     [1, 2]):
            resp = client.post("/api/debug/depth_snapshot", json=body)
            self.assertEqual(resp.status_code, 400, msg=str(body))
        self.assertFalse(self.out_dir.exists() and any(self.out_dir.iterdir()))

    def test_no_reader_or_no_frame_is_503(self):
        resp = self._client(None).post("/api/debug/depth_snapshot", json={})
        self.assertEqual(resp.status_code, 503)
        resp = self._client(_FakeReader(frame=False)).post(
            "/api/debug/depth_snapshot", json={"count": 1})
        self.assertEqual(resp.status_code, 503)


if __name__ == "__main__":
    unittest.main()
