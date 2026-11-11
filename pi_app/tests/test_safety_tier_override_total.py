"""_apply_safety_tier_override must never raise on a malformed detection.

The 0.8 m person/animal stop runs on every depth poll. An exception there
drops the poll's result, so the function must be total. Well-formed input
must decide exactly as before (the old function is the reference below).
"""

from __future__ import annotations

import random
import sys
from pathlib import Path
from types import SimpleNamespace

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pi_app.hardware.oak_depth import OakDepthReader  # noqa: E402

RADIUS = 0.8
WIDTH = 0.40
NAN = float("nan")
INF = float("inf")


def _reference(detections, corridor_p5_mm, safety_stop_radius_m,
               safety_stop_bbox_width=0.0):
    """The function as it was before 2026-11-10 (copied verbatim)."""
    effective_min_mm = corridor_p5_mm
    width_thresh = float(safety_stop_bbox_width)
    for det in detections:
        if getattr(det, "safety_tier", "") != "stop":
            continue
        z_m = getattr(det, "z_m", 0)
        if z_m > 0:
            if det.z_m < safety_stop_radius_m:
                return 0.0, det
            det_mm = det.z_m * 1000.0
            if det_mm < effective_min_mm:
                effective_min_mm = det_mm
            continue
        if width_thresh <= 0.0:
            continue
        if getattr(det, "depth_status", "ok") == "ok":
            continue
        bbox = getattr(det, "bbox", None)
        if bbox is None or len(bbox) < 4:
            continue
        width = float(bbox[2]) - float(bbox[0])
        if width >= width_thresh:
            return 0.0, det
    return effective_min_mm, None


def _new(dets, corridor=3000.0, width=WIDTH):
    return OakDepthReader._apply_safety_tier_override(
        dets, corridor, RADIUS, safety_stop_bbox_width=width)


def _det(tier="stop", z=0.0, status="no_support", bbox=(0.2, 0.1, 0.65, 0.8)):
    return SimpleNamespace(safety_tier=tier, z_m=z, depth_status=status,
                           bbox=bbox, label_name="person")


# --- well-formed input is unchanged -------------------------------------

def test_well_formed_matches_reference_on_random_inputs():
    rng = random.Random(20261110)
    statuses = ["ok", "no_support", "ambiguous", "far_veto", "no_frame",
                "height_veto"]
    for _ in range(4000):
        dets = []
        for _ in range(rng.randint(0, 4)):
            x1 = rng.uniform(0.0, 0.9)
            x2 = rng.uniform(x1, 1.0)
            dets.append(_det(
                tier=rng.choice(["stop", "stop", "slow", "ignore", ""]),
                z=rng.choice([0.0, 0.0, True, False, rng.uniform(0.05, 6.0),
                              rng.uniform(-1.0, 0.0), INF]),
                status=rng.choice(statuses),
                bbox=rng.choice([(x1, 0.1, x2, 0.8), None, (x1, 0.1)]),
            ))
        corridor = rng.choice([INF, rng.uniform(200.0, 8000.0)])
        wt = rng.choice([0.0, 0.40, rng.uniform(0.0, 0.8)])
        old = _reference(dets, corridor, RADIUS, wt)
        new = OakDepthReader._apply_safety_tier_override(
            dets, corridor, RADIUS, safety_stop_bbox_width=wt)
        assert new[0] == old[0]
        assert new[1] is old[1]


# --- malformed input never raises ----------------------------------------

def test_reference_raises_on_non_numeric_z():
    # Proves the failure mode these tests guard against.
    try:
        _reference([_det(z="far")], 3000.0, RADIUS, WIDTH)
    except TypeError:
        return
    raise AssertionError("reference should raise")


def test_non_numeric_z_wide_box_still_stops():
    for bad in ("far", None, [1.0], object()):
        det = _det(z=bad, status="ok")
        eff, stop = _new([det])
        assert eff == 0.0 and stop is det


def test_non_numeric_z_narrow_box_ignored():
    det = _det(z="far", status="ok", bbox=(0.82, 0.1, 1.0, 0.7))
    assert _new([det]) == (3000.0, None)


def test_nan_z_is_unknown_depth_even_when_status_ok():
    det = _det(z=NAN, status="ok")
    eff, stop = _new([det])
    assert eff == 0.0 and stop is det


def test_missing_depth_status_is_unknown_not_ok():
    det = SimpleNamespace(safety_tier="stop", z_m=0.0,
                          bbox=(0.2, 0.1, 0.65, 0.8))
    eff, stop = _new([det])
    assert eff == 0.0 and stop is det


def test_malformed_bbox_does_not_raise_and_others_are_evaluated():
    close = _det(z=0.5, status="ok", bbox=None)
    for bad in ("abc", 5, (None, 0, None, 0), ("a", "b", "c", "d"), (1,),
                [], (NAN, 0, NAN, 0), object()):
        broken = _det(z=0.0, status="no_support", bbox=bad)
        assert _new([broken]) == (3000.0, None)
        eff, stop = _new([broken, close])
        assert eff == 0.0 and stop is close


def test_detection_without_any_readable_field_is_ignored():
    class Bad:
        @property
        def safety_tier(self):
            raise RuntimeError("boom")

    close = _det(z=0.5, status="ok")
    assert _new([Bad()]) == (3000.0, None)
    eff, stop = _new([Bad(), object(), None, close])
    assert eff == 0.0 and stop is close


def test_non_iterable_detections_do_not_raise():
    assert _new(None) == (3000.0, None)
    assert _new(5) == (3000.0, None)


def test_explicit_ok_with_no_range_is_skipped():
    det = _det(z=0.0, status="ok")  # wide box, but a real "no range" reading
    assert _new([det]) == (3000.0, None)
    no_z = SimpleNamespace(safety_tier="stop", depth_status="ok",
                           bbox=(0.2, 0.1, 0.65, 0.8))
    assert _new([no_z]) == (3000.0, None)


def test_iterator_that_raises_fails_safe_to_a_stop():
    def gen():
        yield _det(z=3.0, status="ok")
        raise RuntimeError("boom")

    eff, stop = _new(gen())
    assert eff == 0.0
