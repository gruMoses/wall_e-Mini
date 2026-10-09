#!/usr/bin/env python3
"""Replay raw depth snapshots through the production obstacle corridor.

Input: ``logs/depth_snapshots/*.npz`` files written by the local-only
``POST /api/debug/depth_snapshot`` endpoint (``depth`` uint16 mm,
``rgb_jpeg``, ``meta`` JSON with intrinsics, persons and the on-robot
depth stats). Each frame goes through ``OakDepthReader._poll_depth`` with
the config from ``config.py`` (or overrides), the same way the corridor
speckle tests do, then through the MANUAL throttle curve.

Per frame it prints the corridor reading the robot would publish and a
depth histogram of the valid corridor pixels, so a near "wall" can be
told from a sparse near field (mesh netting, false stereo matches) that
lets far pixels through between the strands.

Usage:
  python3 tools/replay_depth_snapshots.py logs/netting_20261008/depth_snapshots
  python3 tools/replay_depth_snapshots.py <dir-or-files> --rgb-out /tmp/rgb --set corridor_speckle_min_neighbours=24
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from config import config  # noqa: E402
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController  # noqa: E402
from pi_app.hardware.oak_depth import OakDepthReader  # noqa: E402
from pi_app.control.netting_mute import near_field_present  # noqa: E402

try:
    from pi_app.control.follow_me import PersonDetection  # noqa: E402
except Exception:  # pragma: no cover
    PersonDetection = None  # type: ignore[assignment]


class _Frame:
    def __init__(self, arr):
        self._arr = arr

    def getFrame(self):
        return self._arr


class _Queue:
    def __init__(self, frame=None):
        self._frame = frame
        self._served = False

    def tryGet(self):
        if self._frame is not None and not self._served:
            self._served = True
            return self._frame
        return None


def _coerce(value: str):
    low = value.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _corridor_band(depth: np.ndarray, cfg, intrinsics):
    """Replicate the corridor selection of _poll_depth for the histogram."""
    h, w = depth.shape
    rh = cfg.roi_height_pct
    rv = getattr(cfg, "roi_vertical_offset_pct", 0.0)
    cy_norm = max(rh / 2.0, min(1.0 - rh / 2.0, 0.5 + rv))
    y0 = int(h * (cy_norm - rh / 2))
    y1 = int(h * (cy_norm + rh / 2))
    band = depth[y0:y1, :]
    robot_half_mm = getattr(cfg, "robot_width_m", 0.0) * 500.0
    min_depth_mm = int(getattr(cfg, "min_depth_mm", 600))
    if robot_half_mm > 0 and intrinsics:
        fx, _fy, cx, _cy = intrinsics
        x_off = np.abs(np.arange(w, dtype=np.float32) - cx)
        in_corr = (band.astype(np.float32) * x_off[np.newaxis, :]) <= fx * robot_half_mm
    else:
        rw = cfg.roi_width_pct
        in_corr = np.zeros_like(band, dtype=bool)
        in_corr[:, int(w * (0.5 - rw / 2)):int(w * (0.5 + rw / 2))] = True
    valid = (band > min_depth_mm) & in_corr
    return band, in_corr, valid


_BINS_MM = [350, 450, 550, 650, 750, 850, 950, 1050, 1150, 1250, 1350, 1500, 2000, 3000, 5000, 100000]


def _hist_line(vals_mm: np.ndarray) -> str:
    if vals_mm.size == 0:
        return "(no valid corridor pixels)"
    counts, _ = np.histogram(vals_mm, bins=_BINS_MM)
    total = float(vals_mm.size)
    parts = []
    for lo, hi, c in zip(_BINS_MM[:-1], _BINS_MM[1:], counts):
        if c == 0:
            continue
        label = f"{lo/1000:.2f}-{hi/1000:.2f}" if hi < 100000 else f">{lo/1000:.0f}"
        parts.append(f"{label}:{100.0*c/total:.0f}%")
    return " ".join(parts)


def replay_file(path: Path, cfg, persistence: int, use_persons: bool, rgb_out: Path | None):
    data = np.load(path, allow_pickle=False)
    depth = np.asarray(data["depth"], dtype=np.uint16)
    meta = json.loads(str(data["meta"]))
    intr = meta.get("intrinsics")
    reader = OakDepthReader(obstacle_config=cfg, follow_me_config=config.follow_me)
    if intr:
        intr_t = tuple(float(v) for v in intr)
        reader._intrinsics_for = lambda w_, h_, _i=intr_t: _i  # type: ignore[assignment]
    if use_persons and PersonDetection is not None and meta.get("persons"):
        persons = []
        for p in meta["persons"]:
            try:
                persons.append(PersonDetection(
                    x_m=float(p.get("x_m", 0.0)), z_m=float(p.get("z_m", 0.0)),
                    confidence=float(p.get("confidence", 0.0)),
                    bbox=tuple(p.get("bbox") or ()),
                ))
            except TypeError:
                persons = []
                break
        if persons:
            with reader._lock:
                reader._det_state.persons = persons
    for _ in range(max(1, persistence)):
        reader._poll_depth(_Queue(_Frame(depth)), _Queue(None), np)
    dist_m, _age = reader.get_min_distance()
    stats = reader.get_depth_stats()
    oa = ObstacleAvoidanceController(cfg)
    scale_manual = oa.compute_throttle_scale(dist_m, 0.0, is_manual=True)
    scale_auto = ObstacleAvoidanceController(cfg).compute_throttle_scale(dist_m, 0.0, is_manual=False)
    band, in_corr, valid = _corridor_band(depth, cfg, intr)
    vals = band[valid].astype(np.float32)
    near = vals[vals <= cfg.slow_distance_m * 1000.0]
    on_robot = meta.get("depth_stats") or {}
    row = {
        "file": path.name,
        "dist_m": None if not np.isfinite(dist_m) else round(float(dist_m), 2),
        "manual_scale": round(float(scale_manual), 2),
        "auto_scale": round(float(scale_auto), 2),
        "corridor_valid_pct": round(float(stats.corridor_valid_pct), 1),
        "support_px": int(stats.corridor_support_px),
        "near_px": int(stats.corridor_near_px),
        "speckle_px": int(stats.corridor_speckle_px),
        "fallback": bool(stats.corridor_speckle_fallback),
        "near_share_pct": round(100.0 * near.size / max(1, vals.size), 1),
        "robot_dist_m": on_robot.get("min_distance_m"),
        "persons": len(meta.get("persons") or []),
        "near_field": near_field_present(
            dist_m, stats.corridor_near_px, stats.corridor_support_px,
            float(getattr(cfg, "netting_mute_max_distance_m", 0.70)),
            float(getattr(cfg, "netting_mute_min_near_share", 0.85)),
        ),
        "hist": _hist_line(vals),
    }
    if rgb_out is not None:
        rgb = np.asarray(data["rgb_jpeg"], dtype=np.uint8)
        if rgb.size:
            rgb_out.mkdir(parents=True, exist_ok=True)
            (rgb_out / (path.stem + ".jpg")).write_bytes(rgb.tobytes())
    return row


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="npz files or directories")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override an ObstacleAvoidanceConfig field (repeatable)")
    ap.add_argument("--persistence", type=int, default=None,
                    help="polls per frame (default: corridor_persistence_polls)")
    ap.add_argument("--no-persons", action="store_true", help="ignore the recorded person boxes")
    ap.add_argument("--rgb-out", type=Path, default=None, help="write each frame's RGB JPEG here")
    ap.add_argument("--json", action="store_true", help="one JSON object per line instead of a table")
    args = ap.parse_args(argv)

    cfg = config.obstacle_avoidance
    overrides = {}
    for item in args.set:
        key, _, value = item.partition("=")
        if not hasattr(cfg, key):
            ap.error(f"unknown ObstacleAvoidanceConfig field: {key}")
        overrides[key] = _coerce(value)
    if overrides:
        cfg = replace(cfg, **overrides)
    persistence = args.persistence or int(getattr(cfg, "corridor_persistence_polls", 2) or 2)

    files: list[Path] = []
    for p in args.paths:
        pp = Path(p)
        if pp.is_dir():
            files.extend(sorted(pp.glob("*.npz")))
        else:
            files.append(pp)
    if not files:
        ap.error("no .npz files found")

    rows = []
    for f in files:
        try:
            rows.append(replay_file(f, cfg, persistence, not args.no_persons, args.rgb_out))
        except Exception as exc:  # a partial copy or a bad file must not end the run
            print(f"skip {f.name}: {exc}", file=sys.stderr)
    if not rows:
        ap.error("no readable .npz files")
    if args.json:
        for r in rows:
            print(json.dumps(r))
        return 0
    print(f"{'file':34} {'dist':>5} {'man':>4} {'auto':>4} {'valid%':>6} {'supp':>6} {'near':>6} {'spk':>5} {'near%':>5} {'NF':>2}  histogram (valid corridor px)")
    for r in rows:
        d = "clear" if r["dist_m"] is None else f"{r['dist_m']:.2f}"
        print(f"{r['file']:34} {d:>5} {r['manual_scale']:>4.2f} {r['auto_scale']:>4.2f} "
              f"{r['corridor_valid_pct']:>6.1f} {r['support_px']:>6d} {r['near_px']:>6d} "
              f"{r['speckle_px']:>5d} {r['near_share_pct']:>5.1f} {'Y' if r['near_field'] else '.':>2}  {r['hist']}")
    nf = sum(1 for r in rows if r["near_field"])
    print(f"near-field present (netting mute would be active) in {nf}/{len(rows)} frames")
    dists = [r["dist_m"] for r in rows if r["dist_m"] is not None]
    scales = [r["manual_scale"] for r in rows]
    print(f"\n{len(rows)} frames; distance min/median {min(dists) if dists else None}/"
          f"{float(np.median(dists)) if dists else None} m; MANUAL scale min/median "
          f"{min(scales):.2f}/{float(np.median(scales)):.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
