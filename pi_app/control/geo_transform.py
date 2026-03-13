"""Affine transform between pixel coordinates and GPS (lat/lon).

Uses a local-tangent-plane projection (GPS → meters relative to centroid)
to avoid floating-point precision issues with tiny lat/lon deltas at
property scale, then fits a least-squares affine in meter space.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class ControlPoint:
    """A single calibration control point linking pixel and GPS coords."""
    pixel_x: float
    pixel_y: float
    lat: float
    lon: float


@dataclass(frozen=True)
class MapCalibration:
    """Computed affine transform between pixel and GPS coordinates.

    Each matrix is stored as a 6-element tuple (a, b, tx, c, d, ty)
    representing the 2x3 affine: [a b tx; c d ty].
    """
    pixel_to_gps: Tuple[float, ...]    # pixel → (lat, lon)
    gps_to_pixel: Tuple[float, ...]    # (lat, lon) → pixel
    control_points: List[ControlPoint]
    residuals: List[float]             # per-point error in metres
    rms_error_m: float


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Geodesic distance in metres between two (lat, lon) in degrees."""
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _solve_affine_lstsq(src: list, dst: list) -> Tuple[float, ...]:
    """Least-squares affine fit from src points to dst points.

    src/dst are lists of (x, y) tuples.  Returns 6-element affine
    (a, b, tx, c, d, ty) minimising || A @ [x y 1]^T - [u v]^T ||^2.

    Uses normal equations (A^T A)^-1 A^T b instead of numpy to avoid
    the numpy dependency for this single function.
    """
    n = len(src)
    # Build A matrix (n x 3) and B matrix (n x 2)
    # For each point: [x, y, 1] @ [a c; b d; tx ty] = [u, v]
    # Solve two independent systems: one for u-coords, one for v-coords.

    # A^T A (3x3) and A^T b (3x1) for each output dimension
    ata = [[0.0] * 3 for _ in range(3)]
    atb_u = [0.0] * 3
    atb_v = [0.0] * 3

    for i in range(n):
        row = [src[i][0], src[i][1], 1.0]
        u, v = dst[i]
        for r in range(3):
            for c in range(3):
                ata[r][c] += row[r] * row[c]
            atb_u[r] += row[r] * u
            atb_v[r] += row[r] * v

    # Solve 3x3 linear system via Cramer's rule
    def solve3(m, b):
        def det3(m):
            return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

        d = det3(m)
        if abs(d) < 1e-30:
            raise ValueError("Singular matrix — control points may be collinear")

        result = []
        for col in range(3):
            mc = [row[:] for row in m]
            for row in range(3):
                mc[row][col] = b[row]
            result.append(det3(mc) / d)
        return result

    sol_u = solve3(ata, atb_u)  # [a, b, tx]
    sol_v = solve3(ata, atb_v)  # [c, d, ty]

    return (sol_u[0], sol_u[1], sol_u[2], sol_v[0], sol_v[1], sol_v[2])


def _apply_affine(m: Tuple[float, ...], x: float, y: float) -> Tuple[float, float]:
    """Apply a 2x3 affine (a, b, tx, c, d, ty) to point (x, y)."""
    a, b, tx, c, d, ty = m
    return (a * x + b * y + tx, c * x + d * y + ty)


def compute_affine(points: List[ControlPoint]) -> MapCalibration:
    """Compute least-squares affine transform from 3+ control points.

    Internally converts GPS to a local tangent plane (metres relative to
    centroid) so the affine fit is numerically stable.  The final matrices
    compose the GPS↔metre and metre↔pixel transforms into single affines.
    """
    if len(points) < 3:
        raise ValueError(f"Need at least 3 control points, got {len(points)}")

    # Centroid for local tangent plane
    lat0 = sum(p.lat for p in points) / len(points)
    lon0 = sum(p.lon for p in points) / len(points)
    cos_lat = math.cos(math.radians(lat0))

    # Metres per degree at this latitude
    m_per_deg_lat = math.radians(EARTH_RADIUS_M)          # ~111,320 m
    m_per_deg_lon = math.radians(EARTH_RADIUS_M) * cos_lat

    # Convert GPS to local metres
    def gps_to_m(lat, lon):
        return ((lon - lon0) * m_per_deg_lon,
                (lat - lat0) * m_per_deg_lat)

    def m_to_gps(mx, my):
        return (lat0 + my / m_per_deg_lat,
                lon0 + mx / m_per_deg_lon)

    # Build source (pixel) and destination (metre) point lists
    px_pts = [(p.pixel_x, p.pixel_y) for p in points]
    m_pts = [gps_to_m(p.lat, p.lon) for p in points]

    # Fit pixel → metre affine
    px_to_m = _solve_affine_lstsq(px_pts, m_pts)
    # Fit metre → pixel affine (inverse direction)
    m_to_px = _solve_affine_lstsq(m_pts, px_pts)

    # Compose pixel→metre→GPS and GPS→metre→pixel into direct affines.
    # pixel→GPS: first apply px_to_m, then m_to_gps (linear scaling + offset).
    # m_to_gps is: lat = lat0 + my/s_lat, lon = lon0 + mx/s_lon
    # So pixel→GPS = compose(m_to_gps_affine, px_to_m)
    a, b, tx, c, d, ty = px_to_m
    pixel_to_gps = (
        a / m_per_deg_lon,  # maps px → lon contribution
        b / m_per_deg_lon,
        tx / m_per_deg_lon + lon0,
        c / m_per_deg_lat,  # maps px → lat contribution
        d / m_per_deg_lat,
        ty / m_per_deg_lat + lat0,
    )
    # Note: pixel_to_gps output is (lon, lat) due to the x↔lon, y↔lat mapping.
    # We want (lat, lon) in the API, so swap the rows:
    pixel_to_gps = (
        pixel_to_gps[3], pixel_to_gps[4], pixel_to_gps[5],  # lat row
        pixel_to_gps[0], pixel_to_gps[1], pixel_to_gps[2],  # lon row
    )

    # GPS→pixel: first apply gps_to_m_affine, then m_to_px.
    # gps_to_m is: mx = (lon-lon0)*s_lon, my = (lat-lat0)*s_lat
    # Compose with m_to_px:
    a2, b2, tx2, c2, d2, ty2 = m_to_px
    # Input is (lat, lon). In metre space: mx = (lon-lon0)*s_lon, my = (lat-lat0)*s_lat
    # m_to_px output_x = a2*mx + b2*my + tx2
    #                   = a2*s_lon*(lon-lon0) + b2*s_lat*(lat-lat0) + tx2
    #                   = b2*s_lat*lat + a2*s_lon*lon + (tx2 - b2*s_lat*lat0 - a2*s_lon*lon0)
    gps_to_pixel = (
        b2 * m_per_deg_lat,   # lat coefficient for pixel_x
        a2 * m_per_deg_lon,   # lon coefficient for pixel_x
        tx2 - b2 * m_per_deg_lat * lat0 - a2 * m_per_deg_lon * lon0,
        d2 * m_per_deg_lat,   # lat coefficient for pixel_y
        c2 * m_per_deg_lon,   # lon coefficient for pixel_y
        ty2 - d2 * m_per_deg_lat * lat0 - c2 * m_per_deg_lon * lon0,
    )

    # Compute residuals
    residuals = []
    for p in points:
        pred_lat, pred_lon = pixel_to_gps_pt(pixel_to_gps, p.pixel_x, p.pixel_y)
        residuals.append(_haversine_m(p.lat, p.lon, pred_lat, pred_lon))

    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))

    return MapCalibration(
        pixel_to_gps=pixel_to_gps,
        gps_to_pixel=gps_to_pixel,
        control_points=points,
        residuals=residuals,
        rms_error_m=rms,
    )


def pixel_to_gps_pt(
    transform: Tuple[float, ...], px: float, py: float
) -> Tuple[float, float]:
    """Convert pixel coordinates to (lat, lon).

    The transform tuple has the lat row first: (a_lat, b_lat, t_lat, a_lon, b_lon, t_lon).
    Input is (pixel_x, pixel_y), output is (lat, lon).
    """
    a, b, tx, c, d, ty = transform
    lat = a * px + b * py + tx
    lon = c * px + d * py + ty
    return (lat, lon)


def gps_to_pixel_pt(
    transform: Tuple[float, ...], lat: float, lon: float
) -> Tuple[float, float]:
    """Convert (lat, lon) to pixel coordinates.

    The transform tuple maps (lat, lon) → (pixel_x, pixel_y).
    """
    a, b, tx, c, d, ty = transform
    pixel_x = a * lat + b * lon + tx
    pixel_y = c * lat + d * lon + ty
    return (pixel_x, pixel_y)


def save_calibration(
    path: str | Path,
    cal: MapCalibration,
    image_path: str,
    image_width: int,
    image_height: int,
) -> None:
    """Persist calibration to JSON."""
    import datetime

    data = {
        "image_path": image_path,
        "image_width": image_width,
        "image_height": image_height,
        "control_points": [
            {"pixel_x": p.pixel_x, "pixel_y": p.pixel_y,
             "lat": p.lat, "lon": p.lon}
            for p in cal.control_points
        ],
        "pixel_to_gps": list(cal.pixel_to_gps),
        "gps_to_pixel": list(cal.gps_to_pixel),
        "residuals": cal.residuals,
        "rms_error_m": cal.rms_error_m,
        "calibrated_at": datetime.datetime.now().isoformat(),
    }
    Path(path).write_text(json.dumps(data, indent=2))


def load_calibration(path: str | Path) -> Optional[MapCalibration]:
    """Load calibration from JSON.  Returns None if file doesn't exist."""
    p = Path(path)
    if not p.exists():
        return None
    data = json.loads(p.read_text())
    points = [
        ControlPoint(**cp) for cp in data["control_points"]
    ]
    return MapCalibration(
        pixel_to_gps=tuple(data["pixel_to_gps"]),
        gps_to_pixel=tuple(data["gps_to_pixel"]),
        control_points=points,
        residuals=data.get("residuals", []),
        rms_error_m=data.get("rms_error_m", 0.0),
    )
