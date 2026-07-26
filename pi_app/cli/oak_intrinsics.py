#!/usr/bin/env python3
"""Print the OAK-D's factory camera intrinsics. READ-ONLY.

WHY THIS EXISTS
---------------
The obstacle corridor derives focal length by trigonometry from a hand-entered
field of view instead of reading the per-unit calibration the camera ships with:

    fx        = (w / 2) / tan(hfov / 2)              oak_depth.py
    threshold = fx * robot_half_mm
    in_corridor = (depth * |x - cx|) <= threshold

Three sources disagree about that field of view:

    config.py:184 .................................. 81.0 deg
    oak_depth.py fallback default, BOTH call sites .. 73.0 deg
    docs/obstacle_avoidance_ground_plane_plan.md .... 73 deg (board measurement)
    Luxonis spec, OAK-D Lite colour sensor .......... 81 deg DIAGONAL, ~69 deg horizontal

81.0 looks like a diagonal figure sitting in a horizontal field. The error
matters and it points the unsafe way: too large an HFOV yields too small an fx,
which shrinks the corridor threshold, so the mask rejects obstacles that are
inside the robot's real swept path.

This tool ends the argument. It reads the intrinsics out of the device EEPROM
and reports the implied FOV at the resolutions the code actually uses.

USAGE
-----
The OAK can only be opened by one process at a time, so stop the main service
first:

    sudo systemctl stop wall-e.service
    python3 -m pi_app.cli.oak_intrinsics
    sudo systemctl start wall-e.service

Nothing here writes to the device, the EEPROM, or any config file.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

# Resolutions this codebase actually requests, and what each one drives.
#   640x400 is the operative one: StereoDepth.setOutputSize(640, 400), and both
#   fx call sites consume the depth frame.
_RESOLUTIONS = (
    (640, 400, "depth / stereo output — THE ONE THAT DRIVES THE CORRIDOR"),
    (640, 352, "YOLOv8n network input"),
    (640, 480, "RGB preview / gesture stream"),
)

_CANDIDATES = (
    (81.0, "config.py:184 (suspected DFOV in an HFOV field)"),
    (73.0, "oak_depth.py fallback + 2026-03-09 board measurement"),
    (69.0, "Luxonis spec, OAK-D Lite colour HFOV"),
)


def implied_fov_deg(pixels: float, focal_px: float) -> float:
    """Full-angle FOV covered by `pixels` at focal length `focal_px`."""
    return math.degrees(2.0 * math.atan((pixels / 2.0) / focal_px))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    try:
        import depthai as dai
    except ImportError:
        print("depthai not installed", file=sys.stderr)
        return 2

    try:
        device = dai.Device()
    except Exception as exc:
        print(
            "Could not open the OAK-D: %s\n"
            "It is almost certainly held by wall-e.service — stop it first:\n"
            "    sudo systemctl stop wall-e.service" % exc,
            file=sys.stderr,
        )
        return 1

    out: dict = {"sockets": {}}
    try:
        calib = device.readCalibration()
        try:
            connected = [str(s) for s in device.getConnectedCameras()]
        except Exception:
            connected = []
        out["connected_cameras"] = connected

        # CAM_A is the colour sensor; depth is aligned to it on the YOLO path
        # (stereo.setDepthAlign(CAM_A)), so CAM_A carries the operative
        # intrinsics. CAM_C is the rectified-right mono frame, which is what
        # StereoDepth outputs in when depth is NOT aligned.
        sockets = [("CAM_A", dai.CameraBoardSocket.CAM_A),
                   ("CAM_B", dai.CameraBoardSocket.CAM_B),
                   ("CAM_C", dai.CameraBoardSocket.CAM_C)]

        for name, socket in sockets:
            entry: dict = {}
            try:
                spec_fov = getattr(calib, "getFov", None)
                if spec_fov is not None:
                    entry["eeprom_spec_fov_deg"] = round(float(spec_fov(socket)), 3)
            except Exception:
                pass

            for w, h, why in _RESOLUTIONS:
                try:
                    m = calib.getCameraIntrinsics(socket, resizeWidth=w, resizeHeight=h)
                except Exception as exc:
                    entry["%dx%d" % (w, h)] = {"error": str(exc)}
                    continue
                fx, fy = float(m[0][0]), float(m[1][1])
                cx, cy = float(m[0][2]), float(m[1][2])
                entry["%dx%d" % (w, h)] = {
                    "why": why,
                    "fx": round(fx, 2), "fy": round(fy, 2),
                    "cx": round(cx, 2), "cy": round(cy, 2),
                    "implied_hfov_deg": round(implied_fov_deg(w, fx), 2),
                    "implied_vfov_deg": round(implied_fov_deg(h, fy), 2),
                    "principal_offset_px": round(cx - w / 2.0, 2),
                }
            out["sockets"][name] = entry
    finally:
        try:
            device.close()
        except Exception:
            pass

    if args.json:
        print(json.dumps(out, indent=2))
        return 0

    print("OAK-D factory intrinsics (read-only)")
    print("connected cameras: %s" % (out.get("connected_cameras") or "unknown"))
    for name, entry in out["sockets"].items():
        print("\n=== %s ===" % name)
        if "eeprom_spec_fov_deg" in entry:
            print("  EEPROM spec FOV: %.3f deg" % entry["eeprom_spec_fov_deg"])
        for w, h, _why in _RESOLUTIONS:
            key = "%dx%d" % (w, h)
            d = entry.get(key, {})
            if "error" in d:
                print("  %-9s ERROR: %s" % (key, d["error"]))
                continue
            print(
                "  %-9s fx=%-7.2f cx=%-7.2f  implied HFOV %.2f deg / VFOV %.2f deg   [%s]"
                % (key, d["fx"], d["cx"], d["implied_hfov_deg"],
                   d["implied_vfov_deg"], d["why"])
            )
            if abs(d["principal_offset_px"]) > 8.0:
                print("       NOTE: principal point is %+.1f px off centre — the code's "
                      "cx=w/2 assumption is measurably wrong here"
                      % d["principal_offset_px"])

    # The verdict: compare the operative 640x400 CAM_A number to the candidates.
    operative = out["sockets"].get("CAM_A", {}).get("640x400", {})
    measured = operative.get("implied_hfov_deg")
    if measured is None:
        print("\nCould not resolve the operative CAM_A 640x400 intrinsics.")
        return 1

    print("\n" + "=" * 68)
    print("VERDICT — operative HFOV (CAM_A @ 640x400): %.2f deg" % measured)
    print("=" * 68)
    for cand, label in _CANDIDATES:
        fx_cand = (640 / 2.0) / math.tan(math.radians(cand / 2.0))
        err = (fx_cand / operative["fx"] - 1.0) * 100.0
        print("  %5.1f deg -> fx %7.2f  (%+6.1f%% vs measured fx %.2f)   %s"
              % (cand, fx_cand, err, operative["fx"], label))
    print(
        "\nA fx that is too SMALL shrinks the corridor threshold (fx * robot_half_mm)\n"
        "and makes the robot ignore obstacles inside its real swept path."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
