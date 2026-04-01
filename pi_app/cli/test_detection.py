#!/usr/bin/env python3
"""
Standalone YOLOv8n detection validation for OAK-D Lite.

Runs the same detection pipeline as the production system and reports
whether detections are working. Run this before testing follow-me.

Usage:
    python pi_app/cli/test_detection.py
    python pi_app/cli/test_detection.py --duration 30
    python pi_app/cli/test_detection.py --save-frames
    python pi_app/cli/test_detection.py --duration 20 --save-frames --output-dir /tmp/frames
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from collections import Counter, defaultdict
from pathlib import Path

# Resolve project root (wall_e-Mini/) from wherever this file lives
_HERE = Path(__file__).resolve()
_REPO_ROOT = _HERE.parents[2]  # pi_app/cli/../../  →  wall_e-Mini/
sys.path.insert(0, str(_REPO_ROOT))

from config import config  # noqa: E402

# Detection rate threshold to PASS
PASS_THRESHOLD_PCT = 50.0
# Max annotated JPEG frames to save (--save-frames)
_SAVE_FRAMES_MAX = 10


# ---------------------------------------------------------------------------
# Camera presence check
# ---------------------------------------------------------------------------

def _camera_present() -> bool:
    """Return True if an OAK-D device is visible on USB."""
    try:
        import depthai as dai
        if hasattr(dai.Device, "getAllAvailableDevices"):
            return len(dai.Device.getAllAvailableDevices()) > 0
        return len(dai.DeviceDiscovery.getAllAvailableDevices()) > 0
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Pipeline construction
# ---------------------------------------------------------------------------

def _build_pipeline(det_cfg, obs_cfg, fm_cfg, blob_path: Path, use_yolo: bool, save_frames: bool):
    """Build and return (pipeline, det_q, rgb_q).

    rgb_q is None if save_frames is False or OpenCV is unavailable.
    """
    import depthai as dai

    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setOutputSize(640, 400)
    if use_yolo:
        # Extended disparity halves minimum detectable range (~0.35 m vs ~0.7 m)
        stereo.setExtendedDisparity(True)
        # Align depth to RGB so bounding-box spatial positions are correct
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    mono_left.requestOutput((640, 400)).link(stereo.left)
    mono_right.requestOutput((640, 400)).link(stereo.right)

    if use_yolo:
        spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork)
        spatial_nn.setBlobPath(str(blob_path))

        # depthai v3: detectionParser defaults leave NumClasses=0 / CoordinateSize=0 /
        # IouThreshold=0.0, which silently produces zero detections for YOLOv8.
        # These three lines are mandatory.
        dp = spatial_nn.detectionParser
        dp.setNumClasses(80)
        dp.setCoordinateSize(4)
        dp.setAnchors([])        # anchor-free model
        dp.setAnchorMasks({})    # anchor-free model
        dp.setSubtype("yolov8")  # YOLOv8 output is [1,84,4620] not [1,4620,85]; without this the parser misreads the tensor and yields 0 detections
        dp.setIouThreshold(det_cfg.nms_threshold)

        # BGR888p — OAK-D native format. The blob is compiled with
        # --reverse_input_channels + --scale_values=[255,255,255] baked in
        # (see scripts/convert_yolov8n.py), so the MyriadX performs BGR→RGB
        # and [0,255]→[0,1] normalisation internally before inference.
        cam_nn_out = cam_rgb.requestOutput(
            (det_cfg.input_width, det_cfg.input_height),
            dai.ImgFrame.Type.BGR888p,
        )
        cam_nn_out.link(spatial_nn.input)
        stereo.depth.link(spatial_nn.inputDepth)
    else:
        model_desc = dai.NNModelDescription(
            model="luxonis/mobilenet-ssd:300x300", platform="RVC2"
        )
        spatial_nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
            cam_rgb, stereo, model_desc,
        )

    spatial_nn.setConfidenceThreshold(det_cfg.confidence_threshold)
    for _setter in ("setNMSThreshold", "setNmsThreshold", "setIouThreshold"):
        if hasattr(spatial_nn, _setter):
            getattr(spatial_nn, _setter)(det_cfg.nms_threshold)
            break
    spatial_nn.setNumInferenceThreads(1)
    spatial_nn.setNumShavesPerInferenceThread(4)
    spatial_nn.input.setBlocking(False)
    spatial_nn.setBoundingBoxScaleFactor(0.5)
    spatial_nn.setDepthLowerThreshold(int(fm_cfg.min_distance_m * 1000))
    spatial_nn.setDepthUpperThreshold(int(fm_cfg.max_distance_m * 1000))

    det_q = spatial_nn.out.createOutputQueue(maxSize=4, blocking=False)

    # Optional RGB preview for annotated frame saving (separate from NN input)
    rgb_q = None
    if save_frames:
        try:
            import cv2 as _  # noqa: F401 — check available before wiring
            # Request a different resolution so depthai v3 doesn't de-duplicate
            # the requestOutput() call and hand us the NN-only internal output.
            rgb_preview = cam_rgb.requestOutput((640, 480))
            rgb_q = rgb_preview.createOutputQueue(maxSize=2, blocking=False)
        except ImportError:
            print("WARNING: opencv-python not installed — --save-frames disabled.")
        except Exception as e:
            print(f"WARNING: Could not create RGB preview queue: {e}")

    return pipeline, det_q, rgb_q


# ---------------------------------------------------------------------------
# Detection loop
# ---------------------------------------------------------------------------

def _resolve_label(det, use_yolo: bool, det_cfg) -> str:
    name = getattr(det, "labelName", None)
    if name:
        return name
    label = int(getattr(det, "label", -1))
    if use_yolo:
        classes = det_cfg.coco_classes
        return classes[label] if 0 <= label < len(classes) else str(label)
    return str(label)


def run_detection_test(duration_s: float, save_frames: bool, output_dir: Path) -> None:
    try:
        import depthai as dai  # noqa: F401
        import numpy as np     # noqa: F401
    except ImportError as e:
        print(f"\nFATAL: Missing dependency — {e}")
        print("Install: pip install depthai numpy")
        sys.exit(1)

    det_cfg = config.oak_detection
    obs_cfg = config.obstacle_avoidance
    fm_cfg = config.follow_me

    blob_path = (_REPO_ROOT / det_cfg.model_path).resolve()
    use_yolo = det_cfg.model_type == "yolov8n" and blob_path.exists()

    if det_cfg.model_type == "yolov8n" and not blob_path.exists():
        print(f"WARNING: YOLOv8n blob not found: {blob_path}")
        print("         Run: python scripts/convert_yolov8n.py")
        print("         Falling back to MobileNet-SSD (fewer classes).\n")

    model_label = "YOLOv8n (80 COCO classes)" if use_yolo else "MobileNet-SSD (21 VOC classes)"
    print(f"Building pipeline — {model_label}...")

    try:
        pipeline, det_q, rgb_q = _build_pipeline(
            det_cfg, obs_cfg, fm_cfg, blob_path, use_yolo, save_frames
        )
    except Exception as e:
        print(f"\nFATAL: Pipeline construction failed — {e}")
        traceback.print_exc()
        sys.exit(1)

    print("Starting pipeline...")
    try:
        pipeline.start()
    except Exception as e:
        print(f"\nFATAL: Pipeline start failed — {e}")
        traceback.print_exc()
        sys.exit(1)

    print("Warming up (2 s)...\n")
    time.sleep(2.0)

    print(f"Running for {duration_s:.0f}s — point camera at objects / people...\n")
    print(f"{'Frame':>6}  {'Dets':>4}  {'Detections (class @ conf  z_m)'}")
    print("─" * 70)

    total_frames = 0
    frames_with_dets = 0
    # (label_name, confidence, x_m, z_m, bbox_tuple)
    all_dets: list[tuple[str, float, float, float, tuple]] = []
    saved_count = 0

    end_time = time.monotonic() + duration_s

    while time.monotonic() < end_time:
        msg = det_q.tryGet()
        if msg is None:
            time.sleep(0.02)
            continue

        # Drain queue — use newest frame only
        while True:
            newer = det_q.tryGet()
            if newer is None:
                break
            msg = newer

        total_frames += 1
        frame_dets: list[tuple[str, float, float, float, tuple]] = []

        for det in getattr(msg, "detections", []):
            label_name = _resolve_label(det, use_yolo, det_cfg)
            conf = float(getattr(det, "confidence", 0.0))
            spatial = getattr(det, "spatialCoordinates", None)
            x_m = float(getattr(spatial, "x", 0.0)) / 1000.0 if spatial else 0.0
            z_m = float(getattr(spatial, "z", 0.0)) / 1000.0 if spatial else 0.0
            bbox = (
                float(getattr(det, "xmin", 0.0)),
                float(getattr(det, "ymin", 0.0)),
                float(getattr(det, "xmax", 0.0)),
                float(getattr(det, "ymax", 0.0)),
            )
            frame_dets.append((label_name, conf, x_m, z_m, bbox))
            all_dets.append((label_name, conf, x_m, z_m, bbox))

        if frame_dets:
            frames_with_dets += 1
            det_str = "  ".join(
                f"{n}@{c:.0%}({z:.1f}m)" for n, c, _x, z, _ in frame_dets[:6]
            )
            print(f"{total_frames:>6}  {len(frame_dets):>4}  {det_str}")
        else:
            # Print a "no detection" heartbeat every 10 frames to show it's alive
            if total_frames % 10 == 0:
                print(f"{total_frames:>6}  {0:>4}  (no detections)")

        # Optionally save annotated frame
        if (
            save_frames
            and rgb_q is not None
            and saved_count < _SAVE_FRAMES_MAX
            and frame_dets
        ):
            rgb_msg = rgb_q.tryGet()
            if rgb_msg is not None:
                _save_annotated_frame(
                    rgb_msg, frame_dets, output_dir, total_frames, saved_count
                )
                saved_count += 1

    try:
        pipeline.stop()
    except Exception:
        pass

    print("─" * 70)
    _print_summary(
        total_frames, frames_with_dets, all_dets, use_yolo,
        saved_count, output_dir if save_frames else None,
    )


# ---------------------------------------------------------------------------
# Frame saving
# ---------------------------------------------------------------------------

def _save_annotated_frame(
    rgb_msg,
    frame_dets: list,
    output_dir: Path,
    frame_idx: int,
    saved_count: int,
) -> None:
    try:
        import cv2
        frame = rgb_msg.getCvFrame()
        if frame is None:
            return
        h, w = frame.shape[:2]
        for label_name, conf, x_m, z_m, (xmin, ymin, xmax, ymax) in frame_dets:
            x1, y1 = int(xmin * w), int(ymin * h)
            x2, y2 = int(xmax * w), int(ymax * h)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
            text = f"{label_name} {conf:.0%} z={z_m:.1f}m"
            cv2.putText(
                frame, text, (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA,
            )
        out_path = output_dir / f"frame_{frame_idx:04d}.jpg"
        cv2.imwrite(str(out_path), frame)
        print(f"         -> saved {out_path.name}")
    except Exception as e:
        print(f"         -> frame save error: {e}")


# ---------------------------------------------------------------------------
# Summary + verdict
# ---------------------------------------------------------------------------

def _print_summary(
    total_frames: int,
    frames_with_dets: int,
    all_dets: list,
    use_yolo: bool,
    saved_frames: int,
    output_dir: Path | None,
) -> None:
    print()
    print("=" * 60)
    print("  DETECTION TEST SUMMARY")
    print("=" * 60)

    if total_frames == 0:
        print("  ERROR: No frames received from the pipeline.")
        print("         Check depthai installation and USB connection.")
        _print_verdict(False)
        return

    detection_rate = (frames_with_dets / total_frames) * 100.0
    avg_per_frame = len(all_dets) / max(1, frames_with_dets)

    print(f"  Model             : {'YOLOv8n' if use_yolo else 'MobileNet-SSD'}")
    print(f"  Frames processed  : {total_frames}")
    print(f"  Frames w/ dets    : {frames_with_dets}  ({detection_rate:.1f}%)")
    print(f"  Frames w/o dets   : {total_frames - frames_with_dets}")
    print(f"  Total detections  : {len(all_dets)}")
    print(f"  Avg dets/frame*   : {avg_per_frame:.1f}  (* on frames with any detection)")
    print(f"  Detection rate    : {detection_rate:.1f}%")

    if all_dets:
        class_counts: Counter = Counter(n for n, *_ in all_dets)
        class_confs: dict[str, list] = defaultdict(list)
        for n, c, *_ in all_dets:
            class_confs[n].append(c)

        print()
        print("  Top detected classes:")
        for cls, count in class_counts.most_common(10):
            avg_c = sum(class_confs[cls]) / len(class_confs[cls])
            bar = "█" * int(avg_c * 20)
            print(f"    {cls:<22} {count:>5}x  conf {avg_c:.0%}  {bar}")

        overall_avg = sum(c for _, c, *_ in all_dets) / len(all_dets)
        print()
        print(f"  Overall avg confidence : {overall_avg:.0%}")

    if output_dir is not None and saved_frames > 0:
        print()
        print(f"  Annotated frames saved : {saved_frames} → {output_dir}/")

    passed = detection_rate > PASS_THRESHOLD_PCT
    _print_verdict(passed)

    if not passed and total_frames > 0:
        print()
        print(f"  Hint: {detection_rate:.1f}% < {PASS_THRESHOLD_PCT:.0f}% pass threshold.")
        print("  Tips:")
        print("    - Point the camera at a person or common objects and re-run.")
        print("    - Try indoors with good lighting.")
        if not use_yolo:
            print("    - Generate the YOLOv8n blob for better accuracy:")
            print("        python scripts/convert_yolov8n.py")
        print()


def _print_verdict(passed: bool) -> None:
    print()
    if passed:
        print("  ╔══════════════════════════════════╗")
        print("  ║   PASS — detection pipeline OK   ║")
        print("  ╚══════════════════════════════════╝")
    else:
        print("  ╔══════════════════════════════════╗")
        print("  ║   FAIL — check camera / blob     ║")
        print("  ╚══════════════════════════════════╝")
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate YOLOv8n detection pipeline on OAK-D Lite.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python pi_app/cli/test_detection.py
  python pi_app/cli/test_detection.py --duration 30
  python pi_app/cli/test_detection.py --save-frames
  python pi_app/cli/test_detection.py --duration 20 --save-frames --output-dir /tmp/det
        """,
    )
    parser.add_argument(
        "--duration", type=float, default=10.0,
        help="Seconds to run the detection loop (default: 10)",
    )
    parser.add_argument(
        "--save-frames", action="store_true",
        help="Save annotated JPEG frames for visual verification (requires opencv-python)",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("detection_test_frames"),
        help="Directory for saved frames (default: detection_test_frames/)",
    )
    args = parser.parse_args()

    det_cfg = config.oak_detection
    blob_path = (_REPO_ROOT / det_cfg.model_path).resolve()

    print()
    print("=" * 60)
    print("  WALL-E OAK-D LITE — DETECTION PIPELINE VALIDATION")
    print("=" * 60)
    print(f"  Model type  : {det_cfg.model_type}")
    print(f"  Blob path   : {blob_path}  {'[found]' if blob_path.exists() else '[MISSING]'}")
    print(f"  Conf thr    : {det_cfg.confidence_threshold}")
    print(f"  NMS thr     : {det_cfg.nms_threshold}")
    print(f"  Input res   : {det_cfg.input_width}×{det_cfg.input_height}")
    print(f"  Duration    : {args.duration:.0f}s")
    print(f"  Save frames : {args.save_frames}")
    print()

    print("Checking for OAK-D Lite on USB...")
    if not _camera_present():
        print("\nERROR: No OAK-D device found.")
        print("  - Check USB 3.0 cable and port")
        print("  - Run: lsusb | grep 03e7")
        print("  - Install udev rules if needed:")
        print("      echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' \\")
        print("        | sudo tee /etc/udev/rules.d/80-movidius.rules")
        print("      sudo udevadm control --reload-rules && sudo udevadm trigger")
        sys.exit(1)
    print("OAK-D Lite detected.\n")

    if args.save_frames:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    try:
        run_detection_test(args.duration, args.save_frames, args.output_dir)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(0)
    except Exception as e:
        print(f"\nFATAL: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
