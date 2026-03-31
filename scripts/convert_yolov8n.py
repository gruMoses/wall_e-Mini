#!/usr/bin/env python3
"""
Convert YOLOv8n to a DepthAI-compatible .blob file for OAK-D Lite (RVC2/Myriad X).

Run this script ON THE PI (or any Linux x86/arm machine with internet access).
The resulting blob should be copied to ~/wall_e-Mini/models/yolov8n_640x352.blob,
then update config.py:

    oak_detection: OakDetectionConfig = OakDetectionConfig(
        model_type="yolov8n",
        model_path="models/yolov8n_640x352.blob",
    )

Requirements (install once):
    pip install ultralytics blobconverter --break-system-packages

Usage:
    python3 scripts/convert_yolov8n.py [--width 640] [--height 352] [--shaves 6] [--out models/yolov8n_640x352.blob]
"""
import argparse
import shutil
import sys
from pathlib import Path


def check_deps() -> None:
    missing = []
    for pkg in ("ultralytics", "blobconverter"):
        try:
            __import__(pkg)
        except ImportError:
            missing.append(pkg)
    if missing:
        print(f"Missing packages: {missing}")
        print("Install with:  pip install " + " ".join(missing) + " --break-system-packages")
        sys.exit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert YOLOv8n to OAK-D .blob")
    parser.add_argument("--width", type=int, default=640, help="Input width (default 640, must be divisible by 32)")
    parser.add_argument("--height", type=int, default=352, help="Input height (default 352, must be divisible by 32)")
    parser.add_argument("--shaves", type=int, default=5, help="MyriadX shaves (default 5)")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output blob path (default models/yolov8n_WxH.blob)",
    )
    args = parser.parse_args()

    if args.width % 32 != 0 or args.height % 32 != 0:
        print(f"ERROR: width ({args.width}) and height ({args.height}) must both be divisible by 32")
        sys.exit(1)

    out_path: Path = args.out or Path(f"models/yolov8n_{args.width}x{args.height}.blob")

    check_deps()
    from ultralytics import YOLO  # type: ignore
    import blobconverter  # type: ignore

    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Clear blobconverter cache to prevent stale cached blobs (blobconverter caches by param
    # hash, so old blobs without normalization flags would be returned if cache is not cleared).
    _bc_cache_dirs = [
        Path.home() / ".cache" / "blobconverter",
        Path("/tmp/blobconverter"),
    ]
    for _d in _bc_cache_dirs:
        if _d.exists():
            shutil.rmtree(_d, ignore_errors=True)
            print(f"Cleared blobconverter cache: {_d}")

    # Step 1: Export YOLOv8n to ONNX (opset 11 — required for blobconverter/OpenVINO 2022.1 compat)
    # Note: from_openvino path fails because ultralytics 8.4+ exports opset14 IR which the
    # Luxonis compile server (OpenVINO 2022.1) rejects. Exporting to ONNX with opset=11 and
    # letting blobconverter.from_onnx handle the IR conversion server-side is the workaround.
    # ultralytics imgsz format for non-square: [height, width]
    onnx_path = Path("yolov8n.onnx")
    print(f"[1/3] Exporting YOLOv8n to ONNX (imgsz=[{args.height}, {args.width}], opset=11) ...")
    model = YOLO("yolov8n.pt")
    model.export(format="onnx", imgsz=[args.height, args.width], opset=11)
    if not onnx_path.exists():
        candidates = list(Path(".").glob("yolov8n*.onnx"))
        if not candidates:
            print("ERROR: could not find exported yolov8n.onnx")
            sys.exit(1)
        onnx_path = candidates[0]
    print(f"      ONNX written to {onnx_path}")

    # Step 2: Convert ONNX → .blob via blobconverter (server converts ONNX→IR→blob)
    # Bake input normalisation into the blob so the camera can feed raw [0,255] uint8 BGR pixels:
    #   --scale_values      divides each channel by 255, mapping [0,255] → [0,1] as YOLO expects
    #   --mean_values       zero mean (no shift needed for YOLO; explicit to avoid server defaults)
    #   --reverse_input_channels  converts BGR (OAK-D native) to RGB (YOLO training convention)
    #   --layout=NCHW       matches the ONNX export layout so OpenVINO parses shapes correctly
    # Without these, the model receives raw uint8 values; all sigmoid activations saturate and
    # produce zero real detections regardless of threshold.
    print(f"[2/3] Converting to .blob (shaves={args.shaves}) ...")
    blob_path = blobconverter.from_onnx(
        model=str(onnx_path),
        data_type="FP16",
        shaves=args.shaves,
        optimizer_params=[
            "--scale_values=[255,255,255]",
            "--reverse_input_channels",
            "--mean_values=[0,0,0]",
            "--layout=NCHW",
        ],
    )
    print(f"      Blob downloaded to: {blob_path}")

    # Step 3: Copy to destination
    print(f"[3/3] Copying to {out_path} ...")
    shutil.copy(blob_path, out_path)
    print(f"\nDone. Blob at: {out_path}")
    print("\nNext steps:")
    print("  1. Edit config.py OakDetectionConfig:")
    print(f'        model_type="yolov8n", model_path="{out_path}"')
    print("  2. sudo systemctl restart wall-e")


if __name__ == "__main__":
    main()
