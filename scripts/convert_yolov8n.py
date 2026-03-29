#!/usr/bin/env python3
"""
Convert YOLOv8n to a DepthAI-compatible .blob file for OAK-D Lite (RVC2/Myriad X).

Run this script ON THE PI (or any Linux x86/arm machine with internet access).
The resulting blob should be copied to ~/wall_e-Mini/models/yolov8n_416.blob,
then update config.py:

    oak_detection: OakDetectionConfig = OakDetectionConfig(
        model_type="yolov8n",
        model_path="models/yolov8n_416.blob",
    )

Requirements (install once):
    pip install ultralytics blobconverter --break-system-packages

Usage:
    python3 scripts/convert_yolov8n.py [--imgsz 416] [--shaves 6] [--out models/yolov8n_416.blob]
"""
import argparse
import shutil
import subprocess
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
    parser.add_argument("--imgsz", type=int, default=416, help="Input resolution (default 416)")
    parser.add_argument("--shaves", type=int, default=6, help="MyriadX shaves (default 6)")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("models/yolov8n_416.blob"),
        help="Output blob path (default models/yolov8n_416.blob)",
    )
    args = parser.parse_args()

    check_deps()
    from ultralytics import YOLO  # type: ignore
    import blobconverter  # type: ignore

    out_path: Path = args.out
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Step 1: Export YOLOv8n to ONNX (opset 11 — required for blobconverter/OpenVINO 2022.1 compat)
    # Note: from_openvino path fails because ultralytics 8.4+ exports opset14 IR which the
    # Luxonis compile server (OpenVINO 2022.1) rejects. Exporting to ONNX with opset=11 and
    # letting blobconverter.from_onnx handle the IR conversion server-side is the workaround.
    onnx_path = Path("yolov8n.onnx")
    print(f"[1/3] Exporting YOLOv8n to ONNX (imgsz={args.imgsz}, opset=11) ...")
    model = YOLO("yolov8n.pt")
    model.export(format="onnx", imgsz=args.imgsz, opset=11)
    if not onnx_path.exists():
        candidates = list(Path(".").glob("yolov8n*.onnx"))
        if not candidates:
            print("ERROR: could not find exported yolov8n.onnx")
            sys.exit(1)
        onnx_path = candidates[0]
    print(f"      ONNX written to {onnx_path}")

    # Step 2: Convert ONNX → .blob via blobconverter (server converts ONNX→IR→blob)
    print(f"[2/3] Converting to .blob (shaves={args.shaves}) ...")
    blob_path = blobconverter.from_onnx(
        model=str(onnx_path),
        data_type="FP16",
        shaves=args.shaves,
        reverse_input_channels=True,  # OAK-D sends BGR888p; blob expects RGB input
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
