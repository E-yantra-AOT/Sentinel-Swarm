"""
Phase 0 — Export YOLO11n to NCNN INT8 format on Windows PC.

Run this ONCE on your Windows PC (not on the Pi).
It downloads yolo11n.pt (~6MB) and exports it to NCNN format.
The exported folder is then copied to both Pis automatically.

Usage:
    pip install ultralytics          # on Windows if not already installed
    python phase0_setup/export_yolo_ncnn.py

Output:
    C:\\Users\\ASUS\\Code\\Swarm\\phase0_setup\\yolo11n_ncnn_model\\
        yolo11n.ncnn.param
        yolo11n.ncnn.bin
        metadata.yaml
"""

import subprocess
import sys
import os
from pathlib import Path

EXPORT_DIR = Path(__file__).parent / "yolo11n_ncnn_model"
SCRIPT_DIR = Path(__file__).parent

def install_if_missing(package):
    try:
        __import__(package)
    except ImportError:
        print(f"Installing {package}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", package, "-q"])

def main():
    print()
    print("=" * 54)
    print("  Swarm POC — Export YOLO11n → NCNN INT8 (Windows)")
    print("=" * 54)

    install_if_missing("ultralytics")

    from ultralytics import YOLO

    if EXPORT_DIR.exists():
        print(f"\n[SKIP] NCNN model already exists: {EXPORT_DIR}")
        print("       Delete the folder and re-run to force re-export.")
    else:
        print("\n[1/2] Downloading yolo11n.pt and exporting to NCNN INT8...")
        print("      (This takes ~2-3 minutes the first time)")
        model = YOLO("yolo11n.pt")
        exported = model.export(
            format="ncnn",
            imgsz=320,
            int8=True,
            half=False,
        )
        print(f"[1/2] Export done: {exported}")

        # Move to a consistent location in our project
        import shutil
        if str(exported) != str(EXPORT_DIR):
            if EXPORT_DIR.exists():
                shutil.rmtree(EXPORT_DIR)
            shutil.move(str(exported), str(EXPORT_DIR))

    print(f"\n[2/2] Copying NCNN model to both Pis...")

    for user_host, label in [("pi@10.14.206.74", "Robot A"), ("pi2@10.14.206.184", "Robot B")]:
        print(f"      Copying to {label} ({user_host})...")
        result = subprocess.run(
            ["scp", "-r", "-o", "StrictHostKeyChecking=no",
             str(EXPORT_DIR),
             f"{user_host}:~/yolo11n_ncnn_model"],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"      [{label}] ✓ Copied successfully")
        else:
            print(f"      [{label}] ✗ SCP failed: {result.stderr.strip()}")
            print(f"              Run manually: scp -r {EXPORT_DIR} {user_host}:~/yolo11n_ncnn_model")

    print()
    print("=" * 54)
    print("  Done! NCNN model is on both Pis at ~/yolo11n_ncnn_model/")
    print("=" * 54)
    print()

if __name__ == "__main__":
    main()
