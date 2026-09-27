# =============================================================================
# Phase 0 — Quick verify (run on Pi AFTER install_deps.sh completes)
# ssh pi@pi.local "bash -s" < verify_install.sh
# =============================================================================

set -euo pipefail

BOLD="\033[1m"
GREEN="\033[0;32m"
RED="\033[0;31m"
RESET="\033[0m"

echo ""
echo -e "${BOLD}=== Swarm POC — Install Verification ===${RESET}"

source "$HOME/swarm_venv/bin/activate"

python3 - <<'EOF'
import sys
results = []

# OpenCV
try:
    import cv2
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        ret, frame = cap.read()
        cap.release()
        if ret:
            results.append(("PASS", f"opencv + webcam: {frame.shape[1]}x{frame.shape[0]} frame captured"))
        else:
            results.append(("WARN", "opencv OK but could not grab frame — webcam may be busy"))
    else:
        results.append(("WARN", "opencv OK but VideoCapture(0) failed — check webcam connection"))
except ImportError:
    results.append(("FAIL", "opencv not installed"))

# YOLO11n NCNN & model
try:
    import ncnn
    import os
    model_path = os.path.expanduser("~/yolo11n_ncnn_model")
    param_file = os.path.join(model_path, "model.ncnn.param")
    bin_file   = os.path.join(model_path, "model.ncnn.bin")
    if os.path.isfile(param_file) and os.path.isfile(bin_file):
        results.append(("PASS", f"ncnn OK and YOLO11n NCNN model found at {model_path}"))
    elif os.path.isdir(model_path):
        results.append(("WARN", f"NCNN model dir exists but missing param/bin at {model_path}"))
    else:
        results.append(("WARN", f"NCNN model not found at {model_path} — export and copy from Windows"))
except ImportError:
    results.append(("FAIL", "ncnn not installed"))

# pyserial
try:
    import serial
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if ports:
        port_names = [p.device for p in ports]
        results.append(("PASS", f"pyserial OK, serial ports: {', '.join(port_names)}"))
    else:
        results.append(("WARN", "pyserial OK but no serial ports detected — is Arduino connected?"))
except ImportError:
    results.append(("FAIL", "pyserial not installed"))

# digi-xbee
try:
    import digi.xbee
    results.append(("PASS", f"digi-xbee OK: v{digi.xbee.__version__}"))
except ImportError:
    results.append(("FAIL", "digi-xbee not installed"))

# Print results
print()
for status, msg in results:
    if status == "PASS":
        colour = "\033[0;32m"
    elif status == "WARN":
        colour = "\033[1;33m"
    else:
        colour = "\033[0;31m"
    print(f"  [{colour}{status}\033[0m] {msg}")

print()
fails = [r for r in results if r[0] == "FAIL"]
if fails:
    print("  Some checks FAILED. Re-run install_deps.sh.")
    sys.exit(1)
else:
    print("  All checks passed. Ready for Phase 1.")
EOF
