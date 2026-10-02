#!/bin/bash
# =============================================================================
#  Sentinel Swarm — Drone Pi 5 Dependency Installer
#  Run this ONCE on the freshly flashed Pi 5 (hostname: pi3)
#
#  Usage:
#    chmod +x install_drone_deps.sh
#    bash install_drone_deps.sh
# =============================================================================

set -e

VENV="$HOME/drone_venv"
PIP_TMP="$HOME/pip_tmp"
PIP_CACHE="$HOME/pip_cache"

echo "=============================================="
echo " Sentinel Drone — Pi 5 Dependency Installer"
echo "=============================================="
echo ""

# ── System packages ──────────────────────────────────────────────────────────
echo "[1/5] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y \
    python3-venv python3-dev \
    libgl1 libglib2.0-0 \
    libatlas-base-dev \
    git curl

# ── Enable UART on Pi 5 ──────────────────────────────────────────────────────
echo ""
echo "[2/5] Configuring UART (GPIO14/15 → /dev/serial0)..."

# Disable serial console (frees UART for FC comms)
if grep -q "console=serial0" /boot/firmware/cmdline.txt; then
    sudo sed -i 's/console=serial0,[0-9]* //' /boot/firmware/cmdline.txt
    echo "  ✅ Serial console disabled"
else
    echo "  ✅ Serial console already disabled"
fi

# Enable UART in config.txt
if ! grep -q "enable_uart=1" /boot/firmware/config.txt; then
    echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt > /dev/null
    echo "  ✅ UART enabled in config.txt"
else
    echo "  ✅ UART already enabled"
fi

# Add user to dialout group (serial port access)
sudo usermod -aG dialout "$USER"
sudo usermod -aG video "$USER"
echo "  ✅ User added to dialout + video groups"

# ── Python virtual environment ───────────────────────────────────────────────
echo ""
echo "[3/5] Creating Python virtual environment at $VENV..."
mkdir -p "$PIP_TMP" "$PIP_CACHE"
python3 -m venv "$VENV" --system-site-packages
source "$VENV/bin/activate"

# Upgrade pip
pip install --quiet --upgrade pip \
    --cache-dir "$PIP_CACHE" --build "$PIP_TMP"

# ── Python packages ──────────────────────────────────────────────────────────
echo ""
echo "[4/5] Installing Python packages..."
pip install --quiet \
    --cache-dir "$PIP_CACHE" \
    --build "$PIP_TMP" \
    numpy \
    opencv-python-headless \
    pillow \
    ncnn \
    pyserial \
    pymavlink

echo ""
echo "  Installed packages:"
pip show numpy opencv-python-headless ncnn pyserial pymavlink \
    2>/dev/null | grep -E "^(Name|Version):" | paste - -

# ── Verify imports ───────────────────────────────────────────────────────────
echo ""
echo "[5/5] Verifying all imports..."

python3 - <<'PYEOF'
import sys
errors = 0

checks = [
    ("numpy",   "import numpy; print(f'  ✅ numpy {numpy.__version__}')"),
    ("cv2",     "import cv2; print(f'  ✅ opencv {cv2.__version__}')"),
    ("ncnn",    "import ncnn; print(f'  ✅ ncnn OK')"),
    ("serial",  "import serial; print(f'  ✅ pyserial {serial.__version__}')"),
    ("pymavlink","from pymavlink import mavutil; print('  ✅ pymavlink OK')"),
]

for name, stmt in checks:
    try:
        exec(stmt)
    except Exception as e:
        print(f'  ❌ {name}: {e}')
        errors += 1

if errors == 0:
    print("")
    print("  All checks passed! ✅")
else:
    print(f"\n  {errors} check(s) failed ❌")
    sys.exit(1)
PYEOF

echo ""
echo "=============================================="
echo " Installation complete!"
echo ""
echo " IMPORTANT: Reboot the Pi 5 now so UART"
echo " and group changes take effect:"
echo ""
echo "   sudo reboot"
echo ""
echo " After reboot, deploy the YOLO model:"
echo "   (run export_yolo_ncnn.py from Windows)"
echo ""
echo " Then run the drone:"
echo "   source ~/drone_venv/bin/activate"
echo "   python3 ~/drone_companion.py"
echo "=============================================="
