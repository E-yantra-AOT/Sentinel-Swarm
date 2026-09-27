#!/usr/bin/env bash
# =============================================================================
# Phase 0: Dependency Installer (v4 — FINAL, no ultralytics on Pi)
#
# ultralytics requires torch even at import time — not viable on Pi.
# We use ncnn Python bindings directly with a custom thin inference wrapper.
# Packages installed: opencv, ncnn, numpy, pillow, pyserial, digi-xbee
# =============================================================================

set -euo pipefail

GREEN="\033[0;32m"; BOLD="\033[1m"; RESET="\033[0m"

VENV="$HOME/swarm_venv"
PIP_TMP="$HOME/pip_tmp"
PIP_CACHE="$HOME/pip_cache"

export TMPDIR="$PIP_TMP"
mkdir -p "$PIP_TMP" "$PIP_CACHE"

PIP="$VENV/bin/pip"
PY="$VENV/bin/python3"

echo -e "\n${BOLD}=== Swarm POC — Installer v4 (no-torch, no-ultralytics) ===${RESET}"
echo -e "  Host  : $(hostname)\n"

# ── 1. System packages ───────────────────────────────────────────────
echo -e "${BOLD}[1/4] System packages...${RESET}"
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-venv python3-dev libgl1 libglib2.0-0 git 2>&1 | \
    grep -E "^(Get|Inst|Setting|done|0 upgraded)" || true
echo -e "${GREEN}  done${RESET}"

# ── 2. Fresh venv ────────────────────────────────────────────────────
echo -e "\n${BOLD}[2/4] Python venv...${RESET}"
rm -rf "$VENV"
python3 -m venv "$VENV" --system-site-packages
TMPDIR="$PIP_TMP" "$PIP" install --cache-dir "$PIP_CACHE" --upgrade pip --quiet
echo -e "${GREEN}  venv ready — pip $("$PIP" --version | awk '{print $2}')${RESET}"

# ── 3. Install packages (no torch, no ultralytics) ───────────────────
echo -e "\n${BOLD}[3/4] Installing packages...${RESET}"

TMPDIR="$PIP_TMP" "$PIP" install --cache-dir "$PIP_CACHE" \
    "numpy>=1.23" \
    "opencv-python-headless" \
    "pillow>=9.0" \
    --quiet
echo "  → numpy + opencv installed"

TMPDIR="$PIP_TMP" "$PIP" install --cache-dir "$PIP_CACHE" \
    ncnn \
    --quiet
echo "  → ncnn installed"

TMPDIR="$PIP_TMP" "$PIP" install --cache-dir "$PIP_CACHE" \
    pyserial \
    --quiet
echo "  → pyserial installed"

TMPDIR="$PIP_TMP" "$PIP" install --cache-dir "$PIP_CACHE" \
    digi-xbee \
    --quiet
echo "  → digi-xbee installed"

echo -e "${GREEN}  all packages installed${RESET}"

# ── 4. Verify ────────────────────────────────────────────────────────
echo -e "\n${BOLD}[4/4] Verifying...${RESET}"
"$PY" - <<'PYCHECK'
import sys
ok = True
checks = [
    ("numpy",     "numpy"),
    ("cv2",       "opencv-python-headless"),
    ("ncnn",      "ncnn"),
    ("PIL",       "pillow"),
    ("serial",    "pyserial"),
    ("digi.xbee", "digi-xbee"),
]
for mod, label in checks:
    try:
        m = __import__(mod)
        ver = getattr(m, '__version__', '')
        print(f"  [\033[0;32mPASS\033[0m] {label:30s} {ver}")
    except ImportError as e:
        print(f"  [\033[0;31mFAIL\033[0m] {label}: {e}")
        ok = False
if not ok:
    sys.exit(1)
print("\n  All imports OK.")
print("  Note: YOLO inference uses custom ncnn wrapper (no torch needed).")
PYCHECK

echo -e "\n${GREEN}${BOLD}=== Installation complete! ===${RESET}"
echo "  Activate venv : source $VENV/bin/activate"
echo "  Next          : NCNN model will be copied from Windows PC automatically."
