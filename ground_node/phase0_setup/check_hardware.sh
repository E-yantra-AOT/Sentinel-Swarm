#!/usr/bin/env bash
# =============================================================================
# Phase 0: Hardware Check Script
# Run on EACH Raspberry Pi via SSH:
#   ssh pi@pi.local  "bash -s" < check_hardware.sh
#   ssh pi2@pi2.local "bash -s" < check_hardware.sh
# =============================================================================

set -euo pipefail

BOLD="\033[1m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
RESET="\033[0m"

PASS="${GREEN}[PASS]${RESET}"
WARN="${YELLOW}[WARN]${RESET}"
FAIL="${RED}[FAIL]${RESET}"

echo ""
echo -e "${BOLD}======================================================${RESET}"
echo -e "${BOLD}  Swarm POC — Phase 0 Hardware Check${RESET}"
echo -e "${BOLD}======================================================${RESET}"

# -------------------------------------------------------------------
# 1. System Info
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}[1/7] System Information${RESET}"
echo "  Hostname   : $(hostname)"
echo "  IP Address : $(hostname -I | awk '{print $1}')"
echo "  OS         : $(cat /etc/os-release | grep PRETTY_NAME | cut -d= -f2 | tr -d '\"')"
echo "  Kernel     : $(uname -r)"
echo "  Arch       : $(uname -m)"

# -------------------------------------------------------------------
# 2. Python
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}[2/7] Python Environment${RESET}"

if command -v python3 &>/dev/null; then
    PYVER=$(python3 --version)
    echo -e "  $PASS python3 found: $PYVER"
else
    echo -e "  $FAIL python3 NOT found — run: sudo apt install python3"
fi

if command -v pip3 &>/dev/null; then
    echo -e "  $PASS pip3 found: $(pip3 --version | awk '{print $2}')"
else
    echo -e "  $WARN pip3 not found — will be installed by install_deps.sh"
fi

# -------------------------------------------------------------------
# 3. USB Devices (Arduino + XBee)
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}[3/7] USB Devices (lsusb)${RESET}"

LSUSB_OUT=$(lsusb)

# Arduino (CH340 = 1a86:7523, ATmega16U2 = 2341:0001, CH341 = 1a86:7522)
if echo "$LSUSB_OUT" | grep -qiE "2341|1a86|0403:6001"; then
    ARD_LINE=$(echo "$LSUSB_OUT" | grep -iE "2341|1a86|0403:6001" | head -1)
    echo -e "  $PASS Arduino detected    : $ARD_LINE"
else
    echo -e "  $FAIL Arduino NOT detected — check USB Type-A to Type-B cable"
fi

# XBee CP2102 (Silicon Labs VID 10c4)
if echo "$LSUSB_OUT" | grep -qi "10c4"; then
    XB_LINE=$(echo "$LSUSB_OUT" | grep -i "10c4" | head -1)
    echo -e "  $PASS XBee (CP2102) found  : $XB_LINE"
else
    echo -e "  $FAIL XBee CP2102 NOT detected — check XBee USB carrier board"
fi

# USB Webcam (common VIDs: Logitech=046d, Microsoft=045e, Generic=0ac8 etc.)
if echo "$LSUSB_OUT" | grep -qiE "046d|045e|0ac8|1e4e|1908|05a9"; then
    CAM_LINE=$(echo "$LSUSB_OUT" | grep -iE "046d|045e|0ac8|1e4e|1908|05a9" | head -1)
    echo -e "  $PASS USB Webcam found     : $CAM_LINE"
else
    # Fall back: check /dev/video* directly
    if ls /dev/video* &>/dev/null; then
        echo -e "  $PASS USB Webcam found via /dev/video*"
    else
        echo -e "  $FAIL USB Webcam NOT detected — check USB connection"
    fi
fi

# -------------------------------------------------------------------
# 4. Serial Device Paths
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}[4/7] Serial Device Paths${RESET}"

TTY_FOUND=false

if ls /dev/ttyACM* &>/dev/null 2>&1; then
    for dev in /dev/ttyACM*; do
        echo -e "  $PASS Found: $dev  (likely Arduino)"
        TTY_FOUND=true
    done
fi

if ls /dev/ttyUSB* &>/dev/null 2>&1; then
    for dev in /dev/ttyUSB*; do
        echo -e "  $PASS Found: $dev  (likely XBee CP2102)"
        TTY_FOUND=true
    done
fi

if ! $TTY_FOUND; then
    echo -e "  $FAIL No serial devices found in /dev/ttyACM* or /dev/ttyUSB*"
    echo "        → Ensure Arduino and XBee are connected, then re-run."
fi

# -------------------------------------------------------------------
# 5. Camera Devices
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}[5/7] Camera Devices${RESET}"

if ls /dev/video* &>/dev/null 2>&1; then
    for dev in /dev/video*; do
        # Only list actual capture devices (even-numbered in V4L2)
        echo -e "  $PASS Camera device: $dev"
    done
else
    echo -e "  $FAIL No /dev/video* found — USB webcam not detected"
fi

# -------------------------------------------------------------------
# 6. User Group Membership (dialout for serial access)
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}[6/7] User Permissions${RESET}"

CURRENT_USER=$(whoami)
if groups "$CURRENT_USER" | grep -q "dialout"; then
    echo -e "  $PASS User '$CURRENT_USER' is in 'dialout' group — serial port access OK"
else
    echo -e "  $WARN User '$CURRENT_USER' is NOT in 'dialout' group"
    echo "        → Run: sudo usermod -aG dialout $CURRENT_USER"
    echo "        → Then log out and back in (or reboot)"
fi

if groups "$CURRENT_USER" | grep -q "video"; then
    echo -e "  $PASS User '$CURRENT_USER' is in 'video' group — camera access OK"
else
    echo -e "  $WARN User '$CURRENT_USER' is NOT in 'video' group"
    echo "        → Run: sudo usermod -aG video $CURRENT_USER"
fi

# -------------------------------------------------------------------
# 7. Summary
# -------------------------------------------------------------------
echo ""
echo -e "${BOLD}======================================================${RESET}"
echo -e "${BOLD}  Phase 0 Check Complete${RESET}"
echo -e "${BOLD}======================================================${RESET}"
echo ""
echo "  Next step: Run install_deps.sh to install Python packages"
echo "  Command  : bash install_deps.sh"
echo ""
