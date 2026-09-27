#!/usr/bin/env bash
echo "========================================"
echo "  Device Check — $(hostname)"
echo "========================================"

echo ""
echo "[1] USB Devices (lsusb):"
lsusb

echo ""
echo "[2] Serial Ports:"
if ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null; then
    for port in $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null); do
        MODEL=$(udevadm info "$port" 2>/dev/null | grep "ID_MODEL=" | head -1 | cut -d= -f2)
        SERIAL=$(udevadm info "$port" 2>/dev/null | grep "ID_SERIAL_SHORT=" | head -1 | cut -d= -f2)
        echo "  $port  ->  model=$MODEL  serial=$SERIAL"
    done
else
    echo "  NONE found"
fi

echo ""
echo "[3] Camera Devices:"
if ls /dev/video0 2>/dev/null; then
    v4l2-ctl --device /dev/video0 --info 2>/dev/null | grep -E "Driver name|Card type" || echo "  /dev/video0 exists"
else
    echo "  NONE found — check webcam USB"
fi

echo ""
echo "[4] Summary:"
PORTS=$(ls /dev/ttyUSB* 2>/dev/null | wc -l)
CAM=$(ls /dev/video0 2>/dev/null | wc -l)
echo "  Serial ports found : $PORTS  (expect 2: XBee + Arduino)"
echo "  Camera found       : $CAM  (expect 1)"
echo "========================================"
