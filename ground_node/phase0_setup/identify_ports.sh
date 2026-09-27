#!/usr/bin/env bash
# Identify which /dev/ttyUSB* is XBee vs Arduino
for port in /dev/ttyUSB0 /dev/ttyUSB1; do
    echo -n "$port → "
    udevadm info "$port" 2>/dev/null | grep -E "ID_VENDOR=|ID_MODEL=|ID_SERIAL=" | tr '\n' ' '
    echo
done
