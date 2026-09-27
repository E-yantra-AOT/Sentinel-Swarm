#!/usr/bin/env bash
# Read diagnostic output from Arduino serial port
# The sketch runs in setup() so all output is already buffered

VENV="$HOME/swarm_venv"

"$VENV/bin/python3" << 'PYEOF'
import serial, time, sys

PORT = "/dev/ttyUSB1"
BAUD = 115200

print(f"Reading diagnostic output from {PORT}...")
print("=" * 50)

try:
    s = serial.Serial(PORT, BAUD, timeout=1)
    # Reset Arduino so setup() runs fresh and we capture all output
    s.setDTR(False)
    time.sleep(0.3)
    s.setDTR(True)
    s.reset_input_buffer()

    # Read for 18 seconds (enough for all 8 steps to complete)
    deadline = time.time() + 18
    while time.time() < deadline:
        line = s.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(f"  {line}")
    s.close()
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)

print("=" * 50)
print("Done. Report which steps showed wheel movement.")
PYEOF
