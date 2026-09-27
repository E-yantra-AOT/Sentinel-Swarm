#!/usr/bin/env bash
# Phase 2 — Motor Test
# Sends commands to Arduino via USB serial and verifies response
# Robot WILL MOVE — place it on a clear surface first!

VENV="$HOME/swarm_venv"

"$VENV/bin/python3" << 'PYEOF'
import serial, time, sys

ARDUINO_PORT = "/dev/ttyUSB1"
BAUD         = 115200

def send_cmd(s, cmd):
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    time.sleep(0.1)
    resp = s.readline().decode("utf-8", errors="replace").strip()
    return resp

print("=" * 44)
print(f"  Phase 2 Motor Test — {__import__('socket').gethostname()}")
print("=" * 44)

try:
    s = serial.Serial(ARDUINO_PORT, BAUD, timeout=2)
    time.sleep(2.0)   # wait for Arduino reset on serial open
    s.reset_input_buffer()
    print(f"  Opened {ARDUINO_PORT} at {BAUD} baud\n")
except Exception as e:
    print(f"  [FAIL] Cannot open {ARDUINO_PORT}: {e}")
    sys.exit(1)

tests = [
    ("STOP",       "M,0,F,0,F",       0.5),
    ("FWD slow",   "M,100,F,100,F",   1.5),
    ("STOP",       "M,0,F,0,F",       0.5),
    ("TURN LEFT",  "M,60,B,100,F",    1.0),
    ("STOP",       "M,0,F,0,F",       0.5),
    ("TURN RIGHT", "M,100,F,60,B",    1.0),
    ("STOP",       "M,0,F,0,F",       0.5),
    ("REV slow",   "M,100,B,100,B",   1.5),
    ("STOP",       "M,0,F,0,F",       0.3),
]

all_ok = True
for label, cmd, wait in tests:
    resp = send_cmd(s, cmd)
    status = "OK" if resp == "OK" else f"GOT: {repr(resp)}"
    icon   = "[PASS]" if resp == "OK" else "[WARN]"
    print(f"  {icon} {label:<12}  cmd={cmd:<18}  {status}")
    if resp != "OK":
        all_ok = False
    time.sleep(wait)

s.close()
print()
if all_ok:
    print("  ALL PASS — motors responding correctly!")
else:
    print("  Some commands had unexpected responses — check wiring.")
print("=" * 44)
PYEOF
