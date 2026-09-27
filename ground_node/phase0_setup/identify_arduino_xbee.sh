#!/usr/bin/env bash
# Probe each serial port to identify Arduino vs XBee
# Arduino with our sketch responds to a newline with "OK" at 115200 baud
# XBee in transparent mode sends nothing / garbage at 115200

VENV="$HOME/swarm_venv"

echo "Probing serial ports to identify Arduino vs XBee..."
echo ""

"$VENV/bin/python3" << 'PYEOF'
import serial, time, os

ports = []
for p in ["/dev/ttyUSB0", "/dev/ttyUSB1"]:
    if os.path.exists(p):
        ports.append(p)

results = {}
for port in ports:
    try:
        s = serial.Serial(port, 115200, timeout=2)
        time.sleep(2.0)          # wait for Arduino reset
        s.reset_input_buffer()
        s.write(b"M,0,F,0,F\n")  # send stop command
        time.sleep(0.3)
        resp = s.readline().decode("utf-8", errors="replace").strip()
        s.close()
        results[port] = resp
        print(f"  {port}  ->  replied: {repr(resp)}")
    except Exception as e:
        results[port] = ""
        print(f"  {port}  ->  error: {e}")

print()
arduino_port = None
xbee_port    = None

for port, resp in results.items():
    if "OK" in resp or "READY" in resp or "WATCHDOG" in resp:
        arduino_port = port
    else:
        xbee_port = port

if arduino_port:
    print(f"  [IDENTIFIED] Arduino -> {arduino_port}")
else:
    print("  [WARN] Arduino not identified — is sketch flashed and board powered?")

if xbee_port:
    print(f"  [IDENTIFIED] XBee   -> {xbee_port}")
else:
    print("  [WARN] XBee not identified")

print()
if arduino_port and xbee_port:
    print("CONFIG for your Python scripts:")
    print(f"  ARDUINO_PORT = '{arduino_port}'")
    print(f"  XBEE_PORT    = '{xbee_port}'")
PYEOF
