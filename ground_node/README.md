# Sentinel Ground Node

This directory contains the software stack for the ground-based tracking and mesh communication layer of the Sentinel Swarm.

## Hardware Stack
*   **Microcontroller:** Arduino UNO (or compatible ATMega328P) for low-level motor bridging.
*   **Companion Computer:** Raspberry Pi 4B (4GB RAM) for vision and swarm logic.
*   **Vision:** Standard USB Webcam.
*   **RF Communication:** XBee modules (Transparent mode, 9600 baud) for peer-to-peer telemetry.
*   **Motor Driver:** TB6612FNG or similar dual H-bridge.

## Software Architecture

1. **Motor Bridge (`phase1_arduino/`)**
   *   Compiled using PlatformIO.
   *   Listens on serial (115200 baud) for ASCII commands from the Pi.
   *   Includes a 2-second safety watchdog.

2. **Vision Engine (`shared/yolo_ncnn.py`)**
   *   Custom Python wrapper around the `ncnn` library.
   *   Runs object detection models directly on the Pi's ARM CPU.

3. **Mesh Transport Layer (`shared/xbee_transport.py`)**
   *   Thread-safe JSON transport over serial.
   *   Provides a continuous data link between nodes in the swarm.

4. **Tracking Node (`phase4_tracking/Sentinel_Ground.py`)**
   *   The main intelligence script. 
   *   Runs 3 parallel threads: Camera vision, XBee RX, and Motor control.
   *   Prioritizes targets based on highest confidence and actively steers towards them.

## Usage
Start the tracking node:
```bash
python phase4_tracking/Sentinel_Ground.py --id A
```
To enable the debug HTTP stream:
```bash
python phase4_tracking/Sentinel_Ground.py --id A --stream
```
*(Note: Do not run the stream during normal autonomous operation as it increases latency).*
