# Sentinel Aerial Node — Drone Companion Computer

## Hardware Stack

| Component | Part | Interface |
|---|---|---|
| Companion Computer | Raspberry Pi 5 (8GB RAM, 64GB SD) | — |
| Flight Controller | MicoAir H743 V2 (STM32H743, ArduPilot) | UART1 @ 921600 |
| ESC | 55A AM32 (4-in-1, integrated with FC) | DShot/PWM from FC |
| GPS | u-blox NEO-6M | FC UART3 @ 38400 |
| IMU | BMI088 + BMI270 (onboard FC) | FC internal |
| Barometer | SPL06 (onboard FC) | FC internal |
| Camera | USB Webcam 1080p | USB → /dev/video0 |
| Comms | XBee PRO S2C (USB module) | USB → /dev/ttyUSB0 |

---

## Wiring: Pi 5 ↔ MicoAir H743 V2

```
  Raspberry Pi 5               MicoAir H743 V2
  ─────────────                ───────────────────
  GPIO14 (TX, Pin 8)  ──────►  UART1 RX (SERIAL1)
  GPIO15 (RX, Pin 10) ◄──────  UART1 TX (SERIAL1)
  GND    (Pin 6)      ──────── GND
  (Power Pi via 5V 3A BEC separately — NOT from FC)
```

> **Important:** Both Pi 5 GPIO and MicoAir H743 V2 operate at **3.3V logic** — no level shifter needed.

---

## FC Configuration (Mission Planner — do once)

Connect FC to laptop via USB, open Mission Planner → Full Parameter List:

```
SERIAL1_PROTOCOL = 2      ← MAVLink 2 (NOT 1)
SERIAL1_BAUD     = 921    ← 921,600 baud
SERIAL3_PROTOCOL = 5      ← GPS
SERIAL3_BAUD     = 38     ← 38,400 (configure NEO-6M to match)
SYSID_MYGCS      = 255    ← Pi 5 is the GCS
FS_GCS_ENABLE    = 0      ← disable GCS failsafe
AVOID_ENABLE     = 1      ← companion avoidance enabled
```

**After changing parameters → Reboot FC.**

### NEO-6M GPS Configuration (u-center, one-time)
1. Connect NEO-6M via USB adapter to laptop
2. Open u-center → Receiver → Connection → select COM port
3. Tools → Receiver Configuration:
   - Baud: 38400
   - Update rate: 5Hz (200ms)
4. Save to flash (Receiver → Action → Save Config)

---

## Pi 5 Setup

### 1. Flash SD Card
Use Raspberry Pi Imager with these settings:
- OS: Raspberry Pi OS Lite 64-bit (Bookworm)
- Hostname: `pi3`
- Enable SSH: ✅
- Username: `pi3` / Password: your choice
- WiFi: your network SSID + password

### 2. Install Dependencies
```bash
# SSH into Pi 5
ssh pi3@10.219.37.35

# Copy and run installer
bash install_drone_deps.sh

# Reboot for UART + groups to take effect
sudo reboot
```

### 3. Deploy YOLO Model
```powershell
# From Windows (update IP in export_yolo_ncnn.py first)
python .\ground_node\phase0_setup\export_yolo_ncnn.py
```

### 4. Deploy Drone Script
```powershell
# From Windows
scp .\aerial_node\drone_companion.py pi3@10.219.37.35:~/drone_companion.py
```

---

## Running

### Single drone + 2 ground bots (full swarm):
```powershell
.\start_sentinel.ps1
```

### Ground bots only (no drone):
```powershell
.\start_sentinel.ps1 --no-drone
```

### With live MJPEG streams from bots:
```powershell
.\start_sentinel.ps1 --stream
```

### Drone only (manual SSH):
```bash
ssh pi3@10.219.37.35
source ~/drone_venv/bin/activate
python3 ~/drone_companion.py

# Simulation mode (no FC required):
python3 ~/drone_companion.py --sim

# Custom cruise altitude:
python3 ~/drone_companion.py --alt 5.0
```

---

## Autonomous State Machine

```
DISARMED ──(GPS fix ≥ 3D)──► ARMED ──(ESC spinup)──► TAKEOFF
                                                           │
                                                     (alt reached)
                                                           │
                                                           ▼
                              ◄──(target lost)──── SEARCHING ──(casualty found)──►
                              │                                                   │
                              │                                              TRACKING
                              │                                                   │
                              │                                     (conf > 0.75 + close)
                              │                                                   │
                              │                                              RELAYING
                              │                                                   │
                              ▼                                             (5s hover)
                      RTB ◄──(battery < 20%)─────────────────────────────────────┘
                       │
                  (alt < 0.5m)
                       │
                       ▼
                   LANDING → DISARMED
```

### Swarm Behaviour
- The drone joins the same **XBee PAN 3333** mesh as the ground bots
- Robot ID: `DRONE` (ground bots are `A` and `B`)
- When drone detects a casualty, it broadcasts via XBee → ground bots navigate toward area
- When a ground bot has high confidence detection, drone pivots to investigate that zone
- All three nodes run the same **Weighted Borda Count** leader election protocol

---

## Files

| File | Purpose |
|---|---|
| `drone_companion.py` | Main autonomous companion computer script |
| `install_drone_deps.sh` | One-shot Pi 5 dependency installer |
| `aerial_fc.ino` | ESP32 FC firmware (legacy, replaced by MicoAir H743) |
| `aerial_vision_node.py` | Vision-only stub (for testing without FC) |
| `Pi5_Advanced_Vision_Architecture.md` | Research notes on vision stack |
