# Sentinel Swarm v2 — Hardware Schematics

This document describes the complete electrical connections for the production
Sentinel Swarm v2 aerial node.

## Hardware Bill of Materials

| Component                  | Purpose                              | Interface   |
|----------------------------|--------------------------------------|-------------|
| NVIDIA Jetson Orin Nano Super (8GB) | Main compute: SLAM, RT-DETR, Swarm | —       |
| ESP32-S31 (Custom FC)      | 800Hz PID flight controller          | UART        |
| Intel RealSense D435i       | RGB-D stereo depth camera            | USB 3.0     |
| Livox Mid-360               | 360° LiDAR (200K pts/sec)            | Ethernet    |
| FLIR Lepton 3.5             | Thermal camera (160x120, 8.7µm)      | SPI + I2C   |
| LoRa SX1262 Module          | Long-range swarm mesh (868MHz/915MHz)| SPI         |
| DW3000 UWB Module           | Close-range precise ranging (<10cm)  | SPI         |
| 4x 30A BLHeli-S ESC         | Motor drivers                        | PWM (FC)    |
| 4x 2306 2450KV Motors       | Propulsion                           | —           |
| 4S 3300mAh LiPo             | Power supply (14.8V)                 | —           |
| 5V 5A BEC                   | Jetson Orin power regulation         | —           |

---

## Jetson Orin GPIO Pinout

```
Jetson Orin Nano Super J17 (40-pin header)

Pin 1  (3.3V)         → FLIR Lepton VCC
Pin 2  (5V)           → LoRa SX1262 VCC, DW3000 VCC
Pin 6  (GND)          → All module GNDs
Pin 8  (UART0 TX)     → ESP32 RX (UART bridge to FC)
Pin 10 (UART0 RX)     → ESP32 TX
Pin 19 (SPI0 MOSI)    → LoRa SX1262 MOSI, DW3000 MOSI, FLIR Lepton MOSI
Pin 21 (SPI0 MISO)    → LoRa SX1262 MISO, DW3000 MISO, FLIR Lepton MISO
Pin 23 (SPI0 SCLK)    → LoRa SX1262 SCK, DW3000 SCK, FLIR Lepton SCK
Pin 24 (SPI0 CS0)     → LoRa SX1262 NSS (Chip Select)
Pin 26 (SPI0 CS1)     → DW3000 CS
Pin 29 (GPIO01)       → LoRa SX1262 DIO1 (IRQ)
Pin 31 (GPIO02)       → LoRa SX1262 RST
Pin 33 (GPIO03)       → DW3000 IRQ
Pin 36 (GPIO04)       → FLIR Lepton CS
Pin 3  (I2C1 SDA)     → FLIR Lepton CCI SDA
Pin 5  (I2C1 SCL)     → FLIR Lepton CCI SCL

USB 3.0 Port A        → Intel RealSense D435i
Gigabit Ethernet      → Livox Mid-360 (Static IP: 192.168.1.100)
```

---

## ESP32-S31 Flight Controller Pinout

```
ESP32-S31 GPIO Map

GPIO 8  (I2C SDA)  → MPU6050 SDA
GPIO 9  (I2C SCL)  → MPU6050 SCL
GPIO 18 (PWM CH0)  → ESC 1 (Front-Left motor)
GPIO 19 (PWM CH1)  → ESC 2 (Front-Right motor)
GPIO 20 (PWM CH2)  → ESC 3 (Rear-Left motor)
GPIO 21 (PWM CH3)  → ESC 4 (Rear-Right motor)
GPIO 1  (UART0 TX) → Jetson Orin UART RX (Pin 10)
GPIO 2  (UART0 RX) → Jetson Orin UART TX (Pin 8)
```

---

## Power Rail Diagram

```
4S LiPo (14.8V nominal, 16.8V max)
    │
    ├─── 4x BLHeli-S 30A ESCs ─────────── 4x 2306 Motors
    │
    └─── 5V 5A BEC
              │
              ├─── Jetson Orin Nano Super (USB-C PD / 5V barrel)
              ├─── ESP32-S31 (3.3V via onboard regulator)
              ├─── FLIR Lepton 3.5 (3.3V via LDO)
              ├─── LoRa SX1262 (3.3V)
              └─── DW3000 UWB (3.3V)

Note: RealSense D435i is powered directly by the Jetson USB 3.0 port (900mA)
Note: Livox Mid-360 has its own DC input (12-24V) — connect directly to LiPo
      via a DC-DC regulator (12V, 3A)
```

---

## Communication Bus Topology

```
┌─────────────────────────────────────────────────────────┐
│                   JETSON ORIN NANO SUPER                │
│                                                         │
│  Core 0    Core 1-2     Core 3      GPU/NPU    DLA0     │
│  LoRa+     Visual       RT-DETR     TensorRT   Thermal  │
│  UART      SLAM         YOLO        FP8 Inf.   Detector │
└──┬──────────────────────────────────────────────────────┘
   │ USB 3.0         │ Ethernet      │ SPI (shared bus)
   │                 │               │
   ▼                 ▼               ├──► LoRa SX1262 (868MHz)
RealSense D435i  Livox Mid-360      ├──► DW3000 UWB
RGB-D Camera     3D LiDAR           └──► FLIR Lepton 3.5

   │ UART (115200)
   ▼
ESP32-S31 Flight Controller (800Hz FreeRTOS PID)
   │
   ├──► MPU6050 (I2C, 1kHz)
   ├──► ESC 1 (PWM, 18)
   ├──► ESC 2 (PWM, 19)
   ├──► ESC 3 (PWM, 20)
   └──► ESC 4 (PWM, 21)
```
