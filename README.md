# Sentinel: Distributed Autonomous Aerospace & Ground Swarm

**Sentinel** is an advanced **IoRT (Internet of Robotic Things)** and **Edge AI** ecosystem. It is a custom-built, dual-layer autonomous swarm designed for high-performance object tracking and stabilized flight. By bridging on-device neural networking (Edge IoT) with decentralized Machine-to-Machine (M2M) mesh communication and high-speed aerospace flight control, Sentinel serves as a scalable, distributed physical computing platform built entirely from scratch without relying on black-box commercial flight controllers.

## Architecture Overview

The project is divided into two distinct processing nodes that can work collaboratively:

```mermaid
flowchart TD
    subgraph AERIAL["🚁 Aerial Node v2 (Jetson Orin Nano Super — 67 TOPS)"]
        direction TB
        CAM["📷 RealSense D435i\nRGB-D Camera"] --> SLAM["🗺️ Isaac ROS Visual SLAM\nVIO + EKF Fusion\nCores 1-2"]
        LIDAR["📡 Livox Mid-360\n3D LiDAR 360°"] --> SLAM
        CAM --> RTDETR["🤖 RT-DETR-L\nCasualty Detection\nTensorRT FP8 NPU"]
        THERMAL["🌡️ FLIR Lepton 3.5\nThermal Camera"] --> THERMAL_DLA["Thermal DLA0\nHeat Signature\nDetector"]
        THERMAL_DLA --> FUSION["🔀 Sensor Fusion\nTriage Engine"]
        RTDETR --> FUSION
        SLAM --> PLANNER["📍 RRT* Path Planner\n+ Potential Field\nObstacle Avoidance"]
        FUSION --> PLANNER
        PLANNER --> FC_BRIDGE["🔌 Flight Bridge\nUART 115200"]
        LORA["📻 LoRa SX1262\n868MHz Mesh"] --> SWARM_MGR["🐝 Swarm Manager\nBehavior Tree\nLeader Election"]
        UWB["📶 DW3000 UWB\nPrecision Ranging"] --> SWARM_MGR
        SWARM_MGR --> PLANNER
    end

    subgraph FC["⚡ ESP32-S31 Flight Controller (800Hz FreeRTOS)"]
        FC_BRIDGE --> PID["Cascaded PID\nAngle → Rate\nCore 1"]
        IMU["MPU6050\nMahony AHRS"] --> PID
        PID --> MOTORS["4x BLHeli ESC\n→ 2306 Motors"]
        PID --> TELEM["IMU Telemetry\n→ Jetson EKF"]
    end

    subgraph GROUND["🤖 Ground Node (Raspberry Pi 4 — Verified)"]
        PI_CAM["📷 Camera"] --> YOLO["YOLO11n NCNN\nPerson Tracking\nFP16 CPU"]
        YOLO --> HYSTERESIS["Hysteresis\nControl Loop"]
        HYSTERESIS --> ARDUINO["Arduino Bridge\nMotor Commands"]
        XBEE["📡 XBee PRO S2C\nMesh Telemetry"] --> YOLO
        ARDUINO --> WHEELS["Differential Drive\nWheels"]
    end

    LORA_LINK(["☁️ LoRa Swarm Mesh\n868MHz / 915MHz"]) 
    AERIAL <-->|"LoRa Swarm Packets\nCasualty XYZ + Roles"| LORA_LINK
    GROUND <-->|"XBee JSON\nConfidence + Target"| LORA_LINK
```

```mermaid
flowchart LR
    subgraph JETSON["Jetson Orin Nano Super — Core Allocation"]
        C0["Core 0\n────────\nOS Kernel\nLoRa Driver\nUART Bridge"]
        C1["Core 1-2\n────────\nVisual SLAM\nVIO EKF\nLiDAR Fusion"]
        C3["Core 3\n────────\nRT-DETR-L\nSAM2 Segment\nTriage Logic"]
        NPU["GPU/NPU\n────────\nTensorRT FP8\nTriton Server\n~40fps Inference"]
        DLA0["DLA 0\n────────\nThermal Model\nAlways-On\nLow Power"]
        DLA1["DLA 1\n────────\nDepth Completion\nMonocular\nEstimation"]
    end
    C0 <--> C1 <--> C3
    C3 --> NPU
    DLA0 --> C3
    DLA1 --> C1
```



### 1. Aerial Node (`/aerial_node`)
Designed for **GPS-Denied Disaster Management**, the aerial drone pairs a high-performance flight controller with a next-gen edge companion computer.
*   **Edge Autonomy (Raspberry Pi 5 8GB):** Runs a multi-threaded Python engine heavily optimized for the Pi 5's Cortex-A76 cores. Core 1-2 run Monocular **Visual SLAM** for 3D mapping and navigation, while Core 3 runs a dedicated **YOLO11** NCNN pipeline for casualty detection.
*   **LoRa Telemetry:** Bypasses XBee and Wi-Fi to use an SPI-based SX1262 LoRa module for long-range, penetrating communication in disaster zones.
*   **"God-Mode" Flight Controller:** A scratch-built ESP32-S31 flight controller running Asymmetric FreeRTOS.
*   **ArduPilot-Grade Dynamics:** Implements a Cascaded PID architecture (Angle -> Rate) for locked-in stability.
*   **Mahony AHRS Sensor Fusion:** Fuses MPU6050 gyroscope and accelerometer data into 3D spatial orientation using advanced quaternion math.

### 2. Ground Node (`/ground_node`)
A tested and verified companion computer stack (Raspberry Pi + Arduino Bridge) for terrestrial tracking and swarm coordination.
*   **Neural Vision Engine:** Runs an optimized YOLO NCNN wrapper to track human targets in real-time.
*   **Mesh Communication:** Utilizes a lightweight XBee transport layer to broadcast tracking data between multiple swarm nodes.
*   **Hysteresis Control Loop:** Prevents mechanical jitter by implementing deadzone logic during active target pursuit.

## Experiments Conducted

Through extensive real-world testing, the ground-based tracking and communication layers have been verified. The following phases detail our experimental progress and milestones:

*   **Phase 0: Hardware Setup & Inspection (Complete)** - Initialized the chassis, Raspberry Pi, and XBee radios. Resolved brown-out issues by throttling YOLO CPU threads to `num_threads=1`.
*   **Phase 1: Motor Control (Complete)** - Achieved stable motor control bridging between the Pi and the low-level Arduino bridge.
*   **Phase 2: Serial Bridge Verification (Complete)** - Established reliable ASCII-based serial communication (115200 baud) between the Pi and motor controller.
*   **Phase 3: Vision Engine & YOLO (Complete)** - Successfully deployed YOLO11n (exported to NCNN FP16) natively on the Pi CPU for real-time person detection.
*   **Phase 4: Streaming & Telemetry (Complete)** - Set up debug streams to visualize tracking output without interfering with the main control loop.
*   **Phase 5: Hysteresis Tracking (Complete)** - Implemented visual tracking that commands motors dynamically while filtering out minor jitter using hysteresis deadzones.
*   **Phase 6: XBee Mesh Link Verification (Complete)** - Validated bidirectional JSON transport over XBee radios. Both nodes effectively transmit and receive target data, with the robot having the highest tracking confidence leading the pursuit.

**Pending Phases:**
*   **Phase 7: Full Cooperative Ground Test (Complete)** - Implemented a **Weighted Borda Count** leader election protocol. Every 100ms, each robot scores itself and all peers on a composite metric (`0.6×confidence + 0.3×freshness + 0.1×target_size`). The robot with the highest score becomes the **LEADER** and all others defer to its target coordinates. When all robots lose sight, they autonomously split into a coordinated scan sweep (A rotates right, B rotates left) to re-acquire.
*   **Phase 8: Forward-Drive + PID Integration** - Combining advanced PID dynamics with forward movement for smoother pursuit.

## Experiment Gallery

Here is a visual timeline of our physical prototyping and testing:

### 1. Hardware Base (Sentinel Ground Node)
The base robotics chassis for our ground testing, outfitted with Li-ion battery management, integrated H-bridges, and an Arduino-compatible pinout to bridge commands from the companion computer.

<p align="center">
  <img src="assets/IMG20260917142633.jpg" width="400" />
  <br>
  <em>Top view: Sentinel Ground Node hardware base with serial bridge.</em>
</p>

<p align="center">
  <img src="assets/IMG20260917142717.jpg" width="400" />
  <br>
  <em>Bottom view: Motors, wheels, and dual 14500 Li-ion cell power supply.</em>
</p>

### 2. XBee Mesh Network Telemetry
Verifying the XBee RF modules to establish a distributed mesh communication layer for the swarm, avoiding reliance on centralized Wi-Fi access points.

<p align="center">
  <img src="assets/IMG20260915115423.jpg" width="400" />
  <br>
  <em>Digi XBee PRO S2C RF Module.</em>
</p>

<p align="center">
  <img src="assets/IMG20260915124745.jpg" width="400" />
  <img src="assets/IMG20260915124752.jpg" width="400" />
  <br>
  <em>Testing bidirectional serial telemetry (e.g. Hello World data packets) between nodes.</em>
</p>

<p align="center">
  <img src="assets/IMG20260915154806.jpg" width="600" />
  <br>
  <em>Full telemetry loop test: Laptop transmitting to the Sentinel Ground Node (equipped with Raspberry Pi and XBee).</em>
</p>

### 3. Multi-Node Swarm & Vision Testing (YOLO)
Testing the YOLO vision engine to lock onto human targets. Swarm nodes communicate their confidence metrics over the XBee network, allowing multiple nodes to share visual tracking data.

<p align="center">
  <img src="assets/IMG20260922160510.jpg" width="600" />
  <br>
  <em>Testing single and dual ground nodes tracking with YOLO bounding boxes streamed to the command center.</em>
</p>

<p align="center">
  <img src="assets/IMG20260922162435.jpg" width="600" />
  <br>
  <em>Multi-node Swarm setup: Multiple Sentinel Ground Nodes coordinating target data while streaming YOLO feeds to multiple telemetry laptops.</em>
</p>

## Setup and Deployment
*Please refer to the individual `README.md` files inside each node directory for specific hardware wiring and compilation instructions.*