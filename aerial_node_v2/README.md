# Sentinel Swarm v2 — Aerial Node (Production Architecture)

> **Status:** Architectural Design + Full Codebase Complete — Pending Hardware Procurement

## Overview

The v2 aerial node is the **production-grade** successor to the current Pi 5 aerial prototype. It is designed specifically for autonomous **GPS-denied disaster zone operations** — search and rescue, structural collapse reconnaissance, and multi-drone swarm coordination in areas with no communications infrastructure.

This is not a hobbyist build. The hardware stack matches what is used in research-grade military and industrial autonomous systems.

---

## Hardware Stack

| Component | Spec | Role |
|-----------|------|------|
| **NVIDIA Jetson Orin Nano Super** | 8GB, 67 TOPS NPU | Main edge compute |
| **Intel RealSense D435i** | RGB-D, 30fps, 640x480 depth | VIO + 3D detection |
| **Livox Mid-360** | 200K pts/sec, 360° FOV | LiDAR dense mapping |
| **FLIR Lepton 3.5** | 160x120, 8.7µm LWIR thermal | Heat signature detection |
| **LoRa SX1262** | 868/915MHz, 22dBm, +10km range | Swarm mesh telemetry |
| **DW3000 UWB** | <10cm precision ranging | Anti-collision deconfliction |
| **ESP32-S31 (Custom FC)** | 800Hz FreeRTOS dual-core | Low-level PID flight |
| **4x BLHeli-S 30A ESC** | DShot300 protocol | Motor drive |
| **4x 2306 2450KV Motors** | 5" props | Propulsion |
| **4S 3300mAh LiPo** | 14.8V nominal | Power |

---

## Software Stack

| Subsystem | Technology | File |
|-----------|-----------|------|
| **Casualty Detection** | RT-DETR-L (TensorRT FP8 via Triton) | `vision/sentinel_vision.py` |
| **Instance Segmentation** | Grounded-SAM2 (Meta AI) | `vision/sentinel_vision.py` |
| **Thermal Fusion** | Custom threshold + morphological filter | `vision/sentinel_vision.py` |
| **Visual SLAM** | Isaac ROS Visual SLAM + EKF | `slam/sentinel_slam.py` |
| **LiDAR Mapping** | Livox → Voxel Occupancy Grid | `slam/sentinel_slam.py` |
| **Path Planning** | RRT* + Dynamic Potential Field | `slam/sentinel_slam.py` |
| **Swarm Logic** | Behavior Tree + Voronoi Partition | `swarm/sentinel_swarm.py` |
| **Leader Election** | Bully Algorithm (battery-weighted) | `swarm/sentinel_swarm.py` |
| **Formation Control** | Reynolds Flocking Rules | `swarm/sentinel_swarm.py` |
| **Flight Bridge** | UART MAVLink-lite → ESP32 | `flight_bridge/flight_bridge.py` |

---

## Jetson Orin Core Allocation

The 4 Cortex-A76 CPU cores are pinned via `taskset` to prevent resource starvation:

```
Core 0        → OS kernel + LoRa SPI driver + UART bridge to ESP32
Core 1-2      → Isaac ROS Visual SLAM + Livox LiDAR map fusion + EKF
Core 3        → RT-DETR-L + Triage logic + SAM2 dispatch

GPU/NPU       → TensorRT FP8 inference engine (Triton server, ~40fps)
DLA 0         → Thermal detector (always-on, ultra-low power)
DLA 1         → Monocular depth completion network
```

---

## Vision Model Choice: Why RT-DETR over YOLO

For finding casualties in disaster zones, standard YOLO fails because:
- YOLO is CNN-based and struggles with **heavily occluded** victims (e.g., trapped under debris with only an arm visible)
- YOLO depends on **NMS (Non-Maximum Suppression)** post-processing, adding latency

**RT-DETR-L** (Real-Time DEtection TRansformer by Baidu) solves both:
- **Multi-scale self-attention** understands the global context of the scene — it can infer a human body even from a fragment
- **NMS-free** end-to-end detection via bipartite matching
- At FP8 on the Jetson Orin NPU: **~40fps** @ 640×640

On top of RT-DETR, **Grounded-SAM2** (Meta AI) provides pixel-level segmentation of each detected casualty, enabling:
1. Precise body pose estimation (lying vs. seated vs. upright)
2. Measurement of the casualty's bounding volume for rescue planning

---

## Swarm Scenarios

### 1. Search Pattern (Zone Decomposition)
The search zone is automatically divided using **Voronoi decomposition** — each drone receives a cell of the total area proportional to its remaining battery, ensuring full coverage with no redundancy.

### 2. Casualty Found → Rescue Beacon
When any drone detects a casualty, it:
1. Broadcasts the 3D XYZ coordinates over LoRa to all peers.
2. The nearest peer with sufficient battery breaks from the search and hovers above the victim as a **visual strobe beacon** for ground rescue teams.
3. The remaining drones redistribute the orphaned Voronoi cell.

### 3. Perimeter / Combat Security
In security operations, drones form a **convex hull perimeter** around a secured area using **Reynolds Flocking Rules** (Separation + Alignment + Cohesion), maintaining 3-meter gaps.

### 4. LoRa Mesh Relay
If a drone enters an area with poor LoRa coverage (signal loss from peers for >3s), it automatically transitions to `RELAYING` mode — holding position to act as a signal repeater for the rest of the swarm.

### 5. Anti-Collision Deconfliction
The **DW3000 UWB** module measures precise relative distance (< 10cm accuracy) to every neighboring drone. If two drones come within **2 meters**, a local repulsive velocity command overrides the current mission trajectory.

---

## How to Run (Jetson Orin)

```bash
# 1. Install dependencies
pip install tritonclient[grpc] pyrealsense2 pyserial numpy opencv-python

# 2. Launch Livox Mid-360 driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 3. Launch Isaac ROS Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# 4. Launch the Sentinel Engine
python3 main.py

# Optional: Pin threads to cores for performance
taskset -c 0 python3 main.py
```

---

## Hardware Schematics
Full pinout diagrams, power rail topology, and SPI bus layout available in [`schematics/hardware_schematics.md`](schematics/hardware_schematics.md).
