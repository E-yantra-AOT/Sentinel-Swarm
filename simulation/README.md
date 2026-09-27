# Sentinel Swarm — Simulation Package

This package contains everything needed to simulate the **Sentinel Aerial v1** drone in **Gazebo Harmonic** (via ROS2 Jazzy).

---

## Prerequisites

```bash
# ROS2 Jazzy + Gazebo Harmonic
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-robot-state-publisher

# Gazebo Harmonic
sudo apt install gz-harmonic
```

---

## What's Included

| File | Description |
|------|-------------|
| [`urdf/aerial_v1.urdf`](urdf/aerial_v1.urdf) | Full URDF of the Sentinel Aerial v1 quad |
| [`worlds/disaster_zone.sdf`](worlds/disaster_zone.sdf) | Gazebo disaster zone world |
| [`launch/aerial_v1_spawn.launch.py`](launch/aerial_v1_spawn.launch.py) | ROS2 launch — spawns N drones |

---

## URDF — What's Modeled

The [`aerial_v1.urdf`](urdf/aerial_v1.urdf) is a physically accurate description of your actual hardware:

```
sentinel_aerial_v1
├── base_link          (140×140×8mm carbon fiber X-frame)
├── arm_fl/fr/rl/rr    (4× 125mm carbon tubes)
├── motor_fl/fr/rl/rr  (4× 2306 2450KV brushless motors)
├── prop_fl/fr/rl/rr   (4× 5045 tri-blade props — continuous joints, spin in sim)
├── esp32_fc           (ESP32-S31 flight controller board)
├── rpi5               (Raspberry Pi 5 companion computer, stacked above FC)
├── imu_link           (MPU6050 — wired to Gazebo IMU sensor plugin @ 1000Hz)
├── camera_link        (USB webcam — 1280×720, 30fps, 69° FOV, 15° downward tilt)
├── camera_optical_link (ROS optical frame convention)
├── lora_module        (LoRa SX1262 with stub antenna)
└── battery            (4S 3300mAh LiPo — slung below frame as ballast)
```

**Total mass (URDF):** `0.320 + 4×0.020 + 4×0.030 + 4×0.008 + 0.010 + 0.046 + 0.040 + 0.008 + 0.290 = ~0.966 kg`

---

## Running the Simulation

### Single Drone
```bash
ros2 launch sentinel_sim aerial_v1_spawn.launch.py
```

### 5 Drones in a Line Formation
```bash
ros2 launch sentinel_sim aerial_v1_spawn.launch.py num_drones:=5
```

### 10 Drones in a Circle Formation + RViz2
```bash
ros2 launch sentinel_sim aerial_v1_spawn.launch.py \
  num_drones:=10 \
  formation:=circle \
  rviz:=true
```

### 100 Drones in a Grid (Demo for judges)
```bash
ros2 launch sentinel_sim aerial_v1_spawn.launch.py \
  num_drones:=100 \
  formation:=grid
```

---

## ROS2 Topics Published Per Drone

Each drone spawned as `sentinel_NN` publishes on namespace `/sentinel/sentinel_NN/`:

| Topic | Type | Description |
|-------|------|-------------|
| `/sentinel/imu` | `sensor_msgs/Imu` | MPU6050 IMU data (1000Hz, with noise) |
| `/sentinel/camera/image_raw` | `sensor_msgs/Image` | USB webcam feed (720p, 30fps) |
| `/model/sentinel_NN/pose` | `geometry_msgs/PoseStamped` | Drone 6-DoF pose in world frame |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time |

---

## World Description

[`worlds/disaster_zone.sdf`](worlds/disaster_zone.sdf) is a 30×30m urban collapse scenario:

- 🏚️ **Collapsed concrete slabs** (North boundary)
- 🧱 **Standing partial walls** (East & West) with realistic lean angles
- 🪨 **4× Rubble piles** scattered across the zone
- 🚨 **Casualty target** — static human figure lying partially hidden behind rubble (tests YOLO occlusion robustness)
- 🌫️ **Fog** — simulates smoke-filled disaster environment
- 💡 **Low ambient lighting** — simulates overcast disaster conditions

---

## Visualize URDF Without Gazebo (RViz2)

```bash
# Publish robot description
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat urdf/aerial_v1.urdf)"

# Joint state publisher GUI (to spin props manually)
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Open RViz2 and add RobotModel display
rviz2
```
