# Sentinel Swarm - Aerial Node (Pi 5 + LoRa) Architecture

## Overview
For the disaster management deployment, the aerial node requires full **GPS-denied autonomous navigation** and casualty detection. This requires a significant compute upgrade over the ground node's Pi 4 setup. The aerial drone utilizes a **Raspberry Pi 5 (8GB RAM)** to run Visual SLAM (vSLAM) and a high-performance YOLO pipeline simultaneously, bridging commands to the ESP32-S31 flight controller.

## Hardware Stack
*   **Companion Computer:** Raspberry Pi 5 (8GB RAM)
*   **Flight Controller:** ESP32-S31 (Scratch-built, running `aerial_fc.ino`)
*   **Telemetry:** LoRa SX1262 / SX1276 via SPI (Replacing XBee for ultra-long-range mesh in disaster zones)
*   **Vision Sensor:** Monocular High-FOV Camera (e.g., Pi Camera Module 3 or Arducam). *(Future upgrade path mapped for Intel RealSense D435i or Livox Mid-360 LiDAR).*

## Vision AI Trade-offs: YOLO vs RT-DETR

For casualty detection in disaster zones (where humans are often partially obscured by rubble), the choice of AI architecture is critical:

1.  **YOLO (Current Baseline):** A Convolutional Neural Network (CNN). Extremely fast and heavily optimized for edge CPUs via NCNN. However, it relies heavily on Non-Maximum Suppression (NMS) to filter overlapping bounding boxes, which can bottleneck performance and struggle with heavily occluded victims.
2.  **RT-DETR (Real-Time DEtection TRansformer):** A cutting-edge Vision Transformer (ViT) by Baidu. It completely eliminates the NMS bottleneck and uses multi-scale self-attention to understand the global context of an image. **For disaster management, RT-DETR is superior** because its attention mechanism is incredibly robust at detecting partially buried or occluded casualties. *Caveat:* Transformers require massive matrix multiplications; running RT-DETR natively on a Pi 5 CPU will yield lower FPS than YOLO unless paired with a dedicated NPU (like a Hailo-8 or Google Coral TPU).

## Hardware Constraints: The Webcam Issue

While a standard **1080p USB Webcam** is sufficient for the AI object detection (RT-DETR/YOLO), it introduces severe challenges for **Visual SLAM**:
*   **Rolling Shutter:** Standard webcams capture frames line-by-line. When a drone vibrates or turns quickly, the image warps (Jello effect). VSLAM algorithms will lose feature tracking instantly. A **Global Shutter** camera (which captures the whole frame at exactly the same microsecond) is practically mandatory for drone VSLAM.
*   **Scale Ambiguity:** A single (monocular) webcam cannot perceive physical depth. VSLAM can map the geometry, but it won't know if a wall is 1 meter away or 10 meters away without integrating the ESP32's IMU data (Visual-Inertial Odometry) or upgrading to a Stereo Depth camera (like the Intel RealSense).

## Raspberry Pi 5 Core Allocation (4x Cortex-A76 @ 2.4GHz)

To run heavy VSLAM and YOLO concurrently without CPU starvation or brown-outs, we explicitly pin threads to the Pi 5's CPU cores using taskset/Cgroups:

*   **Core 0: OS & LoRa Telemetry**
    *   Handles the Linux kernel interrupts.
    *   Runs the SPI-based LoRa mesh transport layer.
    *   Handles the high-speed UART bridge (`115200` baud) to the ESP32 flight controller.
*   **Core 1 & Core 2: Visual SLAM (ORB-SLAM3 / RTAB-Map)**
    *   Extracts ORB features from the camera feed.
    *   Maintains the local map and calculates 6-DoF odometry (X, Y, Z, Pitch, Roll, Yaw) in real-time, completely replacing the need for GPS.
*   **Core 3: YOLO11 Vision Engine (Casualty Detection)**
    *   Runs YOLO exported to NCNN (FP16).
    *   Dedicated entirely to identifying casualties/humans in the disaster zone.
    *   Cross-references bounding boxes with the VSLAM depth-map to generate real-world 3D coordinates of the casualty.

## The Software Pipeline

1. **Camera Feed (640x480 @ 30fps)** -> Shared Memory Buffer.
2. **VSLAM Thread** reads the buffer, tracks visual features, and calculates the drone's position in a 3D coordinate grid.
3. **YOLO Thread** reads the buffer at ~10fps, scanning for casualties.
4. **Fusion Engine** maps the 2D YOLO bounding box onto the VSLAM 3D point cloud to get the exact XYZ location of the casualty.
5. **LoRa Thread** broadcasts the casualty's XYZ coordinates to the rest of the Sentinel Swarm.
6. **Flight Bridge** calculates velocity vectors based on the VSLAM map to navigate around obstacles and sends standard `CMD,<pitch>,<roll>` over serial to the ESP32.
