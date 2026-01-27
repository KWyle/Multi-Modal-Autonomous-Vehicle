# Vehicle Code Documentation (ROS2 + Gazebo + ML Perception + Sensors)

This document explains how the **vehicle software stack works end-to-end**, including:
- the **Gazebo vehicle model** and simulated sensors,
- the **ML perception (YOLO)** inference pipeline,
- the **ultrasonic sensing** pipeline,
- and the **fusion/decision logic** that generates safe vehicle commands.

> Target stack (current): **ROS2 Humble**, **Gazebo (Classic or Ignition)**, **Python nodes**, **OAK-D Pro depth camera**, **ultrasonic sensors**, deployment on **Jetson Orin Nano**.

---

## System Overview

### What the code does (in one paragraph)
The system runs a simulated or real vehicle that publishes sensor data (camera/depth + ultrasonic). A perception node runs **YOLO inference** on camera frames and publishes detections. An ultrasonic monitor node converts distance readings into a **stop/clear signal**. A fusion/decision node combines these signals into a final **high-level drive command** (e.g., `STOP`, `SLOW`, `GO`) which can be consumed by a controller or vehicle interface.

### High-level architecture
- **Simulation path:** Gazebo publishes sensor topics ➜ ROS2 nodes consume topics ➜ decision outputs command topic
- **Real hardware path:** OAK-D + ultrasonic drivers publish topics ➜ same ROS2 nodes ➜ decision outputs command topic

---


