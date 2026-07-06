# UAV Ground Control Station
### Multi-Layer Telemetry System — STM32 + FreeRTOS + CAN Bus + MAVLink + PX4 SITL + ROS2

![Status](https://img.shields.io/badge/status-in%20progress-orange)
![Platform](https://img.shields.io/badge/platform-STM32F103-blue)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![ROS](https://img.shields.io/badge/ROS2-Jazzy-blue)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> A hardware-in-the-loop UAV telemetry system where real STM32 sensor data drives a PX4 SITL simulation — physical IMU movements directly control a simulated drone in Gazebo, visualized through a custom Python GCS dashboard.

---

## Overview

This project implements a complete 4-layer UAV Ground Control Station architecture:

```
STM32F103 (FreeRTOS)
    ↓  CAN Bus (500 kbps)
    ↓  MAVLink over UART
PX4 SITL  ←  real IMU data replaces synthetic sensors
    ↓  uXRCE-DDS bridge
ROS2 Jazzy  ←  Kalman fusion, waypoint planning, anomaly detection
    ↓  WebSocket
Python GCS Dashboard  ←  live HUD, map, FreeRTOS monitor, alerts
```

**Key highlight:** The STM32 board feeds real GY-87 sensor data (accelerometer, gyroscope, magnetometer, barometer) into PX4 SITL via MAVLink — physically tilting the STM32 moves the simulated drone in Gazebo in real time.

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│              STM32F103 — Hardware Layer              │
│                                                     │
│  [IMU Task]    [Baro Task]   [Mag Task]             │
│  MPU6050       BMP180        QMC5883P               │
│  500ms / I2C   2000ms / I2C  1000ms / I2C           │
│       │              │            │                 │
│       └──────────────┴────────────┘                 │
│                      │ Queue + Mutex                │
│              [CAN TX Task]                          │
│         CAN Bus 500kbps / ID: 0x101–0x10F           │
└──────────────────────┬──────────────────────────────┘
                       │ MAVLink / UART (USB-TTL adaptör)
┌──────────────────────▼──────────────────────────────┐
│           PX4 SITL — Flight Control Layer            │
│                                                     │
│   EKF2 Sensor Fusion   →   Flight Modes             │
│   (IMU + Baro + Mag)       (Offboard / Mission)     │
└──────────────────────┬──────────────────────────────┘
                       │ uXRCE-DDS + Gazebo
┌──────────────────────▼──────────────────────────────┐
│              ROS2 Jazzy — Software Layer             │
│                                                     │
│  px4_ros2    │  Nav2       │  Kalman Node  │  GCS   │
│  interface   │  Waypoints  │  Fusion+Alarm │  Bridge│
└──────────────────────┬──────────────────────────────┘
                       │ WebSocket / CSV log
┌──────────────────────▼──────────────────────────────┐
│            Python GCS Dashboard                      │
│                                                     │
│  Map Panel  │  Telemetry HUD  │  FreeRTOS Monitor   │
│  Waypoints  │  Alt/Speed/Hdg  │  Task states/CPU    │
│             │  Battery/RSSI   │  Anomaly alerts     │
└─────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Part | Interface | Purpose |
|-----------|------|-----------|---------|
| MCU | STM32F103C8T6 (Blue Pill) | — | Main controller |
| IMU | GY-87 (MPU6050 + QMC5883P* + BMP180) | I2C | 10-DOF sensor fusion |
| CAN Transceiver | SN65HVD230 | CAN | Physical CAN layer |
| USB-UART | Generic USB-to-TTL adapter (PL2303 chip) | UART | MAVLink to PC |
| Termination | 120Ω resistor × 2 | CAN bus ends | Signal integrity |

*GY-87 boards are commonly sold with an "HMC5883L" silkscreen, but the actual magnetometer die
found on this unit (via I2C bus scan, addr `0x2C` instead of `0x1E`) is a QMC5883P clone with a
completely different register map — the driver targets the real chip, not the label.

---

## FreeRTOS Task Architecture

| Task | Priority | Period | Function |
|------|----------|--------|----------|
| `IMU_TASK` | High | 500ms | MPU6050 accel + gyro via I2C → Queue |
| `MAG_TASK` | Medium | 1000ms | QMC5883P magnetometer → Queue |
| `BARO_TASK` | Medium | 2000ms | BMP180 pressure + temp → Queue |
| `CAN_TX_TASK` | Medium | triggered | Dequeue → pack → transmit (Mutex protected) |
| `WDG_TASK` | Low | 1000ms | System health, heap monitor, heartbeat |

---

## CAN Bus Message Format

| CAN ID | DLC | Payload | Description |
|--------|-----|---------|-------------|
| 0x101 | 8B | `[ax16][ay16][az16][gx8][gy8][gz8]` | IMU accel + gyro |
| 0x102 | 4B | `[gx16][gy16][gz16][pad]` | Gyro high-res |
| 0x103 | 4B | `[pressure32]` | Barometer |
| 0x104 | 4B | `[temp_raw16][alt16]` | Temperature + altitude |
| 0x105 | 6B | `[mx16][my16][mz16]` | Magnetometer |

---

## Project Status

| Layer | Status | Notes |
|-------|--------|-------|
| STM32 FreeRTOS tasks | ✅ Complete | IMU task, CAN TX task |
| GY-87 I2C driver | ✅ Complete | MPU6050 + BMP180 + QMC5883P all done |
| CAN Bus transmission | ✅ Complete (loopback mode) | SN65HVD230 transceiver |
| MAVLink encoding | ✅ Verified on hardware | Official mavlink/c_library_v2, HEARTBEAT + HIL_SENSOR |
| PX4 SITL first flight | ✅ Complete | Gazebo (`gz_x500`), arm → takeoff → land verified via commander log |
| PX4 SITL HIL bridge | ✅ Complete | `tools/hil_bridge.py` forwards real STM32 sensors into PX4's HITL link at 50Hz (dithered resend); EKF2 runs clean, live `ATTITUDE` streamed from real hardware |
| Gazebo visualization | ✅ Complete | `tools/attitude_to_gazebo.py` teleports a model's pose from PX4's live attitude estimate; physically tilting the board visibly moves it in Gazebo |
| ROS2 uXRCE-DDS bridge | ✅ Complete | Micro-XRCE-DDS-Agent + `px4_msgs`/`px4_ros_com`; `ros2/uav_gcs_bridge` node subscribes to `/fmu/out/vehicle_attitude` + `/fmu/out/sensor_combined`, streaming real STM32 hardware data end-to-end into ROS2 |
| ROS2 Kalman fusion + anomaly alarm | ✅ Complete | `kalman_fusion` node runs an independent 2-state (angle, gyro-bias) Kalman filter on raw gyro/accel and cross-checks it against PX4's EKF2 estimate, flagging sustained disagreement as an anomaly |
| Nav2 waypoint planning | 📋 Planned | Out of scope without real position/actuation feedback |
| Python GCS dashboard | 📋 Planned | rich + WebSocket |

---

## Software Dependencies

```bash
# ROS2
sudo apt install ros-jazzy-desktop

# PX4 SITL
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot && make px4_sitl gz_x500

# ROS2 bridge (uXRCE-DDS)
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git ros2_ws/src/px4_msgs
git clone https://github.com/PX4/px4_ros_com.git ros2_ws/src/px4_ros_com
ln -s $(pwd)/ros2/uav_gcs_bridge ros2_ws/src/uav_gcs_bridge   # this repo's own node
cd ros2_ws && colcon build --symlink-install

# Python
pip install pymavlink python-can rich websockets numpy

# STM32
# STM32CubeIDE + STM32CubeMX
```

---

## Roadmap

- [x] System architecture design
- [x] Hardware selection and procurement
- [x] STM32 FreeRTOS task skeleton
- [x] MPU6050 I2C driver (real accel + gyro data)
- [x] BMP180 I2C driver (real pressure + temp + altitude)
- [x] QMC5883P I2C driver (real magnetometer data)
- [x] Gyro bias + magnetometer hard-iron calibration (verified on hardware)
- [x] CAN Bus frame encoding and transmission
- [x] MAVLink bridge (STM32 → PC) — HEARTBEAT + HIL_SENSOR verified on hardware
- [x] PX4 SITL setup and first simulated flight (Gazebo `gz_x500`, arm → takeoff → land)
- [x] Hardware-in-the-loop: real IMU → PX4 SITL — EKF2 runs clean on real sensor data, live attitude estimate verified
- [x] Gazebo visualization: PX4 attitude → live model pose — physical tilt verified moving the model in real time
- [x] ROS2 uXRCE-DDS bridge: Micro-XRCE-DDS-Agent + `px4_msgs`/`px4_ros_com`, custom `attitude_listener` node verified against real hardware data
- [x] ROS2 Kalman fusion node: independent gyro/accel Kalman filter cross-checked against PX4's EKF2, anomaly alarm on sustained disagreement
- [ ] Nav2 waypoint planning (needs real position/actuation feedback, currently out of scope)
- [ ] Python GCS dashboard (HUD + map + alerts)
- [ ] Demo video: physical STM32 movement → Gazebo drone response (recording)
- [ ] GitHub Actions CI pipeline

---

## Motivation

Modern defence UAV systems rely on exactly this architecture: a dedicated embedded sensor node communicating over CAN Bus, a flight controller running on ARM Cortex-M hardware with a real-time OS, and a ground control station processing telemetry in real time. This project implements that full stack from bare-metal firmware to GCS dashboard, using the same protocols (MAVLink, CAN Bus, FreeRTOS) used in production systems.

---

## Author

**Yusuf Tuzcu** — Electrical & Electronics Engineering, Eskisehir Osmangazi University

IHA-1 UAV Pilot License | github.com/ysftzc

---

*This project is under active development. Stars and feedback welcome.*
