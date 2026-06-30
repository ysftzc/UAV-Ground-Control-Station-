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
│  MPU6050       BMP180        HMC5883L               │
│  50ms / I2C    500ms / I2C   100ms / I2C            │
│       │              │            │                 │
│       └──────────────┴────────────┘                 │
│                      │ Queue + Mutex                │
│              [CAN TX Task]                          │
│         CAN Bus 500kbps / ID: 0x101–0x10F           │
└──────────────────────┬──────────────────────────────┘
                       │ MAVLink / UART (CP2102)
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
| IMU | GY-87 (MPU6050 + HMC5883L + BMP180) | I2C | 10-DOF sensor fusion |
| CAN Transceiver | SN65HVD230 | CAN | Physical CAN layer |
| USB-UART | CP2102 | UART | MAVLink to PC |
| Termination | 120Ω resistor × 2 | CAN bus ends | Signal integrity |

---

## FreeRTOS Task Architecture

| Task | Priority | Period | Function |
|------|----------|--------|----------|
| `IMU_TASK` | High | 50ms | MPU6050 accel + gyro via I2C → Queue |
| `MAG_TASK` | Medium | 100ms | HMC5883L magnetometer → Queue |
| `BARO_TASK` | Medium | 500ms | BMP180 pressure + temp → Queue |
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
| GY-87 I2C driver | 🔄 In progress | MPU6050 + BMP180 |
| CAN Bus transmission | ✅ Complete (loopback mode) | SN65HVD230 transceiver |
| MAVLink encoding | 📋 Planned | mavlink_helpers.h |
| PX4 SITL integration | 📋 Planned | uXRCE-DDS bridge |
| ROS2 software layer | 📋 Planned | px4_ros2 + Nav2 |
| Python GCS dashboard | 📋 Planned | rich + WebSocket |

---

## Software Dependencies

```bash
# ROS2
sudo apt install ros-jazzy-desktop

# PX4 SITL
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot && make px4_sitl gazebo

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
- [ ] GY-87 I2C driver (MPU6050 + BMP180 + HMC5883L)
- [x] CAN Bus frame encoding and transmission
- [ ] MAVLink bridge (STM32 → PC)
- [ ] PX4 SITL setup and first simulated flight
- [ ] Hardware-in-the-loop: real IMU → PX4 SITL
- [ ] ROS2 integration (px4_ros2, Nav2, Kalman node)
- [ ] Python GCS dashboard (HUD + map + alerts)
- [ ] Demo video: physical STM32 movement → Gazebo drone response
- [ ] GitHub Actions CI pipeline

---

## Motivation

Modern defence UAV systems (e.g. Baykar TB2, TAI Aksungur) rely on exactly this architecture: a dedicated embedded sensor node communicating over CAN Bus, a flight controller running on ARM Cortex-M hardware with a real-time OS, and a ground control station processing telemetry in real time. This project implements that full stack from bare-metal firmware to GCS dashboard, using the same protocols (MAVLink, CAN Bus, FreeRTOS) used in production systems.

---

## Author

**Yusuf Tuzcu** — Electrical & Electronics Engineering, Eskisehir Osmangazi University

IHA-1 UAV Pilot License | github.com/ysftzc

---

*This project is under active development. Stars and feedback welcome.*
