# TheFirstBorn - Advanced Autonomous Mobile Robot

![WIP](https://img.shields.io/badge/status-WIP-yellow.svg)
![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blueviolet)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-RTOS-6f42c1)
![Dockerized](https://img.shields.io/badge/Containerized-Docker-blue)
![Platform: Raspberry Pi 5](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-orange)
![Microcontroller: ESP32-S3](https://img.shields.io/badge/MCU-ESP32--S3-lightgrey)

**TheFirstBorn is a modern, autonomous mobile robot built on a professional-grade software and hardware stack. It integrates a real-time ESP32-S3 microcontroller running FreeRTOS with a high-performance Raspberry Pi 5 for advanced navigation and AI tasks within the ROS 2 ecosystem.**

This project serves as a comprehensive showcase of skills in embedded systems, real-time programming, robotics software integration, and hardware design.

## Key Features

*   **Split-Compute Architecture:** Real-time control on ESP32-S3, high-level processing on Raspberry Pi 5.
*   **Professional Robotics Framework:** Natively built for **ROS 2**, with the low-level controller integrated into the ROS 2 graph as a native node via **micro-ROS**.
*   **Autonomous Navigation:** Capable of **SLAM** (Simultaneous Localization and Mapping) and fully autonomous, path-planned navigation with the **Nav2** stack.
*   **AI-Powered Perception:** Supports deployment of lightweight AI models (e.g., TFLite) for onboard object detection and classification.
*   **Reproducible Development:** The entire high-level software environment is **containerized with Docker**, ensuring a consistent and easy-to-deploy setup.
*   **Custom Hardware:** Features a custom-designed PCB using **KiCad** for robust power management and peripheral integration.
*   **Advanced RTOS:** Utilizes **FreeRTOS** on the ESP32-S3 for a lightweight, reliable, and widely-supported firmware foundation.

## Tech Stack

| Category          | Technology / Tool                                                              |
| ----------------- | ------------------------------------------------------------------------------ |
| **Firmware**      | ESP32-S3, **FreeRTOS**, **micro-ROS**, C                                      |
| **High-Level**    | Raspberry Pi 5, Ubuntu 24.04, **ROS 2 Jazzy**, **Docker**                      |
| **Languages**     | C, C++, Python, (exploring **Rust** for performance-critical nodes)            |
| **AI & Vision**   | OpenCV, TensorFlow Lite                                                        |
| **Hardware Design** | **KiCad** (Schematics & PCB Layout)                                            |
| **Version Control** | Git & GitHub                                                                   |

## System Architecture

The robot employs a two-tier, split-compute architecture, which is standard in modern robotics. This separates safety-critical, real-time tasks from computationally intensive, non-real-time tasks.

*   **Real-time Control Layer (ESP32-S3):** Running FreeRTOS, this MCU is responsible for all hard real-time operations:
    *   Motor control via PWM.
    *   Reading wheel encoders for odometry feedback.
    *   Interfacing with low-level sensors like an IMU.
    *   It communicates with the high-level layer by running a **micro-ROS node**, exposing its functionality as ROS 2 topics, services, and parameters.

*   **High-Level Processing Layer (Raspberry Pi 5):** Running a Dockerized ROS 2 environment, this SBC is the robot's brain:
    *   Hosts the main ROS 2 nodes.
    *   Runs SLAM and navigation algorithms (Nav2).
    *   Performs sensor fusion and computer vision processing.
    *   Executes AI/ML models.
    *   Handles all wireless communication.

## System Architecture Diagram

```mermaid
%%{init: {"theme": "base", "themeVariables": {"primaryColor": "#2E86C1", "lineColor": "#34495E", "textColor": "#212F3D"}}}%%
graph TD
    subgraph Robot_Platform["Robot Platform"]
        subgraph RTL["Real-Time Layer (ESP32-S3 on FreeRTOS)"]
            FIRMWARE["Firmware (C / FreeRTOS)"]
            MICROS["micro-ROS Node"]
            DRIVERS["Motor Drivers & Encoders"]
            IMU["IMU (I2C/SPI)"]
            FIRMWARE -->|Controls| DRIVERS
            FIRMWARE -->|Reads| IMU
        end

        subgraph HLL["High-Level Layer (Raspberry Pi 5)"]
            DOCKER_CONTAINER["Docker Container"]
            ROS2["ROS 2 Jazzy"]
            NAV2["Navigation (Nav2)"]
            SLAM["Mapping (SLAM Toolbox)"]
            VISION["Vision/AI (OpenCV, TF Lite)"]
            DOCKER_CONTAINER --> ROS2 --> NAV2 --> SLAM
            ROS2 --> VISION
        end

        subgraph HW["Hardware"]
            POWER["Power System (18650 Li-Ion battery pack & Regulators)"]
            CHASSIS["Chassis & Motors"]
            CAMERA["Camera (CSI/USB)"]
        end

        MICROS -->|micro-ROS over USB-CDC| ROS2
        ROS2 -->|Drives| CAMERA
        FIRMWARE -->|Interfaces with| CHASSIS
        POWER --> FIRMWARE
        POWER --> ROS2
        POWER --> CHASSIS
    end

    subgraph Off_Board["Off-Board Tools"]
        USER_PC["User PC / Remote"]
        RVIZ["RViz2 for Visualization"]
        TELEOP["Remote Control"]
        USER_PC -->|Wi-Fi| ROS2
        USER_PC --> RVIZ
        USER_PC --> TELEOP
    end

    style DOCKER_CONTAINER fill:#2ECC71,stroke:#27AE60,stroke-width:2px,color:#fff
    style RTL fill:#3498DB,stroke:#2980B9,stroke-width:2px,color:#fff
````

## Project Status

* [x] **Phase 0: Architecture & Technology Selection**
* [ ] **Phase 1: Hardware Design & Fabrication**

  * [ ] Component selection (motors, drivers, sensors)
  * [ ] PCB design in KiCad
  * [ ] PCB ordering and assembly
  * [ ] Mechanical chassis design (CAD) and 3D printing
* [ ] **Phase 2: Firmware Development (ESP32-S3)**

  * [ ] FreeRTOS base project setup
  * [ ] Motor driver and encoder implementation
  * [ ] IMU driver integration
  * [ ] micro-ROS node implementation (subscribing to `/cmd_vel`, publishing `/odom`)
* [ ] **Phase 3: High-Level Software (Raspberry Pi)**

  * [ ] Create Dockerfile and Docker Compose for ROS 2 environment
  * [ ] Basic teleoperation node (`teleop_twist_keyboard`)
  * [ ] Create robot description (URDF)
  * [ ] Launch files for core nodes
* [ ] **Phase 4: Integration & SLAM**

  * [ ] Establish robust communication between ESP32-S3 and RPi
  * [ ] Configure `slam_toolbox` with camera/LiDAR to create a map
  * [ ] Tune odometry and TF frames
* [ ] **Phase 5: Autonomous Navigation**

  * [ ] Configure and tune the Nav2 stack
  * [ ] Achieve successful goal-based navigation in a mapped environment
* [ ] **Phase 6: AI & Advanced Capabilities**

  * [ ] Implement a basic object detection node
  * [ ] (Optional) Rewrite a performance-critical node in Rust

## Getting Started

### Prerequisites

1. Git, FreeRTOS development environment (toolchain, ESP-IDF SDK), ESP32-S3 toolchain.
2. Docker and Docker Compose installed on your development machine.
3. A Raspberry Pi 5 with Ubuntu 24.04 and Docker installed.

### Build & Run

WIP

## Folder Structure

```
/TheFirstBorn
├── README.md             # This file: Project overview
├── docs/                 # Detailed documentation, requirements, decisions
├── hardware/             # KiCad files (schematics, PCB) and mechanical CAD
├── firmware/             # FreeRTOS project for the ESP32-S3
├── software/             # ROS 2 packages and Docker configuration for the RPi 5
└── media/                # Images, GIFs, and videos for documentation
```

## Documentation Links

* [Project Requirements](./docs/requirements.md)
* [Design Decision Log](./docs/decisions_log.md)
* [Components list](./docs/components.md)
* [MicroROS tuto on esp32](./docs/microROS_esp32.md)

## License

This project is licensed under the Apache License, Version 2.0.
See the [LICENSE](./LICENSE) file for details.

```
