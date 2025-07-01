# Project Requirements: TheFirstBorn

## 1. Introduction

This document outlines the functional, non-functional, and system requirements for the **TheFirstBorn** autonomous mobile robot project.

> **Document Status: Work in Progress (WIP)**
>
> This is a living document and reflects the project's early stage of development. The requirements listed here represent the **target capabilities** of the final system. As the project progresses through its development phases, these requirements will be refined, detailed, and validated.

The primary goal of this project is to create a modern, advanced autonomous mobile robot that serves as a showcase for professional-grade robotics development practices.

## 2. Guiding Principles

The design and implementation shall adhere to the following high-level principles:

*   **Modularity:** The system is built on a split-compute architecture, separating real-time control from high-level processing.
*   **Industry-Standard Tooling:** The project shall leverage professional and modern tools like ROS 2, Zephyr RTOS, and Docker.
*   **Reproducibility:** The development and deployment environment shall be consistent, portable, and easy to set up using containerization.
*   **Scalability:** The architecture shall support the future addition of more complex sensors, actuators, and AI capabilities.
*   **Open Source:** The project will be developed under the Apache 2.0 license.

## 3. Functional Requirements

### 3.1. Hardware Layer

*   **FR-HW-01:** The system shall be built on a custom-designed chassis.
*   **FR-HW-02:** The system shall include a custom-designed Printed Circuit Board (PCB) created using KiCad for robust power management and peripheral integration.
*   **FR-HW-03:** The power system shall support a 2S LiPo battery and provide regulated power to all electronic components.
*   **FR-HW-04:** The system shall incorporate differential drive motors with corresponding wheel encoders for odometry.
*   **FR-HW-05:** The system shall be equipped with an Inertial Measurement Unit (IMU) for orientation sensing.
*   **FR-HW-06:** The system shall be equipped with a primary vision sensor (e.g., CSI or USB camera) for perception and navigation tasks.

### 3.2. Firmware Layer (Real-Time Control)

*   **FR-FW-01:** The firmware shall run on an STM32H7 microcontroller.
*   **FR-FW-02:** The firmware shall be built upon the Zephyr Real-Time Operating System (RTOS).
*   **FR-FW-03:** The firmware shall perform hard real-time tasks, including motor control via PWM.
*   **FR-FW-04:** The firmware shall read wheel encoder data to calculate and publish odometry information.
*   **FR-FW-05:** The firmware shall interface with and read data from the IMU.
*   **FR-FW-06:** The firmware shall integrate with the ROS 2 ecosystem by running a **micro-ROS** agent.
*   **FR-FW-07:** The firmware shall subscribe to a `/cmd_vel` (Twist) topic to receive velocity commands from the high-level layer.
*   **FR-FW-08:** The firmware shall publish robot odometry data to an `/odom` topic.

### 3.3. High-Level Software Layer (Processing)

*   **FR-SW-01:** The high-level software shall run on a Raspberry Pi 5.
*   **FR-SW-02:** The high-level software stack shall be built on **ROS 2 Jazzy**.
*   **FR-SW-03:** The system shall be capable of Simultaneous Localization and Mapping (SLAM) to generate a map of its environment.
*   **FR-SW-04:** The system shall be capable of autonomous, path-planned navigation within a pre-generated map using the **Nav2** stack.
*   **FR-SW-05:** The system shall perform sensor fusion to combine odometry, IMU, and visual data for accurate localization.
*   **FR-SW-06:** The system shall support the deployment of lightweight AI models (e.g., TensorFlow Lite) for tasks such as object detection or classification.
*   **FR-SW-07:** The system shall expose its state and can be controlled over a wireless network.

### 3.4. User Interaction

*   **FR-UI-01:** The system shall allow for remote manual control (teleoperation).
*   **FR-UI-02:** The system shall provide a real-time visualization of its status, sensor data, and navigation state via **RViz2** on a remote machine.

## 4. Non-Functional Requirements

### 4.1. Technology Stack

*   **NFR-TS-01:** **Microcontroller:** STM32H7A3.
*   **NFR-TS-02:** **Single-Board Computer:** Raspberry Pi 5.
*   **NFR-TS-03:** **Firmware OS:** Zephyr RTOS.
*   **NFR-TS-04:** **High-Level OS:** Ubuntu 24.04 (or compatible).
*   **NFR-TS-05:** **Robotics Framework:** ROS 2 Jazzy.
*   **NFR-TS-06:** **Development Environment:** The entire high-level software environment shall be containerized using **Docker** and orchestrated with Docker Compose for reproducibility.
*   **NFR-TS-07:** **Programming Languages:** The primary languages shall be C (firmware), C++ and Python (ROS 2 nodes). The use of **Rust** for performance-critical nodes is an exploratory goal.
*   **NFR-TS-08:** **Hardware Design Tool:** KiCad.
*   **NFR-TS-09:** **Version Control:** Git & GitHub.

### 4.2. Performance

*   **NFR-P-01:** All safety-critical motor control and sensor reading tasks shall meet hard real-time deadlines, managed by the Zephyr RTOS on the STM32.
*   **NFR-P-02:** The communication link between the microcontroller and the Raspberry Pi (micro-ROS over USB-CDC) shall be robust and have sufficient bandwidth for odometry and command data.

## 5. Phased Implementation Plan & Status

The requirements will be implemented incrementally across several phases. This plan reflects the current project status and roadmap.

| Phase | Key Deliverables / Requirements to Implement                                                                                                 | Status            |
| :---- | :------------------------------------------------------------------------------------------------------------------------------------------- | :---------------- |
| **0** | **Architecture & Technology Selection**                                                                                                      | ✅ **Complete**   |
| **1** | **Hardware Design & Fabrication**<br/>- Component selection<br/>- PCB design & fabrication<br/>- Chassis design & fabrication                  | ⬜️ **Planned**      |
| **2** | **Firmware Development (STM32)**<br/>- Zephyr RTOS setup<br/>- Motor & encoder drivers<br/>- IMU driver<br/>- micro-ROS node (`/cmd_vel`, `/odom`) | ⬜️ **Planned**      |
| **3** | **High-Level Software (Raspberry Pi)**<br/>- Dockerized ROS 2 environment<br/>- Basic teleoperation<br/>- Robot Description (URDF)              | ⬜️ **Planned**      |
| **4** | **Integration & SLAM**<br/>- STM32/RPi communication<br/>- `slam_toolbox` configuration<br/>- Odometry and TF tuning                          | ⬜️ **Planned**      |
| **5** | **Autonomous Navigation**<br/>- Nav2 stack configuration & tuning<br/>- Goal-based navigation                                                  | ⬜️ **Planned**      |
| **6** | **AI & Advanced Capabilities**<br/>- Object detection node (TFLite)<br/>- (Optional) Rust node implementation                                  | ⬜️ **Future Goal**  |
