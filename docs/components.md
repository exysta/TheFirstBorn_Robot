# 🧩 Components List – TheFirstBorn Robot

This document lists the core hardware components used in the design and development of the **TheFirstBorn** autonomous mobile robot. Each part was selected with consideration for performance, cost, availability, and compatibility with the overall system architecture.

---

## 🧠 Compute & Control

| Component              | Model / Description                           | Notes                                             |
|------------------------|-----------------------------------------------|---------------------------------------------------|
| **High-Level SBC**     | Raspberry Pi 5 (8GB)                          | Runs ROS 2 Jazzy in a Docker container.           |
| **Real-Time MCU**      | STM32H7A3 Nucleo Board                        | Handles motor control, encoder readout, IMU, etc. |
| **Firmware Platform**  | Zephyr RTOS                                   | Chosen for its modularity and RTOS features.      |

---

## ⚙️ Locomotion & Power

| Component               | Model / Description                          | Notes                                             |
|-------------------------|----------------------------------------------|---------------------------------------------------|
| **Drive System**        | Tracked drive (2 motors)                     | Better traction and floor adaptability.           |
| **Motors**              | N20 or similar DC gear motors                | Compact, cost-effective, good torque.             |
| **Motor Drivers**       | TB6612FNG Dual H-Bridge or equivalent        | Reliable and STM32-compatible.                    |
| **Encoders**            | Magnetic or optical encoders (motor-mounted) | Used for odometry.                                |
| **Power Supply**        | Lion Battery (~7.4V, 2000–3000mAh)        | Main power source.                                |
| **Voltage Regulators**  | 5V / 3.3V step-down modules (e.g. MP1584)    | Power MCU, SBC, and peripherals.                  |

---

## 📡 Sensors

| Component             | Model / Description                            | Notes                                             |
|------------------------|------------------------------------------------|---------------------------------------------------|
| **IMU**               | BNO080 / BNO085 (GY-BNO080 Module)            | 9DOF + onboard sensor fusion (AHRS).              |
| **Proximity Sensors** | HC-SR04 (Ultrasonic) + TCRT5000 (IR)          | Basic obstacle avoidance and edge detection.      |
| **Camera**            | Raspberry Pi Camera Module (v2 or HQ)         | For future vision-based features.                 |

---

## 🔌 Connectivity

| Component             | Model / Description                            | Notes                                             |
|------------------------|------------------------------------------------|---------------------------------------------------|
| **STM32 ↔ RPi Comm**  | UART over USB (CDC)                            | Enables micro-ROS bridge.                         |
| **Wireless Comm**     | Raspberry Pi 5 Built-in Wi-Fi / Bluetooth     | Used for remote control, SSH, ROS 2 communication.|

---

## 📐 Hardware Design

| Component             | Model / Description                            | Notes                                             |
|------------------------|------------------------------------------------|---------------------------------------------------|
| **PCB Design Tool**   | KiCad                                           | Used to design a custom STM32-based board.        |
| **Chassis**           | Custom 3D-printed or laser-cut acrylic         | CAD-designed for track system and modularity.     |
| **Mounting Hardware** | Assorted screws, standoffs, brackets           | For assembly and component mounting.              |

---

## 🗂️ BOM (Bill of Materials)

> A complete BOM with quantities, links, and pricing will be added during final integration (Phase 1).

---

## 📌 Notes

- All selected components align with the project budget (~€100, excluding Raspberry Pi and STM32 board).
- Most sensors and modules are **readily available** from common online electronics stores (e.g., Aliexpress, Amazon, Digikey).
- The platform is designed for **expandability**, especially in sensing and compute layers.

---

## 📎 Related Files

- [README.md](./README.md) – Full project overview  
- [design_decisions.md](./decisions_log.md) – Justification for selected parts  
