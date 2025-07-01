# Design Decision Log - TheFirstBorn

This log records key design decisions made during the development of the "TheFirstBorn" robot.

**Format:**

*   **Date:** Date the decision was finalized. DD-MM-YYYY
*   **Topic:** Area of the design the decision relates to.
*   **Options Considered:** Brief list of alternatives evaluated.
*   **Decision:** The chosen approach or component.
*   **Rationale:** Justification for the decision (pros/cons, cost, availability, requirements alignment, etc.).

---
**Date:** 01-07-2025  
**Topic:** Final Architecture and Core Technologies  
**Options Considered:**  
- RTOS + Custom Firmware  
- Arduino Framework (baremetal)  
- Zephyr RTOS + micro-ROS  
- FreeRTOS  
- Linux-only ROS 2 implementation  

**Decision:**  
Use **Zephyr RTOS** with **micro-ROS** on **STM32H7A3**, and **ROS 2 Jazzy** containerized on **Raspberry Pi 5** using **Docker Compose**.  

**Rationale:**  
Zephyr offers modern RTOS capabilities, active development, and clean integration with micro-ROS. Chosen for its real-time determinism, modularity, and support for STM32H7 series. ROS 2 Jazzy on Docker provides a professional, reproducible development environment on RPi 5, enabling scalable and maintainable robotics architecture. This decision locks in the core tech stack as described in the README.

---

**Date:** 06-05-2025 (Initial Planning)
**Topic:** Core Processing Architecture
**Options Considered:**
    *   Single High-Power MCU (e.g., STM32H7 alone)
    *   Single Linux Board (e.g., Raspberry Pi alone)
    *   Dual Controller: Real-time MCU + Linux Board
**Decision:** Dual Controller: STM32H7 + Raspberry Pi 5
**Rationale:** Leverages existing hardware owned by the developer. Provides separation of concerns: STM32 excels at real-time control (motors, sensors) critical for robotics, while RPi 5 offers powerful processing, easier networking, camera support, and potential for ROS2 integration needed for higher-level tasks and future expansion. Avoids overloading a single processor.

---

**Date:** 06-05-2025
**Topic:** Inter-Controller Communication
**Options Considered:**
    *   UART (Serial)
    *   SPI
    *   I2C
    *   USB (via USB-to-serial on STM32 side)
    *   Ethernet (requires add-on for STM32)
**Decision:** UART
**Rationale:** Simple, widely supported on both platforms, sufficient bandwidth for initial command/telemetry needs. Low pin count required. Well-understood protocol. SPI/I2C are viable but often used for peripheral communication; USB adds complexity/cost.

---

**Date:** 06-05-2025
**Topic:** Initial Locomotion Type
**Options Considered:**
    *   2-Wheel Differential Drive
    *   4-Wheel Differential Drive
    *   Tracked Drive (Skid Steer)
    *   Omni-directional Wheels (3 or 4)
**Decision:** Tracked Drive (2 Motors)
**Rationale:** Balances cost, simplicity, and performance for indoor use. Tracks offer good traction and handle minor floor imperfections well. Keeps initial motor/driver count low to meet budget. Design will allow for future omni-wheel upgrade for enhanced maneuverability.

---

**Date:** 06-05-2025
**Topic:** Initial Sensing Strategy
**Options Considered:**
    *   Ultrasonic only
    *   IR only
    *   Combination of Ultrasonic and IR
    *   LiDAR (Low cost variants)
    *   Camera (Vision-based)
**Decision:** Combination: IR sensors (Close range / Line) + Ultrasonic sensor (Medium range)
**Rationale:** Provides complementary sensing capabilities at low cost. IR is good for detecting edges or very close obstacles. Ultrasonic gives distance estimates over a slightly longer range. LiDAR is generally outside the budget. Camera is planned for future enhancement (ball following). Sensors interface directly to STM32 for real-time readings.

---

**Date:** 06-05-2025
**Topic:** Wireless Communication Hardware
**Options Considered:**
    *   Add dedicated ESP32 module (for Wi-Fi/BT)
    *   Use RPi 5 built-in Wi-Fi/Bluetooth
    *   Use STM32-compatible Wi-Fi/BT module
**Decision:** Use Raspberry Pi 5 built-in Wi-Fi and Bluetooth
**Rationale:** Leverages existing hardware capability. Simplifies hardware design and reduces cost/complexity compared to adding a separate wireless module. RPi is well-suited for handling network stacks and higher-level communication protocols.

---

**Date:** 06-05-2025
**Topic:** Budget for New Components
**Options Considered:**
    *   < €50 (Very challenging)
    *   ~ €100
    *   ~ €200+ (More flexibility)
**Decision:** Target ~€100 / $100 for new components.
**Rationale:** Stems from user requirement for an "affordable" hobby project. Excludes cost of existing STM32H7 board and RPi 5. Requires careful component selection, particularly for motors, drivers, and potentially chassis materials (favoring DIY/3D printing).
