# Running Micro-ROS over UART on ESP32-S3 (AliExpress Version)

This guide details how to run Micro-ROS over UART on an ESP32-S3 board purchased from AliExpress, using the **ESP-IDF VS Code extension** (ESP-IDF v5.2.5) and **Micro-ROS Jazzy**. Development was done on **Ubuntu 24.04 (Noble)**.

## Hardware Setup
Communication is done over the built-in **USB-C to UART** interface (right-hand port on the board):

- **UART0**
  - **TX Pin:** 43
  - **RX Pin:** 44

---

## 1. Flashing the Program to the ESP32-S3

1. **Create a New ESP-IDF Project**
   - In VS Code, use the ESP-IDF extension to create a new project.  
     The example used does not matter — it will be replaced.

2. **Add the Micro-ROS Component**
   ```bash
   mkdir components
   cd components
   git clone -b jazzy git@github.com:micro-ROS/micro_ros_espidf_component.git
   ```

3. **Open an ESP-IDF Console**  
   (Accessible from the bottom panel in VS Code)

4. **Navigate to Example**
   ```bash
   cd micro_ros_espidf_component/examples/int32_publisher_custom_transport
   ```

5. **Set the Target and Configure**
   ```bash
   idf.py set-target esp32s3
   idf.py menuconfig
   ```

6. **Menuconfig Settings**
   - **Micro-ROS Settings → Micro-ROS Network Interface:**  
     Set to `XRCE-DDS over UART`.
   - **Micro-ROS Settings → UART Settings:**  
     Ideally, set TX/RX pins here. However, ESP-IDF’s `menuconfig` only accepts pins in the range `-1 to 33`.  
     For pins above 33, modify the transport source directly.

7. **Manually Set UART Pins**
   Edit:
   ```
   micro_ros_espidf_component/examples/int32_publisher_custom_transport/main/esp32_serial_transport.c
   ```
   Replace:
   ```c
   #define UART_TXD  43 // instead of CONFIG_MICROROS_UART_TXD
   #define UART_RXD  44 // instead of CONFIG_MICROROS_UART_RXD
   #define UART_RTS  (CONFIG_MICROROS_UART_RTS)
   #define UART_CTS  (CONFIG_MICROROS_UART_CTS)
   ```

8. **Build and Flash**
   ```bash
   idf.py build
   idf.py flash   # Use the USB & OTG port for flashing
   ```
   > After flashing, switch to the UART-over-USB port to avoid bootloader messages interfering with Micro-ROS communication.

---

## 2. Running the Micro-ROS Agent

Follow the instructions from the [Micro-ROS Setup Repository](https://github.com/micro-ROS/micro_ros_setup/tree/jazzy#):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir uros_ws && cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
```

### Create and Build Agent Workspace
```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

### Run the Micro-ROS Agent
Ensure the board is connected to the **UART-over-USB** port or directly to the UART pins.

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

---

## 3. Testing the Connection
1. Press the **reset** button on the ESP32-S3.
2. The firmware pauses if it cannot connect to the Micro-ROS Agent — after reset, you should see communication logs in the agent terminal.

---

## Notes
- If you encounter issues building the Micro-ROS Agent, you may need to update the ROS 2 repository key.
```
