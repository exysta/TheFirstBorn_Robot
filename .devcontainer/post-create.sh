#!/bin/bash
set -euo pipefail
set -x  # Print each command before execution
set +u  # disable 'unbound variable' checks

echo "--- Post-Create: Starting workspace setup ---"
sudo apt update
# Activate or create virtual environment
VENV_PATH="/workspaces/Zephyr_MicroROS_Workspace/.venv"
if [ -f "$VENV_PATH/bin/activate" ]; then
    echo "--- Activating existing Python virtual environment ---"
    source "$VENV_PATH/bin/activate"
else
    echo "!!! Virtual environment not found. Creating one ..."
    python3 -m venv "$VENV_PATH"
    source "$VENV_PATH/bin/activate"
    pip install --upgrade pip
    pip install west

    # Initialize and fetch Zephyr project
    west init /zephyrproject
    cd /zephyrproject
    west update
    west zephyr-export

    # Install Python dependencies explicitly
    pip install -r /zephyrproject/zephyr/scripts/requirements.txt || echo "Zephyr requirements.txt not found"
    pip install -r /zephyrproject/bootloader/mcuboot/zephyr/requirements.txt || echo "MCUBoot requirements.txt not found"

    cd /zephyrproject/zephyr
    west sdk install

    echo "--- Virtual environment created and activated ---"
fi

# Source ROS 2 environment
echo "--- Sourcing ROS: $ROS_DISTRO ---"
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=$(which python3)
export AMENT_PREFIX_PATH=""
source "/opt/ros/$ROS_DISTRO/setup.bash"
pip3 install catkin_pkg lark-parser empy colcon-common-extensions

# Clone micro-ROS setup
echo "--- Post-Create: Installing ROS dependencies ---"
mkdir -p /microros_ws
cd /microros_ws
git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools
echo "--- Post-Create: Building micro-ROS tools ---"
colcon build --packages-select micro_ros_setup

echo "--- Post-Create: Setup complete! You can now build your Zephyr application. ---"