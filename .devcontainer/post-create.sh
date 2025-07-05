#!/bin/bash
set -euo pipefail
set -x  # Print each command before execution
set +u  # disable 'unbound variable' checks temporarily

echo "--- Post-Create: Starting workspace setup ---"

# Detect workspace root (assuming script is run from workspace root)
WORKSPACE_PATH="$(pwd)"
VENV_PATH="$WORKSPACE_PATH/.venv"

sudo apt update

# Activate or create virtual environment
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
    ZEPHYR_DIR="$WORKSPACE_PATH/zephyrproject"
    west init "$ZEPHYR_DIR"
    cd "$ZEPHYR_DIR"
    west update
    west zephyr-export

    pip install -r zephyr/scripts/requirements.txt || echo "Zephyr requirements.txt not found"
    pip install -r bootloader/mcuboot/zephyr/requirements.txt || echo "MCUBoot requirements.txt not found"

    cd "$ZEPHYR_DIR/zephyr"
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
MICROROS_WS="$WORKSPACE_PATH/microros_ws"
mkdir -p "$MICROROS_WS"
cd "$MICROROS_WS"

if [ ! -d "src/micro_ros_setup" ]; then
    git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y || echo "rosdep install failed — some packages may be missing."

# Build micro-ROS tools
echo "--- Post-Create: Building micro-ROS tools ---"
colcon build --packages-select micro_ros_setup

echo "--- Post-Create: Setup complete! You can now build your Zephyr application. ---"
