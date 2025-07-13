#!/bin/bash
set -e

# Activate virtual environment
source .venv/bin/activate

# Export Zephyr base
export ZEPHYR_BASE=$PWD/zephyrproject/zephyr

# Install Python requirements for Zephyr
pip install -r zephyrproject/zephyr/scripts/requirements.txt

# Init west if not already
if [ ! -d "zephyrproject/zephyr" ]; then
  west init -m https://github.com/zephyrproject-rtos/zephyr.git zephyrproject
  cd zephyrproject
  west update
  west zephyr-export
  cd ..
fi
