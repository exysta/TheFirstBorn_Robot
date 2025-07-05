FROM osrf/ros:jazzy-desktop-noble

# Allow overriding workspace path from devcontainer.json
ARG WORKSPACE_PATH=/workspaces/app
ENV WORKSPACE_PATH=$WORKSPACE_PATH
ENV ZEPHYR_DIR=$WORKSPACE_PATH/zephyrproject
ENV ZEPHYR_BASE=$ZEPHYR_DIR/zephyr

# Install ROS 2 full desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop-full=0.11.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install Zephyr dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
    wget xz-utils file make gcc gcc-multilib g++-multilib \
    libsdl2-dev libmagic1 python3-dev python3-pip python3-setuptools \
    python3-tk python3-wheel python3-venv locales && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create workspace and Zephyr folder
RUN mkdir -p $ZEPHYR_DIR
WORKDIR $ZEPHYR_DIR

# Create Python virtual environment
RUN python3 -m venv $WORKSPACE_PATH/.venv

# Activate virtualenv and install west
RUN bash -c "source $WORKSPACE_PATH/.venv/bin/activate && \
    pip install --upgrade pip && \
    pip install west"

# Default to bash shell
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
