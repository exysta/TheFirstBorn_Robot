# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM osrf/ros:jazzy-desktop-noble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop-full=0.11.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
# USER root

# ENV DEBIAN_FRONTEND=noninteractive
# ENV LANG=en_US.UTF-8
# ENV LC_ALL=en_US.UTF-8
# ENV ZEPHYR_BASE=opt/zephyrproject/zephyr
# ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
# ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk

# --- Dépendances système (recommandées par Zephyr) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
    wget xz-utils file make gcc gcc-multilib g++-multilib \
    libsdl2-dev libmagic1 python3-dev python3-pip python3-setuptools \
    python3-tk python3-wheel python3-venv locales && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


# # Création de l'environnement virtuel
RUN mkdir -p /zephyrproject

WORKDIR /zephyrproject

RUN python3 -m venv .venv




# Activation de venv, installation de west, récupération des sources Zephyr
RUN bash -c "source .venv/bin/activate && \
    pip install --upgrade pip && \
    pip install west"
# # --- Configuration micro-ROS workspace ---
# ENV UROS_WS=/opt/microros_ws
# WORKDIR $UROS_WS

# RUN mkdir -p $UROS_WS/src && \
#     cd $UROS_WS/src && \
#     git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git && \
#     cd $UROS_WS && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -y && \
#     . /opt/zephyrproject/.venv/bin/activate && \
#     colcon build && \
#     echo 'source $UROS_WS/install/local_setup.bash' >> /ros_entrypoint.sh

# Shell interactif avec venv activé
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
