FROM osrf/ros:jazzy-desktop-noble

# Arguments and Environment
ENV ZEPHYR_SDK_VERSION=0.17.2
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk
ENV ZEPHYR_PROJECT_DIR=/opt/zephyrproject
ENV ZEPHYR_BASE=${ZEPHYR_PROJECT_DIR}/zephyr
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV DEBIAN_FRONTEND=noninteractive

# Install base dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget curl unzip xz-utils file git cmake ninja-build gperf ccache dfu-util \
    device-tree-compiler libsdl2-dev libmagic1 udev libusb-1.0-0 \
    python3 python3-dev python3-pip python3-setuptools python3-tk python3-wheel \
    python3-venv locales && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Micro editor
RUN curl https://getmic.ro | bash && mv micro /usr/local/bin/

# Install Zephyr SDK
RUN wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz && \
    tar xf zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz && \
    mv zephyr-sdk-${ZEPHYR_SDK_VERSION} ${ZEPHYR_SDK_INSTALL_DIR} && \
    rm zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -t all -c

# Create virtualenv and install west
RUN python3 -m venv /opt/zephyr-venv && \
    /opt/zephyr-venv/bin/pip install --upgrade pip && \
    /opt/zephyr-venv/bin/pip install west

ENV PATH="/opt/zephyr-venv/bin:$PATH"

# Clone and set up Zephyr project under /opt
RUN west init -m https://github.com/zephyrproject-rtos/zephyr.git ${ZEPHYR_PROJECT_DIR} && \
    cd ${ZEPHYR_PROJECT_DIR} && \
    west update && \
    west zephyr-export && \
    pip install -r ${ZEPHYR_PROJECT_DIR}/zephyr/scripts/requirements.txt

# Set default shell and working directory
SHELL ["/bin/bash", "-c"]
WORKDIR /workspaces

CMD ["bash"]
