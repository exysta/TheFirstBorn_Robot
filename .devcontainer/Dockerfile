FROM microros/base:latest

# Arguments and Environment
ENV DEBIAN_FRONTEND=noninteractive

# Install base dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget curl unzip xz-utils file git cmake ninja-build gperf ccache dfu-util \
    device-tree-compiler libsdl2-dev libmagic1 udev libusb-1.0-0 \
    python3 python3-dev python3-pip python3-setuptools python3-tk python3-wheel software-properties-common \
    usbutils openocd clang-tidy\
    python3-venv locales && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Micro editor
RUN curl https://getmic.ro | bash && mv micro /usr/local/bin/

# Set default shell and working directory
SHELL ["/bin/bash", "-c"]
WORKDIR /workspace

CMD ["bash"]
