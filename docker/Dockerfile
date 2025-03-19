################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: Nvidia CUDE 12.4.0 runtime image on Ubuntu 22.04
#       - https://hub.docker.com/layers/nvidia/cuda/12.4.0-runtime-ubuntu22.04/images/sha256-2d205a50fefc321aca51a1d3aa46fb9a633dd2509a10e28c8bd34a6cb2027619
################################################################################################

FROM nvidia/cuda:12.4.0-runtime-ubuntu22.04 AS base

LABEL org.opencontainers.image.title="Docker Image of KochACT-RL"
LABEL org.opencontainers.image.authors="yoseph.huang@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

ARG USERNAME=hrc
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]


################################################################################################
# - Python 3.10 Installation stage
#   - In this stage, I will install python3.10 and the required dependencies.
#   - Building tools will be removed after installation.
################################################################################################

FROM base AS python3.10

RUN apt-get update && apt-get install -y \
    python3.10 \
    python3.10-dev \
    python3.10-distutils \
    python3.10-venv \
    curl \
    libbz2-dev \
    libffi-dev \
    liblzma-dev \
    libncurses5-dev \
    libreadline-dev \
    libsqlite3-dev \
    libssl-dev \
    wget \
    xz-utils \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# Set python3.10 as the default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1


###############################################################################################
# - ROS Humble & Realsense Installation stage
#   - In this stage, I will install ROS2 humble and Intel Realsense dependencies.
################################################################################################

FROM python3.10 AS ros-humble

ARG ROS_DISTRO=humble

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
        /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null 
    
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-realsense2* && \
    ros-$ROS_DISTRO-librealsense2* && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_LOG_DIR=/dev/null" >> ~/.bashrc && \
    rm -rf /var/lib/apt/lists/*


################################################################################################
# - Dev Tools Installation stage
#   - In this stage, I will install some developing tools.
################################################################################################

FROM ros-humble AS dev-tools

# TODO: Install lerobot dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /
# RUN rm -rf lerobot


################################################################################################
# - Release Stage
#   - In this stage, I will configure the final image for release.
################################################################################################

FROM dev-tools AS release

# RUN echo 'export PYTHONPATH="/usr/local/lib/python3.10/dist-packages:/root/lerobot:$PYTHONPATH"' >> ~/.bashrc


CMD ["/bin/bash"]