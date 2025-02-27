################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: Nvidia CUDE 12.4.0 runtime image on Ubuntu 22.04
#       - https://hub.docker.com/layers/nvidia/cuda/12.4.0-runtime-ubuntu22.04/images/sha256-2d205a50fefc321aca51a1d3aa46fb9a633dd2509a10e28c8bd34a6cb2027619
################################################################################################

FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04 AS base

LABEL org.opencontainers.image.title="Docker Image of KochACT-RL"
LABEL org.opencontainers.image.authors="yoseph.huang@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# ARG USERNAME=hrc
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive


################################################################################################
# - Python Building stage
#   - In this stage, I will install and set python3.8.10 as the default python version.
#   - Building tools will be removed after this stage.
################################################################################################

FROM base AS python-build

RUN apt-get update && apt-get install -y \
    cmake \ 
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

# Build Python 3.10.0 from source
RUN wget https://www.python.org/ftp/python/3.10.0/Python-3.10.0.tgz && \
    tar -xvf Python-3.10.0.tgz && \
    cd Python-3.10.0 && \
    ./configure --enable-optimizations && \
    make -j$(nproc) && \
    make altinstall && \
    cd .. && rm -rf Python-3.10.0 Python-3.10.0.tgz


################################################################################################
# - Python Dependencies Installation stage
#   - In this stage, I will install all the python dependencies.
#       - lerobot github repository: https://github.com/hrc-pme/lerobot/blob/963738d983480b1cd19295b2cb0630d0cf5c5bb5/examples/7_get_started_with_real_robot.md
################################################################################################

FROM base AS python-dep

COPY --from=python-build /usr/local/lib/python3.10 /usr/local/lib/python3.10
COPY --from=python-build /usr/local/bin/python3.10 /usr/local/bin/python3.10
COPY --from=python-build /usr/local/bin/pip3.10 /usr/local/bin/pip3


# Set Python 3.10.0 as default
RUN ln -sf /usr/local/bin/python3.10 /usr/bin/python3 && \
    ln -sf /usr/local/bin/pip3.10 /usr/bin/pip

# Install build tools 
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install lerobot dependencies
RUN git clone https://github.com/hrc-pme/lerobot.git /root/lerobot && \
    cd /root/lerobot && \
    git checkout 963738d

WORKDIR /root/lerobot
RUN apt-get update && apt-get upgrade pip -y && \
    pip install --upgrade pip && \
    pip install --no-cache-dir --ignore-installed ".[dynamixel]" 

WORKDIR /root
RUN rm -rf lerobot


################################################################################################
# - ROS Humble & Realsense Installation stage
#   - In this stage, I will install ROS2 humble and Intel Realsense dependencies.
################################################################################################


FROM python-dep AS ros-humble-base

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
        /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null 
    
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-librealsense2* && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*


################################################################################################
# - Dev Tools Installation stage
#   - In this stage, I will install some developing tools.
################################################################################################

FROM ros-humble-base AS release

## Fix some minor issues
# RUN pip3 install setuptools==58.2.0

CMD ["/bin/bash"]