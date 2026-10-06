FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Rome
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8


RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    ninja-build \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    locales \
    unzip \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8


RUN apt-get update && apt-get install -y curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep tag_name | cut -d '"' -f4) && \
    curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-dev-tools


RUN rosdep init || true
RUN rosdep update


RUN pip3 install --break-system-packages \
    empy==3.3.4 \
    pyros-genmsg \
    setuptools

WORKDIR /opt

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

RUN yes | bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh

RUN mkdir -p /root/ros2_px4_ws/src

WORKDIR /root/ros2_px4_ws/src

RUN git clone https://github.com/PX4/px4_msgs.git

RUN git clone -b v2.4.3 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

WORKDIR /root/ros2_px4_ws

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build


RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_px4_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]