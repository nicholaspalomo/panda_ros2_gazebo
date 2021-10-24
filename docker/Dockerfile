### To build this docker container, run:
### docker build -t panda_ros2_gazebo .

ARG PARENT_IMAGE=nvidia/cuda:11.0.3-cudnn8-devel-ubuntu20.04
FROM ${PARENT_IMAGE}

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Setup timezone (in order to prevent interactive install)
ARG TIMEZONE=America/Los_Angeles
ARG DEBIAN_FRONTEND=noninteractive
RUN rm -f /etc/localtime && \
    ln -s /usr/share/zoneinfo/"${TIMEZONE}" /etc/localtime && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        locales \
        locales-all \
        tzdata && \
    rm -rf /var/lib/apt/lists/*

### Install toolchain
ARG PYTHON3_VERSION=3.8
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
        apt-utils \
        software-properties-common \
        git \
        curl \
        wget \
        cmake \
        ninja-build \
        build-essential \
        gnupg2 \
        lsb-release \
        xterm \
        swig \
        python3-pip \
        python3-wheel \
        python${PYTHON3_VERSION} \
        python${PYTHON3_VERSION}-dev \
        libpython${PYTHON3_VERSION}-dev && \
    rm -rf /var/lib/apt/lists/*

### Install ros2
ARG ROS2_DISTRO=foxy
ENV ROS2_DISTRO=${ROS2_DISTRO}

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        ros-${ROS2_DISTRO}-ros-base \
        ros-${ROS2_DISTRO}-xacro \
        ros-${ROS2_DISTRO}-gazebo-dev \
        ros-${ROS2_DISTRO}-gazebo-ros \
        ros-${ROS2_DISTRO}-control-toolbox \
        ros-${ROS2_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS2_DISTRO}-test-msgs \
        ros-${ROS2_DISTRO}-rviz2 \
        ros-${ROS2_DISTRO}-joint-state-publisher && \
    pip3 --default-timeout=100 install \
        argcomplete \
        scipy && \
    rm -rf /var/lib/apt/lists/*

### Set the paths for the project
ARG DEPS_DIR=/root/deps
ENV DEPS_DIR=${DEPS_DIR}
ARG WORKSPACE_DIR=/root/workspace
ENV WORKSPACE_DIR=${WORKSPACE_DIR}
ARG PANDA_ROS2_GAZEBO_DIR=${WORKSPACE_DIR}/src/panda_ros2_gazebo
ENV PANDA_ROS2_GAZEBO_DIR=${PANDA_ROS2_GAZEBO_DIR}

### Install iDynTree
RUN mkdir -p ${DEPS_DIR} && \
    cd ${DEPS_DIR} && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        libeigen3-dev \
        libxml2-dev \
        coinor-libipopt-dev \
        qtbase5-dev \
        qtdeclarative5-dev \
        qtmultimedia5-dev \
        qml-module-qtquick2 \
        qml-module-qtquick-window2 \
        qml-module-qtmultimedia \
        qml-module-qtquick-dialogs \
        qml-module-qtquick-controls \
        qml-module-qt-labs-folderlistmodel \
        qml-module-qt-labs-settings && \
    rm -rf /var/lib/apt/lists/*
    # TODO: Should IDYNTREE_USES_PYTHON be True? Should BUILD_SHARED_LIBS be True?

### Install colcon, vcs toolkit and rosdep prior to building ROS-based projects
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep && \
    rosdep init && \
    rosdep update
# Note: Do not remove apt lists until last ROS-based project is built

### Clone this repository and import the repos required to build/run the project
RUN mkdir -p ${PANDA_ROS2_GAZEBO_DIR} && \
    cd ${PANDA_ROS2_GAZEBO_DIR}/.. && \
    git clone https://github.com/nicholaspalomo/panda_ros2_gazebo.git && \
    vcs import < panda_ros2_gazebo/workspace.repos && \
    rm -rf /var/lib/apt/lists/*

### Add entrypoint
COPY ./entrypoint.bash ./entrypoint.bash
RUN chmod +x ./entrypoint.bash

ENTRYPOINT ["/bin/bash", "entrypoint.bash"]
CMD ["/bin/bash"]
