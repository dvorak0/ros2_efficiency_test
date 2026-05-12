FROM ubuntu:24.04

ARG DIR_NAME=project
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    WS=/ros2_ws \
    ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs openssh-client \
    nano vim less tree htop tmux curl wget zip unzip \
    libglib2.0-0 libgl1 libegl1 \
    locales gnupg2 lsb-release ca-certificates software-properties-common \
    python3-pip linux-tools-common linux-tools-generic procps \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository universe \
 && rm -rf /var/lib/apt/lists/*


# ros2
RUN apt-get update && apt-get install -y software-properties-common \
    && rm -rf /var/lib/apt/lists/*
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-iceoryx-posh \
    ros-${ROS_DISTRO}-iceoryx-binding-c \
    ros-${ROS_DISTRO}-iceoryx-hoofs \
    ros-${ROS_DISTRO}-pcl-conversions \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ament-cmake-target-dependencies \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init

RUN PERF_BIN="$(ls /usr/lib/linux-tools/*/perf 2>/dev/null | head -n 1)"; \
    if [ -n "${PERF_BIN}" ]; then ln -sf "${PERF_BIN}" /usr/local/bin/perf; \
    else echo "WARN: no perf binary found under /usr/lib/linux-tools" >&2; fi

# WORKDIR is the same as the project folder name
WORKDIR ${WS}

COPY ros2_ws/src ./src

RUN apt-get update \
 && rosdep update --rosdistro=${ROS_DISTRO} \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro=${ROS_DISTRO} \
 && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
                  && colcon build \
                       --packages-select shm_msgs ros2_efficiency_test ros2_efficiency_test_py ros2_efficiency_test_shm_py \
                       --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Drop in the runner script.
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
