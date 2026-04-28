FROM ubuntu:22.04

ARG USER_UID=1000
ARG USER_GID=1000
ARG VIDEO_GID=44
ARG USER_NAME=dev
ARG DIR_NAME=project
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    WS=/ros2_ws \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo build-essential cmake git git-lfs openssh-client \
    nano vim less tree htop tmux curl wget zip unzip \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa \
    locales gnupg2 lsb-release ca-certificates software-properties-common \
    python3-pip linux-tools-common linux-tools-generic procps \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository universe \
 && rm -rf /var/lib/apt/lists/*

# --- create the user (handle primary group GID collisions) ---
RUN set -eux; \
  PRIMARY_GID="${USER_GID}"; \
  PRIMARY_G="$(getent group "${PRIMARY_GID}" | cut -d: -f1 || true)"; \
  if [ -n "${PRIMARY_G}" ]; then \
    echo "Using existing group ${PRIMARY_G} (GID ${PRIMARY_GID}) for ${USER_NAME}"; \
    useradd -m -u "${USER_UID}" -g "${PRIMARY_GID}" -G sudo -s /bin/bash "${USER_NAME}"; \
  else \
    echo "Creating group ${USER_NAME} with GID ${PRIMARY_GID}"; \
    groupadd -g "${PRIMARY_GID}" "${USER_NAME}"; \
    useradd -m -u "${USER_UID}" -g "${PRIMARY_GID}" -G sudo -s /bin/bash "${USER_NAME}"; \
  fi; \
  echo "%sudo ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/nopasswd; \
  chmod 0440 /etc/sudoers.d/nopasswd

# --- ensure container user is in the host's VIDEO_GID ---
RUN set -eux; \
  TARGET_GID="${VIDEO_GID}"; \
  EXIST_G="$(getent group "${TARGET_GID}" | cut -d: -f1 || true)"; \
  if [ -n "${EXIST_G}" ]; then \
    echo "GID ${TARGET_GID} already used by group: ${EXIST_G}; adding ${USER_NAME} to it."; \
    usermod -aG "${EXIST_G}" "${USER_NAME}"; \
  else \
    if getent group video >/dev/null; then \
      echo "Setting 'video' group to GID ${TARGET_GID}"; \
      groupmod -g "${TARGET_GID}" video; \
    else \
      echo "Creating 'video' group with GID ${TARGET_GID}"; \
      groupadd -g "${TARGET_GID}" video; \
    fi; \
    usermod -aG video "${USER_NAME}"; \
  fi


# ros2
RUN apt-get update && apt-get install -y software-properties-common \
    && rm -rf /var/lib/apt/lists/*
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-fastrtps \
    ros-humble-fastrtps-cmake-module \
    ros-humble-iceoryx-posh \
    ros-humble-iceoryx-binding-c \
    ros-humble-iceoryx-hoofs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

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

RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
                  && colcon build \
                       --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Drop in the runner script.
COPY ../entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
USER ${USER_NAME}