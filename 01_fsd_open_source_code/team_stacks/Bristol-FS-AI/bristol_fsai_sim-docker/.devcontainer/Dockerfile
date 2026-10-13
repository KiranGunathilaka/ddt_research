FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# Add vscode user with same UID and GID as your host system.
# (based on https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Base tools + ROS deps used by this workspace.
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    python3-pip \
    python3-tk \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    python3-scipy \
    ros-humble-gazebo-plugins \
    libyaml-cpp-dev \
    ros-humble-ackermann-msgs \
    gazebo \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -a -G video ${USERNAME}

# Initialize rosdep if needed, then update.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

# Source the ROS setup for all users.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

USER ${USERNAME}

# # Install ZED SDK
# RUN wget -q --show-progress --progress=bar:force:noscroll https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu20 -O ZED_SDK_Ubuntu20.run && \
#         chmod +x ZED_SDK_Ubuntu20.run && \
#         ./ZED_SDK_Ubuntu20.run -- silent && \
#         rm ZED_SDK_Ubuntu20.run

# IPG CarMaker Dependencies
# libusb-1.0-0-dev
# tzdata