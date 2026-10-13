FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Non-root user matching host UID/GID so files created in the container are owned by the host user.
# (see https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Base tooling + every system / ROS dependency the bundled packages declare in their package.xml.
# Pre-installing here means `rosdep install` at postCreate is a no-op fast-path on warm containers.
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        git \
        curl \
        unzip \
        python3-pip \
        python3-tk \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-skimage \
        python3-scipy \
        python3-numpy \
        python3-pandas \
        python3-matplotlib \
        python3-pil \
        libignition-common3-dev \
        libfmt-dev \
        libgoogle-glog-dev \
        libceres-dev \
        libjsoncpp-dev \
        libyaml-cpp-dev \
        libeigen3-dev \
        ros-humble-desktop \
        ros-humble-ackermann-msgs \
        ros-humble-geodesy \
        ros-humble-foxglove-bridge \
        ros-humble-xacro \
        ros-humble-joy \
        ros-humble-angles \
        ros-humble-velodyne-driver \
        ros-humble-rqt-gui \
        ros-humble-tf2-geometry-msgs \
        ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

# Python deps the upstream Dockerfile pins.
RUN pip3 install --no-cache-dir xacro tqdm pytest-cov "setuptools==68"

# Create the vscode user with passwordless sudo and access to video devices (for any GPU/X passthrough).
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -a -G video ${USERNAME}

# Initialise rosdep once at build time so `rosdep install` runs quickly later.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

# Auto-source ROS for every interactive shell (root + vscode).
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "[ -f /\${PWD##*/}/install/setup.bash ] && source /\${PWD##*/}/install/setup.bash" >> /etc/bash.bashrc

# Foxglove bridge default port — exposed for documentation; the devcontainer also forwards it.
EXPOSE 8765

USER ${USERNAME}
