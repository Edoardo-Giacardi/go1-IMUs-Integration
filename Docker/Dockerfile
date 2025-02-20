
# =============================================
#  Dockerfile
# =============================================


# ARM64 Ubuntu 22.04
FROM arm64v8/ubuntu:jammy

# Create build args for user and workspace
ARG IMAGE=$IMAGE
ARG TAG=$TAG
ARG USERNAME=$USERNAME
ARG USER_UID=$USER_UID
ARG USER_GID=$USER_GID
ARG WORKSPACE=$WORKSPACE

# Use Bash in all RUN instructions
SHELL ["/bin/bash", "-c"]

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# System Setup: locales, apt keys, etc.
RUN apt-get update && \
    apt-get install -y \
    locales \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up UTF-8 locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Download the ROS key and add it to the keyring
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg

# Add the ROS repository
RUN echo "deb [arch=arm64 signed-by=/etc/apt/trusted.gpg.d/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt-get update \
    && apt-get install -y \
    ros-humble-desktop \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-xacro \
    ros-humble-controller-interface \
    ros-humble-realtime-tools \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-control-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-control-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-rqt-reconfigure \
    ros-humble-rviz-visual-tools \
    ros-humble-joint-state-publisher \
    ros-humble-velodyne-description \
    ros-humble-plotjuggler-ros \
    tmux python3-pip \
    python3-setuptools \
    i2c-tools \
    xterm \
    libeigen3-dev \
    nano \
    ros-humble-rviz2 \
    nautilus \
    && rm -rf /var/lib/apt/lists/*

# Install the python packages
RUN pip3 install --upgrade \
    numpy \
    numpy-quaternion \
    quadprog \
    adafruit-circuitpython-bno08x \
    adafruit-extended-bus \
    scipy \
    RPi.GPIO 


# Let the container know the home directory for the new user
ENV HOME=/home/$USERNAME

# Debug: print
RUN echo "USERNAME=$USERNAME" && echo "WORKSPACE=$WORKSPACE"

# Create the new user + give sudo
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get update && apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Add user to i2c group
RUN usermod -aG i2c $USERNAME
    
# Prepare Workspace
RUN mkdir -p /home/$USERNAME/$WORKSPACE/src
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

# Switch to non-root user
ENV USER=$USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME/$WORKSPACE

# install openssh-client and git
RUN sudo apt-get update && sudo apt-get install -y \
    apt-utils \
    git \
    openssh-client &&\
    git clone https://github.com/Edoardo-Giacardi/go1-IMUs-Integration.git /home/$USERNAME/$WORKSPACE/src

# rosdep: install deps + colcon + build
RUN cd /home/$USERNAME/$WORKSPACE && sudo apt-get install -y python3-rosdep && \
    sudo apt-get install -y python3-rosdep python3-colcon-common-extensions && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -r -y && \
    source /opt/ros/humble/setup.bash && \
    colcon build

RUN echo 'parse_git_branch() {' >> ~/.bashrc && \
    echo " git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/[\\1]/'" >> ~/.bashrc && \
    echo '}' >> ~/.bashrc

RUN echo "PS1='<Docker: ${IMAGE}:${TAG}>\\[\\033[1;31m\\]\\u\\[\\033[1;37m\\]@\\[\\033[1;32m\]\\h\[\\033[1;37m\\]:\\[\\033[1;36m\\]\\w\\[\\033[01;33m\\]\$(parse_git_branch)\\[\\033[00m\\]\\$ \\[\\033[0m\\]'" >> ~/.bashrc

# Source ROS & workspace in every new shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/${WORKSPACE}/install/setup.bash ]; then source ~/${WORKSPACE}/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]
