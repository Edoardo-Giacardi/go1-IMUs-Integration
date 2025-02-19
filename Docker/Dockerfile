# base image osrf/ros tag humble-desktop-full
ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=humble-desktop-full
FROM osrf/ros:humble-desktop-full

# Use "bash" as replacement for "sh"
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV DEBIAN_FRONTEND=noninteractive

## Ignition Gazebo Fortress ###


RUN sudo apt-get update \
&& sudo apt-get upgrade -y && \
sudo apt-get install -y lsb-release wget gnupg && \
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
sudo apt-get update && \
sudo apt-get install ignition-fortress -y


RUN sudo apt-get update && sudo apt-get upgrade -y


#ros_ign
RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
sudo apt-get update && \
sudo apt install ros-humble-ros-gz -y

RUN export GZ_VERSION=fortress

RUN apt-get update \
    && apt-get install -y \
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
    ros-humble-ros-ign-bridge \
    ros-humble-ign-ros2-control \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-rqt-reconfigure \
    ros-humble-rviz-visual-tools \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-velodyne-description \
    ros-humble-plotjuggler-ros \
    tmux python3-pip\
    python3-setuptools \
    i2c-tools \
    xterm \
    libeigen3-dev \
    nano \
    ros-humble-rviz2 \
    nautilus

# Adapt your desired python version here    
# ENV PATH=/opt/openrobots/bin:$PATH
# ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
# ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
# ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH  
# ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
# ENV TERM=xterm-256color

# ENV DEBIAN_FRONTEND=dialog

# # Create a new user
ARG USERNAME=$USERNAME
ARG USER_UID=$USER_UID
ARG USER_GID=$USER_UID
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME

#Change HOME environment variable
ENV HOME /home/$USERNAME

# Choose to run as user
# ENV USER $USERNAME

# USER $USERNAME

# # Install the python packages
RUN pip3 install \
    numpy \
    numpy-quaternion \
    quadprog \
    adafruit-circuitpython-bno08x \
    adafruit-extended-bus \
    scipy \
    --upgrade

# Set up auto-source of workspace for ros user
ARG WORKSPACE=$WORKSPACE

# Create the directory
# Debug: Print environment variables
ARG USERNAME=$USERNAME

RUN echo "USERNAME=$USERNAME" && echo "WORKSPACE=$WORKSPACE"

# Try creating the directory
RUN mkdir -p /home/$USERNAME/$WORKSPACE/src && ls -ld /home/$USERNAME/$WORKSPACE/src

RUN mkdir -p /home/$USERNAME/$WORKSPACE/src

### ------------------------------------------------------------- ###
### Use SSH build secret to clone the repositories and submodules ###
### ------------------------------------------------------------- ###

# install openssh-client and git
RUN sudo apt-get -y update
RUN sudo apt-get install -y  \
    apt-utils \
    git \
    openssh-client

# Install necessary dependencies
RUN sudo apt-get -y update && sudo apt-get install -y git openssh-client

# Set up SSH for GitHub and add the host to known hosts
RUN mkdir -p -m 0700 /home/$USERNAME/.ssh && sudo ssh-keyscan github.com >> /home/$USERNAME/.ssh/known_hosts


# Clone the repository using SSH
#RUN --mount=type=ssh GIT_SSH_COMMAND="ssh -i /home/$USERNAME/.ssh/id_ed25519 -o StrictHostKeyChecking=no" \
#   git clone git@github.com:LeoBoticsHub/rbkairos_description.git --branch devel_simulation /home/$USERNAME/$WORKSPACE/src

RUN git clone https://github.com/Edoardo-Giacardi/go1-IMUs-Integration.git /home/$USERNAME/$WORKSPACE/src

# update the submodules in the repository
#RUN --mount=type=ssh cd /home/$USERNAME/$WORKSPACE/src && git submodule update --init --recursive


### ------------------------------------------------------------- ###
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH  
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
ENV TERM=xterm-256color

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Ensure the correct user owns their home directory and workspace
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/$WORKSPACE

ENV USER $USERNAME

USER $USERNAME

# Ensure the user has access to the .ros directory
RUN sudo mkdir -p /home/$USERNAME/.ros && sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.ros

RUN cd /home/$USERNAME/$WORKSPACE && sudo apt install -y python3-rosdep && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    source /opt/ros/humble/setup.bash && \
    colcon build

RUN echo 'parse_git_branch() {' >> ~/.bashrc && \
    echo " git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/[\\1]/'" >> ~/.bashrc && \
    echo '}' >> ~/.bashrc
ARG IMAGE
ARG TAG
RUN echo "PS1='<Docker: ${IMAGE}:${TAG}>\\[\\033[1;31m\\]\\u\\[\\033[1;37m\\]@\\[\\033[1;32m\]\\h\[\\033[1;37m\\]:\\[\\033[1;36m\\]\\w\\[\\033[01;33m\\]\$(parse_git_branch)\\[\\033[00m\\]\\$ \\[\\033[0m\\]'" >> ~/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo 'source /usr/share/gazebo/setup.bash' >> ~/.bashrc
RUN echo "if [ -f ~/${WORKSPACE}/install/setup.bash ]; then source ~/${WORKSPACE}/install/setup.bash; fi" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
