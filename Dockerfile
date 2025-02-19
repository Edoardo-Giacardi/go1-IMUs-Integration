# Use the official ROS2 Humble base image
FROM osrf/ros:humble-desktop

# Set the working directory
WORKDIR /workspace

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    i2c-tools

# Install Adafruit BNO085 library
RUN pip3 install adafruit-circuitpython-bno08x

# Clone your GitHub repository
ARG GITHUB_REPO_URL
RUN git clone ${GITHUB_REPO_URL} /workspace/repo

# Set up automatic pulling of the repo when the container starts
ENTRYPOINT ["git", "-C", "/workspace/repo", "pull", "&&", "bash"]

# Additional command to display Docker image and container name
CMD echo "Image: $ROS_IMAGE_NAME | Container: $HOSTNAME"

