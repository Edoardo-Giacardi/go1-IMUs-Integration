# GO1 IMU Sensor Integration

## Overview
This repository contains the Dockerfile, Python code, and ROS 2 packages necessary for reading data from four Adafruit BNO085 IMU sensors connected to a Raspberry Pi 5, generating ROS 2 messages. 
The repository is structured to be cloned into the `src` directory of your go1 ROS 2 workspace.

### Contents
- **ROS 2 Packages**: Two custom ROS 2 packages to handle IMU data processing and message generation.
- **Dockerfile**: Configuration to build a Docker image suitable for deployment on a Raspberry Pi 5, ensuring a consistent environment for running the ROS 2 nodes.
- **Scripts**: Python scripts included for additional data processing, setup tasks, testing and custom messages.

## Prerequisites
- **ROS 2 Installation**: Ensure that ROS 2 Humble is installed on your system. 
- **Docker**: For building and running containers based on the provided Dockerfile.
- **Python**: Python 3.8 or newer is required for executing the included scripts.

## Installation

### Cloning the Repository
Clone this repository directly into the `src` directory of your `go1` ROS workspace:

