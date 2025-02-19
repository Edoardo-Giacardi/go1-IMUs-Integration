#!/usr/bin/env bash

# ===========================================
# docker_build.bash
# ===========================================
# This script defines build arguments and
# builds the Docker image with a single
# command.
# ===========================================

# -- Build Arguments --
USERNAME="ros"
USER_UID="1000"
USER_GID="1000"
WORKSPACE="ros2_ws"
IMAGE="ros2_mipo"
TAG="latest"

# -- Docker Build --
docker build \
    --build-arg USERNAME="${USERNAME}" \
    --build-arg USER_UID="${USER_UID}" \
    --build-arg USER_GID="${USER_GID}" \
    --build-arg WORKSPACE="${WORKSPACE}" \
    --build-arg IMAGE="${IMAGE}" \
    --build-arg TAG="${TAG}" \
    -t "${IMAGE}:${TAG}" \
    .
