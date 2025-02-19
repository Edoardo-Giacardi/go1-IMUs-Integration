#!/usr/bin/env bash

# Immediately exit if any command within the script returns a non-zero exit status
set -e

# Define necessary variables directly in the script
IMAGE="4IMU_I2C"    # Docker image name (should match the one built via docker_buil.bash)
TAG="1.0"                             # Docker image tag
CMD_INTERACTIVE="/bin/bash"           # Command to run in interactive mode
HOSTNAME="localhost"                  # Hostname for the container
USERNAME="ros"                        # User name inside the container
WORKSPACE="ros2_ws"                   # Workspace directory inside the container


# Usage message
USAGE="Usage: $0 [OPTIONS...]
Options:
  -h, --help           Show help options
  -i, --interactive    Run docker in interactive mode (default command: $CMD_INTERACTIVE)
  -t, --tag <tag>      Select a specific docker tag (default: $TAG)
  -n, --name <name>    Set a specific container name (default: \$IMAGE)
"

# Print help info if requested
if [[ ( $1 == "--help") ||  $1 == "-h" ]]
then
echo -e $USAGE
exit 0
fi



# Read command line arguments
while [ -n "$1" ]; do # while loop starts
case "$1" in
-i|--interactive)
CMD=$CMD_INTERACTIVE
;;
-t|--tag)
TAG="$2"
shift
;;
-n|--name)
CONTAINER_NAME="$2"
shift
;;
*) echo "Option $1 not recognized!"
echo -e $USAGE
exit 0;;
esac
shift
done

CONTAINERS=$(docker ps --format "{{.Names}}")

# Print help info if requested
if [[ -z $CONTAINER_NAME ]]; then
CONTAINER_NAME=$IMAGE
elif [[ ${CONTAINERS[@]} =~ $CONTAINER_NAME  ]]; then
echo -e "The container name: $CONTAINER_NAME already exists. Choose another name."
echo -e $USAGE
exit 0
fi

echo -e "${COLOR_WARN}Running container: ${COLOR_DEFAULT}$CONTAINER_NAME"

# Run docker    
docker run \
  --hostname $HOSTNAME \
--name $CONTAINER_NAME \
--net=host \
--rm \
-w /home/$USERNAME/$WORKSPACE \
-it $IMAGE:$TAG $SHELL \
-c "$CMD"
