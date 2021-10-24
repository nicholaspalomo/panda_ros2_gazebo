#!/usr/bin/env bash

### You can use this script to launch the project from the root of the catkin workspace by calling, e.g.:
## $ ./src/panda_ros2_gazebo/docker/run.bash panda_ros2_gazebo teleop
## to run the 'teleop' example

### Note - need to have Nvidia Docker Toolkit installed:
# # Docker
# curl https://get.docker.com | sh \
#   && sudo systemctl --now enable docker
# # Nvidia Docker
# distribution=$(. /etc/os-release; echo $ID$VERSION_ID) \
#   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
#   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
# sudo apt-get update && sudo apt-get install -y nvidia-docker2
# sudo systemctl restart docker

if [ $# -lt 1 ]; then
    echo "Usage: $0 <docker image> <demo name>"
    exit 1
fi

IMG=$1
ARGS=("$@")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

DOCKER_OPTS=""

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

docker run -it \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "$PWD:/root/workspace" \
    --network host \
    --ipc host \
    --rm \
    -it \
    --privileged \
    --security-opt seccomp=unconfined \
    $DOCKER_OPTS \
    $IMG \
    ${@:2}