#!/usr/bin/env bash
set -e

cd ${WORKSPACE_DIR}

source /opt/ros/${ROS2_DISTRO}/setup.bash

if [ "$1" = "build" ]; then

    MAKEFLAGS="-j1" colcon build --merge-install --parallel-workers 1 --cmake-args -DIDYNTREE_USES_PYTHON=True -DIDYNTREE_USES_IPOPT:BOOL=ON -DCMAKE_BUILD_TYPE=Release

else

    source install/setup.bash

    if [ "$1" = "teleop" ]; then
        ros2 launch panda_ros2_gazebo gazebo.launch.py & ros2 launch panda_ros2_gazebo bringup.launch.py mode:=$1 & ros2 launch panda_teleop panda_teleop_control.launch.py & wait
    else
        ros2 launch panda_ros2_gazebo gazebo.launch.py & ros2 launch panda_ros2_gazebo bringup.launch.py mode:=$1 & wait
    fi

fi