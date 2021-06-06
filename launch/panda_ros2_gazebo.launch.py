# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    robot_description_path = os.path.join(
        get_package_share_directory("panda_ros2_gazebo"),
        "description",
        "models",
        "panda",
        "panda.urdf",
    )
    robot_description_config = xacro.parse(open(robot_description_path))
    xacro.process_doc(robot_description_config)

    robot_description = {"robot_description": robot_description_config.toxml()}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("panda_ros2_gazebo"), "rviz", "rviz.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "panda"],
        output="screen",
    )

    effort_controller_config = os.path.join(
        get_package_share_directory("panda_ros2_gazebo"), "config", "ros_control.yaml"
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_group_effort_controller", "--param-file", effort_controller_config, "-t", "effort_controllers/JointGroupEffortController"],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            # spawn_controller,
        ]
    )
