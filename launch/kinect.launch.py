#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Launch description for the Kinect camera sensor
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros

# Borrowed from: https://bitbucket.org/theconstructcore/box_bot/src/foxy/box_bot_description/launch/spawn_robot_launch_v3.launch.py
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    sensor_model_path = os.path.join(pkg_share,
        "description",
        "models")
    kinect_urdf_path = os.path.join(sensor_model_path, "kinect", "kinect.urdf")

    robot_state_publisher_node = launch_ros.actions.Node(
        namespace='kinect',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', kinect_urdf_path])}]
    )

    spawn_entity = launch_ros.actions.Node(
        namespace='kinect',
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "kinect"],
        output="screen",
    )

    spawn_joint_state_broadcaster = launch_ros.actions.Node(
        namespace='kinect',
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-type", "joint_state_broadcaster/JointStateBroadcaster"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        spawn_entity,
        spawn_joint_state_broadcaster
    ])