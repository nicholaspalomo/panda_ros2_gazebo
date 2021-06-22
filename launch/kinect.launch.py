#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros

# Borrowed from: https://bitbucket.org/theconstructcore/box_bot/src/foxy/box_bot_description/launch/spawn_robot_launch_v3.launch.py
def generate_launch_description():
    pose = [LaunchConfiguration('x', default='1.0'), LaunchConfiguration('y', default='0.0'), LaunchConfiguration('z', default='0.25'),
    LaunchConfiguration('roll', default='0.0'), LaunchConfiguration('pitch', default='0.0'), LaunchConfiguration('yaw', default='3.14159265359')]
    default_namespace = LaunchConfiguration('robot_namespace', default='kinect')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    sensor_model_path = os.path.join(pkg_share,
        "description",
        "models")
    kinect_urdf_path = os.path.join(sensor_model_path, "kinect", "kinect.xacro")

    robot_state_publisher_node = launch_ros.actions.Node(
        namespace=default_namespace,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', kinect_urdf_path])}]
    )

    spawn_entity = launch_ros.actions.Node(
        namespace=default_namespace,
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "kinect", "-x", pose[0], "-y", pose[1], "-z", pose[2], "-R", pose[3], "-P", pose[4], "-Y", pose[5]],
        output="screen",
    )

    spawn_joint_state_broadcaster = launch_ros.actions.Node(
        namespace=default_namespace,
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-type", "joint_state_broadcaster/JointStateBroadcaster"],
        output="screen",
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        namespace=default_namespace,
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='kinect',
            description='namespace of the robot'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='1.0',
            description='Initial pose of kinect - x'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial pose of kinect - y'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.25',
            description='Initial pose of kinect - z'
        ),
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Initial pose of kinect - roll'
        ),
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Initial pose of kinect - pitch'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='3.14159265359',
            description='Initial pose of kinect - yaw'
        ),
        spawn_joint_state_broadcaster,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity
    ])