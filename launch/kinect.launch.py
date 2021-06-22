#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Launch description for the Kinect camera sensor
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros

# Borrowed from: https://bitbucket.org/theconstructcore/box_bot/src/foxy/box_bot_description/launch/spawn_robot_launch_v3.launch.py
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    sensor_model_path = os.path.join(pkg_share,
        "description",
        "models")
    kinect_urdf_path = os.path.join(sensor_model_path, "kinect", "kinect.urdf")

    with open(kinect_urdf_path, 'r') as infp:
        kinect_description = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='panda_ros2_gazebo', executable='spawn_kinect', arguments=[kinect_urdf_path], output='screen', namespace='kinect'),
        Node(
            namespace='kinect',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', kinect_urdf_path])}]),
    ])