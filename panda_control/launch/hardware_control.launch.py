# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_control').find('panda_control')
    default_urdf_path = os.path.join(pkg_share, "description", "models", "panda", "panda.urdf")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz.rviz')
    parameter_file_path = os.path.join(pkg_share, "config", "params.yaml")

    # Rviz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    effort_controller_config = os.path.join(
        get_package_share_directory("panda_control"), "config", "ros_control.yaml"
    )

    # Joint group position controller
    spawn_joint_group_position_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_group_position_controller", "--param-file", effort_controller_config, "--controller-type", "effort_controllers/JointGroupPositionController"],
        output="screen",
    )

    # Joint trajectory controller
    spawn_joint_trajectory_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "--param-file", effort_controller_config, "--controller-type", "joint_trajectory_controller/JointTrajectoryController"],
        output="screen",
    )

    # Simple keyboard control node
    panda_control_node = launch_ros.actions.Node(
        package='panda_control',
        executable='panda',
        name="panda",
        parameters=[
            parameter_file_path,
            {'share_dir' : pkg_share}
        ],
        output='screen',
        arguments=['mode', 'cl_setpoint']
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_urdf_path,
            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),
        rviz_node,
        robot_state_publisher_node,
        # spawn_joint_group_position_controller,
        # spawn_joint_trajectory_controller,
        panda_control_node
    ])