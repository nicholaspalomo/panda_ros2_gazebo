# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    default_camera_name = LaunchConfiguration('camera_name', default='kinect')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    default_model_path = os.path.join(pkg_share,
        "description",
        "models"
    )
    default_urdf_path = os.path.join(default_model_path, "panda", "panda.urdf")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz.rviz')
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(), # "extra_gazebo_args": "-s libgazebo_ros_p3d.so"
    )
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(),
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "panda"],
        output="screen",
    )
    effort_controller_config = os.path.join(
        get_package_share_directory("panda_ros2_gazebo"), "config", "ros_control.yaml"
    )
    spawn_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_group_position_controller", "--param-file", effort_controller_config, "--controller-type", "effort_controllers/JointGroupPositionController"],
        output="screen",
    )
    spawn_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-type", "joint_state_broadcaster/JointStateBroadcaster"],
        output="screen",
    )
    gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[default_model_path])
    gazebo_media_path = SetEnvironmentVariable(name='GAZEBO_MEDIA_PATH', value=[default_model_path])

    kinect_urdf_path = os.path.join(default_model_path, "kinect", "kinect.urdf")
    spawn_kinect_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "kinect.launch.py")),
        launch_arguments={
            'robot_urdf': kinect_urdf_path,
            'x': TextSubstitution(text=str(1.5)),
            'y': TextSubstitution(text=str(0.0)),
            'z': TextSubstitution(text=str(0.25)),
            'roll' : TextSubstitution(text=str(0.0)),
            'pitch' : TextSubstitution(text=str(-0.09817477042)), # -pi/32
            'yaw' : TextSubstitution(text=str(3.14159265359)), # pi
            'robot_name': default_camera_name,
            'robot_namespace': default_camera_name
            }.items()
    )

    launch_description = launch.LaunchDescription([
        gazebo_model_path,
        gazebo_media_path,
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='gui', default_value='false',
                                             description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='camera_name', default_value='kinect',
                                             description='Default camera name'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_urdf_path,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        spawn_joint_state_broadcaster,
        robot_state_publisher_node,
        rviz_node,
        gzclient,
        gzserver,
        spawn_entity,
        spawn_controller
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
    ])

    launch_description.add_action(spawn_kinect_launch_description)

    return launch_description