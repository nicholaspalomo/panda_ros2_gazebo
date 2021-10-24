# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    name = LaunchConfiguration('mode', default='')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    parameter_file_path = os.path.join(pkg_share,
        "config",
        "params.yaml"
    )
    
    panda_node = launch_ros.actions.Node(
        package='panda_ros2_gazebo',
        executable='panda',
        name='panda',
        parameters=[
            parameter_file_path,
            {'share_dir' : pkg_share}
        ],
        output='screen',
        arguments=['mode', name]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mode',
            default_value='follow',
            description='Name of demo to run - see runner.py for options'),
        panda_node
    ])