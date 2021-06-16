import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    name = LaunchConfiguration('name', default='teleop')
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
        arguments=['name', name]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'name',
            default_value='teleop',
            description='Name of demo to run - follow or teleop'),
        panda_node
    ])