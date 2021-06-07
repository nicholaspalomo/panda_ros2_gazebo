from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

# TODO: Launch this demo automatically from the panda_ros2_gazebo.launch.py based on input command line argument from the user

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    default_model_path = os.path.join(pkg_share,
        "description",
        "models",
        "panda",
        "panda.urdf",
    )
    
    panda_pick_and_place_node = launch_ros.actions.Node(
        package='panda_ros2_gazebo',
        # namespace=...
        executable='panda_ros2_gazebo_node',
        name='sim'
    )

    return launch.LaunchDescription([
        panda_pick_and_place_node
    ])