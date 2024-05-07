from launch import LaunchDescription, launch_description_sources
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_path = get_package_share_directory('foxglove_info_publisher')
    return LaunchDescription([
        SetEnvironmentVariable(name='BOUNDARY_DIR', value=package_path+'/boundary'),
        Node(
            package='foxglove_info_publisher',
            executable='foxglove_info_publisher_exe',
            output='screen',
        )
    ])