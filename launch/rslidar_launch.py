from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rslidar_config_file = get_package_share_directory('smb_bringup') + '/config/rslidar_config.yaml'
    
    rslidar = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        parameters=[{'config_path': rslidar_config_file}]
    )
    
    return LaunchDescription([
        rslidar
    ])
