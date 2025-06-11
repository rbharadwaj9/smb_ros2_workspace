from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Run speed_control_node from package roboteq_controller
    speed_control_group = GroupAction([
        Node(namespace='roboteq_controller', 
            package='roboteq_controller', 
            executable='speed_control_node'
        )
    ])


    # Differential drive  launch group
    diff_drive_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("smb_kinematics"), "launch", "smb_differential_drive.launch.py"
                ])
            ),
        ),
    ])



    return LaunchDescription(
        [
            speed_control_group,
            diff_drive_group,
        ]
    )
