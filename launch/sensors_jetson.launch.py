from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rgb_camera_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("smb_bringup"), "launch", "rgb_camera_driver.launch.py"
                ])
            ),
            launch_arguments={
                'serial': "'20023237'" # for Jetson only
            }.items()
        )
    ])

    container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='rgb_camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
                remappings=[
                    ('image_raw', 'rgb_camera/image_raw'),
                    ('image_color', 'rgb_camera/image_debayered')
                ]
            )
        ],
        output='screen',
        parameters=[
                {"use_sim_time": False},
            ],
    )


    return LaunchDescription(
        [
            rgb_camera_group,
            container
        ]
    )