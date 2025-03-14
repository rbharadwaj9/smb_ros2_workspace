from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        DeclareLaunchArgument("simulation", default_value="false", description="Set simulation mode"),
        DeclareLaunchArgument("launch_sensors", default_value="false", description="Launch sensors"),
        DeclareLaunchArgument("use_lidar_odometry", default_value="false", description="Use LiDAR odometry"),
        DeclareLaunchArgument("launch_tracking_cam", default_value="false", description="Launch tracking camera odometry"),
        DeclareLaunchArgument("control_namespace", default_value="control", description="Namespace for the control node"),
        DeclareLaunchArgument("description_name", default_value="smb_description", description="Name for the robot description"),
        DeclareLaunchArgument("command_smb", default_value="true", description="Send twist commands from the software stack to the robot"),
        DeclareLaunchArgument("mpc", default_value="false", description="Launch the MPC node"),
        DeclareLaunchArgument("mpc_track_local_plan", default_value="false", description="Track the plan published from the mpc_path_publisher.py script in smb_mpc package"),
        DeclareLaunchArgument("keyboard_teleop", default_value="false", description="Launch node to send control commands using the keyboard"),
        DeclareLaunchArgument("joystick", default_value="false", description="Launch node to send control commands using the joystick"),
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz"),
    ]

    # Sensors launch group
    sensors_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("smb"), "launch", "sensors.launch.py"
                ])
            ),
            condition=IfCondition(LaunchConfiguration("launch_sensors")),
            launch_arguments={
                "smb_name": LaunchConfiguration("smb_name"),
                "launch_imu_interface": LaunchConfiguration("launch_imu_interface"),
                "launch_rgb_cam": LaunchConfiguration("launch_rgb_cam"),
                "launch_depth_cam": LaunchConfiguration("launch_depth_cam"),
                "launch_tracking_cam": LaunchConfiguration("launch_tracking_cam"),
            }.items(),
        ),
    ])

    # Lidar odometry launch group
    lidar_odometry_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("smb_msf_graph"), "launch", "smb_msf_graph.launch.py"
                ])
            ),
            condition=IfCondition(LaunchConfiguration("use_lidar_odometry")),
        ),
    ])

    # Tracking camera odometry node
    tracking_camera_node = GroupAction([
        Node(
            package="odometry_conversion",
            executable="odometry_conversion_node",
            name="tracking_camera_odometry_conversion",
            parameters=[{
                "in_odom_frame": "tracking_camera_pose_frame",
                "in_sensor_frame": "tracking_camera_pose_frame",
                "out_odom_frame": "tracking_camera_odom",
                "out_base_frame": "base_link",
                "in_odom_topic": "/tracking_camera/odom/sample",
                "out_odom_topic": "/base_odom",
                "is_odom_child": True,
            }],
            condition=IfCondition(LaunchConfiguration("launch_tracking_cam")),
        ),
    ])

    # Low-level controller
    lowlevel_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_lowlevel_controller"), "launch", "smb_lowlevel_controller.launch.py"
            ])
        ),
        launch_arguments={
            "control_namespace": LaunchConfiguration("control_namespace"),
            "description_name": LaunchConfiguration("description_name"),
            "command_smb": LaunchConfiguration("command_smb"),
        }.items(),
    )

    # Control node
    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_control"), "launch", "smb_control.launch.py"
            ])
        ),
        launch_arguments={
            "description_name": LaunchConfiguration("description_name"),
            "control_namespace": LaunchConfiguration("control_namespace"),
            "mpc": LaunchConfiguration("mpc"),
            "mpc_track_local_plan": LaunchConfiguration("mpc_track_local_plan"),
            "keyboard_teleop": LaunchConfiguration("keyboard_teleop"),
            "joystick": LaunchConfiguration("joystick"),
            "tracking_camera": LaunchConfiguration("launch_tracking_cam"),
        }.items(),
    )

    # RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_opc"), "launch", "opc.launch.py"
            ])
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        declared_arguments +
        [
            sensors_group,
            lidar_odometry_group,
            tracking_camera_node,
            lowlevel_controller,
            control_node,
            rviz_launch,
        ]
    )
