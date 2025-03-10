import launch
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # Corrected import

mpc = DeclareLaunchArgument(
        'mpc',
        default_value='false',
        description="Launch the MPC node."
    )  

mpc_track_local_plan = DeclareLaunchArgument(
        'mpc_track_local_plan',
        default_value='false',
        description="Track the plan published from the mpc_path_publisher.py script in smb_mpc package."
    )

control_namespace = DeclareLaunchArgument(
        'control_namespace',
        default_value='control',
        description="Namespace for the control node."
    )

description_name = DeclareLaunchArgument(
        'description_name',
        default_value='smb_description',
        description="Name for the robot description."
    )

command_smb = DeclareLaunchArgument(
        'command_smb',
        default_value='true',
        description="Send twist commands from the software stack to the robot."
    )

keyboard_teleop = DeclareLaunchArgument(
        'keyboard_teleop',
        default_value='false',
        description="Launch node to send control commands (twist msgs) using the keyboard."
    )

joystick = DeclareLaunchArgument(
        'joystick',
        default_value='false',
        description="Launch node to send control commands (twist msgs) using the joystick."
    )

def generate_launch_description():
    return LaunchDescription([
# Declare global simulation parameter
DeclareLaunchArgument('simulation', default_value='false', description='Set simulation mode'),

# Start sensors group
DeclareLaunchArgument('launch_sensors', default_value='false', description='Launch sensors'),
GroupAction([
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('smb'), '/launch/sensors.launch.py']  # Corrected substitution
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_sensors')),
        launch_arguments={
            'smb_name': LaunchConfiguration('smb_name'),
            'launch_imu_interface': LaunchConfiguration('launch_imu_interface'),
            'launch_rgb_cam': LaunchConfiguration('launch_rgb_cam'),
            'launch_depth_cam': LaunchConfiguration('launch_depth_cam'),
            'launch_tracking_cam': LaunchConfiguration('launch_tracking_cam'),
        }.items(),
    ),
]),


    DeclareLaunchArgument('use_lidar_odometry', default_value='false', description='Use LiDAR odometry'),
GroupAction([
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('smb_msf_graph'), '/launch/smb_msf_graph.launch.py']  # Corrected substitution
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_lidar_odometry'))
    ),
]),
DeclareLaunchArgument('launch_tracking_cam', default_value='false', description='Launch tracking camera odometry'),
GroupAction([
    Node(
        package='odometry_conversion',
        executable='odometry_conversion_node',
        name='tracking_camera_odometry_conversion',
        parameters=[{
            'in_odom_frame': 'tracking_camera_pose_frame',
            'in_sensor_frame': 'tracking_camera_pose_frame',
            'out_odom_frame': 'tracking_camera_odom',
            'out_base_frame': 'base_link',
            'in_odom_topic': '/tracking_camera/odom/sample',
            'out_odom_topic': '/base_odom',
            'is_odom_child': True,
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_tracking_cam')),
    ),
]),

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('smb_lowlevel_controller'), '/launch/smb_lowlevel_controller.launch.py']  # Corrected substitution
    ),
    launch_arguments={
        'control_namespace': LaunchConfiguration('control_namespace'),
        'description_name': LaunchConfiguration('description_name'),
        'command_smb': LaunchConfiguration('command_smb'),
    }.items(),
),

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('smb_control'), '/launch/smb_control.launch.py']  # Corrected substitution
    ),
    launch_arguments={
        'description_name': LaunchConfiguration('description_name'),
        'control_namespace': LaunchConfiguration('control_namespace'),
        'mpc': LaunchConfiguration('mpc'),
        'mpc_track_local_plan': LaunchConfiguration('mpc_track_local_plan'),
        'keyboard_teleop': LaunchConfiguration('keyboard_teleop'),
        'joystick': LaunchConfiguration('joystick'),
        'tracking_camera': LaunchConfiguration('launch_tracking_cam'),
    }.items(),
),

# RViz / OPC visualization
DeclareLaunchArgument('launch_rviz', default_value='false', description='Launch RViz'),
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('smb_opc'), '/launch/opc.launch.py']  # Corrected substitution
    ),
    condition=launch.conditions.IfCondition(LaunchConfiguration('launch_rviz')),
),
])