import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_lidar', default_value='true', description='Launch LiDAR'),
        DeclareLaunchArgument('launch_tracking_cam', default_value='true', description='Launch Tracking Camera'),
        DeclareLaunchArgument('launch_depth_cam', default_value='false', description='Launch Depth Camera'),
        DeclareLaunchArgument('launch_imu_interface', default_value='true', description='Launch IMU Interface'),
        DeclareLaunchArgument('launch_rgb_cam', default_value='remote', description='Launch RGB Camera'),
        DeclareLaunchArgument('launch_powerstatus', default_value='true', description='Launch Power Status'),
        DeclareLaunchArgument('tracking_cam_calib_odom_file', default_value='$(find smb)/config/tracking_camera_config.json', description='Path to config for odometry input to tracking camera'),
        DeclareLaunchArgument('smb_name', default_value='$(env SMB_NAME)', description='Name of the SMB in the format smb26x (relevant for calibrations)'),
        DeclareLaunchArgument('GPU_user', default_value='$(env USER)', description='Username to use on the Jetson Xavier GPU'),
    
        # Robosense LiDAR
        GroupAction([
            Node(
                package='rslidar_sdk',
                executable='rslidar_sdk_node',
                name='rslidar_sdk_node',
                output='screen'
            )
        ], condition=IfCondition(LaunchConfiguration('launch_lidar'))),

        # Intel RealSense Tracking Camera T265
        GroupAction([
            Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                namespace='tracking_camera',
                parameters=[{
                    'tracking_module.enable_mapping': False,
                    'tracking_module.enable_pose_jumping': False,
                    'tracking_module.enable_relocalization': False,
                    'device_type': 't265',
                    'enable_pose': True,
                    'enable_accel': True,
                    'enable_gyro': True,
                    'publish_tf': False,
                    'publish_odom_tf': False,
                    'topic_odom_in': '/control/smb_diff_drive/odom',
                    'unite_imu_method': 'linear_interpolation',
                    'calib_odom_file': LaunchConfiguration('tracking_cam_calib_odom_file')
                }],
                output='screen'
            )
        ], condition=IfCondition(LaunchConfiguration('launch_tracking_cam'))),

        # Intel RealSense Depth Camera D435
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_camera.launch.py'])
                ),
                launch_arguments={'camera': 'depth_camera', 'device_type': 'd435'}.items()
            )
        ], condition=IfCondition(LaunchConfiguration('launch_depth_cam'))),

        # IMU Interface for ADIS 16448
        GroupAction([
            Node(
                package='rosserial_python',
                executable='serial_node',
                name='rosserial_python',
                parameters=[{'_port': '/dev/versavis', '_baud': 250000}],
                output='screen',
                respawn=True
            ),
            Node(
                package='versavis_adis16448_receiver',
                executable='versavis_imu_receiver',
                name='versavis_imu_receiver',
                parameters=[{
                    'imu_accelerator_sensitivity': 0.000833,
                    'imu_gyro_sensitivity': 0.04,
                    'imu_acceleration_covariance': 0.043864908,
                    'imu_gyro_covariance': 6e-9,
                    'imu_sub_topic': '/versavis/imu_micro',
                    'imu_pub_topic': '/imu'
                }],
                output='screen'
            )
        ], condition=IfCondition(LaunchConfiguration('launch_imu_interface'))),

        # RGB Camera (FLIR Driver)
        GroupAction([
            Node(
                package='nodelet',
                executable='nodelet',
                name='camera_nodelet_manager',
                parameters=[{'num_worker_threads': 4}],
                output='screen'
            ),
            Node(
                package='nodelet',
                executable='nodelet',
                name='rgb_camera_nodelet',
                parameters=[{
                    'frame_id': 'rgb_camera_optical_link',
                    'serial': '0',
                    'acquisition_frame_rate': 4,
                    'acquisition_frame_rate_enable': True,
                    'image_format_color_coding': 'BayerRG8',
                    'color_processing_algorithm': 'HQ_LINEAR',
                    'camera_info_url': 'package://smb_bringup/config/smb_cam0.yaml',
                    'acquisition_mode': 'Continuous',
                    'enable_trigger': 'Off',
                    'exposure_mode': 'Timed',
                    'exposure_auto': 'Continuous',
                    'line_selector': 'Line2',
                    'line_mode': 'Output',
                    'line_source': 'ExposureActive',
                    'line_inverter': False,
                    'auto_exposure_time_upper_limit': 5000,
                    'auto_exposure_time_lower_limit': 300,
                    'auto_gain': 'Continuous',
                    'auto_white_balance': 'Continuous'
                }],
                output='screen'
            ),
            Node(
                package='nodelet',
                executable='nodelet',
                name='image_proc_debayer',
                output='screen'
            )
        ], condition=IfCondition(LaunchConfiguration('launch_rgb_cam'))),

        # Power Status
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('smb_powerstatus'), 'launch', 'smb_powerstatus.launch.py'])
                )
            )
        ], condition=IfCondition(LaunchConfiguration('launch_powerstatus')))
    ])