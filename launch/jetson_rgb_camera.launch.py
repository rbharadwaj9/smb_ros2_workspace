from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='rgb_camera_container',
            namespace='rgb_camera',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='spinnaker_camera_driver',
                    plugin='spinnaker_camera_driver::CameraDriver',
                    name='rgb_camera_node',
                    parameters=[{
                        'serial_number': '20435008',
                        'frame_id': 'rgb_camera_optical_link',
                        'parameter_file': '/path/to/camera.yaml',
                        'acquisition_frame_rate': 30.0,
                        'image_format_color_coding': 'BayerRG8',
                        'color_processing_algorithm': 'HQ_LINEAR',
                        'exposure_auto': 'Continuous',
                        'gain_auto': 'Continuous',
                    }]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    name='debayer_node',
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                )
            ],
            output='screen',
        )
    ])
