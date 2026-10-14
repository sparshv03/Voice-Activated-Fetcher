from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    source_mode_arg = DeclareLaunchArgument(
        'source_mode',
        default_value='ros_compressed',
    )
    target_class_arg = DeclareLaunchArgument('target_class', default_value='bottle')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image/compressed',
    )
    udp_port_arg = DeclareLaunchArgument('udp_port', default_value='5600')
    gst_rx_pipeline_arg = DeclareLaunchArgument('gst_rx_pipeline', default_value='')
    frame_read_hz_arg = DeclareLaunchArgument('frame_read_hz', default_value='30.0')
    show_window_arg = DeclareLaunchArgument('show_window', default_value='true')

    inference_node = Node(
        package='robo_vision',
        executable='object_det_remote',
        name='objectdet_remote',
        output='screen',
        parameters=[
            {
                'source_mode': LaunchConfiguration('source_mode'),
                'target_class': LaunchConfiguration('target_class'),
                'image_topic': LaunchConfiguration('image_topic'),
                'udp_port': LaunchConfiguration('udp_port'),
                'gst_rx_pipeline': LaunchConfiguration('gst_rx_pipeline'),
                'frame_read_hz': LaunchConfiguration('frame_read_hz'),
                'show_window': LaunchConfiguration('show_window'),
            }
        ],
    )

    return LaunchDescription(
        [
            source_mode_arg,
            target_class_arg,
            image_topic_arg,
            udp_port_arg,
            gst_rx_pipeline_arg,
            frame_read_hz_arg,
            show_window_arg,
            inference_node,
        ]
    )
