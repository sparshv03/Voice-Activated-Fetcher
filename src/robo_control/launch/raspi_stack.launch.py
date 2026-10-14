from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    stream_mode_arg = DeclareLaunchArgument(
        'stream_mode',
        default_value='ros_compressed',
    )
    camera_index_arg = DeclareLaunchArgument('camera_index', default_value='0')
    publish_hz_arg = DeclareLaunchArgument('publish_hz', default_value='20.0')
    jpeg_quality_arg = DeclareLaunchArgument('jpeg_quality', default_value='65')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image/compressed',
    )
    udp_host_arg = DeclareLaunchArgument('udp_host', default_value='')
    udp_port_arg = DeclareLaunchArgument('udp_port', default_value='5600')
    h264_bitrate_arg = DeclareLaunchArgument('h264_bitrate_kbps', default_value='2000')
    h264_keyint_arg = DeclareLaunchArgument('h264_keyint', default_value='30')
    gst_tx_pipeline_arg = DeclareLaunchArgument('gst_tx_pipeline', default_value='')

    serial_port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    serial_baud_arg = DeclareLaunchArgument('baud', default_value='9600')
    slow_distance_arg = DeclareLaunchArgument('slow_distance_cm', default_value='30.0')
    stop_distance_arg = DeclareLaunchArgument('stop_distance_cm', default_value='15.0')
    slow_speed_arg = DeclareLaunchArgument('slow_speed', default_value='500')
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='600')
    max_turn_arg = DeclareLaunchArgument('max_turn', default_value='500')
    control_hz_arg = DeclareLaunchArgument('control_hz', default_value='20.0')

    camera_node = Node(
        package='robo_vision',
        executable='camera_stream',
        name='camera_stream',
        output='screen',
        parameters=[
            {
                'stream_mode': LaunchConfiguration('stream_mode'),
                'camera_index': LaunchConfiguration('camera_index'),
                'publish_hz': LaunchConfiguration('publish_hz'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'image_topic': LaunchConfiguration('image_topic'),
                'udp_host': LaunchConfiguration('udp_host'),
                'udp_port': LaunchConfiguration('udp_port'),
                'h264_bitrate_kbps': LaunchConfiguration('h264_bitrate_kbps'),
                'h264_keyint': LaunchConfiguration('h264_keyint'),
                'gst_tx_pipeline': LaunchConfiguration('gst_tx_pipeline'),
            }
        ],
    )

    control_node = Node(
        package='robo_control',
        executable='propeller_tracker',
        name='propeller_tracker',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'slow_distance_cm': LaunchConfiguration('slow_distance_cm'),
                'stop_distance_cm': LaunchConfiguration('stop_distance_cm'),
                'slow_speed': LaunchConfiguration('slow_speed'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_turn': LaunchConfiguration('max_turn'),
                'control_hz': LaunchConfiguration('control_hz'),
            }
        ],
    )

    return LaunchDescription(
        [
            stream_mode_arg,
            camera_index_arg,
            publish_hz_arg,
            jpeg_quality_arg,
            image_topic_arg,
            udp_host_arg,
            udp_port_arg,
            h264_bitrate_arg,
            h264_keyint_arg,
            gst_tx_pipeline_arg,
            serial_port_arg,
            serial_baud_arg,
            slow_distance_arg,
            stop_distance_arg,
            slow_speed_arg,
            max_speed_arg,
            max_turn_arg,
            control_hz_arg,
            camera_node,
            control_node,
        ]
    )
