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
    udp_host_arg = DeclareLaunchArgument('udp_host', default_value='')
    udp_port_arg = DeclareLaunchArgument('udp_port', default_value='5600')
    h264_bitrate_arg = DeclareLaunchArgument('h264_bitrate_kbps', default_value='2000')
    h264_keyint_arg = DeclareLaunchArgument('h264_keyint', default_value='30')
    gst_tx_pipeline_arg = DeclareLaunchArgument('gst_tx_pipeline', default_value='')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='camera')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image/compressed',
    )
    flip_horizontal_arg = DeclareLaunchArgument('flip_horizontal', default_value='false')

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
                'udp_host': LaunchConfiguration('udp_host'),
                'udp_port': LaunchConfiguration('udp_port'),
                'h264_bitrate_kbps': LaunchConfiguration('h264_bitrate_kbps'),
                'h264_keyint': LaunchConfiguration('h264_keyint'),
                'gst_tx_pipeline': LaunchConfiguration('gst_tx_pipeline'),
                'frame_id': LaunchConfiguration('frame_id'),
                'image_topic': LaunchConfiguration('image_topic'),
                'flip_horizontal': LaunchConfiguration('flip_horizontal'),
            }
        ],
    )

    return LaunchDescription(
        [
            stream_mode_arg,
            camera_index_arg,
            publish_hz_arg,
            jpeg_quality_arg,
            udp_host_arg,
            udp_port_arg,
            h264_bitrate_arg,
            h264_keyint_arg,
            gst_tx_pipeline_arg,
            frame_id_arg,
            image_topic_arg,
            flip_horizontal_arg,
            camera_node,
        ]
    )
