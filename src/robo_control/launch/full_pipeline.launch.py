from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_human_talk_arg = DeclareLaunchArgument('start_human_talk', default_value='true')
    start_arduino_ping_arg = DeclareLaunchArgument('start_arduino_ping', default_value='true')
    start_object_det_arg = DeclareLaunchArgument('start_object_det', default_value='true')
    start_tracker_arg = DeclareLaunchArgument('start_tracker', default_value='true')

    target_class_arg = DeclareLaunchArgument('target_class', default_value='bottle')
    camera_device_arg = DeclareLaunchArgument('camera_device', default_value='/dev/usbcam')
    camera_index_arg = DeclareLaunchArgument('camera_index', default_value='4')
    publish_hz_arg = DeclareLaunchArgument('publish_hz', default_value='20.0')

    tracker_port_arg = DeclareLaunchArgument('tracker_port', default_value='/dev/propeller')
    tracker_baud_arg = DeclareLaunchArgument('tracker_baud', default_value='9600')
    stop_distance_arg = DeclareLaunchArgument('stop_distance_cm', default_value='30.0')
    forward_speed_arg = DeclareLaunchArgument('forward_speed', default_value='800')
    max_turn_arg = DeclareLaunchArgument('max_turn', default_value='800')
    search_turn_speed_arg = DeclareLaunchArgument('search_turn_speed', default_value='260')
    control_hz_arg = DeclareLaunchArgument('control_hz', default_value='20.0')
    ultrasonic_topic_arg = DeclareLaunchArgument('ultrasonic_topic', default_value='ultrasonic_data')

    human_talk_node = Node(
        package='brain',
        executable='human_talk',
        name='human_talk',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_human_talk')),
    )

    arduino_ping_node = Node(
        package='brain',
        executable='arduino_ping',
        name='ultrasonic_serial_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_arduino_ping')),
    )

    object_detection_node = Node(
        package='robo_vision',
        executable='object_det',
        name='objectdetcam2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_object_det')),
        parameters=[
            {
                'target_class': LaunchConfiguration('target_class'),
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_index': LaunchConfiguration('camera_index'),
                'publish_hz': LaunchConfiguration('publish_hz'),
            }
        ],
    )

    chassis_control_node = Node(
        package='robo_control',
        executable='chassis_control',
        name='propeller_tracker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_tracker')),
        parameters=[
            {
                'port': LaunchConfiguration('tracker_port'),
                'baud': LaunchConfiguration('tracker_baud'),
                'stop_distance_cm': LaunchConfiguration('stop_distance_cm'),
                'max_speed': LaunchConfiguration('forward_speed'),
                'max_turn': LaunchConfiguration('max_turn'),
                'control_hz': LaunchConfiguration('control_hz'),
            }
        ],
    )

    wait_for_target_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/object_targets', 'std_msgs/msg/String'],
        output='screen',
    )

    start_remaining_nodes = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_target_cmd,
            on_exit=[
                LogInfo(msg='Received first /object_targets. Launching remaining nodes.'),
                arduino_ping_node,
                object_detection_node,
                chassis_control_node,
            ],
        )
    )

    return LaunchDescription(
        [
            start_human_talk_arg,
            start_arduino_ping_arg,
            start_object_det_arg,
            start_tracker_arg,
            target_class_arg,
            camera_device_arg,
            camera_index_arg,
            publish_hz_arg,
            tracker_port_arg,
            tracker_baud_arg,
            stop_distance_arg,
            forward_speed_arg,
            max_turn_arg,
            search_turn_speed_arg,
            control_hz_arg,
            ultrasonic_topic_arg,
            human_talk_node,
            LogInfo(msg='Waiting for first /object_targets from human_talk before starting other nodes...'),
            wait_for_target_cmd,
            start_remaining_nodes,
        ]
    )
