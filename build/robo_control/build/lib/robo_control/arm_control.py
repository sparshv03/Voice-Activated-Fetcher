#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class PacketMapper(Node):

    def __init__(self):
        super().__init__('packet_mapper')

        # Subscriber: ultrasonic states [s0, s1, s2]
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ultrasonic_data',
            self.callback,
            10
        )

        self.steps_publisher = self.create_publisher(
            Int32MultiArray,
            'joint_steps_abs',
            10
        )

        # Sensor index -> packet number mapping.
        self.sensor_to_packet = {
            0: 1,  # s0 -> packet 1
            1: 2,  # s1 -> packet 2
            2: 3,  # s2 -> packet 3
        }

        self.get_logger().info('Packet Mapper Node Started (ultrasonic mode)')

        # Packet -> Robot command map (3x1 base)
        self.packet_commands = {
            1:  "J=-150,1260,0,-1410,0,840",
            2:  "J=0,1260,0,-1410,0,840",
            3:  "J=180,1260,0,-1410,0,840",
        }

        # Prevent repeated publishes while the same sensor stays active.
        self.last_packet_number = None

    def _parse_command(self, command_string: str):
        _, payload = command_string.split("=", 1)
        return [int(value.strip()) for value in payload.split(",")]

    def _get_active_sensor(self, sensor_values):
        # Mode 1: one-hot or binary-like states (e.g., [1,0,0]).
        if all(value in (0, 1) for value in sensor_values):
            active = [index for index, value in enumerate(sensor_values) if value == 1]

            if not active:
                return None

            if len(active) > 1:
                self.get_logger().warn(
                    f"Multiple sensors active {active}; using s{active[0]}"
                )

            return active[0]

        # Mode 2: raw ultrasonic distances (cm). Smallest positive value wins.
        positive_distances = [
            (index, value) for index, value in enumerate(sensor_values) if value > 0
        ]
        if not positive_distances:
            return None

        active_sensor, _ = min(positive_distances, key=lambda pair: pair[1])
        return active_sensor

    def callback(self, msg: Int32MultiArray):
        values = list(msg.data)

        if len(values) < 3:
            self.get_logger().warn(
                f"Expected 3 sensor states [s0,s1,s2], received {len(values)}"
            )
            return

        sensor_values = values[:3]
        active_sensor = self._get_active_sensor(sensor_values)

        if active_sensor is None:
            self.last_packet_number = None
            return

        packet_number = self.sensor_to_packet.get(active_sensor)
        if packet_number is None:
            self.get_logger().warn(f"No packet mapping for s{active_sensor}")
            return

        if packet_number == self.last_packet_number:
            return

        command_string = self.packet_commands.get(packet_number)
        if command_string is None:
            self.get_logger().warn(f"No command found for packet {packet_number}")
            return

        try:
            step_values = self._parse_command(command_string)
        except (ValueError, IndexError):
            self.get_logger().warn(f"Invalid command format: {command_string}")
            return

        steps_msg = Int32MultiArray()
        steps_msg.data = step_values
        self.steps_publisher.publish(steps_msg)
        self.last_packet_number = packet_number

        self.get_logger().info(
            f"s{active_sensor} active -> packet {packet_number} -> published {step_values}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PacketMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()