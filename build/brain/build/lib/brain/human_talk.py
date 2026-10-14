#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class ObjectInputPublisher(Node):

    def __init__(self):
        super().__init__('human_talk')

        # Latch the last target so late subscribers (started after input) still receive it.
        target_qos = QoSProfile(depth=1)
        target_qos.history = QoSHistoryPolicy.KEEP_LAST
        target_qos.reliability = QoSReliabilityPolicy.RELIABLE
        target_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(String, 'object_targets', target_qos)

        self.get_logger().info("Type an object name and press Enter (Ctrl+C to exit)")

        self.timer = self.create_timer(0.1, self.input_loop)
        self.waiting_for_input = True

    def input_loop(self):
        if self.waiting_for_input:
            try:
                self.waiting_for_input = False

                user_input = input("Enter object: ").strip()

                if user_input == "":
                    self.get_logger().warn("Empty input, try again.")
                else:
                    msg = String()
                    msg.data = user_input
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")

                self.waiting_for_input = True

            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")
                self.waiting_for_input = True


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = ObjectInputPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    