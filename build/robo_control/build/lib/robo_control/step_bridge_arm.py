#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import serial
from std_msgs.msg import Int32MultiArray


class SerialStepBridgeNode(Node):
    def __init__(self, port: str, baud: int, timeout: float, topic: str) -> None:
        super().__init__("serial_step_bridge")
        self.serial = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)
        self.subscription = self.create_subscription(Int32MultiArray, topic, self.callback, 10)
        self.get_logger().info(
            f"Connected to {port} @ {baud}. Subscribed to {topic}"
        )

    def callback(self, msg: Int32MultiArray) -> None:
        values = list(msg.data)
        if len(values) != 6:
            self.get_logger().warn(
                f"Expected 6 values, received {len(values)}. Message ignored."
            )
            return

        try:
            command = "J:" + ",".join(str(int(value)) for value in values) + "\n"
        except (TypeError, ValueError):
            self.get_logger().warn("Received non-integer values in message. Message ignored.")
            return

        self.serial.write(command.encode("ascii"))
        self.serial.flush()
        self.get_logger().info(f"Sent: {command.strip()}")

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to absolute joint step topic and forward to Arduino serial"
    )
    parser.add_argument("--port", required=True, help="Serial port (example: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--timeout", type=float, default=0.2, help="Read/write timeout in seconds")
    parser.add_argument(
        "--topic",
        default="joint_steps_abs",
        help="Topic to subscribe for Int32MultiArray absolute targets",
    )

    if argv is None:
        argv = sys.argv
    return parser.parse_args(remove_ros_args(argv)[1:])


def main(args: list[str] | None = None) -> int:
    cli = parse_args(args)
    rclpy.init(args=args)

    try:
        node = SerialStepBridgeNode(cli.port, cli.baud, cli.timeout, cli.topic)
    except serial.SerialException as exc:
        print(f"Failed to open serial port: {exc}")
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())