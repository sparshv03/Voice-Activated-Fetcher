import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import threading

class UltrasonicSerialNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_serial_node')

        # 🔌 Serial connection (change port if needed)
        self.ser = serial.Serial('/dev/arduino-mega', 115200, timeout=0.01)

        # 📡 Publisher
        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            'ultrasonic_data',
            1   # keep only latest
        )

        # 🧠 Latest sensor data
        self.latest_data = None

        # 🔁 Start serial reading thread
        threading.Thread(target=self.read_serial, daemon=True).start()

        # ⏱ Controlled publish rate (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_data)

        self.get_logger().info("Ultrasonic Serial Node Started")

    # --------------------------
    # 📥 READ FROM SERIAL (FAST)
    # --------------------------
    def read_serial(self):
        while True:
            try:
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors='ignore').strip()

                    if line.startswith("S:"):
                        try:
                            values = list(map(int, line[2:].split(',')))

                            if len(values) == 3:
                                # 🔥 overwrite old data
                                self.latest_data = values

                        except:
                            pass

            except Exception as e:
                self.get_logger().error(f"Serial error: {str(e)}")

    # --------------------------
    # 📡 PUBLISH (CONTROLLED)
    # --------------------------
    def publish_data(self):
        if self.latest_data is not None:
            msg = Int32MultiArray()
            msg.data = self.latest_data

            self.publisher_.publish(msg)

            self.get_logger().info(
                f"Published: {self.latest_data}",
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)

    node = UltrasonicSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()