import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, String

try:
    from robo_vision.perception_utils import compute_signal, detect, get_target
except ImportError:
    from perception_utils import compute_signal, detect, get_target


VALID_CLASSES = ["bottle", "cup", "sports ball"]


class ObjectDetCam2Node(Node):
    def __init__(self):
        super().__init__("objectdetcam2")
        self.declare_parameter("target_class", "bottle")
        self.declare_parameter("camera_index", 4)
        self.declare_parameter("publish_hz", 20.0)
        # Keep capture low-res for speed, then upscale only for display.
        self.declare_parameter("frame_width", 320)
        self.declare_parameter("frame_height", 180)
        self.declare_parameter("display_width", 1280)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("pixelated_display", True)
        self.declare_parameter("camera_buffer_size", 1)
        self.declare_parameter("drop_old_frames", True)
        self.declare_parameter("max_frame_skip", 4)
        self.declare_parameter("inference_imgsz", 640)
        self.declare_parameter("inference_conf", 0.25)

        self.target_class = (
            self.get_parameter("target_class").get_parameter_value().string_value
        )
        self.camera_index = int(
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.frame_width = int(
            self.get_parameter("frame_width").get_parameter_value().integer_value
        )
        self.frame_height = int(
            self.get_parameter("frame_height").get_parameter_value().integer_value
        )
        self.display_width = int(
            self.get_parameter("display_width").get_parameter_value().integer_value
        )
        self.display_height = int(
            self.get_parameter("display_height").get_parameter_value().integer_value
        )
        self.pixelated_display = (
            self.get_parameter("pixelated_display").get_parameter_value().bool_value
        )
        self.camera_buffer_size = max(
            1,
            int(self.get_parameter("camera_buffer_size").get_parameter_value().integer_value),
        )
        self.drop_old_frames = (
            self.get_parameter("drop_old_frames").get_parameter_value().bool_value
        )
        self.max_frame_skip = max(
            0,
            int(self.get_parameter("max_frame_skip").get_parameter_value().integer_value),
        )
        self.inference_imgsz = max(
            160,
            int(self.get_parameter("inference_imgsz").get_parameter_value().integer_value),
        )
        self.inference_conf = float(
            self.get_parameter("inference_conf").get_parameter_value().double_value
        )
        self.inference_conf = max(0.01, min(0.95, self.inference_conf))

        if self.target_class not in VALID_CLASSES:
            self.get_logger().warn(
                f"Invalid target_class '{self.target_class}'. Falling back to 'bottle'."
            )
            self.target_class = "bottle"

        self.offset_pub = self.create_publisher(Float32, "object_offset", 10)
        self.distance_pub = self.create_publisher(Float32, "object_distance", 10)
        target_qos = QoSProfile(depth=1)
        target_qos.history = QoSHistoryPolicy.KEEP_LAST
        target_qos.reliability = QoSReliabilityPolicy.RELIABLE
        target_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "object_targets", self._target_cb, target_qos)

        # Do not publish control signals until a target command arrives.
        self._target_selected = False

        self.cap = cv2.VideoCapture(int(self.camera_index))
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera index {self.camera_index}.")
            raise RuntimeError("Camera open failed")

        if self.frame_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.frame_width))
        if self.frame_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.frame_height))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, float(self.camera_buffer_size))

        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        if self.display_width > 0 and self.display_height > 0:
            cv2.resizeWindow("YOLO Detection", self.display_width, self.display_height)

        self.frame_count = 0
        self.start_time = time.time()
        self.last_status_log = 0.0
        self.log_period_sec = 1.0

        timer_period = 1.0 / max(publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info(
            f"Started objectdetcam2. Target='{self.target_class}', camera={self.camera_index}, "
            f"rate={publish_hz:.1f} Hz, capture={self.frame_width}x{self.frame_height}, "
            f"display={self.display_width}x{self.display_height}, pixelated={self.pixelated_display}, "
            f"drop_old_frames={self.drop_old_frames}, imgsz={self.inference_imgsz}, conf={self.inference_conf:.2f}"
        )
        self.get_logger().info(
            "Waiting for object_targets command before publishing offset/distance."
        )

    def _read_latest_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return False, None

        if not self.drop_old_frames or self.max_frame_skip == 0:
            return True, frame

        skipped = 0
        while skipped < self.max_frame_skip:
            if not self.cap.grab():
                break
            skipped += 1

        if skipped > 0:
            ret_latest, latest = self.cap.retrieve()
            if ret_latest:
                frame = latest

        return True, frame

    def _target_cb(self, msg: String) -> None:
        requested = msg.data.strip().lower()
        if not requested:
            self.get_logger().warn("Received empty object target command; ignoring.")
            return

        if requested not in VALID_CLASSES:
            self.get_logger().warn(
                f"Unsupported target '{requested}'. Valid targets: {', '.join(VALID_CLASSES)}"
            )
            return

        if requested == self.target_class:
            self._target_selected = True
            self.get_logger().info(
                f"Received target '{self.target_class}' from object_targets"
            )
            return

        self.target_class = requested
        self._target_selected = True
        self.get_logger().info(f"Updated target class to '{self.target_class}' from object_targets")

    def process_frame(self):
        ret, frame = self._read_latest_frame()
        if not ret or frame is None:
            self.get_logger().warning("Camera frame grab failed.")
            return

        self.frame_count += 1

        if not self._target_selected:
            display = frame.copy()
            h, w = frame.shape[:2]
            cv2.line(display, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
            cv2.putText(
                display,
                "WAITING FOR OBJECT TARGET...",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                display,
                "Publish one of: bottle, cup, sports ball",
                (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            if self.display_width > 0 and self.display_height > 0:
                interp = cv2.INTER_NEAREST if self.pixelated_display else cv2.INTER_LINEAR
                display = cv2.resize(
                    display,
                    (self.display_width, self.display_height),
                    interpolation=interp,
                )

            cv2.imshow("YOLO Detection", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("'q' pressed. Shutting down node.")
                rclpy.shutdown()
            return

        detections, results = detect(
            frame,
            imgsz=self.inference_imgsz,
            conf=self.inference_conf,
        )
        target = get_target(detections, target_class=self.target_class)
        perception = compute_signal(target, frame.shape[1])

        display = results[0].plot()
        h, w = frame.shape[:2]
        cv2.line(display, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
        cv2.putText(
            display,
            f"TARGET: {self.target_class}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        if perception["found"] and target is not None:
            offset = float(perception["offset_x"])
            distance = float(perception["distance"])

            self.offset_pub.publish(Float32(data=offset))
            self.distance_pub.publish(Float32(data=distance))

            cx = int(target["cx_pixel"])
            cy = int(target["cy_pixel"])
            cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)
            cv2.line(display, (w // 2, h // 2), (cx, cy), (0, 255, 255), 2)

            cv2.putText(
                display,
                f"FOUND: {perception['class']}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                display,
                f"Distance: {distance:.2f}  Offset: {offset:.3f}",
                (20, 115),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                display,
                f"Confidence: {perception['confidence']:.2f}",
                (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            now = time.time()
            if now - self.last_status_log > self.log_period_sec:
                self.last_status_log = now
                self.get_logger().info(
                    f"offset={offset:.3f}, distance={distance:.2f}, class={perception['class']}"
                )
        else:
            cv2.putText(
                display,
                f"SEARCHING FOR {self.target_class.upper()}...",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        if self.frame_count % 30 == 0:
            fps = self.frame_count / max(time.time() - self.start_time, 1e-6)
            cv2.putText(
                display,
                f"FPS: {fps:.1f}",
                (w - 140, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

        if self.display_width > 0 and self.display_height > 0:
            interp = cv2.INTER_NEAREST if self.pixelated_display else cv2.INTER_LINEAR
            display = cv2.resize(display, (self.display_width, self.display_height), interpolation=interp)

        cv2.imshow("YOLO Detection", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("'q' pressed. Shutting down node.")
            rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDetCam2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()