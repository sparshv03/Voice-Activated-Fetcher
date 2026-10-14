import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__("camera_stream")

        self.declare_parameter("stream_mode", "ros_compressed")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "camera/image/compressed")
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("udp_host", "")
        self.declare_parameter("udp_port", 5600)
        self.declare_parameter("h264_bitrate_kbps", 2000)
        self.declare_parameter("h264_keyint", 30)
        self.declare_parameter("gst_tx_pipeline", "")

        self.stream_mode = (
            self.get_parameter("stream_mode").get_parameter_value().string_value
        )

        self.camera_index = int(
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        self.publish_hz = (
            self.get_parameter("publish_hz").get_parameter_value().double_value
        )
        self.jpeg_quality = int(
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.flip_horizontal = (
            self.get_parameter("flip_horizontal").get_parameter_value().bool_value
        )
        self.udp_host = self.get_parameter("udp_host").get_parameter_value().string_value
        self.udp_port = int(
            self.get_parameter("udp_port").get_parameter_value().integer_value
        )
        self.h264_bitrate_kbps = int(
            self.get_parameter("h264_bitrate_kbps").get_parameter_value().integer_value
        )
        self.h264_keyint = int(
            self.get_parameter("h264_keyint").get_parameter_value().integer_value
        )
        self.gst_tx_pipeline = (
            self.get_parameter("gst_tx_pipeline").get_parameter_value().string_value
        )

        self.jpeg_quality = max(20, min(95, self.jpeg_quality))
        self.h264_bitrate_kbps = max(300, self.h264_bitrate_kbps)
        self.h264_keyint = max(5, self.h264_keyint)

        if self.stream_mode not in ("ros_compressed", "gst_udp"):
            self.get_logger().warn(
                f"Invalid stream_mode '{self.stream_mode}'. Falling back to 'ros_compressed'."
            )
            self.stream_mode = "ros_compressed"

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera index {self.camera_index}.")
            raise RuntimeError("Camera open failed")

        self.publisher = None
        self.gst_writer = None
        if self.stream_mode == "ros_compressed":
            self.publisher = self.create_publisher(
                CompressedImage,
                self.image_topic,
                qos_profile_sensor_data,
            )

        self.frame_count = 0
        self.start_time = time.time()
        self.last_warn = 0.0
        self.log_period_sec = 1.0

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_frame)

        if self.stream_mode == "ros_compressed":
            self.get_logger().info(
                "Started camera_stream (ros_compressed). "
                f"camera={self.camera_index}, topic='{self.image_topic}', "
                f"rate={self.publish_hz:.1f} Hz, jpeg_quality={self.jpeg_quality}"
            )
        else:
            self.get_logger().info(
                "Started camera_stream (gst_udp). "
                f"camera={self.camera_index}, host='{self.udp_host}', port={self.udp_port}, "
                f"rate={self.publish_hz:.1f} Hz"
            )

    def _build_gst_tx_pipeline(self, width: int, height: int) -> str:
        if self.gst_tx_pipeline:
            return self.gst_tx_pipeline

        if not self.udp_host:
            return ""

        fps = max(int(round(self.publish_hz)), 1)
        return (
            "appsrc is-live=true block=false format=time ! "
            f"video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
            "videoconvert ! "
            f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={self.h264_bitrate_kbps} "
            f"key-int-max={self.h264_keyint} bframes=0 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.udp_host} port={self.udp_port} sync=false async=false"
        )

    def _init_gst_writer(self, width: int, height: int) -> None:
        pipeline = self._build_gst_tx_pipeline(width, height)
        if not pipeline:
            self.get_logger().error(
                "gst_udp mode requires udp_host or gst_tx_pipeline parameter."
            )
            return

        self.gst_writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            max(self.publish_hz, 1.0),
            (width, height),
            True,
        )
        if not self.gst_writer.isOpened():
            self.get_logger().error(
                "Failed to open GStreamer VideoWriter. Check OpenCV GStreamer support and pipeline."
            )
            self.gst_writer = None

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            now = time.time()
            if now - self.last_warn > self.log_period_sec:
                self.last_warn = now
                self.get_logger().warning("Camera frame grab failed.")
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        if self.stream_mode == "ros_compressed":
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                now = time.time()
                if now - self.last_warn > self.log_period_sec:
                    self.last_warn = now
                    self.get_logger().warning("JPEG encoding failed.")
                return

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            if self.publisher is not None:
                self.publisher.publish(msg)
        else:
            if self.gst_writer is None:
                h, w = frame.shape[:2]
                self._init_gst_writer(w, h)
                if self.gst_writer is None:
                    return
            self.gst_writer.write(frame)

        self.frame_count += 1
        if self.frame_count % 60 == 0:
            elapsed = max(time.time() - self.start_time, 1e-6)
            fps = self.frame_count / elapsed
            self.get_logger().debug(f"Streaming FPS: {fps:.1f}")

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        if hasattr(self, "gst_writer") and self.gst_writer is not None:
            self.gst_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraStreamNode()
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
