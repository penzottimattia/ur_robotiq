import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        # parameters
        self.declare_parameter('input_topic', '')
        self.declare_parameter('device', 0)
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('fps', 10.0)
        # node will first perform a centered square crop on the frame and then
        # optionally resize to `output_size` (square).
        self.declare_parameter('output_size', 0)
        # image encoding to publish: 'bgr8' or 'rgb8'
        self.declare_parameter('encoding', 'rgb8')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.output_size = int(self.get_parameter('output_size').get_parameter_value().integer_value)
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._stop_event = threading.Event()
        self.cap = None

        # If input_topic is specified, subscribe to it instead of capturing from device
        if input_topic:
            self.create_subscription(Image, input_topic, self._image_callback, 10)
            self.get_logger().info(f'USB camera node started (input_topic={input_topic}, topic={topic}, fps={fps})')
        else:
            # video capture
            self.cap = cv2.VideoCapture(device)

            # ensure the device opened correctly; if not, fail fast
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open video device {device}')
                try:
                    self.cap.release()
                except Exception:
                    pass
                raise RuntimeError(f'Cannot open video device {device}')

            # reader thread
            self._reader = threading.Thread(target=self._capture_loop, daemon=True)
            self._reader.start()

            self.get_logger().info(f'USB camera node started (device={device}, topic={topic}, fps={fps})')

        # publisher timer
        period = 1.0 / max(1e-3, fps)
        self.create_timer(period, self._publish_latest_frame)

    def _capture_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            ret, frame = self.cap.read()
            if ret:
                with self._frame_lock:
                    self._latest_frame = frame
            else:
                # small sleep to avoid busy loop when camera not ready
                time.sleep(0.01)

    def _publish_latest_frame(self):
        with self._frame_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()

        if frame is None:
            return

        h, w = frame.shape[:2]

        # 1) Centered square crop
        sq = min(w, h)
        sx = (w - sq) // 2
        sy = (h - sq) // 2
        frame = frame[sy:sy + sq, sx:sx + sq]

        # 2) Resize to requested output_size if provided (square).
        if self.output_size > 0:
            frame = cv2.resize(frame, (self.output_size, self.output_size), interpolation=cv2.INTER_LINEAR)

        try:
            if getattr(self, 'encoding', 'bgr8') == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def _image_callback(self, msg):
        """Callback for receiving images from input topic."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Failed to convert image from input topic: {e}')

    def destroy_node(self):
        # stop reader
        self._stop_event.set()
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = USBCameraNode()
    except Exception as e:
        try:
            logger = rclpy.logging.get_logger('usb_camera_node')
            logger.error(f'Failed to start USB camera node: {e}')
        except Exception:
            print(f'Failed to start USB camera node: {e}')
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
