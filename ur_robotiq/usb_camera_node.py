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
        self.declare_parameter('device', 0)
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('fps', 10.0)
        # node will first perform a centered square crop on the frame and then
        # optionally resize to `output_size` (square).
        self.declare_parameter('output_size', 0)
        # image encoding to publish: 'bgr8' or 'rgb8'
        self.declare_parameter('encoding', 'rgb8')

        device = self.get_parameter('device').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.output_size = int(self.get_parameter('output_size').get_parameter_value().integer_value)
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

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

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._stop_event = threading.Event()

        # reader thread
        self._reader = threading.Thread(target=self._capture_loop, daemon=True)
        self._reader.start()

        # publisher timer
        period = 1.0 / max(1e-3, fps)
        self.create_timer(period, self._publish_latest_frame)

        self.get_logger().info(f'USB camera node started (device={device}, topic={topic}, fps={fps})')

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
