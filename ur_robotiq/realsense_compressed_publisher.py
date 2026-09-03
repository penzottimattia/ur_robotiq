#!/usr/bin/env python3
"""Stream RealSense color frames as sensor_msgs/CompressedImage topics."""

import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class RealSenseCompressedPublisher(Node):
    def __init__(self):
        super().__init__('realsense_compressed_publisher')
        self.declare_parameter(
            'serials', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('topic_prefix', '/realsense')
        self.declare_parameter('camera_name_prefix', 'camera_')
        self.declare_parameter('frame_id_prefix', 'realsense')
        self.declare_parameter('jpeg_quality', 85)

        value = self.get_parameter('serials').value
        if value is None:
            value = []
        elif not isinstance(value, (list, tuple)):
            value = [value]
        requested = [str(item).strip() for item in value if str(item).strip()]

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.topic_prefix = str(
            self.get_parameter('topic_prefix').value).rstrip('/')
        self.camera_name_prefix = str(
            self.get_parameter('camera_name_prefix').value)
        self.frame_id_prefix = str(
            self.get_parameter('frame_id_prefix').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        if not self.camera_name_prefix:
            raise ValueError('camera_name_prefix must not be empty')
        if not (self.camera_name_prefix[0].isalpha() or
                self.camera_name_prefix[0] == '_'):
            raise ValueError(
                'camera_name_prefix must begin with a letter or underscore')

        available = [
            device.get_info(rs.camera_info.serial_number)
            for device in rs.context().query_devices()
        ]
        self.serials = requested or available
        if not self.serials:
            raise RuntimeError('No RealSense devices found and serials is empty')
        missing = sorted(set(self.serials) - set(available))
        if missing:
            raise RuntimeError(
                f'RealSense serials not found: {missing}; available: {available}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_publishers = {}
        self._pipelines = {}
        self._running = True
        self._threads = []

        for serial in self.serials:
            # ROS topic tokens cannot start with a digit. RealSense serials do,
            # so prefix each serial with a configurable alphabetic token.
            camera_name = f'{self.camera_name_prefix}{serial}'
            topic = (
                f'{self.topic_prefix}/{camera_name}/color/'
                'image_raw/compressed'
            )
            self._image_publishers[serial] = self.create_publisher(
                CompressedImage, topic, qos)

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                self.fps,
            )
            pipeline.start(config)
            self._pipelines[serial] = pipeline

            thread = threading.Thread(
                target=self._capture_loop,
                args=(serial,),
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)
            self.get_logger().info(f'Streaming {serial} on {topic}')

    def _capture_loop(self, serial):
        pipeline = self._pipelines[serial]
        publisher = self._image_publishers[serial]
        while self._running and rclpy.ok():
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                frame = frames.get_color_frame()
                if not frame:
                    continue

                image = np.asanyarray(frame.get_data())
                ok, encoded = cv2.imencode(
                    '.jpg',
                    image,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not ok:
                    self.get_logger().warning(
                        f'JPEG encoding failed for {serial}')
                    continue

                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = (
                    f'{self.frame_id_prefix}_{serial}_color_optical_frame')
                msg.format = 'jpeg'
                msg.data = encoded.tobytes()
                publisher.publish(msg)
            except RuntimeError as exc:
                if self._running:
                    self.get_logger().warning(f'RealSense {serial}: {exc}')
                    time.sleep(0.1)

    def destroy_node(self):
        self._running = False
        for thread in self._threads:
            thread.join(timeout=1.5)
        for pipeline in self._pipelines.values():
            try:
                pipeline.stop()
            except RuntimeError:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCompressedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
