#!/usr/bin/env python3
"""Publish Cartesian targets from SpaceNavigator offsets and the current EE TF."""

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import (
    concatenate_matrices,
    quaternion_from_euler,
    quaternion_matrix,
    quaternion_multiply,
    translation_from_matrix,
    translation_matrix,
    unit_vector,
)


class SpacenavCartesianTarget(Node):
    def __init__(self):
        super().__init__('spacenav_cartesian_target')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('ee_frame', 'right_dorsum_link')
        self.declare_parameter('offset_topic', '/spacenav/offset')
        self.declare_parameter('rot_offset_topic', '/spacenav/rot_offset')
        self.declare_parameter('target_topic', '/right_cartesian_controller/target_frame')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('translation_scale', 1.0)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('offset_in_tool_frame', True)
        self.declare_parameter('tf_timeout', 0.05)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ee_frame = str(self.get_parameter('ee_frame').value)
        self.translation_scale = float(self.get_parameter('translation_scale').value)
        self.rotation_scale = float(self.get_parameter('rotation_scale').value)
        self.offset_in_tool_frame = bool(self.get_parameter('offset_in_tool_frame').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        self.translation = Vector3()
        self.rotation = Vector3()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('target_topic').value), 10)
        self.create_subscription(Vector3, str(self.get_parameter('offset_topic').value), self._translation_cb, 10)
        self.create_subscription(Vector3, str(self.get_parameter('rot_offset_topic').value), self._rotation_cb, 10)
        self.create_timer(1.0 / rate, self._tick)
        self._last_tf_warning_ns = 0

    def _translation_cb(self, msg):
        self.translation = msg

    def _rotation_cb(self, msg):
        self.rotation = msg

    def _tick(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            now = self.get_clock().now().nanoseconds
            if now - self._last_tf_warning_ns > 2_000_000_000:
                self.get_logger().warning(
                    f'Cannot transform {self.base_frame} <- {self.ee_frame}: {exc}')
                self._last_tf_warning_ns = now
            return

        current_translation = transform.transform.translation
        rotation = transform.transform.rotation
        current_quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        offset_translation = (
            self.translation.x * self.translation_scale,
            self.translation.y * self.translation_scale,
            self.translation.z * self.translation_scale,
        )
        if self.offset_in_tool_frame:
            offset_translation = translation_from_matrix(concatenate_matrices(
                quaternion_matrix(current_quaternion),
                translation_matrix(offset_translation)))

        offset_quaternion = quaternion_from_euler(
            self.rotation.x * self.rotation_scale,
            self.rotation.y * self.rotation_scale,
            self.rotation.z * self.rotation_scale)
        target_quaternion = unit_vector(
            quaternion_multiply(current_quaternion, offset_quaternion)
            if self.offset_in_tool_frame
            else quaternion_multiply(offset_quaternion, current_quaternion))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = current_translation.x + offset_translation[0]
        msg.pose.position.y = current_translation.y + offset_translation[1]
        msg.pose.position.z = current_translation.z + offset_translation[2]
        msg.pose.orientation.x = target_quaternion[0]
        msg.pose.orientation.y = target_quaternion[1]
        msg.pose.orientation.z = target_quaternion[2]
        msg.pose.orientation.w = target_quaternion[3]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpacenavCartesianTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
