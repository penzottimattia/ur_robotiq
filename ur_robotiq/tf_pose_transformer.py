#!/usr/bin/env python3
"""Transform incoming PoseStamped messages and optionally average them.

The node publishes one output for every ``average_count`` successfully
transformed input poses. Set ``average_count`` to 1 to disable averaging.

ROS parameters:
 - input_topic (string): input PoseStamped topic (default '/mesh_pose')
 - output_topic (string): output PoseStamped topic (default '/object_pose_world')
 - target_frame (string): destination frame (default 'world')
 - average_count (int): number of input poses per output (default 1)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401; registers PoseStamped with tf2
from time import sleep

class TFPoseTransformer(Node):
    def __init__(self):
        super().__init__('tf_pose_transformer')

        self.declare_parameter('input_topic', '/mesh_pose')
        self.declare_parameter('output_topic', '/object_pose_world')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('average_count', 1)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.average_count = int(self.get_parameter('average_count').value)
        if self.average_count < 1:
            self.get_logger().warning('average_count must be >= 1; using 1')
            self.average_count = 1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sleep(1)

        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub = self.create_subscription(
            PoseStamped, self.input_topic, self._pose_cb, 10
        )

        self._pose_batch = []
        self.get_logger().info(
            f'Subscribing to "{self.input_topic}", publishing to '
            f'"{self.output_topic}" in frame "{self.target_frame}"; '
            f'averaging {self.average_count} pose(s) per output'
        )

    def _pose_cb(self, msg: PoseStamped):
        if not msg.header.frame_id:
            self.get_logger().warning(
                'Received PoseStamped with empty header.frame_id; ignoring'
            )
            return

        try:
            if msg.header.frame_id == self.target_frame:
                transformed = msg
            else:
                transformed = self.tf_buffer.transform(msg, self.target_frame)
        except Exception as exc:
            self.get_logger().warning(
                f'Could not transform from {msg.header.frame_id} '
                f'to {self.target_frame}: {exc}'
            )
            return

        self._pose_batch.append(transformed)
        if len(self._pose_batch) < self.average_count:
            return

        output = self._average_poses(self._pose_batch)
        self._pose_batch.clear()  # non-overlapping batches of N detections
        self.pub.publish(output)

    def _average_poses(self, poses):
        """Average position and orientation of target-frame poses."""
        output = PoseStamped()
        output.header.frame_id = self.target_frame
        output.header.stamp = poses[-1].header.stamp

        count = float(len(poses))
        output.pose.position.x = sum(p.pose.position.x for p in poses) / count
        output.pose.position.y = sum(p.pose.position.y for p in poses) / count
        output.pose.position.z = sum(p.pose.position.z for p in poses) / count

        # Quaternions q and -q represent the same rotation. Align every
        # quaternion to the first one before taking a normalized component mean.
        reference = poses[0].pose.orientation
        ref = (reference.x, reference.y, reference.z, reference.w)
        accum = [0.0, 0.0, 0.0, 0.0]
        for stamped_pose in poses:
            q_msg = stamped_pose.pose.orientation
            q = (q_msg.x, q_msg.y, q_msg.z, q_msg.w)
            if sum(a * b for a, b in zip(q, ref)) < 0.0:
                q = tuple(-value for value in q)
            for index, value in enumerate(q):
                accum[index] += value

        norm = math.sqrt(sum(value * value for value in accum))
        if norm < 1e-12:
            # Degenerate mean: retain the first valid orientation.
            averaged_q = ref
        else:
            averaged_q = tuple(value / norm for value in accum)

        orientation = output.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = averaged_q
        return output


def main(args=None):
    rclpy.init(args=args)
    node = TFPoseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()