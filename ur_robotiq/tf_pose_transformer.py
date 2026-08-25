#!/usr/bin/env python3
"""Transform incoming PoseStamped messages and optionally average them.

The node publishes one output for every ``average_count`` successfully
transformed input poses. Set ``average_count`` to 1 to disable averaging.

The transform lookup deliberately uses ``rclpy.time.Time()`` so tf2 applies
the latest transform currently available, regardless of the input pose stamp.

ROS parameters:
- input_topic (string): input PoseStamped topic (default '/mesh_pose')
- output_topic (string): output PoseStamped topic
  (default '/object_pose_world')
- target_frame (string): destination frame (default 'world')
- average_count (int): number of input poses per output (default 1)
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401; registers PoseStamped with tf2


class TFPoseTransformer(Node):
    def __init__(self):
        super().__init__('tf_pose_transformer')

        self.declare_parameter('input_topic', '/mesh_pose')
        self.declare_parameter('output_topic', '/object_pose_world')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('average_count', 1)
        self.declare_parameter('offset_xyz', [0.0, 0.0, 0.0])

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.average_count = max(
            1, int(self.get_parameter('average_count').value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, input_topic, self.pose_callback, 10
        )
        self.pose_batch = []

        self.offset_xyz = self.get_parameter('offset_xyz').value
        if len(self.offset_xyz) != 3:
            raise ValueError(
                f'offset_xyz must be a list of 3 floats, got {self.offset_xyz!r}'
            )

        self.get_logger().info(
            f'Transforming {input_topic} to {self.target_frame} using the '
            f'latest available TF; publishing to {output_topic}; '
            f'average_count={self.average_count}'
        )

    def pose_callback(self, pose_msg):
        try:
            # Time() means "latest available" in tf2. Passing the source frame
            # explicitly avoids using pose_msg.header.stamp for the lookup.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                pose_msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_msg, transform
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Could not transform from {pose_msg.header.frame_id!r} '
                f'to {self.target_frame!r}: {exc}'
            )
            return

        self.pose_batch.append(transformed)
        if len(self.pose_batch) < self.average_count:
            return

        output = self.average_poses(self.pose_batch)
        self.pose_batch.clear()
        self.publisher.publish(output)

    def average_poses(self, poses):
        output = PoseStamped()
        output.header.frame_id = self.target_frame
        # The result represents transforms sampled "now". Use the node clock
        # rather than retaining an unrelated input timestamp.
        output.header.stamp = self.get_clock().now().to_msg()

        count = float(len(poses))
        output.pose.position.x = sum(p.pose.position.x for p in poses) / count + self.offset_xyz[0]
        output.pose.position.y = sum(p.pose.position.y for p in poses) / count + self.offset_xyz[1]
        output.pose.position.z = sum(p.pose.position.z for p in poses) / count + self.offset_xyz[2]

        # Hemisphere-align quaternions before normalized component averaging
        # so q and -q do not cancel even though they represent the same pose.
        reference = poses[0].pose.orientation
        quaternions = []
        for item in poses:
            q = item.pose.orientation
            dot = (
                reference.x * q.x
                + reference.y * q.y
                + reference.z * q.z
                + reference.w * q.w
            )
            sign = -1.0 if dot < 0.0 else 1.0
            quaternions.append(
                (sign * q.x, sign * q.y, sign * q.z, sign * q.w)
            )

        x = sum(q[0] for q in quaternions) / count
        y = sum(q[1] for q in quaternions) / count
        z = sum(q[2] for q in quaternions) / count
        w = sum(q[3] for q in quaternions) / count
        norm = math.sqrt(x * x + y * y + z * z + w * w)

        if norm < 1e-12:
            output.pose.orientation.w = 1.0
        else:
            output.pose.orientation.x = x / norm
            output.pose.orientation.y = y / norm
            output.pose.orientation.z = z / norm
            output.pose.orientation.w = w / norm

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