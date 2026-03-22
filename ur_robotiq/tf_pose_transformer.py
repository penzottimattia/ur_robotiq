#!/usr/bin/env python3
"""Transform incoming PoseStamped from camera frame to world frame and republish.

Parameters (ROS params):
 - input_topic (string): topic to subscribe for incoming PoseStamped (default '/mesh_pose')
 - output_topic (string): topic to publish transformed PoseStamped (default '/object_pose_world')
 - target_frame (string): frame to transform poses into (default 'world')
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class TFPoseTransformer(Node):
    def __init__(self):
        super().__init__('tf_pose_transformer')

        self.declare_parameter('input_topic', '/mesh_pose')
        self.declare_parameter('output_topic', '/object_pose_world')
        self.declare_parameter('target_frame', 'world')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        import time
        time.sleep(1)  # give tf listener time to fill buffer

        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub = self.create_subscription(PoseStamped, self.input_topic, self._pose_cb, 10)

        self.get_logger().info(f'Subscribing to "{self.input_topic}", publishing transformed poses to "{self.output_topic}" in frame "{self.target_frame}"')

    def _pose_cb(self, msg: PoseStamped):
        src_frame = msg.header.frame_id
        if not src_frame:
            self.get_logger().warn('Received PoseStamped with empty header.frame_id; ignoring')
            return

        if src_frame == self.target_frame:
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.target_frame
            out.pose = msg.pose
            self.pub.publish(out)
            return

        try:
            # Prefer using the tf2 Buffer.transform helper which handles message conversion
            transformed = self.tf_buffer.transform(msg, self.target_frame)
            transformed.header.frame_id = self.target_frame
            self.pub.publish(transformed)
        except Exception as e:
            self.get_logger().warning(f'Could not transform from {src_frame} to {self.target_frame}: {e}')


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
