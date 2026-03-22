#!/usr/bin/env python3
"""Publish a mocked grasped object pose following an end-effector frame with jitter.

Parameters (ROS params):
 - end_effector_frame (string): frame to follow (default 'ee_link')
 - target_frame (string): frame to publish the pose in (default 'world')
 - output_topic (string): topic to publish PoseStamped to (default '/mock_grasped_object/pose')
 - rate (float): publish rate in Hz (default 10.0)
 - pos_jitter_std (float): positional jitter std (meters, default 0.005)
 - rot_jitter_std (float): rotational jitter std (radians, default 0.01)
 - offset_xyz (list): offset [x,y,z] in end-effector frame (default [0,0,0])
 - offset_rpy (list): offset [roll,pitch,yaw] in end-effector frame (default [0,0,0])
"""
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, quaternion_multiply


# use tf_transformations helpers for quaternion math


class MockGraspedObject(Node):
    def __init__(self):
        super().__init__('mock_grasped_object')

        self.declare_parameter('end_effector_frame', 'ee_link')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('output_topic', '/mock_grasped_object/pose')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('pos_jitter_std', 0.001)
        self.declare_parameter('rot_jitter_std', 0.01)
        self.declare_parameter('offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('offset_rpy', [0.0, 0.0, 0.0])

        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
        self.pos_jitter_std = float(self.get_parameter('pos_jitter_std').get_parameter_value().double_value)
        self.rot_jitter_std = float(self.get_parameter('rot_jitter_std').get_parameter_value().double_value)
        self.offset_xyz = list(self.get_parameter('offset_xyz').get_parameter_value().double_array_value)
        self.offset_rpy = list(self.get_parameter('offset_rpy').get_parameter_value().double_array_value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        import time
        time.sleep(1)  # give some time for TF buffer to fill

        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        timer_period = 1.0 / max(0.001, self.rate)
        self.create_timer(timer_period, self._publish_pose)

        self.get_logger().info(f'Publishing mocked object poses to "{self.output_topic}" following frame "{self.end_effector_frame}"')

    def _make_offset_pose(self) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.end_effector_frame
        p.pose.position.x = float(self.offset_xyz[0])
        p.pose.position.y = float(self.offset_xyz[1])
        p.pose.position.z = float(self.offset_xyz[2])
        q = quaternion_from_euler(float(self.offset_rpy[0]), float(self.offset_rpy[1]), float(self.offset_rpy[2]))
        # quaternion_from_euler returns [x, y, z, w]
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p

    def _add_jitter(self, pose: PoseStamped) -> PoseStamped:
        # positional jitter
        pose.pose.position.x += random.gauss(0.0, self.pos_jitter_std)
        pose.pose.position.y += random.gauss(0.0, self.pos_jitter_std)
        pose.pose.position.z += random.gauss(0.0, self.pos_jitter_std)

        # rotational jitter: build small euler noise and multiply quaternions
        rx = random.gauss(0.0, self.rot_jitter_std)
        ry = random.gauss(0.0, self.rot_jitter_std)
        rz = random.gauss(0.0, self.rot_jitter_std)
        q_noise = quaternion_from_euler(rx, ry, rz)  # x,y,z,w
        q_curr = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        q_prod = quaternion_multiply(q_noise, q_curr)
        pose.pose.orientation.x = q_prod[0]
        pose.pose.orientation.y = q_prod[1]
        pose.pose.orientation.z = q_prod[2]
        pose.pose.orientation.w = q_prod[3]

        return pose

    def _publish_pose(self):
        try:
            req = self._make_offset_pose()
            # always request the latest available transform by using a zero (unspecified) timestamp
            req.header.stamp = rclpy.time.Time().to_msg()
            transformed = self.tf_buffer.transform(req, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))

            transformed.header.stamp = self.get_clock().now().to_msg()
            out = self._add_jitter(transformed)
            out.header.frame_id = self.target_frame
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warning(f'Could not transform pose from {self.end_effector_frame} to {self.target_frame}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MockGraspedObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
