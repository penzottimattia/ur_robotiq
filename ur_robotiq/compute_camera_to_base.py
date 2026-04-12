#!/usr/bin/env python3
"""
Compute camera-to-base transform using probe link and object pose topic.

Subscribes to a PoseStamped topic (object pose in camera frame) where the object
is the probe. Uses TF to lookup the probe link in the robot base frame, computes
base_T_camera for each incoming message, maintains a rolling buffer, computes
the mean transform, broadcasts it on TF and writes it to a YAML file.

Usage (example):
  ros2 run ur_robotiq compute_camera_to_base --ros-args -p probe_link:=left_probe_link -p base_frame:=left_base -p object_pose_topic:=/object_pose

"""

import math
import sys
from collections import deque

try:
    import yaml
except Exception:
    yaml = None

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf_transformations as tft
import tf2_ros


def pose_to_matrix(pose):
    t = pose.position
    q = pose.orientation
    trans = np.array([t.x, t.y, t.z], dtype=float)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    mat = tft.quaternion_matrix(quat)
    mat[0:3, 3] = trans
    return mat


def matrix_to_trans_rot(mat):
    trans = mat[0:3, 3]
    quat = tft.quaternion_from_matrix(mat)
    # return x,y,z and quaternion x,y,z,w
    return trans, quat


class CameraToBaseNode(Node):
    def __init__(self):
        super().__init__('compute_camera_to_base')

        self.declare_parameter('probe_link', 'left_probe_link')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('object_pose_topic', '/object_pose')
        self.declare_parameter('buffer_size', 20)
        self.declare_parameter('output_file', '/tmp/camera_to_base.yaml')
        self.declare_parameter('broadcast_rate', 1.0)
        self.declare_parameter('shutdown_after_save', True)

        self.probe_link = self.get_parameter('probe_link').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.object_pose_topic = self.get_parameter('object_pose_topic').get_parameter_value().string_value
        self.buffer_size = int(self.get_parameter('buffer_size').get_parameter_value().integer_value)
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.broadcast_rate = float(self.get_parameter('broadcast_rate').get_parameter_value().double_value)
        self.shutdown_after_save = bool(self.get_parameter('shutdown_after_save').get_parameter_value().bool_value)

        self.get_logger().info(f'Probe link: {self.probe_link}, base frame: {self.base_frame}, topic: {self.object_pose_topic}')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.buffer = deque(maxlen=self.buffer_size)

        self.sub = self.create_subscription(PoseStamped, self.object_pose_topic, self._pose_cb, 10)

        self.timer = self.create_timer(1.0 / max(0.001, self.broadcast_rate), self._on_timer)

        self.latest_mean = None
        self.camera_frame_name = None

    def _pose_cb(self, msg: PoseStamped):
        # msg.pose is pose of probe in camera frame -> camera_T_probe
        try:
            # lookup base_T_probe transform from tf
            # using the timestamp from the message to get consistent transforms
            stamp = msg.header.stamp
            # ROS 2 time conversion
            time = rclpy.time.Time.from_msg(stamp)

            # Attempt to get transform base <- probe (target_frame=base_frame, source_frame=probe_link)
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.base_frame, self.probe_link, time, timeout=rclpy.duration.Duration(seconds=0.5)
                )
            except Exception as e:
                # fallback to latest
                tf_stamped = self.tf_buffer.lookup_transform(self.base_frame, self.probe_link, rclpy.time.Time())

            base_T_probe = self._transformstamped_to_matrix(tf_stamped)

            camera_T_probe = pose_to_matrix(msg.pose)

            # base_T_camera = base_T_probe * inverse(camera_T_probe)
            camera_T_probe_inv = np.linalg.inv(camera_T_probe)
            base_T_camera = np.dot(base_T_probe, camera_T_probe_inv)

            self.buffer.append(base_T_camera)
            self.camera_frame_name = msg.header.frame_id
            self.get_logger().debug(f'Appended transform, buffer size: {len(self.buffer)}')

        except Exception as e:
            self.get_logger().error(f'Failed to process PoseStamped / lookup tf: {e}')

    def _transformstamped_to_matrix(self, tf_stamped: TransformStamped):
        t = tf_stamped.transform.translation
        r = tf_stamped.transform.rotation
        trans = np.array([t.x, t.y, t.z], dtype=float)
        quat = np.array([r.x, r.y, r.z, r.w], dtype=float)
        mat = tft.quaternion_matrix(quat)
        mat[0:3, 3] = trans
        return mat

    def compute_mean_transform(self):
        if not self.buffer:
            return None
        mats = np.stack(list(self.buffer), axis=0)  # (N,4,4)
        # Mean translation
        trans_mean = np.mean(mats[:, 0:3, 3], axis=0)
        # Mean rotation by averaging quaternions
        quats = []
        for m in mats:
            q = tft.quaternion_from_matrix(m)
            quats.append(q)
        quats = np.array(quats)
        # ensure scalar-last (x,y,z,w), tft returns that
        q_mean = np.mean(quats, axis=0)
        q_mean = q_mean / np.linalg.norm(q_mean)

        mean_mat = np.eye(4)
        mean_mat[0:3, 3] = trans_mean
        mean_mat[0:3, 0:3] = tft.quaternion_matrix(q_mean)[0:3, 0:3]
        return mean_mat

    def _on_timer(self):
        mean = self.compute_mean_transform()
        if mean is None:
            return
        self.latest_mean = mean
        trans, quat = matrix_to_trans_rot(mean)

        if self.camera_frame_name is None:
            camera_frame = 'camera'
        else:
            camera_frame = self.camera_frame_name

        # Broadcast transform base -> camera
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = camera_frame
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

        # If buffer is full, save and optionally shutdown
        if len(self.buffer) == self.buffer_size:
            self.get_logger().info('Buffer full; saving mean transform to YAML and shutting down' if self.shutdown_after_save else 'Buffer full; saving mean transform to YAML')
            self._save_yaml(camera_frame, trans, quat)
            if self.shutdown_after_save:
                rclpy.shutdown()

    def _save_yaml(self, camera_frame, trans, quat):
        data = {
            'base_frame': self.base_frame,
            'camera_frame': camera_frame,
            'translation': {'x': float(trans[0]), 'y': float(trans[1]), 'z': float(trans[2])},
            'rotation': {'x': float(quat[0]), 'y': float(quat[1]), 'z': float(quat[2]), 'w': float(quat[3])},
        }
        try:
            if yaml is not None:
                with open(self.output_file, 'w') as f:
                    yaml.safe_dump(data, f)
            else:
                # Minimal YAML write
                with open(self.output_file, 'w') as f:
                    f.write(f"base_frame: {data['base_frame']}\n")
                    f.write(f"camera_frame: {data['camera_frame']}\n")
                    f.write('translation:\n')
                    f.write(f"  x: {data['translation']['x']}\n")
                    f.write(f"  y: {data['translation']['y']}\n")
                    f.write(f"  z: {data['translation']['z']}\n")
                    f.write('rotation:\n')
                    f.write(f"  x: {data['rotation']['x']}\n")
                    f.write(f"  y: {data['rotation']['y']}\n")
                    f.write(f"  z: {data['rotation']['z']}\n")
                    f.write(f"  w: {data['rotation']['w']}\n")
            self.get_logger().info(f'Wrote YAML to {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to write YAML: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
