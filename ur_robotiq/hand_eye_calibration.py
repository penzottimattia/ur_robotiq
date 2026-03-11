#!/usr/bin/env python3
"""Hand-eye calibration node

Subscribes to a PoseStamped topic (default /object_pose) that gives the probe
pose in the camera frame. Uses tf to lookup the probe transform in the robot
base frame. Collects many samples, averages them and computes the transform
of the camera with respect to the robot base.

Outputs a YAML file with the transform (camera w.r.t robot base) and publishes
that transform as a static transform (base -> camera).

Quaternion math uses the `tf_transformations` (transformations) library. The
output quaternion in the YAML is in w,x,y,z order as requested.
"""

import os
import time
import yaml
import numpy as np
import threading
import sys
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

# transformation utilities (expects quaternions as x,y,z,w)
try:
    from tf_transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix
except Exception:
    # try alternate import name if available
    import tf_transformations as tft
    quaternion_matrix = tft.quaternion_matrix
    quaternion_from_matrix = tft.quaternion_from_matrix
    euler_from_matrix = tft.euler_from_matrix


def transform_to_matrix(transform_msg: TransformStamped) -> np.ndarray:
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    q_xyzw = [q.x, q.y, q.z, q.w]
    mat = quaternion_matrix(q_xyzw)
    mat[0:3, 3] = [t.x, t.y, t.z]
    return mat


def pose_to_matrix(pose_msg: PoseStamped) -> np.ndarray:
    p = pose_msg.pose.position
    q = pose_msg.pose.orientation
    q_xyzw = [q.x, q.y, q.z, q.w]
    mat = quaternion_matrix(q_xyzw)
    mat[0:3, 3] = [p.x, p.y, p.z]
    return mat


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('hand_eye_calibrator')

        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('probe_frame', 'probe')
        self.declare_parameter('pose_topic', '/object_pose')
        self.declare_parameter('samples', 100)
        self.declare_parameter('output_file', os.path.join(
            os.path.dirname(__file__), '..', 'config', 'camera_to_base.yaml'))
        self.declare_parameter('timeout_s', 1.0)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.probe_frame = self.get_parameter('probe_frame').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.samples = int(self.get_parameter('samples').get_parameter_value().integer_value)
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.timeout_s = float(self.get_parameter('timeout_s').get_parameter_value().double_value)

        self.get_logger().info(f'HandEyeCalibrator: base_frame={self.base_frame}, camera_frame={self.camera_frame}, probe_frame={self.probe_frame}, pose_topic={self.pose_topic}, samples={self.samples}')

        # tf buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        time.sleep(1)  # give some time for tf listener to fill buffer

        self.matrices = []
        # synchronization: ensure compute_and_publish runs only once
        self._lock = threading.Lock()
        self._computed = False

        # start keyboard monitor thread to allow interrupting sampling with any key
        threading.Thread(target=self._keyboard_monitor, daemon=True).start()

        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)

    def _keyboard_monitor(self):
        """Wait for a single keypress on stdin and trigger computation early.

        This uses tty/termios to read a single character without requiring Enter.
        If stdin is not a TTY (e.g. launched as a service), the monitor returns.
        """
        try:
            if not sys.stdin.isatty():
                return
        except Exception:
            return

        self.get_logger().info('Press any key to interrupt sampling and compute transform')
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            # block until a single key is pressed
            _ = sys.stdin.read(1)
            self.get_logger().info('Key pressed, interrupting sampling')
            # trigger compute; compute_and_publish is thread-safe (uses its own lock)
            self.compute_and_publish()
        except Exception as e:
            self.get_logger().warn(f'Keyboard monitor failed: {e}')
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _pose_cb(self, msg: PoseStamped):
        if len(self.matrices) >= self.samples:
            return

        # PoseStamped is the probe pose in the camera frame -> camera_T_probe
        try:
            cam_T_probe = pose_to_matrix(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert PoseStamped to matrix: {e}')
            return

        # Lookup base -> probe transform from the tf tree
        try:
            # use latest available transform
            tf_msg = self.tf_buffer.lookup_transform(self.base_frame, self.probe_frame, rclpy.time.Time())
            base_T_probe = transform_to_matrix(tf_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to lookup transform {self.base_frame} <- {self.probe_frame}: {e}')
            return

        # Compute base_T_camera = base_T_probe * inv(camera_T_probe)
        try:
            inv_cam = np.linalg.inv(cam_T_probe)
            base_T_camera = base_T_probe.dot(inv_cam)
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f'Failed to invert camera->probe matrix: {e}')
            return

        self.matrices.append(base_T_camera)
        self.get_logger().info(f'Collected sample {len(self.matrices)}/{self.samples}')

        if len(self.matrices) >= self.samples:
            self.get_logger().info('Sample collection complete, computing average transform...')
            self.compute_and_publish()

    def compute_and_publish(self):
        # ensure this only runs once
        with self._lock:
            if self._computed:
                return
            self._computed = True

            if len(self.matrices) == 0:
                self.get_logger().warning('No samples collected; aborting compute_and_publish')
                return

            mats = np.array(self.matrices)
            # average translation
            translations = mats[:, 0:3, 3]
            avg_t = np.mean(translations, axis=0)

        # average rotation via orthogonalization of summed rotation matrices
        R_sum = np.zeros((3, 3))
        for m in mats:
            R_sum += m[0:3, 0:3]

        # SVD-based orthogonalization
        U, S, Vt = np.linalg.svd(R_sum)
        R_avg = U.dot(Vt)
        if np.linalg.det(R_avg) < 0:
            # fix reflection
            U[:, -1] *= -1
            R_avg = U.dot(Vt)

        # convert to quaternion (tf_transformations returns x,y,z,w)
        mat4 = np.eye(4)
        mat4[0:3, 0:3] = R_avg
        q_xyzw = quaternion_from_matrix(mat4)

        # compute roll, pitch, yaw from the averaged rotation
        rpy = euler_from_matrix(mat4)

        data = {
            'camera_to_base': {
                'header': {
                    'frame_id': self.base_frame,
                    'child_frame_id': self.camera_frame,
                },
                'transform': {
                    'translation': {
                        'x': float(avg_t[0]),
                        'y': float(avg_t[1]),
                        'z': float(avg_t[2]),
                    },
                    'rotation_xyzw': {
                        'x': float(q_xyzw[0]),
                        'y': float(q_xyzw[1]),
                        'z': float(q_xyzw[2]),
                        'w': float(q_xyzw[3]),
                    },
                    'rpy': {
                        'r': float(rpy[0]),
                        'p': float(rpy[1]),
                        'y': float(rpy[2]),
                    }
                }
            }
        }

        # ensure directory exists
        out_dir = os.path.dirname(self.output_file)
        os.makedirs(out_dir, exist_ok=True)

        with open(self.output_file, 'w') as fh:
            yaml.safe_dump(data, fh, default_flow_style=False)

        self.get_logger().info(f'Wrote averaged camera->base transform to {self.output_file}')

        # publish static transform base -> camera (TransformStamped expects x,y,z,w order)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame
        t.transform.translation.x = float(avg_t[0])
        t.transform.translation.y = float(avg_t[1])
        t.transform.translation.z = float(avg_t[2])
        t.transform.rotation.x = float(q_xyzw[0])
        t.transform.rotation.y = float(q_xyzw[1])
        t.transform.rotation.z = float(q_xyzw[2])
        t.transform.rotation.w = float(q_xyzw[3])

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published static transform {self.base_frame} -> {self.camera_frame}')


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()

    try:
        # spin until we finish collecting samples and publishing
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down hand_eye_calibrator')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
