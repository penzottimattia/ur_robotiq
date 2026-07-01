#!/usr/bin/env python3
"""Bridge two Cartesian and two JointState command streams into a single Float64MultiArray.

The node subscribes to two PoseStamped arm command topics and two JointState gripper command
topics, extracts pose vectors and indexed gripper positions, concatenates them, and republishes
the result as a std_msgs/Float64MultiArray at a fixed rate.
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped


class CartesianCommandStitcherNode(Node):
    def __init__(self):
        super().__init__('cartesian_stitcher_node')

        self.declare_parameter('left_frame_topic', '/left_cartesian_controller/target_pose')
        self.declare_parameter('right_frame_topic', '/right_cartesian_controller/target_pose')
        self.declare_parameter('left_gripper_topic', '/left_hand_controller/state')
        self.declare_parameter('right_gripper_topic', '/right_hand_controller/state')
        self.declare_parameter('gripper_keys', ['index_flexion'])
        self.declare_parameter('output_topic', '/commands')
        self.declare_parameter('publish_rate_hz', 30.0)

        self.left_frame_topic = self.get_parameter('left_frame_topic').value
        self.right_frame_topic = self.get_parameter('right_frame_topic').value
        self.left_gripper_topic = self.get_parameter('left_gripper_topic').value
        self.right_gripper_topic = self.get_parameter('right_gripper_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.gripper_keys = list(self.get_parameter('gripper_keys').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self._lock = threading.Lock()

        self._latest_left_frame = None
        self._latest_right_frame = None
        self._latest_left_gripper = None
        self._latest_right_gripper = None

        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            10,
        )

        self.left_subscription = self.create_subscription(
            PoseStamped,
            self.left_frame_topic,
            self._left_frame_callback,
            10,
        )

        self.right_subscription = self.create_subscription(
            PoseStamped,
            self.right_frame_topic,
            self._right_frame_callback,
            10,
        )

        self.left_gripper_subscription = self.create_subscription(
            JointState,
            self.left_gripper_topic,
            self._left_gripper_callback,
            10,
        )

        self.right_gripper_subscription = self.create_subscription(
            JointState,
            self.right_gripper_topic,
            self._right_gripper_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_stitched_command,
        )

        self.get_logger().info(
            'CartesianCommandStitcherNode initialized with '
            f'left_frame_topic={self.left_frame_topic}, '
            f'right_frame_topic={self.right_frame_topic}, '
            f'left_gripper_topic={self.left_gripper_topic}, '
            f'right_gripper_topic={self.right_gripper_topic}, '
            f'output_topic={self.output_topic}, '
            f'publish_rate_hz={self.publish_rate_hz}'
        )

    def _left_frame_callback(self, msg: PoseStamped):
        with self._lock:
            self._latest_left_frame = self._pose_to_list(msg)

    def _right_frame_callback(self, msg: PoseStamped):
        with self._lock:
            self._latest_right_frame = self._pose_to_list(msg)

    def _left_gripper_callback(self, msg: JointState):
        gripper_values = self._extract_gripper_values(msg)

        if gripper_values is None:
            return

        with self._lock:
            self._latest_left_gripper = gripper_values

    def _right_gripper_callback(self, msg: JointState):
        gripper_values = self._extract_gripper_values(msg)

        if gripper_values is None:
            return

        with self._lock:
            self._latest_right_gripper = gripper_values

    def _pose_to_list(self, msg: PoseStamped):
        return [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ]

    def _extract_gripper_values(self, msg: JointState):
        values = []

        for key in self.gripper_keys:
            if key not in msg.name:
                self.get_logger().warn(
                    f'Gripper key "{key}" not found in JointState names: {list(msg.name)}',
                    throttle_duration_sec=2.0,
                )
                return None

            index = msg.name.index(key)

            if index >= len(msg.position):
                self.get_logger().warn(
                    f'JointState position array too short for key "{key}" at index {index}',
                    throttle_duration_sec=2.0,
                )
                return None

            values.append(msg.position[index])

        return values

    def _publish_stitched_command(self):
        with self._lock:
            if (
                self._latest_left_frame is None
                or self._latest_right_frame is None
                or self._latest_left_gripper is None
                or self._latest_right_gripper is None
            ):
                return

            stitched_command = (
                self._latest_left_frame
                + self._latest_left_gripper
                + self._latest_right_frame
                + self._latest_right_gripper
            )

        stitched_msg = Float64MultiArray()
        stitched_msg.data = stitched_command
        self.publisher.publish(stitched_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianCommandStitcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()