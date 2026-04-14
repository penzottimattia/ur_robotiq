#!/usr/bin/env python3
"""Bridge two GELLO command streams into a single Float64MultiArray.

The node subscribes to two JointState command topics, extracts the position
vectors from each, concatenates them, and republishes the result as a
std_msgs/Float64MultiArray.
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GelloCommandStitcherNode(Node):
    def __init__(self):
        super().__init__('gello_stitcher_node')

        self.declare_parameter('left_command_topic', '/gello_1/command_joints')
        self.declare_parameter('right_command_topic', '/gello_2/command_joints')
        self.declare_parameter('output_topic', '/gello_stitched_commands')

        self.left_command_topic = self.get_parameter('left_command_topic').value
        self.right_command_topic = self.get_parameter('right_command_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self._lock = threading.Lock()
        self._latest_left_positions = None
        self._latest_right_positions = None

        self.publisher = self.create_publisher(Float64MultiArray, self.output_topic, 10)

        self.left_subscription = self.create_subscription(
            JointState,
            self.left_command_topic,
            self._left_command_callback,
            10,
        )
        self.right_subscription = self.create_subscription(
            JointState,
            self.right_command_topic,
            self._right_command_callback,
            10,
        )

        self.get_logger().info(
            'GelloCommandStitcherNode initialized with '
            f'left={self.left_command_topic}, '
            f'right={self.right_command_topic}, '
            f'output={self.output_topic}'
        )

    def _left_command_callback(self, msg):
        with self._lock:
            self._latest_left_positions = list(msg.position)
        self._publish_stitched_command()

    def _right_command_callback(self, msg):
        with self._lock:
            self._latest_right_positions = list(msg.position)
        self._publish_stitched_command()

    def _publish_stitched_command(self):
        with self._lock:
            if self._latest_left_positions is None or self._latest_right_positions is None:
                return

            stitched_positions = (
                list(self._latest_left_positions) + list(self._latest_right_positions)
            )

        stitched_msg = Float64MultiArray()
        stitched_msg.data = stitched_positions
        self.publisher.publish(stitched_msg)


def main():
    rclpy.init()
    node = GelloCommandStitcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()