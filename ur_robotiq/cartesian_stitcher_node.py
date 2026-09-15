#!/usr/bin/env python3
"""Bridge two Cartesian and two JointState command streams into a single Float64MultiArray.

The node subscribes to two PoseStamped arm command topics and two JointState gripper command
topics, extracts pose vectors and indexed gripper positions, optionally estimates Cartesian
linear velocities, concatenates everything, and republishes the result as a
std_msgs/Float64MultiArray at a fixed rate.

Output layout when append_linear_velocity_estimates=True:

[
    left_position.xyz,
    left_orientation.wxyz,
    left_gripper_values...,

    right_position.xyz,
    right_orientation.wxyz,
    right_gripper_values...,

    left_linear_velocity.vx_vy_vz,
    right_linear_velocity.vx_vy_vz,
]

With one gripper key per hand, the output length is:

    7 + 1 + 7 + 1 + 3 + 3 = 22

where each pose is:

[
    position.x,
    position.y,
    position.z,
    orientation.w,
    orientation.x,
    orientation.y,
    orientation.z,
]
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped


class Kalman1DPositionVelocity:
    """Simple 1D constant-velocity Kalman filter.

    State:
        x = [position, velocity]

    Measurement:
        z = position
    """

    def __init__(self, process_variance: float, measurement_variance: float):
        self.q = float(process_variance)
        self.r = float(measurement_variance)

        self.x = 0.0
        self.v = 0.0

        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0

        self.initialized = False

    def reset(self):
        self.x = 0.0
        self.v = 0.0

        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0

        self.initialized = False

    def update(self, measurement: float, dt: float) -> float:
        if not self.initialized:
            self.x = float(measurement)
            self.v = 0.0
            self.initialized = True
            return self.v

        dt = max(float(dt), 1.0e-6)

        # Predict state.
        self.x = self.x + dt * self.v

        # Predict covariance: P = F P F^T + Q.
        # F = [[1, dt],
        #      [0, 1 ]]
        q00 = 0.25 * dt**4 * self.q
        q01 = 0.5 * dt**3 * self.q
        q10 = q01
        q11 = dt**2 * self.q

        p00 = self.p00 + dt * self.p10 + dt * self.p01 + dt**2 * self.p11 + q00
        p01 = self.p01 + dt * self.p11 + q01
        p10 = self.p10 + dt * self.p11 + q10
        p11 = self.p11 + q11

        self.p00 = p00
        self.p01 = p01
        self.p10 = p10
        self.p11 = p11

        # Update with position measurement.
        y = float(measurement) - self.x
        s = self.p00 + self.r

        if s <= 1.0e-12:
            return self.v

        k0 = self.p00 / s
        k1 = self.p10 / s

        self.x = self.x + k0 * y
        self.v = self.v + k1 * y

        # P = (I - K H) P, H = [1, 0]
        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11

        self.p00 = (1.0 - k0) * old_p00
        self.p01 = (1.0 - k0) * old_p01
        self.p10 = old_p10 - k1 * old_p00
        self.p11 = old_p11 - k1 * old_p01

        return self.v


class PoseLinearVelocityEstimator:
    """Estimate Cartesian linear velocity from PoseStamped messages.

    Linear velocity is estimated from position changes.

    If use_kalman_filter=True, a constant-velocity Kalman filter is used per axis.
    If use_kalman_filter=False, raw finite differences are used.
    """

    def __init__(
        self,
        node: Node,
        use_kalman_filter: bool = True,
        position_process_variance: float = 1.0,
        position_measurement_variance: float = 1.0e-4,
        max_dt: float = 1.0,
    ):
        self.node = node
        self.use_kalman_filter = bool(use_kalman_filter)
        self.max_dt = float(max_dt)

        self.prev_position = None
        self.prev_time = None

        self.linear_filters = [
            Kalman1DPositionVelocity(
                position_process_variance,
                position_measurement_variance,
            )
            for _ in range(3)
        ]

        self.latest_linear_velocity = [0.0] * 3

    def reset(self):
        self.prev_position = None
        self.prev_time = None
        self.latest_linear_velocity = [0.0] * 3

        for filt in self.linear_filters:
            filt.reset()

    def update(self, msg: PoseStamped):
        position = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ]

        stamp = self._stamp_to_seconds(msg)

        if self.prev_position is None or self.prev_time is None:
            self.prev_position = position
            self.prev_time = stamp
            self.latest_linear_velocity = [0.0] * 3
            return self.latest_linear_velocity

        dt = stamp - self.prev_time

        if dt <= 1.0e-6 or dt > self.max_dt:
            self.prev_position = position
            self.prev_time = stamp

            # Keep previous estimate if timing is invalid or stale.
            return self.latest_linear_velocity

        raw_linear = [
            (position[i] - self.prev_position[i]) / dt
            for i in range(3)
        ]

        if self.use_kalman_filter:
            linear = [
                self.linear_filters[i].update(position[i], dt)
                for i in range(3)
            ]
        else:
            linear = raw_linear

        self.latest_linear_velocity = linear

        self.prev_position = position
        self.prev_time = stamp

        return self.latest_linear_velocity

    def _stamp_to_seconds(self, msg: PoseStamped) -> float:
        """Use message header stamp when available, otherwise fall back to node clock."""
        stamp = msg.header.stamp

        if stamp.sec == 0 and stamp.nanosec == 0:
            now = self.node.get_clock().now()
            return now.nanoseconds * 1.0e-9

        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


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

        # Linear velocity estimation parameters.
        self.declare_parameter('append_linear_velocity_estimates', True)
        self.declare_parameter('use_kalman_filter', True)
        self.declare_parameter('velocity_max_dt', 1.0)

        # Kalman tuning for linear velocity estimation.
        self.declare_parameter('linear_process_variance', 1.0)
        self.declare_parameter('linear_measurement_variance', 1.0e-4)

        self.left_frame_topic = self.get_parameter('left_frame_topic').value
        self.right_frame_topic = self.get_parameter('right_frame_topic').value
        self.left_gripper_topic = self.get_parameter('left_gripper_topic').value
        self.right_gripper_topic = self.get_parameter('right_gripper_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.gripper_keys = list(self.get_parameter('gripper_keys').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.append_linear_velocity_estimates = bool(
            self.get_parameter('append_linear_velocity_estimates').value
        )
        self.use_kalman_filter = bool(self.get_parameter('use_kalman_filter').value)
        self.velocity_max_dt = float(self.get_parameter('velocity_max_dt').value)

        self.linear_process_variance = float(
            self.get_parameter('linear_process_variance').value
        )
        self.linear_measurement_variance = float(
            self.get_parameter('linear_measurement_variance').value
        )

        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        if self.velocity_max_dt <= 0.0:
            raise ValueError('velocity_max_dt must be greater than zero')

        self._lock = threading.Lock()

        self._latest_left_frame = None
        self._latest_right_frame = None
        self._latest_left_gripper = None
        self._latest_right_gripper = None

        self._latest_left_linear_velocity = None
        self._latest_right_linear_velocity = None

        self.left_linear_velocity_estimator = PoseLinearVelocityEstimator(
            node=self,
            use_kalman_filter=self.use_kalman_filter,
            position_process_variance=self.linear_process_variance,
            position_measurement_variance=self.linear_measurement_variance,
            max_dt=self.velocity_max_dt,
        )

        self.right_linear_velocity_estimator = PoseLinearVelocityEstimator(
            node=self,
            use_kalman_filter=self.use_kalman_filter,
            position_process_variance=self.linear_process_variance,
            position_measurement_variance=self.linear_measurement_variance,
            max_dt=self.velocity_max_dt,
        )

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
            f'publish_rate_hz={self.publish_rate_hz}, '
            f'append_linear_velocity_estimates={self.append_linear_velocity_estimates}, '
            f'use_kalman_filter={self.use_kalman_filter}, '
            f'velocity_max_dt={self.velocity_max_dt}'
        )

    def _left_frame_callback(self, msg: PoseStamped):
        with self._lock:
            self._latest_left_frame = self._pose_to_list(msg)
            self._latest_left_linear_velocity = (
                self.left_linear_velocity_estimator.update(msg)
            )

    def _right_frame_callback(self, msg: PoseStamped):
        with self._lock:
            self._latest_right_frame = self._pose_to_list(msg)
            self._latest_right_linear_velocity = (
                self.right_linear_velocity_estimator.update(msg)
            )

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
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.w),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
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

            values.append(float(msg.position[index]))

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

            if self.append_linear_velocity_estimates and (
                self._latest_left_linear_velocity is None
                or self._latest_right_linear_velocity is None
            ):
                return

            stitched_command = (
                self._latest_left_frame
                + self._latest_left_gripper
                + self._latest_right_frame
                + self._latest_right_gripper
            )

            if self.append_linear_velocity_estimates:
                stitched_command = (
                    stitched_command
                    + self._latest_left_linear_velocity
                    + self._latest_right_linear_velocity
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