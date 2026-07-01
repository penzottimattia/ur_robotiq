#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointStateToTrajectoryNode(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')

        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('trajectory_topic', '/joint_trajectory')
        self.declare_parameter('gripper_topic', '/gripper_trajectory')
        self.declare_parameter('gripper_joint', '')
        self.declare_parameter('gripper_joint_list', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('gripper_threshold', 0.0)
        self.declare_parameter('gripper_full_close_threshold', 0.8)
        self.declare_parameter('gripper_offset', 0.0)

        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        self.gripper_joint = self.get_parameter('gripper_joint').get_parameter_value().string_value
        self.gripper_joint_list = list(
            self.get_parameter('gripper_joint_list').get_parameter_value().string_array_value
        )
        self.gripper_threshold = self.get_parameter('gripper_threshold').get_parameter_value().double_value
        self.gripper_full_close_threshold = (
            self.get_parameter('gripper_full_close_threshold').get_parameter_value().double_value
        )
        self.gripper_offset = self.get_parameter('gripper_offset').get_parameter_value().double_value

        if not self.gripper_joint_list and self.gripper_joint:
            self.gripper_joint_list = [self.gripper_joint]

        self.publisher = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic,
            10,
        )

        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            self.gripper_topic,
            10,
        )

        self._subscription_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile=self._subscription_qos,
        )

    def _with_prefix(self, joint_name: str) -> str:
        if not self.tf_prefix:
            return joint_name
        if joint_name.startswith(self.tf_prefix):
            return joint_name
        return self.tf_prefix + joint_name

    def _without_prefix(self, joint_name: str) -> str:
        if self.tf_prefix and joint_name.startswith(self.tf_prefix):
            return joint_name[len(self.tf_prefix):]
        return joint_name

    def _resolve_gripper_joints(self, msg_names):
        """
        Resolve configured gripper joints against incoming JointState names.

        Returns:
            resolved_gripper_names: names exactly as present in msg.name
            output_gripper_names: names to publish in gripper trajectory
        """
        msg_name_set = set(msg_names)

        resolved_gripper_names = []
        output_gripper_names = []

        for configured_name in self.gripper_joint_list:
            candidates = [
                configured_name,
                self._with_prefix(configured_name),
                self._without_prefix(configured_name),
            ]

            matched_name = None
            for candidate in candidates:
                if candidate in msg_name_set:
                    matched_name = candidate
                    break

            if matched_name is None:
                self.get_logger().warn(
                    f"Configured gripper joint '{configured_name}' was not found in JointState. "
                    f"Available joints: {list(msg_names)}"
                )
                continue

            resolved_gripper_names.append(matched_name)

            # Publish the gripper joint name as it appears in the incoming JointState.
            # This avoids accidentally double-prefixing names.
            output_gripper_names.append(matched_name)

            # If incoming JointState is unprefixed, publish prefixed gripper names.
            # If already prefixed, keep as-is.
            if self.tf_prefix and not matched_name.startswith(self.tf_prefix):
                output_gripper_names[-1] = self._with_prefix(matched_name)

        return resolved_gripper_names, output_gripper_names

    def joint_state_callback(self, msg: JointState):
        if not msg.name:
            self.get_logger().warn("Received JointState with no joint names.")
            return

        if len(msg.position) != len(msg.name):
            self.get_logger().warn(
                f"JointState position/name size mismatch: "
                f"{len(msg.position)} positions for {len(msg.name)} names. Ignoring message."
            )
            return

        has_velocity = len(msg.velocity) == len(msg.name)

        if msg.velocity and not has_velocity:
            self.get_logger().warn(
                f"JointState velocity/name size mismatch: "
                f"{len(msg.velocity)} velocities for {len(msg.name)} names. "
                f"Velocities will be ignored for this message."
            )

        name_to_index = {name: index for index, name in enumerate(msg.name)}

        resolved_gripper_names, output_gripper_names = self._resolve_gripper_joints(msg.name)
        gripper_name_set = set(resolved_gripper_names)

        if resolved_gripper_names:
            self.publish_gripper_trajectory(
                msg,
                name_to_index,
                resolved_gripper_names,
                output_gripper_names,
            )

        self.publish_arm_trajectory(
            msg,
            gripper_name_set,
            has_velocity,
        )

    def publish_gripper_trajectory(
        self,
        msg: JointState,
        name_to_index,
        resolved_gripper_names,
        output_gripper_names,
    ):
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = output_gripper_names

        point = JointTrajectoryPoint()

        gripper_positions = []
        for joint_name in resolved_gripper_names:
            index = name_to_index[joint_name]
            gripper_positions.append(msg.position[index] + self.gripper_offset)

        if gripper_positions:
            if self.gripper_threshold > 0.0:
                gripper_positions[0] = 1.0 if gripper_positions[0] > self.gripper_threshold else 0.0

            if (
                self.gripper_full_close_threshold > 0.0
                and gripper_positions[0] > self.gripper_full_close_threshold
            ):
                gripper_positions[0] = 1.0

        point.positions = gripper_positions

        # Keep your original behavior, although usually time_from_start should be a command duration,
        # not the absolute JointState timestamp.
        point.time_from_start = rclpy.duration.Duration(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
        ).to_msg()

        gripper_msg.points = [point]
        self.gripper_publisher.publish(gripper_msg)

    def publish_arm_trajectory(
        self,
        msg: JointState,
        gripper_name_set,
        has_velocity,
    ):
        traj = JointTrajectory()

        arm_joint_names = []
        arm_positions = []
        arm_velocities = []

        for index, joint_name in enumerate(msg.name):
            if joint_name in gripper_name_set:
                continue

            output_joint_name = joint_name

            # If incoming JointState is unprefixed, publish prefixed arm names.
            # If already prefixed, keep as-is.
            if self.tf_prefix and not joint_name.startswith(self.tf_prefix):
                output_joint_name = self.tf_prefix + joint_name

            arm_joint_names.append(output_joint_name)
            arm_positions.append(msg.position[index])

            if has_velocity:
                arm_velocities.append(msg.velocity[index])

        if not arm_joint_names:
            self.get_logger().warn("No arm joints left after separating gripper joints.")
            return

        traj.joint_names = arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = arm_positions

        if has_velocity:
            point.velocities = arm_velocities

        point.time_from_start = rclpy.duration.Duration(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
        ).to_msg()

        traj.points = [point]
        self.publisher.publish(traj)


def main():
    rclpy.init()
    node = JointStateToTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()