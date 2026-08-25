#!/usr/bin/env python3

from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import MultiDOFCommand


class JointStateToTrajectoryNode(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')

        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('trajectory_topic', '/joint_trajectory')
        self.declare_parameter('gripper_topic', '/gripper_trajectory')
        self.declare_parameter('gripper_as_multi_dof', False)

        # hand_map_file is an alternative to gripper_joint/gripper_joint_list.
        # YAML format:
        #   parent_joint:
        #     child_joint: multiplier
        self.declare_parameter('hand_map_file', '')
        self.declare_parameter('gripper_joint', '')
        self.declare_parameter('gripper_joint_list', [''])
        self.declare_parameter('gripper_threshold', 0.0)
        self.declare_parameter('gripper_full_close_threshold', 0.8)
        self.declare_parameter('gripper_offset', 0.0)

        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        self.hand_map_file = self.get_parameter('hand_map_file').get_parameter_value().string_value
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

        self.hand_map = self._load_hand_map(self.hand_map_file)
        if self.hand_map and (self.gripper_joint or self.gripper_joint_list):
            self.get_logger().info(
                'hand_map_file is configured; gripper_joint and gripper_joint_list will be ignored.'
            )

        self.publisher = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.gripper_as_multi_dof = self.get_parameter('gripper_as_multi_dof').get_parameter_value().bool_value
        if self.gripper_as_multi_dof:
            self.gripper_publisher = self.create_publisher(MultiDOFCommand, self.gripper_topic, 10)
        else:
            self.gripper_publisher = self.create_publisher(JointTrajectory, self.gripper_topic, 10)

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

    def _load_hand_map(self, file_name: str):
        if not file_name:
            return {}

        path = Path(file_name).expanduser()
        try:
            with path.open('r', encoding='utf-8') as stream:
                raw_map = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise RuntimeError(f"Unable to load hand map '{path}': {exc}") from exc

        if not isinstance(raw_map, dict):
            raise RuntimeError(f"Hand map '{path}' must contain a YAML mapping at its root.")

        hand_map = {}
        for parent, children in raw_map.items():
            if not isinstance(parent, str) or not parent:
                raise RuntimeError('Every hand-map parent joint must be a non-empty string.')
            if children is None:
                children = {}
            if not isinstance(children, dict):
                raise RuntimeError(
                    f"Children of hand-map parent '{parent}' must be a YAML mapping."
                )

            converted_children = {}
            for child, multiplier in children.items():
                if not isinstance(child, str) or not child:
                    raise RuntimeError(
                        f"Every child of hand-map parent '{parent}' must be a non-empty string."
                    )
                if isinstance(multiplier, bool) or not isinstance(multiplier, (int, float)):
                    raise RuntimeError(
                        f"Multiplier for '{parent}' -> '{child}' must be numeric."
                    )
                converted_children[child] = float(multiplier)
            hand_map[parent] = converted_children

        self.get_logger().info(
            f"Loaded hand map '{path}' with {len(hand_map)} parent joint(s)."
        )
        return hand_map

    def _with_prefix(self, joint_name: str) -> str:
        if not self.tf_prefix or joint_name.startswith(self.tf_prefix):
            return joint_name
        return self.tf_prefix + joint_name

    def _without_prefix(self, joint_name: str) -> str:
        if self.tf_prefix and joint_name.startswith(self.tf_prefix):
            return joint_name[len(self.tf_prefix):]
        return joint_name

    def _resolve_joint_name(self, configured_name: str, msg_names, warn=False):
        msg_name_set = set(msg_names)
        candidates = [
            configured_name,
            self._with_prefix(configured_name),
            self._without_prefix(configured_name),
        ]
        for candidate in dict.fromkeys(candidates):
            if candidate in msg_name_set:
                return candidate

        # This also makes a map using j_index_fle compatible with an incoming
        # name such as hand_j_index_fle or left_hand_j_index_fle.
        suffix_matches = [
            name for name in msg_names
            if name.endswith(configured_name)
            or self._without_prefix(name).endswith(configured_name)
        ]
        if len(suffix_matches) == 1:
            return suffix_matches[0]

        if warn:
            detail = (
                f"; ambiguous suffix matches: {suffix_matches}"
                if suffix_matches else ''
            )
            self.get_logger().warn(
                f"Configured gripper joint '{configured_name}' was not found in JointState"
                f"{detail}. Available joints: {list(msg_names)}"
            )
        return None

    def _output_name(self, configured_name: str, parent_configured='', parent_resolved='') -> str:
        name = configured_name
        # Preserve an intermediate namespace such as 'hand_' from the resolved
        # parent when the YAML uses the shorter MIA names (j_*).
        if parent_configured and parent_resolved:
            unprefixed_parent = self._without_prefix(parent_resolved)
            if unprefixed_parent.endswith(parent_configured):
                stem = unprefixed_parent[:-len(parent_configured)]
                if stem and not name.startswith(stem):
                    name = stem + name
        return self._with_prefix(name)

    def _resolve_legacy_gripper_joints(self, msg_names):
        resolved_names = []
        output_names = []
        for configured_name in self.gripper_joint_list:
            matched_name = self._resolve_joint_name(configured_name, msg_names)
            if matched_name is None:
                continue
            resolved_names.append(matched_name)
            output_names.append(self._with_prefix(self._without_prefix(matched_name)))
        return resolved_names, output_names

    def _apply_gripper_shaping(self, position: float) -> float:
        position += self.gripper_offset
        if self.gripper_threshold > 0.0:
            position = 1.0 if position > self.gripper_threshold else 0.0
        if (
            self.gripper_full_close_threshold > 0.0
            and position > self.gripper_full_close_threshold
        ):
            position = 1.0
        return position

    def joint_state_callback(self, msg: JointState):
        if not msg.name:
            self.get_logger().warn('Received JointState with no joint names.')
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
        gripper_input_names = set()

        if self.hand_map:
            gripper_input_names = self.publish_mapped_gripper_trajectory(
                msg, name_to_index
            )
        else:
            resolved_names, output_names = self._resolve_legacy_gripper_joints(msg.name)
            gripper_input_names.update(resolved_names)
            if resolved_names:
                self.publish_legacy_gripper_trajectory(
                    msg, name_to_index, resolved_names, output_names
                )

        self.publish_arm_trajectory(msg, gripper_input_names, has_velocity)

    def publish_gripper(self, msg : JointTrajectory):
        if self.gripper_as_multi_dof:
            multi_dof_msg = MultiDOFCommand()
            multi_dof_msg.dof_names = msg.joint_names
            multi_dof_msg.values = msg.points[0].positions if msg.points else []
            self.gripper_publisher.publish(multi_dof_msg)
        else:
            self.gripper_publisher.publish(msg)

    def publish_mapped_gripper_trajectory(self, msg: JointState, name_to_index):
        joint_names = []
        positions = []
        gripper_input_names = set()

        def append_command(name, value):
            if name in joint_names:
                index = joint_names.index(name)
                positions[index] = value
                return
            joint_names.append(name)
            positions.append(value)

        for parent_configured, children in self.hand_map.items():
            parent_resolved = self._resolve_joint_name(parent_configured, msg.name)
            if parent_resolved is None:
                continue

            gripper_input_names.add(parent_resolved)
            parent_position = self._apply_gripper_shaping(
                msg.position[name_to_index[parent_resolved]]
            )
            append_command(
                self._output_name(parent_configured, parent_configured, parent_resolved),
                parent_position,
            )

            for child_configured, multiplier in children.items():
                child_output = self._output_name(
                    child_configured, parent_configured, parent_resolved
                )
                append_command(child_output, parent_position * multiplier)

                # If a mapped child is also present in the incoming JointState,
                # keep it out of the arm trajectory.
                child_resolved = self._resolve_joint_name(
                    child_configured, msg.name, warn=False
                )
                if child_resolved is not None:
                    gripper_input_names.add(child_resolved)

        if not joint_names:
            return gripper_input_names

        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
        ).to_msg()
        gripper_msg.points = [point]
        self.publish_gripper(gripper_msg)
        return gripper_input_names

    def publish_legacy_gripper_trajectory(
        self, msg, name_to_index, resolved_names, output_names
    ):
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = output_names
        point = JointTrajectoryPoint()
        point.positions = [
            self._apply_gripper_shaping(msg.position[name_to_index[name]])
            for name in resolved_names
        ]
        point.time_from_start = rclpy.duration.Duration(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
        ).to_msg()
        gripper_msg.points = [point]
        self.publish_gripper(gripper_msg)
        

    def publish_arm_trajectory(self, msg, gripper_name_set, has_velocity):
        traj = JointTrajectory()
        arm_joint_names = []
        arm_positions = []
        arm_velocities = []

        for index, joint_name in enumerate(msg.name):
            if joint_name in gripper_name_set:
                continue
            output_joint_name = self._with_prefix(joint_name)
            arm_joint_names.append(output_joint_name)
            arm_positions.append(msg.position[index])
            if has_velocity:
                arm_velocities.append(msg.velocity[index])

        if not arm_joint_names:
            self.get_logger().warn('No arm joints left after separating gripper joints.')
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