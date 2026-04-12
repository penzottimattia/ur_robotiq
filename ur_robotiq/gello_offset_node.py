#!/usr/bin/env python3
"""Gello offset computation and joint trajectory command node.

Subscribes to:
- Robot joint states
- Gello joint states

Control modes are switched via ROS2 parameter updates (`SetParameters`):
- 0: idle (no command output)
- 1: normal offset mode
- 2: positive speed mode on one specific joint
- 3: negative speed mode on one specific joint

In normal mode, the node publishes:
target = robot_initial + (gello - gello_initial)
with the gello last joint remapped to the robot gripper range and an
optional gripper offset applied.

In speed modes, a configured gello trigger joint in [0, 1] scales angular
speed (rad/s) for one configured robot joint.
"""

import time

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty


class GelloOffsetNode(Node):
    def __init__(self):
        super().__init__('gello_offset_node')
        
        # Declare parameters
        self.declare_parameter('robot_joint_state_topic', '/joint_states')
        self.declare_parameter('gello_joint_state_topic', '/gello/joint_states')
        self.declare_parameter('command_topic', '/joint_commands')
        self.declare_parameter('gripper_joint_name', '')
        self.declare_parameter('gripper_min', 0.0)
        self.declare_parameter('gripper_max', 0.8)
        self.declare_parameter('gripper_offset', 0.1)
        self.declare_parameter('control_mode', 1)
        self.declare_parameter('speed_mode_joint_name', '')
        self.declare_parameter('speed_trigger_joint_index', -1)
        self.declare_parameter('speed_max_velocity', 1.0)
        self.declare_parameter('mode_transition_delay_seconds', 5.0)
        self.declare_parameter('transition_wait_service_name', 'wait_for_transition_complete')
        self.declare_parameter('tf_prefix', '')
        
        # Get parameter values
        self.robot_joint_state_topic = self.get_parameter('robot_joint_state_topic').get_parameter_value().string_value
        self.gello_joint_state_topic = self.get_parameter('gello_joint_state_topic').get_parameter_value().string_value
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').get_parameter_value().string_value
        self.gripper_min = self.get_parameter('gripper_min').get_parameter_value().double_value
        self.gripper_max = self.get_parameter('gripper_max').get_parameter_value().double_value
        self.gripper_offset = self.get_parameter('gripper_offset').get_parameter_value().double_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().integer_value
        self.speed_mode_joint_name = self.get_parameter('speed_mode_joint_name').get_parameter_value().string_value
        self.speed_trigger_joint_index = self.get_parameter('speed_trigger_joint_index').get_parameter_value().integer_value
        self.speed_max_velocity = self.get_parameter('speed_max_velocity').get_parameter_value().double_value
        self.mode_transition_delay_seconds = self.get_parameter('mode_transition_delay_seconds').get_parameter_value().double_value
        self.transition_wait_service_name = self.get_parameter('transition_wait_service_name').get_parameter_value().string_value
        
        # State variables
        self.latest_robot_positions = None
        self.latest_robot_joint_names = None
        self.robot_initial_positions = None
        self.robot_joint_names = None
        self.latest_gello_positions = None
        self.latest_gello_joint_names = None
        self.gello_initial_positions = None
        self.gello_joint_names = None
        self.initialized = False
        self.speed_mode_last_update_time = None
        self.speed_mode_target_positions = None
        self.mode_transition_block_until_time = None
        self.callback_group = ReentrantCallbackGroup()

        self.parameter_callback = self.add_on_set_parameters_callback(
            self._on_set_parameters
        )

        self.transition_wait_service = self.create_service(
            Empty,
            self.transition_wait_service_name,
            self._handle_transition_wait_service,
            callback_group=self.callback_group,
        )
        
        # Publisher for combined joint-state commands
        self.command_publisher = self.create_publisher(
            JointState,
            self.command_topic,
            10,
        )
        
        # Subscribers
        self.robot_joint_state_sub = self.create_subscription(
            JointState,
            self.robot_joint_state_topic,
            self.robot_joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        
        self.gello_joint_state_sub = self.create_subscription(
            JointState,
            self.gello_joint_state_topic,
            self.gello_joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        
        self.get_logger().debug(
            f'GelloOffsetNode initialized:\n'
            f'  Robot joint states: {self.robot_joint_state_topic}\n'
            f'  Gello joint states: {self.gello_joint_state_topic}\n'
            f'  Command output: {self.command_topic}\n'
            f'  Gripper min/max: {self.gripper_min}/{self.gripper_max}\n'
            f'  Gripper offset: {self.gripper_offset}\n'
            f'  Control mode: {self.control_mode} (0=idle, 1=normal, 2=pos-speed, 3=neg-speed)\n'
            f'  Speed joint: {self.speed_mode_joint_name or "<last arm joint>"}\n'
            f'  Speed trigger joint index: {self.speed_trigger_joint_index}\n'
            f'  Speed max velocity: {self.speed_max_velocity} rad/s\n'
            f'  Mode transition delay: {self.mode_transition_delay_seconds} s\n'
            f'  Transition wait service: {self.transition_wait_service_name}'
        )
    
    def robot_joint_state_callback(self, msg):
        """Store the latest robot joint state and capture the initial reference once."""
        self.latest_robot_joint_names = msg.name
        self.latest_robot_positions = np.array(msg.position)

        if self.robot_initial_positions is not None:
            return

        self.robot_joint_names = msg.name
        self.robot_initial_positions = np.array(msg.position)
        if not self.gripper_joint_name:
            self.gripper_joint_name = self.robot_joint_names[-1]  # Last joint is gripper

        self.get_logger().info(
            f'Captured robot initial positions:\n'
            f'  Joints: {self.robot_joint_names}\n'
            f'  Gripper joint: {self.gripper_joint_name}\n'
            f'  Positions: {self.robot_initial_positions}'
        )

        # Check if initialization is complete
        self._check_initialization()
    
    def gello_joint_state_callback(self, msg):
        """Store the latest gello joint state and capture the initial reference once."""
        self.latest_gello_joint_names = msg.name
        self.latest_gello_positions = np.array(msg.position)

        if self.control_mode == 0:
            return

        if self.control_mode in (2, 3):
            self._publish_speed_mode_command(msg)
            return

        if self._is_mode_transition_block_active():
            return

        if not self.initialized:
            if self.gello_initial_positions is None:
                self.gello_joint_names = msg.name
                self.gello_initial_positions = np.array(msg.position)
                
                self.get_logger().info(
                    f'Captured gello initial positions:\n'
                    f'  Joints: {self.gello_joint_names}\n'
                    f'  Positions: {self.gello_initial_positions}'
                )
                
                # Check if initialization is complete
                self._check_initialization()
            return

        # Extract arm and gripper joints (assume last joint is gripper)
        gello_arm_positions = np.array(msg.position[:-1])
        gello_gripper_position = msg.position[-1]

        gello_arm_initial = self.gello_initial_positions[:-1]

        # Compute current offset from initial gello state for arm
        gello_arm_change = gello_arm_positions - gello_arm_initial

        # Target positions for arm = robot_initial + gello_change
        target_arm_positions = self.robot_initial_positions[:-1] + gello_arm_change

        # Handle gripper: remap from 0-1 to gripper_min/gripper_max
        gripper_value = (
            self.gripper_min
            + (gello_gripper_position * (self.gripper_max - self.gripper_min))
            + self.gripper_offset
        )

        self._publish_joint_command(
            list(self.robot_joint_names[:-1]) + [self.gripper_joint_name],
            target_arm_positions.tolist() + [gripper_value],
            0,
        )

    def _on_set_parameters(self, parameters):
        """Handle runtime updates for control mode and speed-mode parameters."""
        new_control_mode = self.control_mode
        new_speed_mode_joint_name = self.speed_mode_joint_name
        new_speed_trigger_joint_index = self.speed_trigger_joint_index
        new_speed_max_velocity = self.speed_max_velocity
        new_mode_transition_delay_seconds = self.mode_transition_delay_seconds

        for param in parameters:
            if param.name == 'control_mode':
                if param.value not in (0, 1, 2, 3):
                    return SetParametersResult(
                        successful=False,
                        reason='control_mode must be one of {0, 1, 2, 3}',
                    )
                new_control_mode = int(param.value)
            elif param.name == 'speed_mode_joint_name':
                new_speed_mode_joint_name = str(param.value)
            elif param.name == 'speed_trigger_joint_index':
                new_speed_trigger_joint_index = int(param.value)
            elif param.name == 'speed_max_velocity':
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='speed_max_velocity must be >= 0.0',
                    )
                new_speed_max_velocity = float(param.value)
            elif param.name == 'mode_transition_delay_seconds':
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='mode_transition_delay_seconds must be >= 0.0',
                    )
                new_mode_transition_delay_seconds = float(param.value)

        previous_mode = self.control_mode
        transitioning_to_normal = previous_mode != 1 and new_control_mode == 1
        transitioning_to_idle = new_control_mode == 0
        transitioning_to_active_mode = previous_mode == 0 and new_control_mode in (1, 2, 3)
        transitioning_to_normal_from_idle = previous_mode == 0 and new_control_mode == 1
        transitioning_between_active_modes = (
            previous_mode in (1, 2, 3)
            and new_control_mode in (1, 2, 3)
            and previous_mode != new_control_mode
        )

        if transitioning_to_normal:
            if not self._recompute_offsets_from_latest():
                return SetParametersResult(
                    successful=False,
                    reason=(
                        'Cannot transition to normal mode without both latest robot and '
                        'gello joint states to recompute offsets.'
                    ),
                )

        self.control_mode = new_control_mode
        self.speed_mode_joint_name = new_speed_mode_joint_name
        self.speed_trigger_joint_index = new_speed_trigger_joint_index
        self.speed_max_velocity = new_speed_max_velocity
        self.mode_transition_delay_seconds = new_mode_transition_delay_seconds

        if self.control_mode in (2, 3):
            self.speed_mode_last_update_time = None
            self.speed_mode_target_positions = None

        if not transitioning_to_idle and not transitioning_to_normal_from_idle and (
            transitioning_to_active_mode or transitioning_between_active_modes
        ):
            self.mode_transition_block_until_time = (
                self.get_clock().now().nanoseconds
                + int(self.mode_transition_delay_seconds * 1e9)
            )
        else:
            self.mode_transition_block_until_time = None

        self.get_logger().debug(
            f'Updated control configuration: mode={self.control_mode}, '
            f'speed_joint={self.speed_mode_joint_name or "<last arm joint>"}, '
            f'trigger_index={self.speed_trigger_joint_index}, '
            f'max_velocity={self.speed_max_velocity}, '
            f'transition_delay={self.mode_transition_delay_seconds}'
        )

        return SetParametersResult(successful=True, reason='')

    def _recompute_offsets_from_latest(self):
        """Reset normal-mode references from the latest observed robot and gello states."""
        if self.latest_robot_positions is None or self.latest_robot_joint_names is None:
            return False

        if self.latest_gello_positions is None or self.latest_gello_joint_names is None:
            return False

        self.robot_joint_names = list(self.latest_robot_joint_names)
        self.robot_initial_positions = np.array(self.latest_robot_positions, copy=True)
        if not self.gripper_joint_name:
            self.gripper_joint_name = self.robot_joint_names[-1]

        self.gello_joint_names = list(self.latest_gello_joint_names)
        self.gello_initial_positions = np.array(self.latest_gello_positions, copy=True)
        self.initialized = True
        self.get_logger().debug('Offsets recomputed from latest robot and gello joint states.')
        return True

    def _is_mode_transition_block_active(self):
        """Return True while the configured mode-transition delay is still in effect."""
        if self.mode_transition_block_until_time is None:
            return False

        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.mode_transition_block_until_time:
            return True

        self.mode_transition_block_until_time = None
        return False

    def _handle_transition_wait_service(self, _request, response):
        """Block until the current mode-transition delay has elapsed."""
        while self._is_mode_transition_block_active():
            time.sleep(0.05)

        return response

    def _publish_speed_mode_command(self, gello_msg):
        """Publish one-joint speed command based on trigger value in [0, 1]."""
        if self.latest_robot_positions is None or self.latest_robot_joint_names is None:
            return

        if len(self.latest_robot_positions) < 1:
            return

        joint_names = list(self.latest_robot_joint_names)
        if len(joint_names) < 1:
            return

        # Default to the last arm joint (joint before gripper) when not explicitly configured.
        if self.speed_mode_joint_name:
            if self.speed_mode_joint_name not in joint_names:
                self.get_logger().warn(
                    f'speed_mode_joint_name={self.speed_mode_joint_name} not found in robot joints.'
                )
                return
            speed_joint_index = joint_names.index(self.speed_mode_joint_name)
        else:
            speed_joint_index = max(len(joint_names) - 2, 0)

        trigger_index = int(self.speed_trigger_joint_index)
        if trigger_index < 0:
            trigger_index = len(gello_msg.position) - 1

        if trigger_index < 0 or trigger_index >= len(gello_msg.position):
            self.get_logger().warn(
                f'speed_trigger_joint_index={trigger_index} is out of range for gello message size '
                f'{len(gello_msg.position)}.'
            )
            return

        if self._is_mode_transition_block_active():
            return

        trigger = float(np.clip(gello_msg.position[trigger_index], 0.0, 1.0))
        direction = 1.0 if self.control_mode == 2 else -1.0

        now_ns = self.get_clock().now().nanoseconds
        if self.speed_mode_last_update_time is None:
            dt = 0.0
        else:
            dt = max((now_ns - self.speed_mode_last_update_time) * 1e-9, 0.0)
        self.speed_mode_last_update_time = now_ns

        if self.speed_mode_target_positions is None:
            self.speed_mode_target_positions = np.array(self.latest_robot_positions, copy=True)
            self.speed_mode_target_positions[-1] += self.gripper_offset  # Ensure gripper offset is applied in speed mode as well

        target_positions = np.array(self.speed_mode_target_positions, copy=True)
        delta = direction * trigger * self.speed_max_velocity * dt
        self.speed_mode_target_positions[speed_joint_index] += delta
        target_positions[speed_joint_index] = self.speed_mode_target_positions[speed_joint_index]
        target_positions.clip(-2*np.pi, 2*np.pi, out=target_positions)  # Force reasonable joint limits 

        self._publish_joint_command(
            joint_names,
            target_positions.tolist(),
            0,
        )

    def _publish_joint_command(self, joint_names, positions, duration_seconds):
        """Publish a JointState command with a duration encoded in the header stamp."""
        command_msg = JointState()
        command_msg.header.stamp.sec = int(duration_seconds)
        command_msg.header.stamp.nanosec = 0
        command_msg.name = list(joint_names)
        command_msg.position = list(positions)

        self.command_publisher.publish(command_msg)
    
    def _check_initialization(self):
        """Check if both initial states have been captured"""
        if (self.robot_initial_positions is not None and 
            self.gello_initial_positions is not None and
            not self.initialized):
            
            self.initialized = True
            self.get_logger().info(
                f'Initialization complete. Starting offset computation.\n'
                f'  Robot joints: {len(self.robot_joint_names)}\n'
                f'  Gello joints: {len(self.gello_joint_names)} '
                f'(first {len(self.gello_joint_names)-1} for arm, last 1 for gripper)'
            )


def main():
    rclpy.init()
    node = GelloOffsetNode()
    
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
