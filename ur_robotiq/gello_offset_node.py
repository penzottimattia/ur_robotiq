#!/usr/bin/env python3
"""Gello offset computation and joint trajectory command node

Subscribes to:
- Robot joint states (once, to get initial position)
- Gello joint states (continuously)

Computes the offset between gello and robot initial positions, then
publishes adjusted joint states. A downstream joint-state-to-trajectory
node converts those commands into controller trajectories.

Joint mapping is automatically deduced by order:
- First N-1 gello joints map to all robot joints (by index order)
- Last gello joint is treated as the gripper and remapped to gripper_min/gripper_max

The node:
1. Waits for the first robot joint state message as the reference
2. Waits for the first gello joint state message as the reference
3. Computes the offset: offset = gello_initial - robot_initial
4. On each subsequent gello message, publishes: target = robot_initial + (gello - initial_gello_state)
5. Publishes gripper position remapped from [0, 1] to [gripper_min, gripper_max]
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class GelloOffsetNode(Node):
    def __init__(self):
        super().__init__('gello_offset_node')
        
        # Declare parameters
        self.declare_parameter('robot_joint_state_topic', '/joint_states')
        self.declare_parameter('gello_joint_state_topic', '/gello/joint_states')
        self.declare_parameter('command_topic', '/joint_commands')
        self.declare_parameter('reset_service_name', 'reset_offsets')
        self.declare_parameter('pause_service_name', 'pause_publisher')
        self.declare_parameter('close_gripper_service_name', 'close_gripper')
        self.declare_parameter('gripper_joint_name', '')
        self.declare_parameter('gripper_min', 0.0)
        self.declare_parameter('gripper_max', 0.8)
        self.declare_parameter('tf_prefix', '')
        
        # Get parameter values
        self.robot_joint_state_topic = self.get_parameter('robot_joint_state_topic').get_parameter_value().string_value
        self.gello_joint_state_topic = self.get_parameter('gello_joint_state_topic').get_parameter_value().string_value
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').get_parameter_value().string_value
        self.reset_service_name = self.get_parameter('reset_service_name').get_parameter_value().string_value
        self.pause_service_name = self.get_parameter('pause_service_name').get_parameter_value().string_value
        self.close_gripper_service_name = self.get_parameter('close_gripper_service_name').get_parameter_value().string_value
        self.gripper_min = self.get_parameter('gripper_min').get_parameter_value().double_value
        self.gripper_max = self.get_parameter('gripper_max').get_parameter_value().double_value
        
        # State variables
        self.latest_robot_positions = None
        self.latest_robot_joint_names = None
        self.robot_initial_positions = None
        self.robot_joint_names = None
        self.latest_gello_positions = None
        self.latest_gello_joint_names = None
        self.gello_initial_positions = None
        self.gello_joint_names = None
        self.gello_offset = None
        self.initialized = False
        self.paused = False
        
        # Publisher for combined joint-state commands
        self.command_publisher = self.create_publisher(
            JointState,
            self.command_topic,
            10,
        )

        # Service to recompute the offset reference from the latest observed states
        self.reset_service = self.create_service(
            Trigger,
            self.reset_service_name,
            self.reset_offsets_callback,
        )

        # Service to stop publishing until the offsets are explicitly reset.
        self.pause_service = self.create_service(
            Trigger,
            self.pause_service_name,
            self.pause_publisher_callback,
        )

        # Service to send the latest arm state with the gripper driven to a closed position.
        self.close_gripper_service = self.create_service(
            Trigger,
            self.close_gripper_service_name,
            self.close_gripper_callback,
        )
        
        # Subscribers
        self.robot_joint_state_sub = self.create_subscription(
            JointState,
            self.robot_joint_state_topic,
            self.robot_joint_state_callback,
            10
        )
        
        self.gello_joint_state_sub = self.create_subscription(
            JointState,
            self.gello_joint_state_topic,
            self.gello_joint_state_callback,
            10
        )
        
        self.get_logger().info(
            f'GelloOffsetNode initialized:\n'
            f'  Robot joint states: {self.robot_joint_state_topic}\n'
            f'  Gello joint states: {self.gello_joint_state_topic}\n'
            f'  Command output: {self.command_topic}\n'
            f'  Gripper min/max: {self.gripper_min}/{self.gripper_max}'
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
        
        if self.paused:
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
        gripper_value = self.gripper_min + (gello_gripper_position * (self.gripper_max - self.gripper_min))

        self._publish_joint_command(
            list(self.robot_joint_names[:-1]) + [self.gripper_joint_name],
            target_arm_positions.tolist() + [gripper_value],
            0,
        )

    def reset_offsets_callback(self, request, response):
        """Reset the offset reference using the most recently observed joint states."""
        del request

        if self.latest_robot_positions is None:
            response.success = False
            response.message = 'No robot joint state received yet; cannot reset offsets.'
            return response

        if self.latest_gello_positions is None:
            response.success = False
            response.message = 'No gello joint state received yet; cannot reset offsets.'
            return response

        self.robot_joint_names = self.latest_robot_joint_names
        self.robot_initial_positions = np.array(self.latest_robot_positions)
        if not self.gripper_joint_name:
            self.gripper_joint_name = self.robot_joint_names[-1]

        self.gello_joint_names = self.latest_gello_joint_names
        self.gello_initial_positions = np.array(self.latest_gello_positions)
        self.initialized = True
        self.paused = False

        self.get_logger().info(
            'Offset reference reset from latest joint states.\n'
            f'  Robot positions: {self.robot_initial_positions}\n'
            f'  Gello positions: {self.gello_initial_positions}'
        )

        response.success = True
        response.message = 'Offsets recomputed from latest joint states.'
        return response

    def pause_publisher_callback(self, request, response):
        """Pause joint command publishing until the reset service is called."""
        del request

        self.paused = True

        self.get_logger().info('Joint command publishing paused. Call reset_offsets to resume.')

        response.success = True
        response.message = 'Joint command publishing paused.'
        return response

    def close_gripper_callback(self, request, response):
        """Publish the latest arm state with the gripper driven to the closed position."""
        del request

        if self.latest_robot_positions is None or self.latest_robot_joint_names is None:
            response.success = False
            response.message = 'No robot joint state received yet; cannot close gripper.'
            return response

        if len(self.latest_robot_positions) < 1:
            response.success = False
            response.message = 'Robot joint state is empty; cannot close gripper.'
            return response

        joint_names = list(self.latest_robot_joint_names)
        gripper_name = self.gripper_joint_name or joint_names[-1]

        arm_positions = list(np.array(self.latest_robot_positions[:-1], copy=True))
        command_positions = arm_positions + [self.gripper_max]  # Set gripper to fully closed
        command_names = joint_names[:-1] + [gripper_name]

        self._publish_joint_command(
            command_names,
            command_positions,
            10
        )

        response.success = True
        response.message = 'Published close-gripper command using latest robot state.'
        return response

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
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
