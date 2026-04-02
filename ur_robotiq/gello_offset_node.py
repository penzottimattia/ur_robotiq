#!/usr/bin/env python3
"""Gello offset computation and joint trajectory command node

Subscribes to:
- Robot joint states (once, to get initial position)
- Gello joint states (continuously)

Computes the offset between gello and robot initial positions, then
continuously publishes adjusted joint trajectory commands to maintain
the offset relationship.

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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GelloOffsetNode(Node):
    def __init__(self):
        super().__init__('gello_offset_node')
        
        # Declare parameters
        self.declare_parameter('robot_joint_state_topic', '/joint_states')
        self.declare_parameter('gello_joint_state_topic', '/gello/joint_states')
        self.declare_parameter('trajectory_topic', '/joint_trajectory_controller/commands')
        self.declare_parameter('gripper_topic', '/gripper_trajectory')
        self.declare_parameter('gripper_min', 0.0)
        self.declare_parameter('gripper_max', 0.8)
        
        # Get parameter values
        self.robot_joint_state_topic = self.get_parameter('robot_joint_state_topic').get_parameter_value().string_value
        self.gello_joint_state_topic = self.get_parameter('gello_joint_state_topic').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        self.gripper_min = self.get_parameter('gripper_min').get_parameter_value().double_value
        self.gripper_max = self.get_parameter('gripper_max').get_parameter_value().double_value
        
        # State variables
        self.robot_initial_positions = None
        self.robot_joint_names = None
        self.gripper_joint_name = None  # Will be deduced from robot_joint_names
        self.gello_initial_positions = None
        self.gello_joint_names = None
        self.gello_offset = None
        self.initialized = False
        
        # Publisher for trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic,
            10
        )
        
        # Publisher for gripper commands
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            self.gripper_topic,
            10
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
            f'  Trajectory output: {self.trajectory_topic}\n'
            f'  Gripper output: {self.gripper_topic}\n'
            f'  Gripper min/max: {self.gripper_min}/{self.gripper_max}'
        )
    
    def robot_joint_state_callback(self, msg):
        """Capture initial robot joint state (only once)"""
        if self.robot_initial_positions is not None:
            return  # Already captured
        
        self.robot_joint_names = msg.name
        self.robot_initial_positions = np.array(msg.position)
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
        """Capture initial gello state and use for subsequent offset computation"""
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
        
        # Create and publish arm trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.robot_joint_names[:-1]  # All robot joints except gripper
        
        point = JointTrajectoryPoint()
        point.positions = target_arm_positions.tolist()
        point.time_from_start.sec = 0
        
        trajectory.points = [point]
        
        self.trajectory_publisher.publish(trajectory)
        
        # Handle gripper: remap from 0-1 to gripper_min/gripper_max
        gripper_value = self.gripper_min + (gello_gripper_position * (self.gripper_max - self.gripper_min))
        
        gripper_trajectory = JointTrajectory()
        gripper_trajectory.joint_names = [self.gripper_joint_name]
        
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [gripper_value]
        gripper_point.time_from_start.sec = 0
        
        gripper_trajectory.points = [gripper_point]
        
        self.gripper_publisher.publish(gripper_trajectory)
    
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
