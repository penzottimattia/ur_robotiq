#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateToTrajectoryNode(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('trajectory_topic', '/joint_trajectory')
        self.declare_parameter('gripper_topic', '/gripper_command')
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        self.gripper_joint = self.get_parameter('gripper_joint').get_parameter_value().string_value

        self.publisher = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.gripper_publisher = self.create_publisher(Float64MultiArray, self.gripper_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [msg.position.pop(msg.name.index(self.gripper_joint))]
        self.gripper_publisher.publish(gripper_msg)

        msg.name.remove(self.gripper_joint)

        traj = JointTrajectory()
        traj.joint_names = [self.tf_prefix + name for name in msg.name]

        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start = rclpy.duration.Duration(seconds=msg.header.stamp.sec).to_msg()

        traj.points = [point]
        self.publisher.publish(traj)

        time.sleep(msg.header.stamp.sec)


def main():
    rclpy.init()
    node = JointStateToTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
