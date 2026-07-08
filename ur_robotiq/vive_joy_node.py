#!/usr/bin/env python3
"""Translate Vive button presses into gripper commands.

The node subscribes to the /libsurvive/joy topic, which is published by the
libsurvive ROS2 node. It listens for button presses on the Vive controller and
translates them into gripper open/close commands.

The output message type is configurable and may be either:

- std_msgs.msg.Float64MultiArray
- sensor_msgs.msg.JointState
"""

import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np


SUPPORTED_TOPIC_TYPES = (
    Float64MultiArray,
    JointState,
)


def resolve_topic_type(topic_type: str):
    """Resolve a topic type string into a ROS2 message class.

    Supported examples:
        - "Float64MultiArray"
        - "JointState"
        - "std_msgs.msg.Float64MultiArray"
        - "sensor_msgs.msg.JointState"
        - "std_msgs/Float64MultiArray"
        - "sensor_msgs/JointState"
    """

    # Convert ROS-style type names into Python module paths.
    #
    # Example:
    #   std_msgs/Float64MultiArray -> std_msgs.msg.Float64MultiArray
    #   sensor_msgs/JointState     -> sensor_msgs.msg.JointState
    if "/" in topic_type:
        package, msg_name = topic_type.split("/", maxsplit=1)
        topic_type = f"{package}.msg.{msg_name}"

    resolved_type = eval(
        topic_type,
        {
            "__builtins__": {},
            "std_msgs": std_msgs,
            "sensor_msgs": sensor_msgs,
            "Float64MultiArray": Float64MultiArray,
            "JointState": JointState,
        },
        {},
    )

    assert resolved_type in SUPPORTED_TOPIC_TYPES, (
        f"Unsupported topic_type: {topic_type}. "
        f"Supported types are: "
        f"{', '.join(t.__module__ + '.' + t.__name__ for t in SUPPORTED_TOPIC_TYPES)}"
    )

    return resolved_type


class JoyGripperNode(Node):
    def __init__(self):
        super().__init__("joy_gripper_node")

        self.declare_parameter("input_topic", "/libsurvive/joy")
        self.declare_parameter("left_command_topic", "/left_hand_controller/target_state")
        self.declare_parameter("right_command_topic", "/right_hand_controller/target_state")
        self.declare_parameter("left_tracker_id", "")
        self.declare_parameter("right_tracker_id", "")
        self.declare_parameter("gripper_keys", ["gripper_joint"])
        self.declare_parameter("topic_type", "std_msgs/Float64MultiArray")

        self.input_topic = self.get_parameter("input_topic").value
        self.left_command_topic = self.get_parameter("left_command_topic").value
        self.right_command_topic = self.get_parameter("right_command_topic").value
        self.left_tracker_id = self.get_parameter("left_tracker_id").value
        self.right_tracker_id = self.get_parameter("right_tracker_id").value
        self.gripper_keys = self.get_parameter("gripper_keys").value

        topic_type_param = self.get_parameter("topic_type").value
        self.output_msg_type = resolve_topic_type(topic_type_param)

        self.left_gripper_publisher = self.create_publisher(
            self.output_msg_type,
            self.left_command_topic,
            10,
        )

        self.right_gripper_publisher = self.create_publisher(
            self.output_msg_type,
            self.right_command_topic,
            10,
        )

        self.subscription = self.create_subscription(
            Joy,
            self.input_topic,
            self.joy_callback,
            10,
        )

        self._left_gripper_state = False
        self._right_gripper_state = False

        self._previous_left_button_pressed = False
        self._previous_right_button_pressed = False

        self._lock = threading.Lock()

        self.get_logger().info(
            f"Publishing gripper commands as {self.output_msg_type.__module__}."
            f"{self.output_msg_type.__name__}"
        )

    def joy_callback(self, msg: Joy):
        with self._lock:
            button_pressed = bool(np.any(msg.buttons))

            if msg.header.frame_id == self.left_tracker_id:
                rising_edge = button_pressed and not self._previous_left_button_pressed
                self._previous_left_button_pressed = button_pressed

                if rising_edge:
                    self._left_gripper_state = not self._left_gripper_state
                    self.publish_gripper_command(
                        self.left_gripper_publisher,
                        self._left_gripper_state,
                    )

            elif msg.header.frame_id == self.right_tracker_id:
                rising_edge = button_pressed and not self._previous_right_button_pressed
                self._previous_right_button_pressed = button_pressed

                if rising_edge:
                    self._right_gripper_state = not self._right_gripper_state
                    self.publish_gripper_command(
                        self.right_gripper_publisher,
                        self._right_gripper_state,
                    )

    def publish_gripper_command(self, publisher, state: bool):
        value = float(state)

        if self.output_msg_type is JointState:
            msg = JointState()
            msg.name = list(self.gripper_keys)
            msg.position = [value] * len(self.gripper_keys)

        elif self.output_msg_type is Float64MultiArray:
            msg = Float64MultiArray()
            msg.data = [value] * len(self.gripper_keys)

        else:
            raise TypeError(f"Unsupported output message type: {self.output_msg_type}")

        publisher.publish(msg)


def main():
    rclpy.init()
    node = JoyGripperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()