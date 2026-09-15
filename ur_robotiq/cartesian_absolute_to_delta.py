#!/usr/bin/env python3
"""Convert absolute Cartesian PoseStamped targets into world-frame deltas.

For each absolute target, the node computes:

    translation_delta = target_position - current_position
    rotation_delta = target_rotation @ inverse(current_rotation)

Both deltas are expressed in ``target_frame``. The controlled frame is used only
to look up the robot's current pose; it is not used as the output coordinate
frame.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import (
    concatenate_matrices,
    identity_matrix,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
    unit_vector,
)


def pose_matrix(position, orientation):
    quaternion = unit_vector((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ))
    return concatenate_matrices(
        translation_matrix((position.x, position.y, position.z)),
        quaternion_matrix(quaternion),
    )


def transform_matrix(transform):
    return pose_matrix(transform.translation, transform.rotation)


class CartesianAbsoluteToDelta(Node):
    def __init__(self):
        super().__init__('cartesian_absolute_to_delta')

        self.declare_parameter('input_topic', '/cartesian_target_absolute')
        self.declare_parameter('output_topic', '/cartesian_target_delta')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('controlled_frame', 'right_dorsum_link')
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('use_message_stamp', True)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.controlled_frame = str(self.get_parameter('controlled_frame').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.use_message_stamp = bool(
            self.get_parameter('use_message_stamp').value
        )

        if not self.target_frame:
            raise ValueError('target_frame must not be empty')
        if not self.controlled_frame:
            raise ValueError('controlled_frame must not be empty')
        if self.tf_timeout < 0.0:
            raise ValueError('tf_timeout must be non-negative')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            input_topic,
            self.target_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Converting absolute targets on {input_topic} to world-frame '
            f'deltas on {output_topic}; output_frame={self.target_frame}, '
            f'controlled={self.controlled_frame}'
        )

    def lookup(self, target_frame, source_frame, stamp):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            stamp,
            timeout=Duration(seconds=self.tf_timeout),
        )

    def target_callback(self, msg):
        source_frame = msg.header.frame_id
        if not source_frame:
            self.get_logger().warning('Ignoring target with an empty frame_id')
            return

        stamp = (
            Time.from_msg(msg.header.stamp)
            if self.use_message_stamp
            else Time()
        )

        try:
            # Current controlled-frame pose expressed in target_frame/world.
            current_tf = self.lookup(
                self.target_frame,
                self.controlled_frame,
                stamp,
            )
            current_pose = transform_matrix(current_tf.transform)

            # Convert the commanded target pose into target_frame/world.
            target_pose = pose_matrix(msg.pose.position, msg.pose.orientation)
            if source_frame != self.target_frame:
                source_tf = self.lookup(
                    self.target_frame,
                    source_frame,
                    stamp,
                )
                target_pose = concatenate_matrices(
                    transform_matrix(source_tf.transform),
                    target_pose,
                )

            # World-frame translation delta. Do not rotate it into the
            # controlled frame.
            delta_translation = (
                target_pose[0:3, 3] - current_pose[0:3, 3]
            )

            # World/spatial rotational delta: R_delta * R_current = R_target.
            # Translation is handled separately to avoid SE(3) translation
            # coupling from target_pose @ inverse(current_pose).
            delta_rotation = concatenate_matrices(
                target_pose,
                inverse_matrix(current_pose),
            )[0:3, 0:3]

            delta_pose = identity_matrix()
            delta_pose[0:3, 0:3] = delta_rotation
            delta_pose[0:3, 3] = delta_translation

        except (TransformException, ValueError) as exc:
            self.get_logger().warning(
                f'Cannot convert Cartesian target: {exc}'
            )
            return

        quaternion = unit_vector(quaternion_from_matrix(delta_pose))

        output = PoseStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.target_frame
        output.pose.position.x = float(delta_translation[0])
        output.pose.position.y = float(delta_translation[1])
        output.pose.position.z = float(delta_translation[2])
        output.pose.orientation.x = float(quaternion[0])
        output.pose.orientation.y = float(quaternion[1])
        output.pose.orientation.z = float(quaternion[2])
        output.pose.orientation.w = float(quaternion[3])
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianAbsoluteToDelta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()