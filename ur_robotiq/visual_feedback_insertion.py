#!/usr/bin/env python3
"""Visual-feedback insertion controller with shortest-path Z-axis alignment.

The manipulated frame's Z axis is aligned to the reference frame's Z axis by
applying the shortest spatial rotation to the controlled frame, treating
opposite Z directions as already axis-aligned. Translation is computed from
the manipulated frame pose expressed in the reference frame.
Operation is disabled until the start or execute service is called.
"""

import threading
import time
from enum import Enum, auto

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import (
    euler_matrix,
    quaternion_about_axis,
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_multiply,
)


class Phase(Enum):
    ALIGN = auto()
    INSERT = auto()
    COMPLETE = auto()


class VisualFeedbackInsertion(Node):
    """Closed-loop, bounded insertion controller with Z-axis alignment."""

    def __init__(self):
        super().__init__('visual_feedback_insertion')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('reference_frame', 'reference_object')
        self.declare_parameter('manipulated_frame', 'manipulated_object')
        self.declare_parameter('controlled_frame', 'right_dorsum_link')
        self.declare_parameter('command_topic', '/right_cartesian_controller/target_frame')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('lateral_tolerance', 0.002)
        self.declare_parameter('insertion_depth', 0.030)
        self.declare_parameter('depth_tolerance', 0.005)
        self.declare_parameter('max_step', 0.005)
        self.declare_parameter('align_gain', 0.35)
        self.declare_parameter('insert_gain', 0.25)
        self.declare_parameter('max_initial_lateral_error', 0.10)
        self.declare_parameter('transform_timeout', 0.1)
        self.declare_parameter('tf_max_age', 1.0)
        self.declare_parameter('orientation_jitter', 0.0)
        self.declare_parameter('rotation_alignment_once', True)
        self.declare_parameter('rotation_tolerance', 0.01)
        self.declare_parameter('rotation_gain', 0.1)
        self.declare_parameter('max_rotation_step', 0.10)
        self.declare_parameter('dry_run', True)

        self.world = str(self.get_parameter('world_frame').value)
        self.reference = str(self.get_parameter('reference_frame').value)
        self.manipulated = str(self.get_parameter('manipulated_frame').value)
        self.controlled = str(self.get_parameter('controlled_frame').value)
        topic = str(self.get_parameter('command_topic').value)
        rate = max(1.0, float(self.get_parameter('rate_hz').value))

        self.callback_group = ReentrantCallbackGroup()
        self._state_condition = threading.Condition()
        self._execute_active = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, topic, 1)
        self.start_service = self.create_service(
            Trigger, '~/start', self._start, callback_group=self.callback_group
        )
        self.execute_service = self.create_service(
            Trigger, '~/execute', self._execute, callback_group=self.callback_group
        )
        self.stop_service = self.create_service(
            Trigger, '~/stop', self._stop, callback_group=self.callback_group
        )
        self.timer = self.create_timer(
            1.0 / rate, self._update, callback_group=self.callback_group
        )

        self.enabled = False
        self.phase = Phase.ALIGN
        self.rotation_aligned = False
        self.last_status = ''
        self.get_logger().warning(
            'Insertion controller is DISABLED and dry_run=%s. Verify frames, axis, '
            'clearances, collision limits, and emergency stop before enabling.'
            % self.get_parameter('dry_run').value
        )

    def _start(self, _request, response):
        try:
            self._lookup(self.world, self.reference)
            self._lookup(self.world, self.manipulated)
            self._lookup(self.world, self.controlled)
        except TransformException as exc:
            response.success = False
            response.message = 'Start failed: required TF unavailable: %s' % exc
            return response

        self.phase = Phase.ALIGN
        self.rotation_aligned = False
        self.enabled = True
        response.success = True
        response.message = 'Visual-feedback insertion started.'
        return response

    def _execute(self, request, response):
        with self._state_condition:
            self._execute_active = True
        try:
            start_response = self._start(request, Trigger.Response())
            if not start_response.success:
                response.success = False
                response.message = start_response.message
                return response
            while self.enabled and self.phase != Phase.COMPLETE:
                time.sleep(0.01)
        finally:
            with self._state_condition:
                self._execute_active = False
                self._state_condition.notify_all()

        response.success = self.phase == Phase.COMPLETE
        response.message = (
            'Visual-feedback insertion completed.' if response.success
            else 'Visual-feedback insertion aborted before completion.'
        )
        return response

    def _stop(self, _request, response):
        self.enabled = False
        with self._state_condition:
            while self._execute_active:
                self._state_condition.wait(timeout=0.1)
        response.success = True
        response.message = 'Insertion stopped; no further targets will be published.'
        return response

    def _lookup(self, target, source):
        timeout = Duration(seconds=float(self.get_parameter('transform_timeout').value))
        return self.tf_buffer.lookup_transform(target, source, Time(), timeout)

    def _transform_is_fresh(self, transform):
        stamp = transform.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True

        age = self.get_clock().now().nanoseconds - (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        )
        return age <= int(float(self.get_parameter('tf_max_age').value) * 1_000_000_000)

    @staticmethod
    def _quaternion_array(q):
        return np.asarray((q.x, q.y, q.z, q.w), dtype=float)

    @staticmethod
    def _quaternion_message(q):
        q = np.asarray(q, dtype=float)
        q /= np.linalg.norm(q)
        return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

    @staticmethod
    def _shortest_z_alignment(source_q, target_q):
        """Return (axis, angle) rotating source Z onto the closest target axis."""
        source_z = quaternion_matrix(source_q)[:3, 2]
        target_z = quaternion_matrix(target_q)[:3, 2]
        dot = float(np.dot(source_z, target_z))
        if dot < 0.0:
            target_z = -target_z
            dot = -dot
        dot = float(np.clip(dot, -1.0, 1.0))
        angle = float(np.arccos(dot))
        axis = np.cross(source_z, target_z)
        axis_norm = float(np.linalg.norm(axis))

        if axis_norm > np.finfo(float).eps:
            axis /= axis_norm
        else:
            axis = np.array((1.0, 0.0, 0.0), dtype=float)
            angle = 0.0
        return axis, angle

    @staticmethod
    def _limit_norm(vector, limit):
        vector = np.asarray(vector, dtype=float)
        norm = float(np.linalg.norm(vector))
        if norm > limit > 0.0:
            vector *= limit / norm
        return vector

    def _jitter_quaternion(self):
        jitter = float(self.get_parameter('orientation_jitter').value)
        if jitter <= 0.0:
            return np.array((0.0, 0.0, 0.0, 1.0), dtype=float)
        roll, pitch = np.random.normal(0.0, jitter, size=2)
        return quaternion_from_matrix(euler_matrix(float(roll), float(pitch), 0.0))

    def _status(self, text):
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text

    def _update(self):
        if not self.enabled:
            return
        try:
            ref_to_obj = self._lookup(self.reference, self.manipulated)
            world_to_ref = self._lookup(self.world, self.reference)
            world_to_obj = self._lookup(self.world, self.manipulated)
            world_to_control = self._lookup(self.world, self.controlled)
        except TransformException as exc:
            self.get_logger().warning('TF unavailable; command suppressed: %s' % exc)
            return

        if not all(
            self._transform_is_fresh(transform)
            for transform in (ref_to_obj, world_to_ref, world_to_obj, world_to_control)
        ):
            self.get_logger().warning('TF stale; command suppressed.')
            return

        p = np.array((
            ref_to_obj.transform.translation.x,
            ref_to_obj.transform.translation.y,
            ref_to_obj.transform.translation.z,
        ), dtype=float)
        lateral = float(np.linalg.norm(p[:2]))
        max_initial = float(self.get_parameter('max_initial_lateral_error').value)
        if lateral > max_initial:
            self.enabled = False
            self.get_logger().error(
                'Aborted: lateral error %.4f m exceeds %.4f m.' % (lateral, max_initial)
            )
            return

        q_ref = self._quaternion_array(world_to_ref.transform.rotation)
        q_obj = self._quaternion_array(world_to_obj.transform.rotation)
        q_control = self._quaternion_array(world_to_control.transform.rotation)
        rotation_axis, rotation_error = self._shortest_z_alignment(q_obj, q_ref)

        rotation_once = bool(self.get_parameter('rotation_alignment_once').value)
        rotation_tol = float(self.get_parameter('rotation_tolerance').value)
        if rotation_once and not self.rotation_aligned and rotation_error <= rotation_tol:
            self.rotation_aligned = True
            self._status('Z-axis rotation aligned; switching to translation-only feedback.')

        apply_rotation = not (rotation_once and self.rotation_aligned)
        if apply_rotation:
            rotation_step = min(
                rotation_error * float(self.get_parameter('rotation_gain').value),
                float(self.get_parameter('max_rotation_step').value),
            )
            q_delta = quaternion_about_axis(rotation_step, rotation_axis)
            q_target = quaternion_multiply(q_delta, q_control)
        else:
            q_target = q_control

        # Jitter is deliberately separate from the measured alignment rotation.
        q_target = quaternion_multiply(q_target, self._jitter_quaternion())

        # In one-shot mode, finish rotational alignment before translating. In
        # continuous mode, translation and rotational feedback run together.
        translation_enabled = (not rotation_once) or self.rotation_aligned
        correction_ref = np.zeros(3, dtype=float)
        lateral_tol = float(self.get_parameter('lateral_tolerance').value)
        depth = float(self.get_parameter('insertion_depth').value)
        depth_tol = float(self.get_parameter('depth_tolerance').value)

        if translation_enabled:
            if self.phase == Phase.ALIGN and lateral <= lateral_tol:
                self.phase = Phase.INSERT
                self._status('Lateral alignment reached; beginning insertion along -Z reference axis.')

            if self.phase == Phase.ALIGN:
                correction_ref[:2] = -float(self.get_parameter('align_gain').value) * p[:2]
            elif self.phase == Phase.INSERT:
                z_error = -depth - p[2]
                if lateral > lateral_tol:
                    self.phase = Phase.ALIGN
                    self._status('Lateral error left tolerance; returning to alignment phase.')
                    return
                if abs(z_error) <= depth_tol:
                    self.phase = Phase.COMPLETE
                    self.enabled = False
                    self._status('Insertion depth reached; controller stopped.')
                    return
                correction_ref[2] = float(self.get_parameter('insert_gain').value) * z_error
            else:
                return

        correction_ref = self._limit_norm(
            correction_ref, float(self.get_parameter('max_step').value)
        )
        correction_world = quaternion_matrix(q_ref)[:3, :3] @ correction_ref
        current = world_to_control.transform

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.world
        target.pose.position.x = current.translation.x + float(correction_world[0])
        target.pose.position.y = current.translation.y + float(correction_world[1])
        target.pose.position.z = current.translation.z + float(correction_world[2])
        target.pose.orientation = self._quaternion_message(q_target)

        self._status(
            '%s: lateral=%.4f m, object_z=%.4f m, rotation_error=%.4f rad, '
            'step=(%.4f, %.4f, %.4f) m'
            % (self.phase.name, lateral, p[2], rotation_error, *correction_ref)
        )
        if not bool(self.get_parameter('dry_run').value):
            self.publisher.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = VisualFeedbackInsertion()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()