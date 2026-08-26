#!/usr/bin/env python3
"""Translation-only visual-feedback insertion controller.

The node reads two TF frames:
* reference_frame: insertion fixture / hole frame (its +Z is the insertion axis)
* manipulated_frame: frame attached to the object being inserted

It commands translation only. The controlled frame orientation is copied unchanged.
Operation is deliberately disabled until the ``start`` service is called.
"""

import math
import random
import time
from enum import Enum, auto
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply


class Phase(Enum):
    ALIGN = auto()
    INSERT = auto()
    COMPLETE = auto()


class VisualFeedbackInsertion(Node):
    """Closed-loop, bounded, translation-only insertion controller."""

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
        self.declare_parameter('orientation_jitter', 0.0)
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
        self.target_orientation = None
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
        self.last_status = ''
        self.get_logger().warning(
            'Insertion controller is DISABLED and dry_run=%s. Verify frames, axis, '
            'clearances, collision limits, and emergency stop before enabling.'
            % self.get_parameter('dry_run').value
        )

    def _start(self, _request, response):
        try:
            world_to_control = self._lookup(self.world, self.controlled)
            target_orientation = world_to_control.transform.rotation
            jitter_std = float(self.get_parameter('orientation_jitter').value)
            if jitter_std > 0.0:
                jitter = quaternion_from_euler(
                    random.gauss(0.0, jitter_std),
                    random.gauss(0.0, jitter_std),
                    random.gauss(0.0, jitter_std),
                )
                target_orientation = quaternion_multiply(
                    (
                        target_orientation.x,
                        target_orientation.y,
                        target_orientation.z,
                        target_orientation.w,
                    ),
                    jitter,
                )
                target_orientation = Quaternion(
                    x=target_orientation[0],
                    y=target_orientation[1],
                    z=target_orientation[2],
                    w=target_orientation[3],
                )
        except TransformException as exc:
            self.get_logger().warning(
                'Start orientation unavailable; start rejected: %s' % exc
            )
            response.success = False
            response.message = 'Visual-feedback insertion start failed: target orientation unavailable.'
            return response

        self.target_orientation = target_orientation
        self.phase = Phase.ALIGN
        self.enabled = True
        response.success = True
        response.message = 'Visual-feedback insertion started.'
        return response

    def _execute(self, _request, response):
        with self._state_condition:
            self._execute_active = True

        try:
            start_response = Trigger.Response()
            start_response = self._start(_request, start_response)
            if not start_response.success:
                response.success = False
                response.message = start_response.message
                return response

            while self.enabled:
                if self.phase == Phase.COMPLETE:
                    break
                time.sleep(0.01)
        finally:
            with self._state_condition:
                self._execute_active = False
                self._state_condition.notify_all()

        if self.phase == Phase.COMPLETE:
            response.success = True
            response.message = 'Visual-feedback insertion completed.'
        else:
            response.success = False
            response.message = 'Visual-feedback insertion aborted before completion.'
        return response

    def _stop(self, _request, response):
        self.enabled = False
        with self._state_condition:
            while self._execute_active:
                self._state_condition.wait(timeout=0.1)
        self.target_orientation = None
        response.success = True
        response.message = 'Insertion stopped; no further targets will be published.'
        return response

    def _lookup(self, target, source):
        timeout = Duration(seconds=float(self.get_parameter('transform_timeout').value))
        return self.tf_buffer.lookup_transform(target, source, Time(), timeout)

    @staticmethod
    def _rotate(q, v):
        # Quaternion-vector rotation, q=(x,y,z,w), without external dependencies.
        x, y, z, w = q.x, q.y, q.z, q.w
        vx, vy, vz = v
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        )

    @staticmethod
    def _limit_norm(v, limit):
        n = math.sqrt(sum(x * x for x in v))
        if n <= limit or n == 0.0:
            return v
        scale = limit / n
        return tuple(x * scale for x in v)

    def _status(self, text):
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text

    def _update(self):
        if not self.enabled:
            return
        try:
            # manipulated expressed in reference gives the visual insertion error directly.
            ref_to_obj = self._lookup(self.reference, self.manipulated)
            world_to_ref = self._lookup(self.world, self.reference)
            world_to_control = self._lookup(self.world, self.controlled)
        except TransformException as exc:
            self.get_logger().warning('TF unavailable; command suppressed: %s' % exc)
            return

        p = ref_to_obj.transform.translation
        lateral = math.hypot(p.x, p.y)
        max_initial = float(self.get_parameter('max_initial_lateral_error').value)
        if lateral > max_initial:
            self.enabled = False
            self.get_logger().error(
                'Aborted: lateral error %.4f m exceeds %.4f m.' % (lateral, max_initial)
            )
            return

        lateral_tol = float(self.get_parameter('lateral_tolerance').value)
        depth = float(self.get_parameter('insertion_depth').value)
        depth_tol = float(self.get_parameter('depth_tolerance').value)

        if self.phase == Phase.ALIGN and lateral <= lateral_tol:
            self.phase = Phase.INSERT
            self._status('Lateral alignment reached; beginning insertion along -Z reference axis.')

        if self.phase == Phase.ALIGN:
            gain = float(self.get_parameter('align_gain').value)
            correction_ref = (-gain * p.x, -gain * p.y, 0.0)
        elif self.phase == Phase.INSERT:
            # Target manipulated-frame coordinate in reference is (0, 0, -depth).
            z_error = -depth - p.z
            if lateral > lateral_tol:
                self.phase = Phase.ALIGN
                self._status('Lateral error left tolerance; returning to alignment phase.')
                return
            if abs(z_error) <= depth_tol:
                self.phase = Phase.COMPLETE
                self.enabled = False
                self._status('Insertion depth reached; controller stopped.')
                return
            gain = float(self.get_parameter('insert_gain').value)
            correction_ref = (0.0, 0.0, gain * z_error)
        else:
            return

        correction_ref = self._limit_norm(
            correction_ref, float(self.get_parameter('max_step').value)
        )
        correction_world = self._rotate(world_to_ref.transform.rotation, correction_ref)

        current = world_to_control.transform
        if self.target_orientation is None:
            return
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.world
        target.pose.position.x = current.translation.x + correction_world[0]
        target.pose.position.y = current.translation.y + correction_world[1]
        target.pose.position.z = current.translation.z + correction_world[2]
        # Translation only: preserve the fixed target orientation captured at start.
        target.pose.orientation = self.target_orientation

        self._status(
            '%s: lateral=%.4f m, object_z=%.4f m, step=(%.4f, %.4f, %.4f) m'
            % (self.phase.name, lateral, p.z, *correction_ref)
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
