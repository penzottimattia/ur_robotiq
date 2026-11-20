#!/usr/bin/env python3
"""Motion test node: smooth sinusoidal arm + hand motion around the start pose.

Looks up the current left/right tool0 pose (base_link -> tool0) once at
startup, then publishes:

- /target_pose (geometry_msgs/TransformStamped, per side): the start pose plus
  smooth multi-sine position deltas (incommensurate frequencies -> organic
  drift). The orientation is held constant at the start quaternion.
- /target_joint_states (sensor_msgs/JointState): rectified-sine grip targets
  for the MIA hand joints (index/mrl/thumb_fle per side), centered on the
  hand positions captured at startup.

An amplitude envelope (smoothstep, 0 -> 1 over 3 s) guarantees the delta is
exactly zero at t=0, so neither arms nor hands jump at startup. At the end,
the captured start pose is re-asserted for a few seconds so everything
returns home before the node exits.

Usage:
    ros2 run ur_robotiq motion_test
    ros2 run ur_robotiq motion_test --duration 60 --amplitude-scale 0.5
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

HANDS = ["left_hand_j_index_fle", "left_hand_j_mrl_fle", "left_hand_j_thumb_fle",
         "right_hand_j_index_fle", "right_hand_j_mrl_fle", "right_hand_j_thumb_fle"]

ENVELOPE_S = 3.0       # ramp-up time for the amplitude envelope
RETURN_S = 2.0         # start-pose re-assertion time before exit


class MotionTest(Node):
    def __init__(self, amplitude_scale: float):
        super().__init__("motion_test")
        self.amplitude_scale = amplitude_scale
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.pub = self.create_publisher(TransformStamped, "/target_pose", 10)
        self.hpub = self.create_publisher(JointState, "/target_joint_states", 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.hand_base = None      # one-time snapshot of hand positions
        self.start = {}            # side -> ((x,y,z), (qx,qy,qz,qw)) in base_link
        self.t = 0.0
        self.started = False
        self.create_timer(0.05, self.tick)

    def _js_cb(self, msg):
        if self.started:  # base is a one-time snapshot; don't follow live state
            return
        try:
            self.hand_base = [msg.position[msg.name.index(n)] for n in HANDS]
        except (ValueError, IndexError):
            pass

    def _lookup(self, side):
        try:
            tf = self.buf.lookup_transform(f"{side}_base_link", f"{side}_tool0", rclpy.time.Time())
        except Exception:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def tick(self):
        if not self.started:
            for side in ("left", "right"):
                if side not in self.start and self._lookup(side):
                    self.start[side] = self._lookup(side)
                    (x, y, z), q = self.start[side]
                    self.get_logger().info(
                        f"{side} start: pos=({x:.3f},{y:.3f},{z:.3f}) quat=({q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f})")
            if len(self.start) < 2 or self.hand_base is None:
                return
            self.get_logger().info(f"hand start: {['%.3f' % v for v in self.hand_base]}")
            self.started = True
            return

        # envelope 0->1 over ENVELOPE_S (smoothstep): delta is exactly 0 at start
        s = min(1.0, self.t / ENVELOPE_S)
        env = self.amplitude_scale * s * s * (3.0 - 2.0 * s)

        # --- arms: smooth multi-sine deltas around the start pose ---
        for i, side in enumerate(("left", "right")):
            (x0, y0, z0), q0 = self.start[side]
            ph = i * 0.7  # side phase offset
            dx = env * (0.06 * math.sin(0.50 * self.t + ph) + 0.03 * math.sin(0.23 * self.t + 2 * ph))
            dy = env * (0.05 * math.sin(0.31 * self.t + 2 * ph) + 0.02 * math.sin(0.17 * self.t + ph))
            dz = env * (0.04 * math.sin(0.41 * self.t + 3 * ph) + 0.02 * math.sin(0.19 * self.t + 2 * ph))
            m = TransformStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = f"{side}_base_link"
            m.child_frame_id = f"{side}_tool0"
            m.transform.translation.x = x0 + dx
            m.transform.translation.y = y0 + dy
            m.transform.translation.z = z0 + dz
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q0
            self.pub.publish(m)

        # --- hands: rectified-sine grip centered on the start positions ---
        # 0.5*(1 - sin) is smooth and stays in [0, 1] -> the target stays ABOVE
        # the start position; the envelope makes the delta exactly 0 at t=0.
        js = JointState()
        js.name = HANDS
        amps = [0.60, 0.60, 0.40]   # index, mrl, thumb_fle
        freqs = [0.80, 0.55, 0.45]
        lags = [0.0, 0.7, 1.4]
        for i in range(2):          # 0=left, 1=right
            ph = i * 0.9
            for f in range(3):
                base = self.hand_base[3 * i + f]
                js.position.append(base + env * amps[f] * 0.5 * (1.0 - math.sin(freqs[f] * self.t + ph + lags[f])))
        self.hpub.publish(js)
        self.t += 0.05

    def return_home(self, iterations: int):
        """Re-assert the captured start pose (arms + hands) before exit."""
        for _ in range(iterations):
            for side in ("left", "right"):
                if side not in self.start:
                    continue
                (x, y, z), q = self.start[side]
                m = TransformStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = f"{side}_base_link"
                m.child_frame_id = f"{side}_tool0"
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = x, y, z
                m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
                self.pub.publish(m)
            if self.hand_base is not None:
                js = JointState()
                js.name = HANDS
                js.position = list(self.hand_base)
                self.hpub.publish(js)
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="Smooth arm + hand motion test around the start pose.")
    parser.add_argument("--duration", type=float, default=90.0, help="motion duration in seconds (default: 90)")
    parser.add_argument("--amplitude-scale", type=float, default=1.0,
                        help="scales all deltas (0 = hold pose, default: 1.0)")
    args = parser.parse_args()

    rclpy.init()
    node = MotionTest(args.amplitude_scale)
    t0 = time.time()
    print(f"motion test: {args.duration:.0f} s, smooth deltas around start pose", flush=True)
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.return_home(int(RETURN_S / 0.1))
    print("returned to start pose", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
