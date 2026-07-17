#!/usr/bin/env python3
"""ROS2 node that publishes a visualization Marker of type MESH_RESOURCE.

Requires an absolute mesh file path as a positional argument — no default.
The mesh is referenced via a `file://` URI constructed from the absolute
path and its pose is read from a `PoseStamped` topic (default `/mesh_pose`).
By default the marker will use the incoming message's header.frame_id;
pass `--frame` to override that behavior.
"""
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from typing import Optional


class MeshMarkerNode(Node):
    def __init__(self):
        super().__init__('spawn_mesh_marker')

        # Declare parameters for ros2 run --ros-args -p <name>:=<value>
        self.declare_parameter('name', '')
        self.declare_parameter('mesh', '')
        self.declare_parameter('pose_topic', '/mesh_pose')
        self.declare_parameter('frame', '')
        self.declare_parameter('scale', 1.0)

        mesh_path = self.get_parameter('mesh').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        frame_override = self.get_parameter('frame').get_parameter_value().string_value
        scale = float(self.get_parameter('scale').get_parameter_value().double_value)
        self.name = self.get_parameter('name').get_parameter_value().string_value

        if not mesh_path:
            self.get_logger().error('Parameter "mesh" is required and must be an absolute path to the mesh file')
            raise SystemExit('mesh parameter required')

        if not os.path.isabs(mesh_path):
            self.get_logger().error('mesh path must be absolute (provide an absolute filesystem path)')
            raise SystemExit('mesh path must be absolute')

        if not os.path.exists(mesh_path):
            self.get_logger().error(f'mesh not found: {mesh_path}')
            raise SystemExit(f'mesh not found: {mesh_path}')

        self.mesh_resource = f'file://{os.path.abspath(mesh_path)}'
        self.frame_override = frame_override if frame_override else None
        self.scale = scale
        self.latest_pose: Optional[PoseStamped] = None
        self._logged_missing = False
        
        pub_name = f'mesh_marker_{self.name}' if self.name else f'mesh_marker_{os.path.basename(mesh_path)}'.removesuffix('.obj')

        self.pub = self.create_publisher(Marker, f'{pub_name}', 10)
        self.sub = self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_marker)
        self.get_logger().info(f'Waiting for PoseStamped on "{pose_topic}" to publish mesh {self.mesh_resource}')

    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg
        if not self._logged_missing:
            self.get_logger().info(f'Received pose in frame "{msg.header.frame_id}"')
            self._logged_missing = True

    def publish_marker(self):
        if self.latest_pose is None:
            return

        m = Marker()
        # choose frame: override if provided, otherwise use pose header
        frame_id = self.frame_override if self.frame_override else self.latest_pose.header.frame_id
        m.header.frame_id = frame_id
        m.header.stamp = self.latest_pose.header.stamp
        m.ns = 'mesh_marker'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = self.mesh_resource
        m.action = Marker.ADD
        m.pose = self.latest_pose.pose
        m.scale.x = float(self.scale)
        m.scale.y = float(self.scale)
        m.scale.z = float(self.scale)

        # randomize color a bit for better visibility if multiple markers are present
        import random
        rng = random.Random(self.name)  # deterministic color per frame
        m.color.r = rng.uniform(0.2, 1.0)
        m.color.g = rng.uniform(0.2, 1.0)
        m.color.b = rng.uniform(0.2, 1.0)
        m.color.a = 1.0

        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MeshMarkerNode()
    except SystemExit:
        rclpy.shutdown()
        raise

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
