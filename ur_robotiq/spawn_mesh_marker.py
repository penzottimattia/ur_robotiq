#!/usr/bin/env python3
"""ROS2 node that publishes a visualization Marker of type MESH_RESOURCE.

Requires an absolute mesh file path as a positional argument — no default.
The mesh is referenced via a `file://` URI constructed from the absolute
path and its pose is read from a `PoseStamped` topic (default `/mesh_pose`).
By default the marker will use the incoming message's header.frame_id;
pass `--frame` to override that behavior.
"""
import os
import argparse
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from typing import Optional


class MeshMarkerNode(Node):
    def __init__(self, mesh_resource: str, pose_topic: str = '/mesh_pose', frame_override: Optional[str] = None, scale: float = 1.0):
        super().__init__('spawn_mesh_marker')
        self.mesh_resource = mesh_resource
        self.frame_override = frame_override
        self.scale = scale
        self.latest_pose: Optional[PoseStamped] = None
        self._logged_missing = False

        import time
        self.pub = self.create_publisher(Marker, f'visualization_marker_{int(time.time())}', 10)
        self.sub = self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_marker)
        self.get_logger().info(f'Waiting for PoseStamped on "{pose_topic}" to publish mesh {mesh_resource}')

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
        m.color.r = 0.8
        m.color.g = 0.8
        m.color.b = 0.8
        m.color.a = 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        self.pub.publish(m)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Spawn a mesh as a visualization Marker using a PoseStamped topic')
    parser.add_argument('mesh', help='Absolute path to the mesh file (e.g. /abs/path/triangle.obj). REQUIRED')
    parser.add_argument('--pose-topic', default='/mesh_pose', help='PoseStamped topic to subscribe to')
    parser.add_argument('--frame', default=None, help='Optional frame override for the published marker')
    parser.add_argument('--scale', type=float, default=1.0, help='Uniform scale for the mesh')
    args = parser.parse_args(argv)
    mesh_path = args.mesh
    if not os.path.isabs(mesh_path):
        raise SystemExit('mesh path must be absolute (provide an absolute filesystem path)')
    if not os.path.exists(mesh_path):
        raise SystemExit(f'mesh not found: {mesh_path}')

    mesh_resource = f'file://{os.path.abspath(mesh_path)}'

    rclpy.init()
    node = MeshMarkerNode(mesh_resource=mesh_resource, pose_topic=args.pose_topic, frame_override=args.frame, scale=args.scale)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
