#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticCameraTFNode(Node):
    def __init__(self):
        super().__init__('static_camera_tf_node')
        try:
            pkg_share = get_package_share_directory('ur_robotiq')
            cfg_path = os.path.join(pkg_share, 'config', 'camera_to_base.yaml')
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f)

            cam = data.get('camera_to_base', {})
            header = cam.get('header', {})
            tf = cam.get('transform', {})
            translation = tf.get('translation', {})
            rotation = tf.get('rotation_xyzw', {})

            t = TransformStamped()
            t.header.frame_id = header.get('frame_id', 'world')
            t.child_frame_id = header.get('child_frame_id', 'camera')
            t.header.stamp = self.get_clock().now().to_msg()

            t.transform.translation.x = float(translation.get('x', 0.0))
            t.transform.translation.y = float(translation.get('y', 0.0))
            t.transform.translation.z = float(translation.get('z', 0.0))

            t.transform.rotation.w = float(rotation.get('w', 1.0))
            t.transform.rotation.x = float(rotation.get('x', 0.0))
            t.transform.rotation.y = float(rotation.get('y', 0.0))
            t.transform.rotation.z = float(rotation.get('z', 0.0))

            self.broadcaster = StaticTransformBroadcaster(self)
            self.broadcaster.sendTransform(t)
            self.get_logger().info(
                f'Published static transform: {t.child_frame_id} -> {t.header.frame_id}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to publish static transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = StaticCameraTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
