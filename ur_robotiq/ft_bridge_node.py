import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger


class ForceTorqueBridge(Node):
    def __init__(self):
        super().__init__('ft_bridge')

        self.declare_parameter('sensor_topic', 'ft_data')
        self.declare_parameter('output_topic', 'ft_data_zeroed')
        self.declare_parameter('service_name', 'zero_ft')

        sensor_topic = self.get_parameter('sensor_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        service_name = self.get_parameter('service_name').get_parameter_value().string_value

        self._last_msg = None
        self._offset = None  # will hold a WrenchStamped representing zero offset

        self._pub = self.create_publisher(WrenchStamped, output_topic, 10)
        self._sub = self.create_subscription(WrenchStamped, sensor_topic, self._cb_ft, 10)

        self._srv = self.create_service(Trigger, service_name, self._handle_zero)

        self.get_logger().info(f'ForceTorqueBridge subscribing to "{sensor_topic}", '
                               f'publishing zeroed data on "{output_topic}", '
                               f'zero service "{service_name}"')

    def _cb_ft(self, msg: WrenchStamped):
        self._last_msg = msg
        if self._offset is None:
            out = msg
        else:
            out = WrenchStamped()
            out.header = msg.header
            out.wrench.force.x = msg.wrench.force.x - self._offset.wrench.force.x
            out.wrench.force.y = msg.wrench.force.y - self._offset.wrench.force.y
            out.wrench.force.z = msg.wrench.force.z - self._offset.wrench.force.z
            out.wrench.torque.x = msg.wrench.torque.x - self._offset.wrench.torque.x
            out.wrench.torque.y = msg.wrench.torque.y - self._offset.wrench.torque.y
            out.wrench.torque.z = msg.wrench.torque.z - self._offset.wrench.torque.z

        self._pub.publish(out)

    def _handle_zero(self, request, response):
        if self._last_msg is None:
            response.success = False
            response.message = 'No force/torque message received yet; cannot zero.'
            return response

        # copy last_msg into offset
        offset = WrenchStamped()
        offset.header = self._last_msg.header
        offset.wrench.force.x = self._last_msg.wrench.force.x
        offset.wrench.force.y = self._last_msg.wrench.force.y
        offset.wrench.force.z = self._last_msg.wrench.force.z
        offset.wrench.torque.x = self._last_msg.wrench.torque.x
        offset.wrench.torque.y = self._last_msg.wrench.torque.y
        offset.wrench.torque.z = self._last_msg.wrench.torque.z

        self._offset = offset

        response.success = True
        response.message = 'Zero offset set from last received message.'
        self.get_logger().debug('Zero offset updated.')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
