from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from ur_robotiq.vive_joy_node import resolve_topic_type


def test_resolve_topic_type_ros_style_names():
    assert resolve_topic_type('std_msgs/Float64MultiArray') is Float64MultiArray
    assert resolve_topic_type('sensor_msgs/JointState') is JointState


def test_resolve_topic_type_python_style_names():
    assert resolve_topic_type('std_msgs.msg.Float64MultiArray') is Float64MultiArray
    assert resolve_topic_type('sensor_msgs.msg.JointState') is JointState


def test_resolve_topic_type_bare_names():
    assert resolve_topic_type('Float64MultiArray') is Float64MultiArray
    assert resolve_topic_type('JointState') is JointState
