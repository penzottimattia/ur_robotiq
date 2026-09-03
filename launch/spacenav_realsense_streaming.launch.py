import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _serial_list(value):
    value = value.strip()
    if not value:
        return []
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        parsed = [item.strip() for item in value.split(',')]
    if not isinstance(parsed, (list, tuple)):
        parsed = [parsed]
    return [str(item).strip() for item in parsed if str(item).strip()]


def _launch_nodes(context):
    serials = _serial_list(LaunchConfiguration('serials').perform(context))
    return [
        Node(
            package='spacenav',
            executable='spacenav_node',
            name='spacenav_node',
            output='screen',
        ),
        Node(
            package='ur_robotiq',
            executable='spacenav_cartesian_target',
            name='spacenav_cartesian_target',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'target_topic': LaunchConfiguration('target_topic'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'translation_scale': LaunchConfiguration('translation_scale'),
                'rotation_scale': LaunchConfiguration('rotation_scale'),
                'offset_in_tool_frame': LaunchConfiguration('offset_in_tool_frame'),
            }],
        ),
        Node(
            package='ur_robotiq',
            executable='realsense_compressed_publisher',
            name='realsense_compressed_publisher',
            output='screen',
            parameters=[{
                'serials': serials,
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'topic_prefix': LaunchConfiguration('topic_prefix'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='world'),
        DeclareLaunchArgument('ee_frame', default_value='right_dorsum_link'),
        DeclareLaunchArgument(
            'target_topic',
            default_value='/right_cartesian_controller/target_frame'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('translation_scale', default_value='1.0'),
        DeclareLaunchArgument('rotation_scale', default_value='1.0'),
        DeclareLaunchArgument('offset_in_tool_frame', default_value='true'),
        DeclareLaunchArgument('serials', default_value='[]'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('jpeg_quality', default_value='85'),
        DeclareLaunchArgument('topic_prefix', default_value='/realsense'),
        OpaqueFunction(function=_launch_nodes),
    ])
