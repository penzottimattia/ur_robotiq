from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('reference_frame', default_value='reference_object'),
        DeclareLaunchArgument('manipulated_frame', default_value='manipulated_object'),
        DeclareLaunchArgument('controlled_frame', default_value='right_dorsum_link'),
        DeclareLaunchArgument('command_topic', default_value='/right_cartesian_controller/target_frame'),
        DeclareLaunchArgument('insertion_depth', default_value='0.030'),
        DeclareLaunchArgument('dry_run', default_value='true'),
    ]
    node = Node(
        package='ur_robotiq',
        executable='visual_feedback_insertion',
        name='visual_feedback_insertion',
        output='screen',
        parameters=[{
            'world_frame': LaunchConfiguration('world_frame'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'manipulated_frame': LaunchConfiguration('manipulated_frame'),
            'controlled_frame': LaunchConfiguration('controlled_frame'),
            'command_topic': LaunchConfiguration('command_topic'),
            'insertion_depth': LaunchConfiguration('insertion_depth'),
            'dry_run': LaunchConfiguration('dry_run'),
        }],
    )
    return LaunchDescription(args + [node])
