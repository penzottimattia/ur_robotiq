from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('reference_frame', default_value='reference_object'),
        DeclareLaunchArgument('reference_frame_1', default_value=''),
        DeclareLaunchArgument('manipulated_frame', default_value='manipulated_object'),
        DeclareLaunchArgument('manipulated_frame_1', default_value=''),
        DeclareLaunchArgument('depth_1', default_value='0.0'),
        DeclareLaunchArgument('controlled_frame', default_value='right_dorsum_link'),
        DeclareLaunchArgument('command_topic', default_value='/right_cartesian_controller/target_frame'),
        DeclareLaunchArgument('insertion_depth', default_value='0.030'),
        DeclareLaunchArgument('orientation_jitter', default_value='0.0'),
        DeclareLaunchArgument('rotation_alignment_once', default_value='false'),
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
            'reference_frame_1': LaunchConfiguration('reference_frame_1'),
            'manipulated_frame': LaunchConfiguration('manipulated_frame'),
            'manipulated_frame_1': LaunchConfiguration('manipulated_frame_1'),
            'depth_1': LaunchConfiguration('depth_1'),
            'controlled_frame': LaunchConfiguration('controlled_frame'),
            'command_topic': LaunchConfiguration('command_topic'),
            'insertion_depth': LaunchConfiguration('insertion_depth'),
            'orientation_jitter': LaunchConfiguration('orientation_jitter'),
            'rotation_alignment_once': LaunchConfiguration('rotation_alignment_once'),
            'dry_run': LaunchConfiguration('dry_run'),
        }],
    )
    return LaunchDescription(args + [node])
