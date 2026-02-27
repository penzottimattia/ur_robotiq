from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_robot_ip_arg = DeclareLaunchArgument('left_robot_ip', default_value='192.168.0.10')
    right_robot_ip_arg = DeclareLaunchArgument('right_robot_ip', default_value='192.168.0.11')

    left_target_file_arg = DeclareLaunchArgument(
        'left_target_filename',
        default_value='/ws/src/ur_robotiq/config/left_ur3_calibration.yaml',
        description='Output calibration yaml for left robot',
    )
    right_target_file_arg = DeclareLaunchArgument(
        'right_target_filename',
        default_value='/ws/src/ur_robotiq/config/right_ur3_calibration.yaml',
        description='Output calibration yaml for right robot',
    )

    run_left_arg = DeclareLaunchArgument('run_left', default_value='true')
    run_right_arg = DeclareLaunchArgument('run_right', default_value='true')

    calibration_launch = PathJoinSubstitution([
        FindPackageShare('ur_calibration'),
        'launch',
        'calibration_correction.launch.py',
    ])

    left_extract = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(calibration_launch),
        launch_arguments={
            'robot_ip': LaunchConfiguration('left_robot_ip'),
            'target_filename': LaunchConfiguration('left_target_filename'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_left')),
    )

    right_extract = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(calibration_launch),
        launch_arguments={
            'robot_ip': LaunchConfiguration('right_robot_ip'),
            'target_filename': LaunchConfiguration('right_target_filename'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_right')),
    )

    return LaunchDescription([
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_target_file_arg,
        right_target_file_arg,
        run_left_arg,
        run_right_arg,
        left_extract,
        right_extract,
    ])
