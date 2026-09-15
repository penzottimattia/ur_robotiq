import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_calib_file(override: str, ur_type: str) -> str:
    if override:
        return override
    return os.path.join(
        get_package_share_directory('ur_description'),
        'config',
        ur_type,
        'default_kinematics.yaml',
    )


def _transmission_config_file() -> str:
    calibration_dir = os.path.join(
        get_package_share_directory('mia_hand_description'),
        'calibration',
    )
    hand_specific = os.path.join(calibration_dir, 'transmission_config.yaml')
    if os.path.exists(hand_specific):
        return hand_specific
    return os.path.join(calibration_dir, 'transmission_config_default.yaml')


def _launch_setup(context, *args, **kwargs):
    setup = LaunchConfiguration('setup').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    base_poses_file = LaunchConfiguration('base_poses_file').perform(context)
    left_calib_file = _resolve_calib_file(
        LaunchConfiguration('left_calib_file').perform(context),
        'ur5' if setup == 'ur_mia' else 'ur3e',
    )
    right_calib_file = _resolve_calib_file(
        LaunchConfiguration('right_calib_file').perform(context),
        'ur5' if setup == 'ur_mia' else 'ur3e',
    )

    use_mock_hardware = mode == 'full_mock'
    use_calib_probe = mode == 'calib'
    use_mock_end_effectors = mode in ('full_mock', 'mock_grippers_only')

    urdf_name = 'ur_mia.urdf' if setup == 'ur_mia' else 'ur_robotiq.urdf'
    urdf_file = os.path.join(
        get_package_share_directory('ur_robotiq'),
        'urdf',
        urdf_name,
    )

    xacro_cmd = [
        'xacro ',
        urdf_file,
        f' use_mock_hardware:={"true" if use_mock_hardware else "false"}',
        f' use_calib_probe:={"true" if use_calib_probe else "false"}',
        f' base_poses_file:={base_poses_file}',
        f' left_calib_file:={left_calib_file}',
        f' right_calib_file:={right_calib_file}',
    ]

    if setup == 'ur_mia':
        xacro_cmd.append(
            f' use_mock_hands:={"true" if use_mock_end_effectors else "false"}',
        )
    else:
        xacro_cmd.append(
            f' use_mock_grippers:={"true" if use_mock_end_effectors else "false"}',
        )

    robot_description = {'robot_description': Command(xacro_cmd)}
    use_sim_time = LaunchConfiguration('use_sim_time')
    transmission_config = _transmission_config_file()

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            remappings=[('joint_states', 'remapped_joint_states')]
            if setup == 'ur_mia' and mode != 'calib'
            else [],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_joint_state_gui')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    if setup == 'ur_mia' and mode != 'calib':
        nodes.extend([
            Node(
                package='mia_hand_description',
                name='left_rviz2_joint_state_publisher',
                executable='rviz2_joint_state_publisher_node',
                output='screen',
                parameters=[
                    robot_description,
                    transmission_config,
                    {'prefix': 'left_hand_'},
                ],
                remappings=[
                    ('joint_states', 'joint_states'),
                    ('remapped_joint_states', 'left_remapped_joint_states'),
                ],
            ),
            Node(
                package='mia_hand_description',
                name='right_rviz2_joint_state_publisher',
                executable='rviz2_joint_state_publisher_node',
                output='screen',
                parameters=[
                    robot_description,
                    transmission_config,
                    {'prefix': 'right_hand_'},
                ],
                remappings=[
                    ('joint_states', 'left_remapped_joint_states'),
                    ('remapped_joint_states', 'remapped_joint_states'),
                ],
            ),
        ])

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'setup',
            default_value='ur_robotiq',
            description='Robot configuration: ur_robotiq or ur_mia',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='full_mock',
            description='Control mode: full_mock, mock_grippers_only, or calib',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joint_state_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'base_poses_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ur_robotiq'),
                'config',
                'robot_bases.yaml',
            ]),
            description='YAML file with the left->right base transform',
        ),
        DeclareLaunchArgument(
            'left_calib_file',
            default_value='',
            description='Override left arm kinematics YAML (empty = setup default)',
        ),
        DeclareLaunchArgument(
            'right_calib_file',
            default_value='',
            description='Override right arm kinematics YAML (empty = setup default)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
