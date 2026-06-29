from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full_mock',
        description='Control mode: full_mock, mock_grippers_only, or calib',
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    left_robot_ip_arg = DeclareLaunchArgument('left_robot_ip', default_value='0.0.0.0')
    right_robot_ip_arg = DeclareLaunchArgument('right_robot_ip', default_value='0.0.0.0')
    left_mia_port_arg = DeclareLaunchArgument(
        'left_mia_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for left MIA',
    )
    right_mia_port_arg = DeclareLaunchArgument(
        'right_mia_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for right MIA',
    )
    base_poses_file_arg = DeclareLaunchArgument(
        'base_poses_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'robot_bases.yaml',
        ]),
        description='YAML file with the left->right base transform',
    )

    proportional_gain_arg = DeclareLaunchArgument(
        'proportional_gain',
        default_value='1.0',
        description='Proportional gain for arm position controllers (higher values result in stiffer control, but may cause damage to the robot if set too high; typically between 0.5 and 2.0)',
    )
    feedforward_gain_arg = DeclareLaunchArgument(
        'feedforward_gain',
        default_value='0.1',
        description='Feedforward gain for velocity commands in arm controllers (helps improve tracking performance, but may cause overshoot if set too high; typically between 0.0 and 0.5)',
    )

    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'bimanual_hand_controllers.yaml',
        ]),
        description='Controllers YAML for controller_manager',
    )

    # Add calibration file launch arguments so calibration YAMLs are passed through to the URDF xacro
    left_calib_file_arg = DeclareLaunchArgument(
        'left_calib_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'left_ur_calibration.yaml',
        ]),
        description='Calibration YAML for left robot',
    )

    right_calib_file_arg = DeclareLaunchArgument(
        'right_calib_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'right_ur_calibration.yaml',
        ]),
        description='Calibration YAML for right robot',
    )

    urdf_file = PathJoinSubstitution([
        FindPackageShare('ur_robotiq'),
        'urdf',
        'ur_mia.urdf',
    ])

    use_mock_hardware = PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'full_mock'",
    ])
    use_mock_grippers = PythonExpression([
        "'", LaunchConfiguration('mode'), "' in ['full_mock', 'mock_grippers_only', 'calib']",
    ])
    use_calib_probe = PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'calib'",
    ])

    # True when using real grippers attached to the robot (not in any mock mode)
    real_grippers = PythonExpression([
        "'", LaunchConfiguration('mode'), "' not in ['full_mock', 'mock_grippers_only']",
    ])

    robot_description_content = Command([
        'xacro ',
        urdf_file,
        ' use_mock_hardware:=', use_mock_hardware,
        ' use_mock_grippers:=', use_mock_grippers,
        ' use_calib_probe:=', use_calib_probe,
        ' left_robot_ip:=', LaunchConfiguration('left_robot_ip'),
        ' right_robot_ip:=', LaunchConfiguration('right_robot_ip'),
        ' base_poses_file:=', LaunchConfiguration('base_poses_file'),
        # Pass calibration file paths through to the xacro so they are available in the generated robot_description
        ' left_calib_file:=', LaunchConfiguration('left_calib_file'),
        ' right_calib_file:=', LaunchConfiguration('right_calib_file'),
    ])

    robot_description = {'robot_description': robot_description_content}

    def launch_ros2_control(context):
        mode = LaunchConfiguration('mode').perform(context)
        real_grippers = mode not in ('full_mock', 'mock_grippers_only')

        params = [
            robot_description,
            ParameterFile(LaunchConfiguration('controllers_file'), allow_substs=True),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]

        return [Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=params,
        )]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'left_state_broadcaster', 'left_fts_broadcaster', 'right_state_broadcaster', 'right_fts_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    bimanual_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', 'right_arm_controller', 'left_gripper_controller', 'right_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    left_joint_state_to_traj_node = Node(
        package='ur_robotiq',
        executable='joint_state_to_trajectory_node',
        name='left_joint_state_to_trajectory_node',
        output='screen',
        parameters=[{
            'tf_prefix': 'left_',
            'joint_state_topic': '/left_arm_controller/commands',
            'trajectory_topic': '/left_arm_controller/joint_trajectory',
            'gripper_topic': '/left_gripper_controller/joint_trajectory',
            'gripper_joint_list': ['left_hand_j_index_fle', 'left_hand_j_thumb_fle', 'left_hand_j_mrl_fle'],
        }],
    )

    right_joint_state_to_traj_node = Node(
        package='ur_robotiq',
        executable='joint_state_to_trajectory_node',
        name='right_joint_state_to_trajectory_node',
        output='screen',
        parameters=[{
            'tf_prefix': 'right_',
            'joint_state_topic': '/right_arm_controller/commands',
            'trajectory_topic': '/right_arm_controller/joint_trajectory',
            'gripper_topic': '/right_gripper_controller/joint_trajectory',
            'gripper_joint_list': ['right_hand_j_index_fle', 'right_hand_j_thumb_fle', 'right_hand_j_mrl_fle'],
        }],
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        left_robot_ip_arg,
        right_robot_ip_arg,
        base_poses_file_arg,
        proportional_gain_arg,
        feedforward_gain_arg,
        controllers_file_arg,
        left_calib_file_arg,
        right_calib_file_arg,
        robot_state_publisher,
        OpaqueFunction(function=launch_ros2_control),
        joint_state_broadcaster_spawner,
        bimanual_controller_spawner,
        left_joint_state_to_traj_node,
        right_joint_state_to_traj_node,
    ])
