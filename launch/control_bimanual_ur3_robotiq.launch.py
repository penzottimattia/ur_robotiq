from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full_mock',
        description='Control mode: full_mock, mock_grippers_only, or calib',
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    launch_dashboard_clients_arg = DeclareLaunchArgument(
        'launch_dashboard_clients',
        default_value='true',
        description='Launch UR dashboard_client nodes for left and right robots when not using mock UR hardware',
    )
    dashboard_receive_timeout_arg = DeclareLaunchArgument(
        'dashboard_receive_timeout',
        default_value='20.0',
        description='Timeout for UR dashboard client responses',
    )
    run_setup_node_arg = DeclareLaunchArgument(
        'run_setup_node',
        default_value='false',
        description='Run bimanual dashboard setup sequence (power on, release brakes, load program, play)',
    )
    use_gello_arg = DeclareLaunchArgument(
        'use_gello',
        default_value='false',
        description='Launch GELLO publishers and offset nodes for teleoperation',
    )
    left_program_arg = DeclareLaunchArgument(
        'left_program',
        default_value='',
        description='Program path to load on left robot dashboard (optional)',
    )
    right_program_arg = DeclareLaunchArgument(
        'right_program',
        default_value='',
        description='Program path to load on right robot dashboard (optional)',
    )

    left_robot_ip_arg = DeclareLaunchArgument('left_robot_ip', default_value='0.0.0.0')
    right_robot_ip_arg = DeclareLaunchArgument('right_robot_ip', default_value='0.0.0.0')
    left_tool_device_name_arg = DeclareLaunchArgument(
        'left_tool_device_name', default_value='/tmp/ttyUR_left',
        description='Virtual serial device path for left gripper tool communication',
    )
    right_tool_device_name_arg = DeclareLaunchArgument(
        'right_tool_device_name', default_value='/tmp/ttyUR_right',
        description='Virtual serial device path for right gripper tool communication',
    )
    left_tool_tcp_port_arg = DeclareLaunchArgument(
        'left_tool_tcp_port', default_value='54321',
        description='TCP port for left UR tool communication bridge',
    )
    right_tool_tcp_port_arg = DeclareLaunchArgument(
        'right_tool_tcp_port', default_value='54322',
        description='TCP port for right UR tool communication bridge',
    )
    left_gello_port_arg = DeclareLaunchArgument(
        'left_gello_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for left GELLO (used only when use_gello=true)',
    )
    right_gello_port_arg = DeclareLaunchArgument(
        'right_gello_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for right GELLO (used only when use_gello=true)',
    )
    base_poses_file_arg = DeclareLaunchArgument(
        'base_poses_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'robot_bases.yaml',
        ]),
        description='YAML file with left/right robot base poses',
    )

    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'bimanual_controllers.yaml',
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
        'ur_robotiq.urdf',
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
        ' left_gripper_com_port:=', LaunchConfiguration('left_tool_device_name'),
        ' right_gripper_com_port:=', LaunchConfiguration('right_tool_device_name'),
        ' left_tool_tcp_port:=', LaunchConfiguration('left_tool_tcp_port'),
        ' right_tool_tcp_port:=', LaunchConfiguration('right_tool_tcp_port'),
        # Pass calibration file paths through to the xacro so they are available in the generated robot_description
        ' left_calib_file:=', LaunchConfiguration('left_calib_file'),
        ' right_calib_file:=', LaunchConfiguration('right_calib_file'),
    ])

    robot_description = {'robot_description': robot_description_content}

    launch_dashboard_clients = LaunchConfiguration('launch_dashboard_clients')
    dashboard_receive_timeout = LaunchConfiguration('dashboard_receive_timeout')
    start_dashboard_clients = PythonExpression([
        "'", launch_dashboard_clients, "' == 'true' and not (", use_mock_hardware, ")",
    ])
    run_setup_node = PythonExpression([
        "'", LaunchConfiguration('run_setup_node'), "' == 'true' and (", start_dashboard_clients, ")",
    ])

    def launch_ros2_control(context):
        mode = LaunchConfiguration('mode').perform(context)
        real_grippers = mode not in ('full_mock', 'mock_grippers_only')

        params = [
            robot_description,
            LaunchConfiguration('controllers_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
        # When using real grippers connected via UR tool I/O, start them
        # unconfigured so the ros2_control_node doesn't crash before the UR
        # driver creates the virtual serial ports.
        # Activate later with:
        #   ros2 control set_hardware_component_state left_gripper active
        #   ros2 control set_hardware_component_state right_gripper active
        if real_grippers:
            params.append({
                'hardware_components_initial_state.unconfigured': [
                    'left_gripper', 'right_gripper',
                ],
            })

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

    # socat bridges: forward UR tool RS-485 communication to local virtual serial ports.
    # Only launched when using real hardware (not mock).
    left_tool_comm = Node(
        package='ur_robot_driver',
        executable='tool_communication.py',
        name='left_ur_tool_comm',
        output='screen',
        condition=UnlessCondition(use_mock_hardware),
        parameters=[{
            'robot_ip': LaunchConfiguration('left_robot_ip'),
            'tcp_port': LaunchConfiguration('left_tool_tcp_port'),
            'device_name': LaunchConfiguration('left_tool_device_name'),
        }],
    )

    right_tool_comm = Node(
        package='ur_robot_driver',
        executable='tool_communication.py',
        name='right_ur_tool_comm',
        output='screen',
        condition=UnlessCondition(use_mock_hardware),
        parameters=[{
            'robot_ip': LaunchConfiguration('right_robot_ip'),
            'tcp_port': LaunchConfiguration('right_tool_tcp_port'),
            'device_name': LaunchConfiguration('right_tool_device_name'),
        }],
    )

    left_dashboard_client = Node(
        package='ur_robot_driver',
        executable='dashboard_client',
        namespace='left_ur',
        name='dashboard_client',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_dashboard_clients),
        parameters=[
            {'robot_ip': LaunchConfiguration('left_robot_ip')},
            {'receive_timeout': dashboard_receive_timeout},
        ],
    )

    right_dashboard_client = Node(
        package='ur_robot_driver',
        executable='dashboard_client',
        namespace='right_ur',
        name='dashboard_client',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_dashboard_clients),
        parameters=[
            {'robot_ip': LaunchConfiguration('right_robot_ip')},
            {'receive_timeout': dashboard_receive_timeout},
        ],
    )

    # Force/torque bridge nodes (provide zeroing service and zeroed output)
    left_ft_bridge = Node(
        package='ur_robotiq',
        executable='ft_bridge_node',
        name='left_fts_bridge',
        output='screen',
        parameters=[{
            'sensor_topic': 'left_fts_broadcaster/wrench',
            'output_topic': 'left_fts_bridge/wrench',
            'service_name': 'left_fts_bridge/reset_wrench',
        }],
        condition=UnlessCondition(use_mock_hardware)
    )

    right_ft_bridge = Node(
        package='ur_robotiq',
        executable='ft_bridge_node',
        name='right_fts_bridge',
        output='screen',
        parameters=[{
            'sensor_topic': 'right_fts_broadcaster/wrench',
            'output_topic': 'right_fts_bridge/wrench',
            'service_name': 'right_fts_bridge/reset_wrench',
        }],
        condition=UnlessCondition(use_mock_hardware)
    )

    bimanual_setup_node = Node(
        package='ur_robotiq',
        executable='bimanual_setup_node',
        name='bimanual_setup_node',
        output='screen',
        condition=IfCondition(run_setup_node),
        parameters=[
            {'left_namespace': 'left_ur'},
            {'right_namespace': 'right_ur'},
            {'left_program': LaunchConfiguration('left_program')},
            {'right_program': LaunchConfiguration('right_program')},
            {'wait_for_service_timeout': dashboard_receive_timeout},
            {'service_call_timeout': dashboard_receive_timeout},
        ],
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

    # True when using GELLO (teleop) mode
    use_gello = PythonExpression([
        "'", LaunchConfiguration('use_gello'), "' == 'true'",
    ])

    left_joint_state_to_traj_node = Node(
        package='ur_robotiq',
        executable='joint_state_to_trajectory_node',
        name='left_joint_state_to_trajectory_node',
        output='screen',
        condition=UnlessCondition(use_gello),
        parameters=[{
            'tf_prefix': 'left_',
            'joint_state_topic': '/left_arm_controller/commands',
            'trajectory_topic': '/left_arm_controller/joint_trajectory',
            'gripper_topic': '/left_gripper_controller/joint_trajectory',
            'gripper_joint': 'robotiq_85_left_knuckle_joint',
        }],
    )

    right_joint_state_to_traj_node = Node(
        package='ur_robotiq',
        executable='joint_state_to_trajectory_node',
        name='right_joint_state_to_trajectory_node',
        output='screen',
        condition=UnlessCondition(use_gello),
        parameters=[{
            'tf_prefix': 'right_',
            'joint_state_topic': '/right_arm_controller/commands',
            'trajectory_topic': '/right_arm_controller/joint_trajectory',
            'gripper_topic': '/right_gripper_controller/joint_trajectory',
            'gripper_joint': 'robotiq_85_left_knuckle_joint',
        }],
    )

    # Gello launch (includes GELLO publishers and offset nodes)
    gello_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'launch',
            'gello_offset_bimanual.launch.py',
        ]),
        condition=IfCondition(use_gello),
        launch_arguments=[
            ('left_gello_port', LaunchConfiguration('left_gello_port')),
            ('right_gello_port', LaunchConfiguration('right_gello_port')),
        ],
    )

    # After ros2_control_node has started we need to activate hardware gripper components
    # and load/activate controllers. Use a short delay to let the control node come up.
    # For real grippers: perform activation at 5s, then wait another 5s before spawning controllers.
    # For mock configurations: spawn controllers at the original 5s.
    post_start_actions = TimerAction(
        period=5.0,
        actions=[
            # activate real grippers only when not running in mock mode
            ExecuteProcess(
                cmd=['ros2', 'control', 'set_hardware_component_state', 'left_gripper', 'active'],
                output='screen',
                condition=IfCondition(real_grippers),
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'set_hardware_component_state', 'right_gripper', 'active'],
                output='screen',
                condition=IfCondition(real_grippers),
            ),

            # For real grippers: spawn controllers 5s after activation (total ~10s after start)
            TimerAction(
                period=5.0,
                actions=[
                    joint_state_broadcaster_spawner,
                    bimanual_controller_spawner,
                    left_joint_state_to_traj_node,
                    right_joint_state_to_traj_node,
                    # Launch GELLO after controllers are spawned for real grippers
                    gello_launch,
                ],
                condition=IfCondition(real_grippers),
            ),

            # For mock modes: spawn controllers at the original 5s delay
            TimerAction(
                period=0.0,
                actions=[
                    joint_state_broadcaster_spawner,
                    bimanual_controller_spawner,
                    left_joint_state_to_traj_node,
                    right_joint_state_to_traj_node,
                    # Launch GELLO after controllers are spawned for mock modes
                    gello_launch,
                ],
                condition=UnlessCondition(real_grippers),
            ),
        ],
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        launch_dashboard_clients_arg,
        dashboard_receive_timeout_arg,
        run_setup_node_arg,
        use_gello_arg,
        left_program_arg,
        right_program_arg,
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_tool_device_name_arg,
        right_tool_device_name_arg,
        left_tool_tcp_port_arg,
        right_tool_tcp_port_arg,
        left_gello_port_arg,
        right_gello_port_arg,
        base_poses_file_arg,
        controllers_file_arg,
        # include new calibration launch args
        left_calib_file_arg,
        right_calib_file_arg,
        robot_state_publisher,
        left_tool_comm,
        right_tool_comm,
        OpaqueFunction(function=launch_ros2_control),
        left_dashboard_client,
        right_dashboard_client,
        left_ft_bridge,
        right_ft_bridge,
        bimanual_setup_node,
        post_start_actions,
    ])
