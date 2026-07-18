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
    use_trackers_arg = DeclareLaunchArgument(
        'use_trackers',
        default_value='false',
        description='Launch VIVE trackers for arm control (requires libsurvive node running)',
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

    use_gloves_arg = DeclareLaunchArgument(
        'use_gloves',
        default_value='false',
        description='Launch SenseGlove interface nodes for bimanual control',
    )
    left_glove_config_file_arg = DeclareLaunchArgument(
        'left_glove_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'left_glove_interface.yaml',
        ]),
        description='Configuration YAML for left SenseGlove',
    )
    right_glove_config_file_arg = DeclareLaunchArgument(
        'right_glove_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'right_glove_interface.yaml',
        ]),
        description='Configuration YAML for right SenseGlove',
    )

    use_cartesian_stitcher_arg = DeclareLaunchArgument(
        'use_cartesian_stitcher',
        default_value='false',
        description='Launch command stitcher node for bimanual control',
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
        ' left_hand_serial_port:=', LaunchConfiguration('left_mia_port'),
        ' right_hand_serial_port:=', LaunchConfiguration('right_mia_port'),
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
        remappings=[('joint_states', 'remapped_joint_states')],
        condition=IfCondition(use_mock_grippers)
    )

    robot_state_publisher_without_remapper = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(use_mock_grippers),
    )

    left_mia_joint_state_remapper = Node(
        package='mia_hand_description',
        executable='rviz2_joint_state_publisher_node',
        name='left_mia_joint_state_remapper',
        output='screen',
        condition=IfCondition(use_mock_grippers),
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare('ur_robotiq'),
                'config',
                'left_hand_transmission.yaml',
            ]),
            {'prefix': 'left_hand_'},
        ],
    )

    right_mia_joint_state_remapper = Node(
        package='mia_hand_description',
        executable='rviz2_joint_state_publisher_node',
        name='right_mia_joint_state_remapper',
        output='screen',
        condition=IfCondition(use_mock_grippers),
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare('ur_robotiq'),
                'config',
                'right_hand_transmission.yaml',
            ]),
            {'prefix': 'right_hand_'},
        ],
    )

    def launch_state_broadcasters(context):
        mode = LaunchConfiguration('mode').perform(context)
        mock_grippers = mode in ('full_mock', 'mock_grippers_only')
        if mode == 'calib':
            arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager']
        else:
            arguments = ['joint_state_broadcaster', 'left_state_broadcaster', 'right_state_broadcaster', '--controller-manager', '/controller_manager']
        
        return [Node(
            package='controller_manager',
            executable='spawner',
            arguments=arguments,
            output='screen',
            remappings=[('joint_states', 'remapped_joint_states')] if mock_grippers else [],
        )]

    joint_state_broadcaster_spawner = OpaqueFunction(function=launch_state_broadcasters)

    bimanual_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', 'right_arm_controller', 'left_gripper_controller', 'right_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_cartesian_stitcher'), "' == 'true' and '",
            LaunchConfiguration('mode'), "' != 'calib'"
        ]))
    )

    stopped_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_cartesian_controller', 'right_cartesian_controller', 'left_hand_controller', 'right_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_cartesian_stitcher'), "' != 'true' and '",
            LaunchConfiguration('mode'), "' != 'calib'"
        ]))
    )

    left_joint_state_to_traj_node = Node(
        package='ur_robotiq',
        executable='joint_state_to_trajectory_node',
        name='left_joint_state_to_trajectory_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cartesian_stitcher')),
        parameters=[{
            'tf_prefix': 'left_',
            'joint_state_topic': '/left_arm_controller/commands',
            'trajectory_topic': '/left_arm_controller/joint_trajectory',
            'gripper_topic': '/left_gripper_controller/joint_trajectory',
            'gripper_joint_list': ['hand_j_index_fle', 'hand_j_thumb_fle', 'hand_j_mrl_fle'],
        }],
    )

    right_joint_state_to_traj_node = Node(
        package='ur_robotiq',
        executable='joint_state_to_trajectory_node',
        name='right_joint_state_to_trajectory_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cartesian_stitcher')),
        parameters=[{
            'tf_prefix': 'right_',
            'joint_state_topic': '/right_arm_controller/commands',
            'trajectory_topic': '/right_arm_controller/joint_trajectory',
            'gripper_topic': '/right_gripper_controller/joint_trajectory',
            'gripper_joint_list': ['hand_j_index_fle', 'hand_j_thumb_fle', 'hand_j_mrl_fle'],
        }],
    )

    cartesian_stitcher_node = Node(
        package='ur_robotiq',
        executable='cartesian_stitcher_node',
        name='cartesian_stitcher_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cartesian_stitcher')),
        parameters=[{
            'left_frame_topic': '/left_cartesian_controller/target_frame',
            'right_frame_topic': '/right_cartesian_controller/target_frame',
            'left_gripper_topic': '/left_hand_controller/target_state',
            'right_gripper_topic': '/right_hand_controller/target_state',
            'gripper_keys': ['index_flexion'],
            'output_topic': '/commands',
        }],
    )

    left_tracker_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('libsurvive_arm_control'),
            'launch',
            'libsurvive_arm_control.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('use_trackers')),
        launch_arguments=[
            ('prefix', 'left_'),
            ('tracker_frame_id', 'LHR-B618CEC9'),
            ('tool_frame_id', 'left_dorsum_link'),
            ('base_frame_id', 'world'),
            ('tool_output_frame_id', 'left_dorsum_target'),
            ('output_topic_name', '/left_cartesian_controller/target_frame'),
            ('publish_static_tf', 'true'),
            ('config_file', PathJoinSubstitution([
                FindPackageShare('ur_robotiq'),
                'config',
                'base_tracker.yaml',
            ])),
        ],
    )

    right_tracker_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('libsurvive_arm_control'),
            'launch',
            'libsurvive_arm_control.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('use_trackers')),
        launch_arguments=[
            ('prefix', 'right_'),
            ('tracker_frame_id', 'LHR-428A547D'),
            ('tool_frame_id', 'right_dorsum_link'),
            ('base_frame_id', 'world'),
            ('tool_output_frame_id', 'right_dorsum_target'),
            ('output_topic_name', '/right_cartesian_controller/target_frame'),
            ('publish_static_tf', 'false'),
        ],
    )

    left_glove_node = Node(
            package="senseglove_interface",
            executable="senseglove_interface_node",
            name="left_senseglove_interface",
            output="screen",
            parameters=[LaunchConfiguration('left_glove_config_file')],
            condition=IfCondition(LaunchConfiguration('use_gloves'))
        )
    
    right_glove_node = Node(
            package="senseglove_interface",
            executable="senseglove_interface_node",
            name="right_senseglove_interface",
            output="screen",
            parameters=[LaunchConfiguration('right_glove_config_file')],
            condition=IfCondition(LaunchConfiguration('use_gloves'))
        )
    
    vive_hand_node = Node(
            package="ur_robotiq",
            executable="vive_joy_node",
            name="vive_joy_node",
            output="screen",
            parameters=[{
                'input_topic': '/libsurvive/joy',
                'left_command_topic': '/left_hand_controller/commands',
                'right_command_topic': '/right_hand_controller/commands',
                'left_tracker_id': 'LHR-B618CEC9',
                'right_tracker_id': 'LHR-428A547D',
                'gripper_keys': ['index', 'thumb', 'middle'], # just a placeholder to determin array size, keys are not used in the node cause message type is Float64MultiArray
                'topic_type': 'std_msgs/Float64MultiArray'
            }],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration('use_trackers'),
                "' == 'true' and '",
                LaunchConfiguration('use_gloves'),
                "' != 'true' and '",
                LaunchConfiguration('use_cartesian_stitcher'),
                "' != 'true'",
            ]))
        )
    
    vive_gripper_node = Node(
            package="ur_robotiq",
            executable="vive_joy_node",
            name="vive_gripper_node",
            output="screen",
            parameters=[{
                'input_topic': '/libsurvive/joy',
                'left_command_topic': '/left_hand_controller/target_state',
                'right_command_topic': '/right_hand_controller/target_state',
                'left_tracker_id': 'LHR-B618CEC9',
                'right_tracker_id': 'LHR-428A547D',
                'gripper_keys': ['index_flexion'], # must match the keys used in the cartesian stitcher node, which are used to extract the gripper state from the Vive controller message
                'topic_type': 'sensor_msgs/JointState'
            }],
            condition=IfCondition(PythonExpression([
                "'",
                LaunchConfiguration('use_trackers'),
                "' == 'true' and '",
                LaunchConfiguration('use_gloves'),
                "' != 'true' and '",
                LaunchConfiguration('use_cartesian_stitcher'),
                "' == 'true'",
            ]))
        )


    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_mia_port_arg,
        right_mia_port_arg,
        use_trackers_arg,
        use_gloves_arg,
        use_cartesian_stitcher_arg,
        left_glove_config_file_arg,
        right_glove_config_file_arg,
        base_poses_file_arg,
        proportional_gain_arg,
        feedforward_gain_arg,
        controllers_file_arg,
        left_calib_file_arg,
        right_calib_file_arg,
        robot_state_publisher,
        robot_state_publisher_without_remapper,
        left_mia_joint_state_remapper,
        right_mia_joint_state_remapper,
        OpaqueFunction(function=launch_ros2_control),
        joint_state_broadcaster_spawner,
        bimanual_controller_spawner,
        stopped_controller_spawner,
        left_joint_state_to_traj_node,
        right_joint_state_to_traj_node,
        left_tracker_launch,
        right_tracker_launch,
        left_glove_node,
        right_glove_node,
        cartesian_stitcher_node,
        vive_hand_node,
        vive_gripper_node,
    ])