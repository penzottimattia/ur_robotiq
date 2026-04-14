from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_gui_arg = DeclareLaunchArgument('use_joint_state_gui', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    base_poses_file_arg = DeclareLaunchArgument(
        'base_poses_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'robot_bases.yaml',
        ]),
        description='YAML file with left/right robot base poses',
    )

    left_calib_file_arg = DeclareLaunchArgument(
        'left_calib_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'),
            'config',
            'ur3',
            'default_kinematics.yaml',
        ]),
        description='YAML file with left robot kinematics calibration',
    )

    right_calib_file_arg = DeclareLaunchArgument(
        'right_calib_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'),
            'config',
            'ur3',
            'default_kinematics.yaml',
        ]),
        description='YAML file with right robot kinematics calibration',
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
        "'", LaunchConfiguration('mode'), "' in ['full_mock', 'mock_grippers_only']",
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
        ' base_poses_file:=', LaunchConfiguration('base_poses_file'),
        ' left_calib_file:=', LaunchConfiguration('left_calib_file'),
        ' right_calib_file:=', LaunchConfiguration('right_calib_file'),
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_joint_state_gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        mode_arg,
        use_rviz_arg,
        use_gui_arg,
        use_sim_time_arg,
        base_poses_file_arg,
        left_calib_file_arg,
        right_calib_file_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
