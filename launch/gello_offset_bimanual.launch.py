from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch gello offset nodes for bimanual robot control."""
    
    # Declare launch arguments
    left_gello_id_arg = DeclareLaunchArgument(
        'left_gello_id',
        default_value='1',
        description='GELLO device ID for left arm (1 or 2)',
    )
    right_gello_id_arg = DeclareLaunchArgument(
        'right_gello_id',
        default_value='2',
        description='GELLO device ID for right arm (1 or 2)',
    )
    left_gello_port_arg = DeclareLaunchArgument(
        'left_gello_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for left GELLO (override default)',
    )
    right_gello_port_arg = DeclareLaunchArgument(
        'right_gello_port',
        default_value='/dev/ttyUSB2',
        description='Serial port for right GELLO (override default)',
    )
    left_gripper_min_arg = DeclareLaunchArgument(
        'left_gripper_min',
        default_value='0.0',
        description='Minimum position value for left gripper',
    )
    left_gripper_max_arg = DeclareLaunchArgument(
        'left_gripper_max',
        default_value='0.8',
        description='Maximum position value for left gripper',
    )
    right_gripper_min_arg = DeclareLaunchArgument(
        'right_gripper_min',
        default_value='0.0',
        description='Minimum position value for right gripper',
    )
    right_gripper_max_arg = DeclareLaunchArgument(
        'right_gripper_max',
        default_value='0.8',
        description='Maximum position value for right gripper',
    )
    
    # Left GELLO publisher node
    left_gello_publisher_node = Node(
        package='ur_robotiq',
        executable='gello_publisher',
        name='left_gello_publisher',
        parameters=[
            {
                'gello_device': LaunchConfiguration('left_gello_id'),
                'port': LaunchConfiguration('left_gello_port'),
            }
        ],
        output='screen',
    )
    
    # Right GELLO publisher node
    right_gello_publisher_node = Node(
        package='ur_robotiq',
        executable='gello_publisher',
        name='right_gello_publisher',
        parameters=[
            {
                'gello_device': LaunchConfiguration('right_gello_id'),
                'port': LaunchConfiguration('right_gello_port'),
            }
        ],
        output='screen',
    )
    
    # Left gello offset node
    left_gello_offset_node = Node(
        package='ur_robotiq',
        executable='gello_offset_node',
        name='left_gello_offset_node',
        parameters=[
            {
                'gripper_min': LaunchConfiguration('left_gripper_min'),
                'gripper_max': LaunchConfiguration('left_gripper_max'),
                'gripper_joint_name': 'left_robotiq_85_left_knuckle_joint',
                'robot_joint_state_topic': '/left_state_broadcaster/joint_states',
                'trajectory_topic': '/left_arm_controller/joint_trajectory',
                'gripper_topic': '/left_gripper_controller/joint_trajectory',
                'gello_joint_state_topic': ['/gello_', LaunchConfiguration('left_gello_id'), '/joint_states'],
            }
        ],
        output='screen',
    )
    
    # Right gello offset node
    right_gello_offset_node = Node(
        package='ur_robotiq',
        executable='gello_offset_node',
        name='right_gello_offset_node',
        parameters=[
            {
                'gripper_min': LaunchConfiguration('right_gripper_min'),
                'gripper_max': LaunchConfiguration('right_gripper_max'),
                'gripper_joint_name': 'right_robotiq_85_left_knuckle_joint',
                'robot_joint_state_topic': '/right_state_broadcaster/joint_states',
                'trajectory_topic': '/right_arm_controller/joint_trajectory',
                'gripper_topic': '/right_gripper_controller/joint_trajectory',
                'gello_joint_state_topic': ['/gello_', LaunchConfiguration('right_gello_id'), '/joint_states'],
            }
        ],
        output='screen',
    )
    
    return LaunchDescription([
        left_gello_id_arg,
        right_gello_id_arg,
        left_gello_port_arg,
        right_gello_port_arg,
        left_gripper_min_arg,
        left_gripper_max_arg,
        right_gripper_min_arg,
        right_gripper_max_arg,
        left_gello_publisher_node,
        right_gello_publisher_node,
        left_gello_offset_node,
        right_gello_offset_node,
    ])
