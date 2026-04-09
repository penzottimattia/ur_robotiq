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
    mode_transition_delay_seconds_arg = DeclareLaunchArgument(
        'mode_transition_delay_seconds',
        default_value='5.0',
        description='Delay in seconds applied when entering or leaving speed mode',
    )
    
    # Left GELLO publisher node
    left_gello_publisher_node = Node(
        package='ur_robotiq',
        executable='gello_publisher',
        name='left_gello_publisher',
        parameters=[
            {
                'gello_device': LaunchConfiguration('left_gello_id'),
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
                'gripper_offset': 0.1,
                'gripper_joint_name': 'left_robotiq_85_left_knuckle_joint',
                'robot_joint_state_topic': '/left_state_broadcaster/joint_states',
                'command_topic': '/left_arm_controller/commands',
                'gello_joint_state_topic': ['/gello_', LaunchConfiguration('left_gello_id'), '/joint_states'],
                'control_mode': 1,
                'mode_transition_delay_seconds': LaunchConfiguration('mode_transition_delay_seconds'),
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
                'gripper_offset': 0.1,
                'gripper_joint_name': 'right_robotiq_85_left_knuckle_joint',
                'robot_joint_state_topic': '/right_state_broadcaster/joint_states',
                'command_topic': '/right_arm_controller/commands',
                'gello_joint_state_topic': ['/gello_', LaunchConfiguration('right_gello_id'), '/joint_states'],
                'control_mode': 1,
                'mode_transition_delay_seconds': LaunchConfiguration('mode_transition_delay_seconds'),
            }
        ],
        output='screen',
    )
    
    return LaunchDescription([
        left_gello_id_arg,
        right_gello_id_arg,
        left_gripper_min_arg,
        left_gripper_max_arg,
        right_gripper_min_arg,
        right_gripper_max_arg,
        mode_transition_delay_seconds_arg,
        left_gello_publisher_node,
        right_gello_publisher_node,
        left_gello_offset_node,
        right_gello_offset_node,
    ])
