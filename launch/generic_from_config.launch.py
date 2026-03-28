from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _launch_from_config(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_file').perform(context)

    import yaml
    import os

    if not os.path.exists(config_path):
        raise RuntimeError(f'Config file not found: {config_path}')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}

    nodes = []

    # rviz (optional)
    rviz_cfg = cfg.get('rviz', {})
    rviz_config = rviz_cfg.get('config', '') if isinstance(rviz_cfg, dict) else ''
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    ))

    # static camera TF node
    static_cfg = cfg.get('static_camera_tf', {})
    if static_cfg.get('enabled', True):
        nodes.append(Node(
            package='ur_robotiq',
            executable='static_camera_tf',
            name='static_camera_tf',
            output='screen',
        ))

    # spawn mesh markers for each object
    for obj in cfg.get('objects', []) or []:
        mesh = obj.get('mesh')
        if not mesh:
            continue
        pose_topic = obj.get('pose_topic', '/mesh_pose')
        frame = obj.get('frame', '')
        scale = obj.get('scale', 1.0)

        params = [{
            'mesh': mesh,
            'pose_topic': pose_topic,
            'frame': frame,
            'scale': float(scale),
        }]

        nodes.append(Node(
            package='ur_robotiq',
            executable='spawn_mesh_marker',
            name=f"spawn_{obj.get('name', os.path.basename(mesh))}",
            output='screen',
            parameters=params,
        ))

    # mock detected objects (publish mocked poses following frames)
    for i, m in enumerate(cfg.get('mocks', []) or []):
        params = [{
            'frame': m.get('frame', 'ee_link'),
            'target_frame': m.get('target_frame', 'world'),
            'output_topic': m.get('output_topic', '/mock_detected_object/pose'),
            'gt_topic': m.get('gt_topic', '/mock_detected_object/gt_pose'),
            'rate': float(m.get('rate', 10.0)),
            'pos_jitter_std': float(m.get('pos_jitter_std', 0.001)),
            'rot_jitter_std': float(m.get('rot_jitter_std', 0.01)),
            'offset_xyz': m.get('offset_xyz', [0.0, 0.0, 0.0]),
            'offset_rpy': m.get('offset_rpy', [0.0, 0.0, 0.0]),
        }]

        nodes.append(Node(
            package='ur_robotiq',
            executable='mock_detected_object',
            name=f"mock_detected_object_{i}",
            output='screen',
            parameters=params,
        ))

    # camera nodes
    for i, c in enumerate(cfg.get('cameras', []) or []):
        if not c.get('enabled', True):
            continue

        params = [{
            'device': int(c.get('device', 0)),
            'topic': c.get('topic', '/camera/image_raw'),
            'fps': float(c.get('fps', 10.0)),
            'output_size': int(c.get('output_size', 0)),
        }]

        name = c.get('name', f'usb_camera_{i}')

        nodes.append(Node(
            package='ur_robotiq',
            executable='usb_camera_node',
            name=name,
            output='screen',
            parameters=params,
        ))

    # tf pose transformers
    for i, t in enumerate(cfg.get('transforms', []) or []):
        params = [{
            'input_topic': t.get('input_topic', '/mesh_pose'),
            'output_topic': t.get('output_topic', '/object_pose_world'),
            'target_frame': t.get('target_frame', 'world'),
        }]
        nodes.append(Node(
            package='ur_robotiq',
            executable='tf_pose_transformer',
            name=f'tf_pose_transformer_{i}',
            output='screen',
            parameters=params,
        ))

    return nodes


def generate_launch_description():
    config_default = PathJoinSubstitution([
        FindPackageShare('ur_robotiq'),
        'config',
        'generic_launch.yaml',
    ])

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_default,
        description='Path to YAML config describing nodes to launch',
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=_launch_from_config),
    ])
