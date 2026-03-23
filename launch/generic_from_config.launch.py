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
    if rviz_config:
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
        frame = obj.get('frame', None)
        scale = obj.get('scale', None)
        args = [mesh, '--pose-topic', pose_topic]
        if frame:
            args += ['--frame', frame]
        if scale is not None:
            args += ['--scale', str(scale)]

        nodes.append(Node(
            package='ur_robotiq',
            executable='spawn_mesh_marker',
            name=f"spawn_{obj.get('name', os.path.basename(mesh))}",
            output='screen',
            arguments=args,
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
