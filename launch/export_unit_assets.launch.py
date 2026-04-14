import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _str_to_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _run_exporter(context):
    input_xacro = LaunchConfiguration('input_xacro').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    output_urdf_name = LaunchConfiguration('output_urdf_name').perform(context)
    left_calibration_file = LaunchConfiguration('left_calibration_file').perform(context)
    right_calibration_file = LaunchConfiguration('right_calibration_file').perform(context)
    run_left = _str_to_bool(LaunchConfiguration('run_left').perform(context))
    run_right = _str_to_bool(LaunchConfiguration('run_right').perform(context))
    overwrite = _str_to_bool(LaunchConfiguration('overwrite').perform(context))
    no_auto_instantiate_unit = _str_to_bool(
        LaunchConfiguration('no_auto_instantiate_unit').perform(context)
    )
    kinematics_arg_name = LaunchConfiguration('kinematics_arg_name').perform(context)
    xacro_args = LaunchConfiguration('xacro_args').perform(context).strip()

    command = [
        'ros2',
        'run',
        'ur_robotiq',
        'export_unit_assets',
        '--input-xacro',
        input_xacro,
        '--output-dir',
        output_dir,
        '--output-urdf-name',
        output_urdf_name,
        '--kinematics-arg-name',
        kinematics_arg_name,
    ]

    if overwrite:
        command.append('--overwrite')

    if no_auto_instantiate_unit:
        command.append('--no-auto-instantiate-unit')

    if xacro_args:
        command.append('--xacro-args')
        command.extend(shlex.split(xacro_args))

    calibration_files = []
    if run_left:
        calibration_files.append(left_calibration_file)
    if run_right:
        calibration_files.append(right_calibration_file)

    if calibration_files:
        command.append('--calibration-files')
        command.extend(calibration_files)

    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    input_xacro_arg = DeclareLaunchArgument(
        'input_xacro',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'urdf',
            'ur3_robotiq_unit.urdf.xacro',
        ]),
        description='Path to the input xacro file.',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/ur3_unit_export',
        description='Output folder for exported URDF(s) and meshes.',
    )
    output_urdf_name_arg = DeclareLaunchArgument(
        'output_urdf_name',
        default_value='unit.urdf',
        description='Base name for output URDF file(s).',
    )
    left_calibration_file_arg = DeclareLaunchArgument(
        'left_calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'left_ur_calibration.yaml',
        ]),
        description='Left robot calibration file.',
    )
    right_calibration_file_arg = DeclareLaunchArgument(
        'right_calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_robotiq'),
            'config',
            'right_ur_calibration.yaml',
        ]),
        description='Right robot calibration file.',
    )
    run_left_arg = DeclareLaunchArgument(
        'run_left',
        default_value='true',
        description='Include left calibration file in export.',
    )
    run_right_arg = DeclareLaunchArgument(
        'run_right',
        default_value='true',
        description='Include right calibration file in export.',
    )
    overwrite_arg = DeclareLaunchArgument(
        'overwrite',
        default_value='true',
        description='Overwrite output URDF files when they already exist.',
    )
    kinematics_arg_name_arg = DeclareLaunchArgument(
        'kinematics_arg_name',
        default_value='kinematics_parameters_file',
        description='Xacro argument name used for calibration file path.',
    )
    xacro_args_arg = DeclareLaunchArgument(
        'xacro_args',
        default_value='ur_type:=ur3',
        description='Extra xacro args as a single string, e.g. "ur_type:=ur3 use_mock_hardware:=true".',
    )
    no_auto_instantiate_unit_arg = DeclareLaunchArgument(
        'no_auto_instantiate_unit',
        default_value='false',
        description='Disable automatic instantiate_unit:=true addition.',
    )

    run_exporter = OpaqueFunction(function=_run_exporter)

    return LaunchDescription([
        input_xacro_arg,
        output_dir_arg,
        output_urdf_name_arg,
        left_calibration_file_arg,
        right_calibration_file_arg,
        run_left_arg,
        run_right_arg,
        overwrite_arg,
        kinematics_arg_name_arg,
        xacro_args_arg,
        no_auto_instantiate_unit_arg,
        run_exporter,
    ])
