# ur_robotiq

ROS 2 Jazzy package for a bimanual setup with two UR3 manipulators and Robotiq 2F-85 grippers.

## What this package provides

- Combined bimanual URDF/Xacro model (`urdf/ur_robotiq.urdf`)
- Robot base pose configuration (`config/robot_bases.yaml`)
- ros2_control controller configuration (`config/bimanual_controllers.yaml`)
- Launch files to:
  - visualize the bimanual robot
  - control the bimanual system
  - extract left/right UR calibration files

## Dependencies

This package relies on:

- `ur_description`
- `ur_robot_driver`
- `ur_calibration`
- `robotiq_description`
- Standard ROS 2 launch and control packages listed in `package.xml`

## Quick start

Build the package in your workspace:

```bash
cd /ws
colcon build --packages-select ur_robotiq
source install/setup.bash
```

### 1) Visualize bimanual setup

```bash
ros2 launch ur_robotiq view_bimanual_ur3_robotiq.launch.py
```

Useful launch arguments:

- `mode:=full_mock|mock_grippers_only`
- `use_rviz:=true|false`
- `use_joint_state_gui:=true|false`
- `left_robot_ip:=<ip>`
- `right_robot_ip:=<ip>`
- `base_poses_file:=<path/to/yaml>`

### 2) Run bimanual control stack

```bash
ros2 launch ur_robotiq control_bimanual_ur3_robotiq.launch.py
```

Useful launch arguments:

- `mode:=full_mock|mock_grippers_only`
- `launch_dashboard_clients:=true|false`
- `dashboard_receive_timeout:=20.0`
- `run_setup_node:=true|false` (run dashboard setup sequence)
- `left_program:=<robot_program_path>`
- `right_program:=<robot_program_path>`
- `left_robot_ip:=<ip>`
- `right_robot_ip:=<ip>`
- `controllers_file:=<path/to/controllers.yaml>`
- `base_poses_file:=<path/to/yaml>`

Important parameters:

- `left_namespace`, `right_namespace`
- `dashboard_namespace` (default: `dashboard_client`)
- `left_program`, `right_program` (empty means skip load step)
- `require_remote_control` (default: `true`)
- `skip_if_program_running` (default: `true`)
- `wait_for_service_timeout`, `service_call_timeout`

### GELLO control mode switching (SetParameters)

The GELLO offset node now uses ROS2 parameter updates (service type `rcl_interfaces/srv/SetParameters`) to switch runtime control mode through parameter `control_mode`.

Modes:

- `0`: idle (no command publishing)
- `1`: normal offset mode, including the gripper offset
- `2`: positive speed mode for one selected robot joint
- `3`: negative speed mode for one selected robot joint

When transitioning between normal mode and either speed mode, the node waits for a configurable delay before publishing commands again. The same delay applies when entering speed mode and when returning to `control_mode=1`.

When transitioning from any mode into `control_mode=1`, the node always recomputes offsets from the latest robot and GELLO joint states before accepting the transition.

Examples:

```bash
# Idle mode
ros2 param set /left_gello_offset_node control_mode 0

# Normal mode (forces offset recomputation first)
ros2 param set /left_gello_offset_node control_mode 1

# Positive speed mode
ros2 param set /left_gello_offset_node control_mode 2

# Negative speed mode
ros2 param set /left_gello_offset_node control_mode 3
```

Speed mode parameters:

- `speed_mode_joint_name`: robot joint to drive in speed mode (empty means the last arm joint)
- `speed_trigger_joint_index`: index of GELLO joint used as trigger in range `[0, 1]` (default `-1` uses the last GELLO joint)
- `speed_max_velocity`: maximum angular speed in rad/s scaled by trigger value
- `mode_transition_delay_seconds`: delay applied when entering or leaving speed mode
- `gripper_offset`: additional gripper bias applied only in normal mode

Examples:

```bash
ros2 param set /left_gello_offset_node speed_mode_joint_name left_wrist_3_joint
ros2 param set /left_gello_offset_node speed_trigger_joint_index 6
ros2 param set /left_gello_offset_node speed_max_velocity 1.2
ros2 param set /left_gello_offset_node mode_transition_delay_seconds 5.0
```

### 3) Extract calibration for both robots

```bash
ros2 launch ur_robotiq extract_bimanual_calibration.launch.py
```

Useful launch arguments:

- `left_robot_ip:=192.168.0.10`
- `right_robot_ip:=192.168.0.11`
- `left_target_filename:=/ws/src/ur_robotiq/config/left_ur_calibration.yaml`
- `right_target_filename:=/ws/src/ur_robotiq/config/right_ur_calibration.yaml`
- `run_left:=true|false`
- `run_right:=true|false`

### 4) Export single-unit URDF + meshes (for USD prep)

Use the unit xacro directly and generate one calibrated URDF per calibration file:

```bash
ros2 run ur_robotiq export_unit_assets \
  --input-xacro /ws/src/ur_robotiq/urdf/ur3_robotiq_unit.urdf.xacro \
  --output-dir /tmp/ur3_unit_export \
  --output-urdf-name unit.urdf \
  --xacro-args ur_type:=ur3 \
  --calibration-files \
    /ws/src/ur_robotiq/config/left_ur_calibration.yaml \
    /ws/src/ur_robotiq/config/right_ur_calibration.yaml \
  --overwrite
```

Output layout:

- `/tmp/ur3_unit_export/unit_left_ur_calibration.urdf`
- `/tmp/ur3_unit_export/unit_right_ur_calibration.urdf`
- `/tmp/ur3_unit_export/meshes/...` (copied visual/collision assets)

Notes:

- `--calibration-files` is applied via xacro argument `kinematics_parameters_file` by default.
- Use `--kinematics-arg-name` if your xacro expects a different argument name.
- `--xacro-args` accepts additional xacro assignments in `name:=value` format.

Legacy path (already expanded URDF, no per-calibration xacro generation):

```bash
ros2 run ur_robotiq export_unit_assets \
  --input-urdf /tmp/ur3_robotiq_unit_expanded.urdf \
  --output-dir /tmp/ur3_unit_export \
  --output-urdf-name unit.urdf \
  --overwrite
```

Launch wrapper (left/right calibration in one command):

```bash
ros2 launch ur_robotiq export_unit_assets.launch.py \
  run_left:=true \
  run_right:=true \
  output_dir:=/tmp/ur3_unit_export
```

## Notes

- `mode:=full_mock` is the default and is useful for bring-up without real hardware.
- For real robots, set valid robot IP addresses and verify network reachability.

## License

This project is licensed under the Apache License 2.0. See `LICENSE` for details.
