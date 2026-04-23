# ur_robotiq

ROS 2 Jazzy package for a bimanual setup with two UR3 manipulators and Robotiq 2F-85 grippers.

## What this package provides

- Combined bimanual URDF/Xacro model (`urdf/ur_robotiq.urdf`)
- Robot base transform configuration (`config/left_in_right.yaml`)
- ros2_control controller configuration (`config/bimanual_controllers.yaml`)
- Launch files to:
  - visualize the bimanual robot
  - control the bimanual system
  - calibrate camera-to-base transforms
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
- `base_poses_file:=<path/to/yaml>`  (`config/left_in_right.yaml` is the default)

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
- `base_poses_file:=<path/to/yaml>`  (`config/left_in_right.yaml` is the default)

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

When transitioning into any non-idle mode, the node normally waits for a configurable delay before publishing commands again. Transitioning to `control_mode=0` is immediate, and `control_mode=0 -> 1` is also immediate.

The node also exposes an empty ROS 2 service named `wait_for_transition_complete` by default. Call the service after a mode change to block until the transition delay has elapsed and the node can publish again.

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

# Block until the current transition delay has elapsed
ros2 service call /left_gello_offset_node/wait_for_transition_complete std_srvs/srv/Empty {}
```

Speed mode parameters:

- `speed_mode_joint_name`: robot joint to drive in speed mode (empty means the last arm joint)
- `speed_trigger_joint_index`: index of GELLO joint used as trigger in range `[0, 1]` (default `-1` uses the last GELLO joint)
- `speed_max_velocity`: maximum angular speed in rad/s scaled by trigger value
- `mode_transition_delay_seconds`: delay applied when switching into any non-idle mode
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

### 4) Calibrate camera to base

Use the `hand_eye_calibration` node to estimate a camera-to-base transform from a known probe frame and an object pose stream in the camera frame.

Example for the left robot:

```bash
ros2 run ur_robotiq hand_eye_calibration --ros-args \
  -p probe_frame:=left_probe_link \
  -p base_frame:=left_base_link \
  -p camera_frame:=camera_color_optical_frame \
  -p pose_topic:=/object_pose \
  -p output_file:=/ws/src/ur_robotiq/config/camera_to_left_base.yaml
```

Example for the right robot:

```bash
ros2 run ur_robotiq hand_eye_calibration --ros-args \
  -p probe_frame:=right_probe_link \
  -p base_frame:=right_base_link \
  -p camera_frame:=camera_color_optical_frame \
  -p pose_topic:=/object_pose \
  -p output_file:=/ws/src/ur_robotiq/config/camera_to_right_base.yaml
```

The node subscribes to `pose_topic` (default `/object_pose`), averages `samples` pose observations, writes the resulting transform to `output_file`, and also publishes it as a static transform.

The saved YAML uses the `camera_to_base` format and is consumed by `static_tf_broadcaster` from `config/camera_to_base.yaml`.

If you have both left and right camera calibrations, you can derive the relative base transform with:

```bash
python3 scripts/compute_right_in_left.py \
  --left config/camera_to_left_base.yaml \
  --right config/camera_to_right_base.yaml \
  --output config/left_in_right.yaml
```

### 5) Export single-unit URDF + meshes (for USD prep)

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
