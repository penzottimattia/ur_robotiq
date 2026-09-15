# ur_robotiq

ROS 2 integration package for bimanual Universal Robots setups. The repository contains two supported robot configurations:

- **UR3e + Robotiq 2F-85**: two UR3e arms with Robotiq grippers
- **UR5 + MIA hands**: two UR5 arms with Prensilia MIA hands

It provides combined Xacro/URDF descriptions, `ros2_control` configurations, real and mock hardware launch paths, GELLO and Cartesian teleoperation utilities, calibration tools, vision and TF utilities, RealSense streaming and recording, and visual-feedback insertion workflows.

> [!CAUTION]
> Commands in this repository can move physical robots. Start in mock mode, validate the TF tree and controller configuration, keep an emergency stop accessible, and use conservative gains, speeds, forces, and workspace limits.

## Contents

- [Supported configurations](#supported-configurations)
- [Requirements](#requirements)
- [Build](#build)
- [Quick start](#quick-start)
- [UR3e + Robotiq control](#ur3e--robotiq-control)
- [UR5 + MIA control](#ur5--mia-control)
- [GELLO teleoperation](#gello-teleoperation)
- [Calibration](#calibration)
- [Config-driven perception utilities](#config-driven-perception-utilities)
- [SpaceNavigator and RealSense](#spacenavigator-and-realsense)
- [Visual-feedback insertion and recording](#visual-feedback-insertion-and-recording)
- [Unit asset export](#unit-asset-export)
- [Teach Pendant setup](#teach-pendant-setup)
- [Package layout](#package-layout)
- [Testing](#testing)
- [License](#license)

## Supported configurations

### Bimanual UR3e + Robotiq 2F-85

Primary files:

- Robot description: `urdf/ur_robotiq.urdf`
- Reusable unit Xacro: `urdf/ur_robotiq_unit.urdf.xacro`
- Controller configuration: `config/bimanual_controllers.yaml`
- Control launch: `launch/control_bimanual_ur_robotiq.launch.py`

The control stack supports:

- `full_mock`: mock arms and mock grippers
- `mock_grippers_only`: real arms and mock grippers
- `calib`: calibration-probe configuration
- Real arms and real Robotiq grippers when `mode` is set to another value, conventionally `none`

### Bimanual UR5 + MIA hands

Primary files:

- Robot description: `urdf/ur_mia.urdf`
- Reusable unit Xacro: `urdf/ur_mia_unit.urdf.xacro`
- Controller configuration: `config/bimanual_hand_controllers.yaml`
- Control launch: `launch/control_bimanual_ur_mia.launch.py`

This setup supports mock or real arms, mock or serial-connected MIA hands, optional SenseGlove input, Vive tracker control, and Cartesian command stitching.

## Requirements

The package targets ROS 2 and is built as an `ament_python` package. The supplied container is based on **ROS 2 Humble**. Some package metadata still describes the project as Jazzy-oriented, so use a dependency set whose branches match your selected ROS distribution.

Core dependencies include:

- `ur_description`, `ur_robot_driver`, `ur_calibration`
- `robotiq_description` and the Robotiq ROS 2 driver
- `mia_hand_description`, `mia_hand_ros2_control`
- `controller_manager`, `joint_trajectory_controller`, `joint_state_broadcaster`
- `rclpy`, `tf2_ros`, `tf_transformations`
- `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`
- `cv_bridge`, OpenCV, `pyrealsense2`
- Python packages used by individual tools, including `numpy`, `PyYAML`, `h5py`, `pynput`, and `dynamixel_sdk`

Optional workflows additionally require their corresponding packages, for example `spacenav`, `libsurvive_arm_control`, `senseglove_interface`, and Cartesian controller packages.

## Build

Create a ROS 2 workspace, place this repository under `src`, install dependencies, and build:

```bash
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
git clone -b "$ROS_DISTRO" https://github.com/penzottimattia/ur_robotiq.git
git clone -b "$ROS_DISTRO" https://github.com/penzottimattia/ros2_robotiq_gripper.git
git clone -b ros2 https://github.com/penzottimattia/serial.git

cd ~/ur_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For subsequent shells:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ur_ws/install/setup.bash
```

### Docker image

The included `Dockerfile` clones the required repositories, resolves ROS dependencies, creates default UR3e calibration files, and installs the workspace under `/opt/ros/$ROS_DISTRO/ur_robotiq`.

A prebuilt deployment image can be started as follows:

```bash
docker start ur_robotiq || \
  docker run -dit \
    --net host \
    --privileged \
    --name ur_robotiq \
    --entrypoint bash \
    ghcr.io/penzottimattia/ur_robotiq:gello
```

The container needs host networking for ROS and robot connections, plus device access for serial, GELLO, camera, or hand hardware. Prefer explicit `--device` mappings over `--privileged` where practical.

## Quick start

### Visualize either supported setup

Use the unified visualization launch file:

```bash
# UR3e + Robotiq
ros2 launch ur_robotiq view_bimanual_unit.launch.py \
  setup:=ur_robotiq \
  mode:=full_mock

# UR5 + MIA
ros2 launch ur_robotiq view_bimanual_unit.launch.py \
  setup:=ur_mia \
  mode:=full_mock
```

Useful arguments:

```text
setup:=ur_robotiq|ur_mia
mode:=full_mock|mock_grippers_only|calib
use_rviz:=true|false
use_joint_state_gui:=true|false
use_sim_time:=true|false
base_poses_file:=/path/to/robot_bases.yaml
left_calib_file:=/path/to/left_kinematics.yaml
right_calib_file:=/path/to/right_kinematics.yaml
```

### Start the UR3e + Robotiq stack in mock mode

```bash
ros2 launch ur_robotiq control_bimanual_ur_robotiq.launch.py \
  mode:=full_mock
```

### Start the UR5 + MIA stack in mock mode

```bash
ros2 launch ur_robotiq control_bimanual_ur_mia.launch.py \
  mode:=full_mock
```

Inspect all arguments accepted by a launch file with:

```bash
ros2 launch ur_robotiq control_bimanual_ur_robotiq.launch.py -s
ros2 launch ur_robotiq control_bimanual_ur_mia.launch.py -s
```

## UR3e + Robotiq control

### Real-hardware launch

Replace the addresses and ports with values for your installation:

```bash
ros2 launch ur_robotiq control_bimanual_ur_robotiq.launch.py \
  mode:=none \
  left_robot_ip:=192.168.1.4 \
  right_robot_ip:=192.168.1.5 \
  left_custom_port:=50002 \
  right_custom_port:=50102 \
  left_tool_tcp_port:=54321 \
  right_tool_tcp_port:=54322
```

The launch file starts the UR tool-communication bridges, dashboard clients, force/torque bridges, `ros2_control`, controller spawners, and command conversion nodes. Real gripper hardware is initially left unconfigured, activated after startup, and followed by controller activation.

Important arguments include:

```text
mode
left_robot_ip, right_robot_ip
left_custom_port, right_custom_port
left_tool_device_name, right_tool_device_name
left_tool_tcp_port, right_tool_tcp_port
left_calib_file, right_calib_file
base_poses_file
controllers_file
proportional_gain, feedforward_gain
gripper_force_multiplier
gripper_threshold
gripper_full_close_threshold
gripper_offset
launch_dashboard_clients
dashboard_receive_timeout
run_setup_node
left_program, right_program
use_gello
use_gello_stitcher
```

> [!NOTE]
> `config/robot_bases.yaml` is the current default base-pose file. Replace its values with transforms measured for your installation.

### Dashboard operations

The launch can optionally run a setup sequence for both robots:

```bash
ros2 launch ur_robotiq control_bimanual_ur_robotiq.launch.py \
  mode:=none \
  left_robot_ip:=192.168.1.4 \
  right_robot_ip:=192.168.1.5 \
  run_setup_node:=true \
  left_program:=/programs/left_external_control.urp \
  right_program:=/programs/right_external_control.urp
```

The helper script applies validated dashboard actions to both namespaces:

```bash
bash scripts/ur_dashboard.sh brake_release play
bash scripts/ur_dashboard.sh stop power_off
```

Accepted actions are `play`, `pause`, `stop`, `shutdown`, `brake_release`, and `power_off`.

### Force/torque zeroing

For real UR hardware, the control launch starts zeroing bridges for the two force/torque streams:

```bash
ros2 service call /left_fts_bridge/reset_wrench std_srvs/srv/Trigger '{}'
ros2 service call /right_fts_bridge/reset_wrench std_srvs/srv/Trigger '{}'
```

Zeroed wrench topics are:

```text
/left_fts_bridge/wrench
/right_fts_bridge/wrench
```

## UR5 + MIA control

Start the bimanual UR5 + MIA stack:

```bash
ros2 launch ur_robotiq control_bimanual_ur_mia.launch.py \
  mode:=full_mock
```

For real arms and hands, configure robot addresses and serial ports:

```bash
ros2 launch ur_robotiq control_bimanual_ur_mia.launch.py \
  mode:=none \
  left_robot_ip:=192.168.1.4 \
  right_robot_ip:=192.168.1.5 \
  left_mia_port:=/dev/ttyUSB0 \
  right_mia_port:=/dev/ttyUSB1
```

Useful optional integrations:

```text
use_trackers:=true       # Vive/libsurvive Cartesian arm targets
use_gloves:=true         # SenseGlove hand commands
use_cartesian_stitcher:=true
left_glove_config_file:=...
right_glove_config_file:=...
```

Tracker mode requires a running libsurvive stack and site-specific tracker IDs and calibration. The IDs currently present in the launch file should be treated as installation-specific examples.

## GELLO teleoperation

The package supports two GELLO devices with six arm joints and one gripper input each.

### Launch with the UR3e + Robotiq stack

```bash
ros2 launch ur_robotiq control_bimanual_ur_robotiq.launch.py \
  mode:=full_mock \
  use_gello:=true
```

For a stitched 14-value bimanual command stream:

```bash
ros2 launch ur_robotiq control_bimanual_ur_robotiq.launch.py \
  mode:=full_mock \
  use_gello_stitcher:=true
```

### Launch GELLO nodes directly

```bash
ros2 launch ur_robotiq gello_offset_bimanual.launch.py \
  left_gello_id:=1 \
  right_gello_id:=2
```

The default device mappings are installation-specific serial-by-ID paths. Override a publisher's `port` parameter or update the deployment configuration for other devices.

### Runtime control modes

Each `gello_offset_node` exposes the `control_mode` parameter:

- `0`: idle, no command publication
- `1`: normal offset control
- `2`: positive speed mode for one selected robot joint
- `3`: negative speed mode for one selected robot joint

Examples:

```bash
ros2 param set /left_gello_offset_node control_mode 0
ros2 param set /left_gello_offset_node control_mode 1
ros2 param set /left_gello_offset_node control_mode 2
ros2 param set /left_gello_offset_node control_mode 3
```

Speed-mode settings:

```bash
ros2 param set /left_gello_offset_node speed_mode_joint_name left_wrist_3_joint
ros2 param set /left_gello_offset_node speed_trigger_joint_index 6
ros2 param set /left_gello_offset_node speed_max_velocity 1.2
ros2 param set /left_gello_offset_node mode_transition_delay_seconds 5.0
```

When switching into normal mode, offsets are recomputed from the latest robot and GELLO states. Active-to-active mode changes can impose a transition delay. The launch configures explicit wait services:

```bash
ros2 service call /left_gello_offset_node/wait_for_mode_transition std_srvs/srv/Empty '{}'
ros2 service call /right_gello_offset_node/wait_for_mode_transition std_srvs/srv/Empty '{}'
```

### GELLO position-control service

The GELLO publisher can also enable servo position control:

```bash
ros2 service call /gello_1/set_position_control std_srvs/srv/SetBool '{data: true}'
ros2 topic pub /gello_1/command_joints sensor_msgs/msg/JointState \
  '{position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.5]}'
ros2 service call /gello_1/set_position_control std_srvs/srv/SetBool '{data: false}'
```

## Calibration

### Extract UR kinematics

```bash
ros2 launch ur_robotiq extract_bimanual_calibration.launch.py \
  left_robot_ip:=192.168.0.10 \
  right_robot_ip:=192.168.0.11 \
  left_target_filename:=$PWD/config/left_ur_calibration.yaml \
  right_target_filename:=$PWD/config/right_ur_calibration.yaml
```

Use `run_left:=false` or `run_right:=false` to process only one arm.

### Camera-to-base hand-eye calibration

First visualize or launch the robot with `mode:=calib`, then run the calibrator for one side. Example for the left robot:

```bash
ros2 run ur_robotiq hand_eye_calibration --ros-args \
  -p probe_frame:=left_probe_link \
  -p base_frame:=left_base_link \
  -p camera_frame:=robotcam_color_optical_frame \
  -p pose_topic:=/object_pose \
  -p samples:=1000 \
  -p output_file:=$PWD/config/camera_to_left_base.yaml
```

Right-side example:

```bash
ros2 run ur_robotiq hand_eye_calibration --ros-args \
  -p probe_frame:=right_probe_link \
  -p base_frame:=right_base_link \
  -p camera_frame:=camera_color_optical_frame \
  -p pose_topic:=/object_pose \
  -p output_file:=$PWD/config/camera_to_right_base.yaml
```

The node averages observed transforms, writes YAML, and publishes the result as a static transform. Sampling can be ended early with a key press when global keyboard monitoring is available.

### Derive the relative base transform

```bash
python3 scripts/compute_right_in_left.py \
  --left config/camera_to_left_base.yaml \
  --right config/camera_to_right_base.yaml \
  --output config/right_base_in_left_base.yaml
```

Review the generated frame IDs and transform convention before using the result as a robot-base configuration.

## Config-driven perception utilities

`generic_from_config.launch.py` builds a small perception and visualization graph from YAML:

```bash
ros2 launch ur_robotiq generic_from_config.launch.py \
  config_file:=$PWD/config/generic_launch.yaml
```

The schema supports:

```yaml
rviz:
  config: ''

static_camera_tf:
  enabled: true

objects:
  - name: object_name
    mesh: /absolute/path/to/object.obj
    pose_topic: /object_pose
    frame: world
    child_frame: object_frame
    scale: 1.0

mocks:
  - name: object_mock
    frame: tool0
    target_frame: world
    output_topic: /object_pose
    gt_topic: /object_pose_gt
    rate: 10.0
    pos_jitter_std: 0.001
    rot_jitter_std: 0.01
    offset_xyz: [0.0, 0.0, 0.0]
    offset_rpy: [0.0, 0.0, 0.0]

transforms:
  - input_topic: /object_pose
    output_topic: /object_pose_world
    target_frame: world
    average_count: 5
    offset_xyz: [0.0, 0.0, 0.0]

cameras:
  - name: camera
    device: 0
    topic: /camera/image_raw
    fps: 10.0
    output_size: 224
    enabled: true
```

Included examples cover generic object visualization, mocked objects, nut-and-bolt scenes, peg-and-hole scenes, and triangle meshes. Object-file paths under `/ws/src/ur_robotiq/object` are deployment-specific and must exist locally.

## SpaceNavigator and RealSense

### SpaceNavigator Cartesian target + compressed streams

```bash
ros2 launch ur_robotiq spacenav_realsense_streaming.launch.py \
  spacenav_enabled:=true \
  base_frame:=world \
  ee_frame:=right_dorsum_link \
  target_topic:=/right_cartesian_controller/target_frame \
  serials:="['SERIAL_1', 'SERIAL_2']" \
  width:=640 \
  height:=480 \
  fps:=30 \
  jpeg_quality:=85
```

If `serials` is empty, the compressed publisher uses all detected RealSense devices. Topics follow:

```text
/realsense/camera_<serial>/color/image_raw/compressed
```

### Multi-camera HDF5 recorder

```bash
ros2 run ur_robotiq realsense_hdf5_recorder --ros-args \
  -p serials:="['SERIAL_1', 'SERIAL_2']" \
  -p width:=640 \
  -p height:=480 \
  -p fps:=30 \
  -p output_file:=/data/dataset.h5 \
  -p compression:=lzf
```

Recording is service-controlled:

```bash
ros2 service call /start_recording std_srvs/srv/Trigger '{}'
ros2 service call /pause_recording std_srvs/srv/Trigger '{}'
ros2 service call /resume_recording std_srvs/srv/Trigger '{}'
ros2 service call /stop_recording std_srvs/srv/Trigger '{}'
ros2 service call /discard_last_recording std_srvs/srv/Trigger '{}'
```

Each completed start/stop cycle creates `/demos/demo_NNNNNN` with per-camera RGB frames, host timestamps, device timestamps, and frame numbers.

## Visual-feedback insertion and recording

Launch the insertion controller in dry-run mode first:

```bash
ros2 launch ur_robotiq visual_feedback_insertion.launch.py \
  world_frame:=world \
  reference_frame:=reference_object \
  manipulated_frame:=manipulated_object \
  controlled_frame:=right_dorsum_link \
  command_topic:=/right_cartesian_controller/target_frame \
  insertion_depth:=0.030 \
  dry_run:=true
```

Available services:

```bash
ros2 service call /visual_feedback_insertion/start std_srvs/srv/Trigger '{}'
ros2 service call /visual_feedback_insertion/execute std_srvs/srv/Trigger '{}'
ros2 service call /visual_feedback_insertion/execute_post std_srvs/srv/Trigger '{}'
ros2 service call /visual_feedback_insertion/stop std_srvs/srv/Trigger '{}'
```

The controller performs bounded lateral alignment followed by insertion along the reference frame's negative Z axis. It checks TF availability, transform age, initial lateral error, step limits, depth tolerance, and optional Z-axis rotational alignment.

A second translation-only stage is enabled only when all of the following are configured:

```text
reference_frame_post
manipulated_frame_post
depth_post > 0
```

> [!WARNING]
> Set `dry_run:=false` only after verifying frame conventions, controlled-frame orientation, target topic, tolerances, collision clearance, and emergency-stop behavior on the actual system.

The interactive `scripts/peg_hole.sh` helper coordinates insertion, controller switching, and HDF5 recorder services:

```bash
bash scripts/peg_hole.sh
bash scripts/peg_hole.sh --post
bash scripts/peg_hole.sh --bimanual
bash scripts/peg_hole.sh --post --bimanual
```

## Unit asset export

Export calibrated single-unit URDFs and copy all referenced meshes into a self-contained directory:

```bash
ros2 run ur_robotiq export_unit_assets \
  --input-xacro /path/to/ur_robotiq_unit.urdf.xacro \
  --output-dir /tmp/ur3e_unit_export \
  --output-urdf-name unit.urdf \
  --xacro-args ur_type:=ur3e \
  --calibration-files \
    /path/to/left_ur_calibration.yaml \
    /path/to/right_ur_calibration.yaml \
  --overwrite
```

With two calibration files, outputs are named from the calibration stems, for example:

```text
/tmp/ur3e_unit_export/unit_left_ur_calibration.urdf
/tmp/ur3e_unit_export/unit_right_ur_calibration.urdf
/tmp/ur3e_unit_export/meshes/...
```

The launch wrapper provides the same common path:

```bash
ros2 launch ur_robotiq export_unit_assets.launch.py \
  run_left:=true \
  run_right:=true \
  output_dir:=/tmp/ur3e_unit_export
```

A legacy expanded-URDF path is also supported:

```bash
ros2 run ur_robotiq export_unit_assets \
  --input-urdf /tmp/unit_expanded.urdf \
  --output-dir /tmp/unit_export \
  --output-urdf-name unit.urdf \
  --overwrite
```

## Teach Pendant setup

Each physical UR must be prepared for external control:

1. Install and configure [External Control](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html).
2. Configure [networking](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/network_setup.html).
3. Install the [ToolComm Forwarder URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap) when using grippers through UR tool communication.

> [!CAUTION]
> The two robots must use distinct reverse, script-sender, trajectory, script-command, and ToolComm TCP ports where those services share the host network.

The repository defaults use ToolComm ports `54321` and `54322`. If both pendants initially expose `54321`, change one ToolComm Forwarder instance to `54322`, reboot that robot, and verify the active process and port. The original installation used shell access and edited the URCap daemon under its Felix cache. That path can vary by PolyScope and URCap version, so locate the installed daemon rather than assuming a fixed bundle number.

## Package layout

```text
ur_robotiq/
├── config/                  # Controllers, calibration, mock, glove, and scene YAML
├── launch/                  # Control, visualization, calibration, streaming, and utility launches
├── meshes/                  # Package-local visualization meshes
├── scripts/                 # Operational helper scripts
├── test/                    # ament lint tests
├── ur_robotiq/              # Python ROS 2 nodes
├── urdf/                    # Bimanual descriptions and reusable unit Xacros
├── Dockerfile
├── package.xml
├── setup.py
└── README.md
```

Selected executables:

```text
cartesian_absolute_to_delta
cartesian_stitcher_node
export_unit_assets
ft_bridge_node
gello_offset_node
gello_publisher
gello_stitcher_node
hand_eye_calibration
joint_state_to_trajectory_node
mock_detected_object
realsense_compressed_publisher
realsense_hdf5_recorder
spacenav_cartesian_target
spawn_mesh_marker
static_camera_tf
tf_pose_transformer
usb_camera_node
visual_feedback_insertion
vive_joy_node
```

## Testing

After building and sourcing the workspace:

```bash
colcon test --packages-select ur_robotiq
colcon test-result --verbose
```

You can also perform lightweight syntax checks from the repository root:

```bash
python3 -m compileall ur_robotiq launch
```

Hardware-dependent behavior, Xacro expansion, controller activation, TF availability, cameras, and serial devices require integration testing in an appropriately configured ROS environment.

## Troubleshooting

### Launch cannot find a package

Install missing dependencies with `rosdep`, verify that all source repositories are in the workspace, rebuild, and source both the ROS and workspace setup files.

### Real grippers do not activate

Check both ToolComm Forwarder ports, robot IP connectivity, local virtual serial paths, container device permissions, and the delayed hardware activation messages from `controller_manager`.

### GELLO device cannot be opened

Verify the serial-by-ID path, permissions, and that no other process owns the port. Override the node's `port` parameter if the device path differs from the built-in mapping.

### No insertion commands are published

Confirm that `dry_run` is false, the controller has been started, all required TF frames exist and are fresh, and the configured command topic is consumed by an active Cartesian controller.

### RealSense startup fails

List connected devices, provide valid serials, check USB bandwidth and permissions, and verify that the requested resolution and frame rate are supported by every selected device.

## License

Licensed under the Apache License 2.0. See [`LICENSE`](LICENSE) for details.
