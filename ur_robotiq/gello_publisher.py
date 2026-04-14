#!/usr/bin/env python3.10
"""Standalone ROS 2 node that publishes GELLO joint states and supports position control.

Parameters:
  - gello_device: GELLO ID to use (1 or 2). Default: 1

Publishes joint states from the specified GELLO device:
  - GELLO #1 (FTB8HOTK) -> /gello_1/joint_states
  - GELLO #2 (FTB8HP2K) -> /gello_2/joint_states

Supports switching to position control mode via a ROS 2 service, which enables
subscribing to commanded joint positions and holding the servos in place.

Usage (source /opt/ros/humble/setup.bash first):
    # Via launch file (recommended):
    ros2 launch ur_robotiq gello_offset_bimanual.launch.py left_gello_id:=1 right_gello_id:=2

    # Direct node execution:
    ros2 run ur_robotiq gello_publisher --ros-args -p gello_device:=1

Verify with:
    ros2 topic echo /gello_1/joint_states
    ros2 topic echo /gello_2/joint_states

Position control mode:
    ros2 service call /gello_1/set_position_control std_srvs/srv/SetBool "{data: true}"
    ros2 topic pub /gello_1/command_joints sensor_msgs/msg/JointState \
        "{position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.5]}"
    ros2 service call /gello_1/set_position_control std_srvs/srv/SetBool "{data: false}"
"""

import time
from threading import Event, Lock, Thread

import numpy as np
import rclpy
from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
POSITION_CONTROL_MODE = 3
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

GELLO_CONFIGS = {
    "1": {
        "name": "gello_1",
        "port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HOTK-if00-port0",
        "baudrate": 57600,
        "arm_joint_ids": [1, 2, 3, 4, 5, 6],
        "gripper_id": 7,
        "arm_joint_offsets": [
            3 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            6 * np.pi / 2,
            2 * np.pi / 2,
            3 * np.pi / 2,
        ],
        "arm_joint_signs": [1, 1, -1, 1, 1, 1],
        "gripper_open_deg": 190.0,
        "gripper_close_deg": 145.0,
    },
    "2": {
        "name": "gello_2",
        "port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HP2K-if00-port0",
        "baudrate": 57600,
        "arm_joint_ids": [1, 2, 3, 4, 5, 6],
        "gripper_id": 7,
        "arm_joint_offsets": [
            3 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ],
        "arm_joint_signs": [1, 1, -1, 1, 1, 1],
        "gripper_open_deg": 190.0,
        "gripper_close_deg": 145.0,
    },
}

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
]


def auto_detect_gello() -> dict:
    """Detect which GELLO is plugged in by checking serial ports.

    Returns:
        dict: The config for the detected GELLO.

    Raises:
        RuntimeError: If no GELLO or multiple GELLOs are found.
    """
    found = []
    for key, cfg in GELLO_CONFIGS.items():
        if os.path.exists(cfg["port"]):
            found.append((key, cfg))

    if len(found) == 0:
        raise RuntimeError(
            "No GELLO detected. Check USB connection.\n"
            f"Expected ports: {[c['port'] for c in GELLO_CONFIGS.values()]}"
        )
    if len(found) > 1:
        names = [f"--gello {k} ({c['name']})" for k, c in found]
        raise RuntimeError(
            f"Multiple GELLOs detected. Specify which one:\n  " + "\n  ".join(names)
        )

    return found[0][1]


class DynamixelInterface:
    """Dynamixel read/write interface for a GELLO device.

    Handles reading joint positions (with offsets, signs, gripper mapping,
    and exponential smoothing matching DynamixelRobot.get_joint_state())
    and writing goal positions for position control mode.

    Args:
        config: GELLO configuration dictionary.
    """

    def __init__(self, config: dict):
        self._port = config["port"]
        self._baudrate = config["baudrate"]

        arm_ids = config["arm_joint_ids"]
        gripper_id = config["gripper_id"]
        self._all_ids = arm_ids + [gripper_id]
        self._num_arm = len(arm_ids)

        self._joint_offsets = np.array(config["arm_joint_offsets"] + [0.0])
        self._joint_signs = np.array(config["arm_joint_signs"] + [1])

        self._gripper_open = config["gripper_open_deg"] * np.pi / 180.0
        self._gripper_close = config["gripper_close_deg"] * np.pi / 180.0

        self._alpha = 0.99
        self._last_pos = None
        self._torque_enabled = False

        self._port_handler = PortHandler(self._port)
        self._packet_handler = PacketHandler(2.0)

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open port {self._port}")
        if not self._port_handler.setBaudRate(self._baudrate):
            raise RuntimeError(f"Failed to set baudrate {self._baudrate}")

        self._sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._sync_write = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        for dxl_id in self._all_ids:
            if not self._sync_read.addParam(dxl_id):
                raise RuntimeError(f"Failed to add sync read param for servo {dxl_id}")

        self._lock = Lock()
        self._raw_positions = np.zeros(len(self._all_ids))
        self._running = Event()
        self._running.set()
        self._thread = Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        """Background thread that continuously reads servo positions."""
        while self._running.is_set():
            # Reason: lock must cover ALL serial communication to prevent collisions
            # with torque/mode writes happening from the main thread
            with self._lock:
                result = self._sync_read.txRxPacket()
                if result != COMM_SUCCESS:
                    time.sleep(0.01)
                    continue

                positions = []
                for dxl_id in self._all_ids:
                    raw = self._sync_read.getData(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    )
                    if raw > 2**31:
                        raw -= 2**32
                    positions.append(raw * np.pi / 2048.0)

                self._raw_positions = np.array(positions)

            time.sleep(0.005)

    def get_joint_state(self) -> np.ndarray:
        """Process joints identically to DynamixelRobot.get_joint_state().

        Returns:
            np.ndarray: 7 joint values. Joints 1-6 are arm angles in radians.
                Joint 7 is gripper position as a continuous float in [0, 1].
        """
        with self._lock:
            raw = self._raw_positions.copy()

        pos = (raw - self._joint_offsets) * self._joint_signs

        g_pos = (pos[-1] - self._gripper_open) / (
            self._gripper_close - self._gripper_open
        )
        pos[-1] = min(max(0.0, g_pos), 1.0)

        if self._last_pos is None:
            self._last_pos = pos.copy()
        else:
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos.copy()

        return pos

    def enable_position_control(self) -> str:
        """Switch servos to position control mode with torque enabled.

        Returns:
            str: Status message.
        """
        try:
            self._set_torque(False)
            self._set_operating_mode(POSITION_CONTROL_MODE)
            self._set_torque(True)
            self._torque_enabled = True
            return "Position control enabled"
        except RuntimeError as e:
            return f"Failed to enable position control: {e}"

    def disable_position_control(self) -> str:
        """Disable torque and return servos to passive read mode.

        Returns:
            str: Status message.
        """
        try:
            self._set_torque(False)
            self._torque_enabled = False
            return "Position control disabled, servos are passive"
        except RuntimeError as e:
            return f"Failed to disable position control: {e}"

    def command_joints(self, joint_positions: np.ndarray):
        """Send goal positions to all servos.

        Reverses the offset/sign math to convert from robot joint space
        back to raw servo angles.

        Args:
            joint_positions: 7 joint values (6 arm radians + 1 gripper [0,1]).
        """
        if not self._torque_enabled:
            return

        positions = joint_positions.copy()

        # Reason: reverse gripper [0,1] back to raw angle
        positions[-1] = (
            self._gripper_open
            + positions[-1] * (self._gripper_close - self._gripper_open)
        )

        # Reason: reverse offset/sign: raw = (pos / sign) + offset
        raw_angles = (positions / self._joint_signs) + self._joint_offsets

        with self._lock:
            for dxl_id, angle in zip(self._all_ids, raw_angles):
                position_value = int(angle * 2048.0 / np.pi)
                param = [
                    DXL_LOBYTE(DXL_LOWORD(position_value)),
                    DXL_HIBYTE(DXL_LOWORD(position_value)),
                    DXL_LOBYTE(DXL_HIWORD(position_value)),
                    DXL_HIBYTE(DXL_HIWORD(position_value)),
                ]
                self._sync_write.addParam(dxl_id, param)

            self._sync_write.txPacket()
            self._sync_write.clearParam()

    def _set_torque(self, enable: bool):
        """Enable or disable torque on all servos."""
        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._all_ids:
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, dxl_id, ADDR_TORQUE_ENABLE, value
                )
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f"Failed to set torque for servo {dxl_id} (comm={result}, err={error})"
                    )

    def _set_operating_mode(self, mode: int):
        """Set operating mode on all servos. Torque must be disabled first."""
        with self._lock:
            for dxl_id in self._all_ids:
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, dxl_id, ADDR_OPERATING_MODE, mode
                )
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f"Failed to set operating mode for servo {dxl_id}"
                    )

    def close(self):
        """Disable torque, stop reading thread, and close the port."""
        self._running.clear()
        self._thread.join(timeout=2.0)
        if self._torque_enabled:
            try:
                self._set_torque(False)
            except RuntimeError:
                pass
        self._port_handler.closePort()


class GelloJointPublisher(Node):
    """ROS 2 node that publishes GELLO joints and supports position control.

    Topics (namespaced by GELLO name):
        - /<name>/joint_states: Published joint states (50 Hz)
        - /<name>/command_joints: Subscribed for position commands (when enabled)

    Services:
        - /<name>/set_position_control: SetBool to toggle position control mode

    Args:
        config: GELLO hardware configuration dictionary.
        publish_rate: Publishing frequency in Hz.
    """

    def __init__(self, publish_rate: float = 50.0):
        super().__init__('gello_publisher')
        
        # Declare and read gello_device parameter
        self.declare_parameter('gello_device', 1)
        gello_id = str(self.get_parameter('gello_device').value)
        
        if gello_id not in GELLO_CONFIGS:
            raise ValueError(f"Invalid gello_device parameter: {gello_id}. Must be '1' or '2'.")
        
        config = GELLO_CONFIGS[gello_id].copy()
        
        # Allow port override from parameter
        self.declare_parameter('port', '')
        port_param = self.get_parameter('port').value
        if port_param:
            config['port'] = port_param
        
        self._gello_name = config["name"]

        self.get_logger().info(f"Connecting to {self._gello_name} on {config['port']}...")
        self._interface = DynamixelInterface(config)
        self.get_logger().info(f"{self._gello_name} connected successfully.")

        self._joint_pub = self.create_publisher(
            JointState, f"{self._gello_name}/joint_states", 10
        )

        self._cmd_sub = self.create_subscription(
            JointState,
            f"{self._gello_name}/command_joints",
            self._command_callback,
            10,
        )

        self._control_srv = self.create_service(
            SetBool,
            f"{self._gello_name}/set_position_control",
            self._set_position_control_callback,
        )

        self._timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f"Publishing 7 joints at {publish_rate} Hz on /{self._gello_name}/joint_states\n"
            f"  Position control service: /{self._gello_name}/set_position_control\n"
            f"  Command topic:            /{self._gello_name}/command_joints"
        )

    def _publish(self):
        """Timer callback that reads and publishes all 7 joint states."""
        joints = self._interface.get_joint_state()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = JOINT_NAMES
        msg.position = joints.tolist()

        self._joint_pub.publish(msg)

    def _command_callback(self, msg: JointState):
        """Handle incoming joint position commands.

        Args:
            msg: JointState with 7 position values to command.
        """
        if len(msg.position) != 7:
            self.get_logger().warn(
                f"Expected 7 joint positions, got {len(msg.position)}"
            )
            return
        self._interface.command_joints(np.array(msg.position))

    def _set_position_control_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle position control toggle service.

        Args:
            request: SetBool request. data=True enables, data=False disables.
            response: SetBool response with success flag and message.

        Returns:
            SetBool.Response: Result of the operation.
        """
        if request.data:
            result = self._interface.enable_position_control()
            response.success = "enabled" in result.lower()
        else:
            result = self._interface.disable_position_control()
            response.success = "disabled" in result.lower()

        response.message = result
        self.get_logger().info(result)
        return response

    def destroy_node(self):
        """Clean up Dynamixel connection on shutdown."""
        self._interface.close()
        super().destroy_node()


def main():
    """Entry point for the GELLO ROS 2 joint publisher."""
    rclpy.init()
    node = GelloJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
