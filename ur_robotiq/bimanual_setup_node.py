from typing import Optional
import time

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import GetLoadedProgram
from ur_dashboard_msgs.srv import IsInRemoteControl
from ur_dashboard_msgs.srv import IsProgramRunning
from ur_dashboard_msgs.srv import Load


class BimanualSetupNode(Node):
    def __init__(self) -> None:
        super().__init__('bimanual_ur_setup')

        self.declare_parameter('left_namespace', 'left_ur')
        self.declare_parameter('right_namespace', 'right_ur')
        self.declare_parameter('dashboard_namespace', 'dashboard_client')

        self.declare_parameter('left_program', 'ExternalControl.urp')
        self.declare_parameter('right_program', 'ExternalControl.urp')

        self.declare_parameter('require_remote_control', False)
        self.declare_parameter('skip_if_program_running', True)
        self.declare_parameter('wait_for_service_timeout', 20.0)
        self.declare_parameter('service_call_timeout', 20.0)
        self.declare_parameter('post_power_on_wait', 2.0)
        self.declare_parameter('post_brake_release_wait', 2.0)
        self.declare_parameter('post_stop_wait', 5.0)

        self.declare_parameter('power_on_service', 'power_on')
        self.declare_parameter('brake_release_service', 'brake_release')
        self.declare_parameter('play_service', 'play')
        self.declare_parameter('stop_service', 'stop')
        self.declare_parameter('load_program_service', 'load_program')
        self.declare_parameter('program_running_service', 'program_running')
        self.declare_parameter('get_loaded_program_service', 'get_loaded_program')
        self.declare_parameter('remote_control_service', 'is_in_remote_control')

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _norm_namespace(namespace: str) -> str:
        return namespace.strip('/')

    def _service_name(self, robot_namespace: str, service_leaf: str) -> str:
        robot_ns = self._norm_namespace(robot_namespace)
        dashboard_ns = self._norm_namespace(self._param_str('dashboard_namespace'))
        return f'/{robot_ns}/{dashboard_ns}/{service_leaf}'

    def _wait_for_service(self, client, service_name: str) -> bool:
        timeout = self._param_float('wait_for_service_timeout')
        if client.wait_for_service(timeout_sec=timeout):
            return True
        self.get_logger().error(f'Service unavailable after {timeout:.1f}s: {service_name}')
        return False

    def _call_trigger(self, robot_namespace: str, service_leaf: str, action_name: str) -> bool:
        service_name = self._service_name(robot_namespace, service_leaf)
        client = self.create_client(Trigger, service_name)

        if not self._wait_for_service(client, service_name):
            return False

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._param_float('service_call_timeout'))

        if not future.done() or future.result() is None:
            self.get_logger().error(f'{action_name} failed (timeout/no response): {service_name}')
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(f'{action_name} failed: {response.message}')
            return False

        self.get_logger().info(f'{action_name} succeeded: {response.message}')
        return True

    def _is_remote_control(self, robot_namespace: str) -> Optional[bool]:
        service_name = self._service_name(robot_namespace, self._param_str('remote_control_service'))
        client = self.create_client(IsInRemoteControl, service_name)

        if not self._wait_for_service(client, service_name):
            return None

        request = IsInRemoteControl.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._param_float('service_call_timeout'))

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Remote control check failed (timeout/no response): {service_name}')
            return None

        response = future.result()
        if not response.success:
            self.get_logger().error(f'Remote control check failed: {response.answer}')
            return None

        return response.remote_control

    def _is_program_running(self, robot_namespace: str) -> Optional[bool]:
        service_name = self._service_name(robot_namespace, self._param_str('program_running_service'))
        client = self.create_client(IsProgramRunning, service_name)

        if not self._wait_for_service(client, service_name):
            return None

        request = IsProgramRunning.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._param_float('service_call_timeout'))

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Program running check failed (timeout/no response): {service_name}')
            return None

        response = future.result()
        if not response.success:
            self.get_logger().error(f'Program running check failed: {response.answer}')
            return None

        return response.program_running

    def _get_loaded_program(self, robot_namespace: str) -> Optional[str]:
        service_name = self._service_name(robot_namespace, self._param_str('get_loaded_program_service'))
        client = self.create_client(GetLoadedProgram, service_name)

        if not self._wait_for_service(client, service_name):
            return None

        request = GetLoadedProgram.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._param_float('service_call_timeout'))

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Loaded program check failed (timeout/no response): {service_name}')
            return None

        response = future.result()
        if not response.success:
            self.get_logger().error(f'Loaded program check failed: {response.answer}')
            return None

        return response.program_name

    def _load_program(self, robot_namespace: str, program_path: str) -> bool:
        service_name = self._service_name(robot_namespace, self._param_str('load_program_service'))
        client = self.create_client(Load, service_name)

        if not self._wait_for_service(client, service_name):
            return False

        request = Load.Request()
        request.filename = program_path

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._param_float('service_call_timeout'))

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Load program failed (timeout/no response): {service_name}')
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(f'Load program failed: {response.answer}')
            return False

        self.get_logger().info(f'Program loaded: {response.answer}')
        return True

    def _stop_program_first(self, label: str, robot_namespace: str) -> bool:
        self.get_logger().info(f'[{label}] issuing stop as first setup step')
        if not self._call_trigger(robot_namespace, self._param_str('stop_service'), f'[{label}] stop'):
            running_after_failed_stop = self._is_program_running(robot_namespace)
            if running_after_failed_stop is None:
                return False
            if running_after_failed_stop:
                self.get_logger().error(f'[{label}] stop failed and program is still running')
                return False
            self.get_logger().warning(
                f'[{label}] stop returned failure but program is already not running, continuing setup'
            )

        time.sleep(self._param_float('post_stop_wait'))
        return True

    def _setup_single_robot(self, label: str, robot_namespace: str, program_path: str) -> bool:
        self.get_logger().info(f'[{label}] setup started for namespace /{self._norm_namespace(robot_namespace)}')

        running_before_stop = self._is_program_running(robot_namespace)
        if running_before_stop is None:
            return False

        if not self._stop_program_first(label, robot_namespace):
            return False

        require_remote_control = self._param_bool('require_remote_control')
        if require_remote_control:
            remote_control = self._is_remote_control(robot_namespace)
            if remote_control is None:
                return False
            if not remote_control:
                self.get_logger().error(f'[{label}] robot is not in remote control mode')
                return False

        if self._param_bool('skip_if_program_running'):
            if running_before_stop:
                self.get_logger().info(f'[{label}] program already running, restarting with stop/play')
                if not self._call_trigger(robot_namespace, self._param_str('play_service'), f'[{label}] play'):
                    return False

                running_after_restart = self._is_program_running(robot_namespace)
                if running_after_restart is None:
                    return False
                if not running_after_restart:
                    self.get_logger().error(f'[{label}] play returned success but program is not running')
                    return False

                self.get_logger().info(f'[{label}] program restart completed successfully')
                return True

        if not self._call_trigger(robot_namespace, self._param_str('power_on_service'), f'[{label}] power_on'):
            return False
        time.sleep(self._param_float('post_power_on_wait'))

        if not self._call_trigger(robot_namespace, self._param_str('brake_release_service'), f'[{label}] brake_release'):
            return False
        time.sleep(self._param_float('post_brake_release_wait'))

        if program_path:
            loaded_program = self._get_loaded_program(robot_namespace)
            if loaded_program is None:
                return False

            if loaded_program != program_path:
                self.get_logger().info(
                    f'[{label}] loading program. current={loaded_program!r}, target={program_path!r}'
                )
                if not self._load_program(robot_namespace, program_path):
                    return False
            else:
                self.get_logger().info(f'[{label}] target program already loaded: {program_path}')

        if not self._call_trigger(robot_namespace, self._param_str('play_service'), f'[{label}] play'):
            return False

        running_after_play = self._is_program_running(robot_namespace)
        if running_after_play is None:
            return False
        if not running_after_play:
            self.get_logger().error(f'[{label}] play returned success but program is not running')
            return False

        self.get_logger().info(f'[{label}] setup completed successfully')
        return True

    def run(self) -> int:
        left_namespace = self._param_str('left_namespace')
        right_namespace = self._param_str('right_namespace')
        left_program = self._param_str('left_program')
        right_program = self._param_str('right_program')

        left_ok = self._setup_single_robot('left', left_namespace, left_program)
        right_ok = self._setup_single_robot('right', right_namespace, right_program)

        if left_ok and right_ok:
            self.get_logger().info('Bimanual setup completed for both robots')
            return 0

        self.get_logger().error('Bimanual setup failed')
        return 1


def main() -> None:
    rclpy.init()
    node = BimanualSetupNode()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
