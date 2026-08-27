#!/usr/bin/env python3
"""Headless ROS 2 node for recording multiple RealSense color streams to HDF5.

ROS parameters:
  serials     string array, required
  width       integer, default 640
  height      integer, default 480
  fps         integer, default 30
  output_file string, default ./dataset.h5
  compression string, one of: lzf, gzip, none; default lzf

Services:
  /start_recording  std_srvs/srv/Trigger
  /pause_recording  std_srvs/srv/Trigger
  /resume_recording std_srvs/srv/Trigger
  /stop_recording   std_srvs/srv/Trigger
  /discard_last_recording std_srvs/srv/Trigger

The node keeps one HDF5 dataset file. Each start/stop cycle creates one new demo
under /demos/demo_NNNNNN. Cameras stream continuously, but frames are written only
between start and stop service calls.
"""

import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

import h5py
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RealSenseHdf5Recorder(Node):
    def __init__(self) -> None:
        super().__init__('realsense_hdf5_recorder')

        self.declare_parameter('serials', [''])
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('output_file', './dataset.h5')
        self.declare_parameter('compression', 'lzf')

        self.serials: List[str] = [
            str(x) for x in self.get_parameter('serials').value
        ]
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.output_file = Path(
            os.path.expanduser(str(self.get_parameter('output_file').value))
        ).resolve()
        compression_param = str(
            self.get_parameter('compression').value).lower()
        self.compression: Optional[str] = (
            None if compression_param in ('', 'none') else compression_param
        )

        if not self.serials:
            raise RuntimeError(
                "Parameter 'serials' must contain at least one serial")
        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise RuntimeError('width, height, and fps must be positive')
        if self.compression not in (None, 'lzf', 'gzip'):
            raise RuntimeError("compression must be 'lzf', 'gzip', or 'none'")
        if len(set(self.serials)) != len(self.serials):
            raise RuntimeError('serials contains duplicates')

        self.output_file.parent.mkdir(parents=True, exist_ok=True)

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._recording = False
        self._paused = False
        self._file: Optional[h5py.File] = None
        self._datasets: Dict[str, Dict[str, h5py.Dataset]] = {}
        self._demo_name: Optional[str] = None

        self._pipelines: Dict[str, rs.pipeline] = {}
        self._start_cameras()

        self.create_service(Trigger, 'start_recording', self._on_start)
        self.create_service(Trigger, 'pause_recording', self._on_pause)
        self.create_service(Trigger, 'resume_recording', self._on_resume)
        self.create_service(Trigger, 'stop_recording', self._on_stop)
        self.create_service(
            Trigger, 'discard_last_recording', self._on_discard_last)

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name='realsense_hdf5_capture',
            daemon=True,
        )
        self._capture_thread.start()

        self.get_logger().info(
            f'Ready: {len(self.serials)} camera(s), '
            f'{self.width}x{self.height}@{self.fps}, output={self.output_file}'
        )

    def _start_cameras(self) -> None:
        available = {
            device.get_info(rs.camera_info.serial_number)
            for device in rs.context().query_devices()
        }
        missing = [serial for serial in self.serials if serial not in available]
        if missing:
            raise RuntimeError(f'RealSense serial(s) not found: {missing}')

        try:
            for serial in self.serials:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(serial)
                config.enable_stream(
                    rs.stream.color,
                    self.width,
                    self.height,
                    rs.format.rgb8,
                    self.fps,
                )
                pipeline.start(config)
                self._pipelines[serial] = pipeline
                self.get_logger().info(f'Started camera {serial}')

            # Discard initial auto-exposure frames.
            for _ in range(10):
                for pipeline in self._pipelines.values():
                    pipeline.wait_for_frames(2000)
        except Exception:
            self._stop_cameras()
            raise

    def _open_dataset_file(self) -> h5py.File:
        h5 = h5py.File(self.output_file, 'a')
        if 'format_version' not in h5.attrs:
            h5.attrs['format_version'] = 2
            h5.attrs['created_utc'] = datetime.now(timezone.utc).isoformat()
            h5.attrs['width'] = self.width
            h5.attrs['height'] = self.height
            h5.attrs['fps'] = self.fps
            h5.attrs['color_format'] = 'rgb8'
            h5.attrs['serials'] = np.asarray(
                self.serials, dtype=h5py.string_dtype()
            )
            demos_group = h5.require_group('demos')
            demos_group.attrs['completed_count'] = 0
            demos_group.attrs['discarded_count'] = 0
            demos_group.attrs['total_count'] = 0
            h5.flush()
        else:
            expected = {
                'width': self.width,
                'height': self.height,
                'fps': self.fps,
            }
            for key, value in expected.items():
                if int(h5.attrs[key]) != value:
                    h5.close()
                    raise RuntimeError(
                        f"Existing dataset has {key}={int(h5.attrs[key])}, "
                        f"requested {value}"
                    )
            stored_serials = [
                x.decode() if isinstance(x, bytes) else str(x)
                for x in h5.attrs['serials']
            ]
            if stored_serials != self.serials:
                h5.close()
                raise RuntimeError(
                    f'Existing dataset serials={stored_serials}, '
                    f'requested={self.serials}'
                )
            demos_group = h5.require_group('demos')

            # Keep aggregate demo counters on /demos, not on the file root.
            # Migrate counters produced by older recorder versions if present.
            completed = int(demos_group.attrs.get(
                'completed_count', h5.attrs.get('demo_saved_count', 0)
            ))
            discarded = int(demos_group.attrs.get(
                'discarded_count', h5.attrs.get('demo_discarded_count', 0)
            ))
            demos_group.attrs.setdefault('completed_count', completed)
            demos_group.attrs.setdefault('discarded_count', discarded)
            demos_group.attrs.setdefault(
                'total_count', completed + discarded
            )
            for old_key in (
                'demo_saved_count',
                'demo_discarded_count',
                'demo_total_count',
            ):
                if old_key in h5.attrs:
                    del h5.attrs[old_key]
            h5.flush()
        return h5

    @staticmethod
    def _sync_demo_metadata(demos_group: h5py.Group) -> None:
        demos_group.attrs['total_count'] = int(
            demos_group.attrs.get('completed_count', 0)
        ) + int(demos_group.attrs.get('discarded_count', 0))

    @staticmethod
    def _next_demo_name(demos_group: h5py.Group) -> str:
        indices = []
        for name in demos_group.keys():
            if name.startswith('demo_'):
                try:
                    indices.append(int(name[5:]))
                except ValueError:
                    pass
        return f'demo_{max(indices, default=-1) + 1:06d}'

    @staticmethod
    def _last_demo_name(demos_group: h5py.Group) -> Optional[str]:
        indices = []
        for name in demos_group.keys():
            if name.startswith('demo_'):
                try:
                    indices.append(int(name[5:]))
                except ValueError:
                    pass
        if not indices:
            return None
        return f'demo_{max(indices):06d}'

    def _create_demo(self) -> str:
        h5 = self._open_dataset_file()
        try:
            demos_group = h5['demos']
            demo_name = self._next_demo_name(demos_group)
            demo_group = demos_group.create_group(demo_name)
            demo_group.attrs['created_utc'] = datetime.now(
                timezone.utc).isoformat()
            demo_group.attrs['complete'] = False

            datasets: Dict[str, Dict[str, h5py.Dataset]] = {}
            cameras_group = demo_group.create_group('cameras')
            for serial in self.serials:
                group = cameras_group.create_group(serial)
                group.attrs['serial'] = serial
                datasets[serial] = {
                    'rgb': group.create_dataset(
                        'rgb',
                        shape=(0, self.height, self.width, 3),
                        maxshape=(None, self.height, self.width, 3),
                        dtype=np.uint8,
                        chunks=(1, self.height, self.width, 3),
                        compression=self.compression,
                    ),
                    'host_timestamp_ns': group.create_dataset(
                        'host_timestamp_ns', shape=(0,), maxshape=(None,),
                        dtype=np.int64, chunks=True,
                    ),
                    'device_timestamp_ms': group.create_dataset(
                        'device_timestamp_ms', shape=(0,), maxshape=(None,),
                        dtype=np.float64, chunks=True,
                    ),
                    'frame_number': group.create_dataset(
                        'frame_number', shape=(0,), maxshape=(None,),
                        dtype=np.int64, chunks=True,
                    ),
                }

            h5.flush()
            self._file = h5
            self._datasets = datasets
            self._demo_name = demo_name
            self._recording = True
            self._paused = False
            return demo_name
        except Exception:
            h5.close()
            raise

    @staticmethod
    def _append(dataset: h5py.Dataset, value) -> None:
        index = dataset.shape[0]
        dataset.resize(index + 1, axis=0)
        dataset[index] = value

    def _write_frame(
        self,
        serial: str,
        rgb: np.ndarray,
        host_timestamp_ns: int,
        device_timestamp_ms: float,
        frame_number: int,
    ) -> None:
        datasets = self._datasets[serial]
        self._append(datasets['rgb'], rgb)
        self._append(datasets['host_timestamp_ns'], host_timestamp_ns)
        self._append(datasets['device_timestamp_ms'], device_timestamp_ms)
        self._append(datasets['frame_number'], frame_number)

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            for serial, pipeline in self._pipelines.items():
                if self._stop_event.is_set():
                    break
                try:
                    frames = pipeline.wait_for_frames(1000)
                    color = frames.get_color_frame()
                    if not color:
                        continue

                    # Copy frame memory before the RealSense frame is released.
                    rgb = np.asanyarray(color.get_data()).copy()
                    host_ns = time.time_ns()
                    device_ms = float(color.get_timestamp())
                    frame_number = int(color.get_frame_number())

                    # HDF5 access and start/stop transitions are serialized here.
                    with self._lock:
                        if self._recording and not self._paused and self._file is not None:
                            self._write_frame(
                                serial,
                                rgb,
                                host_ns,
                                device_ms,
                                frame_number,
                            )
                except RuntimeError as exc:
                    if not self._stop_event.is_set():
                        self.get_logger().warning(
                            f'Frame wait failed for camera {serial}: {exc}'
                        )
                except Exception as exc:
                    self.get_logger().error(
                        f'Capture/write error for camera {serial}: {exc}'
                    )
                    time.sleep(0.1)

    def _on_start(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._lock:
            if self._recording:
                response.success = False
                response.message = f'Already recording: {self._demo_name}'
                return response
            try:
                demo_name = self._create_demo()
                response.success = True
                response.message = f'{self.output_file}:/demos/{demo_name}'
                self.get_logger().info(
                    f'Recording started: {response.message}')
            except Exception as exc:
                self._close_demo()
                response.success = False
                response.message = f'Could not start recording: {exc}'
                self.get_logger().error(response.message)
        return response

    def _on_pause(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = 'Not recording'
                return response
            if self._paused:
                response.success = False
                response.message = f'Already paused: {self._demo_name}'
                return response

            self._paused = True
            response.success = True
            response.message = f'Paused recording: {self._demo_name}'
            self.get_logger().info(response.message)
        return response

    def _on_resume(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = 'Not recording'
                return response
            if not self._paused:
                response.success = False
                response.message = f'Not paused: {self._demo_name}'
                return response

            self._paused = False
            response.success = True
            response.message = f'Resumed recording: {self._demo_name}'
            self.get_logger().info(response.message)
        return response

    def _on_stop(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = 'Not recording'
                return response

            demo_name = self._demo_name
            counts = {
                serial: int(items['rgb'].shape[0])
                for serial, items in self._datasets.items()
            }
            self._recording = False
            try:
                assert self._file is not None and demo_name is not None
                demo_group = self._file[f'demos/{demo_name}']
                demo_group.attrs['closed_utc'] = datetime.now(
                    timezone.utc
                ).isoformat()
                demo_group.attrs['complete'] = True
                demos_group = self._file['demos']
                demos_group.attrs['completed_count'] = int(
                    demos_group.attrs.get('completed_count', 0)
                ) + 1
                self._sync_demo_metadata(demos_group)
                self._file.flush()
                self._close_demo()
                response.success = True
                response.message = (
                    f'Saved {self.output_file}:/demos/{demo_name}; frames={counts}'
                )
                self.get_logger().info(response.message)
            except Exception as exc:
                self._close_demo()
                response.success = False
                response.message = f'Error while closing {demo_name}: {exc}'
                self.get_logger().error(response.message)
        return response

    def _on_discard_last(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._lock:
            if self._recording:
                response.success = False
                response.message = 'Stop recording before discarding the last demo'
                return response

            h5 = self._open_dataset_file()
            try:
                demos_group = h5['demos']
                last_demo_name = self._last_demo_name(demos_group)
                if last_demo_name is None:
                    response.success = False
                    response.message = 'No recorded demos to discard'
                    return response

                del demos_group[last_demo_name]

                # A stopped demo was previously counted as completed. Discarding
                # reclassifies it, rather than counting the same attempt twice.
                completed = int(demos_group.attrs.get('completed_count', 0))
                if completed > 0:
                    demos_group.attrs['completed_count'] = completed - 1
                demos_group.attrs['discarded_count'] = int(
                    demos_group.attrs.get('discarded_count', 0)
                ) + 1
                self._sync_demo_metadata(demos_group)
                h5.flush()
                response.success = True
                response.message = (
                    f'Discarded {self.output_file}:/demos/{last_demo_name}'
                )
                self.get_logger().info(response.message)
            finally:
                h5.close()
        return response

    def _close_demo(self) -> None:
        self._recording = False
        self._paused = False
        if self._file is not None:
            try:
                self._file.close()
            except Exception:
                pass
        self._file = None
        self._datasets = {}
        self._demo_name = None

    def _stop_cameras(self) -> None:
        for serial, pipeline in list(self._pipelines.items()):
            try:
                pipeline.stop()
            except Exception as exc:
                self.get_logger().warning(
                    f'Could not stop camera {serial}: {exc}')
        self._pipelines.clear()

    def close(self) -> None:
        self._stop_event.set()
        if hasattr(self, '_capture_thread'):
            self._capture_thread.join(timeout=3.0)
        with self._lock:
            if self._recording and self._file is not None:
                # Interrupted demos remain readable but are explicitly incomplete.
                if self._demo_name is not None:
                    demo_group = self._file[f'demos/{self._demo_name}']
                    demo_group.attrs['complete'] = False
                    demo_group.attrs['interrupted_utc'] = datetime.now(
                        timezone.utc
                    ).isoformat()
                self._file.flush()
            self._close_demo()
        self._stop_cameras()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[RealSenseHdf5Recorder] = None
    try:
        node = RealSenseHdf5Recorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()