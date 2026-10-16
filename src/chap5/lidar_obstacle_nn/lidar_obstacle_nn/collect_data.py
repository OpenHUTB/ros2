"""Record /scan data from Gazebo and label each scan automatically.

The labels are derived from the minimum range of the scan, so we can
collect a labelled dataset without any manual annotation:

    min range < danger_distance  -> 0 (near)
    danger_distance <= min range < far_distance -> 1 (far)
    min range >= far_distance     -> 2 (free)

Output: a single .npz file containing:
    scans : float32 array of shape (N, scan_size)
    labels: int64 array of shape (N,)

Usage:
    ros2 run lidar_obstacle_nn collect_data --ros-args \\
        -p output_path:=/tmp/lidar_dataset.npz \\
        -p num_samples:=1000
"""

from __future__ import annotations

import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

DEFAULT_SCAN_SIZE = 360
DEFAULT_DANGER_DISTANCE = 1.0
DEFAULT_FAR_DISTANCE = 3.0
DEFAULT_NUM_SAMPLES = 1000


class DataCollectorNode(Node):
    """Subscribe to /scan, label, accumulate, dump to .npz."""

    def __init__(self) -> None:
        super().__init__('lidar_data_collector')

        self.declare_parameter('output_path', '/tmp/lidar_dataset.npz')
        self.declare_parameter('scan_size', DEFAULT_SCAN_SIZE)
        self.declare_parameter('danger_distance', DEFAULT_DANGER_DISTANCE)
        self.declare_parameter('far_distance', DEFAULT_FAR_DISTANCE)
        self.declare_parameter('num_samples', DEFAULT_NUM_SAMPLES)

        self._output_path = (
            self.get_parameter('output_path').get_parameter_value().string_value
        )
        self._scan_size = int(
            self.get_parameter('scan_size').get_parameter_value().integer_value
        )
        self._danger_distance = float(
            self.get_parameter('danger_distance').get_parameter_value().double_value
        )
        self._far_distance = float(
            self.get_parameter('far_distance').get_parameter_value().double_value
        )
        self._num_samples = int(
            self.get_parameter('num_samples').get_parameter_value().integer_value
        )

        self._scans: list[np.ndarray] = []
        self._labels: list[int] = []

        scan_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)

        self.get_logger().info(
            'Collecting %d samples (scan_size=%d, near<%.2f, free>=%.2f) -> %s'
            % (
                self._num_samples,
                self._scan_size,
                self._danger_distance,
                self._far_distance,
                self._output_path,
            )
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        if len(self._scans) >= self._num_samples:
            return

        raw = np.asarray(msg.ranges, dtype=np.float32)
        if raw.size != self._scan_size:
            idx = np.linspace(0, raw.size - 1, self._scan_size)
            raw = np.interp(idx, np.arange(raw.size), raw).astype(np.float32)

        # Replace inf / nan with the maximum range so the NN sees a
        # well-conditioned vector (consistent with inference_node.py).
        raw = np.nan_to_num(raw, nan=30.0, posinf=30.0, neginf=0.0)
        raw = np.clip(raw, 0.0, 30.0)

        min_range = float(raw.min())
        if min_range < self._danger_distance:
            label = 0
        elif min_range >= self._far_distance:
            label = 2
        else:
            label = 1

        self._scans.append(raw)
        self._labels.append(label)

        if (len(self._scans) % 50) == 0:
            self.get_logger().info(
                'Collected %d / %d samples (last min_range=%.2f, label=%d)'
                % (len(self._scans), self._num_samples, min_range, label)
            )

        if len(self._scans) >= self._num_samples:
            self._dump_and_shutdown()

    def _dump_and_shutdown(self) -> None:
        scans = np.stack(self._scans, axis=0).astype(np.float32)
        labels = np.asarray(self._labels, dtype=np.int64)

        os.makedirs(os.path.dirname(self._output_path) or '.', exist_ok=True)
        np.savez(
            self._output_path,
            scans=scans,
            labels=labels,
            scan_size=self._scan_size,
            danger_distance=self._danger_distance,
            far_distance=self._far_distance,
            class_names=np.asarray(['near', 'far', 'free']),
        )

        unique, counts = np.unique(labels, return_counts=True)
        dist = ', '.join(
            '%s=%d' % (name, int(cnt))
            for name, cnt in zip(['near', 'far', 'free'], counts)
        )
        self.get_logger().info(
            'Saved dataset: %s (shape=%s, %s)'
            % (self._output_path, scans.shape, dist)
        )

        # Trigger clean shutdown once we are done collecting.
        raise SystemExit(0)


def main(args=None) -> None:
    """Entry point for `ros2 run lidar_obstacle_nn collect_data`."""
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
