"""Record /scan data from Gazebo and label each scan automatically.

The labels are derived from the minimum range of the scan, so we can
collect a labelled dataset without any manual annotation:

    min range < danger_distance  -> 0 (near)
    danger_distance <= min range < far_distance -> 1 (far)
    min range >= far_distance     -> 2 (free)

Two collection modes
--------------------

Static mode (default): the original behaviour -- record ``num_samples``
scans from whichever world is running and shut down. The static world
keeps a ~0.6 m obstacle next to the lidar, so this mode yields only
the ``near`` class; use the synthetic generator for the other classes.

Motion mode (``motion_enabled:=true``, requires
``obstacle_world_dynamic.sdf``): the node publishes the slide joint
position on /obstacle_slider/cmd_pos and drives the single obstacle
through a sequence of stations, two distances per class by default
(surface at 0.4/0.9 m -> near, 1.2/2.5 m -> far, 3.5/4.2 m -> free),
collecting ``per_station`` labelled scans at each one. Only scans whose
derived label matches the current station are accepted, so the dataset
is balanced by class and contains real within-class distance
variation. Stations come from the ``joint_positions`` /
``station_labels`` parameter arrays and can be customised.

Output: a single .npz file containing:
    scans : float32 array of shape (N, scan_size)
    labels: int64 array of shape (N,)

Usage:
    # static world
    ros2 run lidar_obstacle_nn collect_data --ros-args \\
        -p output_path:=/tmp/lidar_dataset.npz -p num_samples:=1000

    # dynamic world (start sim first: sim.launch.py world:=dynamic)
    ros2 run lidar_obstacle_nn collect_data --ros-args \\
        -p motion_enabled:=true -p per_station:=150 \\
        -p output_path:=/tmp/lidar_real.npz
"""

from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

DEFAULT_SCAN_SIZE = 360
DEFAULT_DANGER_DISTANCE = 1.0
DEFAULT_FAR_DISTANCE = 3.0
DEFAULT_NUM_SAMPLES = 1000
DEFAULT_PER_STATION = 150
DEFAULT_MAX_RANGE = 30.0

# Default motion-mode stations (joint position, class label).
# Link pose x is 0.6 m at joint 0; the box is 0.4 m deep (half 0.2),
# so surface distance = 0.6 + joint - 0.2 = joint + 0.4.
#   joint 0.0 / 0.5 -> surface 0.4 / 0.9 m -> near  (< 1.0)
#   joint 0.8 / 2.1 -> surface 1.2 / 2.5 m -> far
#   joint 3.1 / 3.6 -> surface 3.5 / 4.2 m -> free (>= 3.0)
DEFAULT_JOINT_POSITIONS = [0.0, 0.5, 0.8, 2.1, 3.1, 3.6]
DEFAULT_STATION_LABELS = [0, 0, 1, 1, 2, 2]


class DataCollectorNode(Node):
    """Subscribe to /scan, label, accumulate, dump to .npz."""

    def __init__(self) -> None:
        super().__init__('lidar_data_collector')

        self.declare_parameter('output_path', '/tmp/lidar_dataset.npz')
        self.declare_parameter('scan_size', DEFAULT_SCAN_SIZE)
        self.declare_parameter('danger_distance', DEFAULT_DANGER_DISTANCE)
        self.declare_parameter('far_distance', DEFAULT_FAR_DISTANCE)
        self.declare_parameter('num_samples', DEFAULT_NUM_SAMPLES)

        # Motion-mode parameters (ignored when motion_enabled is false).
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('cmd_topic', '/obstacle_slider/cmd_pos')
        self.declare_parameter('per_station', DEFAULT_PER_STATION)
        self.declare_parameter('settle_sec', 4.0)
        self.declare_parameter('station_tolerance', 0.15)
        self.declare_parameter('surface_offset', 0.4)
        self.declare_parameter('cmd_pub_hz', 10.0)
        self.declare_parameter('joint_positions', DEFAULT_JOINT_POSITIONS)
        self.declare_parameter('station_labels', DEFAULT_STATION_LABELS)

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

        # --- Motion mode: slide command publisher + phase state ------
        self._motion_enabled = bool(
            self.get_parameter('motion_enabled').get_parameter_value().bool_value
        )
        if self._motion_enabled:
            self._per_station = int(
                self.get_parameter('per_station').get_parameter_value().integer_value
            )
            self._settle_sec = float(
                self.get_parameter('settle_sec').get_parameter_value().double_value
            )
            # Expected surface distance = joint position + surface_offset,
            # where surface_offset = link x offset (0.6) - box half depth
            # (0.2) = 0.4 m in obstacle_world_dynamic.sdf.
            self._station_tolerance = float(
                self.get_parameter('station_tolerance')
                .get_parameter_value().double_value
            )
            surface_offset = float(
                self.get_parameter('surface_offset')
                .get_parameter_value().double_value
            )
            joints = list(
                self.get_parameter('joint_positions')
                .get_parameter_value().double_array_value
            )
            labels = list(
                self.get_parameter('station_labels')
                .get_parameter_value().integer_array_value
            )
            if len(joints) != len(labels) or not joints:
                raise ValueError(
                    'joint_positions and station_labels must be non-empty '
                    'and have the same length (got %d joints, %d labels)'
                    % (len(joints), len(labels))
                )
            names = ['near', 'far', 'free']
            self._phases = [
                (float(j), int(l), names[int(l)], float(j) + surface_offset)
                for j, l in zip(joints, labels)
            ]
            self._phase_idx = 0
            self._accepted_in_phase = 0
            self._skipped_in_phase = 0
            self._phase_t0 = time.monotonic()
            self._last_cmd = None

            cmd_topic = (
                self.get_parameter('cmd_topic').get_parameter_value().string_value
            )
            cmd_hz = float(
                self.get_parameter('cmd_pub_hz').get_parameter_value().double_value
            )
            self._cmd_pub = self.create_publisher(Float64, cmd_topic, 10)
            period = 1.0 / max(cmd_hz, 0.1)
            self.create_timer(period, self._publish_cmd)

            self.get_logger().info(
                'Motion mode: %d samples x %d stations via %s -> %s'
                % (self._per_station, len(self._phases), cmd_topic,
                   self._output_path)
            )
        else:
            self._cmd_pub = None
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

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _preprocess(self, ranges) -> np.ndarray:
        """Resample to scan_size and normalise inf/nan to max range."""
        raw = np.asarray(ranges, dtype=np.float32)
        if raw.size != self._scan_size:
            idx = np.linspace(0, raw.size - 1, self._scan_size)
            raw = np.interp(idx, np.arange(raw.size), raw).astype(np.float32)
        # Replace inf / nan with the maximum range so the NN sees a
        # well-conditioned vector (consistent with inference_node.py).
        raw = np.nan_to_num(raw, nan=DEFAULT_MAX_RANGE, posinf=DEFAULT_MAX_RANGE,
                            neginf=0.0)
        return np.clip(raw, 0.0, DEFAULT_MAX_RANGE)

    def _label_for(self, min_range: float) -> int:
        if min_range < self._danger_distance:
            return 0
        if min_range >= self._far_distance:
            return 2
        return 1

    # ------------------------------------------------------------------
    # Motion mode
    # ------------------------------------------------------------------
    def _station_name(self) -> str:
        joint, _, name, expected = self._phases[self._phase_idx]
        return 'station%d %s(joint=%.1f, surface=%.1f)' % (
            self._phase_idx + 1, name, joint, expected)

    def _publish_cmd(self) -> None:
        if not self._motion_enabled or self._phase_idx >= len(self._phases):
            return
        joint, _, _, _ = self._phases[self._phase_idx]
        if self._last_cmd != joint:
            self.get_logger().info(
                '%s: sending joint position %.2f, waiting %.1f s to settle'
                % (self._station_name(), joint, self._settle_sec)
            )
            self._last_cmd = joint
        msg = Float64()
        msg.data = joint
        self._cmd_pub.publish(msg)

    def _advance_phase(self) -> None:
        self._phase_idx += 1
        self._accepted_in_phase = 0
        self._skipped_in_phase = 0
        self._phase_t0 = time.monotonic()
        self._last_cmd = None

    # ------------------------------------------------------------------
    # Scan callback
    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan) -> None:
        raw = self._preprocess(msg.ranges)
        label = self._label_for(float(raw.min()))

        if self._motion_enabled:
            if self._phase_idx >= len(self._phases):
                return
            _, expected, _, expected_dist = self._phases[self._phase_idx]
            min_range = float(raw.min())
            elapsed = time.monotonic() - self._phase_t0
            # Wait out the slide motion; ignore scans from the old station
            # and enforce a per-station distance band so a passing-through
            # scan (right label, wrong distance) cannot contaminate data.
            at_station = (
                label == expected
                and abs(min_range - expected_dist) <= self._station_tolerance
            )
            if elapsed < self._settle_sec or not at_station:
                self._skipped_in_phase += 1
                if self._skipped_in_phase % 50 == 0:
                    self.get_logger().warn(
                        '%s: %d scans skipped (elapsed=%.1fs, '
                        'min_range=%.2f, label=%d) -- still moving or '
                        'wrong world?'
                        % (self._station_name(), self._skipped_in_phase,
                           elapsed, min_range, label)
                    )
                return
            self._scans.append(raw)
            self._labels.append(label)
            self._accepted_in_phase += 1

            if self._accepted_in_phase % 50 == 0:
                self.get_logger().info(
                    '%s: collected %d / %d (min_range=%.2f)'
                    % (self._station_name(), self._accepted_in_phase,
                       self._per_station, min_range)
                )

            if self._accepted_in_phase >= self._per_station:
                self.get_logger().info('%s complete.' % self._station_name())
                self._advance_phase()
                if self._phase_idx >= len(self._phases):
                    self._dump_and_shutdown()
            return

        # Static mode (original behaviour).
        if len(self._scans) >= self._num_samples:
            return
        self._scans.append(raw)
        self._labels.append(label)

        if (len(self._scans) % 50) == 0:
            self.get_logger().info(
                'Collected %d / %d samples (last min_range=%.2f, label=%d)'
                % (len(self._scans), self._num_samples, float(raw.min()), label)
            )

        if len(self._scans) >= self._num_samples:
            self._dump_and_shutdown()

    def _dump_and_shutdown(self) -> None:
        scans = np.stack(self._scans, axis=0).astype(np.float32)
        labels = np.asarray(self._labels, dtype=np.int64)

        os.makedirs(os.path.dirname(self._output_path) or '.', exist_ok=True)
        extra = {}
        if self._motion_enabled:
            extra['joint_positions'] = np.asarray(
                [p[0] for p in self._phases], dtype=np.float64)
            extra['station_labels'] = np.asarray(
                [p[1] for p in self._phases], dtype=np.int64)
            extra['surface_distances'] = np.asarray(
                [p[3] for p in self._phases], dtype=np.float64)
        np.savez(
            self._output_path,
            scans=scans,
            labels=labels,
            scan_size=self._scan_size,
            danger_distance=self._danger_distance,
            far_distance=self._far_distance,
            class_names=np.asarray(['near', 'far', 'free']),
            source=np.asarray('sim_dynamic' if self._motion_enabled else 'sim_static'),
            **extra,
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
