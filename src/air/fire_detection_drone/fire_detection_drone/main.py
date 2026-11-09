"""Single-process entry point for the fire detection and exploration nodes."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .drone_explorer_node import DroneExplorerNode
from .fire_detector_node import FireDetectorNode


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Run the fire detector, the drone explorer, or both.'
    )
    parser.add_argument(
        '--node',
        choices=('both', 'detector', 'explorer'),
        default='both',
        help='Node(s) to run in this process (default: both).',
    )
    options, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    nodes = []
    if options.node in ('both', 'detector'):
        nodes.append(FireDetectorNode())
    if options.node in ('both', 'explorer'):
        nodes.append(DroneExplorerNode())

    executor = MultiThreadedExecutor(num_threads=max(2, len(nodes) + 1))
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
