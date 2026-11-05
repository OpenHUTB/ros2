"""Module entry point.

This module is the documented ``main.py`` entry of the
``lidar_obstacle_nn`` package. It is intentionally thin so that the
module can be launched directly with:

    python -m lidar_obstacle_nn.main
    # or
    ros2 run lidar_obstacle_nn main

It launches the inference node (which loads the trained MLP weights
and classifies /scan into /obstacle_class). The same node is also
started by ``launch/main.launch.py`` together with the Gazebo
simulation and the ros_gz bridge.
"""

from __future__ import annotations

from lidar_obstacle_nn.inference_node import main as run_inference


def main() -> None:
    """Run the inference node."""
    run_inference()


if __name__ == '__main__':
    main()
