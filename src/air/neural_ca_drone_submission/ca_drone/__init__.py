"""Neural collision-avoidance drone: perception, planning, control and end-to-end.

The package is deliberately dependency-light (NumPy + Pillow) so that the
complete pipeline can run in CI and on a laptop without a GPU, while the same
modules are wrapped by ROS 1 Noetic / ROS 2 Humble nodes for Gazebo runs.
"""

__version__ = "1.0.0"
