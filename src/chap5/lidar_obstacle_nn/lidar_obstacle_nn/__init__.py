"""lidar_obstacle_nn ROS 2 package.

A small PyTorch MLP that classifies Gazebo LiDAR scans into three
obstacle-proximity categories: ``near``, ``far`` and ``free``.

Public submodules:
    - ``inference_node``  : ROS 2 node publishing class predictions.
    - ``collect_data``   : record /scan + label to a .npz dataset.
    - ``train_model``     : train the MLP from the recorded dataset.
"""

__version__ = '0.1.0'
