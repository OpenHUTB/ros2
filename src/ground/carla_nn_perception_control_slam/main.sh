#!/usr/bin/env bash
# 薄包装入口：转发参数到 ROS 2 已安装的模块入口
# 前置：colcon build --packages-select carla_nn_perception_control_slam 之后执行 source install/setup.bash
exec ros2 run carla_nn_perception_control_slam main "$@"
