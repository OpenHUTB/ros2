@echo off
rem 薄包装入口：转发参数到 ROS 2 已安装的模块入口
rem 前置：先 colcon build 本模块，再执行 install\setup.bat
ros2 run carla_nn_perception_control_slam main %*
