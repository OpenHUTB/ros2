cmake_minimum_required(VERSION 3.0.2)
project(mrobot_bringup)

# 本包仅包含 launch 与 arbotix 配置，不编译 C++ 节点。
find_package(catkin REQUIRED)

catkin_package()
