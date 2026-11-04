cmake_minimum_required(VERSION 3.0.2)
project(mrobot_description)

# 本包仅包含 URDF/xacro 模型文件，不编译 C++ 节点。
find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()
