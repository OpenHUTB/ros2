cmake_minimum_required(VERSION 3.0.2)
project(turtle_rainbow_spiral)

## 查找catkin和我们用到的依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
)

## 声明这个catkin包
catkin_package()

## 把rainbow_spiral.py装到bin目录下
catkin_install_python(PROGRAMS
  scripts/rainbow_spiral.py
  DESTINATION  ${CATKIN_PACKAGE_BIN_DESTINATION}
)