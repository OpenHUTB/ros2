cmake_minimum_required(VERSION 3.0.2)
project(octree_uav_3d_pathfinding)

## 空域载具的八叉树三维寻路
##
## 当前版本包含：启动文件、主入口节点（环境与链路自检）、参数配置。
## 后续提交将逐步加入以下节点：
##   - pointcloud_to_octomap   传感器点云 → 八叉树占用地图
##   - octree_astar_planner    八叉树上的 A* 全局寻路
##   - path_smoother           安全裕度与路径平滑
##   - path_follower           路径跟踪与闭环飞行
##
## 说明：octomap 的头文件路径与库通过 octomap_msgs / octomap_ros / pcl_ros
## 由 catkin 间接引入，无需单独 find_package(octomap) / find_package(PCL)。

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs octomap_msgs octomap_ros pcl_conversions pcl_ros
                 roscpp rospy sensor_msgs std_msgs tf tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Python 节点（入口为 main.py）
catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 启动文件、参数配置、模型、RViz 配置
install(DIRECTORY launch config models rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
