cmake_minimum_required(VERSION 3.0.2)
project(octree_uav_3d_pathfinding)

## 空域载具的八叉树三维寻路
##
## 当前版本包含：启动文件、环境与链路自检节点、四旋翼位置控制器、
##               点云转八叉树占用地图节点（C++）。
## 后续提交将逐步加入以下节点：
##   - octree_astar_planner    八叉树上的 A* 全局寻路
##   - path_smoother           安全裕度与路径平滑
##   - path_follower           路径跟踪与闭环飞行
##
## 说明：octomap 的头文件路径与库也可以由 octomap_ros / octomap_msgs 间接引入，
## 但从本包开始出现 C++ 节点后，这里额外显式 find_package(octomap)，
## 使依赖关系一目了然，也避免上游包导出方式变化导致链接失败。

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
  visualization_msgs
)

# 八叉树本体（octomap 1.9.8，来自 liboctomap-dev）
find_package(octomap REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES octree_uav_3d_pathfinding
  CATKIN_DEPENDS geometry_msgs nav_msgs octomap_msgs octomap_ros pcl_conversions pcl_ros
                 roscpp rospy sensor_msgs std_msgs tf tf2_ros visualization_msgs
  DEPENDS octomap
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## C++ 节点：点云 → 八叉树占用地图
add_executable(pointcloud_to_octomap src/pointcloud_to_octomap.cpp)
target_link_libraries(pointcloud_to_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(pointcloud_to_octomap
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Python 节点
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/airsim_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## C++ 节点
install(TARGETS pointcloud_to_octomap
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 启动文件、参数配置、RViz 配置
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

#############
## Testing ##
#############
## 测试规则后续补充
