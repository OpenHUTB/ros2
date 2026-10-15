cmake_minimum_required(VERSION 3.0.2)
project(gazebo_diff_drive_car)

# ============================================================================
# OpenHUTB 地面载具模块 —— Gazebo 差速驱动无人车
# ============================================================================
#
# 目录布局（遵循 OpenHUTB/ros2 仓库约定）：
#
#     gazebo_diff_drive_car/
#     ├── main.py           ★ 统一入口节点（位于包根目录）
#     ├── main.sh           ★ Linux 启动脚本
#     ├── main.bat          ★ Windows 启动脚本
#     ├── launch/main.launch
#     ├── scripts/
#     │   ├── teleop_keyboard.py
#     │   ├── laser_avoid.py
#     │   ├── depth_camera_avoid.py
#     │   ├── odom_recorder.py
#     │   ├── plot_trajectory.py
#     │   └── analyze_trajectory.py
#     ├── urdf/car.xacro
#     ├── worlds/car_world.world
#     ├── config/params.yaml
#     └── rviz/
#
# ============================================================================

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# ============================================================================
# 安装 Python 节点脚本
# ============================================================================
#
# 注意：此处刻意不使用 catkin_install_python()。
# 该宏会在 devel 空间中生成一个「relay 脚本」（内容为 exec(compile(...)) 转发），
# 而 relay 脚本无法作为 Python 模块被 import。
# 本模块的 main.py 需要 `from laser_avoid import LaserAvoidNode`
# 这类跨脚本导入，因此必须原样复制文件。

set(_NODE_SCRIPTS
  main.py
  scripts/teleop_keyboard.py
  scripts/laser_avoid.py
  scripts/depth_camera_avoid.py
  scripts/odom_recorder.py
  scripts/plot_trajectory.py
  scripts/analyze_trajectory.py
)

# ---- (1) 安装空间：标准安装 ----
install(PROGRAMS ${_NODE_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- (2) devel 空间：原样复制为可导入模块 ----
# catkin_make 只把编译产物放进 devel/，install() 规则要到 catkin_make install
# 才生效。为了让 rosrun / roslaunch 在纯 devel 环境下就能找到节点，
# 这里用一个自定义 target 把脚本原样复制到 devel/lib/<pkg>/。
set(_DEVEL_BIN_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(_script ${_NODE_SCRIPTS})
  get_filename_component(_name ${_script} NAME)

  add_custom_command(OUTPUT ${_DEVEL_BIN_DIR}/${_name}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${_DEVEL_BIN_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/${_script}
            ${_DEVEL_BIN_DIR}/${_name}
    COMMAND chmod +x ${_DEVEL_BIN_DIR}/${_name}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${_script}
    COMMENT "Copying ${_name} to devel space"
  )
  list(APPEND _DEVEL_SCRIPTS ${_DEVEL_BIN_DIR}/${_name})
endforeach()

add_custom_target(gazebo_diff_drive_car_scripts ALL DEPENDS ${_DEVEL_SCRIPTS})

# ============================================================================
# 安装其余资源
# ============================================================================

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
