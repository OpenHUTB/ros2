cmake_minimum_required(VERSION 3.0.2)
project(mujoco_offroad_suspension_robot_noetic)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})

# Python 脚本按 ROS 约定安装到 lib/<pkg>/，不要依赖源码目录权限位
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/mujoco_offroad_node.py
  scripts/headless_demo.py
  scripts/physics_only_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 库模块：被入口脚本 import，与入口脚本同装到 lib/<pkg>/。
# 四个入口脚本开头都有 sys.path.insert(0, dirname(__file__))，因此同目录的模块
# 不依赖 PYTHONPATH 就能 import（devel 与 install 空间都是这个布局）。
# 没有走 catkin_python_setup() + setup.py：这几个是扁平模块（offroad_sim_core、
# terrain、attitude_controller、gen_model、plot_curves），单独建 Python 包反而更容易
# 出现包名与模块名两套路径。若以后要把库模块挪出本目录，务必同步改各入口的 sys.path 与 import。
install(FILES
  scripts/offroad_sim_core.py
  scripts/attitude_controller.py
  scripts/terrain.py
  scripts/gen_model.py
  scripts/plot_curves.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 这里刻意不 install 包内的 docs/img：它是 physics_only_demo.py / headless_demo.py
# 的运行时输出目录（程序贴图、gif、曲线、csv、json），每次运行都会重写，
# 装进 share 只会把生成物塞进工作空间。
