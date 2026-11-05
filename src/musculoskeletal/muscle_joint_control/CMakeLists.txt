cmake_minimum_required(VERSION 3.0.2)
project(muscle_joint_control)

# 模块本身不依赖 rospy，本包只用来把它纳入 catkin 工作空间并支持 launch 启动
find_package(catkin REQUIRED)

catkin_package()

# 包根 main.py 是源码树里 `python3 main.py` 的入口，转调 scripts/main.py。
# 安装时只装 scripts/main.py（原名安装为 bin/main.py），两者同名，
# 同时装会冲突，所以这里保持只装一份；launch 里的 type="main.py" 即指向它。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/muscle_model.py
  scripts/joint_model.py
  scripts/stiffness.py
  scripts/simple_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
