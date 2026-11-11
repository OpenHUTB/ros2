cmake_minimum_required(VERSION 3.0.2)
project(two_dof_arm_square)

find_package(catkin REQUIRED)

catkin_package()

# 入口脚本装到 devel/install/lib/<pkg>/，供 launch 以 type=main.py 启动；
# 验证脚本是开发期工具，不装进运行时目录（在包目录下手动执行即可）
catkin_install_python(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 文件装到 share/<pkg>/launch/
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 模型 XML 与 pip 依赖清单装到 share/<pkg>/，保证 install 空间 roslaunch 也能找到模型
install(FILES two_dof_arm_square.xml requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
