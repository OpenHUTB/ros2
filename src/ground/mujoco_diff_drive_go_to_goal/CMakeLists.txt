cmake_minimum_required(VERSION 3.0.2)
project(mujoco_diff_drive_go_to_goal)

find_package(catkin REQUIRED)

catkin_package()

# 入口脚本装到 devel/install/lib/<pkg>/；测试脚本是开发期工具，不装进运行时目录
catkin_install_python(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 模型 XML 与 pip 依赖清单装到 share/<pkg>/，保证 install 空间也能找到模型
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 测试：test/test_go_to_goal.py 是 unittest.TestCase 结构，可被 nose / pytest /
# unittest 自动收集；断言走 self.assert* 方法，-O 优化模式下不会被剔除。
# 这里注册到 catkin 测试目标，`catkin test --pkg mujoco_diff_drive_go_to_goal`
# 会执行；测试不随运行时安装。
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
