cmake_minimum_required(VERSION 3.0.2)
project(mujoco_arm_ball)

find_package(catkin REQUIRED)

catkin_package()

# 纯 Python 包：无编译目标
# include_directories(${catkin_INCLUDE_DIRS})

# 测试：test/verify_armball.py 是开发期的三模式手动验证脚本（ik / swing / clear），
# 不是 unittest 用例结构，因此这里**有意不注册 catkin 测试**：
# 不调用 catkin_add_nosetests / catkin_add_pytests，package.xml 也不加 <test_depend>。
# 所以 `catkin test --pkg mujoco_arm_ball` 不会收集到用例——这是设计选择，不是遗漏。
# 与同目录 mujoco_arm_screw / mujoco_arm_3d / mujoco_arm_pick 的口径一致。
# 手动执行：cd src/chap8/mujoco_arm_ball && python3 test/verify_armball.py ik|swing|clear

# 只把入口脚本装到 devel/lib/<pkg>/，供 launch 以 type=main.py 启动；
# 验证脚本是开发期工具，不装进运行时目录（在包目录下手动执行即可）
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 文件安装到 share/<pkg>/launch/
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
