cmake_minimum_required(VERSION 3.0.2)
project(mujoco_arm_ik)

find_package(catkin REQUIRED)

catkin_package()

# 纯 Python 包：无编译目标
# include_directories(${catkin_INCLUDE_DIRS})

# 入口脚本安装到 devel/lib/<pkg>/，供 launch 以 type=main.py 启动
# verify_ik.py 是开发期手动验证脚本（python3 verify_ik.py ik / gui），
# 有意不注册 catkin 测试：它依赖 MuJoCo 与图形渲染后端，在无头 CI 里无法稳定收集。
catkin_install_python(PROGRAMS
  main.py
  verify_ik.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 文件安装到 share/<pkg>/launch/
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# MuJoCo 模型随包安装，launch 里用 $(find mujoco_arm_ik)/arm_2link.xml 引用
install(FILES arm_2link.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
