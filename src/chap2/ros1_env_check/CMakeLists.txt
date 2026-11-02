cmake_minimum_required(VERSION 3.0.2)
project(ros1_env_check)

# 不 find_package(rospy) 等组件：本模块不依赖 ROS 运行。
# 声明成 catkin 包只是为了与 src/chap2 下其它模块保持一致，
# 并让 catkin_make 把两个入口安装到 devel/lib/ros1_env_check。
find_package(catkin REQUIRED)

catkin_package()

# 两个入口都是可执行脚本（check_env.py 是 Python，main.sh 是 bash 包装），
# 因此用 install(PROGRAMS ...) 而不是 catkin_install_python。
install(PROGRAMS
  check_env.py
  main.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
