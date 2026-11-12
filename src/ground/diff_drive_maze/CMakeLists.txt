cmake_minimum_required(VERSION 3.8)
project(diff_drive_maze)

find_package(ament_cmake REQUIRED)

# 入口脚本装到 install/lib/<pkg>/，作为 ROS 2 可执行入口。
# test/ 下是纯 Python 开发期脚本（直接 `python3 test/xxx.py` 运行，不依赖 ROS
# 测试框架），既不装进运行时目录，也不注册到 ROS 2 测试目标；回归时在包目录直接执行。
install(FILES main.py
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
  DESTINATION lib/${PROJECT_NAME}
)

# 模型 XML 装到 share/<pkg>/models/，install 空间由 ament_index_python 定位
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/models
)

# launch 文件装到 share/<pkg>/launch/，供 ros2 launch 拉起 main.py
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
