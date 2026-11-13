cmake_minimum_required(VERSION 3.8)
project(qos_demo)

find_package(ament_cmake REQUIRED)

# 本模块是纯 Python 教学实现（仅依赖标准库），不需要编译，
# 这里只是把脚本与配图安装到 share 目录，便于 ros2 run / launch 调用。
install(DIRECTORY scripts
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tests
        DESTINATION share/${PROJECT_NAME})

ament_package()
