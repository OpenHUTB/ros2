cmake_minimum_required(VERSION 3.8)
project(cartpole_lqr)

find_package(ament_cmake REQUIRED)

# 安装全部教学脚本（纯 Python，仅依赖标准库）
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
