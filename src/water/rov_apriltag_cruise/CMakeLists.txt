cmake_minimum_required(VERSION 3.8)
project(rov_apriltag_cruise)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# 优先使用发行版软件包提供的 OpenCV（与 cv_bridge 同源，且含 contrib/ArUco）。
# 不使用 /usr/local 下的自编译版本，避免其未编译 contrib 时缺 aruco，或与 cv_bridge 版本不一致。
# 路径中的架构目录由 CMake 自动生成，x86_64 / aarch64 通用；若列举路径均不存在则退回默认搜索。
foreach(_opencv_dir
    "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4"
    "/usr/lib/cmake/opencv4"
    "/usr/share/opencv4")
  if(EXISTS "${_opencv_dir}/OpenCVConfig.cmake")
    set(OpenCV_DIR "${_opencv_dir}" CACHE PATH "OpenCV config directory")
    break()
  endif()
endforeach()
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco highgui)
message(STATUS "Using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

find_package(cv_bridge REQUIRED)

add_executable(apriltag_detector_node src/apriltag_detector.cpp)
target_include_directories(apriltag_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(apriltag_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
)
target_link_libraries(apriltag_detector_node ${OpenCV_LIBS})

install(TARGETS apriltag_detector_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cruise_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  main.py
  main.sh
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch urdf world models config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
