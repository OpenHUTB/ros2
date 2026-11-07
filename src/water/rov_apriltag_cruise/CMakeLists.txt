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
find_package(cv_bridge REQUIRED)

# cv_bridge 依赖系统 OpenCV 4.2，此处显式指定同一套头文件与库，避免与 /usr/local 版本混用
set(OPENCV_INCLUDE_DIR /usr/include/opencv4)
set(OPENCV_LIB_DIR /usr/lib/x86_64-linux-gnu)

add_executable(apriltag_detector_node src/apriltag_detector.cpp)
target_include_directories(apriltag_detector_node PRIVATE ${OPENCV_INCLUDE_DIR})
ament_target_dependencies(apriltag_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
)
target_link_libraries(apriltag_detector_node
  ${OPENCV_LIB_DIR}/libopencv_core.so
  ${OPENCV_LIB_DIR}/libopencv_imgproc.so
  ${OPENCV_LIB_DIR}/libopencv_imgcodecs.so
  ${OPENCV_LIB_DIR}/libopencv_calib3d.so
  ${OPENCV_LIB_DIR}/libopencv_aruco.so
  ${OPENCV_LIB_DIR}/libopencv_highgui.so
)

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
