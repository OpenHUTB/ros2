cmake_minimum_required(VERSION 3.16)
project(mujoco_musculoskeletal_arm_reach)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/arm_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py)
  ament_add_pytest_test(test_control test/test_control.py)
  ament_add_pytest_test(test_simulation test/test_simulation.py)
endif()

ament_package()
