cmake_minimum_required(VERSION 3.16)
project(mujoco_musculoskeletal_knee_tracking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS simulate_tracking.py validate_model.py
        scripts/mujoco_musculoskeletal_knee_tracking_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config assets DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py)
  ament_add_pytest_test(test_simulation test/test_simulation.py)
  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_package()
