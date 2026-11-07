cmake_minimum_required(VERSION 3.8)
project(mujoco_traffic_light_stop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES main.py scene.xml
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
