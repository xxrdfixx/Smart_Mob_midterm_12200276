cmake_minimum_required(VERSION 3.5)
project(fleet_management1)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_package()

# Create an executable target for a wrapper script
add_executable(fleet_management_server src/fleet_management_server.py)
ament_target_dependencies(fleet_management_server rclpy)

install(TARGETS
  fleet_management_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

