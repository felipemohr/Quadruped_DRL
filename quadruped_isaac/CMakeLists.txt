cmake_minimum_required(VERSION 3.8)
project(quadruped_isaac)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_isaacsim.py
  scripts/standalone/quadruped_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
