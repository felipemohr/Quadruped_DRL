cmake_minimum_required(VERSION 3.8)
project(quadruped_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(quadruped_interfaces REQUIRED)

include_directories(include)

add_executable(observation_collector src/ObservationCollector.cpp)
ament_target_dependencies(observation_collector 
  rclcpp tf2 std_msgs nav_msgs sensor_msgs geometry_msgs quadruped_interfaces
)

add_executable(agent_controller src/AgentController.cpp)
ament_target_dependencies(agent_controller 
  rclcpp sensor_msgs trajectory_msgs quadruped_interfaces
)

install(
  TARGETS observation_collector agent_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/agent_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
