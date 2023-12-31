cmake_minimum_required(VERSION 3.8)
project(quadruped_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegJoints.msg"
  "msg/LegIK.msg"
  "srv/LegIK.srv"
  "msg/QuadrupedJoints.msg"
  "msg/QuadrupedIK.msg"
  "srv/QuadrupedIK.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(ik_server src/ik_server.cpp)
ament_target_dependencies(ik_server rclcpp geometry_msgs tf2_eigen Eigen3)

add_executable(ik_node src/InverseKinematics.cpp)
ament_target_dependencies(ik_node rclcpp std_msgs sensor_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(ik_server "${cpp_typesupport_target}")
target_link_libraries(ik_node "${cpp_typesupport_target}")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ik_server
    ik_node
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
