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
find_package(geometry_msgs REQUIRED)
find_package(quadruped_interfaces REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(inverse_kinematics src/InverseKinematics.cpp)
ament_target_dependencies(inverse_kinematics 
  rclcpp geometry_msgs quadruped_interfaces tf2_eigen Eigen3
)

install(
  TARGETS inverse_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
