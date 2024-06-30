
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-sensors6 REQUIRED)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
ign_find_package(ignition-gazebo6 REQUIRED)
ign_find_package(ignition-msgs8 REQUIRED VERSION 8.2)

add_library(FootSensorSystem SHARED FootSensorSystem.cc)
set_property(TARGET FootSensorSystem PROPERTY CXX_STANDARD 17)

install(
  TARGETS FootSensorSystem 
  DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR}
)

target_link_libraries(FootSensorSystem
  PRIVATE ignition-common4::ignition-common4
  PRIVATE ignition-plugin1::register
  PRIVATE ignition-gazebo6::ignition-gazebo6
  PRIVATE ignition-sensors6::ignition-sensors6
)
