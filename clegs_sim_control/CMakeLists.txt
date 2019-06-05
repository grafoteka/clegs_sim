cmake_minimum_required(VERSION 2.8.3)
project(clegs_sim_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  clhero_gait_controller
  roscpp
  controller_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES clhero_sim_control
#  CATKIN_DEPENDS clhero_gait_controller roscpp
#  DEPENDS system_lib
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(clhero_sim_control_interface src/sim_control_interface.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(clhero_sim_control_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(clhero_sim_control_interface ${catkin_LIBRARIES})