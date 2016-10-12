cmake_minimum_required(VERSION 2.8.3)
project(baxter_robot_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arc_utilities
  geometry_msgs
  trajectory_msgs
  control_msgs
  baxter_core_msgs
  roscpp
  sensor_msgs
  std_msgs
  kdl_parser
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include
#  LIBRARIES baxter_robot_controllers
CATKIN_DEPENDS arc_utilities geometry_msgs trajectory_msgs control_msgs baxter_core_msgs roscpp sensor_msgs std_msgs kdl_parser DEPENDS system_lib Eigen)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wconversion")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -O3 -g")

## Declare C++ executables
add_executable(baxter_left_arm_position_controller include/${PROJECT_NAME}/baxter_robot_config.hpp include/${PROJECT_NAME}/baxter_robot_position_controller.hpp src/baxter_left_arm_position_controller.cpp)
add_dependencies(baxter_left_arm_position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(baxter_left_arm_position_controller ${catkin_LIBRARIES})

add_executable(baxter_right_arm_position_controller include/${PROJECT_NAME}/baxter_robot_config.hpp include/${PROJECT_NAME}/baxter_robot_position_controller.hpp src/baxter_right_arm_position_controller.cpp)
add_dependencies(baxter_right_arm_position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(baxter_right_arm_position_controller ${catkin_LIBRARIES})

add_executable(baxter_left_arm_velocity_torque_controller include/${PROJECT_NAME}/baxter_robot_config.hpp include/${PROJECT_NAME}/baxter_robot_velocity_torque_controller.hpp src/baxter_left_arm_velocity_torque_controller.cpp)
add_dependencies(baxter_left_arm_velocity_torque_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(baxter_left_arm_velocity_torque_controller ${catkin_LIBRARIES})

add_executable(baxter_right_arm_velocity_torque_controller include/${PROJECT_NAME}/baxter_robot_config.hpp include/${PROJECT_NAME}/baxter_robot_velocity_torque_controller.hpp src/baxter_right_arm_velocity_torque_controller.cpp)
add_dependencies(baxter_right_arm_velocity_torque_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(baxter_right_arm_velocity_torque_controller ${catkin_LIBRARIES})
