cmake_minimum_required(VERSION 2.8.3)
project(waypoint_control)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  message_generation
  cmake_modules
  app_pathplanner_interface
)

add_message_files(DIRECTORY msg FILES
   PVA.msg
 )

find_package(Eigen REQUIRED)

generate_messages( DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
	DEPENDS Eigen
  INCLUDE_DIRS include
  LIBRARIES waypointControl
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs app_pathplanner_interface
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(waypoint_control src/waypoint_control.cpp)

target_link_libraries(waypoint_control ${catkin_LIBRARIES})

#This makes sure that messages and services are compiled before the rest
add_dependencies(waypoint_control waypoint_control_generate_messages_cpp)
add_dependencies(waypoint_control ${catkin_EXPORTED_TARGETS})

