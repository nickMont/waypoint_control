cmake_minimum_required(VERSION 2.8.3)
project(waypoint_control)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix -DCATKIN_BUILD")

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
	mg_msgs
)
find_package(Eigen REQUIRED)

catkin_package(
	DEPENDS Eigen
	INCLUDE_DIRS include
	LIBRARIES waypointControl
	CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs mg_msgs
	DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

# Glob all of the source files together
add_executable(waypoint_control src/main.cpp src/waypoint_control.cpp)

target_link_libraries(waypoint_control ${catkin_LIBRARIES})

#This makes sure that messages and services are compiled before the rest
add_dependencies(waypoint_control mg_msgs_generate_messages_cpp)
add_dependencies(waypoint_control ${catkin_EXPORTED_TARGETS})

