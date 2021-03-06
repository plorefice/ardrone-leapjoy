cmake_minimum_required(VERSION 2.8.3)
project(ardrone_leapjoy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  joy
  roscpp
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/modules")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Leap REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES leapdrone_ardrone_sim
#  CATKIN_DEPENDS ardrone_autonomy geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LEAP_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(ardrone_leapjoy
	src/ardrone-leapjoy.cpp
	src/leap-interface.cpp
	src/joypad-interface.cpp
	src/drone-interface.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(leapdrone_ardrone_sim_node leapdrone_ardrone_sim_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ardrone_leapjoy
  ${catkin_LIBRARIES}
  ${LEAP_LIBRARIES}
)
