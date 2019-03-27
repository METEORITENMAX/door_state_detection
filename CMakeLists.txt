cmake_minimum_required(VERSION 2.8.3)
project(door_state_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  door_state_detection_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES door_state_detection
   CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
#  DEPENDS system_lib
)
include_directories(
 # include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
add_executable(door_state_detection src/door_state_detection_node.cpp)

target_link_libraries(
    door_state_detection
    ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
