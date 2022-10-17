cmake_minimum_required(VERSION 2.8.3)
project(heading_controller)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  roscpp
  roslint
  rospy
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(HeadingController src/HeadingController.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(HeadingController
   ${catkin_LIBRARIES}
 )
