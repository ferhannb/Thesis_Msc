cmake_minimum_required(VERSION 2.8.3)
project(udp_video_stream)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set (OpenCV_DIR /usr/local/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  roscpp
  roslint
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES udp_video_stream
  CATKIN_DEPENDS custom_msgs roscpp roslint rospy std_msgs sensor_msgs cv_bridge
  DEPENDS OpenCV
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

add_executable(UDPVideoStream
  src/UDPVideoStream.cpp
  src/PracticalSocket.cpp
  )

target_link_libraries(UDPVideoStream
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_udp_video_stream.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
