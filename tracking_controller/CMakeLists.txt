cmake_minimum_required(VERSION 3.0.2)
project(tracking_controller)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tracking_controller
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(tracking_controller
   src/geometric_controller.cpp
 )

## Specify libraries to link a library or executable target against
 target_link_libraries(tracking_controller
   ${catkin_LIBRARIES}
 )

