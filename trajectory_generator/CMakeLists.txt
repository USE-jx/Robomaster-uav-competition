cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES trajectory_generator
  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_library(trajectory_generator
   src/mini_jerk_traj.cpp
 )

 target_link_libraries(trajectory_generator
   ${catkin_LIBRARIES}
 )


