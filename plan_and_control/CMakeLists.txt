cmake_minimum_required(VERSION 3.0.2)
project(plan_and_control)

 add_compile_options(-std=c++14)

 set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)
 find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  uav_msgs
  visualization_msgs
  tracking_controller
  trajectory_generator
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES plan_and_control
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs uav_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)

target_link_libraries(trajectory_publisher_node
   ${catkin_LIBRARIES}
   trajectory_generator
)

add_executable(trajectory_tracking_node src/trajectory_tracking_node.cpp)

target_link_libraries(trajectory_tracking_node
   ${catkin_LIBRARIES}
   tracking_controller
)

add_executable(trajectory_replan_node src/trajectory_replan_node.cpp)

target_link_libraries(trajectory_replan_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   trajectory_generator
)