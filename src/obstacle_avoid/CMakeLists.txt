cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoid)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Declare ROS messages
add_message_files(
  FILES
  PersonBBox.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
