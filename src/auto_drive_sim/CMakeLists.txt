cmake_minimum_required(VERSION 3.0.2)
project(auto_drive_sim)

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

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/drive_decision/dec_main.py

  src/drive_perception/camera/per_camera.py
  src/drive_perception/per_lidar.py
  src/drive_perception/per_DeadReckoning_gps.py
  src/drive_perception/per_nav.py
  src/drive_perception/per_traffic.py
  src/drive_perception/per_person.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
