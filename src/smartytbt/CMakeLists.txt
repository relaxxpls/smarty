cmake_minimum_required(VERSION 3.0.2)

project(smartytbt)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/smarty_select_user.py scripts/smarty_follow_user.py scripts/usertbt_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
