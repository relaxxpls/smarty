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
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS scripts/smarty_select_object.py scripts/smarty_object_detection.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
