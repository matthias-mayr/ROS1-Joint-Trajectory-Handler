cmake_minimum_required(VERSION 3.0.2)
project(joint_trajectory_handler)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  rospy
  sensor_msgs
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib control_msgs rospy sensor_msgs trajectory_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
