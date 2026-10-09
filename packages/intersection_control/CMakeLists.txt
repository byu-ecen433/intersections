cmake_minimum_required(VERSION 3.0.2)
project(intersection_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs duckietown_msgs geometry_msgs
)
