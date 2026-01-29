cmake_minimum_required(VERSION 3.10)
project(sensorless_force_control)

find_package(catkin REQUIRED
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
