cmake_minimum_required(VERSION 3.10)
project(sensorless_force_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

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
  ${CMAKE_CURRENT_SOURCE_DIR}
  functionlib
  ${catkin_INCLUDE_DIRS}
)

add_executable(uvms_api_test
  src/uvms_api_test.cpp
)
target_link_libraries(uvms_api_test
  ${catkin_LIBRARIES}
)
