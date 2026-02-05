cmake_minimum_required(VERSION 3.10)
project(sensorless_force_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

find_package(catkin REQUIRED
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  cola2_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy nav_msgs geometry_msgs sensor_msgs std_msgs cola2_msgs
  DEPENDS
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  functionlib
  ${catkin_INCLUDE_DIRS}
)

add_executable(uvms_api_test
  test/uvms_api_test.cpp
)
target_link_libraries(uvms_api_test
  ${catkin_LIBRARIES}
)

add_executable(girona_controller
  src/girona_controller.cpp
  src/girona_interface.cpp
)
target_link_libraries(girona_controller
  ${catkin_LIBRARIES}
)

add_executable(joint_cmd_watchdog
  src/joint_cmd_watchdog.cpp
)
target_link_libraries(joint_cmd_watchdog
  ${catkin_LIBRARIES}
)

add_executable(task_priority_solver_test
  test/task_priority_solver_test.cpp
)
target_link_libraries(task_priority_solver_test
  ${catkin_LIBRARIES}
)
