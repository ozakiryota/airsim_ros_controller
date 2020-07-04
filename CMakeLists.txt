cmake_minimum_required(VERSION 2.8.3)
project(airsim_vehicle_control)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros_pkgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES airsim_vehicle_control
#  CATKIN_DEPENDS airsim_ros_pkgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(takeoff src/takeoff.cpp)
target_link_libraries(takeoff ${catkin_LIBRARIES})
add_executable(test_flight src/test_flight.cpp)
target_link_libraries(test_flight ${catkin_LIBRARIES})
add_executable(go_straight src/go_straight.cpp)
target_link_libraries(go_straight ${catkin_LIBRARIES})
