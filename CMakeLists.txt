cmake_minimum_required(VERSION 3.0.2)
project(gps_waypoint_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
)

include_directories(include/gps_waypoint_nav)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(angle src/driving_angle.cpp)
target_link_libraries(angle ${catkin_LIBRARIES} -lcurl)

add_executable(nav src/navigation.cpp)
target_link_libraries(nav ${catkin_LIBRARIES})

