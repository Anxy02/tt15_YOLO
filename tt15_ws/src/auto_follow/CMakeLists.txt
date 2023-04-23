cmake_minimum_required(VERSION 3.0.2)
project(auto_follow)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

find_package(roscpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES auto_follow
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(people_detector_node 
  src/people_detector.cpp
  src/people_detector_node.cpp 
)

add_executable(follow_controller 
  src/collision_avoidance/configuration_layer.cpp
  src/reactive_controller.cpp
  src/follow_controller.cpp
)

target_link_libraries(people_detector_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(follow_controller
  ${catkin_LIBRARIES}
)
