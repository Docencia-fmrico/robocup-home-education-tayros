cmake_minimum_required(VERSION 2.8.3)
project(taynav)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  tf
  std_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
  move_base_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    pcl_ros
    roscpp
    rospy
    tf
    std_msgs
    behaviortree_cpp_v3
    roslint
    actionlib
    actionlib_msgs
    move_base_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_executable(set_goal_node src/set_goal_node.cpp)
target_link_libraries(set_goal_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/set_goal_node.cpp
)