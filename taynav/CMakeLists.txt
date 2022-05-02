cmake_minimum_required(VERSION 2.8.3)
project(taynav)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")
set(CMAKE_CXX_FLAGS "-std=c++17")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  nav_msgs
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
    nav_msgs
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



add_executable(tf_to_goal_node src/tf_to_goal.cpp)
target_link_libraries(tf_to_goal_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_library(${PROJECT_NAME} 
src/Set_route/set_route.cpp
)
add_executable(set_route_node src/set_route_node.cpp)
target_link_libraries(set_route_node  ${PROJECT_NAME} ${catkin_LIBRARIES})


roslint_cpp(
  src/tf_to_goal.cpp
)