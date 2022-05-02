cmake_minimum_required(VERSION 2.8.3)
project(tayjarvis)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")
set(CMAKE_CXX_FLAGS "-std=c++17")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
  move_base_msgs
  move_base_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kobuki_msgs 
  sensor_msgs
  taydialog
  nav_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
  move_base_msgs
  move_base_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kobuki_msgs 
  sensor_msgs
  taydialog
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

# Follow BTnodes
add_library(BT_move_node SHARED src/follow_BT/Move.cpp)
add_library(BT_target_reached SHARED src/follow_BT/target_reached.cpp)
add_library(BT_follow SHARED src/follow_BT/Follow.cpp)
add_library(BT_set_goal SHARED src/follow_BT/Set_goal.cpp)

# Person_recognize BT nodes
add_library(BT_welcome_human SHARED src/init_sequence_BT/Welcome_human.cpp)
add_library(BT_localize_suitcase SHARED src/init_sequence_BT/Localize_suitcase.cpp)
add_library(BT_recognize SHARED src/init_sequence_BT/Recognize.cpp)

list(APPEND plugin_libs
  BT_move_node
  BT_target_reached
  BT_follow
  BT_set_goal

  BT_localize_suitcase
  BT_welcome_human
  BT_recognize
)



foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(suitcases_loader 
  src/suitcases_loader.cpp
)
target_link_libraries(suitcases_loader ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_library(${PROJECT_NAME} 
src/Set_route/set_route.cpp
)

add_executable(movement src/follow/Movement.cpp)
target_link_libraries(movement ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(set_route_node src/set_route_node.cpp)
target_link_libraries(set_route_node  ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp(
  src/suitcases_loader.cpp
  src/set_route_node.cpp
  src/follow/Movement.cpp
)
