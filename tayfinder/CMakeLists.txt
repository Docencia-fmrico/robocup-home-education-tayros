cmake_minimum_required(VERSION 2.8.3)
project(tayfinder)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
  move_base_msgs
)

catkin_package(
CATKIN_DEPENDS 
    roscpp std_msgs 
    behaviortree_cpp_v3 
    actionlib_msgs 
    move_base_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(BT_bump_go_finder SHARED src/find_mate_BT/Bump_go.cpp)
add_library(BT_get_mate_data_finder SHARED src/find_mate_BT/Get_mate_data.cpp)
add_library(BT_is_any_mate_finder SHARED src/find_mate_BT/Is_any_mate.cpp)

add_library(BT_move_node_finder SHARED src/back_home_BT/Move.cpp)
add_library(BT_say_data_finder SHARED src/back_home_BT/Say_data.cpp)

list(APPEND plugin_libs
  BT_move_node_finder
  BT_say_data_finder
  BT_bump_go_finder
  BT_is_any_mate_finder
  BT_get_mate_data_finder
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(find_my_mates 
  src/find_my_mates.cpp
)
target_link_libraries(find_my_mates ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})


roslint_cpp(
  src/find_my_mates.cpp
)
