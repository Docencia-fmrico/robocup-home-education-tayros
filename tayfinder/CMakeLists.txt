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
  tf2
  tf2_ros
  tf2_geometry_msgs
  kobuki_msgs 
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
  image_geometry
  taymsgs
  taydialog
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS 
INCLUDE_DIRS include
LIBRARIES
  geometry_msgs
    roscpp std_msgs 
    behaviortree_cpp_v3 
    actionlib_msgs 
    move_base_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    kobuki_msgs 
    sensor_msgs
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
    image_geometry
    taymsgs
    taydialog
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(BT_bump_go_finder SHARED src/find_mate_BT/Bump_go.cpp)
add_library(BT_get_mate_data_finder SHARED src/find_mate_BT/Get_mate_data.cpp)
add_library(BT_is_any_mate_finder SHARED src/find_mate_BT/Is_any_mate.cpp)

add_library(BT_move_node_finder SHARED src/back_home_BT/Move.cpp)
add_library(BT_recovery_node_finder SHARED src/back_home_BT/Recovery.cpp)
add_library(BT_say_data_finder SHARED src/back_home_BT/Say_data.cpp)
add_library(${PROJECT_NAME}  
src/person_localization/PersonLocalizator.cpp
src/person_localization/BbxsTo3D.cpp
src/person_info/Take_person_info.cpp)

list(APPEND plugin_libs
  BT_move_node_finder
  BT_say_data_finder
  BT_bump_go_finder
  BT_is_any_mate_finder
  BT_get_mate_data_finder
  BT_recovery_node_finder
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(find_my_mates src/find_my_mates.cpp)
target_link_libraries(find_my_mates ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(person_localizator src/person_localizator.cpp)
target_link_libraries(person_localizator ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bbxs_to_3dpoint src/bbxs_to_3dpoint.cpp)
target_link_libraries(bbxs_to_3dpoint ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(take_person_info_node src/take_person_info_node.cpp)
target_link_libraries(take_person_info_node ${PROJECT_NAME} ${catkin_LIBRARIES})


roslint_cpp(
  src/find_my_mates.cpp
)
