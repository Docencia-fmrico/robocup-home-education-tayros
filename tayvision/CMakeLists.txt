cmake_minimum_required(VERSION 2.8.3)
project(tayvision)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_geometry_msgs
  roscpp
  tf2
  tf2_ros
  roslint
  std_msgs
  kobuki_msgs
  sensor_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
  behaviortree_cpp_v3
  image_geometry
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS include
  LIBRARIES
  geometry_msgs
  tf2_geometry_msgs
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  roslint
  tf
  tf2
  tf2_ros
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  behaviortree_cpp_v3
  image_geometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} 
src/bbx_to_3d/BbxTo3D.cpp
src/object_bbx_filter/BbxFilter.cpp
src/bbx_color_detection/BbxColorDetector.cpp
src/pointing_detection/PointingDetector.cpp)


add_executable(bbx_to_3d_node src/bbx_to_3d_node.cpp)
target_link_libraries(bbx_to_3d_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bbx_filter_node src/bbx_filter_node.cpp)
target_link_libraries(bbx_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bbx_color_detector_node src/bbx_color_detector_node.cpp)
target_link_libraries(bbx_color_detector_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pointing_detector_node src/pointing_detector_node.cpp)
target_link_libraries(pointing_detector_node ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp(
  
)
