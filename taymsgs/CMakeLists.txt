cmake_minimum_required(VERSION 3.0.2)
project(taymsgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  person.msg
  person_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  move_base_msgs
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime move_base_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

