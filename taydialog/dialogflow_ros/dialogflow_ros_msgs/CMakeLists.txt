cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_service_files(
  FILES
  DialogflowService.srv
)

add_message_files(
  FILES
  DialogflowResult.msg
  DialogflowContext.msg
  DialogflowParameter.msg
  DialogflowRequest.msg
  DialogflowEvent.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dialogflow_ros_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
