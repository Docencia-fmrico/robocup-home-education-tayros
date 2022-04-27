#include <string>
#include "find_mate_BT/Is_any_mate.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <unistd.h>

enum
{
  STOP = 0,
  GO = 1,
};

namespace find_mate
{

IsAnyMate::IsAnyMate(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  std::string mov_publisher =  nh_.param("movement_topic", std::string("/tayros/movement"));
  mov_pub_ = nh_.advertise<std_msgs::Int32>(mov_publisher, 1);
}

void
IsAnyMate::halt()
{
}

BT::NodeStatus
IsAnyMate::tick()
{
  ROS_INFO("I found you!!!!!!!!!");
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0; // -0.14
  goal.target_pose.pose.orientation.w = 1.0; // 0.98

  setOutput<move_base_msgs::MoveBaseGoal>("pos", goal);
  //move_.data = STOP;
  //mov_pub_.publish(move_);  
  //sleep(4);

  return BT::NodeStatus::FAILURE;
}  // namespace find_mate

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_mate::IsAnyMate>("Is_any_mate");
}