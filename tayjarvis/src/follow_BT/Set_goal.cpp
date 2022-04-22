#include <string>
#include "follow_BT/Set_goal.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>


namespace follow_BT
{

SetGoal::SetGoal(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
SetGoal::halt()
{
  ROS_INFO("SetGoal halt");
}

BT::NodeStatus
SetGoal::tick()
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.position.x = 1.21;
  goal.target_pose.pose.position.y = -3.47;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0; // -0.14
  goal.target_pose.pose.orientation.w = 1.0; // 0.98

  setOutput<move_base_msgs::MoveBaseGoal>("pos", goal);
  return BT::NodeStatus::SUCCESS;
}  // namespace follow_BT

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<follow_BT::SetGoal>("Set_goal");
}