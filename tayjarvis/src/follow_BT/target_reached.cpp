#include <string>
#include "follow_BT/target_reached.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>


namespace follow_BT
{

target_reached::target_reached(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
target_reached::halt()
{
  ROS_INFO("Target reached halt");
}

BT::NodeStatus
target_reached::tick()
{
  ROS_INFO("Where I am?????????????????");
  return BT::NodeStatus::SUCCESS;
}  

}// namespace follow_BT

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<follow_BT::target_reached>("Target_reached");
}