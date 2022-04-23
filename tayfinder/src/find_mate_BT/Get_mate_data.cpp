#include <string>
#include "find_mate_BT/Get_mate_data.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>


namespace find_mate
{

Get_mate_data::Get_mate_data(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
Get_mate_data::halt()
{
  ROS_INFO("Get_mate_data halt");
}

BT::NodeStatus
Get_mate_data::tick()
{
  setOutput<int>("data", 60);
  ROS_INFO("\n Im getting your bank account\n");
 // sleep(2);
  return BT::NodeStatus::SUCCESS;
}  // namespace find_mate

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_mate::Get_mate_data>("Get_mate_data");
}