#include <string>
#include "back_home_BT/Say_data.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>


namespace Back_home
{

Say_data::Say_data(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
Say_data::halt()
{
  ROS_INFO("Say_data halt");
}

BT::NodeStatus
Say_data::tick()
{
  data_ = getInput<int>("data").value();
  ROS_INFO("Compradas %d propiedades en Puzela", data_);

  return BT::NodeStatus::SUCCESS;
}  // namespace Back_home

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<Back_home::Say_data>("Say_data");
}