#include <string>
#include "init_sequence_BT/Localize_suitcase.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"

namespace person_recognize
{

Localize_suitcase::Localize_suitcase(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
Localize_suitcase::halt()
{
}


BT::NodeStatus
Localize_suitcase::tick()
{
  ROS_INFO("Veo tu equipajeee");
  return BT::NodeStatus::FAILURE;
}  

}// namespace person_recognize

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<person_recognize::Localize_suitcase>("Localize_suitcase");
}