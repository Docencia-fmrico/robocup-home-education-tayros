#include <string>
#include "init_sequence_BT/Welcome_human.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"
#include "dialog_cbs/dialog_cbs.h"

namespace person_recognize
{

Welcome_human::Welcome_human(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
Welcome_human::halt()
{
}


BT::NodeStatus
Welcome_human::tick()
{
  gb_dialog::DialogManager speaker;
  speaker.welcomeHuman();
  
  return BT::NodeStatus::SUCCESS;
}  

}// namespace person_recognize

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<person_recognize::Welcome_human>("Welcome_human");
}