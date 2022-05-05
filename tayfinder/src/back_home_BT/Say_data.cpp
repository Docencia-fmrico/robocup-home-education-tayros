#include <string>
#include "back_home_BT/Say_data.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include <unistd.h>
#include "taymsgs/person_data.h"
#include "std_msgs/Int32.h"
#include "dialog_cbs/dialog_cbs.h"


namespace Back_home
{

Say_data::Say_data(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  data_comunicated_pub_ = nh_.advertise<std_msgs::Bool>("/tayfinder/data_comunicated", 1);

  comunicated_.data = false;
}

void
Say_data::halt()
{
}

BT::NodeStatus
Say_data::tick()
{
  comunicated_.data = true;
  ROS_INFO("Saiying sata....");
  data_ = getInput<taymsgs::person_data>("data").value();
  speaker_.sayPersonData(&data_);

  data_comunicated_pub_.publish(comunicated_);
  return BT::NodeStatus::SUCCESS;
}  // namespace Back_home

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<Back_home::Say_data>("Say_data");
}