#include <string>
#include "find_mate_BT/Get_mate_data.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include <unistd.h>
#include "taymsgs/person_data.h"


namespace find_mate
{

Get_mate_data::Get_mate_data(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  mate_reached_pub_ = nh_.advertise<std_msgs::Bool>("/tayfinder/mate_reached", 1);
  mate_data_sub_ =  nh_.subscribe<taymsgs::person_data>("/tayfinder/mate_data", 1, &Get_mate_data::data_callback, this);

  reached_.data = false;
  data_received_ = false;  //------------------------------------------------------------
}

void
Get_mate_data::halt()
{
}

void 
Get_mate_data::data_callback(const taymsgs::person_data::ConstPtr& msg)
{
  data_ = *msg;  
  data_received_ = true;
}

BT::NodeStatus
Get_mate_data::tick()
{
  reached_.data = true;
  mate_reached_pub_.publish(reached_);

  if (data_received_)
  {
    setOutput<taymsgs::person_data>("data", data_);
    data_received_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}  // namespace find_mate

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_mate::Get_mate_data>("Get_mate_data");
}