#include <string>
#include "follow_BT/target_reached.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include <unistd.h>
#include "dialog_cbs/dialog_cbs.h"

enum
{
    STOP = 0,
    TF = 1,
};

namespace follow_BT
{

target_reached::target_reached(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), nh_("~")
{
  std::string activation_pub =  nh_.param("set_goal_activation_topic", std::string("/tay_ros/set_goal_activation"));
  act_pub_ = nh_.advertise<std_msgs::Int32>(activation_pub, 1);
}

void
target_reached::halt()
{
  //ROS_INFO("Target reached halt");
}

BT::NodeStatus
target_reached::tick()
{
  std_msgs::Int32 state;
  state.data = STOP;
 // act_pub_.publish(state);

  if (first_execute == 0)
  {
    //litsener.movementIndications();
    first_execute = 1;
  }
  
  ROS_INFO("Its target reached??");
  if (litsener.isCarReached() == "true")
  {
      return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}  

}// namespace follow_BT

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<follow_BT::target_reached>("Target_reached");
}