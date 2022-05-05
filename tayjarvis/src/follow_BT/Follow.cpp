#include <string>
#include "follow_BT/Follow.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"

enum
{
    STOP = 0,
    GO = 1,
};

namespace follow_BT
{

Follow::Follow(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), nh_("~")
{
  std::string activation_pub =  nh_.param("set_goal_activation_topic", std::string("/tay_ros/set_goal_activation"));
  act_pub_ = nh_.advertise<std_msgs::Int32>(activation_pub, 1);
}

void
Follow::halt()
{
 // ROS_INFO("SetGoal halt");
}

BT::NodeStatus
Follow::tick()
{
  ROS_INFO("Following.......");
  std_msgs::Int32 state;
  state.data = GO;
  act_pub_.publish(state);

  setOutput<move_base_msgs::MoveBaseGoal>("pos", goal_);
  return BT::NodeStatus::SUCCESS;
}


} // namespace follow_BT


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<follow_BT::Follow>("Follow");
}