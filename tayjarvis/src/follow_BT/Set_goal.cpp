#include <string>
#include "follow_BT/Set_goal.h"
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

SetGoal::SetGoal(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), nh_("~")
{
  std::string pos_subscriptor =  nh_.param("target_pos_topic", std::string("/tayros/pose_to_go"));
  std::string activation_pub =  nh_.param("set_goal_activation_topic", std::string("tay_ros/set_goal_activation"));
  
  act_pub_ = nh_.advertise<std_msgs::Int32>(activation_pub, 1);
  pos_sub_ =  nh_.subscribe<move_base_msgs::MoveBaseGoal>(pos_subscriptor, 1, &SetGoal::callback, this);
  counter = 0.0;
}

void
SetGoal::halt()
{
}

BT::NodeStatus
SetGoal::tick()
{
  std_msgs::Int32 state;
  state.data = GO;
  act_pub_.publish(state);

  /* ------------------------ Depuration ------------------------ */
 /* goal_.target_pose.pose.position.x = counter+= 0.01;
  goal_.target_pose.pose.position.y = -3.0;
  goal_.target_pose.header.frame_id = "map";
  goal_.target_pose.pose.orientation.w = 1.0; */

  setOutput<move_base_msgs::MoveBaseGoal>("pos", goal_);
  return BT::NodeStatus::SUCCESS;
}

void
SetGoal::callback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
{
  goal_ = *msg;
  goal_.target_pose.header.frame_id = "map";
}

} // namespace follow_BT


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<follow_BT::SetGoal>("Set_goal");
}