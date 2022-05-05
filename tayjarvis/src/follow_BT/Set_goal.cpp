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
  std::string activation_pub =  nh_.param("set_goal_activation_topic", std::string("/tay_ros/set_goal_activation"));
  std::string bbx_filter_active_topic =  nh_.param("activation_bbx_filter", std::string("/tayros/activation_bx_filter"));
  
  act_pub_ = nh_.advertise<std_msgs::Int32>(activation_pub, 1);
  bbx_act_pub_ = nh_.advertise<std_msgs::Int32>(bbx_filter_active_topic, 1);
  pos_sub_ =  nh_.subscribe<move_base_msgs::MoveBaseGoal>(pos_subscriptor, 1, &SetGoal::callback, this);
  counter = 0.0;
  first_msg_recived_ = false;
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
  bbx_act_pub_.publish(state);
  ROS_INFO("Set goal execute");

  /* ------------------------ Depuration ------------------------ */
 /* goal_.target_pose.pose.position.x = counter+= 0.01;
  goal_.target_pose.pose.position.y = -3.0;
  goal_.target_pose.header.frame_id = "map";
  goal_.target_pose.pose.orientation.w = 1.0; */
  if (!first_msg_recived_)
  {
    ROS_INFO("Set goal execute failed");
    return BT::NodeStatus::FAILURE;
  }
  ROS_INFO("Set goal execute succedded");
  setOutput<move_base_msgs::MoveBaseGoal>("pos", goal_);
  return BT::NodeStatus::SUCCESS;
}

void
SetGoal::callback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
{
  ROS_INFO("Goal setted...........");
  goal_ = *msg;
  goal_.target_pose.header.frame_id = "map";
  ROS_INFO("I recived the message");
  first_msg_recived_ = true;
}

} // namespace follow_BT


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<follow_BT::SetGoal>("Set_goal");
}