#include <string>
#include "init_sequence_BT/Localize_suitcase.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include "dialog_cbs/dialog_cbs.h"

enum
{
  GO = 0,
  STOP = 1, 
  LITSEN = 0,
  TALKING = 1,
};

namespace person_recognize
{

Localize_suitcase::Localize_suitcase(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  std::string pub_vel_path = nh_.param("pub_vel_path", std::string("/mobile_base/commands/velocity"));
  std::string act_pub = nh_.param("activation_topic", std::string("/tayros/pointer_detector_activation"));
  std::string side_sub = nh_.param("suticase_side_topic", std::string("/tayros/suitcase_side"));
  std::string bbx_reset = nh_.param("bbx_reset_topic", std::string("/tayros/bbx_reset"));

  pub_vel_ = nh_.advertise<geometry_msgs::Twist>(pub_vel_path, 1);
  activation_pub_ = nh_.advertise<std_msgs::Int32>(act_pub, 1);
  pub_bbx_restart_ = nh_.advertise<std_msgs::Int32>(bbx_reset, 1);
  side_sub_ = nh_.subscribe<std_msgs::Int32>(side_sub, 1, &Localize_suitcase::side_callback, this);
  
  position_ = -1;
}

void 
Localize_suitcase::side_callback(const std_msgs::Int32::ConstPtr& msg)
{
  position_ = msg->data;
}

void
Localize_suitcase::halt()
{
}


BT::NodeStatus
Localize_suitcase::tick()
{
  std_msgs::Int32 state, bbx_reset;
  state.data = GO;
  ROS_INFO("Empezando....");
  sleep(4);

  bbx_reset.data = TRUE;
  pub_bbx_restart_.publish(bbx_reset);
  speaker_.pointBag();
  ROS_INFO("Mantengase quieto");
  sleep(4);

  bbx_reset.data = FALSE;
  pub_bbx_restart_.publish(bbx_reset);
  ROS_INFO("Apnte donde este la maleta %d!!!!", position_);
  activation_pub_.publish(state);

  if (position_ == RIGHT || position_ == LEFT)
  {
    turnTo(position_);
    state.data = STOP;
    activation_pub_.publish(state);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;  
}  

void
Localize_suitcase::turnTo(int pos)
{
  geometry_msgs::Twist cmd;
  if (pos == RIGHT)
  {
    ROS_INFO("Turning right");
    cmd.angular.z = 0.7;
  }

  if (pos == LEFT)
  {
    ROS_INFO("Turning left");
    cmd.angular.z = 0.7;
  }
  pub_vel_.publish(cmd);

  sleep(4);
  cmd.angular.z = 0;
  pub_vel_.publish(cmd);
}



}// namespace person_recognize

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<person_recognize::Localize_suitcase>("Localize_suitcase");
}