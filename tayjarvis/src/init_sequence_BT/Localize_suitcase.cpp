#include <string>
#include "init_sequence_BT/Localize_suitcase.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include "dialog_cbs/dialog_cbs.h"

namespace person_recognize
{

Localize_suitcase::Localize_suitcase(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  /* Topic params */
  std::string pub_vel_path = nh_.param("pub_vel_path", std::string("/mobile_base/commands/velocity"));
  std::string act_pub = nh_.param("activation_topic", std::string("/tayros/pointer_detector_activation"));
  std::string side_sub = nh_.param("suticase_side_topic", std::string("/tayros/suitcase_side"));
  std::string bbx_reset = nh_.param("bbx_reset_topic", std::string("/tayros/bbx_reset"));
  
  /* Numeric params */
  waiting_time_ = nh_.param("wait_pointing_time", double(6.0));
  max_requests_ = nh_.param("max_pointing_requests", int(3));
  turning_time_ = nh_.param("max_pointing_requests", int(2));
  turning_velocity_ =nh_.param("max_pointing_requests", float(0.7));

  /* Topic publishers and subscribers*/
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>(pub_vel_path, 1);
  activation_pub_ = nh_.advertise<std_msgs::Int32>(act_pub, 1);
  pub_bbx_restart_ = nh_.advertise<std_msgs::Int32>(bbx_reset, 1);
  side_sub_ = nh_.subscribe<std_msgs::Int32>(side_sub, 1, &Localize_suitcase::side_callback, this);
  
  /* Initializacion values */
  position_ = -1;
  counter_ = 0;
  last_time_ = 0;
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
  current_time_ = ros::Time::now().toSec();
  ROS_INFO("Localizating suitcase......");
  if(current_time_ - waiting_time_ >= last_time_ || counter_ == 0)
  {
    if (counter_ < max_requests_)
    {
      bbx_reset_.data = TRUE;
      activation_.data = TRUE;
      pub_bbx_restart_.publish(bbx_reset_);
    }

    else
    {
      activation_.data = FALSE;
      ROS_INFO("Request per talking");
    }

    last_time_ = current_time_;
    activation_pub_.publish(activation_);

    speaker_.pointBag(0);
    counter_++;
  }

  if (position_ == RIGHT || position_ == LEFT)
  {
    activation_.data = FALSE;
    activation_pub_.publish(activation_);
    turnTo(position_);

    //speaker_.movementIndications();
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
    cmd.angular.z = turning_velocity_;
  }

  if (pos == LEFT)
  {
    ROS_INFO("Turning left");
    cmd.angular.z = -turning_velocity_;
  }
  pub_vel_.publish(cmd);

  sleep(turning_time_);
  cmd.angular.z = 0;
  pub_vel_.publish(cmd);
}



}// namespace person_recognize

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<person_recognize::Localize_suitcase>("Localize_suitcase");
}