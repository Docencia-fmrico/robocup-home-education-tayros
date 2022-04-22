#include <string>
#include "init_sequence_BT/Recognize.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"

enum{
  FRONT = 0,
  RIGHT_SIDE = 1,
  LEFT_SIDE = 2,
  BACK = 3,
  END = 4,

  TIME_SLEEP = 3,
};

namespace person_recognize
{

Recognize::Recognize(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
    mode_.data = FRONT; 
    pub_ = nh_.advertise<std_msgs::Int32>("/tay/mode_recognition", 1);
}

void
Recognize::halt()
{
}

void
Recognize::speak(std::string prhase)
{
  ROS_INFO("%s", prhase.c_str());
}

BT::NodeStatus
Recognize::tick()
{
  if (mode_.data == END)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    pub_.publish(mode_);
    speak(sentences_[mode_.data++]);
    sleep(TIME_SLEEP);
  }
  return BT::NodeStatus::FAILURE;
}  

}// namespace person_recognize

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<person_recognize::Recognize>("recognize");
}