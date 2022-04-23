#include <string>
#include "find_mate_BT/Bump_go.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include <unistd.h>

enum
{
  STOP = 0,
  GO = 1,
};

namespace find_mate
{

Bump_go::Bump_go(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), nh_("~")
{
  std::string mov_publisher =  nh_.param("movement_topic", std::string("/tayros/movement"));
  mov_pub_ = nh_.advertise<std_msgs::Int32>(mov_publisher, 1);
}

void
Bump_go::halt()
{
}

BT::NodeStatus
Bump_go::tick()
{
  ROS_INFO("Bump gooooo");
  move_.data = GO;
  mov_pub_.publish(move_);
  sleep(4);
  return BT::NodeStatus::SUCCESS;
}  // namespace find_mate

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_mate::Bump_go>("Bump_go");
}