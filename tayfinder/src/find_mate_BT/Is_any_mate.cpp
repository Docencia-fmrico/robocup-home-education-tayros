#include <string>
#include "find_mate_BT/Is_any_mate.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <unistd.h>

enum
{
  STOP = 0,
  GO = 1,
};

namespace find_mate
{

IsAnyMate::IsAnyMate(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  std::string mov_publisher =  nh_.param("movement_topic", std::string("/tayros/movement"));
  mov_pub_ = nh_.advertise<std_msgs::Int32>(mov_publisher, 1);

  pos_sub_ =  nh_.subscribe<move_base_msgs::MoveBaseGoal>("/tayfinder/person/goal", 1, &IsAnyMate::pos_callback, this);
  cord_received_ = false;
}

void
IsAnyMate::halt()
{
}

BT::NodeStatus
IsAnyMate::tick()
{
  if (cord_received_)
  {
    setOutput<move_base_msgs::MoveBaseGoal>("pos", goal_);
    move_.data = STOP;
    mov_pub_.publish(move_);
    cord_received_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}  // namespace find_mate

void 
IsAnyMate::pos_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
{
  goal_ = *msg;
  cord_received_ = true;
}

}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_mate::IsAnyMate>("Is_any_mate");
}