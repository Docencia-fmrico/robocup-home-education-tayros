
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "back_home_BT/Move.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace Back_home
{

Move::Move(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), counter_(0)
{
}

void
Move::on_halt()
{
  ROS_INFO("Move halt");
}

void
Move::on_start()
{
  move_base_msgs::MoveBaseGoal Target = getInput<move_base_msgs::MoveBaseGoal>("pos").value();
  Target.target_pose.header.stamp = ros::Time::now();
  set_goal(Target);
}

bool
Move::is_same_goal(move_base_msgs::MoveBaseGoal goal)
{
  if (goal.target_pose.pose.position.x != posX_)
  {
    return false;
  }
  if (goal.target_pose.pose.position.y != posY_)
  {
    return false;
  }
  if (goal.target_pose.pose.position.z != posZ_)
  {
    return false;
  }
  
  if (goal.target_pose.pose.orientation.x != angularX_)
  {
    return false;
  }
  if (goal.target_pose.pose.orientation.x != angularY_)
  {
    return false;
  }
  if (goal.target_pose.pose.orientation.x != angularZ_)
  {
    return false;
  }
  if (goal.target_pose.pose.orientation.x != angularW_)
  {
    return false;
  }

  return true;
}

BT::NodeStatus
Move::on_tick()
{
  
  move_base_msgs::MoveBaseGoal Target = getInput<move_base_msgs::MoveBaseGoal>("pos").value();

  if (is_same_goal(Target))
  {
    std::cerr << "New_goal........" << std::endl;
    Target.target_pose.header.stamp = ros::Time::now();
  
    posX_ = Target.target_pose.pose.position.x;
    posY_ = Target.target_pose.pose.position.y;
    posZ_ = Target.target_pose.pose.position.z;
    angularX_ = Target.target_pose.pose.orientation.x;
    angularY_ = Target.target_pose.pose.orientation.y;
    angularZ_ = Target.target_pose.pose.orientation.z;
    angularW_ = Target.target_pose.pose.orientation.w;

    set_goal(Target);
  }
  return BT::NodeStatus::RUNNING;
}

void
Move::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//ROS_INFO("Coordinates: X -> %lf,  Y -> %lf", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);
}

}  // namespace Back_home

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<Back_home::Move>(
        name, "move_base", config);
    };

  factory.registerBuilder<Back_home::Move>(
    "Move", builder);
}
