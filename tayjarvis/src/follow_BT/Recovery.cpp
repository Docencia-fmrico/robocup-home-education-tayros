
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

#include "follow_BT/Recovery.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace follow_BT
{

Recovery::Recovery(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), counter_(0)
{
}

void
Recovery::on_halt()
{
}

void
Recovery::on_start()
{
  move_base_msgs::MoveBaseGoal Target = getInput<move_base_msgs::MoveBaseGoal>("pos").value();
  Target.target_pose.header.stamp = ros::Time::now();
  set_goal(Target);
}

BT::NodeStatus
Recovery::on_tick()
{
  
  move_base_msgs::MoveBaseGoal Target = getInput<move_base_msgs::MoveBaseGoal>("pos").value();

  ROS_INFO("QUITEME EL OBSTACULO PORFAVOT");
  sleep(2);
  set_goal(Target);

  return BT::NodeStatus::RUNNING;
}

void
Recovery::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//ROS_INFO("Coordinates: X -> %lf,  Y -> %lf", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);
}

}  // namespace follow_BT

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<follow_BT::Recovery>(
        name, "Recovery", config);
    };

  factory.registerBuilder<follow_BT::Recovery>(
    "Recovery", builder);
}
