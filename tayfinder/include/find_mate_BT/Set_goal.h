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

#ifndef FIND_MATE_BT_SET_GOAL_H
#define FIND_MATE_BT_SET_GOAL_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <string>

#include "ros/ros.h"

namespace find_mate
{

class SetGoal : public BT::ActionNodeBase
{
  public:
    explicit SetGoal(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("pos")};
    }

  private:
    ros::NodeHandle nh_;
};

}  // namespace find_mate

#endif  // FIND_MATE_H_SET_GOAL_H