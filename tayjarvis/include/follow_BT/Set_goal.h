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

#ifndef FOLLOW_BT_SET_GOAL_H
#define FOLLOW_BT_SET_GOAL_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/Int32.h"
#include <string>

#include "ros/ros.h"

namespace follow_BT
{

class SetGoal : public BT::ActionNodeBase
{
  public:
    explicit SetGoal(const std::string& name, 
    const BT::NodeConfiguration& config);

    void halt() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("pos")};
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher act_pub_;
    ros::Publisher bbx_act_pub_;
    ros::Subscriber pos_sub_;

    void callback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg);

    move_base_msgs::MoveBaseGoal goal_;

    bool first_msg_recived_;
    float counter;
};

}  // namespace FOLLOW_BT

#endif  // FOLLOW_BT_SET_GOAL_H