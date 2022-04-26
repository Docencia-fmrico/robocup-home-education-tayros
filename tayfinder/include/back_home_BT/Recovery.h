
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

#ifndef BACK_HOME_BT_RECOVERY_H
#define BACK_HOME_BT_RECOVERY_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <move_base_msgs/MoveBaseAction.h>
#include "back_home_BT/BTNavAction.h"

#include <string>

namespace Back_home
{

class Recovery : public BTNavAction
{
  public:
    explicit Recovery(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

    static BT::PortsList providedPorts() {
      return {BT::InputPort<move_base_msgs::MoveBaseGoal>("pos")};
    }

  private:

    float time_sleep_;
    int counter_;
};

}  // namespace back_home

#endif  // BACK_HOME_BT_RECOVERY_H
