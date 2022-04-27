
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

#ifndef FOLLOW_BT_RECOVERY_H
#define FOLLOW_BT_RECOVERY_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
//#include "follow_BT/BTNavAction.h"

#include <string>

namespace follow_BT
{

class Recovery : public BT::ActionNodeBase//public BTNavAction
{
  public:
    explicit Recovery(const std::string& name,
    const BT::NodeConfiguration & config);

    void halt() ;
    BT::NodeStatus tick() ;
    //void start() ;
   // void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

    static BT::PortsList providedPorts() {
      return {};
    }

  private:

    float time_sleep_;
    int counter_;
};

}  // namespace follow_BT

#endif  // FOLLOW_BT_RECOVERY_H
