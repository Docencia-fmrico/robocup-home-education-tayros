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

#ifndef INIT_SEQUENCE_WELCOME_HUMAN_H
#define INIT_SEQUENCE_WELCOME_HUMAN_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "dialog_cbs/dialog_cbs.h"
#include <string>
#include "std_msgs/Int32.h"
#include "ros/ros.h"

namespace person_recognize
{

class Welcome_human : public BT::ActionNodeBase
{
  public:
    explicit Welcome_human(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { };
    }

  private: 
    ros::NodeHandle nh_;
    bool first_execute_ = true;
    gb_dialog::DialogManager speaker;
};

}  // namespace PERSON_RECOGNIZE

#endif  // INIT_SEQUENCE_WELCOME_HUMAN_H