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

#ifndef BACK_HOME_BT_SAY_DATA_H
#define BACK_HOME_BT_SAY_DATA_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/Bool.h"
#include <string>
#include "dialog_cbs/dialog_cbs.h"
#include "ros/ros.h"
#include "taymsgs/person_data.h"

namespace Back_home
{

class Say_data : public BT::ActionNodeBase
{
  public:
    explicit Say_data(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<taymsgs::person_data>("data")};
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher data_comunicated_pub_;

    gb_dialog::DialogManager speaker_;
    std_msgs::Bool comunicated_;
    taymsgs::person_data data_;
};

}  // namespace Back_home

#endif  // BACK_HOME_BT_SAY_DATA_H