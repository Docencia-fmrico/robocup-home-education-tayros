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

#ifndef FOLLOW_BT_TARGET_REACHED
#define FOLLOW_BT_TARGET_REACHED

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "dialog_cbs/dialog_cbs.h"
#include "follow_BT/BTNavAction.h"
#include "std_msgs/Int32.h"
#include <string>

#include "ros/ros.h"

namespace follow_BT
{

class target_reached : public BT::ActionNodeBase
{
  public:
    explicit target_reached(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {};
    }


  private:
    ros::NodeHandle nh_;
    ros::Publisher act_pub_;
    gb_dialog::DialogManager litsener;
    int first_execute = 0;
};

}  // namespace FOLLOW_BT

#endif  // FOLLOW_BT_TARGET_REACHED_BT