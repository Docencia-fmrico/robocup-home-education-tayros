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

#ifndef FIND_MATE_BT_GET_MATE_DATA_H
#define FIND_MATE_BT_GET_MATE_DATA_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <string>

#include "ros/ros.h"

namespace find_mate
{

class Get_mate_data : public BT::ActionNodeBase
{
  public:
    explicit Get_mate_data(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<int>("data")};
    }

  private:
    ros::NodeHandle nh_;
};

}  // namespace find_mate

#endif  // FIND_MATE_BT_GET_MATE_DATA_H