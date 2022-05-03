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

#ifndef INIT_SEQUENCE_LOCALIZE_SUITCASE_H
#define INIT_SEQUENCE_LOCALIZE_SUITCASE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include "dialog_cbs/dialog_cbs.h"

namespace person_recognize
{

class Localize_suitcase : public BT::ActionNodeBase
{
  public:
    explicit Localize_suitcase(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;
    void side_callback(const std_msgs::Int32::ConstPtr& msg);
    void turnTo(int pos);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { };
    }

  private: 
    ros::NodeHandle nh_;
    ros::Publisher activation_pub_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_bbx_restart_;
    ros::Subscriber side_sub_;

    gb_dialog::DialogManager speaker_;
    std_msgs::Int32 activation_, bbx_reset_;
    double current_time_, last_time_, waiting_time_;

    int position_;
    int counter_, max_requests_;
    int turning_time_;
    float turning_velocity_;
    const int RIGHT = 0;
    const int LEFT = 1;
    const int TRUE = 1;
    const int FALSE = 0;
};

}  // namespace PERSON_RECOGNIZE

#endif  // INIT_SEQUENCE_LOCALIZE_SUITCASE_H