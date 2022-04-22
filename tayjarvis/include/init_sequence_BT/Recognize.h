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

#ifndef PERSON_RECOGNIZE_RECOGNIZE_H
#define PERSON_RECOGNIZE_RECOGNIZE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include "std_msgs/Int32.h"
#include "ros/ros.h"

namespace person_recognize
{

class Recognize : public BT::ActionNodeBase
{
  public:
    explicit Recognize(const std::string& name, const BT::NodeConfiguration& config);

    void halt() override;

    void speak(std::string prhase);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("mode")};
    }

  private: 

    std::string sentences_[4] = {"Ten huevos a mirarme a los ojos", 
                "Eso es mira hacia otro lado cuando te hablo",
                "Si, si por ese otro lado te puedes ir",
                "Ahora me das la espalda puto????" };

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std_msgs::Int32 mode_;
};

}  // namespace PERSON_RECOGNIZE

#endif  // PERSON_RECOGNIZE_RECOGNIZE_H