// Copyright 2022 TayRos
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

#ifndef PERSON_INFO_BT_TAKE_PERSON_INFO_H
#define PERSON_INFO_BT_TAKE_PERSON_INFO_H

#include <string>
#include "person_localizator/PersonLocalizator.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "taymsgs/person_info.h"
#include "std_msgs/String.h"
#include "ros/ros.h"

namespace tayPersonInfo
{

typedef struct
{
    std::string name;
    std::string colorShirt;
    std::string object;
    int zone;
    int id;
    move_base_msgs::MoveBaseGoal goal;
}t_personInfo;

class PersonInfo
{
public:
    PersonInfo();

    void callback_person_info(const taymsgs::person_info::ConstPtr& person);
    void callback_person_color_info(const std_msgs::String::ConstPtr& color);
    void callback_person_object_info(const std_msgs::String::ConstPtr& object);
    bool getPersonName();
    void step();

private:
    ros::NodeHandle nh_;
    ros::Subscriber person_info_sub_;
    ros::Subscriber person_color_sub_;
    ros::Subscriber person_object_sub_;
    ros::Publisher person_feedback_pub_;

    t_personInfo current_person_;

    bool person_taked_;

    bool person_color_taked;
    bool person_object_taked;
    bool name_taked_;
};

} // namespace tayPersonInfo

#endif  // PERSON_INFO_BT_TAKE_PERSON_INFO_H