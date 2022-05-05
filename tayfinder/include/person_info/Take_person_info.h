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
#include "std_msgs/Bool.h"
#include "ros/ros.h"

namespace tayPersonInfo
{

enum{
    PERSON_BUFFER = 4,
};

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
    void callback_person_name(const std_msgs::String::ConstPtr& name);
    void callback_mate_reached(const std_msgs::Bool::ConstPtr& reached);
    void callback_data_comunicated(const std_msgs::Bool::ConstPtr& data_comunicated);

    bool getPersonName();
    void step();
    bool is_id_studied(int id);
private:
    ros::NodeHandle nh_;
    ros::Subscriber person_info_sub_;
    ros::Subscriber person_color_sub_;
    ros::Subscriber person_object_sub_;
    ros::Subscriber person_info_name_;
    ros::Publisher person_feedback_pub_;
    ros::Publisher color_activation_pub_;
    ros::Publisher object_activation_pub_;
    ros::Publisher name_activation_pub_;

    ros::Publisher person_goal_pub_;
    ros::Subscriber goal_reached_sub_;

    ros::Publisher person_data_pub_;
    ros::Subscriber data_comunicated_sub_;

    int id_studied[PERSON_BUFFER];

    t_personInfo current_person_;

    bool person_taked_;
    bool first_time_;
    
    const float MAX_TAKE_DATA_TIME = 60;
    float init_take_info_time_;

    bool person_color_taked;
    bool person_object_taked;
    bool name_taked_;
    bool nav_succes_;
    bool say_data_succes_;
};

} // namespace tayPersonInfo

#endif  // PERSON_INFO_BT_TAKE_PERSON_INFO_H