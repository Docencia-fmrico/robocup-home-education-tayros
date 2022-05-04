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

#include <ros/ros.h>
#include "person_info/Take_person_info.h"
#include "dialog_cbs/dialog_cbs.h"
#include <string>

namespace tayPersonInfo
{

PersonInfo::PersonInfo()
{
    person_info_sub_ =  nh_.subscribe<taymsgs::person_info>("/tayros/person_info", 10, &PersonInfo::callback_person_info, this);
    person_color_sub_ =  nh_.subscribe<std_msgs::String>("/tayvision/person/color", 10, &PersonInfo::callback_person_color_info, this);
    person_object_sub_ =  nh_.subscribe<std_msgs::String>("/tayvision/person/object_info", 10, &PersonInfo::callback_person_object_info, this);
    person_feedback_pub_ = nh_.advertise<std_msgs::Int32>("/tayros/person/feedback", 1);
    person_taked_ = false;
    person_color_taked = true;
    person_object_taked = true;
    name_taked_ = false;
    
}

void
PersonInfo::callback_person_info(const taymsgs::person_info::ConstPtr& person)
{
    if(! person_taked_)
    {
        current_person_.id = person->id;
        current_person_.zone = person->zone;
        current_person_.goal = person->position;
        person_taked_ = true;
    }
}

void
PersonInfo::callback_person_color_info(const std_msgs::String::ConstPtr& color)
{
    if(! person_color_taked)
    {
        current_person_.colorShirt = color->data;
        person_color_taked = true;
    }
}

void
PersonInfo::callback_person_object_info(const std_msgs::String::ConstPtr& object)
{
    if(! person_object_taked)
    {
        current_person_.object = object->data;
        person_object_taked = true;
    }
}


bool
PersonInfo::getPersonName()
{
    gb_dialog::DialogManager player;
    current_person_.name = player.askForName(0);
    return true;
}

void
PersonInfo::step()
{
    if(person_taked_)
    {
        // Publicar goal 

        // If goal succes 
        person_color_taked = false;
        person_object_taked = false;
        name_taked_ = getPersonName();

        if(person_color_taked && person_object_taked && name_taked_)
        {   
            std_msgs::Int32 feedback_id;
            feedback_id.data = current_person_.id;
            person_feedback_pub_.publish(feedback_id);

            std::cout << "Name: " << current_person_.name << std::endl;
            std::cout << "Color: " << current_person_.colorShirt << std::endl;
            std::cout << "Object: " << current_person_.object << std::endl;
            std::cout << "Zone: " << current_person_.zone << std::endl;
            std::cout << "ID: " << current_person_.id << std::endl;

            // Envias info a BT 

            // Bt indica que ya ha comentado la info
            sleep(5);
            if(true)
            {
                person_taked_ = false;
            }
            
        }


    }
}


} // namespace tayPersonInfo