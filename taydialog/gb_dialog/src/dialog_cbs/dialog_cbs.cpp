/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics Labs
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Gines jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */

// MODIFIED BY 2022 TayRos. Rey Juan Carlos University, Software Robotics Eng.
#include <gb_dialog/DialogInterface.h>
#include <dialog_cbs/dialog_cbs.h>
#include <string>

namespace ph = std::placeholders;

namespace gb_dialog
{
DialogManager::DialogManager()
: nh_()
{
  this->registerCallback(std::bind(&DialogManager::noIntentCB, this, ph::_1));
  this->registerCallback(
      std::bind(&DialogManager::carReachedCB, this, ph::_1),
      "carReached");
  this->registerCallback(
      std::bind(&DialogManager::pointBagDialogCB, this, ph::_1),
      "pointBagDialog");
  pointedBag_ = "none";
  carReached_ = false;
}

void
DialogManager::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] noIntentCB: intent [%s]", result.intent.c_str());
}

void
DialogManager::welcomeHuman()
{
  ROS_INFO("[TAY_DIALOG] welcomeHumanCB:");
  speak("Hi_human._Welcome_to_Carry_My_Luggage_Test.");
  ros::Duration(5, 0).sleep();
  speak("My_name_is_TayBot.");
  ros::Duration(2, 0).sleep();
  speak("I_will_help_you_carrying_your_lugagge");
  ros::Duration(4, 0).sleep();
  speak("Please,_follow_my_indications_so_I_can_help_you_doing_the_task.");
  ros::Duration(6, 0).sleep();
}

void
DialogManager::pointBag()
{
  ROS_INFO("[TAY_DIALOG] pointBag:");
  speak("Please,_point_the_bag_you_want_me_to_carry.");
  ros::Duration(4, 0).sleep();
  while(ros::ok())
  {
    listen();
    std::cerr << "[TAY_DIALOG] direction: " << pointedBag_ << std::endl;
    if(pointedBag_ == "none")
    {
      speak("I_didn't_understand_you._Which_one_did_you_say:_the_one_from_my_left_or_the_one_from_my_right?");
      ros::Duration(7,0).sleep();
    }
    else
    {
      break;
    }
    ros::spinOnce();
  }
  std::string answer = pointedBag_ + "bag_selected.Please,_put_your_bag_above_me_so_I_can_carry_it.";
  speak(answer);
  ros::Duration(10,0).sleep();
}

void
DialogManager::pointBagDialogCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] pointBagDialogCB:");
  for (const auto & param : result.parameters)
  {
    std::cout << "THIS IS THE PARAM:" << param << std::endl;
    for(const auto & value : param.value)
    {
      std::cout << "POINTED BAG ES: " << pointedBag_ << std::endl;
      std::cout << "\t" << "THIS IS THE VALUE:" << value << std::endl;
    }
  }
  pointedBag_ = result.fulfillment_text;
}

void
DialogManager::movementIndications()
{
  ROS_INFO("[TAY_DIALOG] movementIndicationsCB:");
  speak("I_will_start_following_you.Please_do_not_get_to_far_from_me.");
  ros::Duration(5,0).sleep();
  speak("Once_we_reach_the_car,_please,_say:_STOP");
  ros::Duration(4,0).sleep();
  while(ros::ok())
  {
    listen();
    ros::Duration(3,0).sleep();
    if(isCarReached() == "true")
    {
      break;
    }
    ros::spinOnce();
  }
  speak("Car_reached._Please,_take_the_bag._I_will_return_to_the_spawnpoint_soon.");
  ros::Duration(6,0).sleep();
}

void
DialogManager::carReachedCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] carReachedCB: intent [%s]", result.intent.c_str());
  carReached_ = result.fulfillment_text;
}
  
void
DialogManager::gotLostCB()
{
  ROS_INFO("[TAY_DIALOG] gotLostCB:");
  speak("I_dont_see_you._Please_,_do_not_move_till_I_find_you.");
  ros::Duration(5, 0).sleep();
}

void
DialogManager::end()
{
    ROS_INFO("[TAY_DIALOG] end");
    speak("THANKS_FOR_CONSIDERING_ME._I_HOPE_I_WILL_SEE_YOU_SOON");
}

std::string
DialogManager::getPointedBag()
{
  return pointedBag_;
}

std::string
DialogManager::isCarReached()
{
  return carReached_;
}

} // namespace gb_dialog
