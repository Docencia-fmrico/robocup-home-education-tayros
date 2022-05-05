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
#include "taymsgs/person_data.h"
#include <gb_dialog/DialogInterface.h>
#include <dialog_cbs/dialog_cbs.h>
#include <std_msgs/String.h>
#include <string>
#include "std_msgs/Int32.h"

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
      std::bind(&DialogManager::askForNameCB, this, ph::_1),
      "askForName");
  this->registerCallback(
      std::bind(&DialogManager::pointBagDialogCB, this, ph::_1),
      "pointBagDialog");
  this->registerCallback(
      std::bind(&DialogManager::startNavCB, this, ph::_1),
      "readyToMove");

  name_pub_ = nh_.advertise<std_msgs::String>("/taydialog/person/name", 1);
  activation_sub_ = nh_.subscribe<std_msgs::Int32>("/taydialog/name/activation", 1, &DialogManager::callback_activation, this);

  pointedBag_ = "none";
  carReached_ = "false";
  readyToMove_ = "false";
  personName_ = "none";
  questionAsked_ = false;
  activation_ = true;
  name_restart_ = true;
}

void
DialogManager::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
{
}

void
DialogManager::welcomeHumanCML()
{
  ROS_INFO("[TAY_DIALOG] welcomeHumanCB:");
  speak("Hi human. Welcome to Carry My Luggage test.");
  ros::Duration(5, 0).sleep();
  speak("My name is TayBot.");
  ros::Duration(2, 0).sleep();
  speak("I will help you carrying your luggage");
  ros::Duration(4, 0).sleep();
  speak("Please, follow my indications so I can help you doing the task.");
  ros::Duration(6, 0).sleep();
}

std::string
DialogManager::pointBag(int flag)
{
  ROS_INFO("[TAY_DIALOG] pointBag:");
  if(flag == 0)
  {
    speak("Please, point the bag you want me to carry.");
    ros::Duration(4, 0).sleep();
  }
  else if (flag == 1)
  {
    speak("Please, say which bag you want me to carry: left or right?");
    ros::Duration(5, 0).sleep();
  }
  else
  {
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "[TAY_DIALOG] direction: " << pointedBag_ << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    std::string answer = pointedBag_ + "bag selected. Please,put the bag above me so I can carry it.";
    speak(answer);
    ros::Duration(7,0).sleep();
    questionAsked_ = false;
  }
  return pointedBag_;
}

void
DialogManager::pointBagDialogCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] pointBagDialogCB:");
  /*
  --------NOTA IMPORTANTE--------
  Los posibles valores son:
  - left
  - right
  - your left
  - your right
  - my left
  - my right
  */
  pointedBag_ = result.fulfillment_text;
  questionAsked_ = true;
  pointBag(2);
}

std::string
DialogManager::startNav(int flag)
{
  ROS_INFO("[TAY_DIALOG] startNav:");
  if(flag == 0)
  {
    speak("Once your are ready to start the journey, say: START");
    ros::Duration(5, 0).sleep();
  }
  else
  {
    questionAsked_ = false;
  }
  return readyToMove_;
}

void
DialogManager::startNavCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] startNavCB:");
  readyToMove_ = result.fulfillment_text;
  questionAsked_ = true;
  startNav(1);
}

std::string
DialogManager::movementIndications(int flag)
{
  ROS_INFO("[TAY_DIALOG] movementIndicationsCB:");
  if (flag == 0)
  {
    speak("I will start following you. Please do not get to far from me.");
    ros::Duration(5,0).sleep();
    speak("Once we reach the car, please, say: STOP");
    ros::Duration(4,0).sleep();
  }
  else
  {
    speak("Car reached. Please, take the bag. I will return to the spawnpoint soon.");
    ros::Duration(10,0).sleep();
    questionAsked_ = false;
  }
  return carReached_;
}

void
DialogManager::carReachedCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] carReachedCB: intent [%s]", result.intent.c_str());
  carReached_ = result.fulfillment_text;
  questionAsked_ = true;
  movementIndications(1);
}
  
void
DialogManager::gotLostCB()
{
  ROS_INFO("[TAY_DIALOG] gotLostCB:");
  speak("I dont see you. Please , do not move till I find you.");
  ros::Duration(5, 0).sleep();
}

void
DialogManager::welcomeHumanFMM()
{
  ROS_INFO("[TAY_DIALOG] welcomeHumanCB:");
  speak("Hi human. Welcome to Find My Mates test.");
  ros::Duration(5, 0).sleep();
  speak("My name is TayBot.");
  ros::Duration(2, 0).sleep();
  speak("I will help you finding your mates");
  ros::Duration(3, 0).sleep();
  speak("Please, follow my indications so I can help you doing the task.");
  ros::Duration(6, 0).sleep();
  speak("Do not move from here, I will come in a few minutes.");
  ros::Duration(6,0).sleep();
}

void 
DialogManager::callback_activation(const std_msgs::Int32::ConstPtr& activator){
  activation_ = (bool)activator->data;
  name_restart_= (bool)activator->data;
}



std::string
DialogManager::askForName(int flag)
{
  ROS_INFO("[TAY_DIALOG] askForName:");
  if (flag == 0)
  {
    speak("Hi there, what is your name?");
    ros::Duration(3,0).sleep();
  }
  else
  {
    questionAsked_ = false;
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "[TAY_DIALOG] person name is: " << personName_ << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    return personName_;
  }
  return personName_;
}
    
void
DialogManager::askForNameCB(dialogflow_ros_msgs::DialogflowResult result)
{
  if(name_restart_){
    personName_ = "none";
  }
  if(activation_){  
    ROS_INFO("[TAY_DIALOG] askForNameCB:");
    personName_ = result.fulfillment_text;
    questionAsked_ = true;
    std_msgs::String name;
    name.data = personName_;
    name_pub_.publish(name);
  }
}

void
DialogManager::end()
{
    ROS_INFO("[TAY_DIALOG] end");
    speak("Thanks for cosidering me!");
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

std::string
DialogManager::getPersonName()
{
  return personName_;
}


void 
getZone(int num, std::string* zone)
{
  if (num == 1)
  {
    *zone = "Cocina";
  }
}

void
DialogManager::sayPersonData(taymsgs::person_data *person_data)
{
  std::string person_name = "I founded " + person_data->name;
  speak(person_name);
  ros::Duration(4, 0).sleep();

  std::string color_shirt = "The shirt is " + person_data->colorShirt;
  speak(color_shirt);
  ros::Duration(3, 0).sleep();

  std::string object_in_hand = "In the hand ther is a " + person_data->object;
  speak(object_in_hand);
  ros::Duration(4, 0).sleep();

  std::string *zone;
  getZone(person_data->zone, zone);
  std::string zone_sent = "He is in the " + *zone;
  speak(zone_sent);
  ros::Duration(3, 0).sleep();

}

} // namespace gb_dialog
