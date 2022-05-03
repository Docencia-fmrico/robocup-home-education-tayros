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
      std::bind(&DialogManager::askForNameCB, this, ph::_1),
      "askForName");
  this->registerCallback(
      std::bind(&DialogManager::pointBagDialogCB, this, ph::_1),
      "pointBagDialog");
  this->registerCallback(
      std::bind(&DialogManager::startNavCB, this, ph::_1),
      "readyToMove");
  pointedBag_ = "none";
  carReached_ = "false";
  readyToMove_ = "false";
  personName_ = "none";
  questionAsked_ = false;
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

void
DialogManager::pointBag(int listenFlag)
{
  ROS_INFO("[TAY_DIALOG] pointBag:");
  if(listenFlag == 0)
  {
      speak("Please, point the bag you want me to carry.");
    ros::Duration(4, 0).sleep();
  }
  else
  {
    speak("Please, say which bag you want me to carry: left or right?");
    ros::Duration(5, 0).sleep();
    while(ros::ok())
    {
      if(!questionAsked_)
      {
        listen();
      }
      if(pointedBag_ != "none")
      {
        break;
      }
      ros::spinOnce();
    }
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "[TAY_DIALOG] direction: " << pointedBag_ << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    std::string answer = pointedBag_ + "bag selected. Please,put the bag above me so I can carry it.";
    speak(answer);
    ros::Duration(7,0).sleep();
    questionAsked_ = false;
  }
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
}

void
DialogManager::startNav()
{
  ROS_INFO("[TAY_DIALOG] startNav:");
  speak("Once your are ready to start the journey, say: START");
  ros::Duration(5, 0).sleep();
  while(ros::ok())
  {
    if(!questionAsked_)
    {
      listen();
    }
    if(readyToMove_ == "true")
    {
      break;
    }
      ros::spinOnce();
  }
  questionAsked_ = false;
}

void
DialogManager::startNavCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] startNavCB:");
  readyToMove_ = result.fulfillment_text;
  questionAsked_ = true;
}

void
DialogManager::movementIndications()
{
  ROS_INFO("[TAY_DIALOG] movementIndicationsCB:");
  speak("I will start following you. Please do not get to far from me.");
  ros::Duration(5,0).sleep();
  speak("Once we reach the car, please, say: STOP");
  ros::Duration(4,0).sleep();
  while(ros::ok())
  {
    if(!questionAsked_)
    {
      listen();
    }
    if(carReached_ == "true")
    {
      break;
    }
    ros::spinOnce();
  }
  speak("Car reached. Please, take the bag. I will return to the spawnpoint soon.");
  ros::Duration(10,0).sleep();
  questionAsked_ = false;
}

void
DialogManager::carReachedCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] carReachedCB: intent [%s]", result.intent.c_str());
  carReached_ = result.fulfillment_text;
  questionAsked_ = true;
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
DialogManager::askForName()
{
  ROS_INFO("[TAY_DIALOG] askForName:");
  speak("Hi there, what is your name?");
  ros::Duration(3,0).sleep();
  while(ros::ok())
  {
    if(!questionAsked_)
    {
      listen();
    }
    if(personName_ != "none")
    {
      break;
    }
    ros::spinOnce();
  }
  questionAsked_ = false;
  std::cout << "-------------------------------------------------" << std::endl;
  std::cout << "[TAY_DIALOG] person name is: " << personName_ << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;
}
    
void
DialogManager::askForNameCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] askForNameCB:");
  personName_ = result.fulfillment_text;
  questionAsked_ = true;
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

} // namespace gb_dialog
