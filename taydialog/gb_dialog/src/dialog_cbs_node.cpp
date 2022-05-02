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

#include "dialog_cbs/dialog_cbs.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dialog_cbs_node");

  gb_dialog::DialogManager forwarder;
  // No quitar este sleep que si no NO DA TIEMPO AL CONSTRUCTOR
  ros::Duration(1, 0).sleep();

  forwarder.welcomeHuman();
  forwarder.pointBag(1);
  forwarder.movementIndications();
  forwarder.end();
  
  ros::spin();
  return 0;
}