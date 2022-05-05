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

#include <string>
#include <memory>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "dialog_cbs/dialog_cbs.h"
#include "ros/package.h"

enum
{
  RECOGNIZE = 0,
  FOLLOWING = 1,
  GOING_HOME = 2,
};

void
get_home_pose(ros::NodeHandle n, move_base_msgs::MoveBaseGoal *home)
{
  home->target_pose.header.frame_id = "map";
  home->target_pose.pose.position.x = n.param("pos_x0_home" , 0.0);
  home->target_pose.pose.position.y = n.param("pos_y0_home", 0.0);
  home->target_pose.pose.position.z = n.param("pos_z0_home", 0.0);
  home->target_pose.pose.orientation.x = n.param("orient_x0_home", 0.0);
  home->target_pose.pose.orientation.y = n.param("orient_y0_home", 0.0);
  home->target_pose.pose.orientation.z = n.param("orient_z0_home", 0.0);
  home->target_pose.pose.orientation.w = n.param("orient_w0_home", 1.0);
}

void
get_arb_pose(ros::NodeHandle n, move_base_msgs::MoveBaseGoal *arb)
{
  arb->target_pose.header.frame_id = "map";
  arb->target_pose.pose.position.x = n.param("pos_x0_arb" , 0.0);
  arb->target_pose.pose.position.y = n.param("pos_y0_arb", 0.0);
  arb->target_pose.pose.position.z = n.param("pos_z0_arb", 0.0);
  arb->target_pose.pose.orientation.x = n.param("orient_x0_arb", 0.0);
  arb->target_pose.pose.orientation.y = n.param("orient_y0_arb", 0.0);
  arb->target_pose.pose.orientation.z = n.param("orient_z0_arb", 0.0);
  arb->target_pose.pose.orientation.w = n.param("orient_w0_arb", 1.0);
}

int main(int argc, char **argv) 
{
  /* BT creation */
  ros::init(argc, argv, "suitcases_loader");
  ros::NodeHandle n("~");


  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("BT_move_node"));
  factory.registerFromPlugin(loader.getOSName("BT_follow"));
  factory.registerFromPlugin(loader.getOSName("BT_target_reached"));
  factory.registerFromPlugin(loader.getOSName("BT_recognize"));
  factory.registerFromPlugin(loader.getOSName("BT_set_goal"));

  //factory.registerFromPlugin(loader.getOSName("BT_recognize"));
  factory.registerFromPlugin(loader.getOSName("BT_welcome_human"));
  factory.registerFromPlugin(loader.getOSName("BT_localize_suitcase"));

  std::string pkgpath = ros::package::getPath("tayjarvis");
  std::string follow_path = pkgpath + "/behavior_trees_xml/follow.xml";
  std::string back_home_path = pkgpath + "/behavior_trees_xml/back_home.xml";
  std::string recognize_path = pkgpath + "/behavior_trees_xml/init_sequence.xml";

  /* Blackboard set */
  move_base_msgs::MoveBaseGoal home;
  move_base_msgs::MoveBaseGoal arb;
  get_home_pose(n, &home);
  get_arb_pose(n, &arb);
  


  auto blackboard = BT::Blackboard::create();
  blackboard->set("Home", home);
  blackboard->set("goal", home);
  blackboard->set("arb", arb);

  BT::Tree follow = factory.createTreeFromFile(follow_path, blackboard);
  BT::Tree back_home = factory.createTreeFromFile(back_home_path, blackboard);
  BT::Tree recognition = factory.createTreeFromFile(recognize_path, blackboard);
  gb_dialog::DialogManager speaker;
  
  //auto publisher_zmq1 = std::make_shared<BT::PublisherZMQ>(follow, 10, 1666, 1667);
  /* Neccesary to use groot */

  ros::Rate loop_rate(10);

  int state = RECOGNIZE;
  bool finish = false;

  /* Bt sequence */
  while (ros::ok() && !finish)
  {
    switch (state)
    {
    case RECOGNIZE:
      if (recognition.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        state = FOLLOWING;     
        //finish = true;
      }
      break;

    case FOLLOWING:
      if (follow.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        state = GOING_HOME;
      //  finish = true;
      }
      break;
    
    case GOING_HOME:
     if (back_home.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        finish = true;
      }
      break;

    default:
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
