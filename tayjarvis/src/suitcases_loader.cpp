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

#include "ros/package.h"

enum
{
  RECOGNIZE = 0,
  FOLLOWING = 1,
  GOING_HOME = 2,
};

int main(int argc, char **argv) 
{
  /* BT creation */
  ros::init(argc, argv, "suitcases_loader");
  ros::NodeHandle n("~");


  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("BT_move_node"));
  factory.registerFromPlugin(loader.getOSName("BT_set_goal"));
  factory.registerFromPlugin(loader.getOSName("BT_target_reached"));

  //factory.registerFromPlugin(loader.getOSName("BT_recognize"));
  factory.registerFromPlugin(loader.getOSName("BT_welcome_human"));
  factory.registerFromPlugin(loader.getOSName("BT_localize_suitcase"));

  std::string pkgpath = ros::package::getPath("tayjarvis");
  std::string follow_path = pkgpath + "/behavior_trees_xml/follow.xml";
  std::string back_home_path = pkgpath + "/behavior_trees_xml/back_home.xml";
  std::string recognize_path = pkgpath + "/behavior_trees_xml/init_sequence.xml";

  /* Blackboard set */
  move_base_msgs::MoveBaseGoal home;

  home.target_pose.header.frame_id = "map";
  home.target_pose.pose.position.x = n.param("pos_x0" , 0.0);
  home.target_pose.pose.position.y = n.param("pos_y0", 0.0);
  home.target_pose.pose.position.z = n.param("pos_z0", 0.0);
  home.target_pose.pose.orientation.x = n.param("orient_x0", 0.0);
  home.target_pose.pose.orientation.y = n.param("orient_y0", 0.0);
  home.target_pose.pose.orientation.z = n.param("orient_z0", 0.0);
  home.target_pose.pose.orientation.w = n.param("orient_w0", 1.0);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("Home", home);
  blackboard->set("goal", home);

  BT::Tree follow = factory.createTreeFromFile(follow_path, blackboard);
  BT::Tree back_home = factory.createTreeFromFile(back_home_path, blackboard);
  BT::Tree recognition = factory.createTreeFromFile(recognize_path, blackboard);
  
  //auto publisher_zmq1 = std::make_shared<BT::PublisherZMQ>(follow, 10, 1666, 1667);
  /* Neccesary to use groot */

  ros::Rate loop_rate(10);

  int state = GOING_HOME;
  bool finish = false;

  /* Bt sequence */
  while (ros::ok() && !finish)
  {
    switch (state)
    {
    case RECOGNIZE:
      if (recognition.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        //state = FOLLOWING;     
        finish = true;
      }
      break;

    case FOLLOWING:
      if (follow.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        //state = GOING_HOME;      
        finish = true;
      }
      break;
    
    case GOING_HOME:
     if (back_home.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        ROS_INFO("Prueba terminada!!!!!!!!!!!!!!");
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
