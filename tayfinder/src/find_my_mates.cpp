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
#include "dialog_cbs/dialog_cbs.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "taymsgs/person_data.h"
#include "ros/package.h"

enum
{
  FIND_MATE = 1,
  INFORM_REFEREE = 2,
};

void
get_arb_pose(ros::NodeHandle n, move_base_msgs::MoveBaseGoal *home)
{
  home->target_pose.header.frame_id = "map";
  home->target_pose.pose.position.x = n.param("pos_x0_home" , 0.99);
  home->target_pose.pose.position.y = n.param("pos_y0_home", 4.99);
  home->target_pose.pose.position.z = n.param("pos_z0_home", 0.0);
  home->target_pose.pose.orientation.x = n.param("orient_x0_home", 0.0);
  home->target_pose.pose.orientation.y = n.param("orient_y0_home", 0.0);
  home->target_pose.pose.orientation.z = n.param("orient_z0_home", 0.0);
  home->target_pose.pose.orientation.w = n.param("orient_w0_home", 1.0);
}

int main(int argc, char **argv) 
{
  /* BT creation */
  ros::init(argc, argv, "kobfinder");
  ros::NodeHandle n("~");
  int num_persons = n.param("num_persons" , 1);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("BT_move_node_finder"));
  factory.registerFromPlugin(loader.getOSName("BT_is_any_mate_finder"));
  factory.registerFromPlugin(loader.getOSName("BT_get_mate_data_finder"));
  factory.registerFromPlugin(loader.getOSName("BT_say_data_finder"));
  factory.registerFromPlugin(loader.getOSName("BT_bump_go_finder"));
  factory.registerFromPlugin(loader.getOSName("BT_recovery_node_finder"));

  std::string pkgpath = ros::package::getPath("tayfinder");
  std::string find_mate_path = pkgpath + "/behavior_trees_xml/find_mate.xml";
  std::string inform_referee_path = pkgpath + "/behavior_trees_xml/back_home.xml";
  //std::string recognize_path = pkgpath + "/behavior_trees_xml/init_sequence.xml";

  /* Blackboard set */
  move_base_msgs::MoveBaseGoal home;

  home.target_pose.header.frame_id = "map";
  home.target_pose.pose.position.x = n.param("pos_x0" , 4.5);
  home.target_pose.pose.position.y = n.param("pos_y0", 0.47);
  home.target_pose.pose.position.z = n.param("pos_z0", 0.0);
  home.target_pose.pose.orientation.x = n.param("orient_x0", 0.0);
  home.target_pose.pose.orientation.y = n.param("orient_y0", 0.0);
  home.target_pose.pose.orientation.z = n.param("orient_z0", -0.99);
  home.target_pose.pose.orientation.w = n.param("orient_w0", 0.12);
  move_base_msgs::MoveBaseGoal arb;
  get_arb_pose(n, &arb);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("Home", home);
  blackboard->set("arb", arb);
  int counter = 0;
  taymsgs::person_data Person_data;
  blackboard->set("data", Person_data);

  BT::Tree find_mate = factory.createTreeFromFile(find_mate_path, blackboard);
  BT::Tree inform_referee = factory.createTreeFromFile(inform_referee_path, blackboard);

  ros::Rate loop_rate(10);

  int state = -1;
  while (speaker.startNav(3) != "true")
  {
      
  }
  state = FIND_MATE;
  bool finish = false;

  /* Bt sequence */
  while (ros::ok() && !finish)
  {
    switch (state)
    {
    case FIND_MATE:
      if (find_mate.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        inform_referee = factory.createTreeFromFile(inform_referee_path, blackboard);
        state = INFORM_REFEREE;      
      }
      break;
    
    case INFORM_REFEREE:
     if (inform_referee.rootNode()->executeTick() == BT::NodeStatus::SUCCESS)
      {
        if (++counter == num_persons)
        {
          finish = true;
        }

        find_mate = factory.createTreeFromFile(find_mate_path, blackboard);
        state = FIND_MATE;
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
