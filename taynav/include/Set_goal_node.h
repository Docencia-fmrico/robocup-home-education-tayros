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

#ifndef SET_GOAL_H
#define SET_GOAL_H

#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32.h"
#include "transforms.h"

#include "ros/ros.h"

namespace Goal_calculate
{

class SetGoal 
{
  public:
    explicit SetGoal();

    void get_dist_angle_tf();
    void calculate_goal();



  private:
    ros::NodeHandle nh_;

    geometry_msgs::TransformStamped map2odom_msg_;
    tf2::Stamped<tf2::Transform> map2odom_;

    geometry_msgs::TransformStamped odom2bf_msg_;
    tf2::Stamped<tf2::Transform> odom2bf_;

    geometry_msgs::TransformStamped bf2person_msg_;
    tf2::Stamped<tf2::Transform> bf2person_;

    tf2::Transform map2person_;

    std::string error_;

    double posX_;
    double posY_;
    double posZ_;

    double yaw_;
    double pitch_;
    double roll_;

    bool transform_made_;

    int activation_;
    ros::Publisher pos_pub_;
    ros::Subscriber sub_activation_;
    void callback(const std_msgs::Int32::ConstPtr& msg);
};

}  // namespace Goal_calculate

#endif  // SET_GOAL_H